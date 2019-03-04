/*
@file app_wifi.c
@brief handles the Wifi manager created by Tony Pottier, storing of credentials
       into NVM and credentials cleanup.

Copyright (c) 2019 Connictro GmbH / Michael Paar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/


#include "app_common_defs.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "app_wifi.h"

#include "mdns.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "http_server.h"
#include "wifi_manager.h"

/* Macros / Constants */
#define MAX_CONNECTION_RETRY     CONFIG_ESP_MAXIMUM_RETRY
#define WIFI_CONNECTION_TIMEOUT  ((CONFIG_ESP_CONNECTION_TIMEOUT*1000)/portTICK_RATE_MS)
#define NVS_STORAGE_NAMESPACE "espwifimgr"
static const char *NVS_SSID_TAG  = "ssid";
static const char *NVS_PWD_TAG   = "password";
static const char *TAG_WF        = "APP_WIFI";
const TickType_t waitCredsOKDelay = 1000 / portTICK_PERIOD_MS;  /* waiting for credentials saved; check every second */


/* Types */

/* External variables */

/* Static variables */

static EventGroupHandle_t    wifi_event_group;
const static int             CONNECTED_BIT = BIT0;

static TaskHandle_t task_http_server = NULL;
static TaskHandle_t task_wifi_manager = NULL;
//static SemaphoreHandle_t wifi_credentials_saved_semaphore;   
static struct wifi_manager_parameter_t wifi_mgr_parameters;


/* Implementation static functions */

/* 
 * Erase Wifi credentials in NVS, this is called after MAX_CONNECTION_RETRY
 * unsuccessful connection attempts. Reboot ESP32 afterwards. 
 *
 * Parameter:  none
 * Returns/Return value: none (reboots ESP32)
 */
static void wifi_erase_nvs_credentials_and_reboot(void)
{
    nvs_handle wifi_nvs_handle;

    ESP_LOGI(TAG_WF, "Erasing Wifi credentials from NVS");
    if (nvs_open(NVS_STORAGE_NAMESPACE, NVS_READWRITE, &wifi_nvs_handle) == ESP_OK)
    {
        nvs_erase_key(wifi_nvs_handle, NVS_SSID_TAG);
        nvs_erase_key(wifi_nvs_handle, NVS_PWD_TAG);
    } 
    nvs_close(wifi_nvs_handle);

    ESP_LOGI(TAG_WF, "Restarting ESP32");
    esp_restart();
}

/*
 * Try to read Wifi credentials from NVS. 
 *
 * Parameter: pointer to wifi_credentials structure (caller owns memory)
 *
 * Returns:   in case of success wifi_credentials structure filled out.
 * Return value: status from underlying NVS API functions
 */
static esp_err_t wifi_read_nvs_credentials(wifi_config_t *wcr)
{
    nvs_handle wifi_nvs_handle;
    esp_err_t status = ESP_OK;
    size_t sz;

    ESP_LOGI(TAG_WF, "open NVS storage");
    status = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READONLY, &wifi_nvs_handle);
    if (status != ESP_OK)
        return status;

    ESP_LOGI(TAG_WF, "trying to read credentials from NVS");
    sz = sizeof(wcr->sta.ssid);
    status = nvs_get_blob(wifi_nvs_handle, NVS_SSID_TAG, (char *)(wcr->sta.ssid), &sz);

    if (status != ESP_OK)
    {
        nvs_close(wifi_nvs_handle);
        ESP_LOGI(TAG_WF, "no valid SSID stored");
        return status;
    } 
    ESP_LOGI(TAG_WF, "SSID %s read from NVS", (char *)(wcr->sta.ssid));

    sz = sizeof(wcr->sta.password);
    status = nvs_get_blob(wifi_nvs_handle, NVS_PWD_TAG, (char *)(wcr->sta.password), &sz);
    if (status != ESP_OK)
    {
        nvs_close(wifi_nvs_handle);
        ESP_LOGI(TAG_WF, "no valid password stored: %s", (char *)(wcr->sta.password));
        wifi_erase_nvs_credentials_and_reboot(); // SSID valid but not password; something is wrong so try again with fresh credentials.
        return status;
    } 
    ESP_LOGI(TAG_WF, "Password ****** read from NVS");

    return status;
}

/*
 * Enter access point mode, start web server and read wifi credentials / store
 * in NVS once received. Reboot afterwards.
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Wifi manager is brought by:

   Copyright (c) 2017 Tony Pottier

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.

   @file main.c
   @author Tony Pottier
   @brief Entry point for the ESP32 application.
   @see https://idyl.io
   @see https://github.com/tonyp7/esp32-wifi-manager
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * Parameter:  none
 * Returns/Return value: none (reboots ESP32)
 */

static void wifi_enter_ap_mode_and_reboot(char *ssid_str)
{
    wifi_mgr_parameters.wifi_credentials_saved_semaphore = xSemaphoreCreateBinary();
    if (wifi_mgr_parameters.wifi_credentials_saved_semaphore == NULL)
    {
        ESP_LOGI(TAG_WF, "FATAL ERROR: can not create semaphore");
        return;
    }
    wifi_mgr_parameters.ssid_str = ssid_str;

    /* disable the default wifi logging */
    esp_log_level_set("wifi", ESP_LOG_NONE);

    /* start the HTTP Server task */
    ESP_LOGI(TAG_WF, "Starting HTTP server");
    xTaskCreate(&http_server, "http_server", 2048, NULL, 5, &task_http_server);

    /* start the wifi manager task */
    ESP_LOGI(TAG_WF, "Starting Wifi manager");
    xTaskCreate(&wifi_manager, "wifi_manager", 4096, &wifi_mgr_parameters, 4, &task_wifi_manager);

    // wait until credentials have been stored in NVS
    xSemaphoreTake(wifi_mgr_parameters.wifi_credentials_saved_semaphore, portMAX_DELAY);

    ESP_LOGI(TAG_WF, "Restarting ESP32");
    esp_restart();  // and finally reboot when finished configuration.
}


/* Wifi event handler, unchanged from ESP32 example */
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

/* Implementation interface functions */

void app_wifi_init(char *ssid_str)
{
    wifi_config_t wifi_config;
    EventBits_t uxBits;

    tcpip_adapter_init();

    if (wifi_read_nvs_credentials(&wifi_config) != ESP_OK)
    {
        ESP_LOGI(TAG_WF, "Entering Access Point Mode");
        wifi_enter_ap_mode_and_reboot(ssid_str);
        // Never reached when finished anyhow, as the above function will reboot the ESP32.
    }
    
    // we're entering the connection retry loop if valid credentials have been retrieved from NVS.
    // (otherwise AP mode leads to reboot)

    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    for (int i=0;i<MAX_CONNECTION_RETRY;i++)
    {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
        ESP_LOGI(TAG_WF, "start the WIFI SSID:[%s] password:[%s]", (char *)(wifi_config.sta.ssid), "******");
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG_WF, "Waiting for wifi");
        uxBits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, WIFI_CONNECTION_TIMEOUT);

        if ((uxBits & CONNECTED_BIT) != 0)
        {
            tcpip_adapter_ip_info_t ip_info;
            // print the local IP address
            ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));

            ESP_LOGI(TAG_WF, "IP Address:  %s", ip4addr_ntoa(&ip_info.ip));
            ESP_LOGI(TAG_WF, "Subnet mask: %s", ip4addr_ntoa(&ip_info.netmask));
            ESP_LOGI(TAG_WF, "Gateway:     %s", ip4addr_ntoa(&ip_info.gw));

            // change hostname to be unique, use the same as we used for AP mode SSID
            tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, ssid_str);

            return; // connection successful so simply return here, our work is done.
        }
        esp_wifi_stop();
    }
    /* Maximum number of connection retries expired; so we erase the stored credentials
     * (they might be wrong, expired or the device moved to a different location). 
     */
    wifi_erase_nvs_credentials_and_reboot();
}


