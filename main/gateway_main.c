/*
@file gateway_main.c 
@brief Main entry point, handling Wifi setup and starting the Nabto Modbus gateway.

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
#include "app_wifi.h"
#include "nabto_app_modbus.h"
#include "nabto_cred_config.h"

/* Macros */
#define MAIN_DELAY               10 /* seconds */

#define NABTO_GW_STACK_SIZE    8592
#define NABTO_GW_TASK_PRIO        1

/* Static variables */
static const char         *SSID_PREFIX = "nabto-modbus-gw-";
static const char         *TAG_MAIN    = "MB_MAIN";
static TaskHandle_t        nabto_modbus_rtu_app_task_handle = NULL;
static nabto_credentials_t nabto_credentials;

//Main application
void app_main()
{
    esp_err_t           ret;
    uint8_t             esp_mac_addr[6];
    static char         ssid_str[MAX_SSID_LEN];

    ESP_LOGI(TAG_MAIN, "[APP] Startup..");
    ESP_LOGI(TAG_MAIN, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG_MAIN, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    nvs_flash_init();

    /* assemble SSID name (prefix-MAC Address) */
    ret = esp_efuse_mac_get_default(esp_mac_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_MAIN, "Unable to get MAC Address from efuse");
        return;
    }
    sprintf(ssid_str, "%s%2x%2x%2x%2x%2x%2x", SSID_PREFIX,
            esp_mac_addr[0], esp_mac_addr[1], esp_mac_addr[2],
            esp_mac_addr[3], esp_mac_addr[4], esp_mac_addr[5]);
    ESP_LOGI(TAG_MAIN, "ESP SSID prefix: %s", ssid_str);

    /* standard ESP32 wifi init:
     * Initialize and connect to Wifi in client mode, or request credentials via AP mode.
     */
    app_wifi_init(ssid_str);

    // disable further default wifi logging (annoying...)
    esp_log_level_set("wifi", ESP_LOG_NONE);

    /* 
     * Initialize FreeModbus in master mode
     */
    /* TODO once we support system configuration (multiple buses, remote change of configuration)
            support it here before starting the modbus instance(s).
            Read configuration from NVS or (remote) file; for each UART number we need to read:
            - baudrate
            - parity setting(s) from config
            - stopbit setting(s) from config
    */
    ESP_ERROR_CHECK(app_modbus_init_master());

    /* 
     * Get Nabto credentials (from NVS or downloaded) + add unique device name as it is needed in the main task
     */
    ESP_ERROR_CHECK(app_nabto_get_credentials(&nabto_credentials, ssid_str));
    nabto_credentials.unique_device_name = ssid_str;

    ret = xTaskCreate( (void*)&modbus_rtu_app_main_task,
                       "nabto_modbus_gw_task",
                       NABTO_GW_STACK_SIZE,
                       (void *)&nabto_credentials,
                       NABTO_GW_TASK_PRIO,
                       &nabto_modbus_rtu_app_task_handle);
    if (ret != pdPASS)
        return;

    while (1)
    {
        vTaskDelay(100000);
        // TODO supervise Nabto main app task, if ended close Nabto and reboot.
    }
    esp_restart();
}


