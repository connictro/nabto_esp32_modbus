/*
@file nabto_cred_config.c
@brief Handles NVS storage and download of Nabto ID, Nabto Preshared Key
       and configuration.

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

/* Includes */

#include "app_common_defs.h"
#include "nabto_cred_config.h"
#include "esp_http_client.h"
#include "nvs.h"
#include "nvs_flash.h"

/* Macros */
#define APP_NABTO_ID_DL_URL_PREFIX (CONFIG_NABTO_ID_DL_URL_PREFIX)
#define NVS_STORAGE_NAMESPACE "nabtocreds"

#define MAX_DOWNLOAD_URL_SIZE  200  // adapt if needed

/* Types, Structures */

/* Static variables */
static const char *NVS_NABTOID_TAG       = "nabtoid";
static const char *NVS_NABTOKEY_TAG      = "nabtokey";
static const char *NVS_MODBUSCFG_TAG     = "modbuscfg";
static const char *NABTO_ID_WHATSTR      = "Nabto ID";
static const char *NABTO_KEY_WHATSTR     = "Nabto Key";
static const char *MODBUSCFG_WHATSTR     = "Modbus config";
static const char *MODBUS_CFG_FILENAME_EXTENSION = "_modbus_config.json";
static const char *TAG                   = "NA_CRED";

/* Root cert for key server, choose your URL. 
   The example embedded is from www.connictro.de; please replace if
   you change the URL.

   The PEM file was extracted from the output of this command:
   openssl s_client -showcerts -connect www.connictro.de:443 </dev/null

   The CA root cert is the last cert given in the chain of certs.

   To embed it in the app binary, the PEM file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/

extern const char keyserver_root_cert_pem_start[] asm("_binary_keyserver_root_cert_pem_start");
extern const char keyserver_root_cert_pem_end[]   asm("_binary_keyserver_root_cert_pem_end");


/* Static functions */

static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data
                // printf("%.*s", evt->data_len, (char*)evt->data);
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}


// Retrieve Nabto ID and pre-shared key from NVS.
static esp_err_t l_nabto_read_credentials_from_nvs(nabto_credentials_t *nabto_credentials)
{
    esp_err_t   status = ESP_OK;
    size_t      sz;

    sz = sizeof(nabto_credentials->nabtoId);
    status = _read_blob_from_nvs(nabto_credentials->nabtoId, NVS_NABTOID_TAG, sz, NABTO_ID_WHATSTR);
    if (status != ESP_OK) return status;

    sz = sizeof(nabto_credentials->nabtoPresharedKey);
    status = _read_blob_from_nvs(nabto_credentials->nabtoPresharedKey, NVS_NABTOKEY_TAG, sz, NABTO_KEY_WHATSTR);

    return status;
}

static esp_err_t l_nabto_write_credentials_to_nvs(nabto_credentials_t *nabto_credentials)
{
    esp_err_t   status = ESP_OK;
 
    if (nabto_credentials == NULL)
        return ESP_FAIL;

    status = _write_blob_to_nvs(nabto_credentials->nabtoId, NVS_NABTOID_TAG, NABTO_MAX_ID_SIZE, NABTO_ID_WHATSTR);
    if (status != ESP_OK) return status;

    status = _write_blob_to_nvs(nabto_credentials->nabtoPresharedKey, NVS_NABTOKEY_TAG, NABTO_MAX_PRESHARED_KEY_SIZE, NABTO_KEY_WHATSTR);
    return status;
}

static esp_err_t l_app_nabto_download_file_to_blob(char *blob, char *url, size_t sz)
{
    esp_err_t   status = ESP_FAIL;

    if (blob == NULL) return status;

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
        .cert_pem = keyserver_root_cert_pem_start,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    status = esp_http_client_perform(client);

    if (status == ESP_OK) 
    {
        int http_status = esp_http_client_get_status_code(client);
        int http_length = esp_http_client_get_content_length(client);

        ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %d", http_status, http_length);

        if (http_status == 200 && http_length < sz)
        {        
            esp_http_client_read(client, blob, http_length);       
            blob[http_length] = '\0';
        } else {
            status = http_status;
        }
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(status));
    }
    esp_http_client_cleanup(client);

    return status;
}

static esp_err_t l_app_nabto_download_credentials(nabto_credentials_t *nabto_credentials, char *url)
{
    char      temp_buffer[sizeof(nabto_credentials_t)];
    esp_err_t status;

    status = l_app_nabto_download_file_to_blob(temp_buffer, url, sizeof(nabto_credentials_t));

    if (status == ESP_OK)
    {        
        char *savep;
        char *tmpId;
        char *tmpKey;

        tmpId = strtok_r(temp_buffer, ";", &savep);
        if (tmpId != NULL)
        {
            ESP_LOGI(TAG, "Nabto ID: %s", tmpId);
            strcpy(nabto_credentials->nabtoId, tmpId);
            tmpKey = strtok_r(NULL, ";", &savep);
            if (tmpKey != NULL)
            {
                strncpy(nabto_credentials->nabtoPresharedKey, tmpKey, NABTO_MAX_PRESHARED_KEY_SIZE);
                status = ESP_OK;
                ESP_LOGI(TAG, "Nabto Key: Successfully downloaded!");
            }
        }
    }
    return status;
}

/* Implementation */

// Retrieve specific config string (key or modbus cfg) from NVS.
esp_err_t _read_blob_from_nvs(char *blob, const char *nvstag, size_t sz, const char *what)
{
    esp_err_t   status = ESP_OK;
    nvs_handle  nabto_nvs_handle;

    ESP_LOGD(TAG, "open NVS storage");
    status = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READONLY, &nabto_nvs_handle);
    if (status != ESP_OK)
        return status;

    ESP_LOGI(TAG, "namespace exists, trying to read %s from NVS", what);
    status = nvs_get_blob(nabto_nvs_handle, nvstag, blob, &sz);

    if (status != ESP_OK)
    {
        ESP_LOGW(TAG, "no valid %s in NVS", what);
        status = ESP_ERR_NOT_FOUND;
    } else {
        ESP_LOGI(TAG, "%s: %s read from NVS", what, blob);
    }
    nvs_close(nabto_nvs_handle);

    return status;
}

// write specific config string (key or modbus cfg) to NVS.
esp_err_t _write_blob_to_nvs(char *blob, const char *nvstag, size_t sz, const char *what)
{
    esp_err_t   status = ESP_OK;
    nvs_handle  nabto_nvs_handle;

    ESP_LOGD(TAG, "open NVS storage");
    status = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READWRITE, &nabto_nvs_handle);
    if (status != ESP_OK)
        return status;

    ESP_LOGI(TAG, "Writing %s to NVS", what);

    status = nvs_set_blob(nabto_nvs_handle, nvstag, blob, sz);
    if (status != ESP_OK) 
    {
        nvs_close(nabto_nvs_handle);
        return status;
    }

    status = nvs_commit(nabto_nvs_handle);
    nvs_close(nabto_nvs_handle);

    return status;   
}

esp_err_t _erase_tag_from_nvs(const char *nvstag, const char *what)
{
    esp_err_t   status = ESP_OK;
    nvs_handle  nabto_nvs_handle;

    ESP_LOGD(TAG, "open NVS storage");
    status = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READWRITE, &nabto_nvs_handle);
    if (status != ESP_OK)
        return status;

    ESP_LOGI(TAG, "Erasing %s from NVS", what);
    status = nvs_erase_key(nabto_nvs_handle, nvstag);
    nvs_close(nabto_nvs_handle);
    return status;   
}

/**
 * Helper for converting Nabto hex credentials.
 */
int hctoi(const unsigned char h)
{
    if (isdigit(h))
    {
        return h - '0';
    } else {
        return toupper(h) - 'A' + 10;
    }
}

/**
 * Retrieves Nabto ID/secret from NVS or server if not available.
 * NOTE: Not recommended for production device since URLs can be guessed (although being on a HTTPS service)... 
 *       For mass production define some secure mechanism instead e.g. server-side application which requires device authentication first,
 *       to avoid counterfeit devices acquiring a Nabto ID.
 *
 */
esp_err_t app_nabto_get_credentials(nabto_credentials_t *nabto_credentials, char *unique_device_name)
{
    esp_err_t   status = ESP_FAIL;

    if ((nabto_credentials == NULL) || (unique_device_name == NULL)) return status;

    status = l_nabto_read_credentials_from_nvs(nabto_credentials);
    if (status != ESP_OK)
    {
        char download_url[MAX_DOWNLOAD_URL_SIZE];

        (void) strcpy(download_url, APP_NABTO_ID_DL_URL_PREFIX);
        (void) strcat(download_url, unique_device_name);
        ESP_LOGI(TAG, "Downloading Nabto credentials from %s", download_url);
        status = l_app_nabto_download_credentials(nabto_credentials, download_url);
        if (status == ESP_OK)
        {
            ESP_LOGI(TAG, "Saving Nabto credentials in NVS");
            status = l_nabto_write_credentials_to_nvs(nabto_credentials);
        }
    }
    return status;
}

/**
 * Retrieves Modbus configuration from NVS or server if not available.
 * NOTE: Not recommended for production device since URLs can be guessed (although being on a HTTPS service)... 
 *       and possibly not flexible enough (change configuration from app...)
 *
 */
esp_err_t nabto_get_modbus_configuration(char *cfg_str, size_t sz, char *unique_device_name)
{
    esp_err_t   status = ESP_FAIL;

    if ((cfg_str == NULL) || (unique_device_name == NULL)) return status;

    status = _read_blob_from_nvs(cfg_str, NVS_MODBUSCFG_TAG, sz, MODBUSCFG_WHATSTR);
    if (status != ESP_OK)
    {   // not found in NVS so try to download & save in NVS if that is successful.
        char download_url[MAX_DOWNLOAD_URL_SIZE];

        (void) strcpy(download_url, APP_NABTO_ID_DL_URL_PREFIX);
        (void) strcat(download_url, unique_device_name);
        (void) strcat(download_url, MODBUS_CFG_FILENAME_EXTENSION); 
        status = l_app_nabto_download_file_to_blob(cfg_str, download_url, sz);
        if (status == ESP_OK)
        {
            status = _write_blob_to_nvs(cfg_str, NVS_MODBUSCFG_TAG, sz, MODBUSCFG_WHATSTR);
        }
    }
    return status;
}


