/*
@file nabto_cred_config.h
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

#ifndef __NABTO_CRED_CONFIG_H__
#define __NABTO_CRED_CONFIG_H__

/* Includes */

/* Macros */
#define NABTO_MAX_ID_SIZE               64
#define NABTO_MAX_PRESHARED_KEY_SIZE    32
#define MAX_SSID_LEN                    32
#define MAX_MODBUS_CONFIGURATION_SIZE 1400
#define URL_MAX_POSTFIX_SIZE            32 /* the file name part of the download URL excluding server address and path */

/* Types, Structures */

typedef struct {
    char nabtoId[NABTO_MAX_ID_SIZE];
    char nabtoPresharedKey[NABTO_MAX_PRESHARED_KEY_SIZE+1]; /* 32 hex chars + terminating zero */
    char *unique_device_name;
} nabto_credentials_t;

/* Function prototypes */

int hctoi(const unsigned char h);

/* Retrieve specific config string from NVS. */
esp_err_t _read_blob_from_nvs(char *blob, const char *nvstag, size_t sz, const char *what);

/* write specific config string to NVS. */
esp_err_t _write_blob_to_nvs(char *blob, const char *nvstag, size_t sz, const char *what);

/* Erase specific config string from NVS. */
esp_err_t _erase_tag_from_nvs(const char *nvstag, const char *what);

/* Retrieves Nabto credentials from NVS or download from server. */
esp_err_t app_nabto_get_credentials(nabto_credentials_t *nabto_credentials, char *unique_device_name);

/* Retrieves modbus configuration string from NVS or download from server. cfg_str must accomodate the configuration and not be smaller than size given in sz. */
esp_err_t nabto_get_modbus_configuration(char *cfg_str, size_t sz, char *unique_device_name);


#endif /* __NABTO_CRED_CONFIG_H__ */

