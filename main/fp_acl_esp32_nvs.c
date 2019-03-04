/*
@file fp_acl_esp32_nvs.c
@brief Handles persistent storage of Nabto ACL into ESP32 NVS.

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
#include "fp_acl_esp32_nvs.h"

/* Macros */

/* Static variables */
static const char *NVS_NABTOPERSIST_TAG  = "nabtopersist";
static const char *NABTO_PERSIST_WHATSTR = "Nabto persistence store";

/* Implementation */

fp_acl_db_status fp_acl_nvs_load(struct fp_mem_state* acl)
{
    esp_err_t status = _read_blob_from_nvs((void *)acl, NVS_NABTOPERSIST_TAG, sizeof(struct fp_mem_state), NABTO_PERSIST_WHATSTR);

    // The status is OK even if the NVS tag did not exist - bootstrap scenario (acl comes initialized with default values already).
    return (status == ESP_OK || status == ESP_ERR_NVS_NOT_FOUND) ? FP_ACL_DB_OK : FP_ACL_DB_LOAD_FAILED;
}

fp_acl_db_status fp_acl_nvs_save(struct fp_mem_state* acl)
{
    esp_err_t status = _write_blob_to_nvs((void *)acl, NVS_NABTOPERSIST_TAG, sizeof(struct fp_mem_state), NABTO_PERSIST_WHATSTR);

    return (status == ESP_OK) ? FP_ACL_DB_OK : FP_ACL_DB_SAVE_FAILED;
}

fp_acl_db_status fp_acl_nvs_reset()
{
    esp_err_t status = _erase_tag_from_nvs(NVS_NABTOPERSIST_TAG, NABTO_PERSIST_WHATSTR);

    return (status == ESP_OK) ? FP_ACL_DB_OK : FP_ACL_DB_SAVE_FAILED;
}

fp_acl_db_status fp_acl_nvs_init(struct fp_mem_persistence* per)
{
    per->load = &fp_acl_nvs_load;
    per->save = &fp_acl_nvs_save;
    return FP_ACL_DB_OK;
}


