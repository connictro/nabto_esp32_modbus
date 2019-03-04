/*
@file nabto_utils.c
@brief Nabto helper functions

Copyright (c) 2019 Connictro GmbH / Michael Paar and Nabto ApS

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
#include "nabto_utils.h"
#include "nabto_cred_config.h"
#include "fp_acl_esp32_nvs.h"
#include "mbcontroller.h"
#include <unabto/unabto_common_main.h>
#include <unabto/unabto_app.h>
#include <modules/fingerprint_acl/fp_acl_ae.h>
#include <modules/fingerprint_acl/fp_acl_memory.h>

/* Static variables */
static const char *TAG    = "NA_UTIL";
static char device_name_[MAX_DEVICE_NAME_LENGTH];
static const char* device_product_ = "";
static const char* device_icon_ = "";

static struct fp_acl_db db_;
struct fp_mem_persistence fp_file_;

/* Implementation */

void debug_dump_acl()
{
    void* it = db_.first();
    if (!it) {
        NABTO_LOG_INFO(("ACL is empty (no paired users)"));
    } else {
        NABTO_LOG_INFO(("ACL entries:"));
        while (it != NULL)
        {
            struct fp_acl_user user;
            fp_acl_db_status res = db_.load(it, &user);
            if (res != FP_ACL_DB_OK)
            {
                NABTO_LOG_WARN(("ACL error %d\n", res));
                return;
            }
            NABTO_LOG_INFO(("%s [%02x:%02x:%02x:%02x:...]: %04x",
                        user.name,
                        user.fp[0], user.fp[1], user.fp[2], user.fp[3],
                        user.permissions));
            it = db_.next(it);
        }
    }
}

void nabto_environment_init(char *modbus_config_str, size_t sz, char *unique_device_name)
{
    ESP_LOGW(TAG, "WARNING: Remote access to the device is turned on by default. Please read TEN36 \"Security in Nabto Solutions\" to understand the security implications.");

    struct fp_acl_settings default_settings;

    default_settings.systemPermissions =
      FP_ACL_SYSTEM_PERMISSION_PAIRING |
      FP_ACL_SYSTEM_PERMISSION_LOCAL_ACCESS |
      FP_ACL_SYSTEM_PERMISSION_REMOTE_ACCESS;
    default_settings.defaultUserPermissions =
      FP_ACL_PERMISSION_LOCAL_ACCESS|
      FP_ACL_PERMISSION_REMOTE_ACCESS;
    default_settings.firstUserPermissions =
      FP_ACL_PERMISSION_ADMIN |
      FP_ACL_PERMISSION_LOCAL_ACCESS |
      FP_ACL_PERMISSION_REMOTE_ACCESS;

    (void)fp_acl_nvs_init(&fp_file_);        // persistence via ESP32 Non Volatile Store (NVS)

    ESP_LOGI(TAG, "Before fp_mem_init, default_settings.systemPermissions = %8.8X, .defaultUserPermissions=%8.8X, .firstUserPermissions=%8.8X", default_settings.systemPermissions, default_settings.defaultUserPermissions, default_settings.firstUserPermissions);
    fp_mem_init(&db_, &default_settings, &fp_file_);

    ESP_LOGI(TAG, "Before acl_ae_init");
    fp_acl_ae_init(&db_);

    snprintf(device_name_, sizeof(device_name_), DEVICE_NAME_DEFAULT);

    (void) nabto_get_modbus_configuration(modbus_config_str, sz, unique_device_name);
}

void nabto_env_application_set_device_name(const char* name)
{
    strncpy(device_name_, name, MAX_DEVICE_NAME_LENGTH);
}

void nabto_env_application_set_device_product(const char* product) 
{
    device_product_ = product;
}

void nabto_env_application_set_device_icon_(const char* icon)
{
    device_icon_ = icon;
}

char *nabto_env_application_get_device_name()
{
    return device_name_;
}

char *nabto_env_application_get_device_product() 
{
    return (char *)device_product_;
}

char *nabto_env_application_get_device_icon_()
{
    return (char *)device_icon_;
}


int copy_buffer(unabto_query_request* read_buffer, uint8_t* dest, uint16_t bufSize, uint16_t* len) {
    uint8_t* buffer;
    if (!(unabto_query_read_uint8_list(read_buffer, &buffer, len))) {
        return AER_REQ_TOO_SMALL;
    }
    if (*len > bufSize) {
        return AER_REQ_TOO_LARGE;
    }
    memcpy(dest, buffer, *len);
    return AER_REQ_RESPONSE_READY;
}

int copy_string(unabto_query_request* read_buffer, char* dest, uint16_t destSize) {
    uint16_t len;
    int res = copy_buffer(read_buffer, (uint8_t*)dest, destSize-1, &len);
    if (res != AER_REQ_RESPONSE_READY) {
        return res;
    }
    dest[len] = 0;
    return AER_REQ_RESPONSE_READY;
}

int write_string(unabto_query_response* write_buffer, const char* string)
{
    //NABTO_LOG_INFO(("write_buffer called: string is %s, length is %d", string, strlen(string)));
    return unabto_query_write_uint8_list(write_buffer, (uint8_t *)string, strlen(string));
}

bool allow_client_access(nabto_connect* connection) {
    bool local = connection->isLocal;
    bool allow = fp_acl_is_connection_allowed(connection) || local;
    NABTO_LOG_INFO(("Allowing %s connect request: %s", (local ? "local" : "remote"), (allow ? "yes" : "no")));
    debug_dump_acl();
    return allow;    
}

