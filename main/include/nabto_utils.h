/*
@file nabto_utils.h
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

#include <modules/fingerprint_acl/fp_acl_ae.h>
#include <modules/fingerprint_acl/fp_acl_memory.h>
#include <modules/fingerprint_acl/fp_acl_file.h>

#ifndef __NABTO_UTILS_H__
#define __NABTO_UTILS_H__

#define DEVICE_NAME_DEFAULT "ESP32_modbus_gw"
#define MAX_DEVICE_NAME_LENGTH 50
#define REQUIRES_GUEST FP_ACL_PERMISSION_NONE
#define REQUIRES_OWNER FP_ACL_PERMISSION_ADMIN

/* Function prototypes */

fp_acl_db_status fp_acl_nvs_load(struct fp_mem_state* acl);
fp_acl_db_status fp_acl_nvs_save(struct fp_mem_state* acl);
void debug_dump_acl();
int copy_buffer(unabto_query_request* read_buffer, uint8_t* dest, uint16_t bufSize, uint16_t* len);
int copy_string(unabto_query_request* read_buffer, char* dest, uint16_t destSize);
int write_string(unabto_query_response* write_buffer, const char* string);
bool allow_client_access(nabto_connect* connection);

void nabto_environment_init(char *modbus_config_str, size_t sz, char *unique_device_name);
void nabto_env_application_set_device_name(const char* name);
void nabto_env_application_set_device_product(const char* product);
void nabto_env_application_set_device_icon_(const char* icon);
char *nabto_env_application_get_device_name(void);
char *nabto_env_application_get_device_product(void);
char *nabto_env_application_get_device_icon_(void);

#endif /* __NABTO_UTILS_H__ */
