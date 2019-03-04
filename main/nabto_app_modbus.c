/*
@file app_modbus.c
@brief Interfacing of FreeModbus Master port to Nabto IoT device solution.

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

/* ESP libraries */
#include "esp_http_client.h"

/* ... FreeModbus */
#include "mbcontroller.h"

/* ... application */
#include <stdio.h>
#include "unabto_config.h"
#include "app_common_defs.h"
#include "nabto_app_modbus.h"
#include "nabto_cred_config.h"
#include "nabto_utils.h"

/* ... Nabto */
#include <modules/util/list_malloc.h>
#include <modules/util/memory_allocation.h>
#include <modules/acl/acl.h>
#include <modules/configuration_store/configuration_store.h>
#include <modules/settings/settings.h>
#include <unabto/unabto_connection.h>
#include <unabto/unabto_common_main.h>
#include <unabto/unabto_app.h>
#include <unabto/unabto_util.h>
#include <unabto/util/unabto_buffer.h>
#include <modules/fingerprint_acl/fp_acl_ae.h>
#include <modules/fingerprint_acl/fp_acl_memory.h>
#include <modules/fingerprint_acl/fp_acl_file.h>

/* Local Macros */
#define MB_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

#ifdef NABTO_LOG_MODULE_CURRENT
#undef NABTO_LOG_MODULE_CURRENT
#endif
#define NABTO_LOG_MODULE_CURRENT NABTO_LOG_MODULE_APPLICATION
#define REQUIRES_GUEST FP_ACL_PERMISSION_NONE
#define REQUIRES_OWNER FP_ACL_PERMISSION_ADMIN

#define MODBUS_MAXIMUM_FRAME_SIZE  MB_SER_PDU_SIZE_MAX

/* Types, Structures */

enum
{
    // Modbus/device related queries
    QUERY_MODBUS_FUNCTION_RAW       = 20000,
    QUERY_MODBUS_CONFIGURATION      = 20001,
    QUERY_MODBUS_FUNCTION_FORMATTED = 20002,
};

enum
{
    QUERY_STATUS_OK,
    QUERY_STATUS_ERROR,
    QUERY_STATUS_OUT_OF_RESOURCES,
    QUERY_STATUS_USER_NOT_FOUND,
    QUERY_STATUS_SETTING_NOT_FOUND
};

enum
{
    MODBUS_FUNCTION_RESULT_SUCCESS,
    MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES,
    MODBUS_FUNCTION_RESULT_NO_REPLY,
    MODBUS_FUNCTION_RESULT_INVALID_BUS,
    MODBUS_FUNCTION_RESULT_OVERSIZE_FRAME
};

/* Static variables */
const char *mb_err_codes_decoded[] = { 
     "OK", 
     "Illegal register address",
     "Illegal argument",
     "Received data error",
     "Timed out!",
     "Unexpected Master busy",
     "Execute function error",
     "Illegal function call",
     "I/O error",
     "Porting layer error",
     "Illegal state error",
     "Out of memory error",
     "Response pending",
     "Response not found",
     "Unknown error"
};

static char modbus_configuration[MAX_MODBUS_CONFIGURATION_SIZE];
static const char *TAG    = "MB_APP";
static const char *N_DEVICE_NAME = "ESP32_MB_GW";
static const char *N_DEVICE_PROD = "Nabto to Modbus gateway";
static const char *N_DEVICE_ICON = "chip-small.png";
static const char* device_interface_id_ = "DC14A962-39C7-4067-8EC6-6A491E45E283";   // TODO how do we get this interface ID???   Apparently the app needs to know; it must be hard coded actually
static uint16_t device_interface_version_major_ = 1;
static uint16_t device_interface_version_minor_ = 0;


static QueueHandle_t xMBAppModbusQueue;
static QueueHandle_t xMBModbusSupervisorQueue = NULL;

/* Static functions */

static esp_err_t setup_nabto(nabto_credentials_t *nabto_credentials)
{
    const char* p;
    unsigned char* up;
    static nabto_main_setup* nms;
    tcpip_adapter_ip_info_t ip_info;

    ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));

    // Configure Nabto
    nms = unabto_init_context();
    nms->ipAddress = ip_info.ip.addr;
    nms->id = nabto_credentials->nabtoId;
    nms->secureAttach = 1;
    nms->secureData = 1;
    nms->cryptoSuite = CRYPT_W_AES_CBC_HMAC_SHA256;

    for (p = nabto_credentials->nabtoPresharedKey, up = nms->presharedKey; *p; p += 2, ++up)
    {
        *up = hctoi(p[0]) * 16 + hctoi(p[1]);  // hex string to byte array
    }
    unabto_init();
    nabto_environment_init(modbus_configuration, MAX_MODBUS_CONFIGURATION_SIZE, nabto_credentials->unique_device_name);
    nabto_env_application_set_device_name(N_DEVICE_NAME);
    nabto_env_application_set_device_product(N_DEVICE_PROD);
    nabto_env_application_set_device_icon_(N_DEVICE_ICON);

    return ESP_OK;
}

static application_event_result abort_modbus_function(struct mb_request_entry* desc, buffer_write_t* writeBuffer, uint8_t bus, uint8_t address, uint8_t result)
{
    if (desc != NULL)  mbsupervisor_buf_free(desc);

    // send 'result' with zero-length 'data'
    if(!unabto_query_write_uint8(writeBuffer, bus) || !unabto_query_write_uint8(writeBuffer, address) || !unabto_query_write_uint8(writeBuffer, result) || !buffer_write_raw_from_array(writeBuffer, NULL, 0) || !unabto_query_write_uint8(writeBuffer, QUERY_STATUS_OK))
    {
        return AER_REQ_RSP_TOO_LARGE;
    }
    return AER_REQ_RESPONSE_READY;
}

static application_event_result begin_modbus_formatted_function_query(application_request* request, buffer_read_t* readBuffer, buffer_write_t* writeBuffer)
{
    uint8_t  bus;
    uint8_t  address;
    uint16_t num_data;
    struct mb_request_entry *desc;
    mb_supervisor_request_t  svr;

    // read input parameters
    if(!unabto_query_read_uint8(readBuffer, &bus) || !unabto_query_read_uint8(readBuffer, &address))
    {
        return AER_REQ_TOO_SMALL;
    }

    if(bus >= MODBUS_NUMBER_OF_BUSSES)
    {
        return abort_modbus_function(NULL, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_INVALID_BUS);
    }
    // TODO "bus" ignored so far (currently just supporting one bus on ESP32). 
    //      In the future we'll maintain multiple modbus supervisors who can run independently/simultaneously.

    // create query request to FreeModbus.
    if (mbsupervisor_buf_alloc(&desc) != ESP_OK)
    {
        return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES);
    }
    svr.sender                = xMBAppModbusQueue;
    svr.req_entry.descriptor  = desc;
    svr.req_resp.req_type     = MB_SUPERV_REQ;
    desc->is_raw_request        = FALSE;
    desc->request_id          = (uint32_t)request;
    desc->u.f.slave_address   = address;

    // read formatted input parameters
    if(!unabto_query_read_uint8 (readBuffer, &(desc->u.f.function_code))) return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES); 
    if(!unabto_query_read_uint16(readBuffer, &(desc->u.f.read_start)))    return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES); 
    if(!unabto_query_read_uint16(readBuffer, &(desc->u.f.read_len)))      return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES); 
    if(!unabto_query_read_uint16(readBuffer, &(desc->u.f.write_start)))   return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES); 
    if(!unabto_query_read_uint16(readBuffer, &(desc->u.f.write_len)))     return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES); 
    if (!unabto_query_read_list_length(readBuffer, &num_data))            return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES); 
    for (int i=0;i<num_data;i++)
    {
        if(!unabto_query_read_uint16(readBuffer, &(desc->u.f.unpacked_data[i]))) return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES); 
    }

    ESP_LOGD(TAG, "sending formatted modbus request %d to slave %d", desc->u.f.function_code, address);
    if ( xQueueSend(xMBModbusSupervisorQueue, (void *)&svr, MB_PAR_INFO_TOUT) != pdTRUE )
    {
        NABTO_LOG_ERROR(("Unable to transfer Modbus message!"));
        return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES);
    }
    if (xQueueReceive(xMBAppModbusQueue, (void *)&svr, MB_PAR_INFO_TOUT) != pdTRUE)
    {
        NABTO_LOG_ERROR(("Error from Modbus supervisor enqueueing message"));
        return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES);
    }
    ESP_LOGD(TAG, "request %8.8X enqueued successfully", (uint32_t)request);
    return AER_REQ_ACCEPTED;
}

static application_event_result begin_modbus_raw_function_query(application_request* request, buffer_read_t* readBuffer, buffer_write_t* writeBuffer)
{
    uint8_t bus;
    uint8_t address;
    buffer_t data;
    struct mb_request_entry *desc;
    mb_supervisor_request_t  svr;

    // read input parameters
    if(!unabto_query_read_uint8(readBuffer, &bus) || !unabto_query_read_uint8(readBuffer, &address))
    {
        return AER_REQ_TOO_SMALL;
    }
    
    {
        uint8_t *temp_data;
        uint16_t temp_len;

        if (!unabto_query_read_uint8_list(readBuffer, &temp_data, &temp_len))
        {
            return AER_REQ_TOO_SMALL;
        }
        unabto_buffer_init(&data, temp_data, temp_len);
    }

    if(bus >= MODBUS_NUMBER_OF_BUSSES)
    {
        return abort_modbus_function(NULL, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_INVALID_BUS);
    }
    if(data.size > MODBUS_MAXIMUM_FRAME_SIZE)
    {
        return abort_modbus_function(NULL, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OVERSIZE_FRAME);
    }

    // create query request to FreeModbus.
    if (mbsupervisor_buf_alloc(&desc) != ESP_OK)
    {
        return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES);
    }

    // TODO "bus" ignored so far (currently just supporting one bus on ESP32). 
    //      In the future we'll maintain multiple modbus supervisors who can run independently/simultaneously.


    svr.sender                = xMBAppModbusQueue;
    svr.req_entry.descriptor  = desc;
    svr.req_resp.req_type     = MB_SUPERV_REQ;
    desc->is_raw_request      = TRUE;
    desc->request_id          = (uint32_t)request;
    memcpy(desc->u.raw_frame, data.data, data.size); // copy frame to message including slave address...
    desc->raw_frame_length    = data.size;
    ESP_LOGD(TAG, "sending raw modbus request to slave %d", address);

    if ( xQueueSend(xMBModbusSupervisorQueue, (void *)&svr, MB_PAR_INFO_TOUT) != pdTRUE )
    {
        NABTO_LOG_ERROR(("Unable to transfer Modbus message!"));
        return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES);
    }
    if (xQueueReceive(xMBAppModbusQueue, (void *)&svr, MB_PAR_INFO_TOUT) != pdTRUE)
    {
        NABTO_LOG_ERROR(("Error from Modbus supervisor enqueueing message"));
        return abort_modbus_function(desc, writeBuffer, bus, address, MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES);
    }
    ESP_LOGD(TAG, "request %8.8X enqueued successfully", (uint32_t)request);
    return AER_REQ_ACCEPTED;
}

static application_event_result end_modbus_function_query(application_request* request, buffer_read_t* readBuffer, buffer_write_t * writeBuffer, struct mb_request_entry *desc)
{
    uint8_t res;

    // Entered here we have everything we need in the descriptor. Copy information to Nabto response.
    switch (desc->result)
    {
        case MB_MRE_NO_ERR:   res = MODBUS_FUNCTION_RESULT_SUCCESS;          break;
        case MB_MRE_OUTOFMEM: res = MODBUS_FUNCTION_RESULT_OUT_OF_RESOURCES; break;
        default:              res = MODBUS_FUNCTION_RESULT_NO_REPLY;         break;
    }

    if(!unabto_query_write_uint8(writeBuffer, 1 /* TODO change once we're supporting multiple buses */) ||
       !unabto_query_write_uint8(writeBuffer, desc->u.f.slave_address) || 
       !unabto_query_write_uint8(writeBuffer, res) )
    {
        goto end_modbus_query_error_cleanup;
    }
 
    if (res == MODBUS_FUNCTION_RESULT_SUCCESS)
    {  // Modbus slave replied.
        if (desc->is_raw_request == TRUE)
        {   // raw request: reply with raw modbus frame received back
            NABTO_LOG_TRACE(("Raw frame (%d bytes) received from modbus supervisor: ", desc->raw_frame_length));
            for (int j=0;j<desc->raw_frame_length;j++){printf("%d ",desc->u.raw_frame[j]);} printf("\n");
            if(!buffer_write_raw_from_array(writeBuffer, desc->u.raw_frame, desc->raw_frame_length))
            {
                goto end_modbus_query_error_cleanup;
            }
        } else {
            NABTO_LOG_TRACE(("Formatted frame received from modbus supervisor"));
            // formatted request: reply with read data received (only on read requests), is always a list of type uint16 (regardless of underlying type like registers or bitfields).
            unabto_list_ctx listCtx;
            uint16_t listLength = desc->u.f.read_len;

            if (listLength > MB_MAX_APPLICATION_DATA)                              goto end_modbus_query_error_cleanup;
            if (!unabto_query_write_list_start(writeBuffer, &listCtx))             goto end_modbus_query_error_cleanup;

            for (int i=0; i<listLength; i++)
            {
                if (!unabto_query_write_uint16(writeBuffer, desc->u.f.unpacked_data[i])) goto end_modbus_query_error_cleanup;
            }
            if (!unabto_query_write_list_end(writeBuffer, &listCtx, listLength))   goto end_modbus_query_error_cleanup;
        }  
    } else {
        NABTO_LOG_TRACE(("Timeout from modbus supervisor!"));
        if (desc->is_raw_request == TRUE)
        {
            if(!buffer_write_raw_from_array(writeBuffer, NULL, 0))
            {
                goto end_modbus_query_error_cleanup;
            }            
        } else {
            unabto_list_ctx listCtx;
            if( !unabto_query_write_list_start(writeBuffer, &listCtx) || !unabto_query_write_list_end(writeBuffer, &listCtx, 0) )
            {
                goto end_modbus_query_error_cleanup;
            }            
        }
    }


    if(!unabto_query_write_uint8(writeBuffer, QUERY_STATUS_OK))                goto end_modbus_query_error_cleanup;

    NABTO_LOG_TRACE(("Application poll ended successfully."));
    mbsupervisor_buf_free(desc);
    return AER_REQ_RESPONSE_READY;

end_modbus_query_error_cleanup:
    NABTO_LOG_ERROR(("Unable to write response to client!"));
    mbsupervisor_buf_free(desc);
    return AER_REQ_RSP_TOO_LARGE;
}

/* Implementation */

esp_err_t app_modbus_init_master()
{
    esp_err_t                eStatus = ESP_FAIL;
    mb_communication_info_t  comm_info;   // Modbus communication parameters
    mb_supervisor_request_t  svr;         // for requests&replies to/from modbus supervisor

    ESP_LOGD(TAG, "Initializing modbus controller");
    // Initialize modbus in master mode, ready for reading/writing from/to modbus slaves.
    ESP_ERROR_CHECK(mbcontroller_init(MB_ROLE_MASTER, &xMBModbusSupervisorQueue));
    ESP_LOGD(TAG, "Modbus controller created, checking supervisor queue handle");
    if (xMBModbusSupervisorQueue == NULL) return eStatus;

    ESP_LOGD(TAG, "Supervisor queue handle OK, setting up modbus controller comm parameters");
    comm_info.mode       = MB_MODE_RTU;      // TODO make configurable by file or NVS settings!
    comm_info.port       = MB_UART_PORT;
    comm_info.baudrate   = MB_DEVICE_SPEED;
    comm_info.parity     = MB_PARITY_NONE;
    ESP_ERROR_CHECK(mbcontroller_setup(comm_info));

    ESP_LOGD(TAG, "Starting modbus controller");
    ESP_ERROR_CHECK(mbcontroller_start());

    xMBAppModbusQueue = xQueueCreate(MB_SUPERVISOR_NOTIFY_QUEUE_SIZE, sizeof(mb_supervisor_request_t));
    MB_CHECK((xMBAppModbusQueue != NULL),
            ESP_ERR_NO_MEM, "mb application queue creation error.");
 
    // for Nabto interfacing, switch modbus supervisor to silent mode (keeps completed requests in pending list until application asks for).
    ESP_LOGD(TAG, "Switching modbus supervisor to silent mode");
    svr.sender            = xMBAppModbusQueue;
    svr.req_resp.req_type = MB_SUPERV_CONFIG_KEEP;
    (void) xQueueSend(xMBModbusSupervisorQueue, &svr, MB_PAR_INFO_TOUT);
    ESP_LOGD(TAG, "Switch request sent, waiting for answer");
    if (xQueueReceive(xMBAppModbusQueue, (void *)&svr, portMAX_DELAY) == pdTRUE)
    {
        if (svr.req_resp.err_code == MB_MRE_NO_ERR) 
        {
            eStatus = ESP_OK;
        }
    }
    ESP_LOGI(TAG, "Modbus init finished with status: %8.8X", eStatus);
    // now we're ready to use the modbus master by sending requests and polling it for replies from it.
    
    return eStatus;
}

void modbus_rtu_app_main_task(void *pvParameter)
{
    nabto_credentials_t *nabto_credentials = (nabto_credentials_t *)pvParameter;

    ESP_LOGI(TAG, "Nabto app main task started.");
    ESP_ERROR_CHECK(setup_nabto(nabto_credentials));
    ESP_LOGI(TAG, "Nabto init completed.");

    while(1) 
    {
        unabto_tick();
        vTaskDelay(10 / portTICK_RATE_MS);
    }    
    // reached only in case of errors.
    unabto_close();
}

// poll active asynchronous queries to see if one has completed. We don't care about handling raw or formatted requests yet.
bool application_poll_query(application_request** request) 
{
    mb_supervisor_request_t  svr;

    // create no-descriptor unspecified peek request to modbus supervisor (returns first finished request found).
    svr.sender                = xMBAppModbusQueue;
    svr.req_resp.req_type     = MB_SUPERV_PEEK;
    svr.req_entry.req_id     = 0;
    ESP_LOGD(TAG, "sending unspecified peek request to modbus supervisor");
    if ( xQueueSend(xMBModbusSupervisorQueue, (void *)&svr, MB_PAR_INFO_TOUT) != pdTRUE )
    {
        NABTO_LOG_ERROR(("Unable to transfer Modbus message!"));
        return false;
    }
    if (xQueueReceive(xMBAppModbusQueue, (void *)&svr, MB_PAR_INFO_TOUT) != pdTRUE)
    {
        NABTO_LOG_ERROR(("Error from Modbus supervisor enqueueing message"));
        return false;
    }
    if (svr.req_resp.err_code != MB_MRE_NOTFOUND)
    {
        // we found a completed request and the receive returned a descriptor as well, which contains the matching application request.
        *request = (application_request*) svr.req_entry.descriptor->request_id;
        return true;
    }
    return false;
}


// end an asynchronous query that signalled its completion, called by the Nabto framework
application_event_result application_poll(application_request* request, buffer_write_t * writeBuffer)
{
    mb_supervisor_request_t  svr;

    // create specific receive request to modbus supervisor (returns matching request and removes from internal list).
    svr.sender                = xMBAppModbusQueue;
    svr.req_resp.req_type     = MB_SUPERV_RECEIVE;
    svr.req_entry.req_id      = (uint32_t)request;
    ESP_LOGD(TAG, "sending specific receive request for %8.8X to modbus supervisor", svr.req_entry.req_id);
    if ( xQueueSend(xMBModbusSupervisorQueue, (void *)&svr, MB_PAR_INFO_TOUT) != pdTRUE )
    {
        NABTO_LOG_ERROR(("Unable to transfer Modbus message!"));
        return AER_REQ_SYSTEM_ERROR;
    }
    if (xQueueReceive(xMBAppModbusQueue, (void *)&svr, MB_PAR_INFO_TOUT) != pdTRUE)
    {
        NABTO_LOG_ERROR(("Error from Modbus supervisor dequeueing message"));
        return AER_REQ_SYSTEM_ERROR;
    }
    if ((svr.req_resp.err_code == MB_MRE_PENDING) || (svr.req_resp.err_code == MB_MRE_NOTFOUND))
    {
        NABTO_LOG_ERROR(("Unexpected return code from dequeued message: %d", svr.req_resp.err_code));
        return AER_REQ_SYSTEM_ERROR;
    }
    return end_modbus_function_query(request, NULL, writeBuffer, svr.req_entry.descriptor);
}

// the Nabto framework calls this function when a connection is lost when a request is in progress
void application_poll_drop(application_request* request)
{
    mb_supervisor_request_t  svr;

    // create specific kill request to modbus supervisor (returns status of underlying requests, purges it from internal list and also frees the descriptor).
    svr.sender                = xMBAppModbusQueue;
    svr.req_resp.req_type     = MB_SUPERV_KILL;
    svr.req_entry.req_id      = (uint32_t)request;
    ESP_LOGD(TAG, "sending specific kill request for %8.8X to modbus supervisor", svr.req_entry.req_id);
    if ( xQueueSend(xMBModbusSupervisorQueue, (void *)&svr, MB_PAR_INFO_TOUT) != pdTRUE )
    {
        NABTO_LOG_ERROR(("Unable to transfer Modbus message!"));
        return;
    }
    if (xQueueReceive(xMBAppModbusQueue, (void *)&svr, MB_PAR_INFO_TOUT) != pdTRUE)
    {
        NABTO_LOG_ERROR(("Error from Modbus supervisor dequeueing message"));
        return;
    }

    if (svr.req_resp.err_code == MB_MRE_NOTFOUND)
    {
        NABTO_LOG_ERROR(("Request not found in FreeModbus Supervisor!"));
        return;
    }
    NABTO_LOG_TRACE(("Application event dropped."));
}

/**
 * Application event function for incoming Nabto req/res events
 */
application_event_result application_event(application_request* request,
                                           unabto_query_request* query_request,
                                           unabto_query_response* query_response) {

    NABTO_LOG_INFO(("Nabto application_event: %u", request->queryId));
    debug_dump_acl();

    // handle requests as defined in interface definition shared with
    // client - for the default demo, see
    // https://github.com/nabto/ionic-starter-nabto/blob/master/www/nabto/unabto_queries.xml

    application_event_result res;

    if (request->queryId >= 11000 && request->queryId < 12000) {
        // default PPKA access control (see unabto/src/modules/fingerprint_acl/fp_acl_ae.c)
        res = fp_acl_ae_dispatch(11000, request, query_request, query_response);
        NABTO_LOG_INFO(("ACL request [%d] handled with status %d", request->queryId, res));
        debug_dump_acl();
        return res;
    }
    
    switch (request->queryId) {
    case 0:
        // get_interface_info.json
        if (!write_string(query_response, device_interface_id_)) return AER_REQ_RSP_TOO_LARGE;
        if (!unabto_query_write_uint16(query_response, device_interface_version_major_)) return AER_REQ_RSP_TOO_LARGE;
        if (!unabto_query_write_uint16(query_response, device_interface_version_minor_)) return AER_REQ_RSP_TOO_LARGE;
        return AER_REQ_RESPONSE_READY;

    case 10000:
        // get_public_device_info.json
        if (!write_string(query_response, nabto_env_application_get_device_name())) return AER_REQ_RSP_TOO_LARGE;
        if (!write_string(query_response, nabto_env_application_get_device_product())) return AER_REQ_RSP_TOO_LARGE;
        if (!write_string(query_response, nabto_env_application_get_device_icon_())) return AER_REQ_RSP_TOO_LARGE;
        if (!unabto_query_write_uint8(query_response, fp_acl_is_pair_allowed(request))) return AER_REQ_RSP_TOO_LARGE;
        if (!unabto_query_write_uint8(query_response, fp_acl_is_user_paired(request))) return AER_REQ_RSP_TOO_LARGE; 
        if (!unabto_query_write_uint8(query_response, fp_acl_is_user_owner(request))) return AER_REQ_RSP_TOO_LARGE;
        return AER_REQ_RESPONSE_READY;

    case 10010:
        // set_device_info.json
        if (!fp_acl_is_request_allowed(request, REQUIRES_OWNER)) return AER_REQ_NO_ACCESS;
        int res = copy_string(query_request, nabto_env_application_get_device_name(), sizeof(nabto_env_application_get_device_name()));
        if (res != AER_REQ_RESPONSE_READY) return res;
        if (!write_string(query_response, nabto_env_application_get_device_name())) return AER_REQ_RSP_TOO_LARGE;
        return AER_REQ_RESPONSE_READY;

    case 11000:
        // get_users.json
        return fp_acl_ae_users_get(request, query_request, query_response); // implied admin priv check
        
    case 11010: 
        // pair_with_device.json
        if (!fp_acl_is_pair_allowed(request)) return AER_REQ_NO_ACCESS;
        res = fp_acl_ae_pair_with_device(request, query_request, query_response); 
        debug_dump_acl();
        return res;

    case 11020:
        // get_current_user.json
        return fp_acl_ae_user_me(request, query_request, query_response); 

    case 11030:
        // get_system_security_settings.json
        return fp_acl_ae_system_get_acl_settings(request, query_request, query_response); // implied admin priv check

    case 11040:
        // set_system_security_settings.json
        return fp_acl_ae_system_set_acl_settings(request, query_request, query_response); // implied admin priv check

    case 11050:
        // set_user_permissions.json
        return fp_acl_ae_user_set_permissions(request, query_request, query_response); // implied admin priv check

    case 11060:
        // set_user_name.json
        return fp_acl_ae_user_set_name(request, query_request, query_response); // implied admin priv check

    case 11070:
        // remove_user.json
        return fp_acl_ae_user_remove(request, query_request, query_response); // implied admin priv check

    case QUERY_MODBUS_FUNCTION_RAW:
        if (!fp_acl_is_request_allowed(request, REQUIRES_GUEST)) return AER_REQ_NO_ACCESS;
        return begin_modbus_raw_function_query(request, query_request, query_response);

    case QUERY_MODBUS_CONFIGURATION:
        if (!write_string(query_response, modbus_configuration)) return AER_REQ_RSP_TOO_LARGE;
        return AER_REQ_RESPONSE_READY;

    case QUERY_MODBUS_FUNCTION_FORMATTED:
        if (!fp_acl_is_request_allowed(request, REQUIRES_GUEST)) return AER_REQ_NO_ACCESS;
        return begin_modbus_formatted_function_query(request, query_request, query_response);

    default:
        NABTO_LOG_WARN(("Unhandled query id: %u", request->queryId));
        return AER_REQ_INV_QUERY_ID;
    }
}

