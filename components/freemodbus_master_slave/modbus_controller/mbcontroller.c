// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// mbcontroller.c
// Implementation of the modbus controller
// The modbus controller is responsible for processing of modbus packet and transfer data
// into parameter instance.

#include <sys/time.h>               // for calculation of time stamp in milliseconds
#include <string.h>                 // for helper functions like memset
#include "esp_log.h"                // for log_write
#include "freertos/FreeRTOS.h"      // for task creation and queue access
#include "freertos/task.h"          // for task api access
#include "freertos/event_groups.h"  // for event groups
#include "mb.h"                     // for mb types definition
#include "mbframe.h"                // for mb frame offset definitions
#include "mbutils.h"                // for mbutils functions definition for stack callback
#include "sdkconfig.h"              // for KConfig values
#include "mbcontroller.h"
#include "pool.h"                   // for deterministic pool memory management

static const char* TAG = "MB_CNTRL";
static const char* TAG_SV = "MB_SUPERV";
#define MB_LOG(...) ESP_LOGI(__VA_ARGS__)

#define MB_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

// The Macros below handle the endianness while transfer N byte data into buffer
#define _XFER_4_RD(dst, src) { \
    *(uint8_t *)(dst)++ = *(uint8_t*)(src + 1); \
    *(uint8_t *)(dst)++ = *(uint8_t*)(src + 0); \
    *(uint8_t *)(dst)++ = *(uint8_t*)(src + 3); \
    *(uint8_t *)(dst)++ = *(uint8_t*)(src + 2); \
    (src) += 4; \
}

#define _XFER_2_RD(dst, src) { \
    *(uint8_t *)(dst)++ = *(uint8_t *)(src + 1); \
    *(uint8_t *)(dst)++ = *(uint8_t *)(src + 0); \
    (src) += 2; \
}

#define _XFER_4_WR(dst, src) { \
    *(uint8_t *)(dst + 1) = *(uint8_t *)(src)++; \
    *(uint8_t *)(dst + 0) = *(uint8_t *)(src)++; \
    *(uint8_t *)(dst + 3) = *(uint8_t *)(src)++; \
    *(uint8_t *)(dst + 2) = *(uint8_t *)(src)++ ; \
}

#define _XFER_2_WR(dst, src) { \
    *(uint8_t *)(dst + 1) = *(uint8_t *)(src)++; \
    *(uint8_t *)(dst + 0) = *(uint8_t *)(src)++; \
}

#ifdef CONFIG_MB_CONTROLLER_SLAVE_ID_SUPPORT

#define MB_ID_BYTE0(id) ((uint8_t)(id))
#define MB_ID_BYTE1(id) ((uint8_t)(((uint16_t)(id) >> 8) & 0xFF))
#define MB_ID_BYTE2(id) ((uint8_t)(((uint32_t)(id) >> 16) & 0xFF))
#define MB_ID_BYTE3(id) ((uint8_t)(((uint32_t)(id) >> 24) & 0xFF))

#define MB_CONTROLLER_SLAVE_ID (CONFIG_MB_CONTROLLER_SLAVE_ID)
#define MB_SLAVE_ID_SHORT      (MB_ID_BYTE3(MB_CONTROLLER_SLAVE_ID))

/* ------------------- Slave role variables ------------------------------------ */

// Slave ID constant
static uint8_t mb_slave_id[] = { MB_ID_BYTE0(MB_CONTROLLER_SLAVE_ID),
                                MB_ID_BYTE1(MB_CONTROLLER_SLAVE_ID),
                                MB_ID_BYTE2(MB_CONTROLLER_SLAVE_ID) };

#endif

// This is array of Modbus address area descriptors
static mb_register_area_descriptor_t mb_area_descriptors[MB_PARAM_COUNT] = { 0 };


/* ------------------- Role independent variables ------------------------------ */

// TODO need to be instance specific, not global anymore to be able to handle multiple buses!

// Event group parameters
static TaskHandle_t mb_controller_task_handle = NULL;
static EventGroupHandle_t mb_controller_event_group = NULL;
static QueueHandle_t mb_controller_notification_queue_handle = NULL;

static BOOL mb_init_done = FALSE;
static uint8_t mb_role = 0;
static uint8_t mb_type = 0;
static uint8_t mb_address = 0;
static uint8_t mb_port = 0;
static uint32_t mb_speed = 0;
static uint16_t mb_parity = 0;

/* ------------------ Master role variables ------------------------------------ */


static volatile uint32_t global_master_req_id = 1; // start with non-zero value
static uint16_t mb_supervisor_throttle = MAX_SIMULTANEOUS_MODBUS_REQUESTS_PER_INSTANCE;
static uint16_t mb_supervisor_current_requests = 0;
static TaskHandle_t mb_supervisor_task_handle = NULL;
static QueueHandle_t xMBSupervisorQueue;           // signal queue betweeen application and modbus supervisor
TAILQ_HEAD(, mb_request_entry) mb_controller_pending_requests_tailq_head;

static const int DISCRETE_DONE_BIT = BIT1;
static const int INPUTREG_DONE_BIT = BIT2;

static EventGroupHandle_t discrete_done_event_group;
static EventGroupHandle_t input_reg_done_event_group;

/* --------------------- Static functions for Slave role ----------------------- */

// The helper function to get time stamp in microseconds
static uint64_t get_time_stamp()
{
    uint64_t time_stamp = esp_timer_get_time();
    return time_stamp;
}

// Helper function to send parameter information to application task
static esp_err_t send_param_info(mb_event_group_t par_type, uint16_t mb_offset,
                                    uint8_t* par_address, uint16_t par_size)
{
    esp_err_t error = ESP_FAIL;
    mb_param_info_t par_info;
    // Check if queue is not full the send parameter information
    par_info.type = par_type;
    par_info.size = par_size;
    par_info.address = par_address;
    par_info.time_stamp = get_time_stamp();
    par_info.mb_offset = mb_offset;
    BaseType_t status = xQueueSend(mb_controller_notification_queue_handle, &par_info, MB_PAR_INFO_TOUT);
    if (pdTRUE == status) {
        ESP_LOGD(TAG, "Queue send parameter info (type, address, size): %d, 0x%.4x, %d",
                par_type, (uint32_t)par_address, par_size);
        error = ESP_OK;
    } else if (errQUEUE_FULL == status) {
        ESP_LOGD(TAG, "Parameter queue is overflowed.");
    }
    return error;
}

static esp_err_t send_param_access_notification(mb_event_group_t event)
{
    esp_err_t err = ESP_FAIL;
    mb_event_group_t bits = (mb_event_group_t)xEventGroupSetBits(mb_controller_event_group, (EventBits_t)event);
    if (bits & event) {
        ESP_LOGD(TAG, "The MB_REG_CHANGE_EVENT = 0x%.2x is set.", (uint8_t)event);
        err = ESP_OK;
    }
    return err;
}

/* --------------------- Static functions for master role ------------------- */

/* Prepares function-specific request from a given request descriptor. This function is supposed
 * to be called from the Modbus supervisor task only.
 *
 *
 */
static eMBMasterReqErrCode mbcontroller_request(struct mb_request_entry *mb_new_request)
{
    sMBMasterQueueType *ps_internal_frame_descriptor;

    if (mb_init_done == FALSE)
    {
        return MB_EILLSTATE;
    }

    if (mb_new_request == NULL)
    {
        return MB_MRE_ILL_ARG;
    }
   
    // Use request ID from caller or automatically assign a new request ID (if zero provided).
    if (mb_new_request->request_id == 0)
    {
        mb_new_request->request_id = global_master_req_id;
        global_master_req_id++;
    }

    ps_internal_frame_descriptor = (sMBMasterQueueType *)palloc();
    if (ps_internal_frame_descriptor == NULL)
    {
        return MB_MRE_OUTOFMEM;
    }
    // The request ID needs to be available to the lower layer as well, so copy it to the modbus master descriptor
    ps_internal_frame_descriptor->request_id = mb_new_request->request_id;


    if (mb_new_request->is_raw_request == TRUE)
    { 
        ESP_LOGD(TAG_SV, "Raw Modbus frame command sending request");
        return eMBMasterReqRawModbusFrames( mb_new_request->u.f.slave_address, mb_new_request->u.raw_frame, mb_new_request->raw_frame_length, T_WAIT_FOREVER, ps_internal_frame_descriptor);
    } else {
        switch (mb_new_request->u.f.function_code)
        {
            case 0x01: // read coil(s)
                ESP_LOGD(TAG_SV, "Read coils command sending request");
                return eMBMasterReqReadCoils(mb_new_request->u.f.slave_address, mb_new_request->u.f.read_start, mb_new_request->u.f.read_len, 
                                             T_WAIT_FOREVER, ps_internal_frame_descriptor);     
            case 0x02: // read discrete input(s)
                ESP_LOGD(TAG_SV, "Read discrete inputs command sending request");
                return eMBMasterReqReadDiscreteInputs(mb_new_request->u.f.slave_address, mb_new_request->u.f.read_start, mb_new_request->u.f.read_len, 
                                                      T_WAIT_FOREVER, ps_internal_frame_descriptor);     
            case 0x03: // read holding register(s)
                ESP_LOGD(TAG_SV, "Read holding register command sending request");
                return eMBMasterReqReadHoldingRegister(mb_new_request->u.f.slave_address, mb_new_request->u.f.read_start, mb_new_request->u.f.read_len, 
                                                       T_WAIT_FOREVER, ps_internal_frame_descriptor);
            case 0x04: // read input register(s)
                ESP_LOGD(TAG_SV, "Read input register command sending request");
                return eMBMasterReqReadInputRegister(mb_new_request->u.f.slave_address, mb_new_request->u.f.read_start, mb_new_request->u.f.read_len, 
                                                     T_WAIT_FOREVER, ps_internal_frame_descriptor);
            case 0x05: // write single coil
                ESP_LOGD(TAG_SV, "Write single coil command sending request");
                return eMBMasterReqWriteCoil(mb_new_request->u.f.slave_address, mb_new_request->u.f.write_start, 
                                             (mb_new_request->u.f.unpacked_data[0] == 0 ? 0 : 0xFF00), T_WAIT_FOREVER, ps_internal_frame_descriptor);
            case 0x06: // write single holding register
                ESP_LOGD(TAG_SV, "Write single holding register command sending request");
                return eMBMasterReqWriteHoldingRegister(mb_new_request->u.f.slave_address, mb_new_request->u.f.write_start, 
                                             mb_new_request->u.f.unpacked_data[0], T_WAIT_FOREVER, ps_internal_frame_descriptor);
            case 0x0f: // write multiple coils
                ESP_LOGD(TAG_SV, "Write multiple coils command sending request");
                {
                    UCHAR   coilData[MAX_COIL_BUF_SIZE];
                    USHORT *unpackedData;
                    UCHAR   coilBitCnt = 0;
                    UCHAR   coilIndex = 0;

                    memset(coilData, 0, MAX_COIL_BUF_SIZE);
                    unpackedData = mb_new_request->u.f.unpacked_data;

                    // stuff bits into bitfields for modbus command
                    for (int i=0;i<mb_new_request->u.f.write_len;i++)
                    {
                        if (*unpackedData++ != 0)  
                        {
                           coilData[coilIndex] |= (1<<coilBitCnt);              
                        }
                        coilBitCnt++;
                        if ((coilBitCnt %8) == 0)
                        {
                            coilBitCnt = 0;
                            coilIndex++;
                        }
                    }
                    return eMBMasterReqWriteMultipleCoils(mb_new_request->u.f.slave_address, mb_new_request->u.f.write_start, mb_new_request->u.f.write_len, 
                                                          coilData, T_WAIT_FOREVER, ps_internal_frame_descriptor);
                }
            case 0x10: // write multiple holding registers
                ESP_LOGD(TAG_SV, "Write multiple holding registers command sending request");
                return eMBMasterReqWriteMultipleHoldingRegister(mb_new_request->u.f.slave_address, 
                                             mb_new_request->u.f.write_start, mb_new_request->u.f.write_len, mb_new_request->u.f.unpacked_data, 
                                             T_WAIT_FOREVER, ps_internal_frame_descriptor);

            case 0x17: // read and write multiple holding registers        
                ESP_LOGD(TAG_SV, "Read+Write multiple holding registers command sending request");
                return eMBMasterReqReadWriteMultipleHoldingRegister(mb_new_request->u.f.slave_address, 
                                             mb_new_request->u.f.read_start, mb_new_request->u.f.read_len, mb_new_request->u.f.unpacked_data,
                                             mb_new_request->u.f.write_start, mb_new_request->u.f.write_len, T_WAIT_FOREVER, ps_internal_frame_descriptor);
            default: // unknown function code
                ESP_LOGW(TAG_SV, "Illegal request received");
                pfree(ps_internal_frame_descriptor); // not needed anymore for illegal request
                return MB_MRE_ILL_FCT;
        }
    }
}

/* Unpacks received data from an internal frame to modbus supervisor descriptor, if not in raw modbus frame mode.
 * This function is supposed to be called from the Modbus supervisor task only, if a receive was successful.
 * The status has already been updated, so we need to get the payload only.
 *
 */
static eMBMasterReqErrCode mbcontroller_unpack(struct mb_request_entry *mb_external_descriptor, sMBMasterQueueType *mb_internal_descriptor)
{
    USHORT               usRegContent;
    USHORT               usRegCount;
    UCHAR               *pucFrame;
    eMBMasterReqErrCode  eStatus = MB_MRE_NO_ERR;

    if ((mb_external_descriptor == NULL) || (mb_internal_descriptor == NULL) )
    {
        return MB_MRE_ILL_ARG;
    }

    if ( MASTER_REQUEST_IS_BROADCAST(mb_internal_descriptor) )
    { 
        // no data expected from broadcast read.
        return eStatus;
    }

    usRegCount   = mb_external_descriptor->u.f.read_len;
    pucFrame     = mb_internal_descriptor->rcv_frame + MB_PDU_FUNC_READ_VALUES_OFF; 

    // Unpack from internal descriptor (raw modbus frame) to external descriptor, depending on modbus function code.
    switch (mb_external_descriptor->u.f.function_code)
    {
        case 0x01: // read coil(s)
        case 0x02: // read discrete input(s)
            ESP_LOGD(TAG_SV, "Unpacking bitfields");
            // response format is the same for commands 0x01 and 0x02. Unpack coils (bitfield) from frame to unpacked_data array.
            for (int i=0, j=0, byte=0; i<usRegCount ; i++)
            {
                usRegContent = (((*pucFrame) & (1<<j)) == 0) ? 0 : 1;
                if (++j==8)
                {
                    j = 0;
                    byte++;
                }
            }
            break;
        case 0x03: // read holding register(s)
        case 0x04: // read input register(s)
        case 0x17: // read and write multiple holding registers        
            ESP_LOGD(TAG_SV, "Unpacking registers");
            // response format is the same for commands 0x03, 0x04 and 0x17. Copy 16-bit data fromframe to unpacked_data array.
            for (int i=0; i<usRegCount;i++)
            {
                usRegContent  = ((USHORT)*pucFrame++) << 8;
                usRegContent |= *pucFrame++;
                mb_external_descriptor->u.f.unpacked_data[i] = usRegContent;
            }
            break;
        default:   // all other write-only commands don't need updating of data.
            break;
    }
    return eStatus;
}

/* --------------------- Main Modbus task ----------------------------------- */

// Modbus task function
static void modbus_task(void *pvParameters)
{
    // Main Modbus stack processing cycle
    for (;;) {
        BaseType_t status = xEventGroupWaitBits(mb_controller_event_group,
                                                (BaseType_t)(MB_EVENT_STACK_STARTED),
                                                pdFALSE, // do not clear bits
                                                pdFALSE,
                                                portMAX_DELAY);
        // Check if stack started then poll for data
        if (status & MB_EVENT_STACK_STARTED)
        {
            (void)eMBPoll(); // allow stack to process data
            (void)xMBPortSerialTxPoll(); // Send response buffer if ready
        }
    }
}

/* --------------------- Modbus controller supervisor task --------------------- */
/* This task handles the application interface to Modbus lower layer functions.
 * It is signal based (using FreeRTOS queues) and uses request-response structure.
 *
 * Always pass a queue entry of type mb_supervisor_request_t, and it will return
 * the response to the sender (sender queue must be provided by sender as there is
 * no FreeRTOS built-in mechanism for that).
 *
 * Parameters required for every request:
 * 
 * xRequest.sender            - Queue handle where sender will listen to
 * xRequest.req_resp.req_type - Request type
 *
 * All requests return a status in xRequest.req_resp.err_code  (overwriting the request as it is an union).
 *
 * Requests types:
 *
 * MB_SUPERV_CONFIG_KEEP: Sets supervisor task to maintaining pending queue mode [DEFAULT]. Completed requests will silently 
 *                        stay in pending queue. Config request will always return success to sender.
 *               
 * MB_SUPERV_CONFIG_AUTORESPOND: Sets supervisor task to return completed requests to original sender automatically, the pending
 *                        queue will be cleared from it. Config request itself will always return success to sender.
 *
 * MB_SUPERV_CONFIG_THROTTLE: Sets throttle (maximum number of simultaneous running requests) in modbus supervisor.
 *                       Use xRequest.req_entry.req_id as parameter.  Not calling this will set throttling to default (10).
 *
 * MB_SUPERV_ALLOC_DESC: Allocates a new descriptor from memory pool.
 *                       Returns descriptor in xRequest.req_entry.descriptor (or NULL if it failed to allocate).                       
 * 
 * MB_SUPERV_RELEASE_DESC: Requires descriptor in xRequest.req_entry.descriptor.
 *                       Descriptor is released to the pool and cannot be used afterwards anymore, returning NULL.
 * 
 * MB_SUPERV_REQ:        Requires descriptor in xRequest.req_entry.descriptor, filled with content according to desired Modbus request.
 *                       Descriptor is placed in pending list and cannot be used by caller afterwards, instead of the
 *                       descriptor pointer the request ID is returned.
 *                       The request ID is created as follows:
 *                       - If the caller provides a non-zero request ID in xRequest.req_entry.descriptor.request_id, it will be used
 *                       - If this field is zero on request, a consecutive-counting request ID is automatically assigned. 
 *                       - It is not recommended to mix both: Either always provide request IDs from the caller, or use zero and let
 *                       the supervisor assign it.
 *
 *                       Calls the appropriate request function and sends a frame descriptor to main modbus task.
 *                       The request descriptor is kept in the pending queue (not sent anywhere else).
 *
 *                       Furthermore, requests can either be based on formatted lists (recommended) or raw modbus frames.
 * 
 *                       For formatted requests fill in:
 *                       xRequest.req_entry.descriptor.u.f.<fields> with <fields> being slave_address, function_code, 
 *                                                                  read_start, read_len, write_start, write_len and unpacked_data[].
 *                       and set xRequest.req_entry.descriptor.is_raw_request to FALSE. 
 *
 *                       For raw modbus requests instead fill in:
 *                       xRequest.req_entry.descriptor.u.raw_frame[] with the raw modbus frame.
 *                       and set xRequest.req_entry.descriptor.is_raw_request to TRUE. 
 *                       NOTE: For raw modbus requests, no error checking will be performed, the frame is taken as is.
 *
 * MB_SUPERV_PEEK:       This can be called with a specific request ID in mind - in this case request ID in xRequest.req-entry.req_id needs to be filled in.
 *                       If the request ID is NULL it returns the first finished request (if any is in the list)
 *                       1. In case of a given request: 
 *                          - Returns descriptor if found, and the status code as defined in eMBMasterReqErrCode.
 *                            MB_MRE_PENDING means the request has not yet finished (but didn't create an error so far), 
 *                            MB_MRE_NOTFOUND if it is not in the pending list (in this case a NULL descriptor is returned).
 *                            MB_MRE_OUTOFMEM cannot happen for this request, 
 *                            MB_MRE_NO_ERR means the request is finished, and the returned descriptor contains everything requested
 *                            (in case of read requests).
 *                            All other return codes mean the request has finished but with error (most likely timed out).
 *                       2. In case of an unspecific search (request ID set to NULL):
 *                          - If no finished request was found an the list was not empty, returns no descriptor and MB_MRE_PENDING.
 *                          - If the list was empty, returns MB_MRE_NOTFOUND (and no descriptor of course).
 *                          - If a finished request was found, returns its descriptor, and its status code as defined in eMBMasterReqErrCode.
 *                            MB_MRE_NO_ERR means the request is finished, and the returned descriptor contains everything requested
 *                            (in case of read requests).
 *                            All other return codes mean the request has finished but with error (most likely timed out).
 *                            (however MB_MRE_PENDING, MB_MRE_NOTFOUND and MR_MBER_OUTOFMEM won't occur in this case).
 *                       In all cases, the descriptor stays in the pending list.
 *  
 * 
 * MB_SUPERV_RECEIVE:    Same as MB_SUPERV_PEEK, but the descriptor is removed from the pending list.
 *                       The caller is required to use the returned descriptor and free it if not needed anymore.
 *
 * MB_SUPERV_KILL:       Similar as MB_SUPERV_RECEIVE, but the descriptor is not only removed from the pending list, but also
 *                       returned to the pool if it existed (so it cannot be used anymore and NULL is returned in the descriptor field).
 *                       The return code still has the same meaning as in MB_SUPERV_PEEK.
 * 
 * MB_SUPERV_RESPONSE:   This is called from lower layers only with a received frame (so xRequest.req_entry.descriptor actually points 
 *                       to a structure of type "struct mb_int_frame_transfer". This request decodes the modbus response,
 *                       (which can contain data, or might be just a timeout status in xRequest.req_resp.err_code) and updates
 *                       status in appropriate descriptor of the pending list.
 *                       If set to autorespond mode, the original sender of the request will be notified. Otherwise, the 
 *                       status is just maintained in the pending queue until it is picked up by PEEK or RECEIVE (or KILL) requests.
 * 
 *    TODO handle retransmits in case of certain errors here. Descriptor already contains a field "retransmit_count" which shall be used -
 *         Decrement retransmit count, if zero not reached submit the descriptor to myself as new request (MB_SUPERV_REQ).
 *         Only if retransmit count is zero, keep the error silently in the list (or respond to caller in case of autorespond option on)
 */

static void modbus_supervisor_task(void *pvParameters) 
{
    QueueHandle_t            sender;
    mb_supervisor_request_t  xRequest;
    struct mb_request_entry *local_desc;
    struct mb_request_entry *local_desc_temp;
    uint32_t                 local_req_id;
    BOOL                     bConfigReplyCompleted = FALSE;
    BOOL                     xNeedUnlinkPending;
    BOOL                     xNeedRemoveDescriptor;

    TAILQ_INIT(&mb_controller_pending_requests_tailq_head); // prepare pending lists for request descriptors

    // Modbus supervisor - application-facing request, response and timeout/retransmit handler.
    for (;;) 
    {
        if (xQueueReceive(xMBSupervisorQueue, (void *)&xRequest, portMAX_DELAY) == pdTRUE)
        {
            xNeedUnlinkPending = FALSE;
            xNeedRemoveDescriptor = FALSE;
            sender = xRequest.sender;
            xRequest.sender = xMBSupervisorQueue;
            ESP_LOGD(TAG_SV, "MB Supervisor request received - type=%d from %8.8X:", xRequest.req_resp.req_type, (uint32_t) sender);
            switch (xRequest.req_resp.req_type)
            {
                case MB_SUPERV_CONFIG_KEEP:
                    bConfigReplyCompleted = FALSE;
                    xRequest.req_resp.err_code = MB_MRE_NO_ERR;
                    (void) xQueueSend(sender, &xRequest, MB_PAR_INFO_TOUT);
                    break;
                case MB_SUPERV_CONFIG_AUTORESPOND:
                    bConfigReplyCompleted = TRUE;
                    xRequest.req_resp.err_code = MB_MRE_NO_ERR;
                    (void) xQueueSend(sender, &xRequest, MB_PAR_INFO_TOUT);
                    break;
                case MB_SUPERV_CONFIG_THROTTLE:
                    xRequest.req_resp.err_code = MB_MRE_NO_ERR;
                    mb_supervisor_throttle = (uint16_t)xRequest.req_entry.req_id;
                    (void) xQueueSend(sender, &xRequest, MB_PAR_INFO_TOUT);
                    break;
                case MB_SUPERV_ALLOC_DESC:
                    // allocate descriptor from pool and return to sender.
                    xRequest.req_entry.descriptor = (struct mb_request_entry *)palloc();
                    xRequest.req_resp.err_code = (xRequest.req_entry.descriptor == NULL) ? MB_MRE_OUTOFMEM: MB_MRE_NO_ERR;
                    (void) xQueueSend(sender, &xRequest, MB_PAR_INFO_TOUT);
                    break;
                case MB_SUPERV_RELEASE_DESC:
                    // return descriptor to pool and return a good status to sender.
                    pfree((void *)xRequest.req_entry.descriptor);
                    xRequest.req_entry.descriptor = NULL;
                    xRequest.req_resp.err_code = MB_MRE_NO_ERR;
                    (void) xQueueSend(sender, &xRequest, MB_PAR_INFO_TOUT);
                    break;
                case MB_SUPERV_REQ:
                    if (mb_supervisor_current_requests > mb_supervisor_throttle)
                    { 
                        xRequest.req_resp.err_code = MB_MRE_OUTOFMEM;
                    } else { 
                        // call appropriate request function, place descriptor into pending queue and return request ID and status of called request to sender.
                        local_desc = xRequest.req_entry.descriptor;                
                        ESP_LOGD(TAG_SV, "Request from application received, descriptor: %8.8X", (uint32_t)local_desc);
                        if (local_desc == NULL)
                        {
                            xRequest.req_resp.err_code = MB_MRE_ILL_ARG;
                        } else {
                            local_desc->req_origin = sender;
                            local_desc->result = MB_MRE_PENDING;                            // all new requests are pending initially
                            xRequest.req_resp.err_code = mbcontroller_request(local_desc);  // this will send a frame to main modbus task
                            if (xRequest.req_resp.err_code == MB_MRE_NO_ERR)                // add descriptor to pending list if this was successful.
                            {
                                TAILQ_INSERT_TAIL(&mb_controller_pending_requests_tailq_head, local_desc, entries);
                                mb_supervisor_current_requests++;
                                xRequest.req_entry.req_id = local_desc->request_id;         // this overwrites the descriptor pointer in the response as we keep it
                            }   
                        }
                    }
                    (void) xQueueSend(sender, &xRequest, MB_PAR_INFO_TOUT);
                    break;
                case MB_SUPERV_KILL:
                    ESP_LOGD(TAG_SV, "Kill pending entry request received (followed by receive)");
                    // search request in pending list, remove it from pending list and return the descriptor (if found) to the pool without using (and no response).
                    xNeedRemoveDescriptor = TRUE;
                    /* FALLTHROUGH */
                case MB_SUPERV_RECEIVE:
                    ESP_LOGD(TAG_SV, "Receive pending entry request received (followed by peek)");
                    // search request in pending list and return descriptor if found (if not return error status), and remove it from pending list if it existed.
                    xNeedUnlinkPending = TRUE;
                    /* FALLTHROUGH */
                case MB_SUPERV_PEEK:
                    ESP_LOGD(TAG_SV, "Peek pending entry request received");
                    // search request in pending list and return descriptor if found (if not return error status), leave it in the pending list if it existed.
                    local_req_id = xRequest.req_entry.req_id;
                    xRequest.req_resp.err_code = MB_MRE_NOTFOUND;
                    xRequest.req_entry.descriptor = NULL;
                    local_desc = NULL;
                    TAILQ_FOREACH_SAFE(local_desc, &mb_controller_pending_requests_tailq_head, entries, local_desc_temp)
                    {
                        if ( ((local_req_id == 0) && (local_desc->result != MB_MRE_PENDING)) ||
                              (local_req_id == local_desc->request_id) )
                        {
                            xRequest.req_entry.descriptor = local_desc;
                            xRequest.req_resp.err_code = local_desc->result;
                            if (xNeedUnlinkPending)
                            {
                                mb_supervisor_current_requests--;
                                TAILQ_REMOVE(&mb_controller_pending_requests_tailq_head, local_desc, entries);
                            }
                            if (xNeedRemoveDescriptor)
                            {
                                pfree((void *)local_desc);
                                xRequest.req_entry.descriptor = NULL;
                                xRequest.req_resp.err_code = MB_MRE_NO_ERR;
                            }
                        }
                    }
                    (void) xQueueSend(sender, &xRequest, MB_PAR_INFO_TOUT); // on success returns the found descriptor.
                    break;
                case MB_SUPERV_RESPONSE: // Signal originators are the Master callback functions below. 
                    ESP_LOGD(TAG_SV, "Data from modbus task received for processing");
                    {
                        
                        /* NOTE The only source of this request is the modbus task (via eMBFunc* handlers). We only need the frame descriptor, 
                         * (actually it is of a different type!), other fields in the request itself like sender are not important.
                         * Process the request and return the descriptor to the pool (it is not needed anymore). In case of an
                         * illegal request (no descriptor), silently drop it and don't return any error.
                         */
                        sMBMasterQueueType  *ps_internal_frame_descriptor = (sMBMasterQueueType *)xRequest.req_entry.descriptor;

                        ESP_LOGD(TAG_SV, "Frame descriptor received: %8.8X", (uint32_t) ps_internal_frame_descriptor);
                        if (ps_internal_frame_descriptor != NULL)
                        {
                            // Search matching request in pending queue and update the status if found.
                            local_req_id = ps_internal_frame_descriptor->request_id;
                            local_desc = NULL;
                            TAILQ_FOREACH_SAFE(local_desc, &mb_controller_pending_requests_tailq_head, entries, local_desc_temp)
                            {
                                if (local_req_id == local_desc->request_id)
                                {
                                    ESP_LOGD(TAG_SV, "result status received (request ID %d): %d, local_desc application-facing descriptor is %8.8X", 
                                                      local_req_id, local_desc->result, (uint32_t)local_desc);
                                    local_desc->result = ps_internal_frame_descriptor->error_type;
                                    if (local_desc->result == MB_MRE_NO_ERR)
                                    {
                                        if (local_desc->is_raw_request == TRUE)
                                        { 
                                            // in raw modbus frame mode, we don't unpack but just copy the received frame back, including the slave addres (first byte) so the length increases by one.
                                            if (ps_internal_frame_descriptor->pdu_recv_length <= MB_SER_PDU_SIZE_MAX)
                                            {
                                                local_desc->raw_frame_length = ps_internal_frame_descriptor->pdu_recv_length+1;
                                                memcpy(local_desc->u.raw_frame, ps_internal_frame_descriptor->rcv_frame, local_desc->raw_frame_length);
                                            }
                                        } else {
                                            if ( ((local_desc->u.f.function_code >= 0x01) && (local_desc->u.f.function_code <= 0x04)) || (local_desc->u.f.function_code >= 0x17) )
                                            {
                                               // If we received a successful read [or read/write] return, unpack and update the data in the descriptor.
                                                local_desc->result = mbcontroller_unpack(local_desc, ps_internal_frame_descriptor);
                                            }
                                        }
                                    } else {
                                        ESP_LOGD(TAG_SV, "received error result %d, not copying/unpacking anything", local_desc->result);
                                    }

                                    xRequest.req_entry.descriptor = local_desc;
                                    xRequest.req_resp.err_code = xRequest.req_entry.descriptor->result;
                                    /* if we should respond immediately, send return message to sender. In this case also purge from pending queue.
                                     * (Otherwise silently keep in pending queue. )
                                     */
                                    if (bConfigReplyCompleted == TRUE)
                                    {
                                        (void) xQueueSend(local_desc->req_origin, &xRequest, MB_PAR_INFO_TOUT);
                                        mb_supervisor_current_requests--;
                                        TAILQ_REMOVE(&mb_controller_pending_requests_tailq_head, local_desc, entries);
                                    }
                                }
                            }
                            pfree(ps_internal_frame_descriptor); // free received FRAME descriptor (not the application descriptor for the response)
                        }
                    }
                    break;
                default: // drop unknown events.
                    break;
            }
        }
    }
}


/* --------------------- Implementation SLAVE role only ----------------------------*/
esp_err_t mbcontroller_set_descriptor(const mb_register_area_descriptor_t descr_info)
{
    MB_CHECK(((descr_info.type < MB_PARAM_COUNT) && (descr_info.type >= MB_PARAM_HOLDING)),
                ESP_ERR_INVALID_ARG, "mb incorrect modbus instance type = (0x%x).",
                (uint32_t)descr_info.type);
    MB_CHECK((descr_info.address != NULL),
                ESP_ERR_INVALID_ARG, "mb instance pointer is NULL.");
    MB_CHECK((descr_info.size >= MB_INST_MIN_SIZE) && (descr_info.size < (MB_INST_MAX_SIZE)),
                ESP_ERR_INVALID_ARG, "mb instance size is incorrect = (0x%x).",
                (uint32_t)descr_info.size);
    mb_area_descriptors[descr_info.type].type = descr_info.type;
    mb_area_descriptors[descr_info.type].start_offset = descr_info.start_offset;
    mb_area_descriptors[descr_info.type].address = (uint8_t*)descr_info.address;
    mb_area_descriptors[descr_info.type].size = descr_info.size;
    return ESP_OK;
}

/* --------------------- Implementation MASTER role only ---------------------------*/
// helpers to directly allocate/free descriptors for modbus supervisor.
esp_err_t mbsupervisor_buf_alloc(struct mb_request_entry **mb_new_request)
{
    *mb_new_request = (struct mb_request_entry *)palloc();
    return (*mb_new_request == NULL) ? ESP_FAIL : ESP_OK;
}
void mbsupervisor_buf_free(struct mb_request_entry *mb_new_request)
{
    pfree((void *)mb_new_request);
}

/* --------------------- Implementation all roles ----------------------------------*/

// Initialization of Modbus controller and supervisor
esp_err_t mbcontroller_init(eMBRole role, QueueHandle_t *sv_queue)
{
    mb_role = (uint8_t) role;
    mb_type = MB_MODE_RTU;
    mb_address = MB_DEVICE_ADDRESS;
    mb_port = MB_UART_PORT;
    mb_speed = MB_DEVICE_SPEED;
    mb_parity = MB_PARITY_NONE;

    // Initialization of the deterministic memory pool allocator
    pool_init();

    // Initialization of active context of the modbus controller
    BaseType_t status = 0;
    // Parameter change notification queue
    mb_controller_event_group = xEventGroupCreate();
    MB_CHECK((mb_controller_event_group != NULL),
            ESP_ERR_NO_MEM, "mb event group error.");

    if (role == MB_ROLE_MASTER)
    {
        discrete_done_event_group = xEventGroupCreate();
        MB_CHECK((mb_controller_event_group != NULL),
            ESP_ERR_NO_MEM, "discrete_done_event_group error.");
        input_reg_done_event_group = xEventGroupCreate();
        MB_CHECK((mb_controller_event_group != NULL),
            ESP_ERR_NO_MEM, "input_reg_done_event_group error.");
    }

    // Parameter change notification queue
    mb_controller_notification_queue_handle = xQueueCreate(
                                                MB_CONTROLLER_NOTIFY_QUEUE_SIZE,
                                                sizeof(mb_param_info_t));
    MB_CHECK((mb_controller_notification_queue_handle != NULL),
            ESP_ERR_NO_MEM, "mb notify queue creation error.");
    // Create modbus controller task
    status = xTaskCreate((void*)&modbus_task,
                            "modbus_task",
                            MB_CONTROLLER_STACK_SIZE,
                            NULL,
                            MB_CONTROLLER_PRIORITY,
                            &mb_controller_task_handle);
    if (status != pdPASS) {
        vTaskDelete(mb_controller_task_handle);
        MB_CHECK((status == pdPASS), ESP_ERR_NO_MEM,
                "mb controller task creation error, xTaskCreate() returns (0x%x).",
                (uint32_t)status);
    }
    assert(mb_controller_task_handle != NULL); // The task is created but handle is incorrect


    // initialize/start modbus supervisor queue and task
    xMBSupervisorQueue = xQueueCreate(MB_SUPERVISOR_NOTIFY_QUEUE_SIZE, sizeof(mb_supervisor_request_t));
    MB_CHECK((xMBSupervisorQueue != NULL),
            ESP_ERR_NO_MEM, "mb supervisor queue creation error.");
    *sv_queue = xMBSupervisorQueue; // return also to caller

    // Create modbus supervisor task
    status = xTaskCreate((void*)&modbus_supervisor_task,
                            "modbus_supervisor_task",
                            MB_SUPERVISOR_STACK_SIZE,
                            NULL,
                            MB_CONTROLLER_PRIORITY,
                            &mb_supervisor_task_handle);
    if (status != pdPASS) {
        vTaskDelete(mb_supervisor_task_handle);
        vTaskDelete(mb_controller_task_handle);
        MB_CHECK((status == pdPASS), ESP_ERR_NO_MEM,
                "mb supervisor task creation error, xTaskCreate() returns (0x%x).",
                (uint32_t)status);
    }
    assert(mb_supervisor_task_handle != NULL); // The task is created but handle is incorrect

    mb_init_done = TRUE;
    return ESP_OK;
}

// Start Modbus controller start function
esp_err_t mbcontroller_start(void)
{
    sMBMasterQueueType *ps_internal_frame_descriptor;

    if (mb_init_done == FALSE)
    {
        return MB_EILLSTATE;
    }

    eMBErrorCode status = MB_EIO;
    // Initialize Modbus stack using mbcontroller parameters
    status = eMBInit((eMBRole)mb_role, (eMBMode)mb_type, (UCHAR)mb_address, (UCHAR)mb_port,
                            (ULONG)mb_speed, (eMBParity)mb_parity);
    MB_CHECK((status == MB_ENOERR), ESP_ERR_INVALID_STATE,
            "mb stack initialization failure, eMBInit() returns (0x%x).", status);
#ifdef CONFIG_MB_CONTROLLER_SLAVE_ID_SUPPORT
    if (mb_role == MB_ROLE_SLAVE)
    {
        status = eMBSetSlaveID(MB_SLAVE_ID_SHORT, TRUE, (UCHAR*)mb_slave_id, sizeof(mb_slave_id));
        MB_CHECK((status == MB_ENOERR), ESP_ERR_INVALID_STATE, "mb stack set slave ID failure.");
    }
#endif
    status = eMBEnable();
    MB_CHECK((status == MB_ENOERR), ESP_ERR_INVALID_STATE,
            "mb stack enable failure, eMBEnable() returned (0x%x).", (uint32_t)status);
    // Set the mbcontroller start flag
    EventBits_t flag = xEventGroupSetBits(mb_controller_event_group,
                                            (EventBits_t)MB_EVENT_STACK_STARTED);
    MB_CHECK((flag & MB_EVENT_STACK_STARTED),
                ESP_ERR_INVALID_STATE, "mb stack start event set error.");

    // send init event to modbus, to instruct it to wait for Ready timeout 
    ps_internal_frame_descriptor = (sMBMasterQueueType *)palloc();
    if (ps_internal_frame_descriptor == NULL)
    {
        return ESP_FAIL;
    }
    (void)xMBMasterPortEventPost(EV_MASTER_INIT, &ps_internal_frame_descriptor);

    return ESP_OK;
}

// Modbus controller destroy function
esp_err_t mbcontroller_destroy(void)
{
    if (mb_init_done == FALSE)
    {
        return MB_EILLSTATE;
    }

    eMBErrorCode mb_error = MB_ENOERR;

    // Stop supervisor task (so it won't post any events to modbus controller anymore).
    (void)vTaskDelete(mb_supervisor_task_handle);

    // Stop polling by clearing correspondent bit in the event group
    EventBits_t flag = xEventGroupClearBits(mb_controller_event_group,
                                    (EventBits_t)MB_EVENT_STACK_STARTED);
    MB_CHECK((flag & MB_EVENT_STACK_STARTED),
                ESP_ERR_INVALID_STATE, "mb stack stop event failure.");
    // Desable and then destroy the Modbus stack
    mb_error = eMBDisable();
    MB_CHECK((mb_error == MB_ENOERR), ESP_ERR_INVALID_STATE, "mb stack disable failure.");
    (void)vTaskDelete(mb_controller_task_handle);
    (void)vQueueDelete(mb_controller_notification_queue_handle);
    (void)vEventGroupDelete(mb_controller_event_group);

    // Since Modbus controller is stopped it won't post anything to modbus supervisor queue anymore, so it's now safe to delete it.
    (void) vQueueDelete(xMBSupervisorQueue);

    mb_error = eMBClose();
    MB_CHECK((mb_error == MB_ENOERR), ESP_ERR_INVALID_STATE,
            "mb stack close failure returned (0x%x).", (uint32_t)mb_error);

    // and finally clean up the deterministic pool
    pool_cleanup();

    return ESP_OK;
}

// Setup modbus controller parameters
esp_err_t mbcontroller_setup(const mb_communication_info_t comm_info)
{
    if (mb_init_done == FALSE)
    {
        return MB_EILLSTATE;
    }

    MB_CHECK(((comm_info.mode == MB_MODE_RTU) || (comm_info.mode == MB_MODE_ASCII)),
                ESP_ERR_INVALID_ARG, "mb incorrect mode = (0x%x).",
                (uint32_t)comm_info.mode);
    MB_CHECK((comm_info.slave_addr <= MB_ADDRESS_MAX),
                ESP_ERR_INVALID_ARG, "mb wrong slave address = (0x%x).",
                (uint32_t)comm_info.slave_addr);
    MB_CHECK((comm_info.port <= UART_NUM_2), ESP_ERR_INVALID_ARG,
                "mb wrong port to set = (0x%x).", (uint32_t)comm_info.port);
    MB_CHECK((comm_info.parity <= UART_PARITY_EVEN), ESP_ERR_INVALID_ARG,
                "mb wrong parity option = (0x%x).", (uint32_t)comm_info.parity);
    mb_type = (uint8_t)comm_info.mode;
    mb_address = (uint8_t)comm_info.slave_addr;
    mb_port = (uint8_t)comm_info.port;
    mb_speed = (uint32_t)comm_info.baudrate;
    mb_parity = (uint8_t)comm_info.parity;
    return ESP_OK;
}

/* ----------------------- Callback functions for Modbus stack (SLAVE role only) ---------------*/
// These are executed by modbus stack to read appropriate type of registers.

// This is required to suppress warning when register start address is zero
#pragma GCC diagnostic ignored "-Wtype-limits"

// Callback function for reading of MB Input Registers (SLAVE role)
eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress,
                                USHORT usNRegs)
{
    assert(pucRegBuffer != NULL);
    USHORT usRegInputNregs = (USHORT)(mb_area_descriptors[MB_PARAM_INPUT].size >> 1); // Number of input registers
    USHORT usInputRegStart = (USHORT)mb_area_descriptors[MB_PARAM_INPUT].start_offset; // Get Modbus start address
    UCHAR* pucInputBuffer = (UCHAR*)mb_area_descriptors[MB_PARAM_INPUT].address; // Get instance address
    USHORT usRegs = usNRegs;
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex;
    // If input or configuration parameters are incorrect then return an error to stack layer
    if ((usAddress >= usInputRegStart)
            && (pucInputBuffer != NULL)
            && (usNRegs >= 1)
            && ((usAddress + usRegs) <= (usInputRegStart + usRegInputNregs + 1))
            && (usRegInputNregs >= 1)) {
        iRegIndex = (USHORT)(usAddress - usInputRegStart - 1);
        iRegIndex <<= 1; // register Address to byte address
        pucInputBuffer += iRegIndex;
        UCHAR* pucBufferStart = pucInputBuffer;
        while (usRegs > 0) {
            _XFER_2_RD(pucRegBuffer, pucInputBuffer);
            iRegIndex += 2;
            usRegs -= 1;
        }
        // Send access notification
        (void)send_param_access_notification(MB_EVENT_INPUT_REG_RD);
        // Send parameter info to application task
        (void)send_param_info(MB_EVENT_INPUT_REG_RD, (uint16_t)usAddress,
                        (uint8_t*)pucBufferStart, (uint16_t)usNRegs);
    } else {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

// Callback function for reading of MB Holding Registers (SLAVE role)
// Executed by stack when request to read/write holding registers is received
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress,
        USHORT usNRegs, eMBRegisterMode eMode)
{
    assert(pucRegBuffer != NULL);
    USHORT usRegHoldingNregs = (USHORT)(mb_area_descriptors[MB_PARAM_HOLDING].size >> 1);
    USHORT usRegHoldingStart = (USHORT)mb_area_descriptors[MB_PARAM_HOLDING].start_offset;
    UCHAR* pucHoldingBuffer = (UCHAR*)mb_area_descriptors[MB_PARAM_HOLDING].address;
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex;
    USHORT usRegs = usNRegs;
    // Check input and configuration parameters for correctness
    if ((usAddress >= usRegHoldingStart)
            && (pucHoldingBuffer != NULL)
            && ((usAddress + usRegs) <= (usRegHoldingStart + usRegHoldingNregs + 1))
            && (usRegHoldingNregs >= 1)
            && (usNRegs >= 1)) {
        iRegIndex = (USHORT) (usAddress - usRegHoldingStart - 1);
        iRegIndex <<= 1; // register Address to byte address
        pucHoldingBuffer += iRegIndex;
        UCHAR* pucBufferStart = pucHoldingBuffer;
        switch (eMode) {
            case MB_REG_READ:
                while (usRegs > 0) {
                    _XFER_2_RD(pucRegBuffer, pucHoldingBuffer);
                    iRegIndex += 2;
                    usRegs -= 1;
                };
                // Send access notification
                (void)send_param_access_notification(MB_EVENT_HOLDING_REG_RD);
                // Send parameter info
                (void)send_param_info(MB_EVENT_HOLDING_REG_RD, (uint16_t)usAddress,
                                (uint8_t*)pucBufferStart, (uint16_t)usNRegs);
                break;
            case MB_REG_WRITE:
                while (usRegs > 0) {
                    _XFER_2_WR(pucHoldingBuffer, pucRegBuffer);
                    pucHoldingBuffer += 2;
                    iRegIndex += 2;
                    usRegs -= 1;
                };
                // Send access notification
                (void)send_param_access_notification(MB_EVENT_HOLDING_REG_WR);
                // Send parameter info
                (void)send_param_info(MB_EVENT_HOLDING_REG_WR, (uint16_t)usAddress,
                                (uint8_t*)pucBufferStart, (uint16_t)usNRegs);
                break;
        }
    } else {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

// Callback function for reading of MB Coils Registers (SLAVE role)
eMBErrorCode eMBRegCoilsCB(UCHAR* pucRegBuffer, USHORT usAddress,
        USHORT usNCoils, eMBRegisterMode eMode)
{
    assert(NULL != pucRegBuffer);
    USHORT usRegCoilNregs = (USHORT)(mb_area_descriptors[MB_PARAM_COIL].size >> 1); // number of registers in storage area
    USHORT usRegCoilsStart = (USHORT)mb_area_descriptors[MB_PARAM_COIL].start_offset; // MB offset of coils registers
    UCHAR* pucRegCoilsBuf = (UCHAR*)mb_area_descriptors[MB_PARAM_COIL].address;
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex;
    USHORT usCoils = usNCoils;
    usAddress--; // The address is already +1
    if ((usAddress >= usRegCoilsStart)
            && (usRegCoilNregs >= 1)
            && ((usAddress + usCoils) <= (usRegCoilsStart + (usRegCoilNregs << 4) + 1))
            && (pucRegCoilsBuf != NULL)
            && (usNCoils >= 1)) {
        iRegIndex = (USHORT) (usAddress - usRegCoilsStart);
        CHAR* pucCoilsDataBuf = (CHAR*)(pucRegCoilsBuf + (iRegIndex >> 3));
        switch (eMode) {
            case MB_REG_READ:
                while (usCoils > 0) {
                    UCHAR ucResult = xMBUtilGetBits((UCHAR*)pucRegCoilsBuf, iRegIndex, 1);
                    xMBUtilSetBits(pucRegBuffer, iRegIndex - (usAddress - usRegCoilsStart), 1, ucResult);
                    iRegIndex++;
                    usCoils--;
                }
                // Send an event to notify application task about event
                (void)send_param_access_notification(MB_EVENT_COILS_WR);
                (void)send_param_info(MB_EVENT_COILS_WR, (uint16_t)usAddress,
                                (uint8_t*)(pucCoilsDataBuf), (uint16_t)usNCoils);
                break;
            case MB_REG_WRITE:
                while (usCoils > 0) {
                    UCHAR ucResult = xMBUtilGetBits(pucRegBuffer,
                            iRegIndex - (usAddress - usRegCoilsStart), 1);
                    xMBUtilSetBits((uint8_t*)pucRegCoilsBuf, iRegIndex, 1, ucResult);
                    iRegIndex++;
                    usCoils--;
                }
                // Send an event to notify application task about event
                (void)send_param_access_notification(MB_EVENT_COILS_WR);
                (void)send_param_info(MB_EVENT_COILS_WR, (uint16_t)usAddress,
                                (uint8_t*)pucCoilsDataBuf, (uint16_t)usNCoils);
                break;
        } // switch ( eMode )
    } else {
        // If the configuration or input parameters are incorrect then return error to stack
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

// Callback function for reading of MB Discrete Input Registers (SLAVE role)
eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress,
                            USHORT usNDiscrete)
{
    assert(pucRegBuffer != NULL);
    USHORT usRegDiscreteNregs = (USHORT)(mb_area_descriptors[MB_PARAM_DISCRETE].size >> 1); // number of registers in storage area
    USHORT usRegDiscreteStart = (USHORT)mb_area_descriptors[MB_PARAM_DISCRETE].start_offset; // MB offset of registers
    UCHAR* pucRegDiscreteBuf = (UCHAR*)mb_area_descriptors[MB_PARAM_DISCRETE].address; // the storage address
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex, iRegBitIndex, iNReg;
    UCHAR* pucDiscreteInputBuf;
    iNReg = usNDiscrete / 8 + 1;
    pucDiscreteInputBuf = (UCHAR*) pucRegDiscreteBuf;
    // It already plus one in modbus function method.
    usAddress--;
    if ((usAddress >= usRegDiscreteStart)
            && (usRegDiscreteNregs >= 1)
            && (pucRegDiscreteBuf != NULL)
            && ((usAddress + usNDiscrete) <= (usRegDiscreteStart + (usRegDiscreteNregs * 16)))
            && (usNDiscrete >= 1)) {
        iRegIndex = (USHORT) (usAddress - usRegDiscreteStart) / 8; // Get register index in the buffer for bit number
        iRegBitIndex = (USHORT)(usAddress - usRegDiscreteStart) % 8; // Get bit index
        UCHAR* pucTempBuf = &pucDiscreteInputBuf[iRegIndex];
        while (iNReg > 0) {
            *pucRegBuffer++ = xMBUtilGetBits(&pucDiscreteInputBuf[iRegIndex++], iRegBitIndex, 8);
            iNReg--;
        }
        pucRegBuffer--;
        // Last discrete
        usNDiscrete = usNDiscrete % 8;
        // Filling zero to high bit
        *pucRegBuffer = *pucRegBuffer << (8 - usNDiscrete);
        *pucRegBuffer = *pucRegBuffer >> (8 - usNDiscrete);
        // Send an event to notify application task about event
        (void)send_param_access_notification(MB_EVENT_DISCRETE_RD);
        (void)send_param_info(MB_EVENT_DISCRETE_RD, (uint16_t)usAddress,
                            (uint8_t*)pucTempBuf, (uint16_t)usNDiscrete);
    } else {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/* ----------------------- Callback functions for Modbus stack (MASTER role) --------------------*/
/**
 * Modbus Master generic callback function from Modbus function handlers. Forwards as asynchronous
 * message to Modbus supervisor.
 *
 * @param psInternalFrameDescriptor Descriptor used for the request.
 *
 * @return result
 */
eMBErrorCode eMBMasterGenericCB(sMBMasterQueueType *psInternalFrameDescriptor)
{
    mb_supervisor_request_t  xRequest;

    xRequest.sender = NULL;
    xRequest.req_resp.req_type = MB_SUPERV_RESPONSE;
    xRequest.req_entry.descriptor = (struct mb_request_entry *)psInternalFrameDescriptor;
    BaseType_t status = xQueueSend(xMBSupervisorQueue, &xRequest, MB_PAR_INFO_TOUT);
    return (status == pdTRUE) ? MB_ENOERR : MB_ENORES;
}

#pragma GCC diagnostic pop   // require GCC


