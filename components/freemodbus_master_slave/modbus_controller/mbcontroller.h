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

//  mbcontroller.h
//  Implementation of the MbController

#ifndef _MODBUS_CONTROLLER
#define _MODBUS_CONTROLLER

#include <stdint.h>                 // for standard int types definition
#include <stddef.h>                 // for NULL and std defines
#include "soc/soc.h"                // for BITN definitions
#include "sdkconfig.h"              // for KConfig options
#include "driver/uart.h"            // for uart port number defines
#include "freertos/queue.h"         // for queue api access
#include "mbtypes.h"                // for modbus definitions and strctures.

/* ----------------------- Generic Defines ------------------------------------------*/
#define MB_INST_MIN_SIZE                    (2) // The minimal size of Modbus registers area in bytes
#define MB_INST_MAX_SIZE                    (65535 * 2) // The maximum size of Modbus area in bytes

#define MB_CONTROLLER_STACK_SIZE            (CONFIG_MB_CONTROLLER_STACK_SIZE) // Stack size for Modbus controller
#define MB_CONTROLLER_PRIORITY              (CONFIG_MB_SERIAL_TASK_PRIO - 1) // priority of MB controller (and supervisor) tasks
#define MB_CONTROLLER_NOTIFY_QUEUE_SIZE     (CONFIG_MB_CONTROLLER_NOTIFY_QUEUE_SIZE) // Number of messages in parameter notification queue
#define MB_CONTROLLER_NOTIFY_TIMEOUT        (pdMS_TO_TICKS(CONFIG_MB_CONTROLLER_NOTIFY_TIMEOUT)) // notification timeout
#define MB_SUPERVISOR_STACK_SIZE            (CONFIG_MB_SUPERVISOR_STACK_SIZE) // Stack size for Modbus controller
#define MB_SUPERVISOR_NOTIFY_QUEUE_SIZE     (CONFIG_MB_SUPERVISOR_NOTIFY_QUEUE_SIZE) // Number of messages in parameter notification queue

// Default port defines
#define MB_DEVICE_ADDRESS   (1)             // Default slave device address in Modbus (relevant in slave role only)
#define MB_DEVICE_SPEED     (9600)          // Default Modbus speed for now hard defined
#define MB_UART_PORT        (UART_NUM_2)    // Default UART port number
#define MB_PAR_INFO_TOUT    (10)            // Timeout for get parameter info
#define MB_PARITY_NONE      (UART_PARITY_DISABLE)

/* -----------------------Master Defines -------------------------------------*/

#define T_WAIT_FOREVER 3

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Event group for parameters notification
 */
typedef enum
{
    MB_EVENT_NO_EVENTS = 0x00,
    MB_EVENT_HOLDING_REG_WR = BIT0,         /*!< Modbus Event Write Holding registers. */
    MB_EVENT_HOLDING_REG_RD = BIT1,         /*!< Modbus Event Read Holding registers. */
    MB_EVENT_INPUT_REG_RD = BIT3,           /*!< Modbus Event Read Input registers. */
    MB_EVENT_COILS_WR = BIT4,               /*!< Modbus Event Write Coils. */
    MB_EVENT_COILS_RD = BIT5,               /*!< Modbus Event Read Coils. */
    MB_EVENT_DISCRETE_RD = BIT6,            /*!< Modbus Event Read Discrete bits. */
    MB_EVENT_STACK_STARTED = BIT7           /*!< Modbus Event Stack started */
} mb_event_group_t;

/**
 * @brief Type of Modbus parameter
 */
typedef enum
{
    MB_PARAM_HOLDING,                /*!< Modbus Holding register. */
    MB_PARAM_INPUT,                  /*!< Modbus Input register. */
    MB_PARAM_COIL,                   /*!< Modbus Coils. */
    MB_PARAM_DISCRETE,               /*!< Modbus Discrete bits. */
    MB_PARAM_COUNT,
    MB_PARAM_UNKNOWN = 0xFF
} mb_param_type_t;

/*!
 * \brief Modbus serial transmission modes (RTU/ASCII).
 */
typedef enum
{
    MB_MODE_RTU,                     /*!< RTU transmission mode. */
    MB_MODE_ASCII,                   /*!< ASCII transmission mode. */
    MB_MODE_TCP                      /*!< TCP mode. */
} mb_mode_type_t;

/**
 * @brief Parameter access event information type
 */
typedef struct {
    uint32_t time_stamp;                    /*!< Timestamp of Modbus Event (uS)*/
    uint16_t mb_offset;                     /*!< Modbus register offset */
    mb_event_group_t type;                  /*!< Modbus event type */
    uint8_t* address;                       /*!< Modbus data storage address */
    size_t size;                            /*!< Modbus event register size (number of registers)*/
} mb_param_info_t;

/**
 * @brief Parameter storage area descriptor (Slave role)
 */
typedef struct {
    uint16_t start_offset;                  /*!< Modbus start address for area descriptor */
    mb_param_type_t type;                   /*!< Type of storage area descriptor */
    void* address;                          /*!< Instance address for storage area descriptor */
    size_t size;                            /*!< Instance size for area descriptor (bytes) */
} mb_register_area_descriptor_t;

/**
 * @brief Request type for communication with MB Supervisor task (Master role)
 */
typedef enum {
    MB_SUPERV_CONFIG_KEEP,                  /*!< Set up supervisor to keep completed requests [DEFAULT] (don't respond but waits for caller actively peeking or receiving them). */
    MB_SUPERV_CONFIG_AUTORESPOND,           /*!< Set up supervisor to automatically respond with completed requests (don't keep them). */
    MB_SUPERV_CONFIG_THROTTLE,              /*!< Set up supervisor request throttling. */
    MB_SUPERV_ALLOC_DESC,                   /*!< Allocate a descriptor from the pool. */
    MB_SUPERV_RELEASE_DESC,                 /*!< Return a descriptor to the pool. */
    MB_SUPERV_REQ,                          /*!< Modbus request from application */
    MB_SUPERV_PEEK,                         /*!< Peek for status of pending request including status - completed requests will still remain in pending queue */
    MB_SUPERV_RECEIVE,                      /*!< Receive pending request and removes it from pending queue */
    MB_SUPERV_KILL,                         /*!< Search for a particular pending request and remove it from the pending queue without getting the information. */
    MB_SUPERV_RESPONSE                      /*!< Response from lower layer received */
} mb_supervisor_request_type_t;


/**
 * @brief Event structure used for communication with MB Supervisor task (Master role)
 */
typedef struct {
    QueueHandle_t                    sender;     /*!< Originator of request, the sender should fill in the queue it is listening to. */
    union {
        mb_supervisor_request_type_t req_type;   /*!< Request type */
        eMBMasterReqErrCode          err_code;   /*!< Request response status */ 
    } req_resp;
    union {
        uint32_t                     req_id;     /*!< Request ID, for use in MB_SUPERV_PEEK and MB_SUPERV_RECEIVE requests to modbus supervisor. */ 
        struct mb_request_entry     *descriptor; /*!< Request/Response descriptor. */
    } req_entry;

} mb_supervisor_request_t;


/**
 * @brief Device communication parameters
 */
typedef struct {
    mb_mode_type_t mode;                    /*!< Modbus communication mode */
    uint8_t slave_addr;                     /*!< Modbus Slave Address */
    uart_port_t port;                       /*!< Modbus communication port (UART) number */
    uint32_t baudrate;                      /*!< Modbus baudrate */
    uart_parity_t parity;                   /*!< Modbus UART parity settings */
} mb_communication_info_t;

/* --------------------- API SLAVE role only ---------------------------------------*/
/**
 * @brief Set Modbus area descriptor (Slave role)
 *
 * @param descr_data Modbus registers area descriptor structure
 *
 * @return
 *     - ESP_OK: The appropriate descriptor is set
 *     - ESP_ERR_INVALID_ARG: The argument is incorrect
 */
esp_err_t mbcontroller_set_descriptor(mb_register_area_descriptor_t descr_data);

/* --------------------- API MASTER role only --------------------------------------*/
/**
 * @brief Allocate descriptor for modbus supervisor (synchronously).
 *
 * @param[out] mb_new_request Returns pointer to new Modbus supervisor descriptor.
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL No resources available.
 */
esp_err_t mbsupervisor_buf_alloc(struct mb_request_entry **mb_new_request);

/**
 * @brief Release descriptor for modbus supervisor (synchronously).
 * @param mb_new_request Descriptor to release.
 *
 */
void mbsupervisor_buf_free(struct mb_request_entry *mb_new_request);

/* --------------------- API all roles ---------------------------------------------*/

/**
 * @brief Initialize modbus controller and stack
 * @param role     Stack role (Master/Slave).
 * @param sv_queue returns queue handler for modbus supervisor.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t mbcontroller_init(eMBRole role, QueueHandle_t *sv_queue);

/**
 * @brief Destroy Modbus controller and stack
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t mbcontroller_destroy(void);

/**
 * @brief Start Modbus communication stack
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_ERR_INVALID_ARG Modbus stack start error
 */
esp_err_t mbcontroller_start(void);

/**
 * @brief Set Modbus communication parameters for the controller
 *
 * @param comm_info Communication parameters structure.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Incorrect parameter data
 */
esp_err_t mbcontroller_setup(mb_communication_info_t comm_info);

#ifdef __cplusplus
}
#endif

#endif /* _MODBUS_CONTROLLER */

