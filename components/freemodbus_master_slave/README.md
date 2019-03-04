# freemodbus_master_slave
lib freemodbus porting for ESP32 including dual-role (master/slave) support

## - Brief description:

FreeModbus is an open source Modbus protocol stack, but supports only slave role. I found some master mode ports on the internet, mainly armink's
port done for the STM32, and erfengwe's port to the ESP32 (Thanks!). Both seem to use lots of static memory and can handle only one
simultaneous transfer, so I added a Modbus Supervisor which is message-passing based. This also eases using FreeModbus on multi-core
controllers and distributed systems. Also the internal design has changed a bit to handle more than one message sent to it (serialized 
at the modbus task level).
Modbus master is handled now only by sending and receiving messages to/from the new Modbus Supervisor task.
Please note I used "role" to distinguish master and slave, since "mode" is already used for the communication mode in Modbus and should not 
be confused.
Main design goal was to interface with a message-passing based IoT platform, but the dual-role stack can be useful for many other applications.

## - Usage in Slave Role:
- Refer to Freemodbus documentation. The only difference is that mbcontroller_init takes the role parameter (set to MB_ROLE_SLAVE) and 
  returns a queue handle to the new Modbus Supervisor process (not used in slave role at the moment). 

## - Usage in Master Role:
- First initialize the modbus controller and supervisor by calling mbcontroller_init() and keeping the supervisor queue handle.
- Then set up communication parameters (similar to slave role) using mbcontroller_setup().
- Afterwards start the processes of modbus controller and supervisor by calling mbcontroller_start().
- Switch modbus supervisor operating mode (keep or reply) if you like to receive modbus operation results when they are completed, or
  if they should kept within modbus supervisor until you ask for it using the peek, receive or kill requests.
  See also the example code below for information.

- Normal operational usage is done via sending messages to the modbus supervisor task and receiving replies (or polling it for progress).
   - Initialize to reply (autorespond) mode (see example below) by sending a MB_SUPERV_CONFIG_AUTORESOND message.
   - Instead, if you like to poll for completed requests from the application, set modbus supervisor to keep mode by sending a MB_SUPERV_CONFIG_KEEP message
     (this is also the default behavior).
   - If required, change throttling by sending a MB_SUPERV_CONFIG_THROTTLE message.
   - To send a modbus command, allocate a descriptor (either - on distributed systems - by requesting one from the supervisor 
     by sending it a MB_SUPERV_ALLOC_DESC message - or simply by calling mbsupervisor_buf_alloc();
   - Fill in the following information:
     a) Message (see also modbus_controller/mbcontroller.h for mb_supervisor_request_t): 
        sender                - Queue handle of the sender (since modbus supervisor and the descriptor 
        req_resp.req_type     - Request type  (note that this is an union and in replies at this place you'll receive the status back)
        req_entry.descriptor  - Pointer to descriptor (note that this is an union and in replies at this place you'll receive the request ID back)
     b) Descriptor (see also modbus/include/mbtypes.h for struct mb_request_entry):
        request_id            - Unique request ID handled by application - if the application doesn't provide, fill with zero and modbus supervisor assigns an ID.
        (req_origin will be internally maintained if needed, no need to supply)
        (result will be provided by modbus supervisor in replies)
        is_raw_request        - Set to TRUE if you like to send raw modbus frames, or FALSE if using formatted list command (recommended).
        In raw frame mode only:
        u.raw_frame           - Fill with the raw frame.
        raw_frame_length      - Give the frame length here.
        In formatted list mode only use instead:
        u.f.slave_address     - slave to send modbus command to
        u.f.function_code     - modbus function code for the particular command (this depends on the slave's capability)
        u.f.read_start        - for all read commands the first address (regardless if register, discrete input, coil or input!)
        u.f.read_len          - for all read commands the amount of registers to read (regardless if register, discrete input, coil or input!)
        u.f.write_start       - for all write commands the first address (regardless if register(s) or coil(s)!)
        u.f.write_len         - for all read commands the amount of registers to write (regardless if register(s) or coil(s)!)
        unpacked_data         - for write commands only, fill with data to write. This also applies to coils, fill with any non-zero value for ON or zero for OFF.
   - Send MB_SUPERV_REQ message to modbus supervisor.
   - In all modes, modbus supervisor will immediately reply with a status message and the request ID if the command has been accepted.
     The descriptor is kept inside in the pending queue and cannot be used by the caller anymore (at least not at this time).
   - In reply (autorespond) mode, the modbus supervisor will send another message when the command has finished (regardless if successfully or with error/timeout).
     This message will then contain the result, and on any modbus read command the data is returned in the "unpacked_data" field.
     The descriptor is automatically purged from the internal pending queue and the caller can decide to reuse it for the next request, or free it using 
     mbsupervisor_buf_free();
     Please note that these buffer alloc/free functions are using a pool of fixed size, so these are very fast, deterministic and don't contribute to memory
     fragmentation. For fully deterministic behavior, just allocate a couple of descriptors (as you observed as maximum usage) in the beginning 
     (after modbus supervisor init) and free them afterwards - as the pool itself uses malloc().
   - In keep (polling) mode, instead you won't get a notification if a command has completed or timed out. Instead, you need to poll modbus supervisor
     using MB_SUPERV_PEEK with either the particular request ID (returns status for that request only), or with request ID set to zero - will return status of
     the first finished request. See modbus_controller/mbcontroller.c for details.
   - To receive the descriptor of a finished request, send MB_SUPERV_RECEIVE message and the modbus supervisor returns the corresponding descriptor with the result
     and returned data (if any). It is also purged from the pending queue, so the caller owns the descriptor (reuse or free it).
     There is no need to call the modbus supervisor again to remove the descriptor for this request ID from the pending queue - it will not cause harm to do
     (modbus supervisor will return MB_MRE_NOTFOUND in this case) but it is just a waste of time.
     The returned data is formatted according to the is_raw_request setting:
     * In raw frame mode, the received modbus frame (without CRC) is returned in u.raw_frame.
     * In formatted list mode, data received is returned in the unpacked_data array. Please note that write-only commands don't return any data and the array
       contains invalid data. The only modbus command which both reads and writes data is function code 0x17, however it depends whether the slave on the network
       is supporting it.
   - To remove any descriptor from the pending queue before the command has finished, send a MB_SUPERV_KILL message. This not only removes the descriptor from
     the pending queue, but also frees it to the pool. Therefore no descriptor is returned from this command.

   - Should it be necessary to tear down the modbus controller and supervisor, call mbcontroller_destroy(). This cleans up all queues and destroys the pool
     as well (also frees all descriptors). Any messages in the queue received back from modbus supervisor after destroying it, shall be considered invalid and discarded,
     any descriptor must not be used anymore.
       

## - Known limitiations:
- Currently Freemodbus_master_slave handles one channel / one UART only, configured by Kconfig parameters. This will change in the future,
  also for slave role, to support multiple stacks running simultaneously (also in different roles), especially slave role will get
  a "sniffer" mode not responding on the modbus but forwarding everything to the application.
- Interfacing to slave and master roles are different (slave role: original freemodbus behavior).
- Stack size configuration defaults can be optimized (likely too much).
- Master role does not support retransmits at the moment.
- Message passing interface only defined for Master role.


## - Thanks to:
- armink for the STM32 port   - see https://github.com/armink/FreeModbus_Slave-Master-RTT-STM32
- erfengwe for the ESP32 port - see https://github.com/erfengwelink/modbus_port_esp32

## - Bugs
Please report via github or to info@connictro.de .

## - Example initialization:

esp_err_t app_modbus_init()
{
    esp_err_t                eStatus = ESP_FAIL;
    mb_communication_info_t  comm_info;   // Modbus communication parameters
    mb_supervisor_request_t  svr;         // for requests&replies to/from modbus supervisor

    ESP_LOGD(TAG_MB, "Initializing modbus controller");
    // Initialize modbus in master mode, ready for reading/writing from/to modbus slaves.
    ESP_ERROR_CHECK(mbcontroller_init(MB_ROLE_MASTER, &xMBModbusSupervisorQueue));
    ESP_LOGD(TAG_MB, "Modbus controller created, checking supervisor queue handle");
    if (xMBModbusSupervisorQueue == NULL) return eStatus;

    ESP_LOGD(TAG_MB, "Supervisor queue handle OK, setting up modbus controller comm parameters");
    comm_info.mode       = MB_MODE_RTU;
    comm_info.port       = MB_UART_PORT;
    comm_info.baudrate   = MB_DEVICE_SPEED;
    comm_info.parity     = MB_PARITY_NONE;
    ESP_ERROR_CHECK(mbcontroller_setup(comm_info));

    ESP_LOGD(TAG_MB, "Starting modbus controller");
    ESP_ERROR_CHECK(mbcontroller_start());

    xMBAppModbusQueue = xQueueCreate(MB_SUPERVISOR_NOTIFY_QUEUE_SIZE, sizeof(mb_supervisor_request_t));
    MB_CHECK((xMBAppModbusQueue != NULL), ESP_ERR_NO_MEM, "mb application queue creation error.");
 
    ESP_LOGD(TAG_MB, "Switching modbus supervisor to reply mode");
    svr.sender            = xMBAppModbusQueue;
    svr.req_resp.req_type = MB_SUPERV_CONFIG_AUTORESPOND;
    (void) xQueueSend(xMBModbusSupervisorQueue, &svr, MB_PAR_INFO_TOUT);
    ESP_LOGD(TAG_MB, "Switch request sent, waiting for answer");
    if (xQueueReceive(xMBAppModbusQueue, (void *)&svr, portMAX_DELAY) == pdTRUE)
    {
        if (svr.req_resp.err_code == MB_MRE_NO_ERR) 
        {
            eStatus = ESP_OK;
        }
    }
    ESP_LOGI(TAG_MB, "Modbus init finished with status: %8.8X", eStatus);
    // now we're ready to use the modbus master by sending requests and receiving replies from it.
    
    return eStatus;
}

