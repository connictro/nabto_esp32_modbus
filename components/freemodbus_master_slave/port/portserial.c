/*
 * FreeModbus Libary: ESP32 Port Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: portother.c,v 1.1 2010/06/06 13:07:20 wolti Exp $
 */
#include "port.h"
#include "driver/uart.h"
#include "freertos/queue.h" // for queue support
#include "soc/uart_reg.h"
#include "driver/gpio.h"
#include "esp_log.h"        // for esp_log
#include "esp_err.h"        // for ESP_ERROR_CHECK macro
   
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "sdkconfig.h"              // for KConfig options

// Definitions of UART default pin numbers
#define MB_UART_RXD   (CONFIG_MB_UART_RXD)
#define MB_UART_TXD   (CONFIG_MB_UART_TXD)
#define MB_UART_RTS   (CONFIG_MB_UART_RTS)
#define MB_UART_EN_PIN MB_UART_RTS

#define RS_RX_MODE                  0
#define RS_TX_MODE                  1
#define MB_BAUD_RATE_DEFAULT        (115200)
#define MB_QUEUE_LENGTH             (CONFIG_MB_QUEUE_LENGTH)

#define MB_SERIAL_TASK_PRIO         (CONFIG_MB_SERIAL_TASK_PRIO)
#define MB_SERIAL_TASK_STACK_SIZE   (CONFIG_MB_SERIAL_TASK_STACK_SIZE)
#define MB_SERIAL_TOUT              (3) // 3.5*8 = 28 ticks, TOUT=3 -> ~24..33 ticks

// Set buffer size for transmission
#define MB_SERIAL_BUF_SIZE          (CONFIG_MB_SERIAL_BUF_SIZE)

// Note: This code uses mixed coding standard from legacy IDF code and used freemodbus stack

// TODO these statics need to be instance-local, to enable handling multiple buses simultaneously by separate instances
// A queue to handle UART event.
static QueueHandle_t xMbUartQueue;
static TaskHandle_t  xMbTaskHandle;
static const CHAR *TAG = "MB_SERIAL";

// The UART hardware port number
static UCHAR ucUartNumber = UART_NUM_2;

static BOOL bRxStateEnabled = FALSE; // Receiver enabled flag
static BOOL bTxStateEnabled = FALSE; // Transmitter enabled flag

static UCHAR ucBuffer[MB_SERIAL_BUF_SIZE]; // Temporary buffer to transfer received data to modbus stack
static USHORT uiRxBufferPos = 0;    // position in the receiver buffer

static BOOL rs485_transceiver_uses_enable = FALSE; /* set by init function, set to true only
                                                      if external transceiver needs enable signal. */
static uint8_t RS485Mode = RS_RX_MODE;

/* ----------------------- static functions ---------------------------------*/
static int rs485_en_pin_cfg()
{
    rs485_transceiver_uses_enable = TRUE;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_SEL_12;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    //init rs485 default in rx mode
    gpio_set_level(MB_UART_EN_PIN, RS485Mode);

    return 0;
}



/* ----------------------- Start implementation -----------------------------*/
int rs485_wait_tx_done()
{
    return uart_wait_tx_done(ucUartNumber, 10);
}

int rs485_trans_toggle(int mode)
{
    if (rs485_transceiver_uses_enable == TRUE)
    {
        if (RS_TX_MODE != RS485Mode)
        {
            if (RS_TX_MODE == mode)
            {
                RS485Mode = mode;
                gpio_set_level(MB_UART_EN_PIN, RS485Mode);
                //ESP_LOGI(TAG, "RS_TX_MODE\r\n");
            }
        }
        else if (RS_RX_MODE != RS485Mode)
        {
            if (RS_RX_MODE == mode)
            {
                RS485Mode = mode;
                gpio_set_level(MB_UART_EN_PIN, RS485Mode);
                //ESP_LOGI(TAG, "RS_RX_MODE\r\n");
            }
        }
        else
        {
            ESP_LOGI(TAG, "input param invalid / already in mode\r\n");
            //ESP_LOGI(TAG, "input param invalid / already in mode ignore this time operate!");
        }
    }
    return 0;
}


// This function can be called from xMBRTUTransmitFSM() of different task
void vMBPortSerialEnable(BOOL bRxEnable, BOOL bTxEnable)
{
    ENTER_CRITICAL_SECTION();

    USHORT usCount = 0;
    BOOL bNeedPoll = FALSE;

    // UCHAR len = get_s_usLength() + 1;   // this was in BARE mater port only; get_s_usLength() not exists anymore

    if (bRxEnable) {
        //uart_enable_rx_intr(ucUartNumber);
        bRxStateEnabled = TRUE;
        vTaskResume(xMbTaskHandle); // Resume receiver task
    } else {
        vTaskSuspend(xMbTaskHandle); // Block receiver task
        bRxStateEnabled = FALSE;
    }
    if (bTxEnable) {
        bTxStateEnabled = TRUE;
        if (!bRxEnable)
        {
            // Continue while all transmit bytes put in buffer or out of buffer
            while((bNeedPoll == FALSE) && (usCount++ < MB_SERIAL_BUF_SIZE)) {
               // Calls the modbus stack callback function to let it fill the UART transmit buffer.
                bNeedPoll = pxMBFrameCBTransmitterEmpty( );  // calls callback xMBRTUTransmitFSM();
            }
            ESP_LOGD(TAG, "MB_TX_buffer sent: (%d) bytes\n", (uint16_t)usCount);
        }
    } else {
        bTxStateEnabled = FALSE;
    }
    EXIT_CRITICAL_SECTION();
}

static void vMBPortSerialRxPoll(size_t xEventSize)
{
    USHORT usLength;

    if (bRxStateEnabled) {
        if (xEventSize > 0) {
            xEventSize = (xEventSize > MB_SERIAL_BUF_SIZE) ?  MB_SERIAL_BUF_SIZE : xEventSize;
            uiRxBufferPos = ((uiRxBufferPos + xEventSize) >= MB_SERIAL_BUF_SIZE) ? 0 : uiRxBufferPos;
            // Get received packet into Rx buffer
            usLength = uart_read_bytes(ucUartNumber, &ucBuffer[uiRxBufferPos], xEventSize, portMAX_DELAY);
            /*
             * Note: in BARE master port there is additional code here (after reading buffer & flushing input)
               It seems it is protocol related and should not belong to the low-level serial port function actually.
               So it intentionally is not active.
               - discards frames of size 1
               - skips 0x00 bytes at the beginning (broadcast address returned by a slave)
               - dumps an error if message size is exactly 5 ???   (should be handled by Modbus stack instead)

                if (event.size > 1) // ignore char 0x00
                {
                    if (event.size == 5)
                    {
                        for (int i = 0; i < event.size; i++)
                            printf("|%02x", dtmp[i]);
                        printf("\r\n---------invalid addr data----------\r\n");
                    }
                    else
                    {
                        if (0 == dtmp[0])
                        {
                            ptr = dtmp + 1;
                            size = event.size - 1;
                        }
                        else
                        {
                            ptr = dtmp;
                            size = event.size;
                        }
                        mb_serial_read(ptr, size);  --> this is equal to the for loop below
                    }
                }
             */
            for(USHORT usCnt = 0; usCnt < usLength; usCnt++ ) {
                // Call the Modbus stack callback function and let it fill the buffers.
                ( void )pxMBFrameCBByteReceived(); // calls callback xMBRTUReceiveFSM() to execute MB state machine
            }
            // The buffer is transferred into Modbus stack and is not needed here any more
            uart_flush_input(ucUartNumber);
            // Send event EV_FRAME_RECEIVED to allow stack process packet
#ifndef MB_TIMER_PORT_ENABLED
            // Let the stack know that T3.5 time is expired and data is received
            (void)pxMBPortCBTimerExpired(); // calls callback xMBRTUTimerT35Expired();
#endif
            ESP_LOGD(TAG, "RX_T35_timeout: %d(bytes in buffer)\n", (uint32_t)usLength);
        }
    }
}

BOOL xMBPortSerialTxPoll()
{
    BOOL bStatus = FALSE;
    USHORT usCount = 0;
    BOOL bNeedPoll = FALSE;

    if( bTxStateEnabled ) {
        // Continue while all response bytes put in buffer or out of buffer
        while((bNeedPoll == FALSE) && (usCount++ < MB_SERIAL_BUF_SIZE)) {
            // Calls the modbus stack callback function to let it fill the UART transmit buffer.
            bNeedPoll = pxMBFrameCBTransmitterEmpty( ); // calls callback xMBRTUTransmitFSM();
        }
        ESP_LOGD(TAG, "MB_TX_buffer sent: (%d) bytes\n", (uint16_t)usCount);
        bStatus = TRUE;
    }
    return bStatus;
}

static void vUartTask(void *pvParameters)
{
    uart_event_t xEvent;
    for(;;) {
        if (xQueueReceive(xMbUartQueue, (void*)&xEvent, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "MB_uart[%d] event:", ucUartNumber);
            //vMBPortTimersEnable();
            switch(xEvent.type) {
                //Event of UART receving data
                case UART_DATA:
                    ESP_LOGD(TAG,"Receive data, len: %d", xEvent.size);
                    // Read received data and send it to modbus stack
                    vMBPortSerialRxPoll(xEvent.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGD(TAG, "hw fifo overflow\n");
                    uart_flush_input(ucUartNumber);   /* mipa added from master port */
                    xQueueReset(xMbUartQueue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGD(TAG, "ring buffer full\n");
                    uart_flush_input(ucUartNumber);  /* mipa master port: moved flush input before queue reset */
                    xQueueReset(xMbUartQueue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGD(TAG, "uart rx break\n");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGD(TAG, "uart parity error\n");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGD(TAG, "uart frame error\n");
                    break;
                // UART pattern detected 
                /*
                 *  Note: This case was in BARE master port but not active since they deactivated pattern recognition initialization.

                      case UART_PATTERN_DET:
                      uart_get_buffered_data_len(ucUartNumber, &buffered_size);
                          int pos = uart_pattern_pop_pos(ucUartNumber);
                          ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                          if (pos == -1)
                          {
                              // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                              // record the position. We should set a larger queue size.
                              // As an example, we directly flush the rx buffer here.
                              uart_flush_input(ucUartNumber);
                          }
                          else
                          {
                              //    uart_read_bytes(ucUartNumber, dtmp, pos, 100 / portTICK_PERIOD_MS);
                              //    uint8_t pat[PATTERN_CHR_NUM + 1];
                              //    memset(pat, 0, sizeof(pat));
                              //    uart_read_bytes(ucUartNumber, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                              //    ESP_LOGI(TAG, "read data: %s", dtmp);
                              //    ESP_LOGI(TAG, "read pat : %s", pat);
                          }
                          break;
                 */
                default:
                    ESP_LOGD(TAG, "uart event type: %d\n", xEvent.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

BOOL xMBPortSerialInit_wCtrlPin(UCHAR ucPORT, ULONG ulBaudRate,
                        UCHAR ucDataBits, eMBParity eParity, BOOL controlEnPin)
{
    esp_err_t xErr = ESP_OK;
    MB_PORT_CHECK((eParity <= MB_PAR_EVEN), FALSE, "mb serial set parity failure.");
    // Set communication port number
    ucUartNumber = ucPORT;
    // Configure serial communication parameters
    UCHAR ucParity = UART_PARITY_DISABLE;
    UCHAR ucData = UART_DATA_8_BITS;
    int rtsPin  = UART_PIN_NO_CHANGE;
    switch(eParity){
        case MB_PAR_NONE:
            ucParity = UART_PARITY_DISABLE;
            break;
        case MB_PAR_ODD:
            ucParity = UART_PARITY_ODD;
            break;
        case MB_PAR_EVEN:
            ucParity = UART_PARITY_EVEN;
            break;
    }
    switch(ucDataBits){
        case 5:
            ucData = UART_DATA_5_BITS;
            break;
        case 6:
            ucData = UART_DATA_6_BITS;
            break;
        case 7:
            ucData = UART_DATA_7_BITS;
            break;
        case 8:
            ucData = UART_DATA_8_BITS;
            break;
        default:
            ucData = UART_DATA_8_BITS;
            break;
    }
    uart_config_t xUartConfig = {
        .baud_rate = ulBaudRate,
        .data_bits = ucData,
        .parity = ucParity,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 2,
    };

    // Set UART config
    xErr = uart_param_config(ucUartNumber, &xUartConfig);
    MB_PORT_CHECK((xErr == ESP_OK),
            FALSE, "mb config failure, uart_param_config() returned (0x%x).", (uint32_t)xErr);

    if (controlEnPin == TRUE)
    {
        rs485_en_pin_cfg();
        rtsPin = MB_UART_EN_PIN;
    }

    //Set UART pins according to configuration (using default pins ie no changes.)
    uart_set_pin(ucUartNumber, MB_UART_TXD, MB_UART_RXD, rtsPin, UART_PIN_NO_CHANGE);

    // Install UART driver, and get the queue.
    xErr = uart_driver_install(ucUartNumber, MB_SERIAL_BUF_SIZE, MB_SERIAL_BUF_SIZE,
            MB_QUEUE_LENGTH, &xMbUartQueue, ESP_INTR_FLAG_LOWMED);
    MB_PORT_CHECK((xErr == ESP_OK), FALSE,
            "mb serial driver failure, uart_driver_install() returned (0x%x).", (uint32_t)xErr);
#ifndef MB_TIMER_PORT_ENABLED
    // Set timeout for TOUT interrupt (T3.5 modbus time)
    xErr = uart_set_rx_timeout(ucUartNumber, MB_SERIAL_TOUT);
    MB_PORT_CHECK((xErr == ESP_OK), FALSE,
            "mb serial set rx timeout failure, uart_set_rx_timeout() returned (0x%x).", (uint32_t)xErr);
#endif
    // Create a task to handle UART events
    BaseType_t xStatus = xTaskCreate(vUartTask, "uart_queue_task", MB_SERIAL_TASK_STACK_SIZE,
                                        NULL, MB_SERIAL_TASK_PRIO, &xMbTaskHandle);
    if (xStatus != pdPASS) {
        vTaskDelete(xMbTaskHandle);
        // Force exit from function with failure
        MB_PORT_CHECK(FALSE, FALSE,
                "mb stack serial task creation error. xTaskCreate() returned (0x%x).",
                (uint32_t)xStatus);
    } else {
        vTaskSuspend(xMbTaskHandle); // Suspend serial task while stack is not started
    }
    uiRxBufferPos = 0;
    return TRUE;
}

void vMBPortSerialClose()
{
    (void)vTaskSuspend(xMbTaskHandle);
    (void)vTaskDelete(xMbTaskHandle);
    ESP_ERROR_CHECK(uart_driver_delete(ucUartNumber));
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
    // Send one byte to UART transmission buffer
    // This function is called by Modbus stack
    UCHAR ucLength = uart_write_bytes(ucUartNumber, &ucByte, 1);
    return (ucLength == 1);
}

BOOL xMBPortSerialPutBytes(CHAR *ucBytes, UCHAR len)
{
    // Send buffer to UART transmission buffer
    // This function is called by Modbus stack
    UCHAR ucLength = uart_write_bytes(ucUartNumber, ucBytes, len);
    return (ucLength == len);
}

// Get one byte from intermediate RX buffer
BOOL xMBPortSerialGetByte(CHAR* pucByte)
{
    assert(pucByte != NULL);
    MB_PORT_CHECK((uiRxBufferPos < MB_SERIAL_BUF_SIZE),
            FALSE, "mb stack serial get byte failure.");
    *pucByte = ucBuffer[uiRxBufferPos];
    uiRxBufferPos++;
    return TRUE;
}


