/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbrtu.c,v 1.18 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbrtu.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"
#include "mbconfig.h"

#include "pool.h"

/* ----------------------- Type definitions ---------------------------------*/
// Master + Slave role states
typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is being received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_XMIT,              /*!< Transmitter is in transfer state. */
    STATE_M_TX_XFWR             /*!< Transmitter is in transfer finish and wait receive state (master role only). */
} eMBSndState;

typedef void ( *pvMBTimerEnable ) ( void );

/* ----------------------- Static variables ---------------------------------*/
static const char *TAG    = "MB_RTU";

// TODO these statics need to be instance-local, to enable handling multiple buses simultaneously by separate instances
static eMBRole eMyRole;

/* Both roles */
static volatile sMBMasterQueueType *rtu_global_descriptor = NULL;
static volatile UCHAR *pucSndBufferCur;
static volatile eMBSndState eSndState;
static volatile eMBRcvState eRcvState;
static volatile USHORT usSndBufferCount;
static volatile USHORT usRcvBufferPos;

/* Master role */
static volatile eMBMasterTimerMode eMasterCurTimerMode;

/* ----------------------- Static functions ---------------------------------*/

/* Functions pointers initialized in eMBGenericRTUInit. 
 * Depending on the role (MB_ROLE_SLAVE or MB_ROLE_MASTER) they are set to the correct implementations.
 */
static pvMBTimerEnable pvMBPortTimerEnable; 

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBGenericRTUInit( eMBRole eRole, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity, sMBMasterQueueType *initDescriptor )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    ULONG           usTimerT35_50us;

    ESP_LOGI(TAG, "entered eMBGenericRTUInit");
    eMyRole = eRole;

    if (initDescriptor == NULL)
    {
        ESP_LOGE(TAG, "eMBGenericRTUInit - no descriptor given");
        return MB_EINVAL;
    }

    rtu_global_descriptor = initDescriptor;
    pvMBPortTimerEnable   = (eMyRole == MB_ROLE_SLAVE) ? vMBPortTimersEnable : vMBMasterPortTimersT35Enable; 

    ( void )ucSlaveAddress;
    ENTER_CRITICAL_SECTION(  );

    /* Modbus RTU uses 8 Databits. */
    if( xMBPortSerialInit( ucPort, ulBaudRate, 8, eParity ) != TRUE )
    {
        eStatus = MB_EPORTERR;
    }
    else
    {
        /* If baudrate > 19200 then we should use the fixed timer values
         * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
         */
        if( ulBaudRate > 19200 )
        {
            usTimerT35_50us = 35;       /* 1800us. */
        }
        else
        {
            /* The timer reload value for a character is given by:
             *
             * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
             *             = 11 * Ticks_per_1s / Baudrate
             *             = 220000 / Baudrate
             * The reload for t3.5 is 1.5 times this value and similary
             * for t3.5.
             */
            usTimerT35_50us = ( 7UL * 220000UL ) / ( 2UL * ulBaudRate );
        }
        if( xMBPortTimersInit( ( USHORT ) usTimerT35_50us, (eRole == MB_ROLE_MASTER) ? TRUE : FALSE ) != TRUE )
        {
            eStatus = MB_EPORTERR;
        }
    }
    EXIT_CRITICAL_SECTION(  );

    return eStatus;
}

void
eMBRTUStart( void )
{
    ESP_LOGI(TAG, "entered eMBRTUStart");
    ENTER_CRITICAL_SECTION(  );
    /* Initially the receiver is in the state STATE_RX_INIT. we start
     * the timer and if no character is received within t3.5 we change
     * to STATE_RX_IDLE. This makes sure that we delay startup of the
     * modbus protocol stack until the bus is free.
     */
    eRcvState = STATE_RX_INIT;
    vMBPortSerialEnable( TRUE, FALSE );
    pvMBPortTimerEnable(  );

    EXIT_CRITICAL_SECTION(  );
}

void
eMBRTUStop( void )
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable( FALSE, FALSE );
    vMBPortTimersDisable(  );
    EXIT_CRITICAL_SECTION(  );
}


eMBErrorCode
eMBRTUReceive( UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength )
{
    eMBErrorCode    eStatus  = MB_ENOERR;
    volatile UCHAR *ucRTUBuf;

    ESP_LOGD(TAG, "entered eMBRTUReceive");

    if (rtu_global_descriptor == 0) return MB_EILLSTATE;

    ucRTUBuf = rtu_global_descriptor->rcv_frame;

    ENTER_CRITICAL_SECTION(  );
    assert( usRcvBufferPos < MB_SER_PDU_SIZE_MAX );

    /* Length and CRC check */
    if( ( usRcvBufferPos >= MB_SER_PDU_SIZE_MIN )
        && ( usMBCRC16( ( UCHAR * ) ucRTUBuf, usRcvBufferPos ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layer
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucRTUBuf[MB_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( USHORT )( usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( UCHAR * ) & ucRTUBuf[MB_SER_PDU_PDU_OFF];

        // DEBUG prints only
        /*
        ESP_LOGI(TAG, "eMBRTUReceive rcv addr = %d, frame: %8.8X, length: %d - low level modbus frame received", (UCHAR)*pucRcvAddress, (uint32_t)ucRTUBuf, usRcvBufferPos);
           for (int i=0; i<usRcvBufferPos;i++)
           {
               printf("%d ", ucRTUBuf[i]);
           }      
           printf("\n");
        */
        // DEBUG prints end
    }
    else
    {
        eStatus = MB_EIO;
    }

    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode vMBRTUInvalidate(BOOL isFinalClosing)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    // descriptor became invalid, so we need a new one except if we're instructed to finally close the RTU driver.
    if (isFinalClosing == FALSE)
    {
        rtu_global_descriptor = (sMBMasterQueueType *)palloc();
        if (rtu_global_descriptor == NULL) 
            return MB_ENORES;
    } else {
        pfree((void *)rtu_global_descriptor);
        rtu_global_descriptor = NULL;
    }
    return eStatus;
}

eMBErrorCode
eMBRTUSend( UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength, sMBMasterQueueType *psInternalFrameDescriptor )   
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          usCRC16;

    ESP_LOGD(TAG, "entered eMBRTUSend, descriptor: %8.8X, receive state: %d", (uint32_t)psInternalFrameDescriptor, eRcvState);

    // TODO eventually need to adapt ASCII and TCP as well (also for slave role) to use descriptors, not global arrays anymore.
    if (eMyRole == MB_ROLE_MASTER)
    {
        // if a RTU internal descriptor exists discard it first in favor of the new one just received from modbus task.
        if (rtu_global_descriptor != NULL) pfree((void *)rtu_global_descriptor);

        rtu_global_descriptor = psInternalFrameDescriptor;
        if ( (rtu_global_descriptor == 0) || (ucSlaveAddress > MB_MASTER_TOTAL_SLAVE_NUM) )
        {
            return MB_EINVAL;
        }
    }

    ENTER_CRITICAL_SECTION(  );

    /* Check if the receiver is still in idle state. If not we were too
     * slow with processing the received frame and the master/slave sent
     * another frame on the network. We have to abort sending the frame.
     */
    if( eRcvState == STATE_RX_IDLE )
    {
        /* First byte before the Modbus-PDU is the slave address. */
        pucSndBufferCur = ( UCHAR * ) pucFrame - 1;
        usSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
        usCRC16 = usMBCRC16( ( UCHAR * ) pucSndBufferCur, usSndBufferCount );
        pucSndBufferCur[usSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
        pucSndBufferCur[usSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );

        /* Activate the transmitter. */
        rs485_trans_toggle(1);
        eSndState = STATE_TX_XMIT;

        // DEBUG prints only
           /*
           printf("pucSndBufferCur = %8.8X, length %d\nLow level modbus frame to send to serial:", (uint32_t)pucSndBufferCur, usSndBufferCount);
           for (int i=0; i<usSndBufferCount;i++)
           {
               printf("%d ", pucFrame[i-1]);
           }      
           printf("\n");
           */
        // DEBUG prints end

        vMBPortSerialEnable( FALSE, TRUE );
        //rs485_wait_tx_done();
        //rs485_trans_toggle(0);
    }
    else
    {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

BOOL
xMBRTUReceiveFSM( void )
{
    UCHAR           ucByte;
    BOOL            xTaskNeedSwitch = FALSE;
    volatile UCHAR *ucRTUBuf;

    //ESP_LOGD(TAG, "entered xMBRTUReceiveFSM, receive state: %d", eRcvState);

    if (rtu_global_descriptor == 0) return MB_EILLSTATE;
    ucRTUBuf = rtu_global_descriptor->rcv_frame;

    if (eMyRole == MB_ROLE_SLAVE)
    {
        assert( eSndState == STATE_TX_IDLE );
    } else {
        assert((eSndState == STATE_TX_IDLE) || (eSndState == STATE_M_TX_XFWR));    
    }

    /* Always read the character. */
    ( void )xMBPortSerialGetByte( ( CHAR * ) & ucByte );

    switch ( eRcvState )
    {
        /* If we have received a character in the init state we have to
         * wait until the frame is finished.
         */
    case STATE_RX_INIT: //0x0
        pvMBPortTimerEnable(  );
        break;

        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
    case STATE_RX_ERROR: //0x3
        pvMBPortTimerEnable(  );
        break;

        /* In the idle state we wait for a new character. If a character
         * is received the t1.5 and t3.5 timers are started and the
         * receiver is in the state STATE_RX_RECEIVCE.
         */
    case STATE_RX_IDLE: //0x1
        if (eMyRole == MB_ROLE_MASTER)
        {
            /* In time of respond timeout,the receiver received a frame.
             * Disable timer of respond timeout and change the transmitter state to idle.
             */
            vMBPortTimersDisable();
            eSndState = STATE_TX_IDLE;
        }
        usRcvBufferPos = 0;
        ucRTUBuf[usRcvBufferPos++] = ucByte;
        eRcvState = STATE_RX_RCV;

        /* Enable t3.5 timers. */
        pvMBPortTimerEnable(  );
        break;

        /* We are currently receiving a frame. Reset the timer after
         * every character received. If more than the maximum possible
         * number of bytes in a modbus frame is received the frame is
         * ignored.
         */
    case STATE_RX_RCV: //0x2
        if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX )
        {
            ucRTUBuf[usRcvBufferPos++] = ucByte;
        }
        else
        {
            eRcvState = STATE_RX_ERROR;
        }
        pvMBPortTimerEnable(  );
        break;
    }
    return xTaskNeedSwitch;
}

BOOL
xMBRTUTransmitFSM( void ) 
{
    BOOL xNeedPoll = FALSE;

    assert( eRcvState == STATE_RX_IDLE );

    //ESP_LOGD(TAG, "entered xMBRTUTransmitFSM, send state: %d", eSndState);

    switch ( eSndState )
    {
        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_M_TX_XFWR://0x2
        //ESP_LOGD(TAG, "in xMBRTUTransmitFSM, send state TX_XFWR");
        break;
    case STATE_TX_IDLE://0x0
        //ESP_LOGD(TAG, "in xMBRTUTransmitFSM, send state TX_IDLE");
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable( TRUE, FALSE );
        rs485_trans_toggle(0);
        break;

    case STATE_TX_XMIT://0x1
        /* check if we are finished. */
        if( usSndBufferCount != 0 )
        {
            xMBPortSerialPutBytes( (CHAR *)pucSndBufferCur, usSndBufferCount );
            pucSndBufferCur+=usSndBufferCount;  /* after bytes just sent. */
            usSndBufferCount = 0;
        }
        else
        {
            if (eMyRole == MB_ROLE_SLAVE)
            {
                xNeedPoll = xMBPortEventPost( EV_FRAME_SENT ); //, rtu_global_descriptor );
                /* Disable transmitter. This prevents another transmit buffer
                 * empty interrupt. */
                vMBPortSerialEnable( TRUE, FALSE );
                rs485_wait_tx_done();
                rs485_trans_toggle(0);
                eSndState = STATE_TX_IDLE;
            } else { /* Master role: Transmit FSM gets called from serial transmit function. */
                /* Disable transmitter. This prevents another transmit buffer
                 * empty interrupt. */
                vMBPortSerialEnable(TRUE, FALSE);
                rs485_wait_tx_done();
                rs485_trans_toggle(0);
                eSndState = STATE_M_TX_XFWR;
                /* If the frame is broadcast, master will enable timer of convert delay,
                 * else master will enable timer of respond timeout. */
                if (MASTER_REQUEST_IS_BROADCAST(rtu_global_descriptor) == TRUE)
                {
                    vMBMasterPortTimersConvertDelayEnable();
                }
                else
                {
                    vMBMasterPortTimersRespondTimeoutEnable();
                }
                xNeedPoll = TRUE;
            }
        }
        break;
    }
    return xNeedPoll;
}

// Slave role only
BOOL
xMBRTUTimerT35Expired( void )
{
    BOOL            xNeedPoll = FALSE;

    switch ( eRcvState )
    {
        /* Timer t35 expired. Startup phase is finished. */
    case STATE_RX_INIT:
        xNeedPoll = xMBPortEventPost( EV_READY ); //, rtu_global_descriptor );
        break;

        /* A frame was received and t35 expired. Notify the listener that
         * a new frame was received. */
    case STATE_RX_RCV:
        xNeedPoll = xMBPortEventPost( EV_FRAME_RECEIVED ); //, rtu_global_descriptor );
        break;

        /* An error occured while receiving the frame. */
    case STATE_RX_ERROR:
        break;

        /* Function called in an illegal state. */
    default:
        assert( ( eRcvState == STATE_RX_INIT ) ||
                ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_ERROR ) );
    }

    vMBPortTimersDisable(  );
    eRcvState = STATE_RX_IDLE;

    return xNeedPoll;
}

// From here on, all functions are for Master role only.
// NOTE: This will be called in interrupt context!
BOOL xMBMasterRTUTimerExpired(void)
{
    BOOL xNeedPoll = FALSE;

    switch (eRcvState)
    {
        /* Timer t35 expired. Startup phase is finished. */
        case STATE_RX_INIT://0x0
            xNeedPoll = xMBMasterPortEventPost(EV_MASTER_READY, (sMBMasterQueueType **)&rtu_global_descriptor);
            break;

        /* A frame was received and t35 expired. Notify the listener that
         * a new frame was received. */
        case STATE_RX_RCV://0x2
            xNeedPoll = xMBMasterPortEventPost(EV_MASTER_FRAME_RECEIVED, (sMBMasterQueueType **)&rtu_global_descriptor);
            break;

        /* An error occured while receiving the frame. */
        case STATE_RX_ERROR://0x3
            rtu_global_descriptor->error_type = MB_MRE_REV_DATA;
            xNeedPoll = xMBMasterPortEventPost(EV_MASTER_ERROR_PROCESS, (sMBMasterQueueType **)&rtu_global_descriptor);
            break;

        /* Function called in an illegal state. */
        default:
            assert(
                (eRcvState == STATE_RX_INIT) || (eRcvState == STATE_RX_RCV) ||
                (eRcvState == STATE_RX_ERROR) || (eRcvState == STATE_RX_IDLE));
            break;
    }
    eRcvState = STATE_RX_IDLE;

    switch (eSndState)
    {
        /* A frame was send finish and convert delay or respond timeout expired.
         * If the frame is broadcast,The master will idle,and if the frame is not
         * broadcast.Notify the listener process error.*/
        case STATE_M_TX_XFWR://0x2
            if (MASTER_REQUEST_IS_BROADCAST(rtu_global_descriptor) == FALSE)
            {
                rtu_global_descriptor->error_type = MB_MRE_TIMEDOUT;
                xNeedPoll = xMBMasterPortEventPost(EV_MASTER_ERROR_PROCESS, (sMBMasterQueueType **)&rtu_global_descriptor);
            }
            break;
        /* Function called in an illegal state. */
        default:
            assert(
                (eSndState == STATE_M_TX_XFWR) || (eSndState == STATE_TX_IDLE));
            break;
    }
    eSndState = STATE_TX_IDLE;

    vMBPortTimersDisable();
    /* If timer mode is convert delay, the master event then turns EV_MASTER_EXECUTE status. */
    if (eMasterCurTimerMode == MB_TMODE_CONVERT_DELAY)
    {
        xNeedPoll = xMBMasterPortEventPost(EV_MASTER_EXECUTE, (sMBMasterQueueType **)&rtu_global_descriptor);
    }

    return xNeedPoll;
}

/* Set Modbus Master current timer mode.*/
void vMBMasterSetCurTimerMode(eMBMasterTimerMode eMBTimerMode)
{
    eMasterCurTimerMode = eMBTimerMode;
}

