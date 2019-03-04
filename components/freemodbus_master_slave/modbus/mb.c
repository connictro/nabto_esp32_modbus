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
 * File: $Id: mb.c,v 1.28 2010/06/06 13:54:40 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "pool.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbframe.h"
#include "mbconfig.h"
#include "mbproto.h"
#include "mbfunc.h"

#if MB_RTU_ENABLED == 1
#include "mbrtu.h"
#endif
#if MB_ASCII_ENABLED == 1
#include "mbascii.h"
#endif
#if MB_TCP_ENABLED == 1
#include "mbtcp.h"
#endif

#ifndef MB_PORT_HAS_CLOSE
#define MB_PORT_HAS_CLOSE 0
#endif

/* if  MB_DEVICE_USE_TYPE == MB_DEVICE_MASTER  */

/* ----------------------- Static variables ---------------------------------*/

static const char* TAG = "MB_C";

// Role independent & role setting
static eMBRole  eMyRole = MB_ROLE_SLAVE;
static eMBMode  eMBCurrentMode;
static enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} eMBState = STATE_NOT_INITIALIZED;

// Slave role specific
static UCHAR    ucMBAddress;
static sMBMasterQueueType *initDescriptor;

// Master role specific 

/* Functions pointer which are initialized in eMBInit( ). Depending on the
 * mode (RTU or ASCII) and role (SLAVE or MASTER) the are set to the correct implementations.
 */
static peMBFrameSend peMBFrameSendCur;
static pvMBFrameStart pvMBFrameStartCur;
static pvMBFrameStop pvMBFrameStopCur;
static peMBFrameReceive peMBFrameReceiveCur;
static pvMBFrameClose pvMBFrameCloseCur;
static pvMBFrameInvalidate pvMBFrameInvalidateCur;

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 */
BOOL( *pxMBFrameCBByteReceived )     ( void );
BOOL( *pxMBFrameCBTransmitterEmpty ) ( void );
BOOL( *pxMBPortCBTimerExpired )      ( void );

BOOL( *pxMBFrameCBReceiveFSMCur )    ( void );
BOOL( *pxMBFrameCBTransmitFSMCur )   ( void );

/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX];

/* ----------------------- Static functions ---------------------------------*/


/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBInit( eMBRole eRole, eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    int             i;
    eMBErrorCode    eStatus = MB_ENOERR;

    ESP_LOGI(TAG, "Entered eMBInit with role %d, mode %d, port %d, baudrate %d, parity %d", (int)eRole, (int)eMode, (int)ucPort, (int)ulBaudRate, (int)eParity);

    eMyRole = eRole;

    // 1. Initialize Function handlers at runtime, dependent on role selection.
    i = 0;
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    xFuncHandlers[i].ucFunctionCode = MB_FUNC_OTHER_REPORT_SLAVEID;
    xFuncHandlers[i].pxHandler      = eMBFuncReportSlaveID;
    i++;
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    xFuncHandlers[i].ucFunctionCode = MB_FUNC_READ_INPUT_REGISTER;
    xFuncHandlers[i].pxHandler      = (eRole == MB_ROLE_SLAVE) ? eMBFuncReadInputRegister : eMBMasterFuncReadInputRegister;
    i++;
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    xFuncHandlers[i].ucFunctionCode = MB_FUNC_READ_HOLDING_REGISTER;
    xFuncHandlers[i].pxHandler      = (eRole == MB_ROLE_SLAVE) ? eMBFuncReadHoldingRegister : eMBMasterFuncReadHoldingRegister;
    i++;
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    xFuncHandlers[i].ucFunctionCode = MB_FUNC_WRITE_MULTIPLE_REGISTERS;
    xFuncHandlers[i].pxHandler      = (eRole == MB_ROLE_SLAVE) ? eMBFuncWriteMultipleHoldingRegister : eMBMasterFuncWriteMultipleHoldingRegister;
    i++;
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    xFuncHandlers[i].ucFunctionCode = MB_FUNC_WRITE_REGISTER;
    xFuncHandlers[i].pxHandler      = (eRole == MB_ROLE_SLAVE) ? eMBFuncWriteHoldingRegister : eMBMasterFuncWriteHoldingRegister;
    i++;
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    xFuncHandlers[i].ucFunctionCode = MB_FUNC_READWRITE_MULTIPLE_REGISTERS;
    xFuncHandlers[i].pxHandler      = (eRole == MB_ROLE_SLAVE) ? eMBFuncReadWriteMultipleHoldingRegister :eMBMasterFuncReadWriteMultipleHoldingRegister;
    i++;
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    xFuncHandlers[i].ucFunctionCode = MB_FUNC_READ_COILS;
    xFuncHandlers[i].pxHandler      = (eRole == MB_ROLE_SLAVE) ? eMBFuncReadCoils : eMBMasterFuncReadCoils;
    i++;
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    xFuncHandlers[i].ucFunctionCode = MB_FUNC_WRITE_SINGLE_COIL;
    xFuncHandlers[i].pxHandler      = (eRole == MB_ROLE_SLAVE) ? eMBFuncWriteCoil : eMBMasterFuncWriteCoil;
    i++;
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    xFuncHandlers[i].ucFunctionCode = MB_FUNC_WRITE_MULTIPLE_COILS;
    xFuncHandlers[i].pxHandler      = (eRole == MB_ROLE_SLAVE) ? eMBFuncWriteMultipleCoils : eMBMasterFuncWriteMultipleCoils;
    i++;
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    xFuncHandlers[i].ucFunctionCode = MB_FUNC_READ_DISCRETE_INPUTS;
    xFuncHandlers[i].pxHandler      = (eRole == MB_ROLE_SLAVE) ? eMBFuncReadDiscreteInputs : eMBMasterFuncReadDiscreteInputs;
    i++;
#endif

    /* check preconditions in slave role only */
    if (eRole == MB_ROLE_SLAVE)
    {
        if( ( ucSlaveAddress == MB_ADDRESS_BROADCAST ) ||
            ( ucSlaveAddress < MB_ADDRESS_MIN ) || ( ucSlaveAddress > MB_ADDRESS_MAX ) )
        {
            eStatus = MB_EINVAL;
            return eStatus;
        } else {
            ucMBAddress = ucSlaveAddress;
        }
    }

    initDescriptor = (sMBMasterQueueType *)palloc();
    if (initDescriptor == NULL)
    {
        eMBState = MB_ENORES;
    }

    switch ( eMode )
    {
#if MB_RTU_ENABLED > 0
    case MB_RTU:
        pvMBFrameStartCur = eMBRTUStart;
        pvMBFrameStopCur = eMBRTUStop;
        peMBFrameSendCur = eMBRTUSend;
        peMBFrameReceiveCur = eMBRTUReceive;
        pvMBFrameInvalidateCur = vMBRTUInvalidate;
        pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
        pxMBFrameCBByteReceived = xMBRTUReceiveFSM;
        pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM;
        pxMBPortCBTimerExpired = (eRole == MB_ROLE_SLAVE) ? xMBRTUTimerT35Expired : xMBMasterRTUTimerExpired;

        eStatus = (eRole == MB_ROLE_SLAVE) ? 
                      eMBRTUInit( ucMBAddress, ucPort, ulBaudRate, eParity, initDescriptor ) :
                      eMBMasterRTUInit(ucPort, ulBaudRate, eParity, initDescriptor );
        break;
#endif

// NOTE: ASCII is not yet implemented for Master role
#if MB_ASCII_ENABLED > 0
    case MB_ASCII:
        pvMBFrameStartCur = eMBASCIIStart;
        pvMBFrameStopCur = eMBASCIIStop;
        peMBFrameSendCur = eMBASCIISend;
        peMBFrameReceiveCur = eMBASCIIReceive;
        pvMBFrameInvalidateCur = vMBASCIIInvalidate;
        pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
        pxMBFrameCBByteReceived = xMBASCIIReceiveFSM;
        pxMBFrameCBTransmitterEmpty = xMBASCIITransmitFSM;
        pxMBPortCBTimerExpired = (eRole == MB_ROLE_SLAVE) ? xMBASCIITimerT1SExpired : xMBMasterASCIITimerExpired;

        eStatus = (eRole == MB_ROLE_SLAVE) ? 
                     eMBASCIIInit( ucMBAddress, ucPort, ulBaudRate, eParity ) : 
                     eMBMasterASCIIInit(ucPort, ulBaudRate, eParity);
        break;
#endif

    default:
        eStatus = MB_EINVAL;
    }

    if( eStatus == MB_ENOERR )
    {
        if( !((eRole == MB_ROLE_SLAVE) ? xMBPortEventInit() : xMBMasterPortQueueInit()))
        {
            /* port dependent event module initalization failed. */
            eStatus = MB_EPORTERR;
        }
        else
        {
            eMBCurrentMode = eMode;
            eMBState = STATE_DISABLED;
        }
    }
    return eStatus;
}

#if MB_TCP_ENABLED > 0
eMBErrorCode
eMBTCPInit( USHORT ucTCPPort )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( ( eStatus = eMBTCPDoInit( ucTCPPort ) ) != MB_ENOERR )
    {
        eMBState = STATE_DISABLED;
    }
    else if( !xMBPortEventInit(  ) )
    {
        /* Port dependent event module initalization failed. */
        eStatus = MB_EPORTERR;
    }
    else
    {
        pvMBFrameStartCur = eMBTCPStart;
        pvMBFrameStopCur = eMBTCPStop;
        peMBFrameReceiveCur = eMBTCPReceive;
        peMBFrameSendCur = eMBTCPSend;
        pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBTCPPortClose : NULL;
        ucMBAddress = MB_TCP_PSEUDO_ADDRESS;
        eMBCurrentMode = MB_TCP;
        eMBState = STATE_DISABLED;
    }
    return eStatus;
}
#endif

eMBErrorCode
eMBRegisterCB( UCHAR ucFunctionCode, pxMBFunctionHandler pxHandler )
{
    int             i;
    eMBErrorCode    eStatus;

    if( ( 0 < ucFunctionCode ) && ( ucFunctionCode <= 127 ) )
    {
        ENTER_CRITICAL_SECTION(  );
        if( pxHandler != NULL )
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( ( xFuncHandlers[i].pxHandler == NULL ) ||
                    ( xFuncHandlers[i].pxHandler == pxHandler ) )
                {
                    xFuncHandlers[i].ucFunctionCode = ucFunctionCode;
                    xFuncHandlers[i].pxHandler = pxHandler;
                    break;
                }
            }
            eStatus = ( i != MB_FUNC_HANDLERS_MAX ) ? MB_ENOERR : MB_ENORES;
        }
        else
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    xFuncHandlers[i].ucFunctionCode = 0;
                    xFuncHandlers[i].pxHandler = NULL;
                    break;
                }
            }
            /* Remove can't fail. */
            eStatus = MB_ENOERR;
        }
        EXIT_CRITICAL_SECTION(  );
    }
    else
    {
        eStatus = MB_EINVAL;
    }
    return eStatus;
}


eMBErrorCode
eMBClose( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        if( pvMBFrameCloseCur != NULL )
        {
            pvMBFrameCloseCur(  );
        }
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    (void) pvMBFrameInvalidateCur(TRUE);
    pfree(initDescriptor);
    return eStatus;
}

eMBErrorCode
eMBEnable( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    ESP_LOGD(TAG, "Entered eMBEnable");
    if( eMBState == STATE_DISABLED )
    {
        /* Activate the protocol stack. */
        pvMBFrameStartCur(  );
        eMBState = STATE_ENABLED;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBDisable( void )
{
    eMBErrorCode    eStatus;

    if( eMBState == STATE_ENABLED )
    {
        pvMBFrameStopCur(  );
        eMBState = STATE_DISABLED;
        eStatus = MB_ENOERR;
    }
    else if( eMBState == STATE_DISABLED )
    {
        eStatus = MB_ENOERR;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBPoll( void )
{
    static UCHAR           *ucMBFrame;
    static UCHAR            ucRcvAddress;
    static UCHAR            ucFunctionCode;
    static USHORT           usLength;
    static eMBException     eException;
    static BOOL             bMasterIsBusy = FALSE;

    int                     i, j;
    eMBErrorCode            eStatus = MB_ENOERR;
    eMBEventType            eSlaveEvent;
    sMBMasterQueueType     *psInternalFrameDescriptor;

    /* Check if the protocol stack is ready. */
    if( eMBState != STATE_ENABLED )
    {
        return MB_EILLSTATE;
    }

    if (eMyRole == MB_ROLE_MASTER)
    {
        /* Check if there is a event (message queue based) available. If not return control to caller.
         * Otherwise we will handle the event. */
        if (xMBMasterPortQueueGet(&psInternalFrameDescriptor) == TRUE)
        {
            ESP_LOGD(TAG, "Received frame descriptor %8.8X, event type: %4.4X", (uint32_t)psInternalFrameDescriptor, psInternalFrameDescriptor->event_type);

            switch (psInternalFrameDescriptor->event_type)
            {
                case EV_MASTER_INIT: //0x200
                    ESP_LOGD(TAG, "Master received init event");
                    bMasterIsBusy = TRUE;
                    (void)pvMBFrameInvalidateCur(FALSE);         // invalidate descriptor in modbus stack
                    pfree(psInternalFrameDescriptor); /* this descriptor is not needed anymore (no other payload but the init event)
                                                       * In all other cases we return it to the modbus supervisor (both on success and error completion states)
                                                       */
                    break;
                case EV_MASTER_ERROR_RESPOND_TIMEOUT: //0x40
                    //printf("%s:EV_MASTER_ERROR_RESPOND_TIMEOUT\r\n", __func__); break;
                    /* FALLTHROUGH */
                case EV_MASTER_ERROR_RECEIVE_DATA: //0x80
                    //printf("%s:EV_MASTER_ERROR_RECEIVE_DATA\r\n", __func__); break;
                    /* FALLTHROUGH */
                case EV_MASTER_ERROR_EXECUTE_FUNCTION: //0x100
                    //printf("%s:EV_MASTER_ERROR_EXECUTE_FUNCTION\r\n", __func__);

                    // return error result to caller (modbus supervisor)
                    (void)eMBMasterGenericCB(psInternalFrameDescriptor);

                    /* FALLTHROUGH */
                case EV_MASTER_PROCESS_SUCCESS: //0x20
                    //printf("%s:EV_MASTER_PROCESS_SUCCESS\r\n", __func__); break;
                    /* Result have been returned to the caller (error processing above or by callbacks in the MASTER_EXECUTE phase).
                     * Master is not busy anymore and postponed requests might now be processed
                     * (if any, moved from postponed to master processing queue).
                     * Furthermore, need to invalidate descriptor in modbus RTU since ownership has been returned to modbus supervisor.
                     */
                    (void)pvMBFrameInvalidateCur(FALSE); // invalidate descriptor in modbus RTU
                    bMasterIsBusy = FALSE;
                    vMBMasterUnpostponeMasterQueue();
                    break;
                case EV_MASTER_READY: //0x01
                    //printf("%s:EV_MASTER_READY\r\n", __func__);
                    ESP_LOGD(TAG, "Master ready!");
                    bMasterIsBusy = FALSE;
                    vMBMasterUnpostponeMasterQueue();
                    break;

                case EV_MASTER_FRAME_RECEIVED: //0x02
                    //printf("%s:EV_MASTER_FRAME_RECEIVED\r\n", __func__);
                    eStatus = peMBFrameReceiveCur(&ucRcvAddress, &ucMBFrame, &usLength);
                    /* Check if the frame is for us. If not, send an error process event. */
                    if ((eStatus == MB_ENOERR) && (ucRcvAddress == psInternalFrameDescriptor->dest_address))
                    {
                        psInternalFrameDescriptor->pdu_recv_length = usLength;
                        (void)xMBMasterPortEventPost(EV_MASTER_EXECUTE, &psInternalFrameDescriptor);
                    } else {
                        psInternalFrameDescriptor->error_type = MB_MRE_REV_DATA;
                        (void)xMBMasterPortEventPost(EV_MASTER_ERROR_PROCESS, &psInternalFrameDescriptor);
                    }
                    break;

                case EV_MASTER_EXECUTE: //0x04
                    //printf("%s:EV_MASTER_EXECUTE\r\n", __func__);
                    ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
                    eException = MB_EX_ILLEGAL_FUNCTION;
                    /* If receive frame has exception. The receive function code highest bit is 1.*/
                    if (ucFunctionCode >> 7)
                    {
                        eException = (eMBException)ucMBFrame[MB_PDU_DATA_OFF];
                    } else {
                        for (i = 0; i < MB_FUNC_HANDLERS_MAX; i++)
                        {
                            /* No more function handlers registered. Abort. */
                            if (xFuncHandlers[i].ucFunctionCode == 0)
                            {
                                break;
                            }
                            else if (xFuncHandlers[i].ucFunctionCode == ucFunctionCode)
                            {
                                /* If master request is broadcast,
                                 * the master need execute function for all slave.
                                 */
                                if (MASTER_REQUEST_IS_BROADCAST(psInternalFrameDescriptor))
                                {
                                    usLength = psInternalFrameDescriptor->pdu_send_length;
                                    for (j = 1; j <= MB_MASTER_TOTAL_SLAVE_NUM; j++)
                                    {
                                        psInternalFrameDescriptor->dest_address = j;
                                    }
                                } else {
                                    eException = xFuncHandlers[i].pxHandler(ucMBFrame, &usLength, psInternalFrameDescriptor);
                                }
                                break;
                            }
                        }
                    }
                    /* If master has exception, Master will send error process.Otherwise the Master is idle.*/
                    if (eException != MB_EX_NONE)
                    {
                        psInternalFrameDescriptor->error_type = MB_MRE_EXE_FUN;
                        (void)xMBMasterPortEventPost(EV_MASTER_ERROR_PROCESS, &psInternalFrameDescriptor);
                    } else {
                        vMBMasterCBRequestSuccess(&psInternalFrameDescriptor);
                    }
                    break;

                case EV_MASTER_FRAME_SENT: //0x08
                    /* In case we received a new request to send a frame, but have not finished processing the current one, postpone to separate queue, and do nothing more. */
                    if (bMasterIsBusy == TRUE)
                    {
                        ESP_LOGD(TAG, "Master busy, postponing event");
                        (void)vMBMasterPortQueuePostpone(&psInternalFrameDescriptor);
                    } else {
                        /* Otherwise we received a valid new request, process it and set Master to busy now. */
                        ESP_LOGD(TAG, "Master ready, received new event ");
                        bMasterIsBusy = TRUE;
                        //printf("%s:EV_MASTER_FRAME_SENT\r\n", __func__);
                        ucMBFrame = psInternalFrameDescriptor->snd_frame;
                        eStatus = peMBFrameSendCur(psInternalFrameDescriptor->dest_address, ucMBFrame, psInternalFrameDescriptor->pdu_send_length,
                                                   psInternalFrameDescriptor);  // mapped to eMBRTUSend, actually we could pass just the descriptor, but keep it compatible with slave role for now.
                    }
                    break;

                case EV_MASTER_ERROR_PROCESS: //0x10
                    //printf("%s:EV_MASTER_ERROR_PROCESS\r\n", __func__);
                    /* Execute specified error process callback function. */
                    ESP_LOGD(TAG, "Master error processing, error code %d", psInternalFrameDescriptor->error_type);
                    ucMBFrame = psInternalFrameDescriptor->snd_frame+1;

                    // this will post another state change event to myself (modbus master task).
                    switch (psInternalFrameDescriptor->error_type)
                    {
                        case MB_MRE_TIMEDOUT: // 4
                            vMBMasterErrorCBRespondTimeout(&psInternalFrameDescriptor);
                            break;
                        case MB_MRE_REV_DATA: // 3
                            vMBMasterErrorCBReceiveData(&psInternalFrameDescriptor);
                            break;
                        case MB_MRE_EXE_FUN:  // 6
                        default:
                            vMBMasterErrorCBExecuteFunction(&psInternalFrameDescriptor);
                        break;
                    }
                    break;
            } /* switch (eMasterEvent) */
	} /* master event processed */
    } else { /* slave role */

        /* Check if there is a event available. If not return control to caller.
         * Otherwise we will handle the event. */
        if( xMBPortEventGet( &eSlaveEvent ) == TRUE )
        {
            switch ( eSlaveEvent )
            {
            case EV_READY:
                break;

            case EV_FRAME_RECEIVED:
                eStatus = peMBFrameReceiveCur( &ucRcvAddress, &ucMBFrame, &usLength );
                if( eStatus == MB_ENOERR )
                {
                    /* Check if the frame is for us. If not ignore the frame. */
                    if( ( ucRcvAddress == ucMBAddress ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) )
                    {
                        ( void )xMBPortEventPost( EV_EXECUTE );
                    }
                }
                break;

            case EV_EXECUTE:
                ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
                eException = MB_EX_ILLEGAL_FUNCTION;
                for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
                {
                    /* No more function handlers registered. Abort. */
                    if( xFuncHandlers[i].ucFunctionCode == 0 )
                    {
                        break;
                    }
                    else if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                    {
                        eException = xFuncHandlers[i].pxHandler( ucMBFrame, &usLength, psInternalFrameDescriptor );
                        break;
                    }
                }

                /* If the request was not sent to the broadcast address we
                 * return a reply. */
                if( ucRcvAddress != MB_ADDRESS_BROADCAST )
                {
                    if( eException != MB_EX_NONE )
                    {
                        /* An exception occurred. Build an error frame. */
                        usLength = 0;
                        ucMBFrame[usLength++] = ( UCHAR )( ucFunctionCode | MB_FUNC_ERROR );
                        ucMBFrame[usLength++] = eException;
                    }
                    if( ( eMBCurrentMode == MB_ASCII ) && MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS )
                    {
                        vMBPortTimersDelay( MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS );
                    }                
                    eStatus = peMBFrameSendCur( ucMBAddress, ucMBFrame, usLength, initDescriptor );
                }
                break;

            case EV_FRAME_SENT:
                break;
            }
        }
    } /* slave */
    return MB_ENOERR;
}

