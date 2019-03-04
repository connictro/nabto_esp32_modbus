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
 * File: $Id: mbport.h,v 1.19 2010/06/06 13:54:40 wolti Exp $
 */

#ifndef _MB_PORT_H
#define _MB_PORT_H

#include "mbtypes.h"
//#include "mbconfig.h"    // included in master port, check if needed

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

#define MASTER_TASK_QUEUE_SIZE    10   // maximum elements in  event queue to/from master modbus task

/* ----------------------- Type definitions ---------------------------------*/

typedef enum
{
    EV_READY,                   /*!< Startup finished. */
    EV_FRAME_RECEIVED,          /*!< Frame received. */
    EV_EXECUTE,                 /*!< Execute function. */
    EV_FRAME_SENT               /*!< Frame sent. */
} eMBEventType;

/*! \ingroup modbus
 *  \brief Master modbus task states
 */
typedef enum {
    EV_MASTER_READY                  = 1 << 0, /*!< Startup finished. 1*/
    EV_MASTER_FRAME_RECEIVED         = 1 << 1, /*!< Frame received. 2*/
    EV_MASTER_EXECUTE                = 1 << 2, /*!< Execute function. 4*/
    EV_MASTER_FRAME_SENT             = 1 << 3, /*!< Frame sent. 8*/
    EV_MASTER_ERROR_PROCESS          = 1 << 4, /*!< Frame error process. 10*/
    EV_MASTER_PROCESS_SUCCESS        = 1 << 5, /*!< Request process success. 20*/
    EV_MASTER_ERROR_RESPOND_TIMEOUT  = 1 << 6, /*!< Request respond timeout. 40*/
    EV_MASTER_ERROR_RECEIVE_DATA     = 1 << 7, /*!< Request receive data error. 80*/
    EV_MASTER_ERROR_EXECUTE_FUNCTION = 1 << 8, /*!< Request execute function error. 100*/
    EV_MASTER_INIT                   = 1 << 9, /*!< Initializing, not accepting any requests until EV_MASTER_READY arrives. 200*/
} eMBMasterEventType;

/*! \ingroup modbus
 *  \brief Internal message passing descriptor for sent/received frames from/to modbus task and serial interface.  
 *         In the queues we post pointers to this structure. It needs to be allocated from the pool
 *         (not malloc, to be deterministic).
 *         We also use it in slave role, but initialize it just statically at eMBInit, avoiding global variables.
 */
typedef struct mb_master_event_message {
    uint32_t                request_id;                     /*!< Master unique internal request ID. */
    eMBMasterEventType      event_type;                     /*!< Master modbus task event / next state. */
    eMBMasterReqErrCode     error_type;                     /*!< Error code in case of master errors is stored here. */
    UCHAR                   dest_address;                   /*!< destination slave address (or 0 for broadcast), this is part of the raw frame transmitted to serial line. */
    UCHAR                   snd_frame[MB_SER_PDU_SIZE_MAX]; /*!< raw frame transmitted to the serial line */
    UCHAR                   rcv_frame[MB_SER_PDU_SIZE_MAX]; /*!< raw frame received from the serial line */
    UCHAR                   pdu_send_length;                /*!< send length of frame. */
    USHORT                  pdu_recv_length;                /*!< length of frame received. */
} __attribute__((packed)) sMBMasterQueueType;


#define USE_ENABLE_PIN_RS485   ( FALSE )    /* Set to TRUE if external RS485 transceiver needs dedicated enable signal (using RTS signal of serial line). */

/* ----------------------- Supporting functions -----------------------------*/
BOOL            xMBPortEventInit( void );

BOOL            xMBPortEventPost( eMBEventType eEvent );

BOOL            xMBPortEventGet(  /*@out@ */ eMBEventType * eEvent );

BOOL            xMBMasterPortQueueInit(void);

static BOOL     xMBMasterPortEventPost(eMBMasterEventType, sMBMasterQueueType **sppMasterQueueEntry);

BOOL            xMBMasterPortQueuePost(sMBMasterQueueType **sppMasterQueueEntry);

BOOL            xMBMasterPortQueueGet(/*@out@ */ sMBMasterQueueType **sppMasterQueueEntry);

void            vMBMasterPortQueuePostpone(sMBMasterQueueType **sppMasterQueueEntry);

void            vMBMasterUnpostponeMasterQueue(void);

/* ----------------------- Serial port functions ----------------------------*/

BOOL            xMBPortSerialInit_wCtrlPin( UCHAR ucPort, ULONG ulBaudRate,
                                   UCHAR ucDataBits, eMBParity eParity,
                                   BOOL  controlEnPin );

static BOOL     xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate,
                                   UCHAR ucDataBits, eMBParity eParity );

void            vMBPortClose( void );

void            xMBPortSerialClose( void );

void            vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable );

BOOL            xMBPortSerialGetByte( CHAR * pucByte );

BOOL            xMBPortSerialPutByte( CHAR ucByte );

BOOL            xMBPortSerialPutBytes(CHAR *ucBytes, UCHAR len);


/* ----------------------- Timers functions ---------------------------------*/
BOOL            xMBPortTimersInit( USHORT usTimeOut50us, BOOL is_master );

void            xMBPortTimersClose( void );

void            vMBPortTimersEnable( void );

void            vMBPortTimersDisable( void );

void            vMBPortTimersDelay( USHORT usTimeOutMS );

void            vMBMasterPortTimersT35Enable(void);

void            vMBMasterPortTimersConvertDelayEnable(void);

void            vMBMasterPortTimersRespondTimeoutEnable(void);

/* ----------------- Callbacks for the master error process -----------------*/
void            vMBMasterErrorCBRespondTimeout(sMBMasterQueueType **sppMasterQueueEntry);

void            vMBMasterErrorCBReceiveData(sMBMasterQueueType **sppMasterQueueEntry);

void            vMBMasterErrorCBExecuteFunction(sMBMasterQueueType **sppMasterQueueEntry);

void            vMBMasterCBRequestSuccess(sMBMasterQueueType **sppMasterQueueEntry);

/* ----------------------- Callback for the protocol stack ------------------*/
/*!
 * \brief Callback function for the porting layer when a new byte is
 *   available.
 *
 * Depending upon the mode this callback function is used by the RTU or
 * ASCII transmission layers. In any case a call to xMBPortSerialGetByte()
 * must immediately return a new character.
 * Both Master and Slave roles are supported by the callbacks.
 *
 * \return <code>TRUE</code> if a event was posted to the queue because
 *   a new byte was received. The port implementation should wake up the
 *   tasks which are currently blocked on the eventqueue.
 */
extern          BOOL( *pxMBFrameCBByteReceived ) ( void );

extern          BOOL( *pxMBFrameCBTransmitterEmpty ) ( void );

extern          BOOL( *pxMBPortCBTimerExpired ) ( void );

/* ----------------------- TCP port functions -------------------------------*/
#if MB_TCP_ENABLED == 1
BOOL            xMBTCPPortInit( USHORT usTCPPort );

void            vMBTCPPortClose( void );

void            vMBTCPPortDisable( void );

BOOL            xMBTCPPortGetRequest( UCHAR **ppucMBTCPFrame, USHORT * usTCPLength );

BOOL            xMBTCPPortSendResponse( const UCHAR *pucMBTCPFrame, USHORT usTCPLength );
#endif

/* ------------Inline implementations (master functions remapping)-----------*/

static inline BOOL xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity)
{
    return xMBPortSerialInit_wCtrlPin(ucPort, ulBaudRate, ucDataBits, eParity, USE_ENABLE_PIN_RS485);
}

static inline BOOL xMBMasterPortEventPost(eMBMasterEventType eEvent, sMBMasterQueueType **sppMasterQueueEntry) 
{
    (*sppMasterQueueEntry)->event_type = eEvent; 
    return xMBMasterPortQueuePost(sppMasterQueueEntry);
}


/* MB_ADDRESS_BROADCAST */

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
