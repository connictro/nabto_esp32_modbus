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
 */


/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbconfig.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_REQ_READ_ADDR_OFF            ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_REQ_READ_DISCCNT_OFF         ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_READ_SIZE                ( 4 )
#define MB_PDU_FUNC_READ_DISCCNT_OFF        ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_READ_SIZE_MIN           ( 1 )

/* ----------------------- Static functions ---------------------------------*/
eMBException    prveMBError2Exception( eMBErrorCode eErrorCode );

/* ----------------------- Start implementation -----------------------------*/
#if MB_ASCII_ENABLED > 0 || MB_RTU_ENABLED > 0
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0

/**
 * This function will request read discrete inputs.
 *
 * @param ucSndAddr salve address
 * @param usDiscreteAddr discrete start address
 * @param usNDiscreteIn discrete total number
 * @param lTimeOut timeout (-1 will waiting forever)
 * @param psInternalFrameDescriptor frame and transfer descriptor
 *
 * @return error code
 */
eMBMasterReqErrCode
eMBMasterReqReadDiscreteInputs( UCHAR ucSndAddr, USHORT usDiscreteAddr, USHORT usNDiscreteIn, LONG lTimeOut, sMBMasterQueueType *psInternalFrameDescriptor )
{
    UCHAR                 *ucMBFrame;
    eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;

    if ( psInternalFrameDescriptor == NULL )        return MB_MRE_ILL_ARG;
    ucMBFrame = psInternalFrameDescriptor->snd_frame;
    if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM )    return MB_MRE_ILL_ARG;

    psInternalFrameDescriptor->dest_address    = ucSndAddr;
    ucMBFrame[MB_PDU_FUNC_OFF]                 = MB_FUNC_READ_DISCRETE_INPUTS;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF]        = usDiscreteAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1]    = usDiscreteAddr;
    ucMBFrame[MB_PDU_REQ_READ_DISCCNT_OFF ]    = usNDiscreteIn >> 8;
    ucMBFrame[MB_PDU_REQ_READ_DISCCNT_OFF + 1] = usNDiscreteIn;
    psInternalFrameDescriptor->pdu_send_length = ( MB_PDU_SIZE_MIN + MB_PDU_REQ_READ_SIZE );
    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT, &psInternalFrameDescriptor );
    return eErrStatus;
}

eMBException
eMBMasterFuncReadDiscreteInputs( UCHAR * pucFrame, USHORT * usLen, sMBMasterQueueType *psInternalFrameDescriptor )
{
    USHORT          usRegAddress;
    USHORT          usDiscreteCnt;
    UCHAR           ucNBytes;
    UCHAR          *ucMBFrame;

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    /* If this request is broadcast, and it's read mode. This request don't need execute. */
    if ( MASTER_REQUEST_IS_BROADCAST(psInternalFrameDescriptor) )
    {
        psInternalFrameDescriptor->error_type = MB_ENOERR;           // send a "good" completion message.
        eRegStatus = eMBMasterGenericCB(psInternalFrameDescriptor);
        eStatus = ( eRegStatus != MB_ENOERR ) ? prveMBError2Exception( eRegStatus ) : MB_EX_NONE; // If an error occured convert it into a Modbus exception.
    }
    else if( *usLen >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READ_SIZE_MIN ) // error checks for valid received frame
    {
    	ucMBFrame = psInternalFrameDescriptor->snd_frame;
        usRegAddress = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1] );
        usRegAddress++;

        usDiscreteCnt = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_DISCCNT_OFF] << 8 );
        usDiscreteCnt |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_DISCCNT_OFF + 1] );

        /* Test if the quantity of coils is a multiple of 8. If not last
         * byte is only partially field with unused coils set to zero. */
        if( ( usDiscreteCnt & 0x0007 ) != 0 )
        {
        	ucNBytes = ( UCHAR )( usDiscreteCnt / 8 + 1 );
        }
        else
        {
        	ucNBytes = ( UCHAR )( usDiscreteCnt / 8 );
        }

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception. 
         */
		if ((usDiscreteCnt >= 1) && ucNBytes == pucFrame[MB_PDU_FUNC_READ_DISCCNT_OFF])
        {
            /* Make callback to post to modbus supervisor. */
            psInternalFrameDescriptor->error_type = MB_ENOERR;           // send a "good" completion message.
            eRegStatus = eMBMasterGenericCB(psInternalFrameDescriptor); 

            /* old callback to fill the buffer. 
               eRegStatus = eMBMasterRegDiscreteCB( &pucFrame[MB_PDU_FUNC_READ_VALUES_OFF], usRegAddress, usDiscreteCnt, psInternalFrameDescriptor );
             */

            /* If an error occured convert it into a Modbus exception. */
            if( eRegStatus != MB_ENOERR )
            {
                eStatus = prveMBError2Exception( eRegStatus );
            }
        }
        else
        {
            eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid read coil register request because the length
         * is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif
#endif
