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
 * File: $Id: mbfunccoils.c,v 1.8 2007/02/18 23:47:16 wolti Exp $
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbfunc_common.h"

/* ----------------------- Defines (MASTER role only) -----------------------*/
#define MB_PDU_REQ_READ_ADDR_OFF            ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_REQ_READ_COILCNT_OFF         ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_READ_SIZE                ( 4 )
#define MB_PDU_FUNC_READ_COILCNT_OFF        ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_READ_SIZE_MIN           ( 1 )

#define MB_PDU_REQ_WRITE_ADDR_OFF           ( MB_PDU_DATA_OFF )
#define MB_PDU_REQ_WRITE_VALUE_OFF          ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_WRITE_SIZE               ( 4 )

#define MB_PDU_REQ_WRITE_MUL_ADDR_OFF       ( MB_PDU_DATA_OFF )
#define MB_PDU_REQ_WRITE_MUL_COILCNT_OFF    ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF    ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_REQ_WRITE_MUL_VALUES_OFF     ( MB_PDU_DATA_OFF + 5 )
#define MB_PDU_REQ_WRITE_MUL_SIZE_MIN       ( 5 )
#define MB_PDU_REQ_WRITE_MUL_COILCNT_MAX    ( 0x07B0 )
#define MB_PDU_FUNC_WRITE_MUL_SIZE          ( 5 )

#if MB_FUNC_READ_COILS_ENABLED > 0

/**
 * This function will request read coil.
 *
 * @param ucSndAddr slave address
 * @param usCoilAddr coil start address
 * @param usNCoils coil total number
 * @param lTimeOut timeout (-1 will waiting forever)
 * @param psInternalFrameDescriptor frame and transfer descriptor
 *
 * @return error code
 */
eMBMasterReqErrCode
eMBMasterReqReadCoils( UCHAR ucSndAddr, USHORT usCoilAddr, USHORT usNCoils, LONG lTimeOut, sMBMasterQueueType *psInternalFrameDescriptor )
{
    UCHAR *ucMBFrame;
    eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;

    if ( psInternalFrameDescriptor == NULL )        return MB_MRE_ILL_ARG;
    ucMBFrame = psInternalFrameDescriptor->snd_frame;
    if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM )    return MB_MRE_ILL_ARG;

    psInternalFrameDescriptor->dest_address    = ucSndAddr;
    ucMBFrame[MB_PDU_FUNC_OFF]                 = MB_FUNC_READ_COILS;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF]        = usCoilAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1]    = usCoilAddr;
    ucMBFrame[MB_PDU_REQ_READ_COILCNT_OFF ]    = usNCoils >> 8;
    ucMBFrame[MB_PDU_REQ_READ_COILCNT_OFF + 1] = usNCoils;
    psInternalFrameDescriptor->pdu_send_length = ( MB_PDU_SIZE_MIN + MB_PDU_REQ_READ_SIZE );

    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT, &psInternalFrameDescriptor );
    return eErrStatus;
}

eMBException
eMBMasterFuncReadCoils( UCHAR * pucFrame, USHORT * usLen, sMBMasterQueueType *psInternalFrameDescriptor )
{
    UCHAR          *ucMBFrame;
    USHORT          usRegAddress;
    USHORT          usCoilCount;
    UCHAR           ucByteCount;

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;


    /* If this request is broadcast, and it's read mode. This request don't need execute. */
    if ( MASTER_REQUEST_IS_BROADCAST(psInternalFrameDescriptor) )
    {
        psInternalFrameDescriptor->error_type = MB_ENOERR;           // send a "good" completion message.
        eRegStatus = eMBMasterGenericCB(psInternalFrameDescriptor);
        eStatus = ( eRegStatus != MB_ENOERR ) ? prveMBError2Exception( eRegStatus ) : MB_EX_NONE; // If an error occured convert it into a Modbus exception.
    }
    else if ( *usLen >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READ_SIZE_MIN ) // error checks for valid received frame
    {
    	ucMBFrame = psInternalFrameDescriptor->snd_frame;
        usRegAddress = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1] );
        usRegAddress++;

        usCoilCount = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_COILCNT_OFF] << 8 );
        usCoilCount |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_COILCNT_OFF + 1] );

        /* Test if the quantity of coils is a multiple of 8. If not last
         * byte is only partially field with unused coils set to zero. */
        if( ( usCoilCount & 0x0007 ) != 0 )
        {
        	ucByteCount = ( UCHAR )( usCoilCount / 8 + 1 );
        }
        else
        {
        	ucByteCount = ( UCHAR )( usCoilCount / 8 );
        }

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception. 
         */
        if( ( usCoilCount >= 1 ) &&
            ( ucByteCount == pucFrame[MB_PDU_FUNC_READ_COILCNT_OFF] ) )
        {
            /* Make callback to post to modbus supervisor. */
            psInternalFrameDescriptor->error_type = MB_ENOERR;           // send a "good" completion message.
            eRegStatus = eMBMasterGenericCB(psInternalFrameDescriptor); 

            // old callback filling buffers
            // eRegStatus = eMBMasterRegCoilsCB( &pucFrame[MB_PDU_FUNC_READ_VALUES_OFF], usRegAddress, usCoilCount, MB_REG_READ, psInternalFrameDescriptor );


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

#if MB_FUNC_WRITE_COIL_ENABLED > 0

/**
 * This function will request write one coil.
 *
 * @param ucSndAddr slave address
 * @param usCoilAddr coil start address
 * @param usCoilData data to be written
 * @param lTimeOut timeout (-1 will waiting forever)
 * @param psInternalFrameDescriptor frame and transfer descriptor
 *
 * @return error code
 *
 * @see eMBMasterReqWriteMultipleCoils
 */

eMBMasterReqErrCode
eMBMasterReqWriteCoil( UCHAR ucSndAddr, USHORT usCoilAddr, USHORT usCoilData, LONG lTimeOut, sMBMasterQueueType *psInternalFrameDescriptor )
{
    UCHAR *ucMBFrame;
    eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;

    if ( psInternalFrameDescriptor == NULL )                    return MB_MRE_ILL_ARG;
    ucMBFrame = psInternalFrameDescriptor->snd_frame;
    if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM )                return MB_MRE_ILL_ARG;
    if ( ( usCoilData != 0xFF00 ) && ( usCoilData != 0x0000 ) ) return MB_MRE_ILL_ARG;

    psInternalFrameDescriptor->dest_address   = ucSndAddr;
    ucMBFrame[MB_PDU_FUNC_OFF]                = MB_FUNC_WRITE_SINGLE_COIL;
    ucMBFrame[MB_PDU_REQ_WRITE_ADDR_OFF]      = usCoilAddr >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_ADDR_OFF + 1]  = usCoilAddr;
    ucMBFrame[MB_PDU_REQ_WRITE_VALUE_OFF ]    = usCoilData >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_VALUE_OFF + 1] = usCoilData;
    psInternalFrameDescriptor->pdu_send_length = ( MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_SIZE );

    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT, &psInternalFrameDescriptor );
    return eErrStatus;
}

eMBException
eMBMasterFuncWriteCoil( UCHAR * pucFrame, USHORT * usLen, sMBMasterQueueType *psInternalFrameDescriptor )
{
    USHORT          usRegAddress;
    //UCHAR           ucBuf[2];

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    if( *usLen == ( MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1] );
        usRegAddress++;

        if( ( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF + 1] == 0x00 ) &&
            ( ( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF ) ||
              ( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0x00 ) ) )
        {
#if 0
            ucBuf[1] = 0;
            if( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF )
            {
                ucBuf[0] = 1;
            }
            else
            {
                ucBuf[0] = 0;
            }
            eRegStatus =
                eMBMasterRegCoilsCB( &ucBuf[0], usRegAddress, 1, MB_REG_WRITE, psInternalFrameDescriptor );
#endif
            /* Make callback to post to modbus supervisor. */
            psInternalFrameDescriptor->error_type = MB_ENOERR;           // send a "good" completion message.
            eRegStatus = eMBMasterGenericCB(psInternalFrameDescriptor); 

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
        /* Can't be a valid write coil register request because the length
         * is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif

#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0

/**
 * This function will request write multiple coils.
 *
 * @param ucSndAddr slave address
 * @param usCoilAddr coil start address
 * @param usNCoils coil total number
 * @param usCoilData data to be written
 * @param lTimeOut timeout (-1 will waiting forever)
 * @param psInternalFrameDescriptor frame and transfer descriptor
 *
 * @return error code
 *
 * @see eMBMasterReqWriteCoil
 */
eMBMasterReqErrCode
eMBMasterReqWriteMultipleCoils( UCHAR ucSndAddr,
		USHORT usCoilAddr, USHORT usNCoils, UCHAR * pucDataBuffer, LONG lTimeOut, sMBMasterQueueType *psInternalFrameDescriptor )
{
    UCHAR *ucMBFrame;
    USHORT                 usRegIndex = 0;
    UCHAR                  ucByteCount;
    eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;

    if ( psInternalFrameDescriptor == NULL )           return MB_MRE_ILL_ARG;
    ucMBFrame = psInternalFrameDescriptor->snd_frame;
    if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM )       return MB_MRE_ILL_ARG;
    if ( usNCoils > MB_PDU_REQ_WRITE_MUL_COILCNT_MAX ) return MB_MRE_ILL_ARG;

    psInternalFrameDescriptor->dest_address         = ucSndAddr;
    ucMBFrame[MB_PDU_FUNC_OFF]                      = MB_FUNC_WRITE_MULTIPLE_COILS;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF]        = usCoilAddr >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF + 1]    = usCoilAddr;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_COILCNT_OFF]     = usNCoils >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_COILCNT_OFF + 1] = usNCoils ;
    if( ( usNCoils & 0x0007 ) != 0 )
    {
        ucByteCount = ( UCHAR )( usNCoils / 8 + 1 );
    }
    else
    {
        ucByteCount = ( UCHAR )( usNCoils / 8 );
    }
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF]     = ucByteCount;
    ucMBFrame += MB_PDU_REQ_WRITE_MUL_VALUES_OFF;
    while( ucByteCount > usRegIndex)
    {
        *ucMBFrame++ = pucDataBuffer[usRegIndex++];
    }
    psInternalFrameDescriptor->pdu_send_length = ( MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_MUL_SIZE_MIN + ucByteCount );

    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT, &psInternalFrameDescriptor );
    return eErrStatus;
}

eMBException
eMBMasterFuncWriteMultipleCoils( UCHAR * pucFrame, USHORT * usLen, sMBMasterQueueType *psInternalFrameDescriptor )
{
    USHORT          usRegAddress;
    USHORT          usCoilCnt;
    UCHAR           ucByteCount;
    UCHAR           ucByteCountVerify;
    UCHAR          *ucMBFrame;

    eMBException    eStatus = MB_EX_NONE;
    eMBErrorCode    eRegStatus;

    /* If this request is broadcast, the *usLen is not need check. */
    if( ( *usLen == MB_PDU_FUNC_WRITE_MUL_SIZE ) || MASTER_REQUEST_IS_BROADCAST(psInternalFrameDescriptor) )
    {
    	ucMBFrame = psInternalFrameDescriptor->snd_frame;
        usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1] );
        usRegAddress++;

        usCoilCnt = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF] << 8 );
        usCoilCnt |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF + 1] );

        ucByteCount = ucMBFrame[MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF];

        /* Compute the number of expected bytes in the request. */
        if( ( usCoilCnt & 0x0007 ) != 0 )
        {
            ucByteCountVerify = ( UCHAR )( usCoilCnt / 8 + 1 );
        }
        else
        {
            ucByteCountVerify = ( UCHAR )( usCoilCnt / 8 );
        }

        if( ( usCoilCnt >= 1 ) && ( ucByteCountVerify == ucByteCount ) )
        {
            /* Make callback to post to modbus supervisor. */
            psInternalFrameDescriptor->error_type = MB_ENOERR;           // send a "good" completion message.
            eRegStatus = eMBMasterGenericCB(psInternalFrameDescriptor); 

            // old callback for buffer fill, not needed anymore
            /* eRegStatus =
                eMBMasterRegCoilsCB( &ucMBFrame[MB_PDU_REQ_WRITE_MUL_VALUES_OFF],
                               usRegAddress, usCoilCnt, MB_REG_WRITE, psInternalFrameDescriptor ); */

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
        /* Can't be a valid write coil register request because the length
         * is incorrect. */
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif


