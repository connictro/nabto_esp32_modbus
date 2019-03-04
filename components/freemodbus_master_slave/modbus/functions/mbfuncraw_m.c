/*
@file mbfuncraw_m.c
@brief Request & function execution for raw modbus frames (without interpreting or data checking).

Copyright (c) 2019 Connictro GmbH / Michael Paar

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
/* ----------------------- Modbus includes ----------------------------------*/
#include "mbfunc_common.h"

/* ----------------------- Start implementation -----------------------------*/
#if MB_ASCII_ENABLED > 0 || MB_RTU_ENABLED > 0


#if MB_FUNC_RAW_MODBUS_FRAMES_ENABLED > 0

/**
 * This function will request using a raw modbus frame.
 *
 * @param pucDataBuffer raw modbus frame to be written
 * @param lTimeOut timeout (-1 will waiting forever)
 * @param psInternalFrameDescriptor frame and transfer descriptor
 *
 * @return error code
 */
eMBMasterReqErrCode
eMBMasterReqRawModbusFrames( UCHAR ucSndAddr, UCHAR * pucRawModbusFrame, USHORT usLen,
                LONG lTimeOut, sMBMasterQueueType *psInternalFrameDescriptor )
{
    UCHAR                 *ucMBFrame;
    eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;

    if ( psInternalFrameDescriptor == NULL )        return MB_MRE_ILL_ARG;
    if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM )    return MB_MRE_ILL_ARG;
    ucMBFrame = psInternalFrameDescriptor->snd_frame;

    psInternalFrameDescriptor->dest_address   = ucSndAddr;
    psInternalFrameDescriptor->pdu_send_length = usLen-1;

    /* copy raw frame to internal descriptor */
    memcpy(ucMBFrame, pucRawModbusFrame+1, usLen-1); // minus one because slave address has already been copied

    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT, &psInternalFrameDescriptor );
    return eErrStatus;
}

#endif
#endif

