/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portevent_m.c v 1.60 2013/08/13 15:07:05 Armink add Master Functions$
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "freertos/semphr.h"

/* ----------------------- Variables ----------------------------------------*/

// TODO these statics need to be instance-local, to enable handling multiple buses simultaneously by separate instances
static xQueueHandle xQueueMasterHdl;
static xQueueHandle xQueueMasterPostponedHdl;

BOOL xMBMasterPortQueueInit(void)
{
    BOOL bStatus = FALSE;
    if (0 != (xQueueMasterHdl = xQueueCreate(MASTER_TASK_QUEUE_SIZE, sizeof(sMBMasterQueueType *))))
    {
        vQueueAddToRegistry(xQueueMasterHdl, "MbMasterPortEventQueue");

        if (0 != (xQueueMasterPostponedHdl = xQueueCreate(MASTER_TASK_QUEUE_SIZE, sizeof(sMBMasterQueueType *))))
        {
            vQueueAddToRegistry(xQueueMasterPostponedHdl, "MbMasterPortPostponedQueue");
            bStatus = TRUE;
        } else {
            vQueueDelete(xQueueMasterHdl);
        }
    }
    return bStatus;
}

void vMBMasterPortQueueClose(void)
{
    if (0 != xQueueMasterHdl)
    {
        vQueueDelete(xQueueMasterHdl);
        xQueueMasterHdl = 0;
    }
}

BOOL xMBMasterPortQueuePost(sMBMasterQueueType **sppMasterQueueEntry)
{
    BOOL bStatus = TRUE;
    if (bMBPortIsWithinException())
    {
        (void)xQueueSendFromISR(xQueueMasterHdl, (const void *)sppMasterQueueEntry, pdFALSE);
    }
    else
    {
        (void)xQueueSend(xQueueMasterHdl, (const void *)sppMasterQueueEntry, pdFALSE);
    }

    return bStatus;
}

void vMBMasterPortQueuePostpone(sMBMasterQueueType **sppMasterQueueEntry)
{
    (void) xQueueSend(xQueueMasterPostponedHdl, (const void *)sppMasterQueueEntry, pdFALSE);
} 

void vMBMasterUnpostponeMasterQueue(void)
{
    sMBMasterQueueType *spMasterQueueEntry;
    while (pdTRUE == xQueueReceive(xQueueMasterPostponedHdl, &spMasterQueueEntry,  0))
    {
        (void)xQueueSendToBack(xQueueMasterHdl, (const void *)&spMasterQueueEntry, pdFALSE);
    }
}

BOOL xMBMasterPortQueueGet(sMBMasterQueueType **sppMasterQueueEntry)
{
    BOOL xEventHappened = FALSE;
    if (pdTRUE == xQueueReceive(xQueueMasterHdl, sppMasterQueueEntry, portTICK_RATE_MS * 10))
    {
        xEventHappened = TRUE;
    }
    return xEventHappened;
}

xQueueHandle xMBMasterPortQueueGetHandle(void)
{
    return xQueueMasterHdl;
}

void vMBMasterErrorCBRespondTimeout(sMBMasterQueueType **sppMasterQueueEntry)
{
    BOOL ret = xMBMasterPortEventPost(EV_MASTER_ERROR_RESPOND_TIMEOUT, sppMasterQueueEntry);
    if (!ret)
        printf("xMBMasterPortEventPost event 'EV_MASTER_ERROR_RESPOND_TIMEOUT' failed!!!\r\n");
}
void vMBMasterCBRequestSuccess(sMBMasterQueueType **sppMasterQueueEntry)
{
    BOOL ret = xMBMasterPortEventPost(EV_MASTER_PROCESS_SUCCESS, sppMasterQueueEntry);
    if (!ret)
        printf("xMBMasterPortEventPost event 'EV_MASTER_PROCESS_SUCCESS' failed!!!\r\n");
}

void vMBMasterErrorCBReceiveData(sMBMasterQueueType **sppMasterQueueEntry)
{
    BOOL ret = xMBMasterPortEventPost(EV_MASTER_ERROR_RECEIVE_DATA, sppMasterQueueEntry);
    if (!ret)
        printf("xMBMasterPortEventPost event 'EV_MASTER_ERROR_RECEIVE_DATA' failed!!!\r\n");
}

void vMBMasterErrorCBExecuteFunction(sMBMasterQueueType **sppMasterQueueEntry)
{
    BOOL ret = xMBMasterPortEventPost(EV_MASTER_ERROR_EXECUTE_FUNCTION, sppMasterQueueEntry);
    if (!ret)
        printf("xMBMasterPortEventPost event 'EV_MASTER_ERROR_EXECUTE_FUNCTION' failed!!!\r\n");
}

