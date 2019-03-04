/*
@file pool.c
@brief Pool buffer handling for deterministic message buffers.

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

#include "pool.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

//#define LOG_POOL_TO_CONSOLE   // direct debug logging to console (not possible in interrupt context) 
//#define DEBUG_TRACE_POOL      // debug tracing (also suitable for tracing alloc/free in interrupt context)

#ifdef LOG_POOL_TO_CONSOLE
#include "esp_log.h"                // for log_write
#endif


/* Macros */
#define FREERTOS_MUTEX                            SemaphoreHandle_t
#define FREERTOS_MUTEX_CREATE                     xSemaphoreCreateMutex
#define FREERTOS_MUTEX_DESTROY(mutex_handle)      vSemaphoreDelete(mutex_handle)

#define POOL_CRIT_SECT_ENTER(lock) do { \
        xSemaphoreTake(lock, portMAX_DELAY); \
} while (0)

#define POOL_CRIT_SECT_LEAVE(lock) do { \
        xSemaphoreGive(lock); \
} while (0)


/* Typedefs */
struct mypool_s {
    FREERTOS_MUTEX p_lock;
    SIMPLEQ_HEAD(, poolbuf_entry) pool_q;
};

/* External variables */

/* Static and global variables */
#ifdef LOG_POOL_TO_CONSOLE
static const char *TAG    = "POOL";
#endif

// TODO we can keep these global, but need to make sure pool keeps track of instances using it - don't initialize twice (if called from multiple instances) and don't clean up if still an instance is using it.
static struct mypool_s mypool;
static int initialized = 0;

/* Forward declarations */

/* Implementation */

/* debugging only */
#ifdef DEBUG_TRACE_POOL

#define MAX_PL_TRACE 500
unsigned int temp_pool_trace_buf1[MAX_PL_TRACE];
unsigned int temp_pool_trace_buf2[MAX_PL_TRACE];
int pool_trace_no = 0;

void pltrace(unsigned int ptr, bool alloced)
{
    if (pool_trace_no < MAX_PL_TRACE)
    {
        temp_pool_trace_buf1[pool_trace_no] = ptr | (alloced==true?0x80000000U:0);
        temp_pool_trace_buf2[pool_trace_no] = (unsigned int)xTaskGetTickCount();
        pool_trace_no++;
    }
}

void pr_pooltrace(void)
{
    bool alloced;
    printf("++++++--> real time trace of pool allocations/deallocations:\n");
    for (int i=0;i<pool_trace_no;i++)
    {
       if (temp_pool_trace_buf1[i] & 0x80000000U) alloced = true; else alloced = false;
       printf("   %s: 0x%X-@-%d \n", (alloced==true?"alloc":"                        free "), temp_pool_trace_buf1[i]&0x7FFFFFFFU, temp_pool_trace_buf2[i]);
    }
}

#endif /* DEBUG_TRACE_POOL */
/* debug end */

void pool_init(void)
{
    SIMPLEQ_INIT(&mypool.pool_q);
    mypool.p_lock = FREERTOS_MUTEX_CREATE();
    initialized++;
}


void pool_cleanup(void)
{
    struct poolbuf_entry *pool_buf;

    if (initialized)
    {
        POOL_CRIT_SECT_ENTER(mypool.p_lock);
        // empty out free queue and return buffer memory to system
        while (!SIMPLEQ_EMPTY(&mypool.pool_q))
        {
            pool_buf = SIMPLEQ_FIRST(&mypool.pool_q);
            if (pool_buf)
            {
                SIMPLEQ_REMOVE_HEAD(&mypool.pool_q, link);
                heap_caps_free(pool_buf);
            }
        }
        POOL_CRIT_SECT_LEAVE(mypool.p_lock);

        FREERTOS_MUTEX_DESTROY(mypool.p_lock);
    }
}

void *palloc(void)
{
    struct poolbuf_entry *pool_buf;

    POOL_CRIT_SECT_ENTER(mypool.p_lock);
    if (!SIMPLEQ_EMPTY(&mypool.pool_q))
    {
        pool_buf = SIMPLEQ_FIRST(&mypool.pool_q);
        if (pool_buf)
        {
            SIMPLEQ_REMOVE_HEAD(&mypool.pool_q, link);
        }
    } else {
        pool_buf = (struct poolbuf_entry *)heap_caps_malloc(sizeof(struct poolbuf_entry), POOL_MEMORY_CAPS);
    }
#ifdef LOG_POOL_TO_CONSOLE
    ESP_LOGI(TAG, "palloc buffer: %8.8X", (uint32_t) pool_buf);
#endif
#ifdef DEBUG_TRACE_POOL
    pltrace((unsigned int)pool_buf, true);
#endif
    POOL_CRIT_SECT_LEAVE(mypool.p_lock);
    return (void *)pool_buf;
}

void pfree(void *pool_buf)
{
    POOL_CRIT_SECT_ENTER(mypool.p_lock);
#ifdef LOG_POOL_TO_CONSOLE
    ESP_LOGI(TAG, "pfree returning buffer %8.8X", (uint32_t) pool_buf);
#endif
#ifdef DEBUG_TRACE_POOL
    pltrace((unsigned int)pool_buf, false);
#endif
    SIMPLEQ_INSERT_TAIL(&mypool.pool_q, (struct poolbuf_entry *)pool_buf ,link);
    POOL_CRIT_SECT_LEAVE(mypool.p_lock);
}

void pfree_fromISR(void *pool_buf)
{
#ifdef DEBUG_TRACE_POOL
    pltrace((unsigned int)pool_buf, false);
#endif
    SIMPLEQ_INSERT_TAIL(&mypool.pool_q, (struct poolbuf_entry *)pool_buf ,link);
}

