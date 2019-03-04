/*
@file pool.h
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

#ifndef __POOL_H__
#define __POOL_H__

#include "queue_berkeley.h"

/* Configure pool entry size and memory type here. */
#define POOL_PAYLOAD_SIZE 532                /* Adapt size here for your application */
#define POOL_MEMORY_CAPS  MALLOC_CAP_8BIT    /* Change to MALLOC_CAP_DMA if you need DMA-capable buffers */

struct poolbuf_entry{
    char buf[POOL_PAYLOAD_SIZE];
    SIMPLEQ_ENTRY(poolbuf_entry) link;
} __attribute__((packed));

#define POOL_SIZE sizeof (struct poolbuf_entry)


/*
 * pool.h / pool.c implements a very simple thread-safe, deterministic pool management.
 * It has only one chunk size.
 * Memory chunks of this fixed size can be allocated and freed.
 * To be fully deterministic, pool buffers can be pre-allocated and freed so malloc()
 * won't kick in during productive operation anymore.
 *
 * Internally it uses a free queue, if this is empty a libc malloc() is used to allocate
 * a new pool buffer. If not (already recycled buffers) these from the queue are used.
 * "pfree" actually does not return a buffer to the system but only to the free queue.
 * Buffers are only finally freed upon pool_cleanup() which is normally done only at program exit.
 */

/* Allocate fixed-size element of POOL_SIZE from pool */
void *palloc(void);

/* Return fixed-size  element of POOL_SIZE to pool*/
void pfree(void *buf);
/* Same, but can be called only from an ISR w/o type check. */
void pfree_fromISR(void *buf);

/* Init and cleanup of pool management. Safe to call cleanup even if it was never initialized. */
void pool_init(void);
void pool_cleanup(void);       
#ifdef MIPA_DEBUG_POOL
void pr_pooltrace(void);
#endif

#endif /* __POOL_H__ */
