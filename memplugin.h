/*
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MEMPLUGIN_H__
#define __MEMPLUGIN_H__

#include <stdlib.h>
#include <string.h>
#include <stdio.h>


#if defined(BUILDOS_LINUX)
#include <xf86drm.h>
#include <omap_drm.h>
#include <omap_drmif.h>
#define DEFAULT_REGION MEM_TILER_1D

#elif defined(BUILDOS_QNX)
/* IPC Headers */
#include <tilermem.h>
#include <SharedMemoryAllocatorUsr.h>
#include <memmgr.h>
#define DEFAULT_REGION MEM_SHARED

#elif defined (BUILDOS_ANDROID)
#define DEFAULT_REGION MEM_CARVEOUT
#endif

#define P2H(p) (&(((MemHeader *)(p))[-1]))
#define H2P(h) ((void *)&(h)[1])

#define GetSz(buf)           ((P2H(buf))->size + sizeof(MemHeader))

/* MemHeader is important because it is necessary to know the           */
/* size of the parameter buffers on IPU for Cache operations               */
/* The size can't be assumed as codec supports different inputs           */
/* For ex: static params can be VIDDEC3_Params, IVIDDEC3_Params */
/* or IH264DEC_Params                                                                   */
typedef struct MemHeader {
    uint32_t size;
    void    *ptr;       /* vptr/handle for mpu access */
    int32_t dma_buf_fd; /* shared dma buf fd */
    uint32_t region;    /* mem region the buffer allocated from */
    /* internal meta data for the buffer */
    uint32_t offset;    /* offset for the actual data with in the buffer */
    int32_t map_fd;     /* mmapped fd */
    void * handle;      /*custom handle for the HLOS memallocator*/
    int flags; /*memory attributes*/
} MemHeader;

typedef enum MemoryRegion {
    MEM_TILER_1D,
    MEM_TILER8_2D,
    MEM_TILER16_2D,
    MEM_CARVEOUT,
    MEM_SHARED,
    MEM_GRALLOC,
    MEM_MAX
} MemRegion;

typedef enum core_type {
    INVALID_CORE = -1,
    IPU = 0,
    DSP = 1,
    MAX_REMOTEDEVICES
}core_type;

/* DCE Error Types */
typedef enum mem_error_status {
    MEM_EOK = 0,
    MEM_EINVALID_INPUT = -1,
    MEM_EOUT_OF_TILER_MEMORY = -2,
    MEM_EOUT_OF_SHMEMORY = -3,
    MEM_EOUT_OF_SYSTEM_MEMORY = -4,
    MEM_EOPEN_FAILURE = -5,
} mem_error_status;

void *memplugin_alloc(int sz, int height, MemRegion region, int align, int flags);
void memplugin_free(void *ptr);
int32_t memplugin_share(void *ptr);

#ifdef BUILDOS_ANDROID
typedef enum BufAccessMode {
    MemAccess_8Bit,
    MemAccess_16Bit,
    MemAccess_32Bit
}BufAccessMode;

typedef struct Mem_2DParams {
    uint32_t nHeight;
    uint32_t nWidth;
    uint32_t nStride;
    BufAccessMode eAccessMode;
}Mem_2DParams;

void *memplugin_alloc_noheader(MemHeader *memHdr, int sz, int height, MemRegion region, int align, int flags);
void memplugin_free_noheader(MemHeader *memHdr);

int memplugin_open();
int memplugin_close();
#endif

#endif /* __MEMPLUGIN_H__ */

