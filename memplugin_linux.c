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

#include "memplugin.h"
#include "dce_priv.h"
#include "libdce.h"

extern int OmapDrm_FD;

struct buffer_handle {
    __u32 handle;
    int   dmaFd;
};

/*  memplugin_alloc - allocates omap_bo buffer with a header above it.
 *  @sz: Size of the buffer requsted
 *  @height: this parameter is currently not used
 *  @region :Used to specify the region from where the memory is allocated->Not used.
 *  @align:Alignment. Not used
 *  @flags: Bit field. As of now, only the least significant 4 bits are considered
 *          to identify the core for which this allocation is needed. This information
 *          is needed to use the right tiler pin/unpin APIs (DSP or IPU).
 *          For future extensibility, many more attributes can be added as bit fields.
 *  Returns a virtual address pointer to omap_bo buffer or the param buffer
 */
void *memplugin_alloc(int sz, int height, MemRegion region, int align, int flags)
{
    MemHeader        *h;

    struct drm_mode_create_dumb creq = {
        .height = sz + sizeof(MemHeader),
        .width = 1,
        .bpp = 8,
    };
    if (drmIoctl(OmapDrm_FD, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0) {
        return (NULL);
    }

    struct drm_mode_map_dumb mreq = {
        .handle = creq.handle,
    };
    if (drmIoctl(OmapDrm_FD, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0) {
        struct drm_mode_destroy_dumb dreq = {
            .handle = creq.handle,
        };
        drmIoctl(OmapDrm_FD, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
        return (NULL);
    }

    h = mmap(0, creq.size, PROT_READ | PROT_WRITE, MAP_SHARED, OmapDrm_FD, mreq.offset);
    if (h == MAP_FAILED) {
        struct drm_mode_destroy_dumb dreq = {
            .handle = creq.handle,
        };
        drmIoctl(OmapDrm_FD, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
        return (NULL);
    }

    struct drm_prime_handle req = {
        .handle = creq.handle,
        .flags = DRM_CLOEXEC,
    };
    if (drmIoctl(OmapDrm_FD, DRM_IOCTL_PRIME_HANDLE_TO_FD, &req) < 0) {
        struct drm_mode_destroy_dumb dreq = {
            .handle = creq.handle,
        };
        drmIoctl(OmapDrm_FD, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
        return (NULL);
    }

    memset(H2P(h), 0, sz);
    h->size = sz;
    h->ptr = calloc(sizeof(struct buffer_handle), 1);
    struct buffer_handle *bh = h->ptr;
    bh->handle = creq.handle;
    bh->dmaFd = req.fd;
    /* get the fd from drm which needs to be closed by memplugin_free */
    h->dma_buf_fd = dup(req.fd);
    h->region = region;
    h->flags = flags;/*Beware: This is a bit field.*/
    /* lock the file descriptor */
    if((flags & 0x0f) == DSP) /*Only the last 4 bits are considered*/
        dsp_dce_buf_lock(1, &(h->dma_buf_fd));
    else
        dce_buf_lock(1, &(h->dma_buf_fd));

    return (H2P(h));
}

/*
 * @ptr: pointer to omap_bo buffer, to be freed
 * @memory_type: Currently dce_free is called on parameter buffers only
 */
void memplugin_free(void *ptr)
{
    if( ptr ) {
        MemHeader   *h = P2H(ptr);
        if( h->dma_buf_fd ) {
            /*
            Identify the core for which this memory was allocated and
            use the appropriate API. Last 4 bits of flags are assumed
            to be containing core Id information.
            */
            if((h->flags & 0x0f) == DSP)
                dsp_dce_buf_unlock(1, &(h->dma_buf_fd));
            else
                dce_buf_unlock(1, &(h->dma_buf_fd));
            /* close the file descriptor */
            close(h->dma_buf_fd);
        }
        /*Finally, Delete the buffer object*/
        struct buffer_handle *bh = h->ptr;
        struct drm_mode_destroy_dumb dreq = {
            .handle = bh->handle,
        };
        munmap(h, h->size + sizeof(MemHeader));
        close(bh->dmaFd);
        free(bh);
        drmIoctl(OmapDrm_FD, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
    }
}

/* memplugin_share - converts the omap_bo buffer into dmabuf
 * @ptr : pointer of omap_bo buffer, to be converted to fd
 * Returns a file discriptor for the omap_bo buffer
 */
int32_t memplugin_share(void *ptr)
{
    if( ptr ) {
        MemHeader   *h = P2H(ptr);
        if( h->dma_buf_fd )
             return (h->dma_buf_fd);
    }
    return (-1);
}
