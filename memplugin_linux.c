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

extern struct omap_device   *OmapDev;


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
    struct omap_bo   *bo = omap_bo_new(OmapDev, sz + sizeof(MemHeader), OMAP_BO_WC);

    if( !bo ) {
        return (NULL);
    }

    h = omap_bo_map(bo);
    memset(H2P(h), 0, sz);
    h->size = sz;
    h->ptr = (void *)bo;
    /* get the fd from drm which needs to be closed by memplugin_free */
    h->dma_buf_fd = omap_bo_dmabuf(bo);
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
        omap_bo_del((struct omap_bo *)h->ptr);
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
