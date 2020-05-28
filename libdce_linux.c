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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>

#include <xf86drm.h>
#include <omap_drm.h>
#include <omap_drmif.h>

#include <ti/ipc/mm/MmRpc.h>
#include "dce_priv.h"
#include "libdce.h"
#include "dce_rpc.h"
#include "memplugin.h"

#define INVALID_DRM_FD (-1)

static int             OmapDrm_FD  = INVALID_DRM_FD;
static int             dce_init_count = 0;
struct omap_device    *OmapDev     = NULL;
extern MmRpc_Handle    MmRpcHandle[];
extern pthread_mutex_t    ipc_mutex;

void *dce_init(void)
{
    dce_error_status    eError = DCE_EOK;
    pthread_mutexattr_t attr;

    DEBUG(" >> dce_init");

    /* Use this for refcount */
    dce_init_count++;

   /* Open omapdrm device only for the first dce_init call */
    if( dce_init_count == 1 ) {
        DEBUG("Open omapdrm device and initializing the mutex...");
        OmapDrm_FD = drmOpenWithType("omapdrm", NULL, DRM_NODE_RENDER);
        _ASSERT(OmapDrm_FD > 0, DCE_EOMAPDRM_FAIL);

        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
        pthread_mutex_init(&ipc_mutex, &attr);
    }

    OmapDev = omap_device_new(OmapDrm_FD);
    _ASSERT(OmapDev != NULL, DCE_EOMAPDRM_FAIL);

EXIT:
    return ((void *)OmapDev);
}

void dce_deinit(void *dev)
{
    omap_device_del(dev);
    dev = NULL;
    if (--dce_init_count == 0) {
        DEBUG("Closing omapdrm device...");
        close(OmapDrm_FD);
        OmapDrm_FD = INVALID_DRM_FD;
    }
    return;
}

int dce_buf_lock(int num, size_t *handle)
{
    int                 i;
    MmRpc_BufDesc      *desc = NULL;
    dce_error_status    eError = DCE_EOK;

    pthread_mutex_lock(&ipc_mutex);

    _ASSERT(num > 0, DCE_EINVALID_INPUT);

    desc = malloc(num * sizeof(MmRpc_BufDesc));
    _ASSERT(desc != NULL, DCE_EOUT_OF_MEMORY);

    for( i = 0; i < num; i++ ) {
        desc[i].handle = handle[i];
    }

    eError = MmRpc_use(MmRpcHandle[IPU], MmRpc_BufType_Handle, num, desc);

    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);
EXIT:
    if( desc ) {
        free(desc);
    }

    pthread_mutex_unlock(&ipc_mutex);

    return (eError);
}

int dce_buf_unlock(int num, size_t *handle)
{
    int                 i;
    MmRpc_BufDesc      *desc = NULL;
    dce_error_status    eError = DCE_EOK;

    pthread_mutex_lock(&ipc_mutex);

    _ASSERT(num > 0, DCE_EINVALID_INPUT);

    desc = malloc(num * sizeof(MmRpc_BufDesc));
    _ASSERT(desc != NULL, DCE_EOUT_OF_MEMORY);

    for( i = 0; i < num; i++ ) {
        desc[i].handle = handle[i];
    }

    eError = MmRpc_release(MmRpcHandle[IPU], MmRpc_BufType_Handle, num, desc);

    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);
EXIT:
    if( desc ) {
        free(desc);
    }

    pthread_mutex_unlock(&ipc_mutex);

    return (eError);
}

/*Memory Management mirror APIs for DSP remoteproc targets*/
void *dsp_dce_alloc(int sz)
{
    /*
      Beware: The last argument is a bit field. As of now only core ID
      is considered to be there in the last 4 bits of the word.
    */
    return (memplugin_alloc(sz, 1, MEM_TILER_1D, 0, DSP));
}

void dsp_dce_free(void *ptr)
{
    memplugin_free(ptr);
}


int dsp_dce_buf_lock(int num, size_t *handle)
{
    int                 i;
    MmRpc_BufDesc      *desc = NULL;
    dce_error_status    eError = DCE_EOK;

    pthread_mutex_lock(&ipc_mutex);

    _ASSERT(num > 0, DCE_EINVALID_INPUT);

    desc = malloc(num * sizeof(MmRpc_BufDesc));
    _ASSERT(desc != NULL, DCE_EOUT_OF_MEMORY);

    for( i = 0; i < num; i++ ) {
        desc[i].handle = handle[i];
    }

    eError = MmRpc_use(MmRpcHandle[DSP], MmRpc_BufType_Handle, num, desc);

    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);
EXIT:
    if( desc ) {
        free(desc);
    }

    pthread_mutex_unlock(&ipc_mutex);

    return (eError);
}

int dsp_dce_buf_unlock(int num, size_t *handle)
{
    int                 i;
    MmRpc_BufDesc      *desc = NULL;
    dce_error_status    eError = DCE_EOK;

    pthread_mutex_lock(&ipc_mutex);

    _ASSERT(num > 0, DCE_EINVALID_INPUT);

    desc = malloc(num * sizeof(MmRpc_BufDesc));
    _ASSERT(desc != NULL, DCE_EOUT_OF_MEMORY);

    for( i = 0; i < num; i++ ) {
        desc[i].handle = handle[i];
    }

    eError = MmRpc_release(MmRpcHandle[DSP], MmRpc_BufType_Handle, num, desc);

    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);
EXIT:
    if( desc ) {
        free(desc);
    }

    pthread_mutex_unlock(&ipc_mutex);
    return (eError);
}
/* Incase of X11 or Wayland the fd can be shared to libdce using this call */
void dce_set_fd(int dce_fd)
{
    if (OmapDrm_FD == INVALID_DRM_FD) {
        OmapDrm_FD = dce_fd;
    }
    else {
        DEBUG("Cannot set the fd, omapdrm device fd is already set");
    }
}

int dce_get_fd(void)
{
    return (OmapDrm_FD);
}

