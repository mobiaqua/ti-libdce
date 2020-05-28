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
#include <errno.h>
#include <semaphore.h>

/* IPC Headers */
#include <ti/ipc/mm/MmRpc.h>

/* DCE Headers */
#include "libdce.h"
#include "dce_rpc.h"
#include "dce_priv.h"
#include "memplugin.h"

/***************** GLOBALS ***************************/
/* Handle used for Remote Communication              */
MmRpc_Handle    MmRpcHandle[MAX_REMOTEDEVICES] = { NULL};
Engine_Handle   gEngineHandle[MAX_INSTANCES][MAX_REMOTEDEVICES] = { {NULL, NULL}};
MmRpc_Handle    MmRpcCallbackHandle = NULL;
static int MmRpcCallback_count = 0;

#ifdef BUILDOS_LINUX
pthread_mutex_t    ipc_mutex;
pthread_mutex_t    dce_callback_mutex;
#else
pthread_mutex_t    ipc_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t    dce_callback_mutex  = PTHREAD_MUTEX_INITIALIZER;
#endif

static int      __ClientCount[MAX_REMOTEDEVICES] = {0};
int             dce_debug = DCE_DEBUG_LEVEL;
const String DCE_DEVICE_NAME[MAX_REMOTEDEVICES]= {"rpmsg-dce","rpmsg-dce-dsp"};
const String DCE_CALLBACK_NAME = "dce-callback";

typedef struct {
    XDAS_UInt32 codec_handle;
    XDM_DataSyncHandle local_dataSyncHandle;
    XDM_DataSyncDesc *local_dataSyncDesc;
    XDAS_Int32 getDataFlag;
    XDM_DataSyncGetFxn (*local_get_DataFxn) (XDM_DataSyncHandle, XDM_DataSyncDesc*);
    XDAS_Int32 putDataFlag;
    XDM_DataSyncGetFxn (*local_put_DataFxn) (XDM_DataSyncHandle, XDM_DataSyncDesc*);
    XDAS_Int32 getBufferFlag;
    pthread_t getDataFxn_thread;
    pthread_t putDataFxn_thread;
    sem_t sem_dec_row_mode;
    sem_t sem_enc_row_mode;
    int row_mode;
    int first_control;
    int receive_numBlocks;
    int total_numBlocks;
} CallbackFlag;

static CallbackFlag callbackmsg[MAX_INSTANCES];

/***************** INLINE FUNCTIONS ******************/
static inline int get_callback(Uint32 codec)
{
    int i;
    for (i = 0; i < MAX_INSTANCES; i++) {
        if( callbackmsg[i].codec_handle == codec ) {
            return i;
        }
    }
    return -1;
}

static inline int update_clients_table(Engine_Handle engine, int core)
{
    int i;
    for(i = 0; i < MAX_INSTANCES; i++)
    {
        if( gEngineHandle[i][core] == 0 ) {
            gEngineHandle[i][core] = engine;
            return i;
        }
    }
    return -1;
}

static inline void Fill_MmRpc_fxnCtx(MmRpc_FxnCtx *fxnCtx, int fxn_id, int num_params, int num_xlts, MmRpc_Xlt *xltAry)
{
    fxnCtx->fxn_id = fxn_id;
    fxnCtx->num_params = num_params;
    fxnCtx->num_xlts = num_xlts;
    fxnCtx->xltAry = xltAry;
}

static inline void Fill_MmRpc_fxnCtx_OffPtr_Params(MmRpc_Param *mmrpc_params, int size, void *base, int offset, size_t handle)
{
    mmrpc_params->type = MmRpc_ParamType_OffPtr;
    mmrpc_params->param.offPtr.size = (size_t)size;
    mmrpc_params->param.offPtr.base = (size_t)base;
    mmrpc_params->param.offPtr.offset = (size_t)offset;
    mmrpc_params->param.offPtr.handle = handle;
}

static inline void Fill_MmRpc_fxnCtx_Ptr_Params(MmRpc_Param *mmrpc_params, int size, void *addr, size_t handle)
{
    mmrpc_params->type = MmRpc_ParamType_Ptr;
    mmrpc_params->param.ptr.size = size;
    mmrpc_params->param.ptr.addr = (size_t)addr;
    mmrpc_params->param.ptr.handle = handle;
}

static inline void Fill_MmRpc_fxnCtx_Scalar_Params(MmRpc_Param *mmrpc_params, int size, int data)
{
    mmrpc_params->type = MmRpc_ParamType_Scalar;
    mmrpc_params->param.scalar.size = size;
    mmrpc_params->param.scalar.data = (size_t)data;
}

static inline void Fill_MmRpc_fxnCtx_Xlt_Array(MmRpc_Xlt *mmrpc_xlt, int index, int32_t offset, size_t base, size_t handle)
{
    /* index : index of params filled in FxnCtx       */
    /* offset : calculated from address of index      */
    mmrpc_xlt->index = index;
    mmrpc_xlt->offset = offset;
    mmrpc_xlt->base = base;
    mmrpc_xlt->handle = handle;
}

static int __inline getCoreIndexFromName(String name)
{
    if( name == NULL )
         return INVALID_CORE;

    if( !strcmp(name,"ivahd_vidsvr") )
         return IPU;
    else if( !strcmp(name, "dsp_vidsvr") )
         return DSP;
    else
         return INVALID_CORE;
}

static int __inline getCoreIndexFromCodec(int codec_id)
{
    if( codec_id == OMAP_DCE_VIDENC2 || codec_id == OMAP_DCE_VIDDEC3 )
         return IPU;
    else if( codec_id == OMAP_DCE_VIDDEC2 )
         return DSP;
    else
         return INVALID_CORE;
}

static int __inline getCoreIndexFromEngine(Engine_Handle engine, int *tabIdx)
{
    int i;
    int core = INVALID_CORE;

    *tabIdx = -1;

    for(i = 0; i < MAX_INSTANCES ; i++) {
        if( engine == gEngineHandle[i][IPU] ) {
            core = IPU;
            *tabIdx = i;
            break;
        }
        else if( engine == gEngineHandle[i][DSP] ) {
            *tabIdx = i;
            core = DSP;
            break;
        }
     }
     return core;
}

/***************** FUNCTIONS ********************************************/
/* Interface for QNX/Linux for parameter buffer allocation                    */
/* These interfaces are implemented to maintain Backward Compatability  */
void *dce_alloc(int sz)
{
    /*
      Beware: The last argument is a bit field. As of now only core ID
      is considered to be there in the last 4 bits of the word
    */
    return (memplugin_alloc(sz, 1, DEFAULT_REGION, 0, IPU));
}

void dce_free(void *ptr)
{
    memplugin_free(ptr);
}

/* dce_callback_putDataFxn is a callback function that runs on different Thread id. */
/* It is an infinite loop notifying client when partial output data is available when outputDataMode = IVIDEO_ROWMODE. */
int dce_callback_putDataFxn(void *codec)
{
    MmRpc_FxnCtx        fxnCtx;
    int32_t             fxnRet;
    int32_t             return_callback;
    dce_error_status    eError = DCE_EOK;
    XDM_DataSyncHandle  *codec_handle = (XDM_DataSyncHandle *) codec;
    int                 id;

    DEBUG("======================START======================== codec_handle 0x%x", (unsigned int) *codec_handle);

    id = get_callback((Uint32) *codec_handle);
    if( id < 0 ) {
        ERROR("Cannot find the entry in callbackmsg");
    } else {
        (callbackmsg[id]).local_dataSyncHandle = *codec_handle;
        /* This is called from VIDDEC3_process, so we can start telling client to get the output data and provide the numBlocks info. */
        /* Call the callback function specified in the dynParams->putDataFxn */
        while (1) {
            if( (callbackmsg[id]).putDataFlag == 1 ) {
                DEBUG("lock (callbackmsg[%d]).putDataFxn_thread 0x%x (callbackmsg[%d]).local_dataSyncHandle 0x%x",
                    id, (callbackmsg[id]).putDataFxn_thread, id, (unsigned int) (callbackmsg[id]).local_dataSyncHandle);
                pthread_mutex_lock(&dce_callback_mutex);
                if( (callbackmsg[id]).row_mode ) {
                    /* Marshall function arguments into the send callback information to codec for put_DataFxn */
                    Fill_MmRpc_fxnCtx(&fxnCtx, DCE_CALLBACK_RPC_PUT_DATAFXN, 2, 0, NULL);
                    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[0]), sizeof(int32_t), (int32_t) (callbackmsg[id]).local_dataSyncHandle);
                    Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[1]), GetSz((callbackmsg[id]).local_dataSyncDesc), (void *) P2H((callbackmsg[id]).local_dataSyncDesc),
                                                    sizeof(MemHeader), memplugin_share((callbackmsg[id]).local_dataSyncDesc));

                    DEBUG("Calling MmRpc_call on MmRpcCallbackHandle %p", MmRpcCallbackHandle);
                    eError = MmRpc_call(MmRpcCallbackHandle, &fxnCtx, &fxnRet);
                    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);

                    /* When this return it means that Codec has called DCE Server with putDataFxn callback that has the numBlock information. */
                    /* At this point, codec should have filled the outputBuffer pointer that is passed in VIDDEC3_process. */
                    DEBUG("From codec (callbackmsg[%d]).local_dataSyncHandle %p (callbackmsg[%d]).local_dataSyncDesc->numBlocks %d ",
                        id, (callbackmsg[id]).local_dataSyncHandle, id, (int32_t) (callbackmsg[id]).local_dataSyncDesc->numBlocks);

                    if( (callbackmsg[id]).local_dataSyncDesc->numBlocks ) {
                        (callbackmsg[id]).receive_numBlocks += (callbackmsg[id]).local_dataSyncDesc->numBlocks;
                        return_callback = (int32_t) ((callbackmsg[id]).local_put_DataFxn)((callbackmsg[id]).local_dataSyncHandle, (callbackmsg[id]).local_dataSyncDesc);
                        if( return_callback < 0 ) {
                            /* dce_callback_putDataFxn getting no output data saved when calling the callback. Ignore and continue. */
                            ERROR("Received return_callback %d when asking client to save the output Data of (callbackmsg[%d]).numBlock %d",
                                return_callback, id, (callbackmsg[id]).local_dataSyncDesc->numBlocks);
                        }

                        DEBUG("(callbackmsg[%d]).local_dataSyncHandle %p (callbackmsg[%d]).receive_numBlocks %d >= (callbackmsg[%d]).total_numBlocks %d (callbackmsg[%d]).putDataFlag %d",
                            id, (callbackmsg[id]).local_dataSyncHandle, id, (callbackmsg[id]).receive_numBlocks, id, (callbackmsg[id]).total_numBlocks, id, (callbackmsg[id]).putDataFlag);
                    } else {
                        /* dce_callback_putDataFxn getting 0 numBlocks from DCE server. Need to stop the loop as it indicates VIDDEC3_process will be returned*/
                        DEBUG("dce_callback_putDataFxn is getting (callbackmsg[%d]).numBlock %d when calling putDataFxn callback",
                            id, (callbackmsg[id]).local_dataSyncDesc->numBlocks);
                        (callbackmsg[id]).putDataFlag = 0;
                    }

                    DEBUG("unlock (callbackmsg[%d]).putDataFxn_thread 0x%x (callbackmsg[%d]).local_dataSyncHandle %p",
                        id, (callbackmsg[id]).putDataFxn_thread, id, (callbackmsg[id]).local_dataSyncHandle);
                    pthread_mutex_unlock(&dce_callback_mutex);
                }
            } else if( (callbackmsg[id]).putDataFlag == 2 ) {
                /* Receive an indication to clean up and exit the thread. */
                DEBUG("CLEAN UP indication due to (callbackmsg[%d]).putDataFlag %d is set.", id, (callbackmsg[id]).putDataFlag);
                pthread_exit(0);
            } else {
                DEBUG("Do nothing (callbackmsg[%d]).local_dataSyncHandle 0x%x because (callbackmsg[%d]).putDataFlag %d is not 1.",
                    id, (unsigned int)(callbackmsg[id]).local_dataSyncHandle, id, (callbackmsg[id]).putDataFlag);
                if ((callbackmsg[id]).receive_numBlocks) {
                    (callbackmsg[id]).receive_numBlocks = 0;
                    DEBUG("Signal sem_dec_row_mode as VIDDEC3_process might wait before returning to client.");
                    sem_post(&((callbackmsg[id]).sem_dec_row_mode));
                }
                sem_wait(&((callbackmsg[id]).sem_dec_row_mode));
            }
        }
    }

EXIT:

    DEBUG("======================END======================== codec_handle 0x%x", (unsigned int) *codec_handle);
    return (0);
}


/* dce_callback_getDataFxn is running on different Thread id. */
/* It is an infinite loop request to client for more input data when inputDataMode = IVIDEO_ROWMODE. */
int dce_callback_getDataFxn(void *codec)
{
    MmRpc_FxnCtx        fxnCtx;
    int32_t             fxnRet;
    int32_t             return_callback;
    dce_error_status    eError = DCE_EOK;
    XDM_DataSyncHandle  *codec_handle = (XDM_DataSyncHandle *) codec;
    int                 id;

    DEBUG("======================START========================");
    DEBUG(" >> dce_callback_getDataFxn codec_handle 0x%x", (unsigned int) *codec_handle);

    id = get_callback((Uint32) *codec_handle);
    if( id < 0 ) {
        ERROR("Cannot find the entry in callbackmsg");
    } else {
        (callbackmsg[id]).local_dataSyncHandle = *codec_handle;
        /* This is called from VIDENC2_process, so we can start request client to fill in input and provide the row information written. */
        /* Call the callback function specified in the dynParams->getDataFxn */
        while( 1 ) {
            if( (callbackmsg[id]).getDataFlag == 1 ) {
                DEBUG("lock (callbackmsg[%d]).getDataFxn_thread 0x%x (callbackmsg[%d]).local_dataSyncHandle %p",
                    id, (callbackmsg[id]).getDataFxn_thread, id, (callbackmsg[id]).local_dataSyncHandle);
                pthread_mutex_lock(&dce_callback_mutex);
                if( (callbackmsg[id]).row_mode ) {
                    /* Calling client callback function to pass data received from IVA-HD codec */
                    return_callback = (int32_t) ((callbackmsg[id]).local_get_DataFxn)((callbackmsg[id]).local_dataSyncHandle, (callbackmsg[id]).local_dataSyncDesc);
                    if( return_callback < 0 ) {
                        /* dce_callback_getDataFxn getting error when calling the callback. Ignore and re-try. */
                        ERROR("dce_callback_getDataFxn is getting return_callback %d when calling getDataFxn callback. Retry callback for more data.", return_callback);
                    } else {
                        (callbackmsg[id]).receive_numBlocks += (callbackmsg[id]).local_dataSyncDesc->numBlocks;

                        if( (callbackmsg[id]).local_dataSyncDesc->numBlocks ) {
                            /* Marshall function arguments into the send callback information to codec for get_dataFxn */
                            Fill_MmRpc_fxnCtx(&fxnCtx, DCE_CALLBACK_RPC_GET_DATAFXN, 2, 0, NULL);
                            Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[0]), sizeof(int32_t), (int32_t) (callbackmsg[id]).local_dataSyncHandle);
                            Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[1]), GetSz((callbackmsg[id]).local_dataSyncDesc), (void *) P2H((callbackmsg[id]).local_dataSyncDesc),
                                                            sizeof(MemHeader), memplugin_share((callbackmsg[id]).local_dataSyncDesc));

                            eError = MmRpc_call(MmRpcCallbackHandle, &fxnCtx, &fxnRet);
                            _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);

                            DEBUG("(callbackmsg[%d]).local_dataSyncHandle %p (callbackmsg[%d]).receive_numBlocks %d >= (callbackmsg[%d]).total_numBlocks %d",
                                id, (callbackmsg[id]).local_dataSyncHandle, id, (callbackmsg[id]).receive_numBlocks, id, (callbackmsg[id]).total_numBlocks);

                            if( (callbackmsg[id]).receive_numBlocks >= (callbackmsg[id]).total_numBlocks ) {
                                /* one full frame has been sent to codec. Need to stop the callback call to client. */
                                /* it will be resumed once VIDENC2_process for the next one. */
                                (callbackmsg[id]).receive_numBlocks = 0;
                                (callbackmsg[id]).getDataFlag = 0;
                                DEBUG("Setting the (callbackmsg[%d]).getDataFlag to 0 to stop calling client to fill", id);
                            }
                        } else {
                            /* dce_callback_getDataFxn getting 0 numBlocks when calling the callback. Ignore and re-try. */
                            DEBUG("Received dataSyncDesc->numBlocks == 0 meaning the callback thread has no data -ignore.");
                        }
                    }
                    DEBUG("unlock (callbackmsg[%d]).getDataFxn_thread 0x%x (callbackmsg[%d]).local_dataSyncHandle %p",
                        id, (callbackmsg[id]).getDataFxn_thread, id, (callbackmsg[id]).local_dataSyncHandle);
                    pthread_mutex_unlock(&dce_callback_mutex);
                }
            } else if( (callbackmsg[id]).getDataFlag == 2 ) {
                /* Receive an indication to clean up and exit the thread. */
                DEBUG("CLEAN UP indication due to (callbackmsg[%d]).getDataFlag %d is set.", id, (callbackmsg[id]).getDataFlag);
                pthread_exit(0);
            } else {
                DEBUG("Do nothing (callbackmsg[%d]).local_dataSyncHandle 0x%x because msg-getDataFlag %d is not 1.",
                    id, (unsigned int)(callbackmsg[id]).local_dataSyncHandle, (callbackmsg[id]).getDataFlag);
                sem_wait(&((callbackmsg[id]).sem_enc_row_mode));
            }
        }
    }

EXIT:
    DEBUG(" << dce_callback_getDataFxn COMPLETE");
    DEBUG("======================END========================");
    return (0);
}

/*=====================================================================================*/
/** dce_ipc_init            : Initialize MmRpc. This function is called within Engine_open().
 *
 * @ return                 : Error Status.
 */
int dce_ipc_init(int core)
{
    MmRpc_Params        args;
    dce_error_status    eError = DCE_EOK;

    DEBUG(" >> dce_ipc_init\n");

    /*First check if maximum clients are already using ipc*/
    if( __ClientCount[core] >= MAX_INSTANCES ) {
        eError = DCE_EXDM_UNSUPPORTED;
        return (eError);
    }

    /* Create remote server insance */
    __ClientCount[core]++;

    if( __ClientCount[core] > 1 ) {
         goto EXIT;
    }

    MmRpc_Params_init(&args);

    eError = MmRpc_create(DCE_DEVICE_NAME[core], &args, &MmRpcHandle[core]);
    _ASSERT_AND_EXECUTE(eError == DCE_EOK, DCE_EIPC_CREATE_FAIL, __ClientCount[core]--);
    DEBUG("open(/dev/%s]) -> 0x%x\n", DCE_DEVICE_NAME[core], (int)MmRpcHandle[core]);

EXIT:
    return (eError);
}

/*=====================================================================================*/
/** dce_ipc_deinit            : DeInitialize MmRpc. This function is called within
 *                              Engine_close().
 */
void dce_ipc_deinit(int core, int tableIdx)
{
    if( __ClientCount[core] == 0 ) {
        DEBUG("Nothing to be done: a spurious call\n");
        return;
    }
    __ClientCount[core]--;

    if ( tableIdx >= 0 )
        gEngineHandle[tableIdx][core] = 0;

    if( __ClientCount[core] > 0 ) {
         goto EXIT;
    }

    if( MmRpcHandle[core] != NULL ) {
         MmRpc_delete(&MmRpcHandle[core]);
         MmRpcHandle[core] = NULL;
    }

EXIT:
    return;
}

/*===============================================================*/
/** Engine_open        : Open Codec Engine.
 *
 * @ param attrs  [in]      : Engine Attributes. This param is not passed to Remote core.
 * @ param name [in]        : Name of Encoder or Decoder codec.
 * @ param ec [out]         : Error returned by Codec Engine.
 * @ return : Codec Engine Handle is returned to be used to create codec.
 *                 In case of error, NULL is returned as Engine Handle.
 */
Engine_Handle Engine_open(String name, Engine_Attrs *attrs, Engine_Error *ec)
{
    MmRpc_FxnCtx        fxnCtx;
    dce_error_status    eError = DCE_EOK;
    dce_engine_open     *engine_open_msg = NULL;
    Engine_Attrs        *engine_attrs = NULL;
    Engine_Handle       engine_handle = NULL;
    int                 coreIdx   = INVALID_CORE;
    int                 tabIdx      = -1;

    DEBUG("START Engine_open ipc_mutex 0x%x", (unsigned int) &ipc_mutex);
    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);

    _ASSERT(name != '\0', DCE_EINVALID_INPUT);

    coreIdx = getCoreIndexFromName(name);
    _ASSERT(coreIdx != INVALID_CORE, DCE_EINVALID_INPUT);
    /* Initialize IPC. In case of Error Deinitialize them */
    _ASSERT(dce_ipc_init(coreIdx) == DCE_EOK, DCE_EIPC_CREATE_FAIL);

    INFO(">> Engine_open Params::name = %s size = %d\n", name, strlen(name));
    /* Allocate Shared memory for the engine_open rpc msg structure*/
    /* Tiler Memory preferred in QNX */
    engine_open_msg = memplugin_alloc(sizeof(dce_engine_open), 1, DEFAULT_REGION, 0, coreIdx);
    _ASSERT_AND_EXECUTE(engine_open_msg != NULL, DCE_EOUT_OF_MEMORY, engine_handle = NULL);

    if( attrs ) {
         engine_attrs = memplugin_alloc(sizeof(Engine_Attrs), 1, DEFAULT_REGION, 0, coreIdx);
         _ASSERT_AND_EXECUTE(engine_attrs != NULL, DCE_EOUT_OF_MEMORY, engine_handle = NULL);
         *engine_attrs = *attrs;
    }
    /* Populating the msg structure with all the params */
    /* Populating all params into a struct avoid individual address translations of name, ec */
    strncpy(engine_open_msg->name, name, strlen(name));
    engine_open_msg->engine_attrs = engine_attrs;

    /* Marshall function arguments into the send buffer */
    Fill_MmRpc_fxnCtx(&fxnCtx, DCE_RPC_ENGINE_OPEN, 1, 0, NULL);
    Fill_MmRpc_fxnCtx_OffPtr_Params(fxnCtx.params, GetSz(engine_open_msg), (void *)P2H(engine_open_msg),
                                    sizeof(MemHeader), memplugin_share(engine_open_msg));

    /* Invoke the Remote function through MmRpc */
    eError = MmRpc_call(MmRpcHandle[coreIdx], &fxnCtx, (int32_t *)(&engine_handle));

    if( ec ) {
         *ec = engine_open_msg->error_code;
    }
    /* In case of Error, the Application will get a NULL Engine Handle */
    _ASSERT_AND_EXECUTE(eError == DCE_EOK, DCE_EIPC_CALL_FAIL, engine_handle = NULL);

    /*Update table*/
    tabIdx = update_clients_table(engine_handle, coreIdx);
    _ASSERT((tabIdx != -1), DCE_EINVALID_INPUT);

EXIT:
    memplugin_free(engine_open_msg);

    if( engine_attrs ) {
         memplugin_free(engine_attrs);
    }
    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);
    DEBUG("END Engine_open ipc_mutex 0x%x", (unsigned int) &ipc_mutex);

    return ((Engine_Handle)engine_handle);
}

/*===============================================================*/
/** Engine_close           : Close Engine.
 *
 * @ param engine  [in]    : Engine Handle obtained in Engine_open() call.
 */
Void Engine_close(Engine_Handle engine)
{
    MmRpc_FxnCtx        fxnCtx;
    int32_t             fxnRet;
    dce_error_status    eError = DCE_EOK;
    int32_t             coreIdx = INVALID_CORE;
    int                 tableIdx = -1;


    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);

    _ASSERT(engine != NULL, DCE_EINVALID_INPUT);

    /* Marshall function arguments into the send buffer */
    Fill_MmRpc_fxnCtx(&fxnCtx, DCE_RPC_ENGINE_CLOSE, 1, 0, NULL);
    Fill_MmRpc_fxnCtx_Scalar_Params(fxnCtx.params, sizeof(Engine_Handle), (int32_t)engine);

    coreIdx = getCoreIndexFromEngine(engine, &tableIdx);
    _ASSERT(coreIdx != INVALID_CORE,DCE_EINVALID_INPUT);

    /* Invoke the Remote function through MmRpc */
    eError = MmRpc_call(MmRpcHandle[coreIdx], &fxnCtx, &fxnRet);
    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);

EXIT:
    if( coreIdx != INVALID_CORE )
        dce_ipc_deinit(coreIdx, tableIdx);

    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);

    return;
}

 /*===============================================================*/
/** get_rproc_info : Get Information from the Remote proc.
 *
 * @ param engine    [in]    : Engine Handle obtained in Engine_open() call.
 * @ param info_type [in]    : Information type as defined in the rproc_info_type
+ */
int32_t get_rproc_info(Engine_Handle engine, rproc_info_type info_type)
{
    MmRpc_FxnCtx        fxnCtx;
    int32_t             fxnRet;
    dce_error_status    eError = DCE_EOK;
    int32_t             coreIdx = INVALID_CORE;
    int                 tableIdx = -1;

    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);

    _ASSERT(engine != NULL, DCE_EINVALID_INPUT);

    /* Marshall function arguments into the send buffer */
    Fill_MmRpc_fxnCtx(&fxnCtx, DCE_RPC_GET_INFO, 1, 0, NULL);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[0]), sizeof(rproc_info_type), (int32_t)info_type);

    coreIdx = getCoreIndexFromEngine(engine, &tableIdx);
    _ASSERT(coreIdx != INVALID_CORE,DCE_EINVALID_INPUT);

    /* Invoke the Remote function through MmRpc */
    eError = MmRpc_call(MmRpcHandle[coreIdx], &fxnCtx, &fxnRet);
    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);

EXIT:
    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);

    return fxnRet;
}


/*===============================================================*/
/** Functions create(), control(), get_version(), process(), delete() are common codec
 * glue function signatures which are same for both encoder and decoder
 */
/*===============================================================*/
/** create         : Create Encoder/Decoder codec.
 *
 * @ param engine  [in]    : Engine Handle obtained in Engine_open() call.
 * @ param name [in]       : Name of Encoder or Decoder codec.
 * @ param params [in]     : Static parameters of codec.
 * @ param codec_id [in]   : To differentiate between Encoder and Decoder codecs.
 * @ return : Codec Handle is returned to be used for control, process, delete calls.
 *                 In case of error, NULL is returned.
 */
static void *create(Engine_Handle engine, String name, void *params, dce_codec_type codec_id)
{
    MmRpc_FxnCtx        fxnCtx;
    dce_error_status    eError = DCE_EOK;
    void                *codec_handle = NULL;
    char                *codec_name = NULL;
    int                 coreIdx = INVALID_CORE;

    _ASSERT(name != '\0', DCE_EINVALID_INPUT);
    _ASSERT(engine != NULL, DCE_EINVALID_INPUT);
    _ASSERT(params != NULL, DCE_EINVALID_INPUT);

    coreIdx = getCoreIndexFromCodec(codec_id);
    _ASSERT(coreIdx != INVALID_CORE, DCE_EINVALID_INPUT);

    /* Allocate shared memory for translating codec name to IPU */
    codec_name = memplugin_alloc(MAX_NAME_LENGTH * sizeof(char), 1, DEFAULT_REGION, 0, coreIdx);
    _ASSERT_AND_EXECUTE(codec_name != NULL, DCE_EOUT_OF_MEMORY, codec_handle = NULL);

    strncpy(codec_name, name, strlen(name));

    /* Marshall function arguments into the send buffer */
    Fill_MmRpc_fxnCtx(&fxnCtx, DCE_RPC_CODEC_CREATE, 4, 0, NULL);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[0]), sizeof(int32_t), codec_id);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[1]), sizeof(Engine_Handle), (int32_t)engine);
    Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[2]), GetSz(codec_name), P2H(codec_name),
                                    sizeof(MemHeader), memplugin_share(codec_name));
    Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[3]), GetSz(params), P2H(params),
                                    sizeof(MemHeader),  memplugin_share(params));
    /* Invoke the Remote function through MmRpc */
    eError = MmRpc_call(MmRpcHandle[coreIdx], &fxnCtx, (int32_t *)(&codec_handle));

    /* In case of Error, the Application will get a NULL Codec Handle */
    _ASSERT_AND_EXECUTE(eError == DCE_EOK, DCE_EIPC_CALL_FAIL, codec_handle = NULL);

EXIT:
    memplugin_free(codec_name);
    return ((void *)codec_handle);
}

/*===============================================================*/
/** control                : Codec control call.
 *
 * @ param codec  [in]     : Codec Handle obtained in create() call.
 * @ param id [in]         : Command id for XDM control operation.
 * @ param dynParams [in]  : Dynamic input parameters to Codec.
 * @ param status [out]    : Codec returned status parameters.
 * @ param codec_id [in]   : To differentiate between Encoder and Decoder codecs.
 * @ return : Status of control() call is returned.
 *                #XDM_EOK                  [0]   :  Success.
 *                #XDM_EFAIL                [-1] :  Failure.
 *                #IPC_FAIL                   [-2] : MmRpc Call failed.
 *                #XDM_EUNSUPPORTED [-3] :  Unsupported request.
 *                #OUT_OF_MEMORY       [-4] :  Out of Shared Memory.
 */
static XDAS_Int32 control(void *codec, int id, void *dynParams, void *status, dce_codec_type codec_id)
{
    MmRpc_FxnCtx        fxnCtx;
    int32_t             fxnRet = XDM_EFAIL;
    dce_error_status    eError = DCE_EOK;
    int                 coreIdx = INVALID_CORE;

    _ASSERT(codec != NULL, DCE_EINVALID_INPUT);
    _ASSERT(dynParams != NULL, DCE_EINVALID_INPUT);
    _ASSERT(status != NULL, DCE_EINVALID_INPUT);

    coreIdx = getCoreIndexFromCodec(codec_id);
    _ASSERT(coreIdx != INVALID_CORE, DCE_EINVALID_INPUT);

    /* Marshall function arguments into the send buffer */
    Fill_MmRpc_fxnCtx(&fxnCtx, DCE_RPC_CODEC_CONTROL, 5, 0, NULL);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[0]), sizeof(int32_t), codec_id);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[1]), sizeof(int32_t), (int32_t)codec);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[2]), sizeof(int32_t), (int32_t)id);
    Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[3]), GetSz(dynParams), P2H(dynParams),
                                    sizeof(MemHeader), memplugin_share(dynParams));
    Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[4]), GetSz(status), P2H(status),
                                    sizeof(MemHeader), memplugin_share(status));

    /* Invoke the Remote function through MmRpc */
    eError = MmRpc_call(MmRpcHandle[coreIdx], &fxnCtx, &fxnRet);
    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);

EXIT:
    return (fxnRet);

}

/*===============================================================*/
/** get_version        : Codec control call to get the codec version. This call has been made
 *                       separate from control call because it involves an additional version
 *                       buffer translation.
 *
 * @ param codec  [in]     : Codec Handle obtained in create() call.
 * @ param id [in]            : Command id for XDM control operation.
 * @ param dynParams [in] : Dynamic input parameters to Codec.
 * @ param status [out]    : Codec returned status parameters.
 * @ param codec_id [in]  : To differentiate between Encoder and Decoder codecs.
 * @ return : Status of control() call is returned.
 *                #XDM_EOK                  [0]   :  Success.
 *                #XDM_EFAIL                [-1] :  Failure.
 *                #IPC_FAIL                   [-2] : MmRpc Call failed.
 *                #XDM_EUNSUPPORTED [-3] :  Unsupported request.
 *                #OUT_OF_MEMORY       [-4] :  Out of Shared Memory.
 */
static XDAS_Int32 get_version(void *codec, void *dynParams, void *status, dce_codec_type codec_id)
{
    MmRpc_FxnCtx        fxnCtx;
    MmRpc_Xlt           xltAry;
    void             * *version_buf = NULL;
    int32_t             fxnRet = XDM_EFAIL;
    dce_error_status    eError = DCE_EOK;
    int                 coreIdx = INVALID_CORE;

    _ASSERT(codec != NULL, DCE_EINVALID_INPUT);
    _ASSERT(dynParams != NULL, DCE_EINVALID_INPUT);
    _ASSERT(status != NULL, DCE_EINVALID_INPUT);

    coreIdx = getCoreIndexFromCodec(codec_id);
    _ASSERT(coreIdx != INVALID_CORE, DCE_EINVALID_INPUT);

    if( codec_id == OMAP_DCE_VIDDEC3 ) {
         version_buf = (void * *)(&(((IVIDDEC3_Status *)status)->data.buf));
    } else if( codec_id == OMAP_DCE_VIDENC2 ) {
         version_buf = (void * *)(&(((IVIDENC2_Status *)status)->data.buf));
    } else if( codec_id == OMAP_DCE_VIDDEC2 ) {
         version_buf = (void * *)(&(((IVIDDEC2_Status *)status)->data.buf));
    }
    _ASSERT(*version_buf != NULL, DCE_EINVALID_INPUT);

    /* Marshall function arguments into the send buffer */
    Fill_MmRpc_fxnCtx(&fxnCtx, DCE_RPC_CODEC_GET_VERSION, 4, 1, &xltAry);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[0]), sizeof(int32_t), codec_id);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[1]), sizeof(int32_t), (int32_t)codec);
    Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[2]), GetSz(dynParams), P2H(dynParams),
                                    sizeof(MemHeader), memplugin_share(dynParams));
    Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[3]), GetSz(status), P2H(status),
                                    sizeof(MemHeader), memplugin_share(status));

    /* Address Translation needed for buffer for version Info */
    Fill_MmRpc_fxnCtx_Xlt_Array(fxnCtx.xltAry, 3,
         MmRpc_OFFSET((int32_t)status, (int32_t)version_buf),
         (size_t)P2H(*version_buf), memplugin_share(*version_buf));

    /* Invoke the Remote function through MmRpc */
    eError = MmRpc_call(MmRpcHandle[coreIdx], &fxnCtx, &fxnRet);
    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);

EXIT:
    return (fxnRet);
}

typedef enum process_call_params {
    CODEC_ID_INDEX = 0,
    CODEC_HANDLE_INDEX,
    INBUFS_INDEX,
    OUTBUFS_INDEX,
    INARGS_INDEX,
    OUTARGS_INDEX,
    OUTBUFS_PTR_INDEX
} process_call_params;

#define LUMA_BUF 0
#define CHROMA_BUF 1
/*===============================================================*/
/** process               : Encode/Decode process.
 *
 * @ param codec  [in]     : Codec Handle obtained in create() call.
 * @ param inBufs [in]     : Input buffer details.
 * @ param outBufs [in]    : Output buffer details.
 * @ param inArgs [in]     : Input arguments.
 * @ param outArgs [out]   : Output arguments.
 * @ param codec_id [in]   : To differentiate between Encoder and Decoder codecs.
 * @ return : Status of the process call.
 *                #XDM_EOK                  [0]   :  Success.
 *                #XDM_EFAIL                [-1] :  Failure.
 *                #IPC_FAIL                   [-2] :  MmRpc Call failed.
 *                #XDM_EUNSUPPORTED [-3] :  Unsupported request.
 */
static XDAS_Int32 process(void *codec, void *inBufs, void *outBufs,
                          void *inArgs, void *outArgs, dce_codec_type codec_id)
{
    MmRpc_FxnCtx        fxnCtx;
    MmRpc_Xlt           xltAry[MAX_TOTAL_BUF];
    int                 fxnRet, count, total_count, numInBufs = 0, numOutBufs = 0;
    dce_error_status    eError = DCE_EOK;
    void                **data_buf = NULL;
    void                **buf_arry = NULL;
    void                **bufSize_arry = NULL;
    int                 numXltAry, numParams;
    int                 coreIdx = INVALID_CORE;

#ifdef BUILDOS_ANDROID
    int32_t    inbuf_offset[MAX_INPUT_BUF];
    int32_t    outbuf_offset[MAX_OUTPUT_BUF];
#endif

    _ASSERT(codec != NULL, DCE_EINVALID_INPUT);
    _ASSERT(inBufs != NULL, DCE_EINVALID_INPUT);
    _ASSERT(outBufs != NULL, DCE_EINVALID_INPUT);
    _ASSERT(inArgs != NULL, DCE_EINVALID_INPUT);
    _ASSERT(outArgs != NULL, DCE_EINVALID_INPUT);

    coreIdx = getCoreIndexFromCodec(codec_id);
    _ASSERT(coreIdx != INVALID_CORE, DCE_EINVALID_INPUT);

    if( codec_id == OMAP_DCE_VIDDEC3 ) {
         numInBufs = ((XDM2_BufDesc *)inBufs)->numBufs;
         numOutBufs = ((XDM2_BufDesc *)outBufs)->numBufs;
         numXltAry = numInBufs + numOutBufs;
         numParams = 6;
    } else if( codec_id == OMAP_DCE_VIDENC2 ) {
         numInBufs = ((IVIDEO2_BufDesc *)inBufs)->numPlanes;
         numOutBufs = ((XDM2_BufDesc *)outBufs)->numBufs;
         numXltAry = numInBufs + numOutBufs;
         numParams = 6;
    } else if( codec_id == OMAP_DCE_VIDDEC2 ) {
         numInBufs = ((XDM1_BufDesc *)inBufs)->numBufs;
         numOutBufs = ((XDM_BufDesc *)outBufs)->numBufs;
         numXltAry = numInBufs + numOutBufs + MAX_OUTPUT_BUFPTRS;/* 2 extra needed for bufs and bufSizes */
         numParams = 7;
    } else{
         eError = DCE_EXDM_UNSUPPORTED;
         return eError;
    }

    /* marshall function arguments into the send buffer                       */
    /* Approach [2] as explained in "Notes" used for process               */
    Fill_MmRpc_fxnCtx(&fxnCtx, DCE_RPC_CODEC_PROCESS, numParams, numXltAry, xltAry);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[CODEC_ID_INDEX]), sizeof(int32_t), codec_id);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[CODEC_HANDLE_INDEX]), sizeof(int32_t), (int32_t)codec);

    Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[INBUFS_INDEX]), GetSz(inBufs), P2H(inBufs),
                                    sizeof(MemHeader), memplugin_share(inBufs));
    Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[OUTBUFS_INDEX]), GetSz(outBufs), P2H(outBufs),
                                    sizeof(MemHeader), memplugin_share(outBufs));
    Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[INARGS_INDEX]), GetSz(inArgs), P2H(inArgs),
                                    sizeof(MemHeader), memplugin_share(inArgs));
    Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[OUTARGS_INDEX]), GetSz(outArgs), P2H(outArgs),
                                    sizeof(MemHeader), memplugin_share(outArgs));

    /* InBufs, OutBufs, InArgs, OutArgs buffer need translation but since they have been */
    /* individually mentioned as fxnCtx Params, they need not be mentioned below again */
    /* Input and Output Buffers have to be mentioned for translation                               */
    for( count = 0, total_count = 0; count < numInBufs; count++, total_count++ ) {
         if( codec_id == OMAP_DCE_VIDDEC3 ) {
              data_buf = (void * *)(&(((XDM2_BufDesc *)inBufs)->descs[count].buf));
         } else if( codec_id == OMAP_DCE_VIDENC2 ) {
              data_buf = (void * *)(&(((IVIDEO2_BufDesc *)inBufs)->planeDesc[count].buf));
         } else if( codec_id == OMAP_DCE_VIDDEC2 ) {
              data_buf = (void * *)(&(((XDM1_BufDesc *)inBufs)->descs[count].buf));
         }
#ifdef BUILDOS_ANDROID
        inbuf_offset[count] = ((MemHeader*)(*data_buf))->offset;
        Fill_MmRpc_fxnCtx_Xlt_Array(&(fxnCtx.xltAry[total_count]), INBUFS_INDEX, MmRpc_OFFSET((int32_t)inBufs,
                                    (int32_t)data_buf), (size_t)(*data_buf),
                                    (size_t)(((MemHeader*)(*data_buf))->dma_buf_fd));
        *data_buf += inbuf_offset[count];
#else
        Fill_MmRpc_fxnCtx_Xlt_Array(&(fxnCtx.xltAry[total_count]), INBUFS_INDEX,
                                    MmRpc_OFFSET((int32_t)inBufs, (int32_t)data_buf),
                                    (size_t)*data_buf, (size_t)*data_buf);
#ifdef BUILDOS_LINUX
        /*Single planar input buffer for Encoder. No adjustments needed for Multiplanar case*/
        if( count == CHROMA_BUF && codec_id == OMAP_DCE_VIDENC2 && ((IVIDEO2_BufDesc *)inBufs)->planeDesc[LUMA_BUF].buf == ((IVIDEO2_BufDesc *)inBufs)->planeDesc[CHROMA_BUF].buf ) {
            if( ((IVIDEO2_BufDesc *)inBufs)->planeDesc[count].memType == XDM_MEMTYPE_RAW ||
               ((IVIDEO2_BufDesc *)inBufs)->planeDesc[count].memType == XDM_MEMTYPE_TILEDPAGE )
                *data_buf += ((IVIDEO2_BufDesc *)inBufs)->planeDesc[LUMA_BUF].bufSize.bytes;
            else
                *data_buf += ((IVIDEO2_BufDesc *)inBufs)->planeDesc[LUMA_BUF].bufSize.tileMem.width *
                                ((IVIDEO2_BufDesc *)inBufs)->planeDesc[LUMA_BUF].bufSize.tileMem.height;
        }
#endif //BUILDOS_LINUX
#endif //BUILDOS_ANDROID
    }

    /* Output Buffers */
    for( count = 0; count < numOutBufs; count++, total_count++ ) {
        if( codec_id == OMAP_DCE_VIDENC2 || codec_id == OMAP_DCE_VIDDEC3 ) {
            if( ((XDM2_BufDesc *)outBufs)->descs[LUMA_BUF].buf != ((XDM2_BufDesc *)outBufs)->descs[CHROMA_BUF].buf ) {
                /* Either Encode usecase or MultiPlanar Buffers for Decode usecase */
                data_buf = (void * *)(&(((XDM2_BufDesc *)outBufs)->descs[count].buf));
#ifdef BUILDOS_ANDROID
                outbuf_offset[count] = ((MemHeader*)(*data_buf))->offset;
                Fill_MmRpc_fxnCtx_Xlt_Array(&(fxnCtx.xltAry[total_count]), OUTBUFS_INDEX, MmRpc_OFFSET((int32_t)outBufs,
                                            (int32_t)data_buf), (size_t)(*data_buf),
                                            (size_t)(((MemHeader*)(*data_buf))->dma_buf_fd));
                *data_buf += outbuf_offset[count];

#else
                Fill_MmRpc_fxnCtx_Xlt_Array(&(fxnCtx.xltAry[total_count]), OUTBUFS_INDEX,
                                            MmRpc_OFFSET((int32_t)outBufs, (int32_t)data_buf),
                                            (size_t)*data_buf, (size_t)*data_buf);
#endif
              }
#if defined(BUILDOS_LINUX)
              else {
                   /* SinglePlanar Buffers for Decode usecase*/
                   data_buf = (void * *)(&(((XDM2_BufDesc *)outBufs)->descs[count].buf));
                   Fill_MmRpc_fxnCtx_Xlt_Array(&(fxnCtx.xltAry[total_count]), OUTBUFS_INDEX,
                        MmRpc_OFFSET((int32_t)outBufs, (int32_t)data_buf),
                        (size_t)*data_buf, (size_t)*data_buf);

                   if( count == CHROMA_BUF ) {
                        if( ((XDM2_BufDesc *)outBufs)->descs[count].memType == XDM_MEMTYPE_RAW ||
                           ((XDM2_BufDesc *)outBufs)->descs[count].memType == XDM_MEMTYPE_TILEDPAGE ) {
                             *data_buf += ((XDM2_BufDesc *)outBufs)->descs[LUMA_BUF].bufSize.bytes;
                        } else {
                             *data_buf += ((XDM2_BufDesc *)outBufs)->descs[LUMA_BUF].bufSize.tileMem.width *
                                  ((XDM2_BufDesc *)outBufs)->descs[LUMA_BUF].bufSize.tileMem.height;
                        }
                   }
              }
#endif
         } else if( codec_id == OMAP_DCE_VIDDEC2 ) {
              if( count == LUMA_BUF ) {
                   buf_arry = (void * *)(&(((XDM_BufDesc *)outBufs)->bufs));

                   Fill_MmRpc_fxnCtx_Xlt_Array(&(fxnCtx.xltAry[total_count]), OUTBUFS_INDEX,
                        MmRpc_OFFSET((int32_t)outBufs, (int32_t)buf_arry),
                        (size_t)P2H(*buf_arry), (size_t)memplugin_share(*buf_arry));

                   total_count++;

                   bufSize_arry = (void * *)(&(((XDM_BufDesc *)outBufs)->bufSizes));

                   Fill_MmRpc_fxnCtx_Xlt_Array(&(fxnCtx.xltAry[total_count]), OUTBUFS_INDEX,
                        MmRpc_OFFSET((int32_t)outBufs, (int32_t)bufSize_arry),
                        (size_t)P2H(*bufSize_arry), (size_t)memplugin_share(*bufSize_arry));

                   total_count++;
              }

              Fill_MmRpc_fxnCtx_OffPtr_Params(&(fxnCtx.params[OUTBUFS_PTR_INDEX]), GetSz(*buf_arry), P2H(*buf_arry),
                   sizeof(MemHeader), memplugin_share(*buf_arry));

              data_buf = (void * *)(&(((XDM_BufDesc *)outBufs)->bufs[count]));

              Fill_MmRpc_fxnCtx_Xlt_Array(&(fxnCtx.xltAry[total_count]), OUTBUFS_PTR_INDEX,
                   MmRpc_OFFSET((int32_t)*buf_arry, (int32_t)data_buf), (size_t)*data_buf, (size_t)*data_buf);
         }
    }

    /* Invoke the Remote function through MmRpc */
    eError = MmRpc_call(MmRpcHandle[coreIdx], &fxnCtx, &fxnRet);
    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);

#ifdef BUILDOS_ANDROID
    for( count = 0; count < numInBufs; count++ ) {
        if( codec_id == OMAP_DCE_VIDDEC3 ) {
            /* restore the actual buf ptr before returing to the mmf */
            data_buf = (void * *)(&(((XDM2_BufDesc *)inBufs)->descs[count].buf));
        } else if( codec_id == OMAP_DCE_VIDDEC2 ) {
            /* restore the actual buf ptr before returing to the mmf */
            data_buf = (void * *)(&(((XDM1_BufDesc *)inBufs)->descs[count].buf));
        }
        *data_buf -= inbuf_offset[count];
    }
    for (count = 0; count < numOutBufs; count++){
        data_buf = (void * *)(&(((XDM2_BufDesc *)outBufs)->descs[count].buf));
        *data_buf -= outbuf_offset[count];
    }
#endif

    eError = (dce_error_status)(fxnRet);

EXIT:
    return (eError);
}

/*===============================================================*/
/** delete                : Delete Encode/Decode codec instance.
 *
 * @ param codec  [in]    : Codec Handle obtained in create() call.
 * @ param codec_id [in]  : To differentiate between Encoder and Decoder codecs.
 * @ return : None.
 */
static void delete(void *codec, dce_codec_type codec_id)
{
    MmRpc_FxnCtx        fxnCtx;
    int32_t             fxnRet;
    dce_error_status    eError = DCE_EOK;
    int                 coreIdx = INVALID_CORE;

    _ASSERT(codec != NULL, DCE_EINVALID_INPUT);
    coreIdx = getCoreIndexFromCodec(codec_id);
    _ASSERT(coreIdx != INVALID_CORE, DCE_EINVALID_INPUT);

    /* Marshall function arguments into the send buffer */
    Fill_MmRpc_fxnCtx(&fxnCtx, DCE_RPC_CODEC_DELETE, 2, 0, NULL);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[0]), sizeof(int32_t), codec_id);
    Fill_MmRpc_fxnCtx_Scalar_Params(&(fxnCtx.params[1]), sizeof(int32_t), (int32_t)codec);

    /* Invoke the Remote function through MmRpc */
    eError = MmRpc_call(MmRpcHandle[coreIdx], &fxnCtx, &fxnRet);
    _ASSERT(eError == DCE_EOK, DCE_EIPC_CALL_FAIL);

EXIT:
    return;
}

/***************** VIDDEC3 Decoder Codec Engine Functions ****************/
VIDDEC3_Handle VIDDEC3_create(Engine_Handle engine, String name,
                              VIDDEC3_Params *params)
{
    VIDDEC3_Handle codec = NULL;
    MmRpc_Params args;
    dce_error_status eError = DCE_EOK;
    int id;

    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);

    id = get_callback(0);
    if (id < 0) {
        /* This is depended on the MAX_INSTANCE, by default it handles only 5 instances of codec instance (full frame or low latency) */
        ERROR("Failed because too many codec clients, Max is %d. MAX_INSTANCES default needs to be changed if required.", MAX_INSTANCES);
        goto EXIT;
    } else { /* Found empty array to be populated */
        if( params->outputDataMode == IVIDEO_NUMROWS ) {
            (callbackmsg[id]).row_mode = 1;
            (callbackmsg[id]).first_control = TRUE;
            (callbackmsg[id]).total_numBlocks = params->maxHeight / 16;
            DEBUG("callbackmsg[%d]->total_numBlocks %d", id, (callbackmsg[id]).total_numBlocks);

            (callbackmsg[id]).local_dataSyncDesc = memplugin_alloc(sizeof(XDM_DataSyncDesc), 1, DEFAULT_REGION, 0, IPU);
            DEBUG("Checking local_dataSyncDesc %p local_put_DataFxn %p local_dataSyncHandle %p",
                (callbackmsg[id]).local_dataSyncDesc, (callbackmsg[id]).local_put_DataFxn, (callbackmsg[id]).local_dataSyncHandle);
            if( (callbackmsg[id]).local_dataSyncDesc == NULL ) {
                goto EXIT;
            }

            /* Create sem_dec_row_mode */
            sem_init(&((callbackmsg[id]).sem_dec_row_mode), 0, 0);

            DEBUG("MmRpcCallbackHandle 0x%x MmRpcCallback_count %d", (int)MmRpcCallbackHandle, MmRpcCallback_count);
            if( !MmRpcCallbackHandle ) {
                /* Need to create another MmRpcHandle for codec callback */
                MmRpc_Params_init(&args);
                eError = MmRpc_create(DCE_CALLBACK_NAME, &args, &MmRpcCallbackHandle);
                _ASSERT_AND_EXECUTE(eError == DCE_EOK, DCE_EIPC_CREATE_FAIL, MmRpcCallbackHandle = NULL);
                DEBUG("open(/dev/%s]) -> 0x%x\n", DCE_CALLBACK_NAME, (int)MmRpcCallbackHandle);
            }
            MmRpcCallback_count++;
        } else if( params->outputDataMode == IVIDEO_ENTIREFRAME ) {
            (callbackmsg[id]).row_mode = 0; /* full frame; the other parameters are not used in full frame mode */
        } else {
            ERROR("outputDataMode %d is not supported.", params->outputDataMode);
            goto EXIT;
        }
        DEBUG("Checking row_mode %d first_control %d",
            (callbackmsg[id]).row_mode, (callbackmsg[id]).first_control);
    }

    DEBUG(">> engine=%p, name=%s, params=%p", engine, name, params);
    codec = create(engine, name, params, OMAP_DCE_VIDDEC3);
    DEBUG("<< codec=%p", codec);

    if( params->outputDataMode == IVIDEO_NUMROWS ) {
        (callbackmsg[id]).codec_handle = (XDAS_UInt32) codec;
        DEBUG("Saving the codec %p to (callbackmsg[%d]).codec_handle = 0x%x", codec, id, (unsigned int) (callbackmsg[id]).codec_handle);
    }

EXIT:
    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);
    return (codec);
}

XDAS_Int32 VIDDEC3_control(VIDDEC3_Handle codec, VIDDEC3_Cmd cmd_id,
                           VIDDEC3_DynamicParams *dynParams, VIDDEC3_Status *status)
{
    XDAS_Int32 ret;
    int id;

    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);

    id = get_callback((Uint32) codec);
    if( id < 0 ) {
        DEBUG("Could not find the entry; control on full frame mode");
    } else {
        DEBUG("Checking codec_handle 0x%x row_mode %d first_control %d",
            (unsigned int) (callbackmsg[id]).codec_handle, (callbackmsg[id]).row_mode, (callbackmsg[id]).first_control);
        if( (callbackmsg[id]).row_mode && (callbackmsg[id]).first_control ) {
            /* dynParams has the function callback; store the information as it will get overwritten by the M4 codec for their own callback Fxn. */
            (callbackmsg[id]).local_put_DataFxn = (void*) dynParams->putDataFxn;
            (callbackmsg[id]).local_dataSyncHandle = dynParams->putDataHandle;
            (callbackmsg[id]).first_control = FALSE;
            DEBUG("Set callback pointer local_get_dataFxn %p local_dataSyncHandle %p",
                (callbackmsg[id]).local_put_DataFxn, (callbackmsg[id]).local_dataSyncHandle);
        }

        if( cmd_id == XDM_FLUSH ) {
            DEBUG("FLUSH HAS BEEN ORDERED");
        }
    }

    DEBUG(">> codec=%p, cmd_id=%d, dynParams=%p, status=%p",
          codec, cmd_id, dynParams, status);
    if( cmd_id == XDM_GETVERSION ) {
         ret = get_version(codec, dynParams, status, OMAP_DCE_VIDDEC3);
    } else {
         ret = control(codec, cmd_id, dynParams, status, OMAP_DCE_VIDDEC3);
    }
    DEBUG("dynParams->putDataFxn %p", dynParams->putDataFxn);
    DEBUG("<< ret=%d", ret);
    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);
    return (ret);
}

XDAS_Int32 VIDDEC3_process(VIDDEC3_Handle codec,
                           XDM2_BufDesc *inBufs, XDM2_BufDesc *outBufs,
                           VIDDEC3_InArgs *inArgs, VIDDEC3_OutArgs *outArgs)
{
    XDAS_Int32 ret;
    pthread_attr_t attr;
    int id;

    DEBUG(">> codec=%p, inBufs=%p, outBufs=%p, inArgs=%p, outArgs=%p",
          codec, inBufs, outBufs, inArgs, outArgs);

    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);
    id = get_callback((Uint32) codec);
    if( id < 0 ) {
        DEBUG("Received VIDDEC3_process for ENTIRE/FULL FRAME decoding.");
    } else {
        pthread_mutex_lock(&dce_callback_mutex);
        DEBUG("Checking row_mode %d SETTING (callbackmsg[id]).putDataFlag = 0", (callbackmsg[id]).row_mode);
        if( (callbackmsg[id]).row_mode ) {
            (callbackmsg[id]).putDataFlag = 0;
            DEBUG("Checking callbackmsg[%d]->putDataFxn_thread %p", id, (void*) (callbackmsg[id]).putDataFxn_thread);
            if( !(callbackmsg[id]).putDataFxn_thread ) {
                /* Need to start a new thread for the callback handling to request for data - process call will be synchronous. */
                pthread_attr_init(&attr);
                pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
                if( pthread_create(&((callbackmsg[id]).putDataFxn_thread), &attr, (void*)dce_callback_putDataFxn, (void*) &codec) ) {
                    pthread_mutex_unlock(&ipc_mutex);
                    return DCE_EXDM_FAIL;
                }
            }
            DEBUG("Checking thread callbackmsg[%d]->putDataFxn_thread %p", id, (void*) (callbackmsg[id]).putDataFxn_thread);

            /* Start the callback to get putDataFxn from codec to client. */
            (callbackmsg[id]).putDataFlag = 1;
            sem_post(&((callbackmsg[id]).sem_dec_row_mode));

            DEBUG("Start the callback to client callbackmsg[%d]->putDataFlag %d on callbackmsg[%d]->local_dataSyncHandle 0x%x",
                id, (callbackmsg[id]).putDataFlag, id, (unsigned int) (callbackmsg[id]).local_dataSyncHandle);
        }
        pthread_mutex_unlock(&dce_callback_mutex);
    }

    ret = process(codec, inBufs, outBufs, inArgs, outArgs, OMAP_DCE_VIDDEC3);
    DEBUG("<< ret=%d", ret);

    if( (id >= 0) && ((callbackmsg[id]).row_mode) ) {
        DEBUG("(callbackmsg[%d]).receive_numBlocks %d >= (callbackmsg[%d]).total_numBlocks %d",
            id, (callbackmsg[id]).receive_numBlocks, id, (callbackmsg[id]).total_numBlocks);

        if( (callbackmsg[id]).receive_numBlocks >= (callbackmsg[id]).total_numBlocks ) {
            DEBUG("Stop the callback to client callbackmsg[%d]->putDataFlag %d reach full frame (callbackmsg[%d]).receive_numBlocks %d",
                id, (callbackmsg[id]).putDataFlag, id, (callbackmsg[id]).receive_numBlocks);
            (callbackmsg[id]).putDataFlag = 0;
            if ((callbackmsg[id]).receive_numBlocks) {
                DEBUG("Waiting for dce_callback_putDataFxn to be in waiting case.");
                sem_wait(&((callbackmsg[id]).sem_dec_row_mode));
            }
        }
    }

    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);
    return (ret);
}

Void VIDDEC3_delete(VIDDEC3_Handle codec)
{
    void *res;
    int id;

    DEBUG(">> codec=%p", codec);

    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);

    id = get_callback((Uint32) codec);
    if( id < 0 ) {
        DEBUG("Delete decode instance in full frame mode");
    } else {
        if( (callbackmsg[id]).row_mode ) {
            MmRpcCallback_count--;
            /* Exit the callback thread to request to client */
            (callbackmsg[id]).putDataFlag = 2;
            DEBUG("Exit the callback to client callbackmsg[%d]->getDataFlag %d callbackmsg[%d]->getDataFxn_thread %p",
                id, (callbackmsg[id]).putDataFlag, id, (void*) ((callbackmsg[id]).putDataFxn_thread));
            sem_post(&((callbackmsg[id]).sem_dec_row_mode));

            pthread_join((callbackmsg[id]).putDataFxn_thread, (void*) &res);
            DEBUG("PTHREAD_JOIN res %d", (int) res);

            /* Clean up the allocation earlier. */
            memplugin_free((callbackmsg[id]).local_dataSyncDesc);

            /* Destroy sem_dec_row_mode */
            sem_destroy(&((callbackmsg[id]).sem_dec_row_mode));

            DEBUG("Checking on MmRpcCallback_count %d MmRpcCallbackHandle 0x%x", MmRpcCallback_count, (unsigned int) MmRpcCallbackHandle);
            if( MmRpcCallback_count == 0 && MmRpcCallbackHandle != NULL ) {
                MmRpc_delete(&MmRpcCallbackHandle);
                MmRpcCallbackHandle = NULL;
            }
        }
    }

    delete(codec, OMAP_DCE_VIDDEC3);
    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);
    DEBUG("<<");
}

/***************** VIDENC2 Encoder Codec Engine Functions ****************/
VIDENC2_Handle VIDENC2_create(Engine_Handle engine, String name,
                              VIDENC2_Params *params)
{
    VIDENC2_Handle codec = NULL;
    MmRpc_Params args;
    dce_error_status eError = DCE_EOK;
    int id;

    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);

    id = get_callback(0);
    if( id < 0 ) {
        /* This is depended on the MAX_INSTANCE, by default it handles only 5 instances of codec instance (full frame or low latency) */
        ERROR("Failed because too many codec clients, Max is %d. MAX_INSTANCES default needs to be changed if required.", MAX_INSTANCES);
        goto EXIT;
    } else { /* Found empty array to be populated */
        if( params->inputDataMode == IVIDEO_NUMROWS ) {
            (callbackmsg[id]).row_mode = 1;
            (callbackmsg[id]).first_control = TRUE;
            (callbackmsg[id]).total_numBlocks = params->maxHeight / 16;
            DEBUG("callbackmsg[%d]->total_numBlocks %d", id, (callbackmsg[id]).total_numBlocks);

            /* Create sem_dec_row_mode */
            sem_init(&((callbackmsg[id]).sem_enc_row_mode), 0, 0);

            (callbackmsg[id]).local_dataSyncDesc = memplugin_alloc(sizeof(XDM_DataSyncDesc), 1, DEFAULT_REGION, 0, IPU);
            DEBUG("Checking local_dataSyncDesc %p local_get_DataFxn %p local_dataSyncHandle %p",
                (callbackmsg[id]).local_dataSyncDesc, (callbackmsg[id]).local_get_DataFxn, (callbackmsg[id]).local_dataSyncHandle);
            if( (callbackmsg[id]).local_dataSyncDesc == NULL ) {
                goto EXIT;
            }

            if( !MmRpcCallbackHandle ) {
                /* Need to create another MmRpcHandle for codec callback */
                MmRpc_Params_init(&args);
                eError = MmRpc_create(DCE_CALLBACK_NAME, &args, &MmRpcCallbackHandle);
                _ASSERT_AND_EXECUTE(eError == DCE_EOK, DCE_EIPC_CREATE_FAIL, MmRpcCallbackHandle = NULL);
                DEBUG("open(/dev/%s]) -> 0x%x\n", DCE_CALLBACK_NAME, (int)MmRpcCallbackHandle);
            }
            MmRpcCallback_count++;
        } else if( params->inputDataMode == IVIDEO_ENTIREFRAME ) {
            (callbackmsg[id]).row_mode = 0; /* full frame; the other parameters are not used in full frame mode */
        } else {
            ERROR("inputDataMode %d is not supported.", params->inputDataMode);
            goto EXIT;
        }
        DEBUG("Checking row_mode %d first_control %d", (callbackmsg[id]).row_mode, (callbackmsg[id]).first_control);
    }

    DEBUG(">> engine=%p, name=%s, params=%p", engine, name, params);
    codec = create(engine, name, params, OMAP_DCE_VIDENC2);
    DEBUG("<< codec=%p", codec);

    if( params->inputDataMode == IVIDEO_NUMROWS ) {
        (callbackmsg[id]).codec_handle = (XDAS_UInt32) codec;
    }

EXIT:
    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);
    return (codec);
}

XDAS_Int32 VIDENC2_control(VIDENC2_Handle codec, VIDENC2_Cmd cmd_id,
                           VIDENC2_DynamicParams *dynParams, VIDENC2_Status *status)
{
    XDAS_Int32 ret;
    int id;

    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);

    id = get_callback((Uint32) codec);
    if( id < 0 ) {
        DEBUG("Could not find the entry in callbackmsg array; might not be rowmode; should be full frame mode");
    } else {
        DEBUG("Checking row_mode %d first_control %d", (callbackmsg[id]).row_mode, (callbackmsg[id]).first_control);
        if( (callbackmsg[id]).row_mode && (callbackmsg[id]).first_control ) {
            /* dynParams has the function callback; store the information as it will get overwritten by the M4 codec for their own callback Fxn. */
            (callbackmsg[id]).local_get_DataFxn = (void*) dynParams->getDataFxn;
            (callbackmsg[id]).local_dataSyncHandle = dynParams->getDataHandle;
            (callbackmsg[id]).first_control = FALSE;
            DEBUG("Set callback pointer local_get_dataFxn %p local_dataSyncHandle %p", (callbackmsg[id]).local_get_DataFxn, (callbackmsg[id]).local_dataSyncHandle);
        }
    }

    DEBUG(">> codec=%p, cmd_id=%d, dynParams=%p, status=%p",
          codec, cmd_id, dynParams, status);
    if( cmd_id == XDM_GETVERSION ) {
         ret = get_version(codec, dynParams, status, OMAP_DCE_VIDENC2);
    } else {
         ret = control(codec, cmd_id, dynParams, status, OMAP_DCE_VIDENC2);
    }

    DEBUG("<< ret=%d", ret);
    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);
    return (ret);
}

XDAS_Int32 VIDENC2_process(VIDENC2_Handle codec,
                           IVIDEO2_BufDesc *inBufs, XDM2_BufDesc *outBufs,
                           VIDENC2_InArgs *inArgs, VIDENC2_OutArgs *outArgs)
{
    XDAS_Int32 ret = 0;
    pthread_attr_t attr;
    int id;

    DEBUG(">> codec=%p, inBufs=%p, outBufs=%p, inArgs=%p, outArgs=%p",
          codec, inBufs, outBufs, inArgs, outArgs);

    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);

    id = get_callback((Uint32) codec);
    if( id < 0 ) {
        DEBUG("Received VIDENC2_process for ENTIRE FRAME encoding because no entry found in callbackmsg");
    } else {
        DEBUG("Checking row_mode %d", (callbackmsg[id]).row_mode);
        if( (callbackmsg[id]).row_mode ) {
            (callbackmsg[id]).getDataFlag = 0;
            DEBUG("Checking callbackmsg[%d]->getDataFxn_thread 0x%x", id, (callbackmsg[id]).getDataFxn_thread);
            if( !(callbackmsg[id]).getDataFxn_thread ) {
                /* Need to start a new thread for the callback handling to request for data - process call will be synchronous. */
                pthread_attr_init(&attr);
                pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
                if( pthread_create(&((callbackmsg[id]).getDataFxn_thread), &attr, (void*)dce_callback_getDataFxn, (void*) &codec) ) {
                    pthread_mutex_unlock(&ipc_mutex);
                    return DCE_EXDM_FAIL;
                }
            }
            DEBUG("Create thread callbackmsg[%d]->getDataFxn_thread 0x%x", id, (unsigned int) (callbackmsg[id]).getDataFxn_thread);

            /* Start the callback to request to client */
            (callbackmsg[id]).getDataFlag = 1;
            sem_post(&((callbackmsg[id]).sem_enc_row_mode));
            DEBUG("Start the callback to client (callbackmsg[%d]).getDataFlag %d on (callbackmsg[%d]).local_dataSyncHandle 0x%x",
                id, (callbackmsg[id]).getDataFlag, id, (unsigned int) (callbackmsg[id]).local_dataSyncHandle);
        }
    }

    ret = process(codec, inBufs, outBufs, inArgs, outArgs, OMAP_DCE_VIDENC2);
    DEBUG("<< ret=%d", ret);

    if( (id >= 0) && ((callbackmsg[id]).row_mode) ) {
        /* Stop the callback to request to client */
        DEBUG("Stop the callback to client callbackmsg[%d]->getDataFlag %d", id, (callbackmsg[id]).getDataFlag);
        (callbackmsg[id]).getDataFlag = 0;
    }

    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);
    return (ret);
}

Void VIDENC2_delete(VIDENC2_Handle codec)
{
    void *res;
    int id;

    DEBUG(">> codec=%p", codec);

    /*Acquire permission to use IPC*/
    pthread_mutex_lock(&ipc_mutex);

    id = get_callback((Uint32) codec);
    if( id < 0 ) {
        DEBUG("Delete encode instance in full frame mode");
    } else {
        if( (callbackmsg[id]).row_mode ) {
            MmRpcCallback_count--;
            /* Exit the callback thread to request to client */
            (callbackmsg[id]).getDataFlag = 2;
            DEBUG("Exit the callback to client (callbackmsg[%d]).getDataFlag %d (callbackmsg[%d]).getDataFxn_thread 0x%x",
                id, (callbackmsg[id]).getDataFlag, id, (unsigned int) (callbackmsg[id]).getDataFxn_thread);
            sem_post(&((callbackmsg[id]).sem_enc_row_mode));

            pthread_join((callbackmsg[id]).getDataFxn_thread, (void*) &res);
            DEBUG("PTHREAD_JOIN res %d", (int) res);

            /* Clean up the allocation earlier. */
            memplugin_free((callbackmsg[id]).local_dataSyncDesc);

            /* Destroy sem_dec_row_mode */
            sem_destroy(&((callbackmsg[id]).sem_enc_row_mode));

            DEBUG("Check on MmRpcCallback_count %d", MmRpcCallback_count);
            if( MmRpcCallback_count == 0 && MmRpcCallbackHandle != NULL ) {
                MmRpc_delete(&MmRpcCallbackHandle);
                MmRpcCallbackHandle = NULL;
            }
        }
    }

    delete(codec, OMAP_DCE_VIDENC2);
    /*Relinquish IPC*/
    pthread_mutex_unlock(&ipc_mutex);
    DEBUG("<<");
}

/***************** VIDDEC2 Decoder Codec Engine Functions ****************/
VIDDEC2_Handle VIDDEC2_create(Engine_Handle engine, String name,
                              VIDDEC2_Params *params)
{
    VIDDEC2_Handle    codec;

    DEBUG(">> engine=%p, name=%s, params=%p", engine, name, params);
    codec = create(engine, name, params, OMAP_DCE_VIDDEC2);
    DEBUG("<< codec=%p", codec);
    return (codec);
}

XDAS_Int32 VIDDEC2_control(VIDDEC2_Handle codec, VIDDEC2_Cmd id,
                           VIDDEC2_DynamicParams *dynParams, VIDDEC2_Status *status)
{
    XDAS_Int32    ret;

    DEBUG(">> codec=%p, id=%d, dynParams=%p, status=%p",
          codec, id, dynParams, status);
    if( id == XDM_GETVERSION ) {
         ret = get_version(codec, dynParams, status, OMAP_DCE_VIDDEC2);
    } else {
         ret = control(codec, id, dynParams, status, OMAP_DCE_VIDDEC2);
    }
    DEBUG("<< ret=%d", ret);
    return (ret);
}

XDAS_Int32 VIDDEC2_process(VIDDEC2_Handle codec,
                           XDM1_BufDesc *inBufs, XDM_BufDesc *outBufs,
                           VIDDEC2_InArgs *inArgs, VIDDEC2_OutArgs *outArgs)
{
    XDAS_Int32    ret;

    DEBUG(">> codec=%p, inBufs=%p, outBufs=%p, inArgs=%p, outArgs=%p",
          codec, inBufs, outBufs, inArgs, outArgs);
    ret = process(codec, inBufs, outBufs, inArgs, outArgs, OMAP_DCE_VIDDEC2);
    DEBUG("<< ret=%d", ret);
    return (ret);
}

Void VIDDEC2_delete(VIDDEC2_Handle codec)
{
    DEBUG(">> codec=%p", codec);
    delete(codec, OMAP_DCE_VIDDEC2);
    DEBUG("<<");
}
