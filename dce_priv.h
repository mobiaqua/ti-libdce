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

#ifndef __DCE_PRIV_H__
#define __DCE_PRIV_H__

extern int dce_debug;

/********************* MACROS ************************/
/***************** TRACE MACROS  *********************/
/* Need to make it OS specific and support different trace levels */

/* set desired trace level:
 *   1 - error
 *   2 - error, debug
 *   3 - error, debug, info  (very verbose)
 */
#ifdef DCE_DEBUG_ENABLE
#define ERROR(FMT,...)   TRACE(1, "ERROR: " FMT, ##__VA_ARGS__)
#define DEBUG(FMT,...)   TRACE(2, "DEBUG: " FMT, ##__VA_ARGS__)
#define INFO(FMT,...)    TRACE(3, "INFO: " FMT, ##__VA_ARGS__)
#else
#define ERROR(FMT,...)
#define DEBUG(FMT,...)
#define INFO(FMT,...)
#endif

#ifdef BUILDOS_QNX
#include <sys/slog.h>
#define TRACE(lvl,FMT, ...)  do if ((lvl) <= dce_debug) { \
        slogf(42, _SLOG_INFO, "%s:%d:\t%s\t" FMT, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__); \
} while( 0 )

#elif defined BUILDOS_LINUX
#define TRACE(lvl,FMT, ...)  do if ((lvl) <= dce_debug) { \
        printf("%s:%d:\t%s\t" FMT,__FILE__, __LINE__,__FUNCTION__ ,##__VA_ARGS__); \
}while( 0 )

#elif defined BUILDOS_ANDROID
#include <utils/Log.h>
#define LOG_TAG "libdce"
#define TRACE(lvl,FMT, ...)  do if ((lvl) <= dce_debug) { \
        ALOGE("%s:%d:\t%s\t" FMT,__FILE__, __LINE__,__FUNCTION__ ,##__VA_ARGS__); \
}while( 0 )
#endif

/***************** ASSERT MACROS *********************/
#define _ASSERT_AND_EXECUTE(_COND_, _ERRORCODE_, _EXPR_) do { \
        if( !(_COND_)) { eError = _ERRORCODE_; \
                         ERROR("Failed %s error val %d", # _COND_, _ERRORCODE_); \
                         _EXPR_; \
                         goto EXIT; } \
} while( 0 )

#define _ASSERT(_COND_, _ERRORCODE_) do { \
        if( !(_COND_)) { eError = _ERRORCODE_; \
                         ERROR("Failed %s error val %d", # _COND_, _ERRORCODE_); \
                         goto EXIT; } \
} while( 0 )

#endif /* __DCE_PRIV_H__ */

