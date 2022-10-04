/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the 
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/
/**
 * @file libARSAL/ARSAL_Mutex.c
 * @brief This file contains sources about mutex abstraction layer
 * @date 05/18/2012
 * @author frederic.dhaeyer@parrot.com
 */
#include <stdlib.h>
#include <config.h>
#include <libARSAL/ARSAL_Mutex.h>
#include <libARSAL/ARSAL_Time.h>
#include <libARSAL/ARSAL_Print.h>

#if HAVE_DECL_SYS_GETTID
#include <sys/syscall.h>
#endif
#include <errno.h>

#if defined(HAVE_PTHREAD_H)
#include <pthread.h>
#else
#error The pthread.h header is required in order to build the library
#endif

/**
 * Tag for ARSAL_PRINT
 */
#define ARSAL_MUTEX_TAG "ARSAL_Mutex"

int ARSAL_Mutex_Init(ARSAL_Mutex_t *mutex)
{
    int result = 0;

#if defined(HAVE_PTHREAD_H)
    pthread_mutex_t *pmutex = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
    *mutex = (ARSAL_Mutex_t)pmutex;
    result = pthread_mutex_init((pthread_mutex_t *)*mutex, NULL);
#endif

    return result;
}

int ARSAL_Mutex_Destroy(ARSAL_Mutex_t *mutex)
{
    int result = 0;

#if defined(HAVE_PTHREAD_H)
    result = pthread_mutex_destroy((pthread_mutex_t *)*mutex);
    free(*mutex);
#endif

    return result;
}

int ARSAL_Mutex_Lock(ARSAL_Mutex_t *mutex)
{
    int result = 0;

#if defined(HAVE_PTHREAD_H)
    result = pthread_mutex_lock((pthread_mutex_t *)*mutex);
    if (result != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_FATAL, ARSAL_MUTEX_TAG, "Mutex operation failed! errno = %d , %s ; thread_id = %ld",
                    result,
                    strerror(result),
#if HAVE_DECL_SYS_GETTID
                    (long)syscall(SYS_gettid)
#else
                    0L
#endif
            );
    }
#endif

    return result;
}

int ARSAL_Mutex_Trylock(ARSAL_Mutex_t *mutex)
{
    int result = 0;

#if defined(HAVE_PTHREAD_H)
    result = pthread_mutex_trylock((pthread_mutex_t *)*mutex);
    if ( (result != 0) && (result != EBUSY) )
    {
        ARSAL_PRINT(ARSAL_PRINT_FATAL, ARSAL_MUTEX_TAG, "Mutex operation failed! errno = %d , %s ; thread_id = %ld",
                    result,
                    strerror(result),
#if HAVE_DECL_SYS_GETTID
                    (long)syscall(SYS_gettid)
#else
                    0L
#endif
            );
    }
#endif

    return result;
}

int ARSAL_Mutex_Unlock(ARSAL_Mutex_t *mutex)
{
    int result = 0;

#if defined(HAVE_PTHREAD_H)
    result = pthread_mutex_unlock((pthread_mutex_t *)*mutex);
    if (result != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_FATAL, ARSAL_MUTEX_TAG, "Mutex operation failed! errno = %d , %s ; thread_id = %ld",
                    result,
                    strerror(result),
#if HAVE_DECL_SYS_GETTID
                    (long)syscall(SYS_gettid)
#else
                    0L
#endif
            );
    }
#endif

    return result;
}

int ARSAL_Cond_Init(ARSAL_Cond_t *cond)
{
    int result = 0;

#if defined(HAVE_PTHREAD_H)
    pthread_cond_t *pcond = (pthread_cond_t *)malloc(sizeof(pthread_cond_t));
    *cond = (ARSAL_Cond_t)pcond;
    result = pthread_cond_init((pthread_cond_t *)*cond, NULL);
#endif

    return result;
}

int ARSAL_Cond_Destroy(ARSAL_Cond_t *cond)
{
    int result = 0;

#if defined(HAVE_PTHREAD_H)
    result = pthread_cond_destroy((pthread_cond_t *)*cond);
    free(*cond);
#endif

    return result;
}

int ARSAL_Cond_Wait(ARSAL_Cond_t *cond, ARSAL_Mutex_t *mutex)
{
    int result = 0;

#if defined(HAVE_PTHREAD_H)
    result = pthread_cond_wait((pthread_cond_t *)*cond, (pthread_mutex_t *)*mutex);
    if (result != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_FATAL, ARSAL_MUTEX_TAG, "Mutex/Cond operation failed! errno = %d , %s ; thread_id = %ld",
                    result,
                    strerror(result),
#if HAVE_DECL_SYS_GETTID
                    (long)syscall(SYS_gettid)
#else
                    0L
#endif
            );
    }
#endif

    return result;
}

int ARSAL_Cond_Timedwait(ARSAL_Cond_t *cond, ARSAL_Mutex_t *mutex, int timeout)
{
    int result = 0;
    struct timespec ts;
    ARSAL_Time_GetLocalTime(&ts, NULL);
    ts.tv_nsec += MSEC_TO_NSEC(timeout % SEC_TO_MSEC(1));
    ts.tv_sec  += MSEC_TO_SEC(timeout);
    ts.tv_sec  += NSEC_TO_SEC(ts.tv_nsec);
    ts.tv_nsec %= SEC_TO_NSEC(1);

#if defined(HAVE_PTHREAD_H)
    result = pthread_cond_timedwait((pthread_cond_t *)*cond, (pthread_mutex_t *)*mutex, &ts);
    if ( (result != 0) && (result != ETIMEDOUT) )
    {
        ARSAL_PRINT(ARSAL_PRINT_FATAL, ARSAL_MUTEX_TAG, "Mutex/Cond operation failed! errno = %d , %s ; thread_id = %ld",
                    result,
                    strerror(result),
#if HAVE_DECL_SYS_GETTID
                    (long)syscall(SYS_gettid)
#else
                    0L
#endif
            );
    }
#endif

    return result;
}

int ARSAL_Cond_Signal(ARSAL_Cond_t *cond)
{
    int result = 0;

#if defined(HAVE_PTHREAD_H)
    result = pthread_cond_signal((pthread_cond_t *)*cond);
    if (result != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_FATAL, ARSAL_MUTEX_TAG, "Mutex/Cond operation failed! errno = %d , %s ; thread_id = %ld",
                    result,
                    strerror(result),
#if HAVE_DECL_SYS_GETTID
                    (long)syscall(SYS_gettid)
#else
                    0L
#endif
            );
    }
#endif

    return result;
}

int ARSAL_Cond_Broadcast(ARSAL_Cond_t *cond)
{
    int result = 0;

#if defined(HAVE_PTHREAD_H)
    result = pthread_cond_broadcast((pthread_cond_t *)*cond);
    if (result != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_FATAL, ARSAL_MUTEX_TAG, "Mutex/Cond operation failed! errno = %d , %s ; thread_id = %ld",
                    result,
                    strerror(result),
#if HAVE_DECL_SYS_GETTID
                    (long)syscall(SYS_gettid)
#else
                    0L
#endif
            );
    }
#endif

    return result;
}
