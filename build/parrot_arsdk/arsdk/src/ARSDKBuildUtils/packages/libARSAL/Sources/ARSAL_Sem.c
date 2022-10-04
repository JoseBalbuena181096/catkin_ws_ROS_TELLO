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
 * @file libARSAL/ARSAL_Sem.c
 * @brief This file contains sources about semaphore abstraction layer
 * @date 12/27/2012
 * @author nicolas.brulez@parrot.com
 */
#include <stdlib.h>
#include <config.h>
#include <libARSAL/ARSAL_Sem.h>
#include <libARSAL/ARSAL_Time.h>
#include <errno.h>

#if defined(HAVE_SEMAPHORE_H) &&                \
    defined(HAVE_SEM_DESTROY) &&                \
    defined(HAVE_SEM_GETVALUE) &&               \
    defined(HAVE_SEM_INIT) &&                   \
    defined(HAVE_SEM_POST) &&                   \
    defined(HAVE_SEM_TIMEDWAIT) &&              \
    defined(HAVE_SEM_TRYWAIT) &&                \
    defined(HAVE_SEM_WAIT)
#define __SAL_USE_POSIX_SEM (1)
#else
#define __SAL_USE_POSIX_SEM (0)
#endif

#if __SAL_USE_POSIX_SEM
#include <semaphore.h>
#else
#include <libARSAL/ARSAL_Mutex.h>
#include <inttypes.h>

/*
 * In this case, we use a custom-built semaphore type which use a lock,
 * a condition, and a counter. Algos for each function is described within
 * functions
 */

#define ARSAL_SEM_ERRNO_TRANSFORM(RESBUF)                   \
    do                                                      \
    {                                                       \
        int __res = (RESBUF);                               \
        if (0 < __res)                                      \
        {                                                   \
            errno = __res;                                  \
            (RESBUF) = -1;                                  \
        }                                                   \
        /* No else: no error so no need to set errno. */    \
    } while (0)

typedef struct {
    ARSAL_Mutex_t lock;
    ARSAL_Cond_t cond;
    int32_t count;
} ARSAL_Sem_CustomImpl_t;
#endif

int ARSAL_Sem_Init(ARSAL_Sem_t *sem, int shared, int value)
{
    int result = -1;

    if (NULL == sem)
    {
        errno = EINVAL;
        return result;
    }
    /* No else. */

#if __SAL_USE_POSIX_SEM

    sem_t *psem = (sem_t *) calloc (1, sizeof (sem_t));
    if (NULL != psem)
    {
        result = sem_init(psem, shared, value);
        if (0 == result)
        {
            *sem = (ARSAL_Sem_t)psem;
        }
        else
        {
            free (psem);
            psem = NULL;
        }
    }
    /* No else: if calloc failed, return default value -1. */

#else

    /*
     * Custom init algo:
     * Alloc memory
     * Init mutex / condition
     * Set internal counter to 'value'
     */

    int isMalloc = 0, isMutexInit = 0;
    ARSAL_Sem_CustomImpl_t *psem = (ARSAL_Sem_CustomImpl_t *) malloc (sizeof (ARSAL_Sem_CustomImpl_t));
    if (NULL != psem)
    {
        isMalloc = 1;
        result = ARSAL_Mutex_Init (&(psem->lock));
        ARSAL_SEM_ERRNO_TRANSFORM (result);
        if (0 == result)
        {
            isMutexInit = 1;
            result = ARSAL_Cond_Init (&(psem->cond));
            ARSAL_SEM_ERRNO_TRANSFORM (result);
            psem->count = value;
        }
        /* No else. */
    }
    /* No else. */

    if (0 == result)
    {
        *sem = (ARSAL_Sem_t)psem;
    }
    else
    {
        if (1 == isMutexInit)
        {
            ARSAL_Mutex_Destroy (&(psem->lock));
        }
        /* No else: no need to destroy a mutex that wasn't created. */
        if (1 == isMalloc)
        {
            free (psem);
        }
        /* No else: no need to free memory that wasn't allocated. */
    }

#endif

    return result;
}

int ARSAL_Sem_Destroy(ARSAL_Sem_t *sem)
{
    int result = -1;

    if (NULL == sem || NULL == *sem)
    {
        errno = EINVAL;
        return result;
    }
    /* No else. It's kinda obvious why. */

#if __SAL_USE_POSIX_SEM

    result = sem_destroy((sem_t *)*sem);
    free(*sem);
    *sem = NULL;

#else

    /*
     * Custom destroy algo:
     * Broadcast signal to condition, destroy it
     * Destroy mutex
     * Free memory
     */

    ARSAL_Sem_CustomImpl_t *psem = (ARSAL_Sem_CustomImpl_t *)*sem;
    result = ARSAL_Cond_Broadcast (&(psem->cond));
    ARSAL_SEM_ERRNO_TRANSFORM (result);

    if (0 == result)
    {
        result = ARSAL_Cond_Destroy (&(psem->cond));
        ARSAL_SEM_ERRNO_TRANSFORM (result);
    }
    /* No else. */

    if (0 == result)
    {
        result = ARSAL_Mutex_Destroy (&(psem->lock));
        ARSAL_SEM_ERRNO_TRANSFORM (result);
    }
    /* No else. */

    free (*sem);
    *sem = NULL;

#endif

    return result;
}

int ARSAL_Sem_Wait(ARSAL_Sem_t *sem)
{
    int result = -1;

    if (NULL == sem || NULL == *sem)
    {
        errno = EINVAL;
        return result;
    }
    /* No else: arguments are all good so just go on. */

#if __SAL_USE_POSIX_SEM

    while (((result = sem_wait((sem_t *)*sem)) == -1) &&
           (errno == EINTR));

#else

    /*
     * Custom wait algo:
     * Lock mutex
     * Check if counter is > 0
     *  NO         YES
     *  |          | - Decrement counter
     *  |          \ - Unlock mutex
     *  | - Wait for condition
     *  | - Decrement counter
     *  \ - Unlock mutex
     */

    ARSAL_Sem_CustomImpl_t *psem = (ARSAL_Sem_CustomImpl_t *)*sem;
    int unlockRes = 0;
    result = ARSAL_Mutex_Lock (&(psem->lock));
    ARSAL_SEM_ERRNO_TRANSFORM (result);

    if (0 == result && 0 >= psem->count)
    {
        result = ARSAL_Cond_Wait (&(psem->cond), &(psem->lock));
        ARSAL_SEM_ERRNO_TRANSFORM (result);
    }
    /* No else. */

    if (0 == result)
    {
        if (0 < psem->count)
        {
            (psem->count)--;
        }
        /* No else: don't decrement count below 0. */
    }
    /* No else. */

    unlockRes = ARSAL_Mutex_Unlock (&(psem->lock));
    if (0 != unlockRes)
    {
        errno = unlockRes;
        result = -1;
    }
    /* No else. */

#endif

    return result;
}

int ARSAL_Sem_Trywait(ARSAL_Sem_t *sem)
{
    int result = -1;

    if (NULL == sem || NULL == *sem)
    {
        errno = EINVAL;
        return result;
    }
    /* No else. */

#if __SAL_USE_POSIX_SEM

    result = sem_trywait((sem_t *)*sem);

#else

    /*
     * Custom trywait algo:
     * Trylock mutex (if errno is EBUSY, set it to EAGAIN)
     * Check if counter is > 0
     *  NO         YES
     *  |          | - Decrement counter
     *  |          \ - Unlock mutex
     *  | - Set errno to EAGAIN and result to -1
     *  \ - Unlock mutex
     */

    ARSAL_Sem_CustomImpl_t *psem = (ARSAL_Sem_CustomImpl_t *)*sem;
    int unlockRes = 0;
    result = ARSAL_Mutex_Trylock (&(psem->lock));
    ARSAL_SEM_ERRNO_TRANSFORM (result);
    if (-1 == result && EBUSY == errno)
    {
        errno = EAGAIN;
    }
    /* No else. */

    if (0 == result && 0 >= psem->count)
    {
        result = -1;
        errno = EAGAIN;
    }
    /* No else. */

    if (0 == result)
    {
        if (0 < psem->count)
        {
            (psem->count)--;
        }
        /* No else: don't decrement count below 0. */
    }
    /* No else. */

    unlockRes = ARSAL_Mutex_Unlock (&(psem->lock));
    if (0 != unlockRes)
    {
        result = -1;
        errno = unlockRes;
    }
    /* No else. */

#endif

    return result;
}

int ARSAL_Sem_Timedwait(ARSAL_Sem_t *sem, const struct timespec *timeout)
{
    int result = -1;

    if (NULL == sem || NULL == *sem)
    {
        errno = EINVAL;
        return result;
    } /* MUST BE INIT TO -1 */
    /* No else. */

#if __SAL_USE_POSIX_SEM

    struct timespec finalTime = {0};
    ARSAL_Time_GetLocalTime(&finalTime, NULL);
    finalTime.tv_nsec += timeout->tv_nsec;
    finalTime.tv_sec += timeout->tv_sec + NSEC_TO_SEC(finalTime.tv_nsec);
    finalTime.tv_nsec %= SEC_TO_NSEC(1);

    while (((result = sem_timedwait((sem_t *)*sem, &finalTime)) == -1) &&
           (errno == EINTR));

#else

    /*
     * Custom timedwait algo:
     * Lock mutex
     * Check if counter is > 0
     *  NO         YES
     *  |          | - Decrement counter
     *  |          \ - Unlock mutex
     *  | - Timedwait on condition
     *  | - If timeout -> set result to -1 and errno to ETIMEDOUT
     *  | - Else       -> Decrement counter
     *  \ - Unlock mutex
     */

    ARSAL_Sem_CustomImpl_t *psem = (ARSAL_Sem_CustomImpl_t *)*sem;
    int unlockRes = 0;
    result = ARSAL_Mutex_Lock (&(psem->lock));
    ARSAL_SEM_ERRNO_TRANSFORM (result);

    if (0 == result && 0 >= psem->count)
    {
        int msToWait = SEC_TO_MSEC(timeout->tv_sec) + NSEC_TO_MSEC(timeout->tv_nsec);
        result = ARSAL_Cond_Timedwait (&(psem->cond), &(psem->lock), msToWait);
        ARSAL_SEM_ERRNO_TRANSFORM (result);
    }
    /* No else. */

    if (0 == result)
    {
        if (0 < psem->count)
        {
            (psem->count)--;
        }
        /* No else: don't decrement count below 0. */
    }
    /* No else. */

    unlockRes = ARSAL_Mutex_Unlock (&(psem->lock));
    if (0 != unlockRes)
    {
        result = -1;
        errno = unlockRes;
    }
    /* No else. */

#endif

    return result;
}

int ARSAL_Sem_Post(ARSAL_Sem_t *sem)
{
    int result = -1;

    if (NULL == sem || NULL == *sem)
    {
        errno = EINVAL;
        return result;
    }
    /* No else. */

#if __SAL_USE_POSIX_SEM

    result = sem_post ((sem_t *)*sem);

#else

    /*
     * Custom post algo:
     * Lock mutex
     * Increment counter
     * Unlock mutex
     * Signal condition
     */

    ARSAL_Sem_CustomImpl_t *psem = (ARSAL_Sem_CustomImpl_t *)*sem;
    int unlockRes = 0;
    result = ARSAL_Mutex_Lock (&(psem->lock));
    ARSAL_SEM_ERRNO_TRANSFORM (result);

    if (0 == result)
    {
        if (INT32_MAX == psem->count)
        {
            result = -1;
            errno = EOVERFLOW;
        }
        else
        {
            (psem->count)++;
        }
    }
    /* No else. */

    unlockRes = ARSAL_Mutex_Unlock (&(psem->lock));
    if (0 != unlockRes)
    {
        result = -1;
        errno = unlockRes;
    }
    /* No else. */

    if (0 == result)
    {
        result = ARSAL_Cond_Signal (&(psem->cond));
        ARSAL_SEM_ERRNO_TRANSFORM (result);
    }
    /* No else. */

#endif

    return result;
}

int ARSAL_Sem_Getvalue(ARSAL_Sem_t *sem, int *value)
{
    int result = -1;

    if (NULL == sem || NULL == *sem || NULL == value)
    {
        errno = EINVAL;
        return result;
    }
    /* No else. */

#if __SAL_USE_POSIX_SEM

    result = sem_getvalue((sem_t *)*sem, value);

#else

    /*
     * Custom getvalue algo:
     * Lock mutex
     * Read counter to *value
     * Unlock mutex
     */

    ARSAL_Sem_CustomImpl_t *psem = (ARSAL_Sem_CustomImpl_t *)*sem;
    int unlockRes = 0;
    result = ARSAL_Mutex_Lock (&(psem->lock));
    ARSAL_SEM_ERRNO_TRANSFORM (result);

    if (0 == result)
    {
        *value = psem->count;
    }
    /* No else. */

    unlockRes = ARSAL_Mutex_Unlock (&(psem->lock));
    if (0 != unlockRes)
    {
        result = -1;
        errno = unlockRes;
    }
    /* No else. */

#endif

    return result;
}
