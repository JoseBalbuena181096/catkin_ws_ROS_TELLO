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
 * @file libARSAL/ARSAL_Time.c
 * @brief This file contains sources about time abstraction layer
 * @date 05/18/2012
 * @author frederic.dhaeyer@parrot.com
 */
#include <config.h>
#include <libARSAL/ARSAL_Time.h>

int ARSAL_Time_GetTime (struct timespec *res)
{
    int result = -1;
    if (res == NULL)
    {
        return result;
    }
    /* No else --> Args check (return -1) */

#if defined(HAVE_CLOCK_GETTIME) && defined (HAVE_DECL_CLOCK_MONOTONIC)

    result = clock_gettime(CLOCK_MONOTONIC, res);

#elif defined(HAVE_GETTIMEOFDAY)

    struct timeval tv;
    result = gettimeofday(&tv, NULL);
    TIMEVAL_TO_TIMESPEC(&tv, res);

#else

    time_t t = time(NULL);
    if (t < 0)
    {
        result = (int)t;
    }
    else
    {
        result = 0;
        res->tv_sec = t;
        res->tv_nsec = 0;
    }

#endif

    return result;
}

int ARSAL_Time_GetLocalTime (struct timespec *res, struct tm *localTime)
{
    int result = -1;
    if (res == NULL && localTime == NULL)
    {
        return result;
    }
    /* No else --> Args check (return -1) */

    struct timespec ts;

#if defined(HAVE_CLOCK_GETTIME) && defined (HAVE_DECL_CLOCK_REALTIME)

    result = clock_gettime(CLOCK_REALTIME, &ts);

#elif defined(HAVE_GETTIMEOFDAY)

    struct timeval tv;
    result = gettimeofday(&tv, NULL);
    TIMEVAL_TO_TIMESPEC(&tv, &ts);

#else

    time_t t = time(NULL);
    if (t < 0)
    {
        result = (int)t;
    }
    else
    {
        result = 0;
        ts.tv_sec = t;
        ts.tv_nsec = 0;
    }

#endif

    if (result == 0)
    {
        if (res != NULL)
        {
            res->tv_sec = ts.tv_sec;
            res->tv_nsec = ts.tv_nsec;
        }
        /* No else --> Do nothing if res is NULL */
        if (localTime != NULL)
        {
            localtime_r (&(ts.tv_sec), localTime);
        }
        /* No else --> Do nothing if localTime is NULL */
    }
    /* No else --> Do nothing if the gettime failed */

    return result;
}

int ARSAL_Time_TimevalEquals (struct timeval *t1, struct timeval *t2)
{
    int result = 0;
    if ((t1 == NULL) ||
        (t2 == NULL))
    {
        return result;
    }
    /* No else --> Args check (return 0) */

    if ((t1->tv_sec == t2->tv_sec) &&
        (t1->tv_usec == t2->tv_usec))
    {
        result = 1;
    }
    /* Else the Timevals are not equal ; result = 0 */

    return result;
}

int ARSAL_Time_TimespecEquals (struct timespec *t1, struct timespec *t2)
{
    int result = 0;
    if ((t1 == NULL) ||
        (t2 == NULL))
    {
        return result;
    }
    /* No else --> Args check (return 0) */

    if ((t1->tv_sec == t2->tv_sec) &&
        (t1->tv_nsec == t2->tv_nsec))
    {
        result = 1;
    }
    /* Else the Timespecs are not equal ; result = 0 */

    return result;
}

int32_t ARSAL_Time_ComputeTimevalMsTimeDiff (struct timeval *start, struct timeval *end)
{
    int32_t result = -1;
    struct timeval diff;
    if ((start == NULL) ||
        (end == NULL))
    {
        return result;
    }
    /* No else --> Args check (return -1) */

    diff.tv_sec = end->tv_sec - start->tv_sec;
    if (start->tv_usec > end->tv_usec)
    {
        diff.tv_sec--;
        diff.tv_usec = SEC_TO_USEC(1) - (start->tv_usec - end->tv_usec);
    }
    else
    {
        diff.tv_usec = end->tv_usec - start->tv_usec;
    }

    result = SEC_TO_MSEC(diff.tv_sec) + USEC_TO_MSEC(diff.tv_usec);

    return result;
}

int32_t ARSAL_Time_ComputeTimespecMsTimeDiff (struct timespec *start, struct timespec *end)
{
    int32_t result = -1;
    struct timespec diff;
    if ((start == NULL) ||
        (end == NULL))
    {
        return result;
    }
    /* No else --> Args check (return -1) */

    diff.tv_sec = end->tv_sec - start->tv_sec;
    if (start->tv_nsec > end->tv_nsec)
    {
        diff.tv_sec--;
        diff.tv_nsec = SEC_TO_NSEC(1) - (start->tv_nsec - end->tv_nsec);
    }
    else
    {
        diff.tv_nsec = end->tv_nsec - start->tv_nsec;
    }

    result = SEC_TO_MSEC(diff.tv_sec) + NSEC_TO_MSEC(diff.tv_nsec);

    return result;
}
