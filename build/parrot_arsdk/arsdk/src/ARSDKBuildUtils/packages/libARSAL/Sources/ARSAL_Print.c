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
 * @file libARSAL/ARSAL_Print.c
 * @brief This file contains sources about debug print abstraction layer
 * @date 06/01/2012
 * @author frederic.dhaeyer@parrot.com
 */
#include <config.h>
#ifdef HAVE_ANDROID_LOG_H
#include <android/log.h>
#else
#endif
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <libARSAL/ARSAL_Print.h>

#if defined(DEBUG)
static eARSAL_PRINT_LEVEL minLevel = ARSAL_PRINT_VERBOSE;
#else
static eARSAL_PRINT_LEVEL minLevel = ARSAL_PRINT_INFO;
#endif

static ARSAL_Print_Callback_t ARSAL_Print_Callback = NULL;


static const char* cARSAL_Print_prefixTable[ARSAL_PRINT_MAX] =
{
    [ARSAL_PRINT_FATAL]   = "[WTF]",
    [ARSAL_PRINT_ERROR]   = "[ERR]",
    [ARSAL_PRINT_WARNING] = "[WNG]",
    [ARSAL_PRINT_INFO]    = "[INF]",
    [ARSAL_PRINT_DEBUG]   = "[DBG]",
    [ARSAL_PRINT_VERBOSE] = "[VRB]",
};

static int ARSAL_Print_PrintRaw_VA(eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va) ARSAL_ATTRIBUTE_FORMAT_PRINTF(3, 0);

#ifdef HAVE_ANDROID_LOG_H
static int ARSAL_Print_PrintRaw_VA(eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va)
{
    int result = -1;

    switch(level)
    {
    case ARSAL_PRINT_FATAL:
        result = __android_log_vprint (ANDROID_LOG_FATAL, tag, format, va);
        break;
    case ARSAL_PRINT_ERROR:
        result = __android_log_vprint (ANDROID_LOG_ERROR, tag, format, va);
        break;
    case ARSAL_PRINT_WARNING:
        result = __android_log_vprint (ANDROID_LOG_WARN, tag, format, va);
        break;
    case ARSAL_PRINT_INFO:
        result = __android_log_vprint (ANDROID_LOG_INFO, tag, format, va);
        break;
    case ARSAL_PRINT_DEBUG:
        result = __android_log_vprint (ANDROID_LOG_DEBUG, tag, format, va);
        break;
    case ARSAL_PRINT_VERBOSE:
        result = __android_log_vprint (ANDROID_LOG_VERBOSE, tag, format, va);
        break;

    default:
        break;
    }

    return result;
}
#else

static int ARSAL_Print_PrintRaw_VA(eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va)
{
    int result = -1;

    switch(level)
    {
    case ARSAL_PRINT_ERROR:
    case ARSAL_PRINT_FATAL:
        fprintf (stderr, "%s %s | ", cARSAL_Print_prefixTable [level], tag);
        result = vfprintf(stderr, format, va);
        break;
    case ARSAL_PRINT_WARNING:
    case ARSAL_PRINT_INFO:
    case ARSAL_PRINT_DEBUG:
    case ARSAL_PRINT_VERBOSE:
        printf ("%s %s | ", cARSAL_Print_prefixTable [level], tag);
        result = vprintf(format, va);
        break;

    default:
        break;
    }

    return result;
}
#endif

int ARSAL_Print_SetMinimumLevel(eARSAL_PRINT_LEVEL ulevel)
{
    int res = 1;
    int level = ulevel;

    if (level <= ARSAL_PRINT_VERBOSE &&
        level >= ARSAL_PRINT_FATAL)
    {
        minLevel = level;
        res = 0;
    }

    return res;
}

eARSAL_PRINT_LEVEL ARSAL_Print_GetMinimumLevel(void)
{
    return minLevel;
}

void ARSAL_Print_SetCallback( ARSAL_Print_Callback_t callback)
{
    ARSAL_Print_Callback = callback;
}

const char* ARSAL_Print_GetLevelDescription(eARSAL_PRINT_LEVEL level)
{
    eARSAL_PRINT_LEVEL levelToDescribe;
    
    switch(level)
    {
        case ARSAL_PRINT_ERROR:
        case ARSAL_PRINT_FATAL:
        case ARSAL_PRINT_WARNING:
        case ARSAL_PRINT_INFO:
        case ARSAL_PRINT_DEBUG:
        case ARSAL_PRINT_VERBOSE:
            levelToDescribe = level;
            break;
            
        default:
            levelToDescribe = ARSAL_PRINT_FATAL;
            break;
    }
    
    return cARSAL_Print_prefixTable[levelToDescribe];
}

int ARSAL_Print_PrintRaw(eARSAL_PRINT_LEVEL level, const char *tag, const char *format, ...)
{
    int result = -1;
    va_list va;

    if (level > minLevel)
    {
        return result;
    }

    va_start(va, format);
    {
        if ( ARSAL_Print_Callback == NULL )
            result = ARSAL_Print_PrintRaw_VA(level, tag, format, va);
        else
            result = ARSAL_Print_Callback(level, tag, format, va);
    }
    va_end(va);

    return result;
}

int ARSAL_Print_PrintRawEx(eARSAL_PRINT_LEVEL level, const char *func, int line, const char *tag, const char *format, ...)
{
    char nowTimeStr[ARSAL_PRINT_DATE_STRING_LENGTH];
    char msg[512];
    struct timespec ts;
    struct tm tm;
    va_list va;
    int len = 0;

    ARSAL_Time_GetLocalTime(&ts, &tm);
    strftime(nowTimeStr, ARSAL_PRINT_DATE_STRING_LENGTH, "%H:%M:%S", &tm);

    va_start(va, format);
    len = vsnprintf(msg, sizeof(msg), format, va);
    va_end(va);

    return ARSAL_Print_PrintRaw(level, tag, "%s:%03d | %s:%d - %s%s",
            nowTimeStr, (int)NSEC_TO_MSEC(ts.tv_nsec), func, line, msg,
            len <= 0 || ((size_t)len) > (sizeof(msg) - 1) || msg[len - 1] != '\n' ? "\n" : "");
}

void ARSAL_Print_DumpData(FILE *file, uint8_t tag, const void *data, size_t size, size_t sizeDump, const struct timespec *ts)
{
    uint64_t timestampUS = 0;
    struct timespec ts2;
    struct {
        uint8_t magic;
        uint8_t tag;
        uint8_t size[4];
        uint8_t sizeDump[4];
        uint8_t ts[6];
    } header;

    if (file == NULL || data == NULL)
        return;

    if (sizeDump == 0)
        sizeDump = size;

    if (ts == NULL)
        ARSAL_Time_GetTime(&ts2);
    else
        ts2 = *ts;
    timestampUS = (uint64_t)ts2.tv_sec * 1000 * 1000 + ts2.tv_nsec / 1000;

    /* Setup header */
    header.magic = '!';
    header.tag = tag;
    header.size[0] = size & 0xff;
    header.size[1] = (size >> 8) & 0xff;
    header.size[2] = (size >> 16) & 0xff;
    header.size[3] = (size >> 24) & 0xff;
    header.sizeDump[0] = sizeDump & 0xff;
    header.sizeDump[1] = (sizeDump >> 8) & 0xff;
    header.sizeDump[2] = (sizeDump >> 16) & 0xff;
    header.sizeDump[3] = (sizeDump >> 24) & 0xff;
    header.ts[0] = timestampUS & 0xff;
    header.ts[1] = (timestampUS >> 8) & 0xff;
    header.ts[2] = (timestampUS >> 16) & 0xff;
    header.ts[3] = (timestampUS >> 24) & 0xff;
    header.ts[4] = (timestampUS >> 32) & 0xff;
    header.ts[5] = (timestampUS >> 40) & 0xff;

    /* Write header and data without thread mix */
    flockfile(file);
    fwrite(&header, 1, sizeof(header), file);
    fwrite(data, 1, sizeDump, file);
    funlockfile(file);
}

void ARSAL_Print_DumpRotateFiles(const char *basePath, int count)
{
    char file0[512] = "";
    char file1[512] = "";

    int i = 0;
    if (basePath == NULL)
        return;

    for (i = count; i > 0; i--)
    {
        snprintf(file1, sizeof(file1), "%s.%d", basePath, i);
        if (i == 1)
        {
            snprintf(file0, sizeof(file0), "%s", basePath);
        }
        else
        {
            snprintf(file0, sizeof(file0), "%s.%d", basePath, i - 1);
        }

        if (rename(file0, file1)  < 0 && errno != ENOENT)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, "Dump", "Failed to rename '%s' in '%s': err=%d(%s)",
                    file0, file1, errno, strerror(errno));
        }
    }
}
