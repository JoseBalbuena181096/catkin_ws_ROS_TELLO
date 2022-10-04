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
 * @file ARDATATRANSFER_MediasQueue.c
 * @brief libARDataTransfer MediasQueue c file.
 * @date 19/12/2013
 * @author david.flattin.ext@parrot.com
 **/

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <libARSAL/ARSAL_Mutex.h>
#include <libARSAL/ARSAL_Print.h>

#include "libARDataTransfer/ARDATATRANSFER_Error.h"
#include "libARDataTransfer/ARDATATRANSFER_Manager.h"
#include "libARDataTransfer/ARDATATRANSFER_DataDownloader.h"
#include "libARDataTransfer/ARDATATRANSFER_MediasDownloader.h"
#include "ARDATATRANSFER_MediasQueue.h"

#define ARDATATRANSFER_MEDIASQUEUE_TAG          "MediasQueue"

/*****************************************
 *
 *             Private implementation:
 *
 *****************************************/

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasQueue_New(ARDATATRANSFER_MediasQueue_t *queue)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    int resultSys = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIASQUEUE_TAG, "%s", "");

    if (queue == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK)
    {
        memset(queue, 0, sizeof(ARDATATRANSFER_MediasQueue_t));
    }

    if (result == ARDATATRANSFER_OK)
    {
        resultSys = ARSAL_Mutex_Init(&queue->lock);

        if (resultSys != 0)
        {
            result = ARDATATRANSFER_ERROR_SYSTEM;
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        queue->medias = (ARDATATRANSFER_FtpMedia_t **)calloc(ARDATATRANSFER_MEDIA_QUEUE_SIZE, sizeof(ARDATATRANSFER_FtpMedia_t *));

        if (queue->medias == NULL)
        {
            result = ARDATATRANSFER_ERROR_ALLOC;
        }
        else
        {
            queue->count = ARDATATRANSFER_MEDIA_QUEUE_SIZE;
        }
    }

    if (result != ARDATATRANSFER_OK)
    {
        ARDATATRANSFER_MediasQueue_Delete(queue);
    }

    return result;
}

void ARDATATRANSFER_MediasQueue_Delete(ARDATATRANSFER_MediasQueue_t *queue)
{
    int i;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIASQUEUE_TAG, "%s", "");

    if (queue != NULL)
    {
        ARSAL_Mutex_Lock(&queue->lock);
        if (queue->medias != NULL)
        {
            for (i=0; i<queue->count; i++)
            {
                ARDATATRANSFER_FtpMedia_t *media = queue->medias[i];

                if (media != NULL)
                {
                    queue->medias[i] = NULL;
                    free(media);
                }
            }
        }
        ARSAL_Mutex_Unlock(&queue->lock);

        ARSAL_Mutex_Destroy(&queue->lock);

        if (queue->medias != NULL)
        {
            free(queue->medias);
            queue->medias = NULL;
        }
    }
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasQueue_Add(ARDATATRANSFER_MediasQueue_t *queue, ARDATATRANSFER_FtpMedia_t *ftpMedia)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    int index = -1;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIASQUEUE_TAG, "%s", "");

    if ((queue == NULL) || (ftpMedia == NULL))
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK)
    {
        ARSAL_Mutex_Lock(&queue->lock);

        result = ARDATATRANSFER_MediasQueue_GetFreeIndex(queue, &index);

        if (result == ARDATATRANSFER_OK)
        {
            queue->medias[index] = ftpMedia;
        }

        ARSAL_Mutex_Unlock(&queue->lock);
    }

    return result;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasQueue_RemoveAll(ARDATATRANSFER_MediasQueue_t *queue)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    ARDATATRANSFER_FtpMedia_t *ftpMedia = NULL;
    int i;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIASQUEUE_TAG, "%s", "");

    if (queue == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK)
    {
        ARSAL_Mutex_Lock(&queue->lock);

        for (i=0; i<queue->count; i++)
        {
            ftpMedia = queue->medias[i];

            if (ftpMedia)
            {
                free(ftpMedia);
                queue->medias[i] = NULL;
            }
        }

        ARSAL_Mutex_Unlock(&queue->lock);
    }

    return result;
}

ARDATATRANSFER_FtpMedia_t * ARDATATRANSFER_MediasQueue_Pop(ARDATATRANSFER_MediasQueue_t *queue, eARDATATRANSFER_ERROR *error)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    ARDATATRANSFER_FtpMedia_t *ftpMedia = NULL;
    int i;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIASQUEUE_TAG, "%s", "");

    if (queue == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK)
    {
        ARSAL_Mutex_Lock(&queue->lock);

        for (i=0; (i < queue->count) && (ftpMedia == NULL); i++)
        {
            if (queue->medias[i] != NULL)
            {
                ftpMedia = queue->medias[i];
                queue->medias[i] = NULL;
            }
        }

        ARSAL_Mutex_Unlock(&queue->lock);
    }

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIASQUEUE_TAG, "retrun %p, %d", ftpMedia, result);

    *error = result;
    return ftpMedia;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasQueue_GetFreeIndex(ARDATATRANSFER_MediasQueue_t *queue, int *freeIndex)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    ARDATATRANSFER_FtpMedia_t **old;
    int index = -1;
    int i;

    *freeIndex = -1;

    for (i=0; (i < queue->count) && (index == -1); i++)
    {
        if (queue->medias[i] == NULL)
        {
            index = i;
        }
    }

    if (index == -1)
    {
        old = queue->medias;
        index = queue->count;

        queue->medias = (ARDATATRANSFER_FtpMedia_t **)realloc(queue->medias, (queue->count + ARDATATRANSFER_MEDIA_QUEUE_SIZE) * sizeof(ARDATATRANSFER_FtpMedia_t *));

        if (queue->medias == NULL)
        {
            queue->medias = old;
            result = ARDATATRANSFER_ERROR_ALLOC;
        }
        else
        {
            queue->count += ARDATATRANSFER_MEDIA_QUEUE_SIZE;
        }

        if (result == ARDATATRANSFER_OK)
        {
            for (i=index; i<queue->count; i++)
            {
                queue->medias[i] = NULL;
            }
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        *freeIndex = index;
    }

    return result;
}


