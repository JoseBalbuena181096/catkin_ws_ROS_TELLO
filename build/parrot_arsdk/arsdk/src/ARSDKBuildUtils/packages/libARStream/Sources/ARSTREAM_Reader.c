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
 * @file ARSTREAM_Reader.c
 * @brief Stream reader on network
 * @date 03/22/2013
 * @author nicolas.brulez@parrot.com
 */

#include <config.h>

/*
 * System Headers
 */

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

/*
 * Private Headers
 */

#include "ARSTREAM_Buffers.h"
#include "ARSTREAM_NetworkHeaders.h"

/*
 * ARSDK Headers
 */

#include <libARStream/ARSTREAM_Reader.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Mutex.h>
#include <libARSAL/ARSAL_Endianness.h>

/*
 * Macros
 */

#define ARSTREAM_READER_TAG "ARSTREAM_Reader"
#define ARSTREAM_READER_DATAREAD_TIMEOUT_MS (500)

#define ARSTREAM_READER_EFFICIENCY_AVERAGE_NB_FRAMES (15)

/**
 * Sets *PTR to VAL if PTR is not null
 */
#define SET_WITH_CHECK(PTR,VAL)                 \
    do                                          \
    {                                           \
        if (PTR != NULL)                        \
        {                                       \
            *PTR = VAL;                         \
        }                                       \
    } while (0)

/*
 * Types
 */

struct ARSTREAM_Reader_t {
    /* Configuration on New */
    ARNETWORK_Manager_t *manager;
    int dataBufferID;
    int ackBufferID;
    uint32_t maxFragmentSize;
    int32_t maxAckInterval;
    ARSTREAM_Reader_FrameCompleteCallback_t callback;
    void *custom;

    /* Current frame storage */
    uint32_t currentFrameBufferSize; // Usable length of the buffer
    uint32_t currentFrameSize;       // Actual data length
    uint8_t *currentFrameBuffer;

    /* Output frame storage */
    uint32_t outputFrameBufferSize; // Usable length of the buffer
    uint8_t *outputFrameBuffer;

    /* Acknowledge storage */
    ARSAL_Mutex_t ackPacketMutex;
    ARSTREAM_NetworkHeaders_AckPacket_t ackPacket;
    ARSAL_Mutex_t ackSendMutex;
    ARSAL_Cond_t ackSendCond;

    /* Thread status */
    int threadsShouldStop;
    int dataThreadStarted;
    int ackThreadStarted;

    /* Efficiency calculations */
    int efficiency_nbUseful [ARSTREAM_READER_EFFICIENCY_AVERAGE_NB_FRAMES];
    int efficiency_nbTotal  [ARSTREAM_READER_EFFICIENCY_AVERAGE_NB_FRAMES];
    int efficiency_index;

    /* Filters */
    ARSTREAM_Filter_t **filters;
    int nbFilters;
};

/*
 * Internal functions declarations
 */

/**
 * @brief ARNETWORK_Manager_Callback_t for ARNETWORK_... calls
 * @param IoBufferId Unused as we always send on one unique buffer
 * @param dataPtr Unused as we don't need to free the memory
 * @param customData Unused
 * @param status Unused
 * @return ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT
 */
eARNETWORK_MANAGER_CALLBACK_RETURN ARSTREAM_Reader_NetworkCallback (int IoBufferId, uint8_t *dataPtr, void *customData, eARNETWORK_MANAGER_CALLBACK_STATUS status);

/*
 * Internal functions implementation
 */

//TODO: Network, NULL callback should be ok ?
eARNETWORK_MANAGER_CALLBACK_RETURN ARSTREAM_Reader_NetworkCallback (int IoBufferId, uint8_t *dataPtr, void *customData, eARNETWORK_MANAGER_CALLBACK_STATUS status)
{
    /* Avoid "unused parameters" warnings */
    (void)IoBufferId;
    (void)dataPtr;
    (void)customData;
    (void)status;

    /* Dummy return value */
    return ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT;
}

/*
 * Implementation
 */

void ARSTREAM_Reader_InitStreamDataBuffer (ARNETWORK_IOBufferParam_t *bufferParams, int bufferID, int maxFragmentSize, uint32_t maxNumberOfFragment)
{
    ARSTREAM_Buffers_InitStreamDataBuffer (bufferParams, bufferID, sizeof (ARSTREAM_NetworkHeaders_DataHeader_t), maxFragmentSize, maxNumberOfFragment);
}

void ARSTREAM_Reader_InitStreamAckBuffer (ARNETWORK_IOBufferParam_t *bufferParams, int bufferID)
{
    ARSTREAM_Buffers_InitStreamAckBuffer (bufferParams, bufferID);
}

ARSTREAM_Reader_t* ARSTREAM_Reader_New (ARNETWORK_Manager_t *manager, int dataBufferID, int ackBufferID, ARSTREAM_Reader_FrameCompleteCallback_t callback, uint8_t *frameBuffer, uint32_t frameBufferSize, uint32_t maxFragmentSize, int32_t maxAckInterval, void *custom, eARSTREAM_ERROR *error)
{
    ARSTREAM_Reader_t *retReader = NULL;
    int ackPacketMutexWasInit = 0;
    int ackSendMutexWasInit = 0;
    int ackSendCondWasInit = 0;
    eARSTREAM_ERROR internalError = ARSTREAM_OK;
    /* ARGS Check */
    if ((manager == NULL) ||
        (callback == NULL) ||
        (frameBuffer == NULL) ||
        (frameBufferSize == 0) ||
        (maxFragmentSize == 0) ||
        (maxAckInterval < -1))
    {
        SET_WITH_CHECK (error, ARSTREAM_ERROR_BAD_PARAMETERS);
        return retReader;
    }

    /* Alloc new reader */
    retReader = malloc (sizeof (ARSTREAM_Reader_t));
    if (retReader == NULL)
    {
        internalError = ARSTREAM_ERROR_ALLOC;
    }

    /* Copy parameters */
    if (internalError == ARSTREAM_OK)
    {
        retReader->manager = manager;
        retReader->dataBufferID = dataBufferID;
        retReader->ackBufferID = ackBufferID;
        retReader->maxFragmentSize = maxFragmentSize;
        retReader->maxAckInterval = maxAckInterval;
        retReader->callback = callback;
        retReader->custom = custom;
        retReader->outputFrameBufferSize = frameBufferSize;
        retReader->outputFrameBuffer = frameBuffer;
    }

    /* Setup internal mutexes/conditions */
    if (internalError == ARSTREAM_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init (&(retReader->ackPacketMutex));
        if (mutexInitRet != 0)
        {
            internalError = ARSTREAM_ERROR_ALLOC;
        }
        else
        {
            ackPacketMutexWasInit = 1;
        }
    }
    if (internalError == ARSTREAM_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init (&(retReader->ackSendMutex));
        if (mutexInitRet != 0)
        {
            internalError = ARSTREAM_ERROR_ALLOC;
        }
        else
        {
            ackSendMutexWasInit = 1;
        }
    }
    if (internalError == ARSTREAM_OK)
    {
        int condInitRet = ARSAL_Cond_Init (&(retReader->ackSendCond));
        if (condInitRet != 0)
        {
            internalError = ARSTREAM_ERROR_ALLOC;
        }
        else
        {
            ackSendCondWasInit = 1;
        }
    }

    /* Setup internal variables */
    if (internalError == ARSTREAM_OK)
    {
        int i;
        retReader->currentFrameBufferSize = 0;
        retReader->currentFrameBuffer = NULL;
        retReader->currentFrameSize = 0;
        retReader->threadsShouldStop = 0;
        retReader->dataThreadStarted = 0;
        retReader->ackThreadStarted = 0;
        retReader->efficiency_index = 0;
        for (i = 0; i < ARSTREAM_READER_EFFICIENCY_AVERAGE_NB_FRAMES; i++)
        {
            retReader->efficiency_nbTotal [i] = 0;
            retReader->efficiency_nbUseful [i] = 0;
        }
        retReader->filters = NULL;
        retReader->nbFilters = 0;
    }

    if ((internalError != ARSTREAM_OK) &&
        (retReader != NULL))
    {
        if (ackPacketMutexWasInit == 1)
        {
            ARSAL_Mutex_Destroy (&(retReader->ackPacketMutex));
        }
        if (ackSendMutexWasInit == 1)
        {
            ARSAL_Mutex_Destroy (&(retReader->ackSendMutex));
        }
        if (ackSendCondWasInit == 1)
        {
            ARSAL_Cond_Destroy (&(retReader->ackSendCond));
        }
        free (retReader);
        retReader = NULL;
    }

    SET_WITH_CHECK (error, internalError);
    return retReader;
}

void ARSTREAM_Reader_StopReader (ARSTREAM_Reader_t *reader)
{
    if (reader != NULL)
    {
        reader->threadsShouldStop = 1;
        /* Force unblock the ACK thread to allow it to shutdown quickly.
         * This is necessary if maxAckInterval is set to -1, 0 or a large value.
         * If maxAckInterval >= 0, an ACK packet will be sent as a side-effect. */
        if (reader->ackThreadStarted == 1)
        {
            ARSAL_Mutex_Lock (&(reader->ackSendMutex));
            ARSAL_Cond_Signal (&(reader->ackSendCond));
            ARSAL_Mutex_Unlock (&(reader->ackSendMutex));
        }
    }
}

eARSTREAM_ERROR ARSTREAM_Reader_Delete (ARSTREAM_Reader_t **reader)
{
    eARSTREAM_ERROR retVal = ARSTREAM_ERROR_BAD_PARAMETERS;
    if ((reader != NULL) &&
        (*reader != NULL))
    {
        int canDelete = 0;
        if (((*reader)->dataThreadStarted == 0) &&
            ((*reader)->ackThreadStarted == 0))
        {
            canDelete = 1;
        }

        if (canDelete == 1)
        {
            ARSAL_Mutex_Destroy (&((*reader)->ackPacketMutex));
            ARSAL_Mutex_Destroy (&((*reader)->ackSendMutex));
            ARSAL_Cond_Destroy (&((*reader)->ackSendCond));
            free ((*reader)->filters);
            free (*reader);
            *reader = NULL;
            retVal = ARSTREAM_OK;
        }
        else
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_READER_TAG, "Call ARSTREAM_Reader_StopReader before calling this function");
            retVal = ARSTREAM_ERROR_BUSY;
        }
    }
    return retVal;
}

void* ARSTREAM_Reader_RunDataThread (void *ARSTREAM_Reader_t_Param)
{
    uint8_t *recvData = NULL;
    int recvSize;
    uint16_t previousFNum = UINT16_MAX;
    int skipCurrentFrame = 0;
    int packetWasAlreadyAck = 0;
    ARSTREAM_Reader_t *reader = (ARSTREAM_Reader_t *)ARSTREAM_Reader_t_Param;
    ARSTREAM_NetworkHeaders_DataHeader_t *header = NULL;
    int recvDataLen = reader->maxFragmentSize + sizeof (ARSTREAM_NetworkHeaders_DataHeader_t);

    /* Parameters check */
    if (reader == NULL)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_READER_TAG, "Error while starting %s, bad parameters", __FUNCTION__);
        return (void *)0;
    }

    /* Alloc and check */
    recvData = malloc (recvDataLen);
    if (recvData == NULL)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_READER_TAG, "Error while starting %s, can not alloc memory", __FUNCTION__);
        return (void *)0;
    }
    header = (ARSTREAM_NetworkHeaders_DataHeader_t *)recvData;

    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARSTREAM_READER_TAG, "Stream reader thread running");
    reader->dataThreadStarted = 1;

    // If we don't have filters, use the output buffer as the current one
    if (reader->nbFilters == 0)
    {
        reader->currentFrameBuffer = reader->outputFrameBuffer;
        reader->currentFrameBufferSize = reader->outputFrameBufferSize;
    }

    while (reader->threadsShouldStop == 0)
    {
        eARNETWORK_ERROR err = ARNETWORK_Manager_ReadDataWithTimeout (reader->manager, reader->dataBufferID, recvData, recvDataLen, &recvSize, ARSTREAM_READER_DATAREAD_TIMEOUT_MS);
        if (ARNETWORK_OK != err)
        {
            if (ARNETWORK_ERROR_BUFFER_EMPTY != err)
            {
                ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_READER_TAG, "Error while reading stream data: %s", ARNETWORK_Error_ToString (err));
            }
        }
        else
        {
            int cpIndex, cpSize, endIndex, filterEndIndex;
            ARSAL_Mutex_Lock (&(reader->ackPacketMutex));
            if (header->frameNumber != reader->ackPacket.frameNumber)
            {
                reader->efficiency_index ++;
                reader->efficiency_index %= ARSTREAM_READER_EFFICIENCY_AVERAGE_NB_FRAMES;
                reader->efficiency_nbTotal [reader->efficiency_index] = 0;
                reader->efficiency_nbUseful [reader->efficiency_index] = 0;
                skipCurrentFrame = 0;
                reader->currentFrameSize = 0;
                reader->ackPacket.frameNumber = header->frameNumber;
                uint32_t nackPackets = ARSTREAM_NetworkHeaders_AckPacketCountNotSet (&(reader->ackPacket), header->fragmentsPerFrame);
                if (nackPackets != 0)
                {
                    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARSTREAM_READER_TAG, "Dropping a frame (missing %d fragments)", nackPackets);
                }
                ARSTREAM_NetworkHeaders_AckPacketResetUpTo (&(reader->ackPacket), header->fragmentsPerFrame);
            }
            packetWasAlreadyAck = ARSTREAM_NetworkHeaders_AckPacketFlagIsSet (&(reader->ackPacket), header->fragmentNumber);
            ARSTREAM_NetworkHeaders_AckPacketSetFlag (&(reader->ackPacket), header->fragmentNumber);

            reader->efficiency_nbTotal [reader->efficiency_index] ++;
            if (packetWasAlreadyAck == 0)
            {
                reader->efficiency_nbUseful [reader->efficiency_index] ++;
            }

            ARSAL_Mutex_Unlock (&(reader->ackPacketMutex));

            ARSAL_Mutex_Lock (&(reader->ackSendMutex));
            ARSAL_Cond_Signal (&(reader->ackSendCond));
            ARSAL_Mutex_Unlock (&(reader->ackSendMutex));


            cpIndex = reader->maxFragmentSize * header->fragmentNumber;
            cpSize = recvSize - sizeof (ARSTREAM_NetworkHeaders_DataHeader_t);
            endIndex = cpIndex + cpSize;
            filterEndIndex = endIndex;
            if (reader->nbFilters > 0)
            {
                int i;
                for (i = 0; i < reader->nbFilters; i++)
                {
                    ARSTREAM_Filter_t *filter = reader->filters[i];
                    filterEndIndex = filter->getOutputSize(filter->context,
                                                           filterEndIndex);
                }
            }

            while ((((uint32_t)endIndex > reader->currentFrameBufferSize) ||
                    ((uint32_t)filterEndIndex > reader->outputFrameBufferSize)) &&
                   (skipCurrentFrame == 0) &&
                   (packetWasAlreadyAck == 0))
            {
                uint32_t nextFrameBufferSize = endIndex;
                uint32_t dummy;
                uint8_t *nextFrameBuffer;
                // If we have at least a filter, chain resize the buffers
                if (reader->nbFilters > 0)
                {
                    ARSTREAM_Filter_t *firstFilter = reader->filters[0];
                    nextFrameBuffer = firstFilter->getBuffer(firstFilter->context,
                                                             nextFrameBufferSize);
                    int i;
                    int finalOutputSize = firstFilter->getOutputSize(firstFilter->context,
                                                                     nextFrameBufferSize);
                    // Update final output size by requesting it from each filter
                    for (i = 1; i < reader->nbFilters; i++)
                    {
                        ARSTREAM_Filter_t *filter = reader->filters[i];
                        finalOutputSize = filter->getOutputSize(filter->context,
                                                                finalOutputSize);
                    }

                    // Resize actual output buffer if needed
                    if ((uint32_t)finalOutputSize > reader->outputFrameBufferSize)
                    {
                        uint32_t newOutputSize = finalOutputSize;
                        uint8_t *tmpFrame = reader->callback (ARSTREAM_READER_CAUSE_FRAME_TOO_SMALL, reader->outputFrameBuffer, reader->currentFrameSize, 0, 0, &newOutputSize, reader->custom);
                        if (newOutputSize < (uint32_t)finalOutputSize)
                        {
                            skipCurrentFrame = 1;
                        }
                        reader->callback (ARSTREAM_READER_CAUSE_COPY_COMPLETE, reader->outputFrameBuffer, reader->currentFrameSize, 0, skipCurrentFrame, &dummy, reader->custom);
                        reader->outputFrameBuffer = tmpFrame;
                        reader->outputFrameBufferSize = finalOutputSize;
                    }

                    // Copy into new buffer
                    if (nextFrameBuffer != NULL)
                    {
                        memcpy(nextFrameBuffer, reader->currentFrameBuffer, reader->currentFrameSize);
                    }
                    else
                    {
                        skipCurrentFrame = 1;
                    }
                    firstFilter->releaseBuffer(firstFilter->context,
                                               reader->currentFrameBuffer);
                }
                // Else, direclty resize the output buffer (and copy)
                else
                {
                    nextFrameBuffer = reader->callback (ARSTREAM_READER_CAUSE_FRAME_TOO_SMALL, reader->outputFrameBuffer, reader->currentFrameSize, 0, 0, &nextFrameBufferSize, reader->custom);
                    if (nextFrameBufferSize >= reader->currentFrameSize && nextFrameBufferSize > 0)
                    {
                        memcpy (nextFrameBuffer, reader->currentFrameBuffer, reader->currentFrameSize);
                    }
                    else
                    {
                        skipCurrentFrame = 1;
                    }
                    //TODO: Add "SKIP_FRAME"
                    reader->callback (ARSTREAM_READER_CAUSE_COPY_COMPLETE, reader->outputFrameBuffer, reader->currentFrameSize, 0, skipCurrentFrame, &dummy, reader->custom);
                    reader->outputFrameBuffer = nextFrameBuffer;
                    reader->outputFrameBufferSize = nextFrameBufferSize;
                }
                reader->currentFrameBuffer = nextFrameBuffer;
                reader->currentFrameBufferSize = nextFrameBufferSize;
            }

            if (skipCurrentFrame == 0)
            {
                if (packetWasAlreadyAck == 0)
                {
                    memcpy (&(reader->currentFrameBuffer)[cpIndex], &recvData[sizeof (ARSTREAM_NetworkHeaders_DataHeader_t)], recvSize - sizeof (ARSTREAM_NetworkHeaders_DataHeader_t));
                }

                if ((uint32_t)endIndex > reader->currentFrameSize)
                {
                    reader->currentFrameSize = endIndex;
                }

                ARSAL_Mutex_Lock (&(reader->ackPacketMutex));
                if (ARSTREAM_NetworkHeaders_AckPacketAllFlagsSet (&(reader->ackPacket), header->fragmentsPerFrame))
                {
                    if (header->frameNumber != previousFNum)
                    {
                        int nbMissedFrame = 0;
                        int isFlushFrame = ((header->frameFlags & ARSTREAM_NETWORK_HEADERS_FLAG_FLUSH_FRAME) != 0) ? 1 : 0;
                        ARSAL_PRINT (ARSAL_PRINT_VERBOSE, ARSTREAM_READER_TAG, "Ack all in frame %d (isFlush : %d)", header->frameNumber, isFlushFrame);
                        if (header->frameNumber != previousFNum + 1)
                        {
                            nbMissedFrame = header->frameNumber - previousFNum - 1;
                            ARSAL_PRINT (ARSAL_PRINT_INFO, ARSTREAM_READER_TAG, "Missed %d frames !", nbMissedFrame);
                        }
                        previousFNum = header->frameNumber;
                        skipCurrentFrame = 1;
                        // If we have filters, apply them !
                        if (reader->nbFilters > 0)
                        {
                            int i;
                            ARSTREAM_Filter_t *filter;
                            ARSTREAM_Filter_t *nextFilter;
                            uint8_t *inBuffer = reader->currentFrameBuffer;
                            int inSize = reader->currentFrameSize;
                            uint8_t *outBuffer;
                            int outSize;
                            int maxOutSize;
                            // Chain filters
                            for (i = 0; i < (reader->nbFilters - 1); i++)
                            {
                                filter = reader->filters[i];
                                nextFilter = reader->filters[i+1];
                                maxOutSize = filter->getOutputSize(filter->context,
                                                                   inSize);
                                outBuffer = nextFilter->getBuffer(nextFilter->context,
                                                                  maxOutSize);
                                outSize = filter->filterBuffer(filter->context,
                                                               inBuffer, inSize,
                                                               outBuffer, maxOutSize);
                                filter->releaseBuffer(filter->context,
                                                      inBuffer);
                                inBuffer = outBuffer;
                                inSize = outSize;
                            }
                            // Apply last filter
                            filter = reader->filters[reader->nbFilters-1];
                            outSize = filter->filterBuffer(filter->context,
                                                           inBuffer, inSize,
                                                           reader->outputFrameBuffer,
                                                           reader->outputFrameBufferSize);
                            filter->releaseBuffer(filter->context,
                                                  inBuffer);
                            reader->outputFrameBuffer = reader->callback (ARSTREAM_READER_CAUSE_FRAME_COMPLETE, reader->outputFrameBuffer, outSize, nbMissedFrame, isFlushFrame, &(reader->outputFrameBufferSize), reader->custom);
                            // Get a new buffer from first filter
                            filter = reader->filters[0];
                            reader->currentFrameBuffer = filter->getBuffer(filter->context,
                                                                           reader->currentFrameBufferSize);
                        }
                        // No filters, directly talk to the callback
                        else
                        {
                            reader->outputFrameBuffer = reader->callback (ARSTREAM_READER_CAUSE_FRAME_COMPLETE, reader->currentFrameBuffer, reader->currentFrameSize, nbMissedFrame, isFlushFrame, &(reader->outputFrameBufferSize), reader->custom);
                            reader->currentFrameBuffer = reader->outputFrameBuffer;
                            reader->currentFrameBufferSize = reader->outputFrameBufferSize;
                        }
                    }
                }
                ARSAL_Mutex_Unlock (&(reader->ackPacketMutex));
            }
        }
    }

    free (recvData);

    reader->callback (ARSTREAM_READER_CAUSE_CANCEL, reader->outputFrameBuffer, reader->currentFrameSize, 0, 0, &(reader->outputFrameBufferSize), reader->custom);
    if (reader->nbFilters > 0)
    {
        ARSTREAM_Filter_t *filter = reader->filters[0];
        filter->releaseBuffer(filter->context,
                              reader->currentFrameBuffer);
    }

    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARSTREAM_READER_TAG, "Stream reader thread ended");
    reader->dataThreadStarted = 0;
    return (void *)0;
}

void* ARSTREAM_Reader_RunAckThread (void *ARSTREAM_Reader_t_Param)
{
    ARSTREAM_NetworkHeaders_AckPacket_t sendPacket = {
        .frameNumber = 0,
        .highPacketsAck = 0,
        .lowPacketsAck = 0
    };
    ARSTREAM_Reader_t *reader = (ARSTREAM_Reader_t *)ARSTREAM_Reader_t_Param;
    memset(&sendPacket, 0, sizeof(sendPacket));

    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARSTREAM_READER_TAG, "Ack sender thread running");
    reader->ackThreadStarted = 1;

    while (reader->threadsShouldStop == 0)
    {
        int isPeriodicAck = 0;
        ARSAL_Mutex_Lock (&(reader->ackSendMutex));
        if (reader->maxAckInterval <= 0)
        {
            ARSAL_Cond_Wait (&(reader->ackSendCond), &(reader->ackSendMutex));
        }
        else
        {
            int retval = ARSAL_Cond_Timedwait (&(reader->ackSendCond), &(reader->ackSendMutex), reader->maxAckInterval);
            if (retval == -1 && errno == ETIMEDOUT)
            {
                isPeriodicAck = 1;
            }
        }
        ARSAL_Mutex_Unlock (&(reader->ackSendMutex));

        /* Only send an ACK if the maxAckInterval value allows it. */
        if ((reader->maxAckInterval > 0) ||
            ((reader->maxAckInterval == 0) && (isPeriodicAck == 0)))
        {
            ARSAL_Mutex_Lock (&(reader->ackPacketMutex));
            sendPacket.frameNumber = htods  (reader->ackPacket.frameNumber);
            sendPacket.highPacketsAck = htodll (reader->ackPacket.highPacketsAck);
            sendPacket.lowPacketsAck  = htodll (reader->ackPacket.lowPacketsAck);
            ARSAL_Mutex_Unlock (&(reader->ackPacketMutex));
            ARNETWORK_Manager_SendData (reader->manager, reader->ackBufferID, (uint8_t *)&sendPacket, sizeof (sendPacket), NULL, ARSTREAM_Reader_NetworkCallback, 1);
        }
    }

    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARSTREAM_READER_TAG, "Ack sender thread ended");
    reader->ackThreadStarted = 0;
    return (void *)0;
}

float ARSTREAM_Reader_GetEstimatedEfficiency (ARSTREAM_Reader_t *reader)
{
    if (reader == NULL)
    {
        return -1.0f;
    }
    float retVal = 1.0f;
    uint32_t totalPackets = 0;
    uint32_t usefulPackets = 0;
    int i;
    ARSAL_Mutex_Lock (&(reader->ackPacketMutex));
    for (i = 0; i < ARSTREAM_READER_EFFICIENCY_AVERAGE_NB_FRAMES; i++)
    {
        totalPackets += reader->efficiency_nbTotal [i];
        usefulPackets += reader->efficiency_nbUseful [i];
    }
    ARSAL_Mutex_Unlock (&(reader->ackPacketMutex));
    if (totalPackets == 0)
    {
        retVal = 0.0f; // We didn't receive anything yet ... not really efficient
    }
    else if (usefulPackets > totalPackets)
    {
        retVal = 1.0f; // If this happens, it means that we have a big problem
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_READER_TAG, "Computed efficiency is greater that 1.0 ...");
    }
    else
    {
        retVal = (1.f * usefulPackets) / (1.f * totalPackets);
    }
    return retVal;
}

void* ARSTREAM_Reader_GetCustom (ARSTREAM_Reader_t *reader)
{
    void *ret = NULL;
    if (reader != NULL)
    {
        ret = reader->custom;
    }
    return ret;
}

eARSTREAM_ERROR ARSTREAM_Reader_AddFilter (ARSTREAM_Reader_t *reader, ARSTREAM_Filter_t *filter)
{
    if (reader == NULL || filter == NULL)
    {
        return ARSTREAM_ERROR_BAD_PARAMETERS;
    }

    if (reader->dataThreadStarted != 0 ||
        reader->ackThreadStarted != 0)
    {
        return ARSTREAM_ERROR_BUSY;
    }

    eARSTREAM_ERROR err = ARSTREAM_OK;
    ARSTREAM_Filter_t **newFilters =
        realloc(reader->filters,
                (reader->nbFilters + 1) * sizeof (ARSTREAM_Filter_t *));
    if (newFilters != NULL)
    {
        reader->filters = newFilters;
        newFilters[reader->nbFilters] = filter;
        reader->nbFilters++;
    }
    else
    {
        err = ARSTREAM_ERROR_ALLOC;
    }
    return err;
}
