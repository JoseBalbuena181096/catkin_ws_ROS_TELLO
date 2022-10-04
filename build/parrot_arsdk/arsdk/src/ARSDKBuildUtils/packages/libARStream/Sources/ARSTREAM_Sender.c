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
 * @file ARSTREAM_Sender.c
 * @brief Stream sender over network
 * @date 03/21/2013
 * @author nicolas.brulez@parrot.com
 */

#include <config.h>

/*
 * System Headers
 */

#include <stdlib.h>
#include <string.h>

#include <errno.h>

/*
 * Private Headers
 */

#include "ARSTREAM_Buffers.h"
#include "ARSTREAM_NetworkHeaders.h"

/*
 * ARSDK Headers
 */

#include <libARStream/ARSTREAM_Sender.h>
#include <libARSAL/ARSAL_Mutex.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Endianness.h>

/*
 * Macros
 */

/**
 * Tag for ARSAL_PRINT
 */
#define ARSTREAM_SENDER_TAG "ARSTREAM_Sender"

/**
 * Configuration : Enable retries (0,1)
 * 0 -> Don't retry sending a frame (count on wifi retries)
 * 1 -> Retry frame sends after some times if the acknowledge didn't come
 */
#define ENABLE_RETRIES (1)

/* Warning */
#if ENABLE_RETRIES == 0
#warning Retry is disabled in this build
#endif

/**
 * Configuration : Enable acknowledge wait
 * 0 -> Consider all frames given to network as "sent"
 * 1 -> Wait for an ARSTREAM_Reader full acknowledge of a frame before trying the next frame
 */
#define ENABLE_ACK_WAIT (0)

/* Warning */
//#if ENABLE_ACK_WAIT == 0
//#warning Ack wait is disabled in this build
//#endif

/**
 * Latency used when the network can't give us a valid value
 */
#define ARSTREAM_SENDER_DEFAULT_ESTIMATED_LATENCY_MS (100)

/**
 * Number of frames for the moving average of efficiency
 */
#define ARSTREAM_SENDER_EFFICIENCY_AVERAGE_NB_FRAMES (15)

/**
 * Number of previous frames to memorize
 */
#define ARSTREAM_SENDER_PREVIOUS_FRAME_NB_SAVE (10)

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

typedef struct {
    uint32_t frameNumber;
    uint32_t frameSize;
    uint8_t *frameBuffer;
    int isHighPriority;
} ARSTREAM_Sender_Frame_t;

struct ARSTREAM_Sender_t {
    /* Configuration on New */
    ARNETWORK_Manager_t *manager;
    int dataBufferID;
    int ackBufferID;
    ARSTREAM_Sender_FrameUpdateCallback_t callback;
    uint32_t maxNumberOfNextFrames;
    uint32_t maxFragmentSize;
    uint32_t maxNumberOfFragment;
    void *custom;

    /* Other configuration */
    int minRetryTimeMs;
    int maxRetryTimeMs;

    /* Current frame storage */
    ARSTREAM_Sender_Frame_t currentFrame;
    int currentFrameNbFragments;
    int currentFrameCbWasCalled;
    ARSAL_Mutex_t packetsToSendMutex;
    ARSTREAM_NetworkHeaders_AckPacket_t packetsToSend;

    /* Acknowledge storage */
    ARSAL_Mutex_t ackMutex;
    ARSTREAM_NetworkHeaders_AckPacket_t ackPacket;

    /* Next frame storage */
    ARSAL_Mutex_t nextFrameMutex;
    ARSAL_Cond_t  nextFrameCond;
    uint32_t nextFrameNumber;
    uint32_t indexAddNextFrame;
    uint32_t indexGetNextFrame;
    uint32_t numberOfWaitingFrames;
    ARSTREAM_Sender_Frame_t *nextFrames;

    /* Previous frame storage (for LATE_ACKs) */
    int *previousFramesStatus;
    int previousFrameIndex;

    /* Thread status */
    int threadsShouldStop;
    int dataThreadStarted;
    int ackThreadStarted;

    /* Efficiency calculations */
    int efficiency_nbFragments [ARSTREAM_SENDER_EFFICIENCY_AVERAGE_NB_FRAMES];
    int efficiency_nbSent [ARSTREAM_SENDER_EFFICIENCY_AVERAGE_NB_FRAMES];
    int efficiency_index;

    /* Filters */
    ARSTREAM_Filter_t **filters;
    int nbFilters;
};

typedef struct {
    ARSTREAM_Sender_t *sender;
    uint32_t frameNumber;
    int fragmentIndex;
} ARSTREAM_Sender_NetworkCallbackParam_t;

/*
 * Internal functions declarations
 */

/**
 * @brief Flush the new frame queue
 * @param sender The sender to flush
 * @warning Must be called within a sender->nextFrameMutex lock
 */
static void ARSTREAM_Sender_FlushQueue (ARSTREAM_Sender_t *sender);

/**
 * @brief Add a frame to the new frame queue
 * @param sender The sender which should send the frame
 * @param size The frame size, in bytes
 * @param buffer Pointer to the buffer which contains the frame
 * @param wasFlushFrame Boolean-like (0/1) flag, active if the frame is added after a flush (high priority frame)
 * @return the number of frames previously in queue (-1 if queue is full)
 */
static int ARSTREAM_Sender_AddToQueue (ARSTREAM_Sender_t *sender, uint32_t size, uint8_t *buffer, int wasFlushFrame);

/**
 * @brief Pop a frame from the new frame queue
 * @param sender The sender
 * @param newFrame Pointer in which the function will save the new frame infos
 * @return 1 if a new frame is available
 * @return 0 if no new frame should be sent (queue is empty, or filled with low-priority frame)
 */
static int ARSTREAM_Sender_PopFromQueue (ARSTREAM_Sender_t *sender, ARSTREAM_Sender_Frame_t *newFrame);

/**
 * @brief ARNETWORK_Manager_Callback_t for ARNETWORK_... calls
 * @param IoBufferId Unused as we always send on one unique buffer
 * @param dataPtr Unused as we don't need to free the memory
 * @param customData (ARSTREAM_Sender_NetworkCallbackParam_t *) Sender + fragment index
 * @param status Network information
 * @return ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT
 *
 * @warning customData is a malloc'd pointer, and must be freed within this callback, during last call
 */
eARNETWORK_MANAGER_CALLBACK_RETURN ARSTREAM_Sender_NetworkCallback (int IoBufferId, uint8_t *dataPtr, void *customData, eARNETWORK_MANAGER_CALLBACK_STATUS status);

/**
 * @brief Signals that the current frame of the sender was acknowledged
 * @param sender The sender
 */
static void ARSTREAM_Sender_FrameWasAck (ARSTREAM_Sender_t *sender);

/**
 * @brief Calls LATE_ACK callback if required
 * @param sender The sender
 * @param frameId The id of the late acknowledged frame
 * @return 1 if the function called the callback with LATE_ACK
 * @return 0 if the LATE_ACK was already sent for this frame, or if any other error occured
 */
static int ARSTREAM_Sender_SendLateAck (ARSTREAM_Sender_t *sender, uint16_t frameId);

/**
 * @brief Internal wrapper around the callback calls
 * This wrapper includes checks for framePointer value, and avoids calling
 * the actual callback on invalid frames
 * @param sernder The sender
 * @param status Why the call was made
 * @param framePointer Pointer to the frame which was sent/cancelled
 * @param frameSize Size, in bytes, of the frame
 */
static void ARSTREAM_Sender_CallCallback (ARSTREAM_Sender_t *sender, eARSTREAM_SENDER_STATUS status, uint8_t *framePointer, uint32_t frameSize, int isCurrent);

/*
 * Internal functions implementation
 */

static void ARSTREAM_Sender_FlushQueue (ARSTREAM_Sender_t *sender)
{
    while (sender->numberOfWaitingFrames > 0)
    {
        ARSTREAM_Sender_Frame_t *nextFrame = &(sender->nextFrames [sender->indexGetNextFrame]);
        ARSTREAM_Sender_CallCallback (sender, ARSTREAM_SENDER_STATUS_FRAME_CANCEL, nextFrame->frameBuffer, nextFrame->frameSize, 0);
        sender->indexGetNextFrame++;
        sender->indexGetNextFrame %= sender->maxNumberOfNextFrames;
        sender->numberOfWaitingFrames--;
    }
}

static int ARSTREAM_Sender_AddToQueue (ARSTREAM_Sender_t *sender, uint32_t size, uint8_t *buffer, int wasFlushFrame)
{
    int retVal;
    ARSAL_Mutex_Lock (&(sender->nextFrameMutex));
    retVal = sender->numberOfWaitingFrames;
    if (sender->currentFrameCbWasCalled == 0)
    {
        retVal++;
    }
    if (wasFlushFrame == 1)
    {
        ARSTREAM_Sender_FlushQueue (sender);
    }
    if (sender->numberOfWaitingFrames < sender->maxNumberOfNextFrames)
    {
        ARSTREAM_Sender_Frame_t *nextFrame = &(sender->nextFrames [sender->indexAddNextFrame]);
        sender->nextFrameNumber++;
        nextFrame->frameNumber = sender->nextFrameNumber;
        nextFrame->frameBuffer = buffer;
        nextFrame->frameSize   = size;
        nextFrame->isHighPriority = wasFlushFrame;

        sender->indexAddNextFrame++;
        sender->indexAddNextFrame %= sender->maxNumberOfNextFrames;

        sender->numberOfWaitingFrames++;

        ARSAL_Cond_Signal (&(sender->nextFrameCond));
    }
    else
    {
        retVal = -1;
    }
    ARSAL_Mutex_Unlock (&(sender->nextFrameMutex));
    return retVal;
}

static int ARSTREAM_Sender_PopFromQueue (ARSTREAM_Sender_t *sender, ARSTREAM_Sender_Frame_t *newFrame)
{
    int retVal = 0;
    int hadTimeout = 0;
    ARSAL_Mutex_Lock (&(sender->nextFrameMutex));
    // Check if a frame is ready and of good priority
    if (sender->numberOfWaitingFrames > 0)
    {
#if ENABLE_ACK_WAIT == 1
        ARSTREAM_Sender_Frame_t *frame = &(sender->nextFrames [sender->indexGetNextFrame]);
        // Give the next frame only if :
        // 1> It's an high priority frame
        // 2> The previous frame was fully acknowledged
        if ((frame->isHighPriority == 1) ||
            (sender->currentFrameCbWasCalled == 1))
#endif
        {
            retVal = 1;
            sender->numberOfWaitingFrames--;
        }
    }
    // If not, wait for a frame ready event
    if (retVal == 0)
    {
        struct timespec start, end;
        int timewaited = 0;
        int waitTime = ARNETWORK_Manager_GetEstimatedLatency (sender->manager);
        if (waitTime < 0) // Unable to get latency
        {
            waitTime = ARSTREAM_SENDER_DEFAULT_ESTIMATED_LATENCY_MS;
        }
        waitTime += 5; // Add some time to avoid optimistic waitTime, and 0ms waitTime
        if (waitTime > sender->maxRetryTimeMs)
            waitTime = sender->maxRetryTimeMs;
        if (waitTime < sender->minRetryTimeMs)
            waitTime = sender->minRetryTimeMs;
#if ENABLE_RETRIES == 0
        waitTime = 100000; // Put an extremely long wait time (100 sec) to simulate a "no retry" case
#endif

        while ((retVal == 0) &&
               (hadTimeout == 0))
        {
            ARSAL_Time_GetTime(&start);
            int err = ARSAL_Cond_Timedwait (&(sender->nextFrameCond), &(sender->nextFrameMutex), waitTime - timewaited);
            ARSAL_Time_GetTime(&end);
            timewaited += ARSAL_Time_ComputeTimespecMsTimeDiff (&start, &end);
            if (err == ETIMEDOUT)
            {
                hadTimeout = 1;
            }
            if (sender->numberOfWaitingFrames > 0)
            {
#if ENABLE_ACK_WAIT == 1
                ARSTREAM_Sender_Frame_t *frame = &(sender->nextFrames [sender->indexGetNextFrame]);
                // Give the next frame only if :
                // 1> It's an high priority frame
                // 2> The previous frame was fully acknowledged
                if ((frame->isHighPriority == 1) ||
                    (sender->currentFrameCbWasCalled == 1))
#endif
                {
                    retVal = 1;
                    sender->numberOfWaitingFrames--;
                }
            }
        }
    }
    // If we got a new frame, apply filters then copy it
    if (retVal == 1)
    {
        ARSTREAM_Sender_Frame_t *frame = &(sender->nextFrames [sender->indexGetNextFrame]);
        sender->indexGetNextFrame++;
        sender->indexGetNextFrame %= sender->maxNumberOfNextFrames;

        // Apply filters
        int inSize = frame->frameSize;
        int outSize = 0;
        int maxOutSize = 0;
        uint8_t *inBuffer = frame->frameBuffer;
        uint8_t *outBuffer = NULL;
        int i;
        ARSTREAM_Filter_t *prevFilter = NULL;
        for (i = 0; i < sender->nbFilters; i++)
        {
            ARSTREAM_Filter_t *filter = sender->filters[i];
            maxOutSize = filter->getOutputSize(filter->context,
                                               inSize);
            outBuffer = filter->getBuffer(filter->context,
                                          maxOutSize);
            outSize = filter->filterBuffer(filter->context,
                                           inBuffer, inSize,
                                           outBuffer, maxOutSize);
            if (prevFilter != NULL)
            {
                // We're in a chain, release the input buffer to the
                // previous filter
                prevFilter->releaseBuffer(prevFilter->context,
                                          inBuffer);
            }
            else
            {
                // We're the first filter, release the input buffer to the
                // application
                ARSTREAM_Sender_CallCallback (sender,
                                              ARSTREAM_SENDER_STATUS_FRAME_SENT,
                                              frame->frameBuffer,
                                              frame->frameSize,
                                              0);
            }
            inBuffer = outBuffer;
            inSize = outSize;
        }
        newFrame->frameNumber = frame->frameNumber;
        newFrame->frameBuffer = inBuffer;
        newFrame->frameSize   = inSize;
        newFrame->isHighPriority = frame->isHighPriority;
    }
    ARSAL_Mutex_Unlock (&(sender->nextFrameMutex));
    return retVal;
}

eARNETWORK_MANAGER_CALLBACK_RETURN ARSTREAM_Sender_NetworkCallback (int IoBufferId, uint8_t *dataPtr, void *customData, eARNETWORK_MANAGER_CALLBACK_STATUS status)
{
    eARNETWORK_MANAGER_CALLBACK_RETURN retVal = ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT;

    /* Get params */
    ARSTREAM_Sender_NetworkCallbackParam_t *cbParams = (ARSTREAM_Sender_NetworkCallbackParam_t *)customData;

    /* Get Sender */
    ARSTREAM_Sender_t *sender = cbParams->sender;

    /* Get packetIndex */
    int packetIndex = cbParams->fragmentIndex;

    /* Get frameNumber */
    uint32_t frameNumber = cbParams->frameNumber;

    /* Remove "unused parameter" warnings */
    (void)IoBufferId;
    (void)dataPtr;

    switch (status)
    {
    case ARNETWORK_MANAGER_CALLBACK_STATUS_SENT:
        ARSAL_Mutex_Lock (&(sender->packetsToSendMutex));
        // Modify packetsToSend only if it refers to the frame we're sending
        if (frameNumber == sender->packetsToSend.frameNumber)
        {
            ARSAL_PRINT (ARSAL_PRINT_VERBOSE, ARSTREAM_SENDER_TAG, "Sent packet %d", packetIndex);
            if (1 == ARSTREAM_NetworkHeaders_AckPacketUnsetFlag (&(sender->packetsToSend), packetIndex))
            {
                ARSAL_PRINT (ARSAL_PRINT_VERBOSE, ARSTREAM_SENDER_TAG, "All packets were sent");
            }
        }
        else
        {
            ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARSTREAM_SENDER_TAG, "Sent a packet for an old frame [packet %d, current frame %d]", frameNumber,sender->packetsToSend.frameNumber);
        }
        ARSAL_Mutex_Unlock (&(sender->packetsToSendMutex));
        /* Free cbParams */
        free (cbParams);
        break;
    case ARNETWORK_MANAGER_CALLBACK_STATUS_CANCEL:
        /* Free cbParams */
        free (cbParams);
        break;
    default:
        break;
    }
    return retVal;
}


static void ARSTREAM_Sender_FrameWasAck (ARSTREAM_Sender_t *sender)
{
    ARSTREAM_Sender_CallCallback (sender, ARSTREAM_SENDER_STATUS_FRAME_SENT, sender->currentFrame.frameBuffer, sender->currentFrame.frameSize, 1);
    sender->currentFrameCbWasCalled = 1;
    ARSAL_Mutex_Lock (&(sender->nextFrameMutex));
    ARSAL_Cond_Signal (&(sender->nextFrameCond));
    ARSAL_Mutex_Unlock (&(sender->nextFrameMutex));
}

static int ARSTREAM_Sender_SendLateAck (ARSTREAM_Sender_t *sender, uint16_t frameId)
{
    int retVal = 0;
    int deltaNum = sender->currentFrame.frameNumber - frameId;
    int index = (ARSTREAM_SENDER_PREVIOUS_FRAME_NB_SAVE + sender->previousFrameIndex - deltaNum) % ARSTREAM_SENDER_PREVIOUS_FRAME_NB_SAVE;
    if (sender->previousFramesStatus[index] == 0)
    {
        sender->previousFramesStatus[index] = 1;
        retVal = 1;
        ARSTREAM_Sender_CallCallback (sender, ARSTREAM_SENDER_STATUS_FRAME_LATE_ACK, NULL, 0, 0);
    }
    return retVal;
}

static void ARSTREAM_Sender_CallCallback (ARSTREAM_Sender_t *sender, eARSTREAM_SENDER_STATUS status, uint8_t *framePointer, uint32_t frameSize, int isCurrent)
{
    int needToCall = 1;
    // Dont call if the frame is null, except for LATE_ACKs
    if (framePointer == NULL && status != ARSTREAM_SENDER_STATUS_FRAME_LATE_ACK)
    {
        needToCall = 0;
    }

    if (needToCall == 1)
    {
        // Release to filter if
        //  - We are calling the callback on a current frame (i.e. not one in
        //    the frame queue, which is not yet filtered !)
        //  - We have at least one filter. Otherwise, the process buffer is
        //    still the same as the one given in SendNewFrame
        if (isCurrent && sender->nbFilters > 0)
        {
            ARSTREAM_Filter_t *lastFilter = sender->filters[sender->nbFilters - 1];
            lastFilter->releaseBuffer(lastFilter->context,
                                      framePointer);
        }
        else
        {
            sender->callback(status, framePointer, frameSize, sender->custom);
        }
    }
}

/*
 * Implementation
 */

void ARSTREAM_Sender_InitStreamDataBuffer (ARNETWORK_IOBufferParam_t *bufferParams, int bufferID, int maxFragmentSize, uint32_t maxFragmentPerFrame)
{
    ARSTREAM_Buffers_InitStreamDataBuffer (bufferParams, bufferID, sizeof (ARSTREAM_NetworkHeaders_DataHeader_t), maxFragmentSize, maxFragmentPerFrame);
}

void ARSTREAM_Sender_InitStreamAckBuffer (ARNETWORK_IOBufferParam_t *bufferParams, int bufferID)
{
    ARSTREAM_Buffers_InitStreamAckBuffer (bufferParams, bufferID);
}

ARSTREAM_Sender_t* ARSTREAM_Sender_New (ARNETWORK_Manager_t *manager, int dataBufferID, int ackBufferID, ARSTREAM_Sender_FrameUpdateCallback_t callback, uint32_t framesBufferSize, uint32_t maxFragmentSize, uint32_t maxNumberOfFragment,  void *custom, eARSTREAM_ERROR *error)
{
    ARSTREAM_Sender_t *retSender = NULL;
    int packetsToSendMutexWasInit = 0;
    int ackMutexWasInit = 0;
    int nextFrameMutexWasInit = 0;
    int nextFrameCondWasInit = 0;
    int nextFramesArrayWasCreated = 0;
    int previousFramesArrayWasCreated = 0;
    eARSTREAM_ERROR internalError = ARSTREAM_OK;
    /* ARGS Check */
    if ((manager == NULL) ||
        (callback == NULL) ||
        (maxFragmentSize == 0) ||
        (maxNumberOfFragment > ARSTREAM_NETWORK_HEADERS_MAX_FRAGMENTS_PER_FRAME))
    {
        SET_WITH_CHECK (error, ARSTREAM_ERROR_BAD_PARAMETERS);
        return retSender;
    }

    /* Alloc new sender */
    retSender = malloc (sizeof (ARSTREAM_Sender_t));
    if (retSender == NULL)
    {
        internalError = ARSTREAM_ERROR_ALLOC;
    }

    /* Copy parameters */
    if (internalError == ARSTREAM_OK)
    {
        retSender->manager = manager;
        retSender->dataBufferID = dataBufferID;
        retSender->ackBufferID = ackBufferID;
        retSender->callback = callback;
        retSender->custom = custom;
        retSender->maxNumberOfFragment = maxNumberOfFragment;
        retSender->maxNumberOfNextFrames = framesBufferSize;
        retSender->maxFragmentSize = maxFragmentSize;
    }

    /* Initialize internal config */
    if (internalError == ARSTREAM_OK)
    {
        retSender->minRetryTimeMs = ARSTREAM_SENDER_DEFAULT_MINIMUM_TIME_BETWEEN_RETRIES_MS;
        retSender->maxRetryTimeMs = ARSTREAM_SENDER_DEFAULT_MAXIMUM_TIME_BETWEEN_RETRIES_MS;
    }

    /* Setup internal mutexes/sems */
    if (internalError == ARSTREAM_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init (&(retSender->packetsToSendMutex));
        if (mutexInitRet != 0)
        {
            internalError = ARSTREAM_ERROR_ALLOC;
        }
        else
        {
            packetsToSendMutexWasInit = 1;
        }
    }
    if (internalError == ARSTREAM_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init (&(retSender->ackMutex));
        if (mutexInitRet != 0)
        {
            internalError = ARSTREAM_ERROR_ALLOC;
        }
        else
        {
            ackMutexWasInit = 1;
        }
    }
    if (internalError == ARSTREAM_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init (&(retSender->nextFrameMutex));
        if (mutexInitRet != 0)
        {
            internalError = ARSTREAM_ERROR_ALLOC;
        }
        else
        {
            nextFrameMutexWasInit = 1;
        }
    }
    if (internalError == ARSTREAM_OK)
    {
        int condInitRet = ARSAL_Cond_Init (&(retSender->nextFrameCond));
        if (condInitRet != 0)
        {
            internalError = ARSTREAM_ERROR_ALLOC;
        }
        else
        {
            nextFrameCondWasInit = 1;
        }
    }

    /* Allocate next frame storage */
    if (internalError == ARSTREAM_OK)
    {
        retSender->nextFrames = malloc (framesBufferSize * sizeof (ARSTREAM_Sender_Frame_t));
        if (retSender->nextFrames == NULL)
        {
            internalError = ARSTREAM_ERROR_ALLOC;
        }
        else
        {
            nextFramesArrayWasCreated = 1;
        }
    }

    /* Allocate previous frame storage */
    if (internalError == ARSTREAM_OK)
    {
        retSender->previousFramesStatus = calloc (ARSTREAM_SENDER_PREVIOUS_FRAME_NB_SAVE, sizeof (int));
        if (retSender->previousFramesStatus == NULL)
        {
            internalError = ARSTREAM_ERROR_ALLOC;
        }
        else
        {
            previousFramesArrayWasCreated = 1;
        }
    }

    /* Setup internal variables */
    if (internalError == ARSTREAM_OK)
    {
        int i;
        retSender->currentFrame.frameNumber = 0;
        retSender->currentFrame.frameBuffer = NULL;
        retSender->currentFrame.frameSize   = 0;
        retSender->currentFrame.isHighPriority = 0;
        retSender->currentFrameNbFragments = 0;
        retSender->currentFrameCbWasCalled = 0;
        retSender->nextFrameNumber = 0;
        retSender->indexAddNextFrame = 0;
        retSender->indexGetNextFrame = 0;
        retSender->numberOfWaitingFrames = 0;
        retSender->previousFrameIndex = 0;
        retSender->threadsShouldStop = 0;
        retSender->dataThreadStarted = 0;
        retSender->ackThreadStarted = 0;
        retSender->efficiency_index = 0;
        for (i = 0; i < ARSTREAM_SENDER_EFFICIENCY_AVERAGE_NB_FRAMES; i++)
        {
            retSender->efficiency_nbFragments [i] = 0;
            retSender->efficiency_nbSent [i] = 0;
        }
        retSender->filters = NULL;
        retSender->nbFilters = 0;
    }

    if ((internalError != ARSTREAM_OK) &&
        (retSender != NULL))
    {
        if (packetsToSendMutexWasInit == 1)
        {
            ARSAL_Mutex_Destroy (&(retSender->packetsToSendMutex));
        }
        if (ackMutexWasInit == 1)
        {
            ARSAL_Mutex_Destroy (&(retSender->ackMutex));
        }
        if (nextFrameMutexWasInit == 1)
        {
            ARSAL_Mutex_Destroy (&(retSender->nextFrameMutex));
        }
        if (nextFrameCondWasInit == 1)
        {
            ARSAL_Cond_Destroy (&(retSender->nextFrameCond));
        }
        if (nextFramesArrayWasCreated == 1)
        {
            free (retSender->nextFrames);
        }
        if (previousFramesArrayWasCreated == 1)
        {
            free (retSender->previousFramesStatus);
        }
        free (retSender);
        retSender = NULL;
    }

    SET_WITH_CHECK (error, internalError);
    return retSender;
}


eARSTREAM_ERROR ARSTREAM_Sender_SetTimeBetweenRetries (ARSTREAM_Sender_t *sender, int minWaitTimeMs, int maxWaitTimeMs)
{
    eARSTREAM_ERROR err = ARSTREAM_OK;
    if (sender == NULL ||
        minWaitTimeMs < 0 ||
        maxWaitTimeMs < minWaitTimeMs)
    {
        err = ARSTREAM_ERROR_BAD_PARAMETERS;
    }

    if (err == ARSTREAM_OK)
    {
        sender->minRetryTimeMs = minWaitTimeMs;
        sender->maxRetryTimeMs = maxWaitTimeMs;
    }
    return err;
}

void ARSTREAM_Sender_StopSender (ARSTREAM_Sender_t *sender)
{
    if (sender != NULL)
    {
        sender->threadsShouldStop = 1;
    }
    // When stopping the sender, add a dummy flush frame in the queue
    // in order to unlock the data thread. Without this, the thread might
    // stop after sender->maxRetryTimeMs, instead of immediately. When this
    // time is set to ARSTREAM_SENDER_INFINITE_TIME_BETWEEN_RETRIES, it means
    // That the thread will be joinable 100 seconds after this call.
    ARSTREAM_Sender_AddToQueue(sender, 0, NULL, 1);
}

eARSTREAM_ERROR ARSTREAM_Sender_Delete (ARSTREAM_Sender_t **sender)
{
    eARSTREAM_ERROR retVal = ARSTREAM_ERROR_BAD_PARAMETERS;
    if ((sender != NULL) &&
        (*sender != NULL))
    {
        int canDelete = 0;
        if (((*sender)->dataThreadStarted == 0) &&
            ((*sender)->ackThreadStarted == 0))
        {
            canDelete = 1;
        }

        if (canDelete == 1)
        {
            ARSAL_Mutex_Lock (&((*sender)->nextFrameMutex));
            ARSTREAM_Sender_FlushQueue (*sender);
            ARSAL_Mutex_Unlock (&((*sender)->nextFrameMutex));
            ARSAL_Mutex_Destroy (&((*sender)->packetsToSendMutex));
            ARSAL_Mutex_Destroy (&((*sender)->ackMutex));
            ARSAL_Mutex_Destroy (&((*sender)->nextFrameMutex));
            ARSAL_Cond_Destroy (&((*sender)->nextFrameCond));
            free ((*sender)->nextFrames);
            free ((*sender)->previousFramesStatus);
            free ((*sender)->filters);
            free (*sender);
            *sender = NULL;
            retVal = ARSTREAM_OK;
        }
        else
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_SENDER_TAG, "Call ARSTREAM_Sender_StopSender before calling this function");
            retVal = ARSTREAM_ERROR_BUSY;
        }
    }
    return retVal;
}

eARSTREAM_ERROR ARSTREAM_Sender_SendNewFrame (ARSTREAM_Sender_t *sender, uint8_t *frameBuffer, uint32_t frameSize, int flushPreviousFrames, int *nbPreviousFrames)
{
    eARSTREAM_ERROR retVal = ARSTREAM_OK;
    // Args check
    if ((sender == NULL) ||
        (frameBuffer == NULL) ||
        (frameSize == 0) ||
        ((flushPreviousFrames != 0) &&
         (flushPreviousFrames != 1)))
    {
        retVal = ARSTREAM_ERROR_BAD_PARAMETERS;
    }
    if ((retVal == ARSTREAM_OK) &&
        (frameSize > (sender->maxFragmentSize * sender->maxNumberOfFragment)))
    {
        retVal = ARSTREAM_ERROR_FRAME_TOO_LARGE;
    }

    if (retVal == ARSTREAM_OK)
    {
        int res = ARSTREAM_Sender_AddToQueue (sender, frameSize, frameBuffer, flushPreviousFrames);
        if (res < 0)
        {
            retVal = ARSTREAM_ERROR_QUEUE_FULL;
        }
        else if (nbPreviousFrames != NULL)
        {
            *nbPreviousFrames = res;
        }
        // No else : do nothing if the nbPreviousFrames pointer is not set
    }
    return retVal;
}

eARSTREAM_ERROR ARSTREAM_Sender_FlushFramesQueue (ARSTREAM_Sender_t *sender)
{
    eARSTREAM_ERROR retVal = ARSTREAM_OK;
    if (sender == NULL)
    {
        retVal = ARSTREAM_ERROR_BAD_PARAMETERS;
    }
    if (retVal == ARSTREAM_OK)
    {
        ARSAL_Mutex_Lock (&(sender->nextFrameMutex));
        ARSTREAM_Sender_FlushQueue (sender);
        ARSAL_Mutex_Unlock (&(sender->nextFrameMutex));
    }
    return retVal;
}

void* ARSTREAM_Sender_RunDataThread (void *ARSTREAM_Sender_t_Param)
{
    /* Local declarations */
    ARSTREAM_Sender_t *sender = (ARSTREAM_Sender_t *)ARSTREAM_Sender_t_Param;
    uint8_t *sendFragment = NULL;
    uint32_t sendSize = 0;
    uint16_t nbPackets = 0;
    uint16_t cnt;
    int numbersOfFragmentsSentForCurrentFrame = 0;
    uint32_t lastFragmentSize = 0;
    ARSTREAM_NetworkHeaders_DataHeader_t *header = NULL;
    ARSTREAM_Sender_Frame_t nextFrame = {
        .frameNumber = 0,
        .frameSize = 0,
        .frameBuffer = NULL,
        .isHighPriority = 0
    };
    int firstFrame = 1;

    /* Parameters check */
    if (sender == NULL)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_SENDER_TAG, "Error while starting %s, bad parameters", __FUNCTION__);
        return (void *)0;
    }

    /* Alloc and check */
    sendFragment = malloc (sender->maxFragmentSize + sizeof (ARSTREAM_NetworkHeaders_DataHeader_t));
    if (sendFragment == NULL)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_SENDER_TAG, "Error while starting %s, can not alloc memory", __FUNCTION__);
        return (void *)0;
    }
    header = (ARSTREAM_NetworkHeaders_DataHeader_t *)sendFragment;

    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARSTREAM_SENDER_TAG, "Sender thread running");
    sender->dataThreadStarted = 1;

    while (sender->threadsShouldStop == 0)
    {
        int waitRes;
        waitRes = ARSTREAM_Sender_PopFromQueue (sender, &nextFrame);
        // Check again if we should be stopping (after the wait).
        // If we're trying to send the dummy frame from ARSTREAM_Sender_StopSender
        // we need to make sure that we never dereference the pointer, as its NULL
        if (sender->threadsShouldStop != 0)
        {
            break;
        }
        ARSAL_Mutex_Lock (&(sender->ackMutex));
        if (waitRes == 1)
        {
            int previousWasAck = 1;
            ARSAL_PRINT (ARSAL_PRINT_VERBOSE, ARSTREAM_SENDER_TAG, "Previous frame was sent in %d packets. Frame size was %d packets", numbersOfFragmentsSentForCurrentFrame, nbPackets);
            sender->efficiency_nbFragments [sender->efficiency_index ] = nbPackets;
            sender->efficiency_nbSent [sender->efficiency_index] = numbersOfFragmentsSentForCurrentFrame;
            numbersOfFragmentsSentForCurrentFrame = 0;
            /* We have a new frame to send */
            sender->efficiency_index ++;
            sender->efficiency_index %= ARSTREAM_SENDER_EFFICIENCY_AVERAGE_NB_FRAMES;
            sender->efficiency_nbSent [sender->efficiency_index] = 0;
            sender->efficiency_nbFragments [sender->efficiency_index] = 0;

            /* Cancel current frame if it was not already sent */
            /* Do not do it for the first "NULL" frame that is in the
             * ARStream Sender before any call to SendNewFrame */
            if (sender->currentFrameCbWasCalled == 0 && firstFrame == 0)
            {
#ifdef DEBUG
                ARSTREAM_NetworkHeaders_AckPacketDump ("Cancel frame:", &(sender->ackPacket));
                ARSAL_PRINT (ARSAL_PRINT_VERBOSE, ARSTREAM_SENDER_TAG, "Receiver acknowledged %d of %d packets", ARSTREAM_NetworkHeaders_AckPacketCountSet (&(sender->ackPacket), nbPackets), nbPackets);
#endif

                previousWasAck = 0;
                ARNETWORK_Manager_FlushInputBuffer (sender->manager, sender->dataBufferID);

                ARSTREAM_Sender_CallCallback(sender, ARSTREAM_SENDER_STATUS_FRAME_CANCEL, sender->currentFrame.frameBuffer, sender->currentFrame.frameSize, 1);
            }
            sender->currentFrameCbWasCalled = 0; // New frame
            firstFrame = 0;

            /* Save next frame data into current frame data */
            sender->currentFrame.frameNumber = nextFrame.frameNumber;
            sender->currentFrame.frameBuffer = nextFrame.frameBuffer;
            sender->currentFrame.frameSize   = nextFrame.frameSize;
            sender->currentFrame.isHighPriority = nextFrame.isHighPriority;
            sendSize = nextFrame.frameSize;

            sender->previousFramesStatus[sender->previousFrameIndex] = previousWasAck;
            sender->previousFrameIndex = (sender->previousFrameIndex + 1) % ARSTREAM_SENDER_PREVIOUS_FRAME_NB_SAVE;


            /* Reset ack packet - No packets are ack on the new frame */
            sender->ackPacket.frameNumber = sender->currentFrame.frameNumber;
            ARSTREAM_NetworkHeaders_AckPacketReset (&(sender->ackPacket));

            /* Reset packetsToSend - update frame number */
            ARSAL_Mutex_Lock (&(sender->packetsToSendMutex));
            sender->packetsToSend.frameNumber = sender->currentFrame.frameNumber;
            ARSTREAM_NetworkHeaders_AckPacketReset (&(sender->packetsToSend));
            ARSAL_Mutex_Unlock (&(sender->packetsToSendMutex));

            /* Update stream data header with the new frame number */
            header->frameNumber = sender->currentFrame.frameNumber;
            header->frameFlags = 0;
            header->frameFlags |= (sender->currentFrame.isHighPriority != 0) ? ARSTREAM_NETWORK_HEADERS_FLAG_FLUSH_FRAME : 0;

            /* Compute number of fragments / size of the last fragment */
            if (0 < sendSize)
            {
                uint32_t maxFragSize = sender->maxFragmentSize;
                lastFragmentSize = maxFragSize;
                nbPackets = sendSize / maxFragSize;
                if (sendSize % maxFragSize)
                {
                    nbPackets++;
                    lastFragmentSize = sendSize % maxFragSize;
                }
            }
            sender->currentFrameNbFragments = nbPackets;

            ARSAL_PRINT (ARSAL_PRINT_VERBOSE, ARSTREAM_SENDER_TAG, "New frame has size %d (=%d packets)", sendSize, nbPackets);
        }
        ARSAL_Mutex_Unlock (&(sender->ackMutex));
        /* END OF NEW FRAME BLOCK */

        /* Flag all non-ack packets as "packet to send" */
        ARSAL_Mutex_Lock (&(sender->packetsToSendMutex));
        ARSAL_Mutex_Lock (&(sender->ackMutex));
        ARSTREAM_NetworkHeaders_AckPacketReset (&(sender->packetsToSend));
        for (cnt = 0; cnt < nbPackets; cnt++)
        {
            if (0 == ARSTREAM_NetworkHeaders_AckPacketFlagIsSet (&(sender->ackPacket), cnt))
            {
                ARSTREAM_NetworkHeaders_AckPacketSetFlag (&(sender->packetsToSend), cnt);
            }
        }

        /* Send all "packets to send" */
        for (cnt = 0; cnt < nbPackets; cnt++)
        {
            if (ARSTREAM_NetworkHeaders_AckPacketFlagIsSet (&(sender->packetsToSend), cnt))
            {
                eARNETWORK_ERROR netError = ARNETWORK_OK;
                uint32_t maxFragSize = sender->maxFragmentSize;
                numbersOfFragmentsSentForCurrentFrame ++;
                int currFragmentSize = (cnt == nbPackets-1) ? lastFragmentSize : maxFragSize;
                header->fragmentNumber = cnt;
                header->fragmentsPerFrame = nbPackets;
                memcpy (&sendFragment[sizeof (ARSTREAM_NetworkHeaders_DataHeader_t)], &(sender->currentFrame.frameBuffer)[maxFragSize*cnt], currFragmentSize);
                ARSTREAM_Sender_NetworkCallbackParam_t *cbParams = malloc (sizeof (ARSTREAM_Sender_NetworkCallbackParam_t));
                cbParams->sender = sender;
                cbParams->fragmentIndex = cnt;
                cbParams->frameNumber = sender->packetsToSend.frameNumber;
                ARSAL_Mutex_Unlock (&(sender->packetsToSendMutex));
                netError = ARNETWORK_Manager_SendData (sender->manager, sender->dataBufferID, sendFragment, currFragmentSize + sizeof (ARSTREAM_NetworkHeaders_DataHeader_t), (void *)cbParams, ARSTREAM_Sender_NetworkCallback, 1);
                if (netError != ARNETWORK_OK)
                {
                    ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_SENDER_TAG, "Error occurred during sending of the fragment ; error: %d : %s", netError, ARNETWORK_Error_ToString(netError));
                }

                ARSAL_Mutex_Lock (&(sender->packetsToSendMutex));
            }
        }
        ARSAL_Mutex_Unlock (&(sender->ackMutex));
        ARSAL_Mutex_Unlock (&(sender->packetsToSendMutex));
    }
    /* END OF PROCESS LOOP */

    if (sender->currentFrameCbWasCalled == 0 && firstFrame == 0)
    {
#ifdef DEBUG
        ARSTREAM_NetworkHeaders_AckPacketDump ("Cancel frame:", &(sender->ackPacket));
        ARSAL_PRINT (ARSAL_PRINT_VERBOSE, ARSTREAM_SENDER_TAG, "Receiver acknowledged %d of %d packets", ARSTREAM_NetworkHeaders_AckPacketCountSet (&(sender->ackPacket), nbPackets), nbPackets);
#endif
        ARSTREAM_Sender_CallCallback (sender, ARSTREAM_SENDER_STATUS_FRAME_CANCEL, sender->currentFrame.frameBuffer, sender->currentFrame.frameSize, 1);
    }

    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARSTREAM_SENDER_TAG, "Sender thread ended");
    sender->dataThreadStarted = 0;

    if(sendFragment)
    {
        free(sendFragment);
        sendFragment = NULL;
    }

    return (void *)0;
}


void* ARSTREAM_Sender_RunAckThread (void *ARSTREAM_Sender_t_Param)
{
    ARSTREAM_NetworkHeaders_AckPacket_t recvPacket;
    int recvSize;
    ARSTREAM_Sender_t *sender = (ARSTREAM_Sender_t *)ARSTREAM_Sender_t_Param;

    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARSTREAM_SENDER_TAG, "Ack thread running");
    sender->ackThreadStarted = 1;

    ARSTREAM_NetworkHeaders_AckPacketReset (&recvPacket);

    while (sender->threadsShouldStop == 0)
    {
        eARNETWORK_ERROR err = ARNETWORK_Manager_ReadDataWithTimeout (sender->manager, sender->ackBufferID, (uint8_t *)&recvPacket, sizeof (recvPacket), &recvSize, 1000);
        if (ARNETWORK_OK != err)
        {
            if (ARNETWORK_ERROR_BUFFER_EMPTY != err)
            {
                ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_SENDER_TAG, "Error while reading ACK data: %s", ARNETWORK_Error_ToString (err));
            }
        }
        else if (recvSize != sizeof (recvPacket))
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_SENDER_TAG, "Read %d octets, expected %zu", recvSize, sizeof (recvPacket));
        }
        else
        {
            /* Switch recvPacket endianness */
            recvPacket.frameNumber = dtohs (recvPacket.frameNumber);
            recvPacket.highPacketsAck = dtohll (recvPacket.highPacketsAck);
            recvPacket.lowPacketsAck = dtohll (recvPacket.lowPacketsAck);

            /* Apply recvPacket to sender->ackPacket if frame numbers are the same */
            ARSAL_Mutex_Lock (&(sender->ackMutex));
            if (sender->ackPacket.frameNumber == recvPacket.frameNumber)
            {
                ARSTREAM_NetworkHeaders_AckPacketSetFlags (&(sender->ackPacket), &recvPacket);
                if ((sender->currentFrameCbWasCalled == 0) &&
                    (ARSTREAM_NetworkHeaders_AckPacketAllFlagsSet (&(sender->ackPacket), sender->currentFrameNbFragments) == 1))
                {
                    ARSTREAM_Sender_FrameWasAck (sender);
                }
            }
            else if (ARSTREAM_NetworkHeaders_AckPacketAllFlagsSet (&recvPacket, sender->maxNumberOfFragment) == 1)
            {
                ARSTREAM_Sender_SendLateAck (sender, recvPacket.frameNumber);
            }
            ARSAL_Mutex_Unlock (&(sender->ackMutex));
        }
    }

    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARSTREAM_SENDER_TAG, "Ack thread ended");
    sender->ackThreadStarted = 0;
    return (void *)0;
}

float ARSTREAM_Sender_GetEstimatedEfficiency (ARSTREAM_Sender_t *sender)
{
    if (sender == NULL)
    {
        return -1.0f;
    }
    float retVal = 1.0f;
    uint32_t totalPackets = 0;
    uint32_t sentPackets = 0;
    int i;
    ARSAL_Mutex_Lock (&(sender->ackMutex));
    for (i = 0; i < ARSTREAM_SENDER_EFFICIENCY_AVERAGE_NB_FRAMES; i++)
    {
        totalPackets += sender->efficiency_nbFragments [i];
        sentPackets += sender->efficiency_nbSent [i];
    }
    ARSAL_Mutex_Unlock (&(sender->ackMutex));
    if (sentPackets == 0)
    {
        retVal = 1.0f; // We didn't send any packet yet, so we have a 100% success !
    }
    else if (totalPackets > sentPackets)
    {
        retVal = 1.0f; // If this happens, it means that we have a big problem
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSTREAM_SENDER_TAG, "Computed efficiency is greater that 1.0 ...");
    }
    else
    {
        retVal = (1.f * totalPackets) / (1.f * sentPackets);
    }
    return retVal;
}

void* ARSTREAM_Sender_GetCustom (ARSTREAM_Sender_t *sender)
{
    void *ret = NULL;
    if (sender != NULL)
    {
        ret = sender->custom;
    }
    return ret;
}

eARSTREAM_ERROR ARSTREAM_Sender_AddFilter (ARSTREAM_Sender_t *sender, ARSTREAM_Filter_t *filter)
{
    if (sender == NULL || filter == NULL)
    {
        return ARSTREAM_ERROR_BAD_PARAMETERS;
    }

    if (sender->dataThreadStarted != 0 ||
        sender->ackThreadStarted != 0)
    {
        return ARSTREAM_ERROR_BUSY;
    }

    eARSTREAM_ERROR err = ARSTREAM_OK;
    ARSTREAM_Filter_t **newFilters =
        realloc(sender->filters,
                (sender->nbFilters + 1) * sizeof (ARSTREAM_Filter_t *));
    if (newFilters != NULL)
    {
        sender->filters = newFilters;
        newFilters[sender->nbFilters] = filter;
        sender->nbFilters++;
    }
    else
    {
        err = ARSTREAM_ERROR_ALLOC;
    }
    return err;
}
