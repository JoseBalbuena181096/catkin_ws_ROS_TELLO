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
 * @file ARNETWORK_Sender.c
 * @brief manage the data sending
 * @date 28/09/2012
 * @author maxime.maitre@parrot.com
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <stdlib.h>

#include <stddef.h>

#include <errno.h>

#include <unistd.h>
#include <string.h>

#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Socket.h>
#include <libARSAL/ARSAL_Endianness.h>

#include <libARNetwork/ARNETWORK_Error.h>
#include <libARNetworkAL/ARNETWORKAL_Frame.h>
#include "ARNETWORK_DataDescriptor.h"
#include <libARNetwork/ARNETWORK_Manager.h>
#include "ARNETWORK_Manager.h"
#include "ARNETWORK_IOBuffer.h"

#include "ARNETWORK_Sender.h"

/*****************************************
 *
 *             define :
 *
 *****************************************/

#define ARNETWORK_SENDER_TAG "ARNETWORK_Sender"
#define ARNETWORK_SENDER_MILLISECOND 1

/*****************************************
 *
 *             private header:
 *
 *****************************************/

/**
 * @brief add data to the sender buffer and callback with sent status
 * @param senderPtr the pointer on the Sender
 * @param inputBufferPtr Pointer on the input buffer
 * @param isRetry Don't increment sequence number for retries
 * @return error eARNETWORK_ERROR
 * @note only call by ARNETWORK_Sender_ThreadRun()
 * @see ARNETWORK_Sender_ThreadRun()
 */
eARNETWORK_ERROR ARNETWORK_Sender_AddToBuffer (ARNETWORK_Sender_t *senderPtr, ARNETWORK_IOBuffer_t *inputBufferPtr, int isRetry);

/**
 * @brief call the Callback this timeout status
 * @param senderPtr the pointer on the Sender
 * @param inputBufferPtr Pointer on the input buffer
 * @return eARNETWORK_MANAGER_CALLBACK_RETURN
 * @note only call by ARNETWORK_Sender_ThreadRun()
 * @see ARNETWORK_Sender_ThreadRun()
 */
eARNETWORK_MANAGER_CALLBACK_RETURN ARNETWORK_Sender_TimeOutCallback (ARNETWORK_Sender_t *senderPtr, const ARNETWORK_IOBuffer_t *inputBufferPtr);

/**
 * @brief manage the return of the callback
 * @param senderPtr the pointer on the Sender
 * @param[in] inputBufferPtr Pointer on the input buffer
 * @param[in] callbackReturn return of the callback
 * @note only call by ARNETWORK_Sender_ThreadRun()
 * @see ARNETWORK_Sender_ThreadRun()
 */
void ARNETWORK_Sender_ManageTimeOut (ARNETWORK_Sender_t *senderPtr, ARNETWORK_IOBuffer_t *inputBufferPtr, eARNETWORK_MANAGER_CALLBACK_RETURN callbackReturn);



void ARNETWORK_Sender_ManageIOBufferIsInRemovingStatus(ARNETWORK_IOBuffer_t *inputBufferPtr, eARNETWORK_MANAGER_CALLBACK_STATUS callbackStatus);

/*****************************************
 *
 *             implementation :
 *
 *****************************************/

ARNETWORK_Sender_t* ARNETWORK_Sender_New (ARNETWORKAL_Manager_t *networkALManager, unsigned int numberOfInputBuffer, ARNETWORK_IOBuffer_t **inputBufferPtrArr, unsigned int numberOfInternalInputBuffer, ARNETWORK_IOBuffer_t **internalInputBufferPtrArr, ARNETWORK_IOBuffer_t **inputBufferPtrMap, int pingDelayMs)
{
    /** -- Create a new sender -- */

    /** local declarations */
    ARNETWORK_Sender_t* senderPtr =  NULL;
    eARNETWORK_ERROR error = ARNETWORK_OK;

    /** Create the sender */
    senderPtr =  malloc (sizeof (ARNETWORK_Sender_t));

    if (senderPtr)
    {
        if(networkALManager != NULL)
        {
            senderPtr->networkALManager = networkALManager;
        }
        else
        {
            error = ARNETWORK_ERROR_BAD_PARAMETER;
        }

        if(error == ARNETWORK_OK)
        {
            senderPtr->isAlive = 1;
            senderPtr->numberOfInputBuff = numberOfInputBuffer;
            senderPtr->inputBufferPtrArr = inputBufferPtrArr;
            senderPtr->numberOfInternalInputBuff = numberOfInternalInputBuffer;
            senderPtr->internalInputBufferPtrArr = internalInputBufferPtrArr;
            senderPtr->inputBufferPtrMap = inputBufferPtrMap;
            senderPtr->minimumTimeBetweenSendsMs = ARNETWORK_SENDER_MILLISECOND;
            senderPtr->isPingRunning = 0;
            senderPtr->hadARNetworkALOverflowOnPreviousRun = 0;
            if (pingDelayMs == 0)
            {
                senderPtr->minTimeBetweenPings = ARNETWORK_SENDER_MINIMUM_TIME_BETWEEN_PINGS_MS;
            }
            else
            {
                senderPtr->minTimeBetweenPings = pingDelayMs;
            }
            ARSAL_Time_GetTime(&(senderPtr->pingStartTime));
        }

        /* Create the mutex/condition */
        if ( (error == ARNETWORK_OK) &&
             (ARSAL_Mutex_Init (&(senderPtr->nextSendMutex)) != 0))
        {
            error = ARNETWORK_ERROR_NEW_BUFFER;
        }

        if ( (error == ARNETWORK_OK) &&
             (ARSAL_Cond_Init (&(senderPtr->nextSendCond)) != 0))
        {
            error = ARNETWORK_ERROR_NEW_BUFFER;
        }

        if ( (error == ARNETWORK_OK) &&
             (ARSAL_Mutex_Init (&(senderPtr->pingMutex)) != 0))
        {
            error = ARNETWORK_ERROR_NEW_BUFFER;
        }

        /** delete the sender if an error occurred */
        if (error != ARNETWORK_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_SENDER_TAG, "error: %s", ARNETWORK_Error_ToString (error));
            ARNETWORK_Sender_Delete (&senderPtr);
        }
    }

    return senderPtr;
}

void ARNETWORK_Sender_Delete (ARNETWORK_Sender_t **senderPtrAddr)
{
    /** -- Delete the sender -- */

    /** local declarations */
    ARNETWORK_Sender_t *senderPtr = NULL;

    if (senderPtrAddr != NULL)
    {
        senderPtr = *senderPtrAddr;

        if (senderPtr != NULL)
        {
            ARSAL_Cond_Destroy (&(senderPtr->nextSendCond));
            ARSAL_Mutex_Destroy (&(senderPtr->nextSendMutex));
            ARSAL_Mutex_Destroy (&(senderPtr->pingMutex));

            free (senderPtr);
            senderPtr = NULL;
        }
        *senderPtrAddr = NULL;
    }
}

void* ARNETWORK_Sender_ThreadRun (void* data)
{
    /** -- Manage the sending of the data on the sender' -- */

    /** local declarations */
    ARNETWORK_Sender_t *senderPtr = data;
    int inputBufferIndex = 0;
    ARNETWORK_IOBuffer_t *inputBufferPtrTemp = NULL; /**< pointer of the input buffer in processing */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    int waitTimeMs = 0;
    struct timespec now;
    struct timespec sleepStart;
    int sleepDurationMs = 0;
    int timeDiffMs;

    while (senderPtr->isAlive)
    {
        waitTimeMs = 500;
        for (inputBufferIndex = 0; inputBufferIndex < senderPtr->numberOfInputBuff && waitTimeMs > 0; ++inputBufferIndex)
        {
            inputBufferPtrTemp = senderPtr->inputBufferPtrArr[inputBufferIndex];
            error = ARNETWORK_IOBuffer_Lock(inputBufferPtrTemp);
            switch (inputBufferPtrTemp->dataType)
            {
                // Low latency : no wait if any data available
            case ARNETWORKAL_FRAME_TYPE_DATA_LOW_LATENCY:
                if ((error == ARNETWORK_OK) &&
                    (!ARNETWORK_RingBuffer_IsEmpty (inputBufferPtrTemp->dataDescriptorRBuffer)))
                {
                    waitTimeMs = 0;
                }
                break;
                // Acknowledged buffer :
                //  - If waiting an ack, wait time = time before ack timeout
                //  - If not waiting an ack and not empty, wait time = time before next send
            case ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK:
                if (error == ARNETWORK_OK)
                {
                    if (ARNETWORK_IOBuffer_IsWaitAck(inputBufferPtrTemp))
                    {
                        if (inputBufferPtrTemp->ackWaitTimeCount < waitTimeMs)
                        {
                            waitTimeMs = inputBufferPtrTemp->ackWaitTimeCount;
                        }
                    }
                    else if (!ARNETWORK_RingBuffer_IsEmpty (inputBufferPtrTemp->dataDescriptorRBuffer))
                    {
                        if (inputBufferPtrTemp->waitTimeCount < waitTimeMs)
                        {
                            waitTimeMs = inputBufferPtrTemp->waitTimeCount;
                        }
                    }
                }
                break;
                // All non Ack buffers
                //  - 
            default:
                if ((error == ARNETWORK_OK) &&
                    (!ARNETWORK_RingBuffer_IsEmpty (inputBufferPtrTemp->dataDescriptorRBuffer)))
                {
                    if (inputBufferPtrTemp->waitTimeCount < waitTimeMs)
                    {
                        waitTimeMs = inputBufferPtrTemp->waitTimeCount;
                    }
                }
                break;
            }
            ARNETWORK_IOBuffer_Unlock(inputBufferPtrTemp);
        }
        // Force a minimum wait time after an ARNetworkAL Overflow
        if ((senderPtr->hadARNetworkALOverflowOnPreviousRun > 0) &&
            (waitTimeMs < ARNETWORK_SENDER_WAIT_TIME_ON_ARNETWORKAL_OVERFLOW_MS))
        {
            waitTimeMs = ARNETWORK_SENDER_WAIT_TIME_ON_ARNETWORKAL_OVERFLOW_MS;
        }
        senderPtr->hadARNetworkALOverflowOnPreviousRun = 0;

        ARSAL_Time_GetTime(&sleepStart);
        if (waitTimeMs > 0)
        {
            if (waitTimeMs < senderPtr->minimumTimeBetweenSendsMs)
            {
                waitTimeMs = senderPtr->minimumTimeBetweenSendsMs;
            }
            ARSAL_Mutex_Lock (&(senderPtr->nextSendMutex));
            ARSAL_Cond_Timedwait (&(senderPtr->nextSendCond), &(senderPtr->nextSendMutex), waitTimeMs);
            ARSAL_Mutex_Unlock (&(senderPtr->nextSendMutex));
        }

        /** Process internal input buffers */
        ARSAL_Time_GetTime(&now);
        sleepDurationMs = ARSAL_Time_ComputeTimespecMsTimeDiff (&sleepStart, &now);
        ARSAL_Mutex_Lock (&(senderPtr->pingMutex));
        timeDiffMs = ARSAL_Time_ComputeTimespecMsTimeDiff (&(senderPtr->pingStartTime), &now);
        /* Send only new pings if ping function is active (min time > 0) */
        if (senderPtr->minTimeBetweenPings > 0)
        {
            int maxWaitTime = senderPtr->minTimeBetweenPings;
            if (ARNETWORK_SENDER_PING_TIMEOUT_MS > maxWaitTime)
            {
                maxWaitTime = ARNETWORK_SENDER_PING_TIMEOUT_MS;
            }

            /* Send new ping if :
             *  -> DT > minTimeBetweenPings AND we're not waiting for a ping
             *  -> DT > maxWaitTime
             */
            if (((senderPtr->isPingRunning == 0) &&
                 (timeDiffMs > senderPtr->minTimeBetweenPings)) ||
                (timeDiffMs > maxWaitTime))
            {
                if (timeDiffMs > ARNETWORK_SENDER_PING_TIMEOUT_MS)
                {
                    senderPtr->lastPingValue = -1;
                }
                inputBufferPtrTemp = senderPtr->inputBufferPtrMap[ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PING];
                error = ARNETWORK_IOBuffer_Lock (inputBufferPtrTemp);
                if (error == ARNETWORK_OK) {
                    ARNETWORK_IOBuffer_AddData (inputBufferPtrTemp, (uint8_t *)&now, sizeof (now), NULL, NULL, 1);
                    ARNETWORK_IOBuffer_Unlock (inputBufferPtrTemp);
                } else {
                    ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_SENDER_TAG, "ARNETWORK_IOBuffer_Lock() failed; error: %s", ARNETWORK_Error_ToString (error));
                }
                senderPtr->pingStartTime.tv_sec = now.tv_sec;
                senderPtr->pingStartTime.tv_nsec = now.tv_nsec;
                senderPtr->isPingRunning = 1;
            }
        }

        ARSAL_Mutex_Unlock (&(senderPtr->pingMutex));

        for (inputBufferIndex = 0; inputBufferIndex < senderPtr->networkALManager->maxIds ; inputBufferIndex++)
        {
            inputBufferPtrTemp = senderPtr->inputBufferPtrMap[inputBufferIndex];
            if (inputBufferPtrTemp != NULL)
            {
                ARNETWORK_Sender_ProcessBufferToSend (senderPtr, inputBufferPtrTemp, (waitTimeMs > 0) ? sleepDurationMs : 0);
            }
        }

        senderPtr->networkALManager->send(senderPtr->networkALManager);
    }

    return NULL;
}

void ARNETWORK_Sender_ProcessBufferToSend (ARNETWORK_Sender_t *senderPtr, ARNETWORK_IOBuffer_t *buffer, int hasWaitedMs)
{
    eARNETWORK_MANAGER_CALLBACK_RETURN callbackReturn = ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT;
    eARNETWORK_ERROR error = ARNETWORK_OK;
    /** lock the IOBuffer */
    error = ARNETWORK_IOBuffer_Lock(buffer);

    if(error == ARNETWORK_OK)
    {
        /** decrement the time to wait */
        if ((buffer->waitTimeCount > 0) && (hasWaitedMs > 0))
        {
            if (hasWaitedMs > buffer->waitTimeCount)
            {
                buffer->waitTimeCount = 0;
            }
            else
            {
                buffer->waitTimeCount -= hasWaitedMs;
            }
        }

        if (ARNETWORK_IOBuffer_IsWaitAck (buffer))
        {
            /** decrement the time to wait before considering as a timeout */
            if ((buffer->ackWaitTimeCount > 0) && (hasWaitedMs > 0))
            {
                if (hasWaitedMs > buffer->ackWaitTimeCount)
                {
                    buffer->ackWaitTimeCount = 0;
                }
                else
                {
                    buffer->ackWaitTimeCount -= hasWaitedMs;
                }
            }

            if (buffer->ackWaitTimeCount == 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARNETWORK_SENDER_TAG, "[%p] Timeout waiting for ack in buffer %d", senderPtr, buffer->ID);
                if (buffer->retryCount == 0)
                {
                    /** if there are timeout and too sending retry ... */

                    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARNETWORK_SENDER_TAG, "!!! too retry !!!");

                    callbackReturn = ARNETWORK_Sender_TimeOutCallback (senderPtr, buffer);

                    ARNETWORK_Sender_ManageTimeOut (senderPtr, buffer, callbackReturn);

                }
                else
                {
                    /** if there is a timeout, retry to send the data */

                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARNETWORK_SENDER_TAG, "[%p] Will retry sending data of buffer %d", senderPtr, buffer->ID);
                    error = ARNETWORK_Sender_AddToBuffer (senderPtr, buffer, 1);
                    if (error == ARNETWORK_OK)
                    {
                        /** reset the timeout counter*/
                        buffer->ackWaitTimeCount = buffer->ackTimeoutMs;

                        /** decrement the number of retry still possible is retryCount isn't -1 */
                        if (buffer->retryCount > 0)
                        {
                            -- (buffer->retryCount);
                        }
                    }
                }
            }
        }

        else if ((!ARNETWORK_RingBuffer_IsEmpty (buffer->dataDescriptorRBuffer)) && (buffer->waitTimeCount == 0))
        {
            /** try to add the latest data of the input buffer in the sending buffer; callback with sent status */
            if (ARNETWORK_Sender_AddToBuffer (senderPtr, buffer, 0) == ARNETWORK_OK)
            {
                buffer->waitTimeCount = buffer->sendingWaitTimeMs;

                switch (buffer->dataType)
                {
                case ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK:
                    /**
                     * reinitialize the input buffer parameters,
                     * save the sequence wait for the acknowledgement,
                     * and pass on waiting acknowledgement.
                     */
                    buffer->isWaitAck = 1;
                    buffer->ackWaitTimeCount = buffer->ackTimeoutMs;
                    buffer->retryCount = buffer->numberOfRetry;
                    break;

                case ARNETWORKAL_FRAME_TYPE_DATA:
                    /** pop the data sent */
                    ARNETWORK_IOBuffer_PopData (buffer);
                    break;

                case ARNETWORKAL_FRAME_TYPE_DATA_LOW_LATENCY:
                    /** pop the data sent */
                    ARNETWORK_IOBuffer_PopData (buffer);
                    break;

                case ARNETWORKAL_FRAME_TYPE_ACK:
                    /** pop the acknowledgement sent */
                    ARNETWORK_IOBuffer_PopData (buffer);
                    break;

                default:
                    ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_SENDER_TAG, "[%p] dataType: %d unknow \n", senderPtr, buffer->dataType);
                    break;
                }
            }
        }

        /** unlock the IOBuffer */
        ARNETWORK_IOBuffer_Unlock(buffer);
    }
}

void ARNETWORK_Sender_Stop (ARNETWORK_Sender_t *senderPtr)
{
    /** -- Stop the sending -- */
    senderPtr->isAlive = 0;
}

void ARNETWORK_Sender_SignalNewData (ARNETWORK_Sender_t *senderPtr)
{
    ARSAL_Cond_Signal (&(senderPtr->nextSendCond));
}

eARNETWORK_ERROR ARNETWORK_Sender_AckReceived (ARNETWORK_Sender_t *senderPtr, int identifier, uint8_t seqNumber)
{
    /** -- Receive an acknowledgment fo a data -- */

    /** local declarations */
    ARNETWORK_IOBuffer_t *inputBufferPtr = NULL;
    eARNETWORK_ERROR error = ARNETWORK_OK;

    inputBufferPtr = senderPtr->inputBufferPtrMap[identifier];

    if (inputBufferPtr != NULL)
    {
        /** lock the IOBuffer */
        error = ARNETWORK_IOBuffer_Lock (inputBufferPtr);

        if (error == ARNETWORK_OK)
        {
            /**
             *  Transmit the acknowledgment to the input buffer.
             *     if the acknowledgment is suiarray the waiting data is popped
             */
            error = ARNETWORK_IOBuffer_AckReceived (inputBufferPtr, seqNumber);

            /** unlock the IOBuffer */
            ARNETWORK_IOBuffer_Unlock (inputBufferPtr);

            /* Wake up the send thread to update the wait time of buffers */
            ARNETWORK_Sender_SignalNewData (senderPtr);
        }
    }
    else
    {
        error = ARNETWORK_ERROR_ID_UNKNOWN;
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_Sender_Flush (ARNETWORK_Sender_t *senderPtr)
{
    /** -- Flush all IoBuffer -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    int inputIndex;
    ARNETWORK_IOBuffer_t *inputBufferTemp = NULL;

    /** for each input buffer */
    for (inputIndex = 0; inputIndex < senderPtr->numberOfInputBuff && error == ARNETWORK_OK; ++inputIndex)
    {
        inputBufferTemp = senderPtr->inputBufferPtrArr[inputIndex];
        /** lock the IOBuffer */
        error = ARNETWORK_IOBuffer_Lock (inputBufferTemp);

        if (error == ARNETWORK_OK)
        {
            /**  flush the IoBuffer */
            error = ARNETWORK_IOBuffer_Flush (inputBufferTemp);

            /** unlock the IOBuffer */
            ARNETWORK_IOBuffer_Unlock (inputBufferTemp);
        }
    }

    return error;
}

void ARNETWORK_Sender_Reset (ARNETWORK_Sender_t *senderPtr)
{
    /** -- Reset the Sender -- */

    /** local declarations */

    /** flush all IoBuffer */
    ARNETWORK_Sender_Flush (senderPtr);
}

/*****************************************
 *
 *             private implementation:
 *
 *****************************************/

eARNETWORK_ERROR ARNETWORK_Sender_AddToBuffer (ARNETWORK_Sender_t *senderPtr, ARNETWORK_IOBuffer_t *inputBufferPtr, int isRetry)
{
    /** -- add data to the sender buffer and callback with sent status -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    ARNETWORK_DataDescriptor_t dataDescriptor;

    /** pop data descriptor*/
    error = ARNETWORK_RingBuffer_Front (inputBufferPtr->dataDescriptorRBuffer, (uint8_t*) &dataDescriptor);

    if (error == ARNETWORK_OK)
    {
        ARNETWORKAL_Frame_t frame = {
            .type = 0,
            .id = 0,
            .seq = 0,
            .size = 0,
            .dataPtr = NULL,
        };
        if (isRetry == 0)
        {
            inputBufferPtr->seq++;
        }
        frame.type = inputBufferPtr->dataType;
        frame.id = inputBufferPtr->ID;
        frame.seq = inputBufferPtr->seq;
        frame.size = offsetof (ARNETWORKAL_Frame_t, dataPtr) + dataDescriptor.dataSize;
        frame.dataPtr = dataDescriptor.data;
        eARNETWORKAL_MANAGER_RETURN alStatus = senderPtr->networkALManager->pushFrame(senderPtr->networkALManager, &frame);
        switch(alStatus)
        {
        case ARNETWORKAL_MANAGER_RETURN_DEFAULT:
            /** callback with sent status */
            if (dataDescriptor.callback != NULL)
            {
                if (frame.type == ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK)
                {
                    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARNETWORK_SENDER_TAG, "[%p] Will send ack command : project = %X | class = %X | command = %X - %X", senderPtr, frame.dataPtr[0], frame.dataPtr[1], frame.dataPtr[2], frame.dataPtr[3]);
                }
                dataDescriptor.callback (inputBufferPtr->ID, dataDescriptor.data, dataDescriptor.customData, ARNETWORK_MANAGER_CALLBACK_STATUS_SENT);
            }
            break;
        case ARNETWORKAL_MANAGER_RETURN_BUFFER_FULL:
            senderPtr->hadARNetworkALOverflowOnPreviousRun = 1;
            ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_SENDER_TAG, "[%p] Not enough space to send a packet of type %d, size %d, for buffer %d", senderPtr, frame.type, frame.size, frame.id);
            switch (inputBufferPtr->dataType)
            {
            case ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK:
            case ARNETWORKAL_FRAME_TYPE_ACK:
                // Keep acks and ack data, report an error
                error = ARNETWORK_ERROR_BUFFER_SIZE;
                break;
            case ARNETWORKAL_FRAME_TYPE_DATA:
            case ARNETWORKAL_FRAME_TYPE_DATA_LOW_LATENCY:
            default:
                // Discard non ack data and low latency data, report "ok"
                error = ARNETWORK_OK;
                break;
            }
            break;
        default:
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARNETWORK_SENDER_TAG, "[%p] pushFrame returned an unexpected status : %d", senderPtr, alStatus);
                error = ARNETWORK_ERROR;
            break;
        }
    }

    return error;
}

eARNETWORK_MANAGER_CALLBACK_RETURN ARNETWORK_Sender_TimeOutCallback (ARNETWORK_Sender_t *senderPtr, const ARNETWORK_IOBuffer_t *inputBufferPtr)
{
    /** -- call the Callback this timeout status -- */

    /** local declarations */
    ARNETWORK_DataDescriptor_t dataDescriptor;
    eARNETWORK_MANAGER_CALLBACK_RETURN callbackRetrun = ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT;

    /** get dataDescriptor */
    ARNETWORK_RingBuffer_Front (inputBufferPtr->dataDescriptorRBuffer, (uint8_t*) &dataDescriptor);

    ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_SENDER_TAG, "[%p] Did timeout sending command : project = %X | class = %X | command = %X - %X", senderPtr, dataDescriptor.data[0], dataDescriptor.data[1], dataDescriptor.data[2], dataDescriptor.data[3]);
    
    /** callback with timeout status*/
    if (dataDescriptor.callback != NULL)
    {
        callbackRetrun = dataDescriptor.callback (inputBufferPtr->ID, dataDescriptor.data, dataDescriptor.customData, ARNETWORK_MANAGER_CALLBACK_STATUS_TIMEOUT);
    }

    return callbackRetrun;
}

void ARNETWORK_Sender_ManageTimeOut (ARNETWORK_Sender_t *senderPtr, ARNETWORK_IOBuffer_t *inputBufferPtr, eARNETWORK_MANAGER_CALLBACK_RETURN callbackReturn)
{
    /**  -- Manager the return of the callback -- */

    /** local declarations */

    switch (callbackReturn)
    {
    case ARNETWORK_MANAGER_CALLBACK_RETURN_RETRY :
        /** reset the retry counter */
        inputBufferPtr->retryCount = inputBufferPtr->numberOfRetry;
        break;

    case ARNETWORK_MANAGER_CALLBACK_RETURN_DATA_POP :
        /** pop the data*/
        ARNETWORK_IOBuffer_PopDataWithCallBack (inputBufferPtr, ARNETWORK_MANAGER_CALLBACK_STATUS_CANCEL);

        /** force the waiting acknowledge at 0 */
        inputBufferPtr->isWaitAck = 0;

        break;

    case ARNETWORK_MANAGER_CALLBACK_RETURN_FLUSH :
        /** fluch all IOBuffers */
        ARNETWORK_Sender_Flush (senderPtr);
        break;

    default:
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_SENDER_TAG, "[%p] Bad CallBack return :%d", senderPtr, callbackReturn);
        break;
    }
}

int ARNETWORK_Sender_GetPing (ARNETWORK_Sender_t *senderPtr)
{
    int retVal = -1;
    ARSAL_Mutex_Lock (&(senderPtr->pingMutex));
    if (senderPtr->isPingRunning == 1)
    {
        struct timespec now;
        ARSAL_Time_GetTime(&now);
        retVal = ARSAL_Time_ComputeTimespecMsTimeDiff (&(senderPtr->pingStartTime), &now);
    }
    if ((senderPtr->lastPingValue > retVal) ||
        (senderPtr->lastPingValue == -1))
    {
        retVal = senderPtr->lastPingValue;
    }
    ARSAL_Mutex_Unlock (&(senderPtr->pingMutex));
    return retVal;
}


void ARNETWORK_Sender_GotPingAck (ARNETWORK_Sender_t *senderPtr, struct timespec *startTime, struct timespec *endTime)
{
    ARSAL_Mutex_Lock (&(senderPtr->pingMutex));
    if ((senderPtr->isPingRunning == 1) &&
        (ARSAL_Time_TimespecEquals (startTime, &(senderPtr->pingStartTime))))
    {
        senderPtr->lastPingValue = ARSAL_Time_ComputeTimespecMsTimeDiff (startTime, endTime);
        senderPtr->isPingRunning = 0;
    }
    ARSAL_Mutex_Unlock (&(senderPtr->pingMutex));
}

void ARNETWORK_Sender_SendPong (ARNETWORK_Sender_t *senderPtr, uint8_t *data, int dataSize)
{
    ARNETWORK_IOBuffer_t *inputBufferPtrTemp;
    eARNETWORK_ERROR err = ARNETWORK_OK;
    inputBufferPtrTemp = senderPtr->inputBufferPtrMap[ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PONG];
    err = ARNETWORK_IOBuffer_Lock (inputBufferPtrTemp);
    if (err != ARNETWORK_OK) {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_SENDER_TAG, "ARNETWORK_IOBuffer_Lock() failed; error: %s", ARNETWORK_Error_ToString (err));
        return;
    }

    ARNETWORK_IOBuffer_AddData (inputBufferPtrTemp, data, dataSize, NULL, NULL, 1);
    ARNETWORK_IOBuffer_Unlock (inputBufferPtrTemp);
}
