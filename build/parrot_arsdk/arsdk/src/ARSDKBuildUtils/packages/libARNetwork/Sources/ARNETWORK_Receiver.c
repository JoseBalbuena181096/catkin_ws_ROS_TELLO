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
 * @file ARNETWORK_Receiver.c
 * @brief manage the data received
 * @date 28/09/2012
 * @author maxime.maitre@parrot.com
 **/

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <stdlib.h>

#include <stddef.h>

#include <errno.h>

#include <string.h>

#ifdef ENABLE_MONITOR_INCOMING_DATA
#include <sys/eventfd.h>
#endif

#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Mutex.h>
#include <libARSAL/ARSAL_Sem.h>
#include <libARSAL/ARSAL_Socket.h>
#include <libARSAL/ARSAL_Endianness.h>

#include <libARNetwork/ARNETWORK_Error.h>
#include <libARNetworkAL/ARNETWORKAL_Frame.h>
#include "ARNETWORK_DataDescriptor.h"
#include "ARNETWORK_Manager.h"
#include <libARNetwork/ARNETWORK_Manager.h>
#include "ARNETWORK_IOBuffer.h"
#include "ARNETWORK_Sender.h"

#include "ARNETWORK_Receiver.h"

/*****************************************
 *
 *             define :
 *
 *****************************************/

#define ARNETWORK_RECEIVER_TAG "ARNETWORK_Receiver"

/*****************************************
 *
 *             private header:
 *
 *****************************************/

/**
 * @brief copy the data received to the output buffer
 * @param receiverPtr the pointer on the receiver
 * @param outputBufferPtr[in] pointer on the output buffer
 * @param framePtr[in] pointer on the frame received
 * @return eARNETWORK_ERROR.
 * @pre only call by ARNETWORK_Sender_ThreadRun()
 * @see ARNETWORK_Sender_ThreadRun()
 */
eARNETWORK_ERROR ARNETWORK_Receiver_CopyDataRecv (ARNETWORK_Receiver_t *receiverPtr, ARNETWORK_IOBuffer_t *outputBufferPtr, ARNETWORKAL_Frame_t *framePtr);

/*****************************************
 *
 *             implementation :
 *
 *****************************************/


ARNETWORK_Receiver_t* ARNETWORK_Receiver_New (ARNETWORKAL_Manager_t *networkALManager, unsigned int numberOfOutputBuff, ARNETWORK_IOBuffer_t **outputBufferPtrArr, ARNETWORK_IOBuffer_t **outputBufferPtrMap)
{
    /** -- Create a new receiver -- */

    /** local declarations */
    ARNETWORK_Receiver_t *receiverPtr = NULL;
    eARNETWORK_ERROR error = ARNETWORK_OK;

    /** Create the receiver */
    receiverPtr =  malloc (sizeof (ARNETWORK_Receiver_t));

    if (receiverPtr)
    {
        if(networkALManager != NULL)
        {
            receiverPtr->networkALManager = networkALManager;
        }
        else
        {
            error = ARNETWORK_ERROR_BAD_PARAMETER;
        }

        if(error == ARNETWORK_OK)
        {
            receiverPtr->isAlive = 1;
            receiverPtr->senderPtr = NULL;

            receiverPtr->numberOfOutputBuff = numberOfOutputBuff;
            receiverPtr->outputBufferPtrArr = outputBufferPtrArr;

            receiverPtr->outputBufferPtrMap = outputBufferPtrMap;
#ifdef ENABLE_MONITOR_INCOMING_DATA
            receiverPtr->inputEventFd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
#endif
        }

        /** delete the receiver if an error occurred */
        if (error != ARNETWORK_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_RECEIVER_TAG,"error: %s", ARNETWORK_Error_ToString (error));
            ARNETWORK_Receiver_Delete (&receiverPtr);
        }
    }

    return receiverPtr;
}

void ARNETWORK_Receiver_Delete (ARNETWORK_Receiver_t **receiverPtrAddr)
{
    /** -- Delete the Receiver -- */

    /** local declarations */
    ARNETWORK_Receiver_t *receiverPtr = NULL;

    if (receiverPtrAddr)
    {
        receiverPtr = *receiverPtrAddr;

        if (receiverPtr)
        {
#ifdef ENABLE_MONITOR_INCOMING_DATA
            close(receiverPtr->inputEventFd);
            receiverPtr->inputEventFd = -1;
#endif
            free (receiverPtr);
            receiverPtr = NULL;
        }
        *receiverPtrAddr = NULL;
    }
}

void* ARNETWORK_Receiver_ThreadRun (void *data)
{
    /** -- Manage the reception of the data on the Receiver' socket. -- */

    /** local declarations */
    ARNETWORK_Receiver_t *receiverPtr = data;
    ARNETWORKAL_Frame_t frame ;
    ARNETWORK_IOBuffer_t* outBufferPtrTemp = NULL;
    eARNETWORK_ERROR error = ARNETWORK_OK;
    eARNETWORKAL_MANAGER_RETURN result = ARNETWORKAL_MANAGER_RETURN_DEFAULT;
    uint8_t ackSeqNumData = 0;
    struct timespec now;

    while (receiverPtr->isAlive)
    {
        /** wait a receipt */
        if (receiverPtr->networkALManager->receive(receiverPtr->networkALManager) == ARNETWORKAL_MANAGER_RETURN_DEFAULT)
        {
            /** for each frame present in the receiver buffer */
            result = receiverPtr->networkALManager->popFrame(receiverPtr->networkALManager, &frame);
            while (result == ARNETWORKAL_MANAGER_RETURN_DEFAULT)
            {
                /* Special handling of internal frames */
                if (frame.id < ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_MAX)
                {
                    switch (frame.id)
                    {
                    case ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PING:
                        /* Ping, send the corresponding pong */
                    {
                        ARNETWORK_Sender_SendPong (receiverPtr->senderPtr, frame.dataPtr, frame.size - offsetof(ARNETWORKAL_Frame_t, dataPtr));
                    }
                    break;
                    case ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PONG:
                        /* Pong, tells the sender that we got a response */
                    {
                        struct timespec dataTime;
                        memcpy (&dataTime, frame.dataPtr, sizeof (struct timespec));
                        ARSAL_Time_GetTime(&now);
                        ARNETWORK_Sender_GotPingAck (receiverPtr->senderPtr, &dataTime, &now);
                    }
                    break;
                    default:
                        /* Do nothing as we don't know how to handle it */
                        break;
                    }
                }

                /** management by the command type */
                switch (frame.type)
                {
                case ARNETWORKAL_FRAME_TYPE_ACK:

                    /** get the acknowledge sequence number from the data */
                    memcpy (&ackSeqNumData, frame.dataPtr, sizeof(uint8_t));
                    ARSAL_PRINT (ARSAL_PRINT_VERBOSE, ARNETWORK_RECEIVER_TAG, "[%p] - TYPE: ARNETWORKAL_FRAME_TYPE_ACK | SEQ:%d | ID:%d | SEQ ACK : %d", receiverPtr, frame.seq, frame.id, ackSeqNumData);
                    /** transmit the acknowledgement to the sender */
                    error = ARNETWORK_Sender_AckReceived (receiverPtr->senderPtr, ARNETWORK_Manager_IDAckToIDInput (receiverPtr->networkALManager, frame.id), ackSeqNumData);
                    if (error != ARNETWORK_OK)
                    {
                        switch (error)
                        {
                        case ARNETWORK_ERROR_IOBUFFER_BAD_ACK:
                            ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARNETWORK_RECEIVER_TAG, "[%p] Bad acknowledge, error: %s", receiverPtr, ARNETWORK_Error_ToString (error));
                            break;

                        default:
                            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_RECEIVER_TAG, "[%p] Acknowledge received, error: %s", receiverPtr, ARNETWORK_Error_ToString (error));
                            break;
                        }
                    }
                    break;

                case ARNETWORKAL_FRAME_TYPE_DATA:
                    ARSAL_PRINT (ARSAL_PRINT_VERBOSE, ARNETWORK_RECEIVER_TAG, "[%p] - TYPE: ARNETWORKAL_FRAME_TYPE_DATA | SEQ:%d | ID:%d", receiverPtr, frame.seq, frame.id);

                    /** push the data received in the output buffer targeted */
                    outBufferPtrTemp = receiverPtr->outputBufferPtrMap[frame.id];

                    if (outBufferPtrTemp != NULL)
                    {
                        /** lock the IOBuffer */
                        error = ARNETWORK_IOBuffer_Lock(outBufferPtrTemp);
                        if(error == ARNETWORK_OK)
                        {
                            int accept = ARNETWORK_IOBuffer_ShouldAcceptData (outBufferPtrTemp, frame.seq);
                            if (accept > 0)
                            {
                                error = ARNETWORK_Receiver_CopyDataRecv(receiverPtr, outBufferPtrTemp, &frame);
                            }
                            else if (accept == 0)
                            {
                                ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARNETWORK_RECEIVER_TAG, "[%p] Received a retry for buffer %d", receiverPtr, outBufferPtrTemp->ID);
                            }
                            else
                            {
                                ARSAL_PRINT (ARSAL_PRINT_WARNING, ARNETWORK_RECEIVER_TAG, "[%p] Received an old frame for buffer %d", receiverPtr, outBufferPtrTemp->ID);
                            }

                            /** unlock the IOBuffer */
                            ARNETWORK_IOBuffer_Unlock(outBufferPtrTemp);

                            if(error != ARNETWORK_OK)
                            {
                                ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_RECEIVER_TAG, "[%p] data received, error: %s", receiverPtr, ARNETWORK_Error_ToString (error));
                            }
                        }
                    }
                    break;

                case ARNETWORKAL_FRAME_TYPE_DATA_LOW_LATENCY:
                    ARSAL_PRINT (ARSAL_PRINT_VERBOSE, ARNETWORK_RECEIVER_TAG, "[%p] - TYPE: ARNETWORKAL_FRAME_TYPE_DATA_LOW_LATENCY | SEQ:%d | ID:%d", receiverPtr, frame.seq, frame.id);

                    /** push the data received in the output buffer targeted */
                    outBufferPtrTemp = receiverPtr->outputBufferPtrMap[frame.id];

                    if (outBufferPtrTemp != NULL)
                    {
                        /** lock the IOBuffer */
                        error = ARNETWORK_IOBuffer_Lock(outBufferPtrTemp);
                        if(error == ARNETWORK_OK)
                        {
                            int accept = ARNETWORK_IOBuffer_ShouldAcceptData (outBufferPtrTemp, frame.seq);
                            if (accept > 0)
                            {
                                error = ARNETWORK_Receiver_CopyDataRecv(receiverPtr, outBufferPtrTemp, &frame);
                            }
                            else if (accept == 0)
                            {
                                ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARNETWORK_RECEIVER_TAG, "[%p] Received a retry for buffer %d", receiverPtr, outBufferPtrTemp->ID);
                            }
                            else
                            {
                                ARSAL_PRINT (ARSAL_PRINT_WARNING, ARNETWORK_RECEIVER_TAG, "[%p] Received an old frame for buffer %d", receiverPtr, outBufferPtrTemp->ID);
                            }

                            /** unlock the IOBuffer */
                            ARNETWORK_IOBuffer_Unlock(outBufferPtrTemp);

                            if(error != ARNETWORK_OK)
                            {
                                ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_RECEIVER_TAG, "[%p] data received, error: %s", receiverPtr, ARNETWORK_Error_ToString (error));
                            }
                        }
                    }
                    break;

                case ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK:
                    ARSAL_PRINT (ARSAL_PRINT_VERBOSE, ARNETWORK_RECEIVER_TAG, "[%p] - TYPE: ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK | SEQ:%d | ID:%d", receiverPtr, frame.seq, frame.id);
                    /**
                     * push the data received in the output buffer targeted,
                     * save the sequence of the command and return an acknowledgement
                     */
                    outBufferPtrTemp = receiverPtr->outputBufferPtrMap[frame.id];

                    if (outBufferPtrTemp != NULL)
                    {
                        /** lock the IOBuffer */
                        error = ARNETWORK_IOBuffer_Lock(outBufferPtrTemp);
                        if(error == ARNETWORK_OK)
                        {
                            /** OutBuffer->seqWaitAck used to save the last seq */
                            int accept = ARNETWORK_IOBuffer_ShouldAcceptData (outBufferPtrTemp, frame.seq);
                            if (accept > 0)
                            {
                                error = ARNETWORK_Receiver_CopyDataRecv(receiverPtr, outBufferPtrTemp, &frame);
                            }
                            else if (accept == 0)
                            {
                                ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARNETWORK_RECEIVER_TAG, "[%p] Received a retry for buffer %d", receiverPtr, outBufferPtrTemp->ID);
                            }
                            else
                            {
                                ARSAL_PRINT (ARSAL_PRINT_WARNING, ARNETWORK_RECEIVER_TAG, "[%p] Received an old frame for buffer %d", receiverPtr, outBufferPtrTemp->ID);
                            }

                            /** unlock the IOBuffer */
                            ARNETWORK_IOBuffer_Unlock(outBufferPtrTemp);

                            // data are copied to the IOBuffer, send ACK
                            if (error == ARNETWORK_OK)
                            {
                                 /** sending ack even if the seq is not correct */
                                 error = ARNETWORK_Receiver_ReturnACK(receiverPtr, frame.id, frame.seq);
                                 if(error != ARNETWORK_OK)
                                 {
                                     int level = ARSAL_PRINT_ERROR;
                                     if (error == ARNETWORK_ERROR_BUFFER_SIZE)
                                     {
                                         level = ARSAL_PRINT_DEBUG;
                                     }
                                     ARSAL_PRINT(level, ARNETWORK_RECEIVER_TAG, "[%p] ReturnACK, error: %s", receiverPtr, ARNETWORK_Error_ToString(error));
                                 }
                            }
                            else
                            {
                                ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_RECEIVER_TAG, "[%p] data with ack received, error: %s", receiverPtr, ARNETWORK_Error_ToString (error));
                            }
                        }
                    }
                    break;

                default:
                    ARSAL_PRINT (ARSAL_PRINT_WARNING, ARNETWORK_RECEIVER_TAG, "[%p] !!! command type: %d not known  !!!", receiverPtr, frame.type);
                    break;
                }

                /** get the next frame*/
                result = receiverPtr->networkALManager->popFrame(receiverPtr->networkALManager, &frame);
            }
        }
    }

    return NULL;
}

void ARNETWORK_Receiver_Stop (ARNETWORK_Receiver_t *receiverPtr)
{
    /** -- stop the reception -- */
    receiverPtr->isAlive = 0;
}

eARNETWORK_ERROR ARNETWORK_Receiver_ReturnACK (ARNETWORK_Receiver_t *receiverPtr, int id, uint8_t seq)
{
    /** -- return an acknowledgement -- */
    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    ARNETWORK_IOBuffer_t* ACKIOBufferPtr = receiverPtr->outputBufferPtrMap[ARNETWORK_Manager_IDOutputToIDAck (receiverPtr->networkALManager, id)];

    if (ACKIOBufferPtr != NULL)
    {
        int isEmpty = ARNETWORK_RingBuffer_IsEmpty(ACKIOBufferPtr->dataDescriptorRBuffer);
        error = ARNETWORK_IOBuffer_AddData (ACKIOBufferPtr, (uint8_t*) &seq, sizeof(seq), NULL, NULL, 1);
        if (error == ARNETWORK_OK && isEmpty > 0)
        {
            ARNETWORK_Sender_SignalNewData (receiverPtr->senderPtr);
        }
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_Receiver_GetEventFd(ARNETWORK_Receiver_t *receiverPtr, int *fd)
{
	eARNETWORK_ERROR err = ARNETWORK_OK;

#ifdef ENABLE_MONITOR_INCOMING_DATA
	if (receiverPtr && fd)
		*fd = receiverPtr->inputEventFd;
	else
		err = ARNETWORK_ERROR_BAD_PARAMETER;
#else
	err = ARNETWORK_ERROR_RECEIVER;
#endif
	return err;
}

eARNETWORK_ERROR ARNETWORK_Receiver_WriteEventFd(ARNETWORK_Receiver_t *receiverPtr, uint64_t value)
{
	eARNETWORK_ERROR err = ARNETWORK_OK;

#ifdef ENABLE_MONITOR_INCOMING_DATA
	int ret;
	do {
		ret = write(receiverPtr->inputEventFd, &value, sizeof(value));
	} while ((ret < 0) && (errno == EINTR));
	if (ret < 0) {
		ret = errno;
		ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_RECEIVER_TAG, "[%p] Error: can't write to eventfd %s", receiverPtr, strerror(ret));
		err = ARNETWORK_ERROR_RECEIVER;
	}
#else
	err = ARNETWORK_ERROR_RECEIVER;
#endif

	return err;
}

eARNETWORK_ERROR ARNETWORK_Receiver_ReadEventFd(ARNETWORK_Receiver_t *receiverPtr, uint64_t *value)
{
	eARNETWORK_ERROR err = ARNETWORK_OK;

#ifdef ENABLE_MONITOR_INCOMING_DATA
	int ret;
	do {
		ret = read(receiverPtr->inputEventFd, value, sizeof(*value));
	} while ((ret < 0) && (errno == EINTR));
	if (ret < 0) {
		ret = errno;
		ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_RECEIVER_TAG, "[%p] Error: can't read from eventfd %s", receiverPtr, strerror(ret));
		err = ARNETWORK_ERROR_RECEIVER;
	}
#else
	err = ARNETWORK_ERROR_RECEIVER;
#endif

	return err;
}

/*****************************************
 *
 *             private implementation:
 *
 *****************************************/

eARNETWORK_ERROR ARNETWORK_Receiver_CopyDataRecv (ARNETWORK_Receiver_t *receiverPtr, ARNETWORK_IOBuffer_t *outputBufferPtr, ARNETWORKAL_Frame_t *framePtr)
{
    /** -- copy the data received to the output buffer -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    int semError = 0;
    int dataSize = 0;

    int nbNew = ARNETWORK_IOBuffer_ShouldAcceptData (outputBufferPtr, framePtr->seq);

    /** get the data size*/
    dataSize = framePtr->size - offsetof (ARNETWORKAL_Frame_t, dataPtr);

    /** if the output buffer can copy the data */
    if (ARNETWORK_IOBuffer_CanCopyData (outputBufferPtr))
    {
        /** copy the data in the IOBuffer */
        error = ARNETWORK_IOBuffer_AddData (outputBufferPtr, framePtr->dataPtr, dataSize, NULL, NULL, 1);
    }
    else
    {
        error = ARNETWORK_ERROR_BAD_PARAMETER;
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_RECEIVER_TAG, "[%p] Error: output buffer can't copy data", receiverPtr);
    }

    if (error == ARNETWORK_OK)
    {
        /** Keep buffer "miss count" accurate */
        outputBufferPtr->nbPackets++;
        outputBufferPtr->nbNetwork += nbNew;
        outputBufferPtr->seq = framePtr->seq;
        /** post a semaphore to indicate data ready to be read */
        semError = ARSAL_Sem_Post (&(outputBufferPtr->outputSem));

        if (semError)
        {
            error = ARNETWORK_ERROR_SEMAPHORE;
        }

#ifdef ENABLE_MONITOR_INCOMING_DATA
        /* write 1 in eventfd to wake up consumer */
        error = ARNETWORK_Receiver_WriteEventFd(receiverPtr, 1);
#endif
    }

    return error;
}
