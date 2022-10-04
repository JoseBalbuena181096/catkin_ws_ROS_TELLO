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
 * @file ARNETWORK_Manager.c
 * @brief network manager allow to send data acknowledged or not.
 * @date 28/09/2012
 * @author maxime.maitre@parrot.com
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <stdlib.h>

#include <inttypes.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Mutex.h>
#include <libARSAL/ARSAL_Socket.h>
#include <libARSAL/ARSAL_Time.h>

#include <libARNetwork/ARNETWORK_Error.h>
#include <libARNetworkAL/ARNETWORKAL_Frame.h>
#include <libARNetworkAL/ARNETWORKAL_Error.h>
#include "ARNETWORK_RingBuffer.h"
#include "ARNETWORK_DataDescriptor.h"
#include <libARNetwork/ARNETWORK_IOBufferParam.h>
#include "ARNETWORK_IOBuffer.h"
#include "ARNETWORK_Sender.h"
#include "ARNETWORK_Receiver.h"

#include <libARNetwork/ARNETWORK_Manager.h>
#include "ARNETWORK_Manager.h"

/*****************************************
 *
 *             define :
 *
 *****************************************/

#define ARNETWORK_MANAGER_TAG "ARNETWORK_Manager"

/*****************************************
 *
 *             private header:
 *
 *****************************************/

/**
 * @brief create manager's IOBuffers.
 * @warning only call by ARNETWORK_Manager_New()
 * @pre managerPtr->outputBufferPtrArr and managerPtr->inputBufferPtrArr must be allocated and not set to NULL.
 * @param manager The Manager
 * @param[in] inputParamArray array of the parameters of creation of the inputs. The array must contain as many parameters as the number of input buffer.
 * @param[in] outputParamArray array of the parameters of creation of the outputs. The array must contain as many parameters as the number of output buffer.
 * @return error equal to ARNETWORK_OK if the IOBuffer are correctly created otherwise see eARNETWORK_ERROR.
 * @see ARNETWORK_Manager_New()
 */
eARNETWORK_ERROR ARNETWORK_Manager_CreateIOBuffer (ARNETWORK_Manager_t *manager, ARNETWORK_IOBufferParam_t *inputParamArray, ARNETWORK_IOBufferParam_t *outputParamArray);

/**
 * @brief function called on disconnect
 * @param manager The networkAL manager
 * @param customData The custom Data
 */
void ARNETWORK_Manager_OnDisconnect (ARNETWORKAL_Manager_t *alManager, void *customData);

/*****************************************
 *
 *             implementation :
 *
 *****************************************/

ARNETWORK_Manager_t* ARNETWORK_Manager_New (ARNETWORKAL_Manager_t *networkALManager, unsigned int numberOfInput, ARNETWORK_IOBufferParam_t *inputParamArr, unsigned int numberOfOutput, ARNETWORK_IOBufferParam_t *outputParamArr, int pingDelayMs, ARNETWORK_Manager_OnDisconnect_t onDisconnectCallback, void* customData, eARNETWORK_ERROR *error)
{
    /* -- Create a new Manager -- */

    /* local declarations */
    ARNETWORK_Manager_t *manager = NULL;
    eARNETWORK_ERROR localError = ARNETWORK_OK;
    eARNETWORKAL_ERROR errorAL = ARNETWORKAL_OK;

    /* check parameters */
    if (networkALManager == NULL)
    {
        localError = ARNETWORK_ERROR_BAD_PARAMETER;
    }
    /* No Else: the checking parameters sets localError to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing */

    if (localError == ARNETWORK_OK)
    {
        /* Create the Manager */
        manager = malloc (sizeof (ARNETWORK_Manager_t));
        if (manager != NULL)
        {
            /* Initialize to default values */
            manager->networkALManager = NULL;
            manager->sender = NULL;
            manager->receiver = NULL;
            manager->inputBufferArray = NULL;
            manager->outputBufferArray = NULL;
            manager->internalInputBufferArray = NULL;
            manager->numberOfOutput = 0;
            manager->numberOfOutputWithoutAck = 0;
            manager->numberOfInput = 0;
            manager->numberOfInputWithoutAck = 0;
            manager->numberOfInternalInputs = 0;
            manager->inputBufferMap = NULL;
            manager->outputBufferMap = NULL;
            manager->onDisconnect = onDisconnectCallback;
            manager->customData = customData;
        }
        else
        {
            localError = ARNETWORK_ERROR_ALLOC;
        }
    }
    /* No else: skipped by an error */

    if (localError == ARNETWORK_OK)
    {
        manager->networkALManager = networkALManager;

        /* set the onDisconnect callback */
        errorAL = ARNETWORKAL_Manager_SetOnDisconnectCallback (networkALManager, &ARNETWORK_Manager_OnDisconnect, (void *) manager);

        /* Manage the networkAL error */
        switch (errorAL)
        {
        case ARNETWORKAL_OK:
            /* Do nothing : no error */
            break;

        case ARNETWORKAL_ERROR_MANAGER_OPERATION_NOT_SUPPORTED:
            /* setOnDisconnectCallback not supported by this networkALManager */
            ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARNETWORK_MANAGER_TAG, "setOnDisconnectCallback not supported by this networkALManager");
            break;

        case ARNETWORKAL_ERROR_BAD_PARAMETER:
            /* bad parameter */
            localError = ARNETWORK_ERROR_BAD_PARAMETER;
            break;

        default:
            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_MANAGER_TAG, "error %d returned by setOnDisconnectCallback unexpected.", errorAL);
            break;
        }
    }
    /* No else: skipped by an error */

    /*
     * For each output buffer a buffer of acknowledgement is add and referenced
     * in the output buffer list and the input buffer list.
     */

    if (localError == ARNETWORK_OK)
    {
        /*
         * Allocate the output buffer list of size of the number of output plus the number of
         * buffer of acknowledgement. The size of the list is then two times the number of output.
         */
        manager->numberOfOutputWithoutAck = numberOfOutput;
        manager->numberOfOutput = 2 * numberOfOutput;
        manager->outputBufferArray = calloc (manager->numberOfOutput, sizeof (ARNETWORK_IOBuffer_t*));
        if (manager->outputBufferArray == NULL)
        {
            localError = ARNETWORK_ERROR_ALLOC;
            manager->numberOfOutput = 0;
            manager->numberOfOutputWithoutAck = 0;
        }
    }
    /* No else: skipped by an error */

    if (localError == ARNETWORK_OK)
    {
        /*
         * Allocate the input buffer list of size of the number of input plus the number of
         * buffer of acknowledgement.
         * The size of the list is then number of input plus the number of output.
         */
        manager->numberOfInputWithoutAck = numberOfInput;
        manager->numberOfInput = numberOfInput + numberOfOutput;
        manager->inputBufferArray = calloc (manager->numberOfInput, sizeof (ARNETWORK_IOBuffer_t*));
        if (manager->inputBufferArray == NULL)
        {
            localError = ARNETWORK_ERROR_ALLOC;
            manager->numberOfInput = 0;
            manager->numberOfInputWithoutAck = numberOfOutput;
        }
    }
    /* No else: skipped by an error */

    if (localError == ARNETWORK_OK)
    {
        /*
         * Allocate the internal input buffer list
         * Size is the number of internal buffers
         */
        manager->numberOfInternalInputs = ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_MAX;
        manager->internalInputBufferArray = calloc (manager->numberOfInternalInputs, sizeof (ARNETWORK_IOBuffer_t*));
        if (manager->internalInputBufferArray == NULL)
        {
            localError = ARNETWORK_ERROR_ALLOC;
            manager->numberOfInternalInputs = 0;
        }
    }
    /* No else: skipped by an error */

    if (localError == ARNETWORK_OK)
    {
        /* Allocate the output buffer map  storing the IOBuffer by their identifier */
        manager->outputBufferMap = calloc (manager->networkALManager->maxIds, sizeof (ARNETWORK_IOBuffer_t*));
        if (manager->outputBufferMap == NULL)
        {
            localError = ARNETWORK_ERROR_ALLOC;
        }
    }
    /* No else: skipped by an error */

    if (localError == ARNETWORK_OK)
    {
        /* Allocate the input buffer map  storing the IOBuffer by their identifier */
        manager->inputBufferMap = calloc (manager->networkALManager->maxIds, sizeof (ARNETWORK_IOBuffer_t*));
        if (manager->inputBufferMap == NULL)
        {
            localError = ARNETWORK_ERROR_ALLOC;
        }
    }
    /* No else: skipped by an error */

    if ((localError == ARNETWORK_OK) && (networkALManager->maxBufferSize == 0))
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_MANAGER_TAG, "maxBufferSize is 0. Did you initialize ARNetworkAL correctly?");
        localError = ARNETWORK_ERROR_BAD_PARAMETER;
    }

    if (localError == ARNETWORK_OK)
    {
        /* Create manager's IOBuffers and stor it in the inputMap and outputMap*/
        localError = ARNETWORK_Manager_CreateIOBuffer (manager, inputParamArr, outputParamArr);
    }
    /* No else: skipped by an error */

    if (localError == ARNETWORK_OK)
    {
        /* Create the Sender */
        manager->sender = ARNETWORK_Sender_New (manager->networkALManager, manager->numberOfInput, manager->inputBufferArray, manager->numberOfInternalInputs, manager->internalInputBufferArray, manager->inputBufferMap, pingDelayMs);
        if (manager->sender == NULL)
        {
            localError = ARNETWORK_ERROR_MANAGER_NEW_SENDER;
        }
    }
    /* No else: skipped by an error */

    if (localError == ARNETWORK_OK)
    {
        /* Create the Receiver */
        manager->receiver = ARNETWORK_Receiver_New (manager->networkALManager, manager->numberOfOutput, manager->outputBufferArray, manager->outputBufferMap);
        if (manager->receiver != NULL)
        {
            manager->receiver->senderPtr = manager->sender;
        }
        else
        {
            localError = ARNETWORK_ERROR_MANAGER_NEW_RECEIVER;
        }
    }
    /* No else: skipped by an error */

    /* delete the Manager if an error occurred */
    if (localError != ARNETWORK_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_MANAGER_TAG, "error: %s", ARNETWORK_Error_ToString (localError));
        ARNETWORK_Manager_Delete (&manager);
    }
    /* No else: skipped by an error */

    /* return the error */
    if (error != NULL)
    {
        *error = localError;
    }
    /* No else: error is nor returned */

    return manager;
}

void ARNETWORK_Manager_Delete (ARNETWORK_Manager_t **manager)
{
    /** -- Delete the Manager -- */

    /** local declarations */
    int bufferIndex = 0;

    if (manager)
    {
        if ((*manager))
        {
            ARNETWORK_Sender_Delete (&((*manager)->sender));
            ARNETWORK_Receiver_Delete (&((*manager)->receiver));

            /* Delete all internal buffers */
            for (bufferIndex = 0; bufferIndex < (*manager)->numberOfInternalInputs; ++bufferIndex)
            {
                ARNETWORK_IOBuffer_t **buffer = &((*manager)->internalInputBufferArray[bufferIndex]);
                if (*buffer != NULL)
                {
                    ARNETWORK_IOBuffer_Delete (buffer);
                }
                (*manager)->internalInputBufferArray[bufferIndex] = NULL;
            }
            free ((*manager)->internalInputBufferArray);
            (*manager)->internalInputBufferArray = NULL;

            /** Delete all output buffers including the buffers of acknowledgement */
            for (bufferIndex = 0; bufferIndex< (*manager)->numberOfOutput ; ++bufferIndex)
            {
                ARNETWORK_IOBuffer_Delete (&((*manager)->outputBufferArray[bufferIndex]));
            }
            free ((*manager)->outputBufferArray);
            (*manager)->outputBufferArray = NULL;

            /** Delete the input buffers but not the buffers of acknowledgement already deleted */
            for (bufferIndex = 0; bufferIndex< (*manager)->numberOfInputWithoutAck ; ++bufferIndex)
            {
                ARNETWORK_IOBuffer_Delete (&((*manager)->inputBufferArray[bufferIndex]));
            }
            free ((*manager)->inputBufferArray);
            (*manager)->inputBufferArray = NULL;

            free ((*manager)->inputBufferMap);
            (*manager)->inputBufferMap = NULL;

            free ((*manager)->outputBufferMap);
            (*manager)->outputBufferMap = NULL;

            (*manager)->networkALManager = NULL;

            free ((*manager));
            (*manager) = NULL;
        }
    }
}

void* ARNETWORK_Manager_SendingThreadRun (void *data)
{
    /** -- Manage the sending of the data -- */

    /** local declarations */
    ARNETWORK_Manager_t *manager = data;
    void *ret = NULL;

    /** check paratemters */
    if (manager != NULL)
    {
        ret = ARNETWORK_Sender_ThreadRun (manager->sender);
    }
    else
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_MANAGER_TAG, "error: %s", ARNETWORK_Error_ToString (ARNETWORK_ERROR_BAD_PARAMETER));
    }

    return ret;
}

void* ARNETWORK_Manager_ReceivingThreadRun (void *data)
{
    /** -- Manage the reception of the data -- */

    /** local declarations */
    ARNETWORK_Manager_t *manager = data;
    void *ret = NULL;

    /** check paratemters */
    if (manager != NULL)
    {
        ret = ARNETWORK_Receiver_ThreadRun (manager->receiver);
    }
    else
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARNETWORK_MANAGER_TAG,"error: %s", ARNETWORK_Error_ToString (ARNETWORK_ERROR_BAD_PARAMETER));
    }

    return ret;
}

void ARNETWORK_Manager_Stop (ARNETWORK_Manager_t *manager)
{
    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARNETWORK_MANAGER_TAG, "%s", "");
    /** -- stop the threads of sending and reception -- */

    /** check paratemters */
    if (manager != NULL)
    {
        ARNETWORK_Sender_Stop (manager->sender);
        ARNETWORK_Receiver_Stop (manager->receiver);

        /* unlock all functions of the networkAL to permit to join the threads */
        if (manager->networkALManager->unlock != NULL)
        {
            manager->networkALManager->unlock (manager->networkALManager);
        }
    }
}

eARNETWORK_ERROR ARNETWORK_Manager_Flush (ARNETWORK_Manager_t *manager)
{
    /** -- Flush all buffers of the network manager -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    int bufferIndex = 0;

#ifdef ENABLE_MONITOR_INCOMING_DATA
    /* read from eventfd before flushing input buffers */
    uint64_t unused;
    error = ARNETWORK_Receiver_ReadEventFd(manager->receiver, &unused);
#endif

    /** Flush all output buffers including the buffers of acknowledgement */
    for (bufferIndex = 0 ; ((bufferIndex< manager->numberOfOutput) && (error == ARNETWORK_OK)) ; ++bufferIndex)
    {
        ARNETWORK_Manager_FlushOutputBuffer (manager, bufferIndex);
    }

    /** Flush the input buffers but not the buffers of acknowledgement already flushed */
    for (bufferIndex = 0 ; ((bufferIndex< manager->numberOfInputWithoutAck) && (error == ARNETWORK_OK)) ; ++bufferIndex)
    {
        ARNETWORK_Manager_FlushInputBuffer (manager, bufferIndex);
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_Manager_GetInputDataEventFd (ARNETWORK_Manager_t *managerPtr, int *fd)
{
    return managerPtr ? ARNETWORK_Receiver_GetEventFd(managerPtr->receiver, fd) : ARNETWORK_ERROR_BAD_PARAMETER;
}

eARNETWORK_ERROR ARNETWORK_Manager_SendData (ARNETWORK_Manager_t *manager, int inputBufferID, uint8_t *data, int dataSize, void *customData, ARNETWORK_Manager_Callback_t callback, int doDataCopy)
{
    /** -- Add data to send in a IOBuffer using fixed size data -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    ARNETWORK_IOBuffer_t *inputBuffer = NULL;
    int bufferWasEmpty = 0;

    /** check paratemters:
     *  -   the manager ponter is not NUL
     *  -   the data pointer is not NULL
     *  -   the callback is not NULL
     */
    if ((manager != NULL) && (data != NULL) && (callback != NULL))
    {
        /** get the address of the inputBuffer */
        inputBuffer = manager->inputBufferMap[inputBufferID];

        if (inputBuffer == NULL)
        {
            error = ARNETWORK_ERROR_ID_UNKNOWN;
        }
    }
    else
    {
        error = ARNETWORK_ERROR_BAD_PARAMETER;
    }

    if (error == ARNETWORK_OK)
    {
        /** lock the IOBuffer */
        error = ARNETWORK_IOBuffer_Lock(inputBuffer);
    }

    if(error == ARNETWORK_OK)
    {
        bufferWasEmpty = ARNETWORK_RingBuffer_IsEmpty(inputBuffer->dataDescriptorRBuffer);
    }

    if(error == ARNETWORK_OK)
    {
        /** add the data in the inputBuffer */
        error = ARNETWORK_IOBuffer_AddData (inputBuffer, data, dataSize, customData, callback, doDataCopy);
        ARNETWORK_IOBuffer_Unlock(inputBuffer);
    }

    if (error == ARNETWORK_OK)
    {
        if ((inputBuffer->dataType == ARNETWORKAL_FRAME_TYPE_DATA_LOW_LATENCY) ||
            (bufferWasEmpty > 0))
        {
            ARNETWORK_Sender_SignalNewData (manager->sender);
        }
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_Manager_ReadData (ARNETWORK_Manager_t *manager, int outputBufferID, uint8_t *data, int dataLimitSize, int *readSize)
{
    /** -- Read data received in a IOBuffer using variable size data (blocking function) -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    ARNETWORK_IOBuffer_t *outputBuffer = NULL;
    int semError = 0;

    /** check paratemters */
    if (manager != NULL)
    {
        /** get the address of the outputBuffer */
        outputBuffer = manager->outputBufferMap[outputBufferID];

        /** check outputBuffer */
        if (outputBuffer == NULL)
        {
            error = ARNETWORK_ERROR_ID_UNKNOWN;
        }
    }
    else
    {
        error = ARNETWORK_ERROR_BAD_PARAMETER;
    }

    if (error == ARNETWORK_OK)
    {
        /** try to take the semaphore */
        semError = ARSAL_Sem_Wait (&(outputBuffer->outputSem));

        if (semError)
        {
            switch (errno)
            {
            case EAGAIN : /** no semaphore */
                error = ARNETWORK_ERROR_BUFFER_EMPTY;
                break;

            default:
                error = ARNETWORK_ERROR_SEMAPHORE;
                break;
            }
        }
    }

    /** read data */

    if (error == ARNETWORK_OK)
    {
        /** lock the IOBuffer */
        error = ARNETWORK_IOBuffer_Lock (outputBuffer);
    }

    if (error == ARNETWORK_OK)
    {
        error = ARNETWORK_IOBuffer_ReadData (outputBuffer, data, dataLimitSize, readSize);

        /** unlock the IOBuffer */
        ARNETWORK_IOBuffer_Unlock (outputBuffer);
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_Manager_TryReadData (ARNETWORK_Manager_t *manager, int outputBufferID, uint8_t *data, int dataLimitSize, int *readSize)
{
    /** -- try to read data received in a IOBuffer using variable size data (non-blocking function) -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    ARNETWORK_IOBuffer_t *outputBuffer = NULL;
    int semError = 0;

    /** check paratemters */
    if (manager != NULL)
    {
        /** get the address of the outputBuffer */
        outputBuffer = manager->outputBufferMap[outputBufferID];

        /** check outputBuffer */
        if (outputBuffer == NULL)
        {
            error = ARNETWORK_ERROR_ID_UNKNOWN;
        }
    }
    else
    {
        error = ARNETWORK_ERROR_BAD_PARAMETER;
    }

    if (error == ARNETWORK_OK)
    {
        /** try to take the semaphore */
        semError = ARSAL_Sem_Trywait (&(outputBuffer->outputSem));

        if (semError)
        {
            switch (errno)
            {
            case EAGAIN : /** no semaphore */
                error = ARNETWORK_ERROR_BUFFER_EMPTY;
                break;

            default:
                error = ARNETWORK_ERROR_SEMAPHORE;
                break;
            }
        }
    }

    /** read data */

    if (error == ARNETWORK_OK)
    {
        /** lock the IOBuffer */
        error = ARNETWORK_IOBuffer_Lock (outputBuffer);
    }

    if (error == ARNETWORK_OK)
    {
        error = ARNETWORK_IOBuffer_ReadData (outputBuffer, data, dataLimitSize, readSize);

        /** unlock the IOBuffer */
        ARNETWORK_IOBuffer_Unlock (outputBuffer);
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_Manager_ReadDataWithTimeout (ARNETWORK_Manager_t *manager, int outputBufferID, uint8_t *data, int dataLimitSize, int *readSize, int timeoutMs)
{
    /** -- Read, with timeout, a data received in IOBuffer using variable size data -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    ARNETWORK_IOBuffer_t *outputBuffer = NULL;
    int semError = 0;
    struct timespec semTimeout;

    /** check paratemters */
    if (manager != NULL)
    {
        outputBuffer = manager->outputBufferMap[outputBufferID];

        /** check pOutputBuffer */
        if (outputBuffer == NULL)
        {
            error = ARNETWORK_ERROR_ID_UNKNOWN;
        }
    }
    else
    {
        error = ARNETWORK_ERROR_BAD_PARAMETER;
    }

    if (error == ARNETWORK_OK)
    {
        /** convert timeoutMs in timespec */
        semTimeout.tv_sec = timeoutMs / 1000;
        semTimeout.tv_nsec = (timeoutMs % 1000) * 1000000;

        /** try to take the semaphore with timeout*/
        semError = ARSAL_Sem_Timedwait (&(outputBuffer->outputSem), &semTimeout);

        if (semError)
        {
            switch (errno)
            {
            case ETIMEDOUT : /** semaphore time out */
                error = ARNETWORK_ERROR_BUFFER_EMPTY;
                break;

            default:
                error = ARNETWORK_ERROR_SEMAPHORE;
                break;
            }
        }
    }

    /** read data */

    if (error == ARNETWORK_OK)
    {
        /** lock the IOBuffer */
        error = ARNETWORK_IOBuffer_Lock (outputBuffer);
    }

    if (error == ARNETWORK_OK)
    {
        error = ARNETWORK_IOBuffer_ReadData (outputBuffer, data, dataLimitSize, readSize);

        /** unlock the IOBuffer */
        ARNETWORK_IOBuffer_Unlock (outputBuffer);
    }

    return error;
}

/*****************************************
 *
 *             private implementation:
 *
 *****************************************/

eARNETWORK_ERROR ARNETWORK_Manager_CreateIOBuffer (ARNETWORK_Manager_t *manager, ARNETWORK_IOBufferParam_t *inputParamArray, ARNETWORK_IOBufferParam_t *outputParamArray)
{
    /** -- Create manager's IoBuffers --*/

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    int inputIndex = 0;
    int outputIndex = 0;
    int indexAckOutput = 0;
    ARNETWORK_IOBufferParam_t paramNewACK;
    ARNETWORK_IOBufferParam_t paramPingBuffer;
    ARNETWORK_IOBufferParam_t paramPongBuffer;

    /** Initialize the default parameters for the buffers of acknowledgement. */
    ARNETWORK_IOBufferParam_DefaultInit (&paramNewACK);
    paramNewACK.dataType = ARNETWORKAL_FRAME_TYPE_ACK;
    paramNewACK.numberOfCell = 1;
    paramNewACK.dataCopyMaxSize = sizeof (( (ARNETWORKAL_Frame_t *)NULL)->seq);
    paramNewACK.isOverwriting = 0;

    /** Initialize the ping buffers parameters */
    ARNETWORK_IOBufferParam_DefaultInit (&paramPingBuffer);
    paramPingBuffer.dataType = ARNETWORKAL_FRAME_TYPE_DATA;
    paramPingBuffer.numberOfCell = 1;
    paramPingBuffer.dataCopyMaxSize = sizeof (struct timespec);
    paramPingBuffer.isOverwriting = 1;
    ARNETWORK_IOBufferParam_DefaultInit (&paramPongBuffer);
    paramPongBuffer.dataType = ARNETWORKAL_FRAME_TYPE_DATA;
    paramPongBuffer.numberOfCell = 1;
    paramPongBuffer.dataCopyMaxSize = 32; // The struct timespec can be greater than our own in the remote.
    paramPongBuffer.isOverwriting = 1;

    /**
     *  For each output buffer a buffer of acknowledgement is add and referenced
     *  in the output buffer list and the input buffer list.
     */

    /*
     * Create the input/output buffers for the "library reserved" part
     * - Start by creating the input-only buffers
     * - Then create all output buffers, and add loopback buffers to the end of the internal input buffers
     */
    outputIndex = 0;
    inputIndex  = 0;
    /* Generate inputs */
    // Iterate on all ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_XXX values
    //  - ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PING
    if (error == ARNETWORK_OK)
    {
        paramPingBuffer.ID = ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PING;
        manager->internalInputBufferArray[inputIndex] = ARNETWORK_IOBuffer_New (&paramPingBuffer, 1);
        if (manager->internalInputBufferArray[inputIndex] == NULL)
        {
            error = ARNETWORK_ERROR_MANAGER_NEW_IOBUFFER;
        }
        manager->inputBufferMap [ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PING] = manager->internalInputBufferArray [inputIndex];
        inputIndex++;
    }
    //  - ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PONG
    if (error == ARNETWORK_OK)
    {
        paramPongBuffer.ID = ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PONG;
        manager->internalInputBufferArray[inputIndex] = ARNETWORK_IOBuffer_New (&paramPongBuffer, 1);
        if (manager->internalInputBufferArray[inputIndex] == NULL)
        {
            error = ARNETWORK_ERROR_MANAGER_NEW_IOBUFFER;
        }
        manager->inputBufferMap [ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PONG] = manager->internalInputBufferArray [inputIndex];
        inputIndex++;
    }

    /* Generate outputs */
    // Iterate on all ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_XXX values
    //  - Don't create output buffer for ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PING;
    //  - Don't create output buffer for ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PONG;

    /* END OF INTERNAL BUFFERS CREATION */

    /** Create the output buffers and the buffers of acknowledgement */
    for (outputIndex = 0; outputIndex < manager->numberOfOutputWithoutAck && error == ARNETWORK_OK ; ++outputIndex)
    {
        /** check parameters */
        /** -   all output buffer must have the ability to copy */
        /** -   id must be within range ]ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_MAX; ackIdOffset] */
        if ((outputParamArray[outputIndex].ID >= (manager->networkALManager->maxIds / 2)) ||
            (outputParamArray[outputIndex].ID <  ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_MAX) ||
            (outputParamArray[outputIndex].dataCopyMaxSize == 0))
        {
            if (outputParamArray[outputIndex].dataCopyMaxSize == 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARNETWORK_MANAGER_TAG, "outputParamArray[%d].dataCopyMaxSize == 0", outputIndex);
            }
            else
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARNETWORK_MANAGER_TAG, "outputParamArray[%d] has a bad ID (%d). The ID should be in the range : ]%d; %d]", outputIndex, outputParamArray[outputIndex].ID, ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_MAX, (manager->networkALManager->maxIds / 2));
            }
            
            error = ARNETWORK_ERROR_BAD_PARAMETER;
        }

        /** Check for special (negative) values for dataCopyMaxSize. */
        if (outputParamArray[outputIndex].dataCopyMaxSize < 0)
        {
            if (outputParamArray[outputIndex].dataCopyMaxSize == ARNETWORK_IOBUFFERPARAM_DATACOPYMAXSIZE_USE_MAX)
            {
                /* Set dataCopyMaxSize to the maximum value allowed by ARNetworkAL. */
                outputParamArray[outputIndex].dataCopyMaxSize = manager->networkALManager->maxBufferSize;
            }
            else
            {
                error = ARNETWORK_ERROR_BAD_PARAMETER;
            }
        }

        if (error == ARNETWORK_OK)
        {
            /** Create the output buffer */
            manager->outputBufferArray[outputIndex] = ARNETWORK_IOBuffer_New (&(outputParamArray[outputIndex]), 0);
            if (manager->outputBufferArray[outputIndex] == NULL)
            {
                error = ARNETWORK_ERROR_MANAGER_NEW_IOBUFFER;
            }
        }

        if (error == ARNETWORK_OK)
        {
            /** Create the buffer of acknowledgement associated with the output buffer */

            paramNewACK.ID = ARNETWORK_Manager_IDOutputToIDAck (manager->networkALManager, outputParamArray[outputIndex].ID);
            indexAckOutput = manager->numberOfOutputWithoutAck + outputIndex;

            manager->outputBufferArray[indexAckOutput] = ARNETWORK_IOBuffer_New (&paramNewACK, 1);
            if (manager->outputBufferArray[indexAckOutput] == NULL)
            {
                error = ARNETWORK_ERROR_MANAGER_NEW_IOBUFFER;
            }
        }

        if (error == ARNETWORK_OK)
        {
            /** store buffer of acknowledgement at the end of the output and input buffer lists. */
            manager->inputBufferArray[manager->numberOfInputWithoutAck + outputIndex] = manager->outputBufferArray[indexAckOutput];

            /** store the outputBuffer and the buffer of acknowledgement in the IOBuffer Maps*/
            manager->outputBufferMap[manager->outputBufferArray[outputIndex]->ID] = manager->outputBufferArray[outputIndex];
            manager->outputBufferMap[manager->outputBufferArray[indexAckOutput]->ID] = manager->outputBufferArray[indexAckOutput];
            manager->inputBufferMap[manager->outputBufferArray[indexAckOutput]->ID] = manager->outputBufferArray[indexAckOutput];
        }
    }

    /** Create the input buffers */
    for (inputIndex = 0; inputIndex< manager->numberOfInputWithoutAck && error == ARNETWORK_OK ; ++inputIndex)
    {
        /** check parameters: */
        /** -   id is smaller than the id acknowledge offset */
        /** -   dataCopyMaxSize isn't too big */
        if ((inputParamArray[inputIndex].ID >= (manager->networkALManager->maxIds / 2)) ||
            (inputParamArray[inputIndex].ID <  ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_MAX))
        {
            error = ARNETWORK_ERROR_BAD_PARAMETER;
        }

        /** Check for special (negative) values for dataCopyMaxSize. */
        if (inputParamArray[inputIndex].dataCopyMaxSize < 0)
        {
            if (inputParamArray[inputIndex].dataCopyMaxSize == ARNETWORK_IOBUFFERPARAM_DATACOPYMAXSIZE_USE_MAX)
            {
                /* Set dataCopyMaxSize to the maximum value allowed by ARNetworkAL (minus the size of the header). */
                inputParamArray[inputIndex].dataCopyMaxSize = manager->networkALManager->maxBufferSize;
            }
            else
            {
                /* Unknown special value. */
                error = ARNETWORK_ERROR_BAD_PARAMETER;
            }
        }

        /* Check final buffer size. */
        if (inputParamArray[inputIndex].dataCopyMaxSize > 0 && ((uint32_t)inputParamArray[inputIndex].dataCopyMaxSize > manager->networkALManager->maxBufferSize))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARNETWORK_MANAGER_TAG, "Final dataCopyMaxSize is higher than tha maximum allowed data size (%d).", manager->networkALManager->maxBufferSize);
            error = ARNETWORK_ERROR_BAD_PARAMETER;
        }

        if (error == ARNETWORK_OK)
        {
            /** Create the intput buffer */
            manager->inputBufferArray[inputIndex] = ARNETWORK_IOBuffer_New (&(inputParamArray[inputIndex]), 0);
            if (manager->inputBufferArray[inputIndex] == NULL)
            {
                error = ARNETWORK_ERROR_MANAGER_NEW_IOBUFFER;
            }
        }

        if (error == ARNETWORK_OK)
        {
            /** store the inputBuffer in the ioBuffer Map */
            manager->inputBufferMap[manager->inputBufferArray[inputIndex]->ID] = manager->inputBufferArray[inputIndex];
        }
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_Manager_FlushInputBuffer (ARNETWORK_Manager_t *manager, int inBufferID)
{
    eARNETWORK_ERROR error = ARNETWORK_OK;
    if (manager == NULL)
    {
        error = ARNETWORK_ERROR_BAD_PARAMETER;
    }
    else
    {
        ARNETWORK_IOBuffer_t *buffer = manager->inputBufferMap[inBufferID];
        if (buffer != NULL)
        {
            /** lock the IOBuffer */
            error = ARNETWORK_IOBuffer_Lock (buffer);

            if (error == ARNETWORK_OK)
            {
                /** flush the IOBuffer*/
                error = ARNETWORK_IOBuffer_Flush (buffer);

                /** unlock the IOBuffer */
                ARNETWORK_IOBuffer_Unlock (buffer);
            }
        }
        else
        {
            error = ARNETWORK_ERROR_BAD_PARAMETER;
        }
    }
    return error;
}

eARNETWORK_ERROR ARNETWORK_Manager_FlushOutputBuffer (ARNETWORK_Manager_t *manager, int outBufferID)
{
    eARNETWORK_ERROR error = ARNETWORK_OK;
    if (manager == NULL)
    {
        error = ARNETWORK_ERROR_BAD_PARAMETER;
    }
    else
    {
        ARNETWORK_IOBuffer_t *buffer = manager->outputBufferMap[outBufferID];

        if (buffer != NULL)
        {
            /** lock the IOBuffer */
            error = ARNETWORK_IOBuffer_Lock (buffer);

            if (error == ARNETWORK_OK)
            {
                /** flush the IOBuffer*/
                error = ARNETWORK_IOBuffer_Flush (buffer);

                /** unlock the IOBuffer */
                ARNETWORK_IOBuffer_Unlock (buffer);
            }
        }
        else
        {
            error = ARNETWORK_ERROR_BAD_PARAMETER;
        }
    }
    return error;
}

int ARNETWORK_Manager_GetEstimatedLatency (ARNETWORK_Manager_t *manager)
{
    int result = -1;
    if (manager != NULL)
    {
        result = ARNETWORK_Sender_GetPing (manager->sender);
    }
    return result;
}

int ARNETWORK_Manager_GetEstimatedMissPercentage (ARNETWORK_Manager_t *manager, int outBufferID)
{
    eARNETWORK_ERROR error = ARNETWORK_OK;
    int result = 0;
    if (manager == NULL)
    {
        error = ARNETWORK_ERROR_BAD_PARAMETER;
    }
    else
    {
        ARNETWORK_IOBuffer_t *buffer = manager->outputBufferMap[outBufferID];

        if (buffer != NULL)
        {
            /** lock the IOBuffer */
            error = ARNETWORK_IOBuffer_Lock (buffer);

            if (error == ARNETWORK_OK)
            {
                /** Gets the buffer estimated miss percentage */
                result = ARNETWORK_IOBuffer_GetEstimatedMissPercentage (buffer);
                if (result < 0)
                {
                    error = (eARNETWORK_ERROR)result;
                }

                /** unlock the IOBuffer */
                ARNETWORK_IOBuffer_Unlock (buffer);
            }
        }
        else
        {
            error = ARNETWORK_ERROR_BAD_PARAMETER;
        }
    }

    if (error != ARNETWORK_OK)
    {
        return (int)error;
    }
    else
    {
        return result;
    }
}

eARNETWORK_ERROR ARNETWORK_Manager_SetMinimumTimeBetweenSends (ARNETWORK_Manager_t *manager, int minimumTimeMs)
{
    if ((manager == NULL) ||
        (manager->sender == NULL))
    {
        return ARNETWORK_ERROR_BAD_PARAMETER;
    }
    manager->sender->minimumTimeBetweenSendsMs = minimumTimeMs;
    return ARNETWORK_OK;
}

void ARNETWORK_Manager_OnDisconnect (ARNETWORKAL_Manager_t *alManager, void *customData)
{
    /* -- function called on disconnect -- */

    ARNETWORK_Manager_t *manager = (ARNETWORK_Manager_t *)customData;
    
    if (manager != NULL)
    {
        ARNETWORK_Manager_Stop (manager);
    }

    if ((manager != NULL) && (alManager != NULL) && (manager->onDisconnect != NULL))
    {
        manager->onDisconnect (manager, alManager, manager->customData);
    }
}
