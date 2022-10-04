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
 * @file ARNETWORK_IOBuffer.c
 * @brief input or output buffer, used by ARNetwork_Receiver or ARNetwork_Sender
 * @date 28/09/2012
 * @author maxime.maitre@parrot.com
 **/

/*****************************************
 *
 *             include file :
 *
 ******************************************/

#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Mutex.h>

#include <libARNetwork/ARNETWORK_Error.h>
#include "ARNETWORK_RingBuffer.h"
#include "ARNETWORK_DataDescriptor.h"
#include <libARNetwork/ARNETWORK_Manager.h>
#include "ARNETWORK_IOBuffer.h"

#include <libARNetworkAL/ARNETWORKAL_Frame.h>

/*****************************************
 *
 *             define :
 *
 ******************************************/

#define ARNETWORK_IOBUFFER_TAG "ARNETWORK_IOBuffer"
#define ARNETWORK_IOBUFFER_MAXSEQVALUE (256)
#define ARNETWORK_IOBUFFER_DELTASEQ (-10)

/**
 * @brief free the data pointed by the data descriptor
 * @param IOBuffer The IOBuffer
 * @param dataDescriptor The data descriptor of the data to free
 * @return error equal to ARNETWORK_OK if the data are correctly deleted otherwise see eARNETWORK_ERROR
 **/
static inline eARNETWORK_ERROR ARNETWORK_IOBuffer_FreeData(ARNETWORK_IOBuffer_t *IOBuffer, ARNETWORK_DataDescriptor_t *dataDescriptor)
{
    /** -- free the last data of the IOBuffer -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;

    if(dataDescriptor->isUsingDataCopy)
    {
        /** if the data has been copied in the dataCopyRBuffer */
        /** pop data copy*/
        error = ARNETWORK_RingBuffer_PopFrontWithSize(IOBuffer->dataCopyRBuffer, NULL, dataDescriptor->dataSize);
    }
    else
    {
        /** if the data is store out of the ARNetwork */
        /** callback with free status */
        if(dataDescriptor->callback != NULL)
        {
            dataDescriptor->callback(IOBuffer->ID, dataDescriptor->data, dataDescriptor->customData, ARNETWORK_MANAGER_CALLBACK_STATUS_FREE);
        }
    }
    
    /** callback with done status */
    /** the date will not more used */
    if(dataDescriptor->callback != NULL)
    {
        dataDescriptor->callback(IOBuffer->ID, NULL, dataDescriptor->customData, ARNETWORK_MANAGER_CALLBACK_STATUS_DONE);
    }
    
    return error;
}

/*****************************************
 *
 *             implementation :
 *
 ******************************************/

ARNETWORK_IOBuffer_t* ARNETWORK_IOBuffer_New(const ARNETWORK_IOBufferParam_t *param, int isInternal)
{
    /** -- Create a new input or output buffer -- */

    /** local declarations */
    ARNETWORK_IOBuffer_t *IOBuffer = NULL;
    eARNETWORK_ERROR error = ARNETWORK_OK;
    int res = 0;

    if (param == NULL)
        return NULL;

    /** Create the input or output buffer in accordance with parameters set in the ARNETWORK_IOBufferParam_t */
    IOBuffer = calloc(1, sizeof(ARNETWORK_IOBuffer_t));
    if (IOBuffer == NULL)
        return NULL;

    /** Initialize to default values */
    IOBuffer->dataDescriptorRBuffer = NULL;
    IOBuffer->dataCopyRBuffer = NULL;
    res = ARSAL_Mutex_Init(&(IOBuffer->mutex));
    if (res != 0) {
        error = ARNETWORK_ERROR_MUTEX;
        goto error;
    }
    res = ARSAL_Sem_Init(&(IOBuffer->outputSem), 0, 0);
    if (res < 0) {
        error = ARNETWORK_ERROR_SEMAPHORE;
        goto error;
    }

    if (!isInternal) {
        res = ARNETWORK_IOBufferParam_Check(param);
        if (res < 0) {
            error = ARNETWORK_ERROR_BAD_PARAMETER;
            goto error;
        }
    }

    IOBuffer->ID = param->ID;
    IOBuffer->dataType = param->dataType;
    IOBuffer->sendingWaitTimeMs = param->sendingWaitTimeMs;
    IOBuffer->ackTimeoutMs = param->ackTimeoutMs;

    if (param->numberOfRetry >= 0) {
        IOBuffer->numberOfRetry = param->numberOfRetry;
    } else {
        /** if numberOfRetry equal 0 disable the retry function with -1 value */
        IOBuffer->numberOfRetry = -1;
    }

    IOBuffer->isWaitAck = 0;
    IOBuffer->seq = 0;
    IOBuffer->alreadyHadData = 0;
    IOBuffer->nbPackets = 0;
    IOBuffer->nbNetwork = 0;
    IOBuffer->waitTimeCount = param->sendingWaitTimeMs;
    IOBuffer->ackWaitTimeCount = param->ackTimeoutMs;
    IOBuffer->retryCount = 0;

    /** Create the RingBuffer for the information of the data*/
    IOBuffer->dataDescriptorRBuffer = ARNETWORK_RingBuffer_NewWithOverwriting(param->numberOfCell, sizeof(ARNETWORK_DataDescriptor_t), param->isOverwriting);
    if (IOBuffer->dataDescriptorRBuffer == NULL) {
        error = ARNETWORK_ERROR_NEW_RINGBUFFER;
        goto error;
    }

    /** if the parameters have a size of data copy */
    if (param->dataCopyMaxSize > 0) {
        /** Create the RingBuffer for the copy of the data*/
        IOBuffer->dataCopyRBuffer = ARNETWORK_RingBuffer_NewWithOverwriting(param->numberOfCell, param->dataCopyMaxSize, param->isOverwriting);
        if (IOBuffer->dataCopyRBuffer == NULL) {
            error = ARNETWORK_ERROR_NEW_BUFFER;
            goto error;
        }
    }

    return IOBuffer;

error:
    /** delete the inOutput buffer if an error occurred */
    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARNETWORK_IOBUFFER_TAG,"error: %s", ARNETWORK_Error_ToString (error));
    ARNETWORK_IOBuffer_Delete(&IOBuffer);
    return NULL;
}

void ARNETWORK_IOBuffer_Delete(ARNETWORK_IOBuffer_t **IOBuffer)
{
    /** -- Delete the input or output buffer -- */

    /** local declarations */

    if(IOBuffer != NULL)
    {
        if((*IOBuffer) != NULL)
        {
            ARSAL_Mutex_Destroy(&((*IOBuffer)->mutex));
            ARSAL_Sem_Destroy(&((*IOBuffer)->outputSem));

            ARNETWORK_IOBuffer_CancelAllData((*IOBuffer));

            ARNETWORK_RingBuffer_Delete(&((*IOBuffer)->dataDescriptorRBuffer));
            ARNETWORK_RingBuffer_Delete(&((*IOBuffer)->dataCopyRBuffer));

            free((*IOBuffer));
            (*IOBuffer) = NULL;
        }
    }
}

eARNETWORK_ERROR ARNETWORK_IOBuffer_AckReceived(ARNETWORK_IOBuffer_t *IOBuffer, uint8_t seqNumber)
{
    /** -- Receive an acknowledgement to a IOBuffer -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;

    /** delete the data if the sequence number received is same as the sequence number expected */
    if(IOBuffer->isWaitAck && IOBuffer->seq == seqNumber)
    {
        IOBuffer->isWaitAck = 0;
        error = ARNETWORK_IOBuffer_PopDataWithCallBack(IOBuffer, ARNETWORK_MANAGER_CALLBACK_STATUS_ACK_RECEIVED);
    }
    else
    {
        error = ARNETWORK_ERROR_IOBUFFER_BAD_ACK;
    }

    return error;
}

int ARNETWORK_IOBuffer_IsWaitAck(ARNETWORK_IOBuffer_t *IOBuffer)
{
    /** -- Get if the IOBuffer is waiting an acknowledgement -- */

    /** local declarations */
    int isWaitAckCpy = 0;

    isWaitAckCpy = IOBuffer->isWaitAck;

    return isWaitAckCpy;
}

eARNETWORK_ERROR ARNETWORK_IOBuffer_Lock( ARNETWORK_IOBuffer_t *IOBuffer)
{
    /** -- Lock the IOBuffer's mutex -- **/

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    int lockingReturn = 0;

    /** lock the IOBuffer */
    lockingReturn = ARSAL_Mutex_Lock(&(IOBuffer->mutex));

    if(lockingReturn != 0)
    {
        switch(lockingReturn)
        {
        case EDEADLK:
            error = ARNETWORK_ERROR_MUTEX_DOUBLE_LOCK;
            break;

        default:
            error = ARNETWORK_ERROR_MUTEX;
            ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_IOBUFFER_TAG, "locking return : %d unexpected", lockingReturn);
            break;
        }
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_IOBuffer_Unlock( ARNETWORK_IOBuffer_t *IOBuffer)
{
    /** -- Unlock the IOBuffer's mutex -- **/

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    int unlockingReturn = 0;

    /** unlock the IOBuffer if there is not correctly unlocked */
    unlockingReturn = ARSAL_Mutex_Unlock(&(IOBuffer->mutex));

    if(unlockingReturn != 0)
    {
        switch(unlockingReturn)
        {
        case EDEADLK:
            error = ARNETWORK_ERROR_MUTEX_DOUBLE_LOCK;
            break;

        default:
            error = ARNETWORK_ERROR_MUTEX;
            ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_IOBUFFER_TAG, "unlocking return : %d unexpected", unlockingReturn);
            break;
        }
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_IOBuffer_CancelAllData(ARNETWORK_IOBuffer_t *IOBuffer)
{
    /** -- cancel all remaining data -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    eARNETWORK_ERROR deleteError = ARNETWORK_OK;

    /** pop all data with the ARNETWORK_MANAGER_CALLBACK_STATUS_CANCEL status */
    while(deleteError == ARNETWORK_OK)
    {
        deleteError = ARNETWORK_IOBuffer_PopDataWithCallBack(IOBuffer, ARNETWORK_MANAGER_CALLBACK_STATUS_CANCEL);
    }

    if(deleteError != ARNETWORK_ERROR_BUFFER_EMPTY)
    {
        error = deleteError;
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_IOBuffer_PopData(ARNETWORK_IOBuffer_t *IOBuffer)
{
    /** -- Pop the later data of the IOBuffer and free it -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    ARNETWORK_DataDescriptor_t dataDescriptor;

    /** pop and get the data descriptor */
    error = ARNETWORK_RingBuffer_PopFront(IOBuffer->dataDescriptorRBuffer, (uint8_t*) &dataDescriptor);
    if(error == ARNETWORK_OK)
    {
        /** free data */
        error = ARNETWORK_IOBuffer_FreeData(IOBuffer, &dataDescriptor);
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_IOBuffer_PopDataWithCallBack(ARNETWORK_IOBuffer_t *IOBuffer, eARNETWORK_MANAGER_CALLBACK_STATUS callbackStatus)
{
    /** -- Pop the later data of the IOBuffer with callback calling and free it -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    ARNETWORK_DataDescriptor_t dataDescriptor;

    /** pop and get the data descriptor */
    error = ARNETWORK_RingBuffer_PopFront(IOBuffer->dataDescriptorRBuffer, (uint8_t*) &dataDescriptor);
    if(error == ARNETWORK_OK)
    {
        /** callback with the reason of the data popping */
        if(dataDescriptor.callback != NULL)
        {
            dataDescriptor.callback(IOBuffer->ID, dataDescriptor.data, dataDescriptor.customData, callbackStatus);
        }

        /** free data */
        error = ARNETWORK_IOBuffer_FreeData(IOBuffer, &dataDescriptor);
    }

    return error;
}

eARNETWORK_ERROR ARNETWORK_IOBuffer_Flush(ARNETWORK_IOBuffer_t *IOBuffer)
{
    /** -- Flush the IoBuffer -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;

    /**  delete all data */
    while(error == ARNETWORK_OK)
    {
        error = ARNETWORK_IOBuffer_PopDataWithCallBack(IOBuffer, ARNETWORK_MANAGER_CALLBACK_STATUS_CANCEL);
    }

    /** if the error occurred is "buffer empty" there is no error */
    if(error == ARNETWORK_ERROR_BUFFER_EMPTY)
    {
        error = ARNETWORK_OK;
    }

    /** state reset */
    IOBuffer->isWaitAck = 0;
    IOBuffer->alreadyHadData = 0;
    IOBuffer->waitTimeCount = IOBuffer->sendingWaitTimeMs;
    IOBuffer->ackWaitTimeCount = IOBuffer->ackTimeoutMs;
    IOBuffer->retryCount = 0;

    /** reset semaphore */
    ARSAL_Sem_Destroy(&(IOBuffer->outputSem));
    ARSAL_Sem_Init(&(IOBuffer->outputSem), 0, 0);

    return error;
}

eARNETWORK_ERROR ARNETWORK_IOBuffer_AddData(ARNETWORK_IOBuffer_t *IOBuffer, uint8_t *data, size_t dataSize, void *customData, ARNETWORK_Manager_Callback_t callback, int doDataCopy)
{
    /** -- Add data in a IOBuffer -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    ARNETWORK_DataDescriptor_t dataDescriptor;
    int numberOfFreeCell = 0;

    /** initialize dataDescriptor */
    dataDescriptor.data = data;
    dataDescriptor.dataSize = dataSize;
    dataDescriptor.customData = customData;
    dataDescriptor.callback = callback;
    dataDescriptor.isUsingDataCopy = 0;

    /** get the number of free cell */
    numberOfFreeCell = ARNETWORK_RingBuffer_GetFreeCellNumber(IOBuffer->dataDescriptorRBuffer);

    /** if the buffer is not full or it is overwriting */
    if( (IOBuffer->dataDescriptorRBuffer->isOverwriting == 1) || (numberOfFreeCell > 0) )
    {
        /** if there is overwriting */
        if(numberOfFreeCell == 0)
        {
            /** if the buffer is full, cancel the data lost by the overwriting */
            /** Delete the data Overwritten */
            error = ARNETWORK_IOBuffer_PopDataWithCallBack(IOBuffer, ARNETWORK_MANAGER_CALLBACK_STATUS_CANCEL);
        }

        /** if data copy is asked */
        if( (error == ARNETWORK_OK) && (doDataCopy) )
        {
            /** check if the IOBuffer can copy and if the size of the copy buffer is large enough */
            if( (ARNETWORK_IOBuffer_CanCopyData(IOBuffer)) && (dataSize <= IOBuffer->dataCopyRBuffer->cellSize) )
            {
                /** copy data in the dataCopyRBuffer and get the address of the data copy in descData */
                error =  ARNETWORK_RingBuffer_PushBackWithSize(IOBuffer->dataCopyRBuffer, data, dataSize, &(dataDescriptor.data));

                /** set the flag to indicate the copy of the data */
                dataDescriptor.isUsingDataCopy = 1;
            }
            else
            {
                error = ARNETWORK_ERROR_BAD_PARAMETER;
            }
        }

        if(error == ARNETWORK_OK)
        {
            /** push dataDescriptor in the IOBuffer */
            error = ARNETWORK_RingBuffer_PushBack(IOBuffer->dataDescriptorRBuffer, (uint8_t*) &dataDescriptor);
            IOBuffer->alreadyHadData = 1;
        }
    }
    else
    {
        error = ARNETWORK_ERROR_BUFFER_SIZE;
    }

    return error;
}

int ARNETWORK_IOBuffer_ShouldAcceptData (ARNETWORK_IOBuffer_t *IOBuffer, uint8_t seqnum)
{
    int retVal = -1;
    int maxDelta = ARNETWORK_IOBUFFER_DELTASEQ;
    if (IOBuffer == NULL)
    {
        return retVal;
    }

    if (IOBuffer->alreadyHadData == 0)
    {
        return 1; // Accept any data, regardless of its sequence number
    }

    retVal = seqnum - IOBuffer->seq;
    if (retVal < maxDelta)// If the packet is more than 10 seq old, it might be a loopback
    {
        retVal += ARNETWORK_IOBUFFER_MAXSEQVALUE;
    }
    // All other cases should keep their value.
    return retVal;
}

eARNETWORK_ERROR ARNETWORK_IOBuffer_ReadData(ARNETWORK_IOBuffer_t *IOBuffer, uint8_t *data, size_t dataLimitSize, int *readSize)
{
    /** -- read data received in a IOBuffer -- */

    /** local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    ARNETWORK_DataDescriptor_t dataDescriptor;
    int localReadSize = 0;

    /** get data descriptor*/
    error = ARNETWORK_RingBuffer_Front(IOBuffer->dataDescriptorRBuffer, (uint8_t*) &dataDescriptor);

    if( error == ARNETWORK_OK )
    {
        /** data size check */
        if(dataDescriptor.dataSize <= dataLimitSize)
        {
            /** data copy */
            memcpy(data, dataDescriptor.data, dataDescriptor.dataSize);

            /** set size of data read */
            localReadSize = dataDescriptor.dataSize;

            /** pop the data */
            ARNETWORK_IOBuffer_PopData(IOBuffer);
        }
        else
        {
            error = ARNETWORK_ERROR_BUFFER_SIZE;
        }
    }

    /** return the size of the data read */
    if(readSize != NULL)
    {
        *readSize = localReadSize;
    }

    return error;
}

int ARNETWORK_IOBuffer_GetEstimatedMissPercentage (ARNETWORK_IOBuffer_t *IOBuffer)
{
    if (IOBuffer == NULL)
    {
        return ARNETWORK_ERROR_BAD_PARAMETER;
    }
    /* Get the number of missed packets */
    int nbMissed = IOBuffer->nbNetwork - IOBuffer->nbPackets;
    /* Convert it to percentage missed */
    if (IOBuffer->nbNetwork == 0) { return 0; } // Avoid divide by zero
    return (100 * nbMissed) / IOBuffer->nbNetwork;
}
