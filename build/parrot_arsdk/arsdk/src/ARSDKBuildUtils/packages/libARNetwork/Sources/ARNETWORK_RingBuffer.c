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
 * @file ARNETWORK_RingBuffer.c
 * @brief Ring buffer, multithread safe with overwriting possibility.
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
#include <inttypes.h>

#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Mutex.h>

#include <libARNetwork/ARNETWORK_Error.h>

#include "ARNETWORK_RingBuffer.h"

/*****************************************
 *
 *             define :
 *
 ******************************************/

#define ARNETWORK_RINGBUFFER_TAG "ARNETWORK_RingBuffer"

/*****************************************
 *
 *             internal functions :
 *
 ******************************************/

/* Normalize indices if they are both larger than the entire buffer size
 * so that they do not overflow. */
static inline void ARNETWORK_RingBuffer_NormalizeIndexes(ARNETWORK_RingBuffer_t *ringBuffer)
{
    size_t buffer_size = ringBuffer->cellSize * ringBuffer->numberOfCell;
    if (ringBuffer->indexInput >= buffer_size && ringBuffer->indexOutput >= buffer_size)
    {
        ringBuffer->indexInput %= buffer_size;
        ringBuffer->indexOutput %= buffer_size;
    }
    /* No else: the Indexes are already normalized. */
}

/**
 * @brief Return the number of free cell of the ring buffer
 * @param ringBuffer the ring buffer which will give the number of its free cells
 * @return number of free cell of the ring buffer 
**/
static inline int ARNETWORK_RingBuffer_GetFreeCellNumberUnlocked(const ARNETWORK_RingBuffer_t *ringBuffer)
{
    return ringBuffer->numberOfCell - ( (ringBuffer->indexInput - ringBuffer->indexOutput) / ringBuffer->cellSize );
}

/**
 * @brief Check if the ring buffer is empty
 * @param ringBuffer the ring buffer which will check if it is empty
 * @return equal to 1 if the ring buffer is empty else 0
**/
static inline int ARNETWORK_RingBuffer_IsEmptyUnlocked(const ARNETWORK_RingBuffer_t *ringBuffer)
{
    return (ringBuffer->indexInput == ringBuffer->indexOutput) ? 1 : 0;
}

/*****************************************
 *
 *             implementation :
 *
 ******************************************/

ARNETWORK_RingBuffer_t* ARNETWORK_RingBuffer_New(unsigned int numberOfCell, unsigned int cellSize)
{
    /** -- Create a new ring buffer not overwriarray -- */
    return ARNETWORK_RingBuffer_NewWithOverwriting( numberOfCell, cellSize, 0 );
}

ARNETWORK_RingBuffer_t* ARNETWORK_RingBuffer_NewWithOverwriting(unsigned int numberOfCell, unsigned int cellSize, int isOverwriting)
{
    /* -- Create a new ring buffer -- */

    /* local declarations */
    int err = 0;
    ARNETWORK_RingBuffer_t* ringBuffer = calloc(1, sizeof(ARNETWORK_RingBuffer_t));
    if (ringBuffer == NULL)
        return NULL;

    ringBuffer->numberOfCell = numberOfCell;
    ringBuffer->cellSize = cellSize;
    ringBuffer->indexInput = 0;
    ringBuffer->indexOutput = 0;
    ringBuffer->isOverwriting = isOverwriting;
    err = ARSAL_Mutex_Init(&ringBuffer->mutex);
    if (err != 0)
        goto error;

    ringBuffer->dataBuffer = malloc(cellSize * numberOfCell);
    if (ringBuffer->dataBuffer == NULL)
        goto error;

    return ringBuffer;

error:
    ARNETWORK_RingBuffer_Delete(&ringBuffer);
    return NULL;
}

void ARNETWORK_RingBuffer_Delete(ARNETWORK_RingBuffer_t **ringBuffer)
{
    /* -- Delete the ring buffer -- */

    if (ringBuffer != NULL)
    {
        if((*ringBuffer) != NULL)
        {
            ARSAL_Mutex_Destroy(&((*ringBuffer)->mutex));
            free((*ringBuffer)->dataBuffer);
            (*ringBuffer)->dataBuffer = NULL;

            free(*ringBuffer);
            (*ringBuffer) = NULL;
        }
        /* No else: No ringBuffer to delete */
    }
    /* No else: Parameters check (stops the processing) */
}

eARNETWORK_ERROR ARNETWORK_RingBuffer_PushBack(ARNETWORK_RingBuffer_t *ringBuffer, const uint8_t *newData) //inline ?
{
    /* -- Add the new data at the back of the ring buffer -- */

    return ARNETWORK_RingBuffer_PushBackWithSize(ringBuffer, newData, ringBuffer->cellSize, NULL);
}

eARNETWORK_ERROR ARNETWORK_RingBuffer_PushBackWithSize(ARNETWORK_RingBuffer_t *ringBuffer, const uint8_t *newData, int dataSize, uint8_t **dataCopy)
{
    /* -- Add the new data at the back of the ring buffer with specification of the data size -- */

    /* local declarations */
    int error = ARNETWORK_OK;
    uint8_t* buffer = NULL;

    ARSAL_Mutex_Lock(&(ringBuffer->mutex));

    /* check if the has enough free cell or the buffer is overwriting */
    if ((ARNETWORK_RingBuffer_GetFreeCellNumberUnlocked(ringBuffer)) || (ringBuffer->isOverwriting))
    {
        if (!ARNETWORK_RingBuffer_GetFreeCellNumberUnlocked(ringBuffer))
        {
            (ringBuffer->indexOutput) += ringBuffer->cellSize;
        }
        /* No else: the ringBuffer is not full */

        buffer = ringBuffer->dataBuffer + ( ringBuffer->indexInput % (ringBuffer->numberOfCell * ringBuffer->cellSize) );

        memcpy(buffer, newData, dataSize);

        /* return the pointer on the data copy in the ring buffer */
        if(dataCopy != NULL)
        {
            *dataCopy = buffer;
        }
        /* No else: data are not returned */

        ringBuffer->indexInput += ringBuffer->cellSize;
        ARNETWORK_RingBuffer_NormalizeIndexes(ringBuffer);
    }
    else
    {
        error = ARNETWORK_ERROR_BUFFER_SIZE;
    }

    ARSAL_Mutex_Unlock(&(ringBuffer->mutex));

    return error;
}

eARNETWORK_ERROR ARNETWORK_RingBuffer_PopFront(ARNETWORK_RingBuffer_t *ringBuffer, uint8_t *dataPop) //see inline
{
    /* -- Pop the oldest data -- */

    return ARNETWORK_RingBuffer_PopFrontWithSize(ringBuffer, dataPop, ringBuffer->cellSize);
}

eARNETWORK_ERROR ARNETWORK_RingBuffer_PopFrontWithSize(ARNETWORK_RingBuffer_t *ringBuffer, uint8_t *dataPop, int dataSize)
{
    /* -- Pop the oldest data -- */

    /* local declarations */
    uint8_t *buffer = NULL;
    eARNETWORK_ERROR error = ARNETWORK_OK;

    ARSAL_Mutex_Lock(&(ringBuffer->mutex));

    if (!ARNETWORK_RingBuffer_IsEmptyUnlocked(ringBuffer))
    {
        if(dataPop != NULL)
        {
            /* get the address of the front data */
            buffer = ringBuffer->dataBuffer + (ringBuffer->indexOutput % (ringBuffer->numberOfCell * ringBuffer->cellSize));
            memcpy(dataPop, buffer, dataSize);
        }
        /* No else: the data popped is not returned  */
        (ringBuffer->indexOutput) += ringBuffer->cellSize;
        ARNETWORK_RingBuffer_NormalizeIndexes(ringBuffer);
    }
    else
    {
        error = ARNETWORK_ERROR_BUFFER_EMPTY;
    }

    ARSAL_Mutex_Unlock(&(ringBuffer->mutex));

    return error;
}

eARNETWORK_ERROR ARNETWORK_RingBuffer_Front(ARNETWORK_RingBuffer_t *ringBuffer, uint8_t *frontData)
{
    /* -- Return a pointer on the front data -- */

    /* local declarations */
    eARNETWORK_ERROR error = ARNETWORK_OK;
    uint8_t *buffer = NULL;

    ARSAL_Mutex_Lock(&(ringBuffer->mutex));

    /* get the address of the front data */
    buffer = ringBuffer->dataBuffer + (ringBuffer->indexOutput % (ringBuffer->numberOfCell * ringBuffer->cellSize));

    if( !ARNETWORK_RingBuffer_IsEmptyUnlocked(ringBuffer) )
    {
        memcpy(frontData, buffer, ringBuffer->cellSize);
    }
    else
    {
        error = ARNETWORK_ERROR_BUFFER_EMPTY;
    }

    ARSAL_Mutex_Unlock(&(ringBuffer->mutex));

    return error;
}

void ARNETWORK_RingBuffer_Print(ARNETWORK_RingBuffer_t *ringBuffer)
{
    /* -- Print the state of the ring buffer -- */

    ARSAL_Mutex_Lock(&(ringBuffer->mutex));

    ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_RINGBUFFER_TAG," pointer dataBuffer :%p \n",ringBuffer->dataBuffer);
    ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_RINGBUFFER_TAG," numberOfCell :%d \n",ringBuffer->numberOfCell);
    ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_RINGBUFFER_TAG," cellSize :%d \n",ringBuffer->cellSize);
    ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_RINGBUFFER_TAG," indexOutput :%d \n",ringBuffer->indexOutput);
    ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_RINGBUFFER_TAG," indexInput :%d \n",ringBuffer->indexInput);
    ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_RINGBUFFER_TAG," overwriting :%d \n",ringBuffer->isOverwriting);
    ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_RINGBUFFER_TAG," data : \n");

    ARSAL_Mutex_Unlock(&(ringBuffer->mutex));

    ARNETWORK_RingBuffer_DataPrint(ringBuffer);
}

void ARNETWORK_RingBuffer_DataPrint(ARNETWORK_RingBuffer_t *ringBuffer)
{
    /* -- Print the contents of the ring buffer -- */

    /* local declarations */
    uint8_t *byteIterator = NULL;
    unsigned int cellIndex = 0;
    unsigned int byteIndex = 0;

    ARSAL_Mutex_Lock(&(ringBuffer->mutex));

    /* for all cell of the ringBuffer */
    for (cellIndex = ringBuffer->indexOutput ; cellIndex < ringBuffer->indexInput ; cellIndex += ringBuffer->cellSize )
    {
        byteIterator = ringBuffer->dataBuffer + (cellIndex % (ringBuffer->numberOfCell * ringBuffer->cellSize) );

        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_RINGBUFFER_TAG,"    - 0x: ");

        /* for all byte of the cell */
        for(byteIndex = 0 ; byteIndex < ringBuffer->cellSize ; ++byteIndex)
        {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_RINGBUFFER_TAG,"%2x | ",*((uint8_t*)byteIterator));
            ++byteIterator;
        }
        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARNETWORK_RINGBUFFER_TAG,"\n");
    }

    ARSAL_Mutex_Unlock(&(ringBuffer->mutex));
}

int ARNETWORK_RingBuffer_GetFreeCellNumber(ARNETWORK_RingBuffer_t *ringBuffer)
{
    int numberOfFreeCell = -1;
    
    ARSAL_Mutex_Lock(&(ringBuffer->mutex));
    
    numberOfFreeCell = ringBuffer->numberOfCell - ( (ringBuffer->indexInput - ringBuffer->indexOutput) / ringBuffer->cellSize );
    
    ARSAL_Mutex_Unlock(&(ringBuffer->mutex));
    
    return numberOfFreeCell;
}


int ARNETWORK_RingBuffer_IsEmpty(ARNETWORK_RingBuffer_t *ringBuffer)
{
    int isEmpty = 0;
    
    ARSAL_Mutex_Lock(&(ringBuffer->mutex));
    
    isEmpty = (ringBuffer->indexInput == ringBuffer->indexOutput) ? 1 : 0;
    
    ARSAL_Mutex_Unlock(&(ringBuffer->mutex));
    
    return isEmpty;
}
