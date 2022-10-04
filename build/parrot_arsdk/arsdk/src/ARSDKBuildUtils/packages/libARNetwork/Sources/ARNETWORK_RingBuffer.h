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
 * @file ARNETWORK_RingBuffer.h
 * @brief Ring buffer, multithread safe with overwriting possibility.
 * @date 05/18/2012
 * @author maxime.maitre@parrot.com
**/

#ifndef _ARNETWORK_RINGBUFFER_PRIVATE_H_
#define _ARNETWORK_RINGBUFFER_PRIVATE_H_

#include <libARSAL/ARSAL_Mutex.h>
#include <inttypes.h>

/**
 * @brief Basic ring buffer, multithread safe
 * @warning before to be used the ring buffer must be created through ARNETWORK_RingBuffer_New() or ARNETWORK_RingBuffer_NewWithOverwriting()
 * @post after its using the ring buffer must be deleted through ARNETWORK_RingBuffer_Delete()
**/
typedef struct  
{
    uint8_t *dataBuffer; /**< the data buffer*/
    unsigned int numberOfCell; /**< Maximum number of data stored*/
    unsigned int cellSize; /**< Size of one data in byte*/
    unsigned int isOverwriting; /**< Indicator of overwriting possibility (1 = true | 0 = false)*/
    
    unsigned int indexInput; /**< Index of the data input*/
    unsigned int indexOutput; /**< Index of the data output*/
    
    ARSAL_Mutex_t mutex; /**< Mutex to take before to use the ringBuffer*/

}ARNETWORK_RingBuffer_t;

/**
 * @brief Create a new ring buffer
 * @warning This function allocate memory
 * @post ARNETWORK_RingBuffer_Delete() must be called to delete the ring buffer and free the memory allocated
 * @param[in] numberOfCell Maximum number of data cell of the ring buffer
 * @param[in] cellSize size of one data cell of the ring buffer
 * @return Pointer on the new ring buffer
 * @see ARNETWORK_RingBuffer_NewWithOverwriting()
 * @see ARNETWORK_RingBuffer_Delete()
**/
ARNETWORK_RingBuffer_t* ARNETWORK_RingBuffer_New(unsigned int numberOfCell, unsigned int cellSize); 

/**
 * @brief Create a new ring buffer.
 * @warning This function allocate memory
 * @post ARNETWORK_RingBuffer_Delete() must be called to delete the ring buffer and free the memory allocated
 * @param[in] numberOfCell Maximum number of data cell of the ring buffer
 * @param[in] cellSize size of one data cell of the ring buffer
 * @param[in] isOverwriting set to 1 allow the overwriting if the buffer is full otherwise set 0
 * @return Pointer on the new ring buffer
 * @see ARNETWORK_RingBuffer_NewWithOverwriting()
 * @see ARNETWORK_RingBuffer_Delete()
**/
ARNETWORK_RingBuffer_t* ARNETWORK_RingBuffer_NewWithOverwriting(unsigned int numberOfCell, unsigned int cellSize, int isOverwriting); 

/**
 * @brief Delete the ring buffer
 * @warning This function free memory
 * @param ringBuffer Pointer to the ring buffer to delete
 * @see ARNETWORK_RingBuffer_New()
 * @see ARNETWORK_RingBuffer_NewWithOverwriting()
**/
void ARNETWORK_RingBuffer_Delete(ARNETWORK_RingBuffer_t **ringBuffer);

/**
 * @brief Add the new data at the back of the ring buffer
 * @warning newData must be different of NULL
 * @param ringBuffer The ring buffer which will push back
 * @param[in] newData pointer on the data to add
 * @return error eARNETWORK_ERROR
**/
eARNETWORK_ERROR ARNETWORK_RingBuffer_PushBack(ARNETWORK_RingBuffer_t *ringBuffer, const uint8_t *newData);

/**
 * @brief Add the new data at the back of the ring buffer
 * @warning newData must be different of NULL
 * @warning data size must not be more than ring buffer's cell size
 * @note if data size is less than ring buffer's cell size, the bytes at the cell end are not set. 
 * @param ringBuffer the ring buffer which will push back
 * @param[in] newData the data to add
 * @param[in] dataSize size in byte of the data to copy from newData
 * @param[out] dataCopy address to return the pointer on the data copy in the ring buffer ; can be equal to NULL 
 * @return error eARNETWORK_ERROR
**/
eARNETWORK_ERROR ARNETWORK_RingBuffer_PushBackWithSize(ARNETWORK_RingBuffer_t *ringBuffer, const uint8_t *newData, int dataSize, uint8_t **dataCopy);

/**
 * @brief Pop the oldest data
 * @param ringBuffer the ring buffer which will pop front
 * @param[out] dataPop pointer on the data popped
 * @return error eARNETWORK_ERROR
**/
eARNETWORK_ERROR ARNETWORK_RingBuffer_PopFront(ARNETWORK_RingBuffer_t *ringBuffer, uint8_t *dataPop);

/**
 * @brief Pop the oldest data
 * @warning dataSize must be less or equal of the ring buffer's cell size.
 * @note if data size is less than ring buffer's cell size, the bytes at the cell end are lost.
 * @param ringBuffer the ring buffer which will pop front
 * @param[out] dataPop the data popped
 * @param[in] dataSize size to copy from the front data to the dataPop
 * @return error eARNETWORK_ERROR
**/
eARNETWORK_ERROR ARNETWORK_RingBuffer_PopFrontWithSize(ARNETWORK_RingBuffer_t *ringBuffer, uint8_t *dataPop, int dataSize);

/**
 * @brief Return the number of free cell of the ring buffer
 * @param ringBuffer the ring buffer which will give the number of its free cells
 * @return number of free cell of the ring buffer 
**/
int ARNETWORK_RingBuffer_GetFreeCellNumber(ARNETWORK_RingBuffer_t *ringBuffer);

/**
 * @brief Check if the ring buffer is empty
 * @param ringBuffer the ring buffer which will check if it is empty
 * @return equal to 1 if the ring buffer is empty else 0
**/
int ARNETWORK_RingBuffer_IsEmpty(ARNETWORK_RingBuffer_t *ringBuffer);

/**
 * @brief Return a pointer on the front data
 * @param ringBuffer the ring buffer which will give its front data
 * @param[out] frontData the front data
 * @return error eARNETWORK_ERROR
**/
eARNETWORK_ERROR ARNETWORK_RingBuffer_Front(ARNETWORK_RingBuffer_t *ringBuffer, uint8_t *frontData);

/**
 * @brief Clean the ring buffer
 * @param ringBuffer the ring buffer to clean
**/
static inline void ARNETWORK_RingBuffer_Clean(ARNETWORK_RingBuffer_t *ringBuffer)
{
    ringBuffer->indexInput = ringBuffer->indexOutput;
}

/**
 * @brief Print the state of the ring buffer
 * @param ringBuffer the ring buffer to print
**/
void ARNETWORK_RingBuffer_Print(ARNETWORK_RingBuffer_t *ringBuffer);

/**
 * @brief Print the contents of the ring buffer
 * @param ringBuffer the ring buffer which will print its data
**/
void ARNETWORK_RingBuffer_DataPrint(ARNETWORK_RingBuffer_t *ringBuffer);

#endif /** _ARNETWORK_RINGBUFFER_PRIVATE_H_ */

