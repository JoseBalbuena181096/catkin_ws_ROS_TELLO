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
 * @file ARNETWORK_Manager.h
 * @brief network manager allow to send data acknowledged or not.
 * @date 05/18/2012
 * @author maxime.maitre@parrot.com
 */

#ifndef _NETWORK_MANAGER_PRIVATE_H_
#define _NETWORK_MANAGER_PRIVATE_H_

#include "ARNETWORK_IOBuffer.h"
#include "ARNETWORK_Sender.h"
#include "ARNETWORK_Receiver.h"
#include <libARNetwork/ARNETWORK_Manager.h>
#include <libARNetworkAL/ARNETWORKAL_Manager.h>

typedef enum {
    ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PING = 0, /**< Ping buffer id - ping requests */
    ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PONG, /**< Pong buffer id - ping reply */
    ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_MAX, /**< Should always be kept less or equal to 10 */
} eARNETWORK_MANAGER_INTERNAL_BUFFER_ID;

/**
 * @brief get the identifier of the output buffer storing the acknowledgment for an output buffer storing data acknowledged.
 * @param[in] ID identifier of the output buffer waiting an acknowledgment.
 * @return identifier of the output buffer storing the acknowledgment.
 */
static inline int ARNETWORK_Manager_IDOutputToIDAck (ARNETWORKAL_Manager_t *alManager, int identifier)
{
    return identifier + (alManager->maxIds / 2);
}

/**
 * @brief get the identifier of the output buffer storing data acknowledged for an output buffer storing acknowledgments.
 * @param[in] ID identifier of the output buffer storing the acknowledgment.
 * @return identifier of the output buffer waiting an acknowledgment.
 */
static inline int ARNETWORK_Manager_IDAckToIDInput (ARNETWORKAL_Manager_t *alManager, int identifier)
{
    return identifier - (alManager->maxIds / 2);
}

/**
 * @brief network manager allow to send data acknowledged or not.
 */
struct ARNETWORK_Manager_t
{
    ARNETWORKAL_Manager_t *networkALManager; /**< Pointer on the OS specific manager */
    ARNETWORK_Sender_t *sender; /**< The sender */
    ARNETWORK_Receiver_t *receiver; /**< The receiver */
    ARNETWORK_IOBuffer_t **inputBufferArray; /**< The array storing the input buffer */
    ARNETWORK_IOBuffer_t **outputBufferArray; /**< The array storing the output buffer */
    ARNETWORK_IOBuffer_t **internalInputBufferArray; /**< The array storing the internal input buffers */
    int numberOfInput; /**< Number of input buffer */
    int numberOfOutput; /**< Number of output buffer */
    int numberOfInputWithoutAck; /**< Number of input buffer without the  buffers of acknowledgement */
    int numberOfOutputWithoutAck; /**< Number of output buffer without the  buffers of acknowledgement */
    int numberOfInternalInputs; /**< Number of internal input buffers */
    ARNETWORK_IOBuffer_t **inputBufferMap; /**< array storing the inputBuffers by their identifier */
    ARNETWORK_IOBuffer_t **outputBufferMap; /**< array storing the outputBuffers by their identifier */
    ARNETWORK_Manager_OnDisconnect_t onDisconnect; /**< Manager specific on disconnect function */
    void *customData; /**< custom data sent to the callbacks */
};

#endif /** _NETWORK_MANAGER_PRIVATE_H_ */
