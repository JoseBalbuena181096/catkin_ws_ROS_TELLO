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
#ifndef _ARDISCOVERY_CONNECTION_PRIVATE_H_
#define _ARDISCOVERY_CONNECTION_PRIVATE_H_

#include <libARDiscovery/ARDISCOVERY_Error.h>
#include <libARDiscovery/ARDISCOVERY_Connection.h>

/**
 * @brief Low level communication related structure
 */
typedef struct ARDISCOVERY_Connection_ComData_t
{
    uint8_t *buffer; /**< data buffer */
    uint32_t size; /**< size of the data */
    uint32_t capacity; /**< size allocated of the data buffer */
} ARDISCOVERY_Connection_ComData_t;

/**
 * @brief Global negotiation related structure (declared in public header)
 */
struct ARDISCOVERY_Connection_ConnectionData_t
{
    ARDISCOVERY_Connection_ComData_t txData;        /**< Tx negociation node */
    ARDISCOVERY_Connection_ComData_t rxData;        /**< Rx negociation node */
    uint8_t isAlive;                                   /**< is alive flag */
    ARSAL_Sem_t runningSem;                             /**< running Semaphore */
    ARDISCOVERY_Connection_SendJsonCallback_t sendJsoncallback; /**< callback use to send json information of the connection */
    ARDISCOVERY_Connection_ReceiveJsonCallback_t receiveJsoncallback; /**< callback use to receive json information of the connection */
    void *customData;                           /**< Custom data for callback use */
    int32_t socket;                             /**< socket used to negociate */
    struct sockaddr_in address;                 /**< address used to negociate */
    
    int abortPipe[2];
};

#endif /* _ARDISCOVERY_CONNECTION_PRIVATE_H_ */
