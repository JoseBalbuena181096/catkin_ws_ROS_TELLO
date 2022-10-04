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
/*
 * GENERATED FILE
 *  Do not modify this file, it will be erased during the next configure run 
 */

/**
 * @file ARNETWORK_Error.c
 * @brief ToString function for eARNETWORK_ERROR enum
 */

#include <libARNetwork/ARNETWORK_Error.h>

const char* ARNETWORK_Error_ToString (eARNETWORK_ERROR error)
{
    switch (error)
    {
    case ARNETWORK_OK:
        return "No error";
        break;
    case ARNETWORK_ERROR:
        return "Unknown generic error";
        break;
    case ARNETWORK_ERROR_ALLOC:
        return "Memory allocation error";
        break;
    case ARNETWORK_ERROR_BAD_PARAMETER:
        return "Bad parameters";
        break;
    case ARNETWORK_ERROR_ID_UNKNOWN:
        return "Given IOBuffer identifier is unknown";
        break;
    case ARNETWORK_ERROR_BUFFER_SIZE:
        return "Insufficient free space in the buffer";
        break;
    case ARNETWORK_ERROR_BUFFER_EMPTY:
        return "Buffer is empty, nothing was read";
        break;
    case ARNETWORK_ERROR_SEMAPHORE:
        return "Error when using a semaphore";
        break;
    case ARNETWORK_ERROR_MUTEX:
        return "Error when using a mutex";
        break;
    case ARNETWORK_ERROR_MUTEX_DOUBLE_LOCK:
        return "A mutex is already locked by the same thread";
        break;
    case ARNETWORK_ERROR_MANAGER:
        return "Unknown ARNETWORK_Manager error";
        break;
    case ARNETWORK_ERROR_MANAGER_NEW_IOBUFFER:
        return "IOBuffer creation error";
        break;
    case ARNETWORK_ERROR_MANAGER_NEW_SENDER:
        return "Sender creation error";
        break;
    case ARNETWORK_ERROR_MANAGER_NEW_RECEIVER:
        return "Receiver creation error";
        break;
    case ARNETWORK_ERROR_NEW_BUFFER:
        return "Buffer creation error";
        break;
    case ARNETWORK_ERROR_NEW_RINGBUFFER:
        return "RingBuffer creation error";
        break;
    case ARNETWORK_ERROR_IOBUFFER:
        return "Unknown IOBuffer error";
        break;
    case ARNETWORK_ERROR_IOBUFFER_BAD_ACK:
        return "Bad sequence number for the acknowledge";
        break;
    case ARNETWORK_ERROR_RECEIVER:
        return "Unknown Receiver error";
        break;
    case ARNETWORK_ERROR_RECEIVER_BUFFER_END:
        return "Receiver buffer too small";
        break;
    case ARNETWORK_ERROR_RECEIVER_BAD_FRAME:
        return "Bad frame content on network";
        break;
    default:
        break;
    }
    return "Unknown value";
}
