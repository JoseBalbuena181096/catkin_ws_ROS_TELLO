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
 * @file ARSTREAM_Buffers.h
 * @brief Infos of libARNetwork buffers
 * @date 03/22/2013
 * @author nicolas.brulez@parrot.com
 */

#ifndef _ARSTREAM_BUFFERS_PRIVATE_H_
#define _ARSTREAM_BUFFERS_PRIVATE_H_

/*
 * System Headers
 */

/*
 * Private Headers
 */
#include "ARSTREAM_NetworkHeaders.h"

/*
 * ARSDK Headers
 */
#include <libARNetwork/ARNETWORK_IOBufferParam.h>

/*
 * Macros
 */
#define ARSTREAM_BUFFERS_DATA_BUFFER_TYPE            (ARNETWORKAL_FRAME_TYPE_DATA_LOW_LATENCY)
#define ARSTREAM_BUFFERS_DATA_BUFFER_SEND_EVERY_MS   (0) // Zero means "send every time we can"
#define ARSTREAM_BUFFERS_DATA_BUFFER_NUMBER_OF_CELLS (128)
#define ARSTREAM_BUFFERS_DATA_BUFFER_COPY_MAX_SIZE   (ARNETWORK_IOBUFFERPARAM_DATACOPYMAXSIZE_USE_MAX)
#define ARSTREAM_BUFFERS_DATA_BUFFER_OVERWRITE       (1)

#define ARSTREAM_BUFFERS_ACK_BUFFER_TYPE             (ARNETWORKAL_FRAME_TYPE_DATA_LOW_LATENCY)
#define ARSTREAM_BUFFERS_ACK_BUFFER_SEND_EVERY_MS    (0) // Zero means "send every time we can"
#define ARSTREAM_BUFFERS_ACK_BUFFER_NUMBER_OF_CELLS  (1000) // TODO: Change to 1 when mantis 115578 will be fixed
#define ARSTREAM_BUFFERS_ACK_BUFFER_COPY_MAX_SIZE    (sizeof (ARSTREAM_NetworkHeaders_AckPacket_t))
#define ARSTREAM_BUFFERS_ACK_BUFFER_OVERWRITE        (1)

/*
 * Types
 */

/*
 * Functions declarations
 */
void ARSTREAM_Buffers_InitStreamDataBuffer (ARNETWORK_IOBufferParam_t *bufferParams, int bufferID, int headerSize, int maxFragmentSize, uint32_t maxFragmentPerFrame);

void ARSTREAM_Buffers_InitStreamAckBuffer (ARNETWORK_IOBufferParam_t *bufferParams, int bufferID);

#endif /* _ARSTREAM_BUFFERS_PRIVATE_H_ */
