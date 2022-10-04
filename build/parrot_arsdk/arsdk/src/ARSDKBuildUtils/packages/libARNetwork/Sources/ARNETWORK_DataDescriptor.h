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
 * @file ARNETWORK_DataDescriptor.h
 * @brief ARNETWORK_DataDescriptor_t used by ARNETWORK_IOBuffer_t
 * @date 05/18/2012
 * @author maxime.maitre@parrot.com
**/

#ifndef _ARNETWORK_DATADESCRIPTOR_PRIVATE_H_
#define _ARNETWORK_DATADESCRIPTOR_PRIVATE_H_

#include <libARNetwork/ARNETWORK_Error.h>
#include <libARNetwork/ARNETWORK_Manager.h>

/*****************************************
 * 
 *             define:
 *
******************************************/

/*****************************************
 * 
 *             header:
 *
******************************************/

/**
 * @brief data sent by the network manager
**/
typedef struct  
{
    uint8_t *data; /**< data to send*/
    size_t dataSize; /**< size of the data */
    void *customData; /**< custom data */
    ARNETWORK_Manager_Callback_t callback; /**< call back use when the data are sent or timeout occurred */
    int isUsingDataCopy; /**< Indicator of using copy of data */
    
}ARNETWORK_DataDescriptor_t;

#endif /** _ARNETWORK_DATADESCRIPTOR_PRIVATE_H_ */
