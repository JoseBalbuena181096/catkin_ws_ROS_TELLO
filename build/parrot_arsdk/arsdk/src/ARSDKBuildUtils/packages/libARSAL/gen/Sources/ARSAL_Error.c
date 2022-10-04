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
 * @file ARSAL_Error.c
 * @brief ToString function for eARSAL_ERROR enum
 */

#include <libARSAL/ARSAL_Error.h>

const char* ARSAL_Error_ToString (eARSAL_ERROR error)
{
    switch (error)
    {
    case ARSAL_OK:
        return "No error";
        break;
    case ARSAL_ERROR:
        return "ARSAL Generic error";
        break;
    case ARSAL_ERROR_ALLOC:
        return "ARSAL alloc error";
        break;
    case ARSAL_ERROR_SYSTEM:
        return "ARSAL system error";
        break;
    case ARSAL_ERROR_BAD_PARAMETER:
        return "ARSAL bad parameter error";
        break;
    case ARSAL_ERROR_FILE:
        return "ARSAL file error";
        break;
    case ARSAL_ERROR_MD5:
        return "ARSAL md5 error";
        break;
    case ARSAL_ERROR_BLE_CONNECTION:
        return "BLE connection generic error";
        break;
    case ARSAL_ERROR_BLE_NOT_CONNECTED:
        return "BLE is not connected";
        break;
    case ARSAL_ERROR_BLE_DISCONNECTION:
        return "BLE disconnection error";
        break;
    case ARSAL_ERROR_BLE_SERVICES_DISCOVERING:
        return "BLE network services discovering error";
        break;
    case ARSAL_ERROR_BLE_CHARACTERISTICS_DISCOVERING:
        return "BLE network characteristics discovering error";
        break;
    case ARSAL_ERROR_BLE_CHARACTERISTIC_CONFIGURING:
        return "BLE network characteristic configuring error";
        break;
    case ARSAL_ERROR_BLE_STACK:
        return "BLE stack generic error";
        break;
    case ARSAL_ERROR_BLE_TIMEOUT:
        return "BLE timeout";
        break;
    case ARSAL_ERROR_BLE_NO_DATA:
        return "BLE no data";
        break;
    default:
        break;
    }
    return "Unknown value";
}
