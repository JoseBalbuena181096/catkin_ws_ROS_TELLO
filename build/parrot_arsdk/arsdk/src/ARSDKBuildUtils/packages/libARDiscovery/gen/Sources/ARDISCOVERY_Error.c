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
 * @file ARDISCOVERY_Error.c
 * @brief ToString function for eARDISCOVERY_ERROR enum
 */

#include <libARDiscovery/ARDISCOVERY_Error.h>

const char* ARDISCOVERY_Error_ToString (eARDISCOVERY_ERROR error)
{
    switch (error)
    {
    case ARDISCOVERY_OK:
        return "No error";
        break;
    case ARDISCOVERY_ERROR:
        return "Unknown generic error";
        break;
    case ARDISCOVERY_ERROR_SIMPLE_POLL:
        return "Avahi failed to create simple poll object";
        break;
    case ARDISCOVERY_ERROR_BUILD_NAME:
        return "Avahi failed to create simple poll object";
        break;
    case ARDISCOVERY_ERROR_CLIENT:
        return "Avahi failed to create client";
        break;
    case ARDISCOVERY_ERROR_CREATE_CONFIG:
        return "Failed to create config file";
        break;
    case ARDISCOVERY_ERROR_DELETE_CONFIG:
        return "Failed to delete config file";
        break;
    case ARDISCOVERY_ERROR_ENTRY_GROUP:
        return "Avahi failed to create entry group";
        break;
    case ARDISCOVERY_ERROR_ADD_SERVICE:
        return "Avahi failed to add service";
        break;
    case ARDISCOVERY_ERROR_GROUP_COMMIT:
        return "Avahi failed to commit group";
        break;
    case ARDISCOVERY_ERROR_BROWSER_ALLOC:
        return "Avahi failed to allocate desired number of browsers";
        break;
    case ARDISCOVERY_ERROR_BROWSER_NEW:
        return "Avahi failed to create one browser";
        break;
    case ARDISCOVERY_ERROR_ALLOC:
        return "Failed to allocate connection resources";
        break;
    case ARDISCOVERY_ERROR_INIT:
        return "Wrong type to connect as";
        break;
    case ARDISCOVERY_ERROR_SOCKET_CREATION:
        return "Socket creation error";
        break;
    case ARDISCOVERY_ERROR_SOCKET_PERMISSION_DENIED:
        return "Socket access permission denied";
        break;
    case ARDISCOVERY_ERROR_SOCKET_ALREADY_CONNECTED:
        return "Socket is already connected";
        break;
    case ARDISCOVERY_ERROR_ACCEPT:
        return "Socket accept failed";
        break;
    case ARDISCOVERY_ERROR_SEND:
        return "Failed to write frame to socket";
        break;
    case ARDISCOVERY_ERROR_READ:
        return "Failed to read frame from socket";
        break;
    case ARDISCOVERY_ERROR_SELECT:
        return "Failed to select sets";
        break;
    case ARDISCOVERY_ERROR_TIMEOUT:
        return "timeout error";
        break;
    case ARDISCOVERY_ERROR_ABORT:
        return "Aborted by the user";
        break;
    case ARDISCOVERY_ERROR_PIPE_INIT:
        return "Failed to intitialize a pipe";
        break;
    case ARDISCOVERY_ERROR_BAD_PARAMETER:
        return "Bad parameters";
        break;
    case ARDISCOVERY_ERROR_BUSY:
        return "discovery is busy";
        break;
    case ARDISCOVERY_ERROR_SOCKET_UNREACHABLE:
        return "host or net is not reachable";
        break;
    case ARDISCOVERY_ERROR_OUTPUT_LENGTH:
        return "the length of the output is to small";
        break;
    case ARDISCOVERY_ERROR_JNI:
        return "JNI error";
        break;
    case ARDISCOVERY_ERROR_JNI_VM:
        return "JNI virtual machine, not initialized";
        break;
    case ARDISCOVERY_ERROR_JNI_ENV:
        return "null JNI environment";
        break;
    case ARDISCOVERY_ERROR_JNI_CALLBACK_LISTENER:
        return "null jni callback listener";
        break;
    case ARDISCOVERY_ERROR_CONNECTION:
        return "Connection error";
        break;
    case ARDISCOVERY_ERROR_CONNECTION_BUSY:
        return "Product already connected";
        break;
    case ARDISCOVERY_ERROR_CONNECTION_NOT_READY:
        return "Product not ready to connect";
        break;
    case ARDISCOVERY_ERROR_CONNECTION_BAD_ID:
        return "It is not the good Product";
        break;
    case ARDISCOVERY_ERROR_DEVICE:
        return "Device generic error";
        break;
    case ARDISCOVERY_ERROR_DEVICE_OPERATION_NOT_SUPPORTED:
        return "The current device does not support this operation";
        break;
    case ARDISCOVERY_ERROR_JSON:
        return "Json generic error";
        break;
    case ARDISCOVERY_ERROR_JSON_PARSSING:
        return "Json parssing error";
        break;
    case ARDISCOVERY_ERROR_JSON_BUFFER_SIZE:
        return "The size of the buffer storing the Json is too small";
        break;
    default:
        break;
    }
    return "Unknown value";
}
