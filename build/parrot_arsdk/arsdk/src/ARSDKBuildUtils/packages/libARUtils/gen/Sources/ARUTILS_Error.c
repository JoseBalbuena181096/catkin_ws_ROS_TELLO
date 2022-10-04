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
 * @file ARUTILS_Error.c
 * @brief ToString function for eARUTILS_ERROR enum
 */

#include <libARUtils/ARUTILS_Error.h>

const char* ARUTILS_Error_ToString (eARUTILS_ERROR error)
{
    switch (error)
    {
    case ARUTILS_OK:
        return "No error";
        break;
    case ARUTILS_ERROR:
        return "Unknown generic error";
        break;
    case ARUTILS_ERROR_ALLOC:
        return "Memory allocation error";
        break;
    case ARUTILS_ERROR_BAD_PARAMETER:
        return "Bad parameters error";
        break;
    case ARUTILS_ERROR_SYSTEM:
        return "System error";
        break;
    case ARUTILS_ERROR_NOT_IMPLEMENTED:
        return "Function not implemented";
        break;
    case ARUTILS_ERROR_CURL_ALLOC:
        return "curl allocation error";
        break;
    case ARUTILS_ERROR_CURL_SETOPT:
        return "curl set option error";
        break;
    case ARUTILS_ERROR_CURL_GETINFO:
        return "curl get info error";
        break;
    case ARUTILS_ERROR_CURL_PERFORM:
        return "curl perform error";
        break;
    case ARUTILS_ERROR_FILE_NOT_FOUND:
        return "file not found error";
        break;
    case ARUTILS_ERROR_FTP_CONNECT:
        return "ftp connect error";
        break;
    case ARUTILS_ERROR_FTP_CODE:
        return "ftp code error";
        break;
    case ARUTILS_ERROR_FTP_SIZE:
        return "ftp file size error";
        break;
    case ARUTILS_ERROR_FTP_RESUME:
        return "ftp resume error";
        break;
    case ARUTILS_ERROR_FTP_CANCELED:
        return "ftp user canceled error";
        break;
    case ARUTILS_ERROR_FTP_FILE:
        return "ftp file error";
        break;
    case ARUTILS_ERROR_FTP_MD5:
        return "ftp md5 error";
        break;
    case ARUTILS_ERROR_HTTP_CONNECT:
        return "http connect error";
        break;
    case ARUTILS_ERROR_HTTP_CODE:
        return "http code error";
        break;
    case ARUTILS_ERROR_HTTP_AUTHORIZATION_REQUIRED:
        return "http authorization required";
        break;
    case ARUTILS_ERROR_HTTP_ACCESS_DENIED:
        return "http access denied";
        break;
    case ARUTILS_ERROR_HTTP_SIZE:
        return "http file size error";
        break;
    case ARUTILS_ERROR_HTTP_RESUME:
        return "http resume error";
        break;
    case ARUTILS_ERROR_HTTP_CANCELED:
        return "http user canceled error";
        break;
    case ARUTILS_ERROR_BLE_FAILED:
        return "BLE ftp failed error";
        break;
    case ARUTILS_ERROR_NETWORK_TYPE:
        return "Network type, not available for the platform error";
        break;
    case ARUTILS_ERROR_RFCOMM_FAILED:
        return "RFComm ftp failed error";
        break;
    default:
        break;
    }
    return "Unknown value";
}
