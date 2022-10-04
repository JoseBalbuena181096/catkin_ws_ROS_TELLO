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
 * @file ARUPDATER_Error.c
 * @brief ToString function for eARUPDATER_ERROR enum
 */

#include <libARUpdater/ARUPDATER_Error.h>

const char* ARUPDATER_Error_ToString (eARUPDATER_ERROR error)
{
    switch (error)
    {
    case ARUPDATER_OK:
        return "No error";
        break;
    case ARUPDATER_ERROR:
        return "Unknown generic error";
        break;
    case ARUPDATER_ERROR_ALLOC:
        return "Memory allocation error";
        break;
    case ARUPDATER_ERROR_BAD_PARAMETER:
        return "Bad parameters error";
        break;
    case ARUPDATER_ERROR_SYSTEM:
        return "System error";
        break;
    case ARUPDATER_ERROR_THREAD_PROCESSING:
        return "Thread processing error";
        break;
    case ARUPDATER_ERROR_MANAGER:
        return "Generic manager error";
        break;
    case ARUPDATER_ERROR_MANAGER_ALREADY_INITIALIZED:
        return "The uploader or downloader is already initilized in the manager";
        break;
    case ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED:
        return "The uploader or downloader is not initialized in the manager";
        break;
    case ARUPDATER_ERROR_MANAGER_BUFFER_TOO_SMALL:
        return "The given buffer is too small";
        break;
    case ARUPDATER_ERROR_PLF:
        return "Generic PLF error";
        break;
    case ARUPDATER_ERROR_PLF_FILE_NOT_FOUND:
        return "Plf File not found";
        break;
    case ARUPDATER_ERROR_DOWNLOADER:
        return "Generic Updater error";
        break;
    case ARUPDATER_ERROR_DOWNLOADER_ARUTILS_ERROR:
        return "error on a ARUtils operation";
        break;
    case ARUPDATER_ERROR_DOWNLOADER_DOWNLOAD:
        return "error downloading a file";
        break;
    case ARUPDATER_ERROR_DOWNLOADER_PLATFORM_ERROR:
        return "error on a platform name";
        break;
    case ARUPDATER_ERROR_DOWNLOADER_PHP_APP_OUT_TO_DATE_ERROR:
        return "This app version is out to date";
        break;
    case ARUPDATER_ERROR_DOWNLOADER_PHP_ERROR:
        return "error given by the PHP script on server";
        break;
    case ARUPDATER_ERROR_DOWNLOADER_RENAME_FILE:
        return "error when renaming files";
        break;
    case ARUPDATER_ERROR_DOWNLOADER_FILE_NOT_FOUND:
        return "Plf file not found in the downloader";
        break;
    case ARUPDATER_ERROR_DOWNLOADER_MD5_DONT_MATCH:
        return "MD5 checksum does not match with the remote file";
        break;
    case ARUPDATER_ERROR_UPLOADER:
        return "Generic Uploader error";
        break;
    case ARUPDATER_ERROR_UPLOADER_ARUTILS_ERROR:
        return "error on a ARUtils operation in uploader";
        break;
    case ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR:
        return "error on a ARDataTransfer operation in uploader";
        break;
    case ARUPDATER_ERROR_UPLOADER_ARSAL_ERROR:
        return "error on a ARSAL operation in uploader";
        break;
    default:
        break;
    }
    return "Unknown value";
}
