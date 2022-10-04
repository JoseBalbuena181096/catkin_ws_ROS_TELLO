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
//
//  ARUPDATER_Updater.h
//  ARUpdaterCLibProject
//
//  Created by Djavan Bertrand on 23/05/2014.
//
//

#ifndef _ARUPDATER_DOWNLOADER_PRIVATE_H_
#define _ARUPDATER_DOWNLOADER_PRIVATE_H_

#include <libARUpdater/ARUPDATER_Error.h>
#include <libARUpdater/ARUPDATER_Downloader.h>
#include <libARSAL/ARSAL_Mutex.h>
#include "ARUPDATER_DownloadInformation.h"

struct ARUPDATER_Downloader_t
{
    char *rootFolder;

    eARUPDATER_Downloader_Platforms appPlatform;
    char *appVersion;
    char *variant;

    void *downloadArg;
    void *willDownloadPlfArg;
    void *progressArg;
    void *completionArg;

    int isRunning;
    int isCanceled;

    int updateHasBeenChecked;
    ARUPDATER_DownloadInformation_t **downloadInfos;
    ARUPDATER_Manager_BlacklistedFirmware_t **blacklistedVersions;
    eARDISCOVERY_PRODUCT *productList;
    int productCount;

    ARSAL_MD5_Manager_t *md5Manager;

    ARSAL_Mutex_t requestLock;
    ARSAL_Mutex_t downloadLock;
    ARUTILS_Http_Connection_t *requestConnection;
    ARUTILS_Http_Connection_t *downloadConnection;
    
    ARSAL_Mutex_t requestBlacklistLock;
    ARUTILS_Http_Connection_t *requestBlacklistConnection;

    ARUPDATER_Downloader_ShouldDownloadPlfCallback_t shouldDownloadCallback;
    ARUPDATER_Downloader_WillDownloadPlfCallback_t willDownloadPlfCallback;
    ARUPDATER_Downloader_PlfDownloadProgressCallback_t plfDownloadProgressCallback;
    ARUPDATER_Downloader_PlfDownloadCompletionCallback_t plfDownloadCompletionCallback;
};

char *ARUPDATER_Downloader_GetPlatformName(eARUPDATER_Downloader_Platforms platform);

#endif
