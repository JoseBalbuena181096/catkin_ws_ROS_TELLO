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
 * @file ARUPDATER_Updater.c
 * @brief libARUpdater PlfSender c file.
 * @date 23/05/2014
 * @author djavan.bertrand@parrot.com
 **/

#ifndef _ARUPDATER_UPLOADER_PRIVATE_H_
#define _ARUPDATER_UPLOADER_PRIVATE_H_

#include <libARUpdater/ARUPDATER_Uploader.h>
#include <libARDataTransfer/ARDATATRANSFER_Uploader.h>
#include <libARDataTransfer/ARDATATRANSFER_Downloader.h>
#include <libARSAL/ARSAL_Mutex.h>

/* forward declaration */
struct mux_ctx;

struct ARUPDATER_Uploader_t
{
    char *rootFolder;
    eARDISCOVERY_PRODUCT product;
    int isAndroidApp;
    char *subfolder;

    /* transport layer: ftp or mux */
    ARUTILS_Manager_t *ftpManager;
    struct mux_ctx *mux;
    /* mux vars */
    int fd;
    size_t size;
    size_t n_written;
    void *chunk;
    size_t chunk_id;
    int pipefds[2];

    int isRunning;
    int isCanceled;
    int isUploadThreadRunning;
    int isDownloadMd5ThreadRunning;

    ARSAL_MD5_Manager_t *md5Manager;

    ARSAL_Mutex_t uploadLock;

    ARDATATRANSFER_Manager_t* dataTransferManager;

    ARUPDATER_Uploader_PlfUploadProgressCallback_t progressCallback;
    ARUPDATER_Uploader_PlfUploadCompletionCallback_t completionCallback;
    void *progressArg;
    void *completionArg;

    eARDATATRANSFER_ERROR uploadError;
};

void ARUPDATER_Uploader_ProgressCallback(void* arg, float percent);
void ARUPDATER_Uploader_CompletionCallback(void* arg, eARDATATRANSFER_ERROR error);

eARUPDATER_ERROR ARUPDATER_Uploader_ThreadRunAndroidDelos(ARUPDATER_Manager_t *manager);
eARUPDATER_ERROR ARUPDATER_Uploader_ThreadRunNormal(ARUPDATER_Manager_t *manager);
eARUPDATER_ERROR ARUPDATER_Uploader_ThreadRunMux(ARUPDATER_Manager_t *manager);

#endif
