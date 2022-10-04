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
 * @file ARDATATRANSFER_Downloader.c
 * @brief libARDataTransfer Downloader c file.
 * @date 21/05/2014
 * @author david.flattin.ext@parrot.com
 **/

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>

#include <libARSAL/ARSAL_Sem.h>
#include <libARSAL/ARSAL_Mutex.h>
#include <libARSAL/ARSAL_Print.h>

#include <libARUtils/ARUTILS_Error.h>
#include <libARUtils/ARUTILS_Manager.h>

#include "libARDataTransfer/ARDATATRANSFER_Error.h"
#include "libARDataTransfer/ARDATATRANSFER_Manager.h"
#include "libARDataTransfer/ARDATATRANSFER_Downloader.h"
#include "libARDataTransfer/ARDATATRANSFER_Uploader.h"
#include "libARDataTransfer/ARDATATRANSFER_DataDownloader.h"
#include "libARDataTransfer/ARDATATRANSFER_MediasDownloader.h"
#include "ARDATATRANSFER_Downloader.h"
#include "ARDATATRANSFER_Uploader.h"
#include "ARDATATRANSFER_MediasQueue.h"
#include "ARDATATRANSFER_DataDownloader.h"
#include "ARDATATRANSFER_MediasDownloader.h"
#include "ARDATATRANSFER_Manager.h"


#define ARDATATRANSFER_DATA_UPLOADER_TAG          "Downloader"

eARDATATRANSFER_ERROR ARDATATRANSFER_Downloader_New (ARDATATRANSFER_Manager_t *manager, ARUTILS_Manager_t *ftpManager, const char *remotePath, const char *localPath, ARDATATRANSFER_Downloader_ProgressCallback_t progressCallback, void *progressArg, ARDATATRANSFER_Downloader_CompletionCallback_t completionCallback, void *completionArg, eARDATATRANSFER_DOWNLOADER_RESUME resume)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    //eARUTILS_ERROR resultUtil = ARUTILS_OK;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_DATA_UPLOADER_TAG, "%s", "");
    
    if ((manager == NULL) || (ftpManager == NULL))
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }
    
    if (result == ARDATATRANSFER_OK)
    {
        if (manager->downloader != NULL)
        {
            result = ARDATATRANSFER_ERROR_ALREADY_INITIALIZED;
        }
        else
        {
            manager->downloader = (ARDATATRANSFER_Downloader_t *)calloc(1, sizeof(ARDATATRANSFER_Downloader_t));
            
            if (manager->downloader == NULL)
            {
                result = ARDATATRANSFER_ERROR_ALLOC;
            }
        }
    }
    
    if (result == ARDATATRANSFER_OK)
    {
        manager->downloader->resume = resume;
        manager->downloader->ftpManager = ftpManager;
        
        strncpy(manager->downloader->remotePath, remotePath, ARUTILS_FTP_MAX_PATH_SIZE);
        manager->downloader->remotePath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
        
        strncpy(manager->downloader->localPath, localPath, ARUTILS_FTP_MAX_PATH_SIZE);
        manager->downloader->localPath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
        
        manager->downloader->progressCallback = progressCallback;
        manager->downloader->progressArg = progressArg;
        manager->downloader->completionCallback = completionCallback;
        manager->downloader->completionArg = completionArg;
    }
    
    return result;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_Downloader_Delete (ARDATATRANSFER_Manager_t *manager)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_DATA_UPLOADER_TAG, "%s", "");
    
    if (manager == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }
    
    if (result == ARDATATRANSFER_OK)
    {
        if (manager->downloader == NULL)
        {
            result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
        }
        else
        {
            free(manager->downloader);
            manager->downloader = NULL;
        }
    }
    
    return result;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_Downloader_GetSize (ARDATATRANSFER_Manager_t *manager, double *fileSize)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR resultUtil = ARUTILS_OK;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_DATA_UPLOADER_TAG, "%p", manager);
    
    if ((manager == NULL) || (manager->downloader ==  NULL))
    {
        result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
    }
    
    if (result == ARDATATRANSFER_OK)
    {
        resultUtil = ARUTILS_Manager_Ftp_Size(manager->downloader->ftpManager, manager->downloader->remotePath, fileSize);
        
        if (resultUtil != ARUTILS_OK)
        {
            result = ARDATATRANSFER_ERROR_FTP;
        }
    }
    
    return result;
}

void* ARDATATRANSFER_Downloader_ThreadRun (void *managerArg)
{
    ARDATATRANSFER_Manager_t *manager = (ARDATATRANSFER_Manager_t *)managerArg;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR resultUtil = ARUTILS_OK;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_DATA_UPLOADER_TAG, "%p", manager);
    
    if ((manager != NULL) && (manager->downloader !=  NULL))
    {
        resultUtil = ARUTILS_Manager_Ftp_Get(manager->downloader->ftpManager, manager->downloader->remotePath, manager->downloader->localPath, ARDATATRANSFER_Downloader_Ftp_ProgressCallback, manager, (manager->downloader->resume == ARDATATRANSFER_DOWNLOADER_RESUME_TRUE) ? FTP_RESUME_TRUE : FTP_RESUME_FALSE);
        
        if (resultUtil != ARUTILS_OK)
        {
            result = ARDATATRANSFER_ERROR_FTP;
        }
        if (manager->downloader->completionCallback != NULL)
        {
            manager->downloader->completionCallback(manager->downloader->completionArg, result);
        }
    }
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_DATA_UPLOADER_TAG, "exiting");
    
    return NULL;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_Downloader_CancelThread (ARDATATRANSFER_Manager_t *manager)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR resultUtil = ARUTILS_OK;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_DATA_UPLOADER_TAG, "%s", "");
    
    if (manager == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }
    
    if (result == ARDATATRANSFER_OK)
    {
        if (manager->downloader == NULL)
        {
            result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
        }
    }
    
    if (result == ARDATATRANSFER_OK)
    {
        resultUtil = ARUTILS_Manager_Ftp_Connection_Cancel(manager->downloader->ftpManager);
        if (resultUtil != ARUTILS_OK)
        {
            result = ARDATATRANSFER_ERROR_FTP;
        }
    }
    
    return result;
}

/*****************************************
 *
 *             Private implementation:
 *
 *****************************************/

void ARDATATRANSFER_Downloader_Ftp_ProgressCallback(void* arg, float percent)
{
    ARDATATRANSFER_Manager_t *manager = (ARDATATRANSFER_Manager_t *)arg;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_DATA_UPLOADER_TAG, "%s", "");
    
    if (manager->downloader->progressCallback != NULL)
    {
        manager->downloader->progressCallback(manager->downloader->progressArg, percent);
    }
}
