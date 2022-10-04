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
 * @file ARDATATRANSFER_Uploader.c
 * @brief libARDataTransfer Uploader c file.
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


#define ARDATATRANSFER_DATA_UPLOADER_TAG          "Uploader"

eARDATATRANSFER_ERROR ARDATATRANSFER_Uploader_New (ARDATATRANSFER_Manager_t *manager, ARUTILS_Manager_t *ftpManager, const char *remotePath, const char *localPath, ARDATATRANSFER_Uploader_ProgressCallback_t progressCallback, void *progressArg, ARDATATRANSFER_Uploader_CompletionCallback_t completionCallback, void *completionArg, eARDATATRANSFER_UPLOADER_RESUME resume)
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
        if (manager->uploader != NULL)
        {
            result = ARDATATRANSFER_ERROR_ALREADY_INITIALIZED;
        }
        else
        {
            manager->uploader = (ARDATATRANSFER_Uploader_t *)calloc(1, sizeof(ARDATATRANSFER_Uploader_t));
            
            if (manager->uploader == NULL)
            {
                result = ARDATATRANSFER_ERROR_ALLOC;
            }
        }
    }
    
    if (result == ARDATATRANSFER_OK)
    {
        manager->uploader->resume = resume;
        manager->uploader->ftpManager = ftpManager;

        strncpy(manager->uploader->remotePath, remotePath, ARUTILS_FTP_MAX_PATH_SIZE);
        manager->uploader->remotePath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';

        strncpy(manager->uploader->localPath, localPath, ARUTILS_FTP_MAX_PATH_SIZE);
        manager->uploader->localPath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
        
        manager->uploader->progressCallback = progressCallback;
        manager->uploader->progressArg = progressArg;
        manager->uploader->completionCallback = completionCallback;
        manager->uploader->completionArg = completionArg;
    }
        
    return result;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_Uploader_Delete (ARDATATRANSFER_Manager_t *manager)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_DATA_UPLOADER_TAG, "%s", "");
    
    if (manager == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }
    
    if (result == ARDATATRANSFER_OK)
    {
        if (manager->uploader == NULL)
        {
            result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
        }
        else
        {
            free(manager->uploader);
            manager->uploader = NULL;
        }
    }
    
    return result;
}

void* ARDATATRANSFER_Uploader_ThreadRun (void *managerArg)
{
    ARDATATRANSFER_Manager_t *manager = (ARDATATRANSFER_Manager_t *)managerArg;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR resultUtil = ARUTILS_OK;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_DATA_UPLOADER_TAG, "%p", manager);
    
    if ((manager != NULL) && (manager->uploader !=  NULL))
    {
        resultUtil = ARUTILS_Manager_Ftp_Put(manager->uploader->ftpManager, manager->uploader->remotePath, manager->uploader->localPath, ARDATATRANSFER_Uploader_Ftp_ProgressCallback, manager, (manager->uploader->resume == ARDATATRANSFER_UPLOADER_RESUME_TRUE) ? FTP_RESUME_TRUE : FTP_RESUME_FALSE);
        
        if (resultUtil != ARUTILS_OK)
        {
            result = ARDATATRANSFER_ERROR_FTP;
        }
        if (manager->uploader->completionCallback != NULL)
        {
            manager->uploader->completionCallback(manager->uploader->completionArg, result);
        }
    }
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_DATA_UPLOADER_TAG, "exiting");
    
    return NULL;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_Uploader_CancelThread (ARDATATRANSFER_Manager_t *manager)
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
        if (manager->uploader == NULL)
        {
            result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
        }
    }
    
    if (result == ARDATATRANSFER_OK)
    {
        resultUtil = ARUTILS_Manager_Ftp_Connection_Cancel(manager->uploader->ftpManager);
        if (resultUtil != ARUTILS_OK)
        {
            result = ARDATATRANSFER_ERROR_FTP;
        }
    }
    
    return result;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_Uploader_Rename (ARDATATRANSFER_Manager_t *manager, const char *oldNamePath, const char *newNamePath)
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
        if (manager->uploader == NULL)
        {
            result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
        }
        else
        {
            resultUtil = ARUTILS_Manager_Ftp_Rename(manager->uploader->ftpManager, oldNamePath, newNamePath);
        }
        
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

void ARDATATRANSFER_Uploader_Ftp_ProgressCallback(void* arg, float percent)
{
    ARDATATRANSFER_Manager_t *manager = (ARDATATRANSFER_Manager_t *)arg;
    
    if (manager->uploader->progressCallback != NULL)
    {
        manager->uploader->progressCallback(manager->uploader->progressArg, percent);
    }
}
