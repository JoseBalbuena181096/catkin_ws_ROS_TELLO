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
 * @file ARUPDATER_DownloadInformation.c
 * @brief libARUpdater Download information c file.
 * @date 23/05/2014
 * @author djavan.bertrand@parrot.com
 **/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libARSAL/ARSAL_Print.h>
#include "ARUPDATER_DownloadInformation.h"

/* ***************************************
 *
 *             define :
 *
 *****************************************/

#define ARUPDATER_DOWNLOAD_INFORMATION_TAG "DownloadInformation"

/* ***************************************
 *
 *             function implementation :
 *
 *****************************************/

ARUPDATER_DownloadInformation_t* ARUPDATER_DownloadInformation_New(const char *const downloadUrl, const char *const md5Expected, const char *const plfVersion, int remoteSize, const eARDISCOVERY_PRODUCT product, eARUPDATER_ERROR *error)
{
    ARUPDATER_DownloadInformation_t *downloadInfo = NULL;
    eARUPDATER_ERROR err = ARUPDATER_OK;
    
    if(err == ARUPDATER_OK)
    {
        /* Create the dlInfo */
        downloadInfo = malloc (sizeof (ARUPDATER_DownloadInformation_t));
        if (downloadInfo == NULL)
        {
            err = ARUPDATER_ERROR_ALLOC;
        }
    }
    
    /* Initialize to default values */
    if(err == ARUPDATER_OK)
    {
        if (downloadUrl != NULL)
        {
            downloadInfo->downloadUrl = malloc(strlen(downloadUrl) + 1);
            strcpy(downloadInfo->downloadUrl, downloadUrl);
        }
        else
        {
            downloadInfo->downloadUrl = NULL;
        }
        
        if (md5Expected != NULL)
        {
            downloadInfo->md5Expected = malloc(strlen(md5Expected) + 1);
            strcpy(downloadInfo->md5Expected, md5Expected);
        }
        else
        {
            downloadInfo->md5Expected = NULL;
        }
        
        if (plfVersion != NULL)
        {
            downloadInfo->plfVersion = malloc(strlen(plfVersion) + 1);
            strcpy(downloadInfo->plfVersion, plfVersion);
        }
        else
        {
            downloadInfo->plfVersion = NULL;
        }
        
        downloadInfo->remoteSize = remoteSize;
        
        downloadInfo->product = product;
    }
    
    /* delete the downloader if an error occurred */
    if (err != ARUPDATER_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_DOWNLOAD_INFORMATION_TAG, "error: %s", ARUPDATER_Error_ToString (err));
        ARUPDATER_DownloadInformation_Delete (&downloadInfo);
    }
    
    /* return the error */
    if (error != NULL)
    {
        *error = err;
    } // else let error at its previous value
    
    return downloadInfo;

}

void ARUPDATER_DownloadInformation_Delete(ARUPDATER_DownloadInformation_t **downloadInfo)
{
    ARUPDATER_DownloadInformation_t *downloadInfoPtr = NULL;
    
    if (downloadInfo)
    {
        downloadInfoPtr = *downloadInfo;
        
        if (downloadInfoPtr)
        {
            // Uninitialize here
            if (downloadInfoPtr->downloadUrl != NULL)
            {
                free(downloadInfoPtr->downloadUrl);
                downloadInfoPtr->downloadUrl = NULL;
            }
            
            if (downloadInfoPtr->md5Expected != NULL)
            {
                free(downloadInfoPtr->md5Expected);
                downloadInfoPtr->md5Expected = NULL;
            }
            
            if (downloadInfoPtr->plfVersion != NULL)
            {
                free(downloadInfoPtr->plfVersion);
                downloadInfoPtr->plfVersion = NULL;
            }
            
            free (downloadInfoPtr);
            downloadInfoPtr = NULL;
        }
        
        *downloadInfo = NULL;
    }
}