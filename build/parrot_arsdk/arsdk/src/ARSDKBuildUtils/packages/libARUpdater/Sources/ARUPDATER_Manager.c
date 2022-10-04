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
 * @file ARUPDATER_Manager.c
 * @brief libARUpdater Manager c file.
 * @date07/01/2014
 * @author david.flattin.ext@parrot.com
 **/
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <libARSAL/ARSAL_Print.h>

#include <libARUpdater/ARUPDATER_Error.h>
#include <libARUpdater/ARUPDATER_Manager.h>
#include "ARUPDATER_Manager.h"
#include "ARUPDATER_Utils.h"

#define ARUPDATER_MANAGER_TAG   "ARUPDATER_Manager"

#define ARUPDATER_MANAGER_VERSION_ELEMENT_BUFFER_SIZE   15
#define ARUPDATER_MANAGER_FULL_VERSION_BUFFER_SIZE      127
#define ARUPDATER_MANAGER_VERSION_SEPARATOR             "."

ARUPDATER_Manager_t* ARUPDATER_Manager_New(eARUPDATER_ERROR *error)
{
    ARUPDATER_Manager_t *manager = NULL;
    eARUPDATER_ERROR err = ARUPDATER_OK;
    
    /* Check parameters */
    if(err == ARUPDATER_OK)
    {
        /* Create the Manager */
        manager = malloc (sizeof (ARUPDATER_Manager_t));
        if (manager == NULL)
        {
            err = ARUPDATER_ERROR_ALLOC;
        }
    }

    /* Initialize to default values */
    if (ARUPDATER_OK == err)
    {
        manager->downloader = NULL;
        manager->uploader = NULL;
    }
    
    /* delete the Manager if an error occurred */
    if (err != ARUPDATER_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_MANAGER_TAG, "error: %s", ARUPDATER_Error_ToString (err));
        ARUPDATER_Manager_Delete (&manager);
    }
    
    /* return the error */
    if (error != NULL)
    {
        *error = err;
    }
    
    return manager;
}

void ARUPDATER_Manager_Delete (ARUPDATER_Manager_t **managerPtrAddr)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_MANAGER_TAG, "     ");

    if (managerPtrAddr != NULL)
    {
        ARUPDATER_Manager_t *manager = *managerPtrAddr;
        
        if (manager != NULL)
        {
            if (manager->downloader != NULL)
            {
                ARUPDATER_Downloader_Delete(manager);
            }
            
            if (manager->uploader != NULL)
            {
                ARUPDATER_Uploader_Delete(manager);
            }
                        
            free(manager);
            *managerPtrAddr = NULL;
        }
    }
}

int ARUPDATER_Manager_PlfVersionIsUpToDate(ARUPDATER_Manager_t *manager,
		eARDISCOVERY_PRODUCT product, const char *remoteVersion,
		const char *rootFolder, char *localVersionBuffer,
		size_t bufferSize, eARUPDATER_ERROR *error)
{
    eARUPDATER_ERROR err = ARUPDATER_OK;
    ARUPDATER_PlfVersion local;
    ARUPDATER_PlfVersion remote;
    int ret, retVal = 1;

    char *device = NULL;
    char *productFolder = NULL;
    char *plfFilename = NULL;
    char *sourceFilePath = NULL;
    
    if ((manager == NULL) ||
        (rootFolder == NULL))
    {
        err = ARUPDATER_ERROR_BAD_PARAMETER;
    }
    
    if (err == ARUPDATER_OK)
    {
        uint16_t productId = ARDISCOVERY_getProductID(product);
        device = malloc(ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE);
        snprintf(device, ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE, "%04x", productId);
        
        int productFolderLength = strlen(rootFolder) + strlen(ARUPDATER_MANAGER_PLF_FOLDER) + strlen(device) + strlen(ARUPDATER_MANAGER_FOLDER_SEPARATOR) + 1;
        char *slash = strrchr(rootFolder, ARUPDATER_MANAGER_FOLDER_SEPARATOR[0]);
        if ((slash != NULL) && (strcmp(slash, ARUPDATER_MANAGER_FOLDER_SEPARATOR) != 0))
        {
            productFolderLength += 1;
        }
        productFolder = (char*) malloc(productFolderLength);
        strcpy(productFolder, rootFolder);
        
        if ((slash != NULL) && (strcmp(slash, ARUPDATER_MANAGER_FOLDER_SEPARATOR) != 0))
        {
            strcat(productFolder, ARUPDATER_MANAGER_FOLDER_SEPARATOR);
        }
        strcat(productFolder, ARUPDATER_MANAGER_PLF_FOLDER);
        strcat(productFolder, device);
        strcat(productFolder, ARUPDATER_MANAGER_FOLDER_SEPARATOR);
        
        plfFilename = NULL;
        err = ARUPDATER_Utils_GetPlfInFolder(productFolder, &plfFilename);
    }
    
    if (err == ARUPDATER_OK)
    {
        sourceFilePath = malloc(strlen(productFolder) + strlen(plfFilename) + 1);
        strcpy(sourceFilePath, productFolder);
        strcat(sourceFilePath, plfFilename);

        /* Extract plf version from local file */
        err = ARUPDATER_Utils_ReadPlfVersion(sourceFilePath, &local);
    }

    if (err == ARUPDATER_OK)
    {

        /* Convert local version to string  */
        ARUPDATER_Utils_PlfVersionToString(&local, localVersionBuffer, bufferSize);

        /* Parse remote version and convert it into PlfVersion structure */
        ARUPDATER_Utils_PlfVersionFromString(remoteVersion, &remote);

        /* Compare versions */
        ret = ARUPDATER_Utils_PlfVersionCompare(&local, &remote);
        retVal = (ret > 0) ? 0 : 1;

        ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_MANAGER_TAG, "remote:'%s' local:'%s' uptodate=%d", remoteVersion, localVersionBuffer, retVal);
    }
    
    if (productFolder)
    {
        free(productFolder);
    }
    if (sourceFilePath)
    {
        free(sourceFilePath);
    }
    if (device)
    {
        free(device);
    }
    if (plfFilename)
    {
        free(plfFilename);
    }
    
    if (error != NULL)
    {
        *error = err;
    }
    
    return retVal;
}
