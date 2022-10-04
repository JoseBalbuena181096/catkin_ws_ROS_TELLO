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
 * @file ARUPDATER_Downloader.c
 * @brief libARUpdater Downloader c file.
 * @date 23/05/2014
 * @author djavan.bertrand@parrot.com
 **/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/stat.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Error.h>
#include <libARUtils/ARUTILS_Http.h>
#include "ARUPDATER_Manager.h"
#include "ARUPDATER_Downloader.h"
#include "ARUPDATER_Utils.h"
#include <json-c/json.h>

/* ***************************************
 *
 *             define :
 *
 *****************************************/
#define ARUPDATER_DOWNLOADER_TAG   "ARUPDATER_Downloader"

#define ARUPDATER_DOWNLOADER_SERVER_URL                    "download.parrot.com"
#define ARUPDATER_DOWNLOADER_BEGIN_URL                     "/Drones/"
#define ARUPDATER_DOWNLOADER_PHP_URL                       "/update.php"
#define ARUPDATER_DOWNLOADER_PHP_BLACKLIST_FIRM_URL        "firmware_blacklist.php"
#define ARUPDATER_DOWNLOADER_PARAM_MAX_LENGTH              255
#define ARUPDATER_DOWNLOADER_VERSION_BUFFER_MAX_LENGHT     10
#define ARUPDATER_DOWNLOADER_PRODUCT_PARAM                 "?product="
#define ARUPDATER_DOWNLOADER_SERIAL_PARAM                  "&serialNo="
#define ARUPDATER_DOWNLOADER_VERSION_PARAM                 "&version="
#define ARUPDATER_DOWNLOADER_APP_PLATFORM_PARAM            "&platform="
#define ARUPDATER_DOWNLOADER_APP_PLATFORM_PARAM_BEGIN      "?platform="
#define ARUPDATER_DOWNLOADER_APP_VERSION_PARAM             "&appVersion="
#define ARUPDATER_DOWNLOADER_VARIANT_PARAM                 "&variant="
#define ARUPDATER_DOWNLOADER_VERSION_SEPARATOR             "."
#define ARUPDATER_DOWNLOADER_DOWNLOADED_FILE_PREFIX        "tmp_"
#define ARUPDATER_DOWNLOADER_DOWNLOADED_FILE_SUFFIX        ".tmp"
#define ARUPDATER_DOWNLOADER_SERIAL_DEFAULT_VALUE          "0000"

#define ARUPDATER_DOWNLOADER_PHP_ERROR_OK                       "0"
#define ARUPDATER_DOWNLOADER_PHP_ERROR_UPDATE                   "5"
#define ARUPDATER_DOWNLOADER_PHP_ERROR_APP_VERSION_OUT_TO_DATE  "3"

#define ARUPDATER_DOWNLOADER_CHUNK_SIZE                    255
#define ARUPDATER_DOWNLOADER_MD5_TXT_SIZE                  32
#define ARUPDATER_DOWNLOADER_MD5_HEX_SIZE                  16

#define ARUPDATER_DOWNLOADER_HTTP_HEADER                   "http://"

#define ARUPDATER_DOWNLOADER_ANDROID_PLATFORM_NAME         "Android"
#define ARUPDATER_DOWNLOADER_IOS_PLATFORM_NAME             "iOS"

#define ARUPDATER_DOWNLOADER_FIRST_BLACKLIST_ALLOC         10

/* ***************************************
 *
 *             function implementation :
 *
 *****************************************/

eARUPDATER_ERROR ARUPDATER_Downloader_New(ARUPDATER_Manager_t* manager, const char *const rootFolder, ARSAL_MD5_Manager_t *md5Manager, eARUPDATER_Downloader_Platforms appPlatform, const char* const appVersion, ARUPDATER_Downloader_ShouldDownloadPlfCallback_t shouldDownloadCallback, void *downloadArg, ARUPDATER_Downloader_WillDownloadPlfCallback_t willDownloadPlfCallback, void *willDownloadPlfArg, ARUPDATER_Downloader_PlfDownloadProgressCallback_t progressCallback, void *progressArg, ARUPDATER_Downloader_PlfDownloadCompletionCallback_t completionCallback, void *completionArg)
{
    ARUPDATER_Downloader_t *downloader = NULL;
    eARUPDATER_ERROR err = ARUPDATER_OK;
    int i = 0;

    // Check parameters
    if ((manager == NULL) || (rootFolder == NULL) || (md5Manager == NULL) || (appVersion == NULL))
    {
        err = ARUPDATER_ERROR_BAD_PARAMETER;
    }

    if (err == ARUPDATER_OK)
    {
        if (manager->downloader != NULL)
        {
            err = ARUPDATER_ERROR_MANAGER_ALREADY_INITIALIZED;
        }
        else
        {
             /* Create the downloader */
            downloader = malloc (sizeof (ARUPDATER_Downloader_t));
            if (downloader == NULL)
            {
                err = ARUPDATER_ERROR_ALLOC;
            } else {
                manager->downloader = downloader;
            }
        }
    }
    /* Initialize to default values */
    if(err == ARUPDATER_OK)
    {
        int rootFolderLength = strlen(rootFolder) + 1;
        char *slash = strrchr(rootFolder, ARUPDATER_MANAGER_FOLDER_SEPARATOR[0]);
        if ((slash != NULL) && (strcmp(slash, ARUPDATER_MANAGER_FOLDER_SEPARATOR) != 0))
        {
            rootFolderLength += 1;
        }
        downloader->rootFolder = (char*) malloc(rootFolderLength);
        strcpy(downloader->rootFolder, rootFolder);

        if ((slash != NULL) && (strcmp(slash, ARUPDATER_MANAGER_FOLDER_SEPARATOR) != 0))
        {
            strcat(downloader->rootFolder, ARUPDATER_MANAGER_FOLDER_SEPARATOR);
        }

        downloader->appPlatform = appPlatform;
        downloader->appVersion = malloc(strlen(appVersion)+1);
        strcpy(downloader->appVersion, appVersion);
        downloader->variant = NULL;

        downloader->md5Manager = md5Manager;

        downloader->downloadArg = downloadArg;
        downloader->willDownloadPlfArg = willDownloadPlfArg;
        downloader->progressArg = progressArg;
        downloader->completionArg = completionArg;
        downloader->productList = NULL;
        downloader->productCount = 0;

        downloader->shouldDownloadCallback = shouldDownloadCallback;
        downloader->willDownloadPlfCallback = willDownloadPlfCallback;
        downloader->plfDownloadProgressCallback = progressCallback;
        downloader->plfDownloadCompletionCallback = completionCallback;

        downloader->isRunning = 0;
        downloader->isCanceled = 0;
        downloader->updateHasBeenChecked = 0;

        downloader->requestConnection = NULL;
        downloader->requestBlacklistConnection = NULL;
        downloader->downloadConnection = NULL;

        downloader->downloadInfos = malloc(sizeof(ARUPDATER_DownloadInformation_t*) * ARDISCOVERY_PRODUCT_MAX);
        if (downloader->downloadInfos == NULL)
        {
            err = ARUPDATER_ERROR_ALLOC;
        }
        else
        {
            for (i = 0; i < ARDISCOVERY_PRODUCT_MAX; i++)
            {
                downloader->downloadInfos[i] = NULL;
            }
        }
        manager->downloader->productList = malloc(sizeof(eARDISCOVERY_PRODUCT) * ARDISCOVERY_PRODUCT_MAX);
        if (manager->downloader->productList == NULL)
        {
            err = ARUPDATER_ERROR_ALLOC;
        }
        else
        {
            manager->downloader->productCount = ARDISCOVERY_PRODUCT_MAX;
            for (i=0; i<ARDISCOVERY_PRODUCT_MAX; i++)
            {
                manager->downloader->productList[i] = i;
            }
        }
        downloader->blacklistedVersions = calloc(ARDISCOVERY_PRODUCT_MAX, sizeof(ARUPDATER_Manager_BlacklistedFirmware_t*));
        if (downloader->blacklistedVersions == NULL)
        {
            err = ARUPDATER_ERROR_ALLOC;
        }
        else
        {
            for (i=0; i<ARDISCOVERY_PRODUCT_MAX; i++)
            {
                downloader->blacklistedVersions[i] = malloc(sizeof(ARUPDATER_Manager_BlacklistedFirmware_t));
                if (downloader->blacklistedVersions[i] != NULL)
                {
                    downloader->blacklistedVersions[i]->product = i;
                    downloader->blacklistedVersions[i]->versions = calloc(ARUPDATER_DOWNLOADER_FIRST_BLACKLIST_ALLOC, sizeof(char *));
                    downloader->blacklistedVersions[i]->nbVersionAllocated = ARUPDATER_DOWNLOADER_FIRST_BLACKLIST_ALLOC;
                    downloader->blacklistedVersions[i]->nbVersionBlacklisted = 0;
                }
            }

            // add here blacklisted version for RollingSpider
            /* Example how to add blacklisted version in app
             downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_MINIDRONE]->versions[0] = strdup("2.0.0");
            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_MINIDRONE]->nbVersionBlacklisted = 1;*/

            // add here blacklisted version for JS

            // add here blacklisted version for Bebop

            // add here blacklisted version for SkyController

            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_SKYCONTROLLER_2]->versions[0] = strdup("0.9.1");
            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_SKYCONTROLLER_2]->versions[1] = strdup("1.0.0");
            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_SKYCONTROLLER_2]->nbVersionBlacklisted = 2;

            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_EVINRUDE]->versions[0] = strdup("1.0.0");
            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_EVINRUDE]->versions[1] = strdup("1.0.2");
            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_EVINRUDE]->versions[2] = strdup("1.0.3");
            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_EVINRUDE]->nbVersionBlacklisted = 3;

            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_BEBOP_2]->versions[0] = strdup("3.4.0");
            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_BEBOP_2]->nbVersionBlacklisted = 1;

            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_MINIDRONE_DELOS3]->versions[0] = strdup("0.3.3");
            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_MINIDRONE_DELOS3]->nbVersionBlacklisted = 1;

            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_MINIDRONE_WINGX]->versions[0] = strdup("0.3.3");
            downloader->blacklistedVersions[ARDISCOVERY_PRODUCT_MINIDRONE_WINGX]->nbVersionBlacklisted = 1;
        }
    }

    if (err == ARUPDATER_OK)
    {
        int resultSys = ARSAL_Mutex_Init(&manager->downloader->requestLock);

        if (resultSys != 0)
        {
            err = ARUPDATER_ERROR_SYSTEM;
        }
    }

    if (err == ARUPDATER_OK)
    {
        int resultSys = ARSAL_Mutex_Init(&manager->downloader->requestBlacklistLock);

        if (resultSys != 0)
        {
            err = ARUPDATER_ERROR_SYSTEM;
        }
    }

    if (err == ARUPDATER_OK)
    {
        int resultSys = ARSAL_Mutex_Init(&manager->downloader->downloadLock);

        if (resultSys != 0)
        {
            err = ARUPDATER_ERROR_SYSTEM;
        }
    }

    /* delete the downloader if an error occurred */
    if (err != ARUPDATER_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_DOWNLOADER_TAG, "error: %s", ARUPDATER_Error_ToString (err));
        ARUPDATER_Downloader_Delete (manager);
    }

    return err;
}


eARUPDATER_ERROR ARUPDATER_Downloader_Delete(ARUPDATER_Manager_t *manager)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    if (manager == NULL)
    {
        error = ARUPDATER_ERROR_BAD_PARAMETER;
    }
    else
    {
        if (manager->downloader == NULL)
        {
            error = ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
        }
        else
        {
            if (manager->downloader->isRunning != 0)
            {
                error = ARUPDATER_ERROR_THREAD_PROCESSING;
            }
            else
            {
                ARSAL_Mutex_Destroy(&manager->downloader->requestLock);
                ARSAL_Mutex_Destroy(&manager->downloader->requestBlacklistLock);
                ARSAL_Mutex_Destroy(&manager->downloader->downloadLock);

                free(manager->downloader->rootFolder);

                free(manager->downloader->appVersion);

                free(manager->downloader->variant);

                int product = 0;
                for (product = 0; product < ARDISCOVERY_PRODUCT_MAX; product++)
                {
                    ARUPDATER_DownloadInformation_t *downloadInfo = manager->downloader->downloadInfos[product];
                    if (downloadInfo != NULL)
                    {
                        ARUPDATER_DownloadInformation_Delete(&downloadInfo);
                        manager->downloader->downloadInfos[product] = NULL;
                    }

                    ARUPDATER_Manager_BlacklistedFirmware_t *blacklistedVersions = manager->downloader->blacklistedVersions[product];
                    int j = 0;
                    for (j = 0; j < blacklistedVersions->nbVersionBlacklisted; j++)
                    {
                        if (blacklistedVersions->versions[j] != NULL)
                        {
                            free(blacklistedVersions->versions[j]);
                        }
                    }
                    free(blacklistedVersions->versions);
                }
                free(manager->downloader->downloadInfos);
                free(manager->downloader->blacklistedVersions);

                if (manager->downloader->productList != NULL)
                {
                    free(manager->downloader->productList);
                    manager->downloader->productList = NULL;
                }

                free(manager->downloader);
                manager->downloader = NULL;
            }
        }
    }

    return error;
}

eARUPDATER_ERROR ARUPDATER_Downloader_SetVariant(ARUPDATER_Manager_t *manager, const char* const variant)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;

    if (manager == NULL || variant == NULL || variant[0] == '\0')
    {
        error = ARUPDATER_ERROR_BAD_PARAMETER;
    }
    else if (manager->downloader == NULL)
    {
        error = ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
    }
    else
    {
        free(manager->downloader->variant);
        manager->downloader->variant = strdup(variant);
        if (manager->downloader->variant == NULL)
        {
            error = ARUPDATER_ERROR_ALLOC;
        }
    }

    return error;
}

eARUPDATER_ERROR ARUPDATER_Downloader_SetUpdatesProductList(ARUPDATER_Manager_t *manager, eARDISCOVERY_PRODUCT *productList, int productCount)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    int i;

    if (manager == NULL)
    {
        error = ARUPDATER_ERROR_BAD_PARAMETER;
    }
    else if (manager->downloader == NULL)
    {
        error = ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
    }

    if (ARUPDATER_OK == error)
    {
        if (manager->downloader->productList != NULL)
        {
            free(manager->downloader->productList);
            manager->downloader->productList= NULL;
            manager->downloader->productCount = 0;
        }

        if (productList == NULL)
        {
            manager->downloader->productList = malloc(sizeof(eARDISCOVERY_PRODUCT) * ARDISCOVERY_PRODUCT_MAX);
            if (manager->downloader->productList == NULL)
            {
                error = ARUPDATER_ERROR_ALLOC;
            }
            else
            {
                manager->downloader->productCount = ARDISCOVERY_PRODUCT_MAX;
                for (i=0; i<ARDISCOVERY_PRODUCT_MAX; i++)
                {
                    manager->downloader->productList[i] = i;
                }
            }
        }
        else
        {
            manager->downloader->productList = malloc(sizeof(eARDISCOVERY_PRODUCT) * productCount);
            if (manager->downloader->productList == NULL)
            {
                error = ARUPDATER_ERROR_ALLOC;
            }
            else
            {
                memcpy(manager->downloader->productList, productList, sizeof(eARDISCOVERY_PRODUCT) * productCount);
                manager->downloader->productCount = productCount;
            }
        }
    }

    return error;
}

int ARUPDATER_Downloader_CheckUpdatesSync(ARUPDATER_Manager_t *manager, eARUPDATER_ERROR *err)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    int nbUpdatesToDownload = 0;
    ARUPDATER_PlfVersion v;
    eARUTILS_ERROR utilsError = ARUTILS_OK;
    char *device = NULL;
    char *deviceFolder = NULL;
    char *existingPlfFilePath = NULL;
    uint32_t dataSize;
    char *dataPtr = NULL;
    char *data;
    ARSAL_Sem_t requestSem;
    char *platform = NULL;
    int ret;
    char *plfFolder = NULL;
    int productIndex = 0;

    if (manager == NULL)
    {
        error = ARUPDATER_ERROR_BAD_PARAMETER;
        goto end;
    }
    else if (manager->downloader == NULL)
    {
        error = ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
        goto end;
    }

    manager->downloader->updateHasBeenChecked = 1;

    plfFolder = malloc(strlen(manager->downloader->rootFolder) + strlen(ARUPDATER_MANAGER_PLF_FOLDER) + 1);
    if (plfFolder == NULL) {
        error = ARUPDATER_ERROR_ALLOC;
        goto end;
    }
    strcpy(plfFolder, manager->downloader->rootFolder);
    strcat(plfFolder, ARUPDATER_MANAGER_PLF_FOLDER);

    platform = ARUPDATER_Downloader_GetPlatformName(manager->downloader->appPlatform);
    if (platform == NULL) {
        error = ARUPDATER_ERROR_DOWNLOADER_PLATFORM_ERROR;
        goto end;
    }

    while ((error == ARUPDATER_OK) && (productIndex < manager->downloader->productCount) && (manager->downloader->isCanceled == 0))
    {
        // for each product, check if update is needed
        eARDISCOVERY_PRODUCT product = manager->downloader->productList[productIndex];
        uint16_t productId = ARDISCOVERY_getProductID(product);

        device = malloc(ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE);
        snprintf(device, ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE, "%04x", productId);
        char *fileName = NULL;

        // read the header of the plf file
        deviceFolder = malloc(strlen(plfFolder) + strlen(device) + strlen(ARUPDATER_MANAGER_FOLDER_SEPARATOR) + 1);
        if (!deviceFolder) {
            error = ARUPDATER_ERROR_ALLOC;
        } else {
            strcpy(deviceFolder, plfFolder);
            strcat(deviceFolder, device);
            strcat(deviceFolder, ARUPDATER_MANAGER_FOLDER_SEPARATOR);
            error = ARUPDATER_Utils_GetPlfInFolder(deviceFolder, &fileName);
        }

        if (error == ARUPDATER_OK)
        {
            // file path = deviceFolder + plfFilename + \0
            existingPlfFilePath = malloc(strlen(deviceFolder) + strlen(fileName) + 1);
            if (!existingPlfFilePath) {
                error = ARUPDATER_ERROR_ALLOC;
            } else {
                strcpy(existingPlfFilePath, deviceFolder);
                strcat(existingPlfFilePath, fileName);

                error = ARUPDATER_Utils_ReadPlfVersion(existingPlfFilePath, &v);
            }
        }
        // else if the file does not exist, force to download
        else if (error == ARUPDATER_ERROR_PLF_FILE_NOT_FOUND)
        {
            /* set version to 0.0.0 */
            v.type = ARUPDATER_PLF_TYPE_PROD;
            v.edit = 0;
            v.ver = 0;
            v.ext = 0;
            v.patch = 0;

            error = ARUPDATER_OK;

            // also check that the directory exists
            FILE *dir = fopen(plfFolder, "r");
            if (dir == NULL)
            {
                ret = mkdir(plfFolder, S_IRWXU);
                if (ret < 0 && errno != EEXIST) {
                    ret = errno;
                    ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_DOWNLOADER_TAG, "mkdir '%s' error: %s", plfFolder, strerror(ret));
                }
            }
            else
            {
                fclose(dir);
            }

            dir = fopen(deviceFolder, "r");
            if (dir == NULL)
            {
                ret = mkdir(deviceFolder, S_IRWXU);
                if (ret < 0 && errno != EEXIST) {
                    ret = errno;
                    ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_DOWNLOADER_TAG, "mkdir '%s' error: %s", deviceFolder, strerror(ret));
                }
            }
            else
            {
                fclose(dir);
            }
        }

        free(fileName);

        // init the request semaphore
        ARSAL_Mutex_Lock(&manager->downloader->requestLock);
        if (error == ARUPDATER_OK)
        {
            int resultSys = ARSAL_Sem_Init(&requestSem, 0, 0);
            if (resultSys != 0)
            {
                error = ARUPDATER_ERROR_SYSTEM;
            }
        }

        // init the connection
        if (error == ARUPDATER_OK)
        {
            manager->downloader->requestConnection = ARUTILS_Http_Connection_New(&requestSem, ARUPDATER_DOWNLOADER_SERVER_URL, 80, HTTPS_PROTOCOL_FALSE, NULL, NULL, &utilsError);
            if (utilsError != ARUTILS_OK)
            {
                ARUTILS_Http_Connection_Delete(&manager->downloader->requestConnection);
                manager->downloader->requestConnection = NULL;
                ARSAL_Sem_Destroy(&requestSem);
                error = ARUPDATER_ERROR_DOWNLOADER_ARUTILS_ERROR;
            }
        }
        ARSAL_Mutex_Unlock(&manager->downloader->requestLock);

        // request the php
        if (error == ARUPDATER_OK)
        {
            char buffer[ARUPDATER_DOWNLOADER_VERSION_BUFFER_MAX_LENGHT];
            // create the url params
            char *params = malloc(ARUPDATER_DOWNLOADER_PARAM_MAX_LENGTH);
            strcpy(params, ARUPDATER_DOWNLOADER_PRODUCT_PARAM);
            strcat(params, device);

            strcat(params, ARUPDATER_DOWNLOADER_SERIAL_PARAM);
            strcat(params, ARUPDATER_DOWNLOADER_SERIAL_DEFAULT_VALUE);

            strcat(params, ARUPDATER_DOWNLOADER_VERSION_PARAM);
            ARUPDATER_Utils_PlfVersionToString(&v, buffer, sizeof(buffer));
            strcat(params, buffer);

            strcat(params, ARUPDATER_DOWNLOADER_APP_PLATFORM_PARAM);
            strcat(params, platform);

            strcat(params, ARUPDATER_DOWNLOADER_APP_VERSION_PARAM);
            strcat(params, manager->downloader->appVersion);

            if (manager->downloader->variant != NULL) {
                strcat(params, ARUPDATER_DOWNLOADER_VARIANT_PARAM);
                strcat(params, manager->downloader->variant);
            }

            char *endUrl = malloc(strlen(ARUPDATER_DOWNLOADER_BEGIN_URL) + strlen(device) + strlen(ARUPDATER_DOWNLOADER_PHP_URL) + strlen(params) + 1);
            strcpy(endUrl, ARUPDATER_DOWNLOADER_BEGIN_URL);
            strcat(endUrl, device);
            strcat(endUrl, ARUPDATER_DOWNLOADER_PHP_URL);
            strcat(endUrl, params);

            utilsError = ARUTILS_Http_Get_WithBuffer(manager->downloader->requestConnection, endUrl, (uint8_t**)&dataPtr, &dataSize, NULL, NULL);
            if (utilsError != ARUTILS_OK)
            {
                error = ARUPDATER_ERROR_DOWNLOADER_ARUTILS_ERROR;
            }

            ARSAL_Mutex_Lock(&manager->downloader->requestLock);
            if (manager->downloader->requestConnection != NULL)
            {
                ARUTILS_Http_Connection_Delete(&manager->downloader->requestConnection);
                manager->downloader->requestConnection = NULL;
                ARSAL_Sem_Destroy(&requestSem);
            }
            ARSAL_Mutex_Unlock(&manager->downloader->requestLock);

            free(endUrl);
            endUrl = NULL;
            free(params);
            params = NULL;
        }

        // check if plf file need to be updated
        if (error == ARUPDATER_OK)
        {
            data = dataPtr;
            char *result = NULL;
            char *svg = NULL;
            result = strtok_r(data, "|", &svg);

            // if this plf is not up to date
            if(strcmp(result, ARUPDATER_DOWNLOADER_PHP_ERROR_UPDATE) == 0)
            {
                nbUpdatesToDownload++;
                char *downloadUrl = strtok_r(NULL, "|", &svg);
                char *remoteMD5 = strtok_r(NULL, "|", &svg);
                char *remoteSizeStr = strtok_r(NULL, "|", &svg);
                int remoteSize = 0;
                if (remoteSizeStr != NULL)
                {
                    remoteSize = atoi(remoteSizeStr);
                }
                char *remoteVersion = strtok_r(NULL, "\n", &svg);

                manager->downloader->downloadInfos[product] = ARUPDATER_DownloadInformation_New(downloadUrl, remoteMD5, remoteVersion, remoteSize, product, &error);
            }
            else if(strcmp(result, ARUPDATER_DOWNLOADER_PHP_ERROR_OK) == 0)
            {
                manager->downloader->downloadInfos[product] = NULL;
            }
            else if(strcmp(result, ARUPDATER_DOWNLOADER_PHP_ERROR_APP_VERSION_OUT_TO_DATE) == 0)
            {
                error = ARUPDATER_ERROR_DOWNLOADER_PHP_APP_OUT_TO_DATE_ERROR;
            }
            else
            {
                error = ARUPDATER_ERROR_DOWNLOADER_PHP_ERROR;
            }
        }

        if (deviceFolder != NULL)
        {
            free(deviceFolder);
            deviceFolder = NULL;
        }
        if (existingPlfFilePath != NULL)
        {
            free(existingPlfFilePath);
            existingPlfFilePath = NULL;
        }
        if (device != NULL)
        {
            free(device);
            device = NULL;
        }
        if (dataPtr != NULL)
        {
            free(dataPtr);
            dataPtr = NULL;
        }

        productIndex++;
    }

end:
    free(plfFolder);
    plfFolder = NULL;

    if (err != NULL)
    {
        *err = error;
    }

    return nbUpdatesToDownload;
}

void* ARUPDATER_Downloader_CheckUpdatesAsync(void *managerArg)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    int nbUpdatesToDownload = 0;

    ARUPDATER_Manager_t *manager = NULL;
    if (managerArg != NULL)
    {
        manager = (ARUPDATER_Manager_t*)managerArg;
    }
    else
    {
        error = ARUPDATER_ERROR_BAD_PARAMETER;
    }

    if ((manager == NULL) || (manager->downloader == NULL))
    {
        error = ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
    }

    if (ARUPDATER_OK == error)
    {
        nbUpdatesToDownload = ARUPDATER_Downloader_CheckUpdatesSync(manager, &error);
    }

    if ((manager != NULL) && (manager->downloader != NULL) && (manager->downloader->shouldDownloadCallback != NULL))
    {
        manager->downloader->shouldDownloadCallback(manager->downloader->downloadArg, nbUpdatesToDownload, error);
    }

    return (void*)error;
}

void* ARUPDATER_Downloader_ThreadRun(void *managerArg)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    int resultSys = -1;
    int nbDownloadsToDo = 0;
    int productIndex = 0;
    eARUTILS_ERROR utilsError = ARUTILS_OK;
    ARSAL_Sem_t dlSem;
    eARDISCOVERY_PRODUCT product;
    uint16_t productId = 0;
    ARUPDATER_DownloadInformation_t *downloadInfo = NULL;
    const char *downloadUrl = NULL;
    char *remoteMD5 = NULL;
    char *remoteVersion = NULL;
    char *downloadEndUrl = NULL;
    char downloadServer[512];
    char downloadedFinalFilePath[512];
    char *downloadedFileName = NULL;
    char deviceFolder[512];
    char downloadedFilePath[512];
    const char *urlWithoutHttpHeader = NULL;
    const char delimiter = '/';
    int serverLength = 0;
    eARSAL_ERROR arsalError;

    ARUPDATER_Manager_t *manager = (ARUPDATER_Manager_t*)managerArg;
    if ((manager == NULL) ||
        (manager->downloader == NULL))
        return NULL;

    manager->downloader->isRunning = 1;

    // if the check has not already been done, do it
    if (!manager->downloader->updateHasBeenChecked) {
        nbDownloadsToDo = ARUPDATER_Downloader_CheckUpdatesSync(manager, &error);
        if (nbDownloadsToDo <= 0)
            goto end;
    }

    while ((productIndex < manager->downloader->productCount) && (!manager->downloader->isCanceled)) {
        /* for each product, check if update is needed */
        product = manager->downloader->productList[productIndex];
        productId = ARDISCOVERY_getProductID(product);

        downloadInfo = manager->downloader->downloadInfos[product];
        if (error == ARUPDATER_OK && downloadInfo != NULL)
        {
            downloadUrl = downloadInfo->downloadUrl;
            remoteMD5 = downloadInfo->md5Expected;
            remoteVersion = downloadInfo->plfVersion;

            if (manager->downloader->willDownloadPlfCallback != NULL)
                manager->downloader->willDownloadPlfCallback(manager->downloader->completionArg, product, remoteVersion);

            downloadedFileName = strrchr(downloadUrl, ARUPDATER_MANAGER_FOLDER_SEPARATOR[0]);
            if(downloadedFileName == NULL || strlen(downloadedFileName) == 0)
                break;

            downloadedFileName = &downloadedFileName[1];

            snprintf(deviceFolder, sizeof(deviceFolder), "%s%s%04x%s",
                    manager->downloader->rootFolder,
                    ARUPDATER_MANAGER_PLF_FOLDER,
                    productId,
                    ARUPDATER_MANAGER_FOLDER_SEPARATOR);

            snprintf(downloadedFilePath, sizeof(downloadedFilePath), "%s%s%s%s",
                    deviceFolder,
                    ARUPDATER_DOWNLOADER_DOWNLOADED_FILE_PREFIX,
                    downloadedFileName,
                    ARUPDATER_DOWNLOADER_DOWNLOADED_FILE_SUFFIX);

            snprintf(downloadedFinalFilePath, sizeof(downloadedFinalFilePath), "%s%s",
                    deviceFolder,
                    downloadedFileName);

            /* explode the download url into server and endUrl */
            if (strncmp(downloadUrl, ARUPDATER_DOWNLOADER_HTTP_HEADER, strlen(ARUPDATER_DOWNLOADER_HTTP_HEADER)) != 0) {
                error = ARUPDATER_ERROR_DOWNLOADER_PHP_ERROR;
                break;
            }

            /* construct the url */
            urlWithoutHttpHeader = downloadUrl + strlen(ARUPDATER_DOWNLOADER_HTTP_HEADER);

            downloadEndUrl = strchr(urlWithoutHttpHeader, delimiter);
            serverLength = strlen(urlWithoutHttpHeader) - strlen(downloadEndUrl);
            snprintf(downloadServer, serverLength + 1, "%s", urlWithoutHttpHeader);

            ARSAL_Mutex_Lock(&manager->downloader->downloadLock);
            /* init the request semaphore */
            resultSys = ARSAL_Sem_Init(&dlSem, 0, 0);
            if (resultSys != 0) {
                error = ARUPDATER_ERROR_SYSTEM;
                ARSAL_Mutex_Unlock(&manager->downloader->downloadLock);
                break;
            }

            manager->downloader->downloadConnection = ARUTILS_Http_Connection_New(&dlSem, downloadServer, 80, HTTPS_PROTOCOL_FALSE, NULL, NULL, &utilsError);
            if (utilsError != ARUTILS_OK) {
                ARUTILS_Http_Connection_Delete(&manager->downloader->downloadConnection);
                manager->downloader->downloadConnection = NULL;
                error = ARUPDATER_ERROR_DOWNLOADER_ARUTILS_ERROR;
                ARSAL_Sem_Destroy(&dlSem);
                ARSAL_Mutex_Unlock(&manager->downloader->downloadLock);
                break;
            }

            ARSAL_Mutex_Unlock(&manager->downloader->downloadLock);

            /* download the file */
            if (!manager->downloader->isCanceled) {
                utilsError = ARUTILS_Http_Get(manager->downloader->downloadConnection, downloadEndUrl, downloadedFilePath, manager->downloader->plfDownloadProgressCallback, manager->downloader->progressArg);
                if (utilsError != ARUTILS_OK) {
                    error = ARUPDATER_ERROR_DOWNLOADER_ARUTILS_ERROR;

                    /* Delete Connection */
                    ARSAL_Mutex_Lock(&manager->downloader->downloadLock);
                    if (manager->downloader->downloadConnection != NULL) {
                        ARUTILS_Http_Connection_Delete(&manager->downloader->downloadConnection);
                        manager->downloader->downloadConnection = NULL;
                        ARSAL_Sem_Destroy(&dlSem);
                    }
                    ARSAL_Mutex_Unlock(&manager->downloader->downloadLock);
                    break;
                }
            }

            /* Delete Connection */
            ARSAL_Mutex_Lock(&manager->downloader->downloadLock);
            if (manager->downloader->downloadConnection != NULL) {
                ARUTILS_Http_Connection_Delete(&manager->downloader->downloadConnection);
                manager->downloader->downloadConnection = NULL;
            }
            ARSAL_Sem_Destroy(&dlSem);
            ARSAL_Mutex_Unlock(&manager->downloader->downloadLock);

            /* check md5 match */
            arsalError = ARSAL_MD5_Manager_Check(manager->downloader->md5Manager, downloadedFilePath, remoteMD5);
            if (ARSAL_OK != arsalError) {
                /* delete the downloaded file if md5 don't match */
                unlink(downloadedFilePath);
                error = ARUPDATER_ERROR_DOWNLOADER_MD5_DONT_MATCH;
                break;
            }

            if (rename(downloadedFilePath, downloadedFinalFilePath) != 0) {
                error = ARUPDATER_ERROR_DOWNLOADER_RENAME_FILE;
                break;
            }
        }

        productIndex++;
    }

end:
    /* delete the content of the downloadInfos */
    manager->downloader->updateHasBeenChecked = 0;
    for (productIndex = 0; productIndex < ARDISCOVERY_PRODUCT_MAX; productIndex++) {
        downloadInfo = manager->downloader->downloadInfos[productIndex];
        if (downloadInfo != NULL) {
            ARUPDATER_DownloadInformation_Delete(&downloadInfo);
            manager->downloader->downloadInfos[productIndex] = NULL;
        }
    }

    if (error != ARUPDATER_OK)
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_DOWNLOADER_TAG, "error: %s", ARUPDATER_Error_ToString (error));

    manager->downloader->isRunning = 0;

    if (manager->downloader->plfDownloadCompletionCallback)
        manager->downloader->plfDownloadCompletionCallback(manager->downloader->completionArg, error);

    return (void*)error;
}

eARUPDATER_ERROR ARUPDATER_Downloader_CancelThread(ARUPDATER_Manager_t *manager)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;

    if (manager == NULL)
    {
        error = ARUPDATER_ERROR_BAD_PARAMETER;
    }

    if ((error == ARUPDATER_OK) && (manager->downloader == NULL))
    {
        error = ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
    }

    if (error == ARUPDATER_OK)
    {
        manager->downloader->isCanceled = 1;

        ARSAL_Mutex_Lock(&manager->downloader->requestLock);
        if (manager->downloader->requestConnection != NULL)
        {
            ARUTILS_Http_Connection_Cancel(manager->downloader->requestConnection);
        }
        ARSAL_Mutex_Unlock(&manager->downloader->requestLock);

        ARSAL_Mutex_Lock(&manager->downloader->requestBlacklistLock);
        if (manager->downloader->requestBlacklistConnection != NULL)
        {
            ARUTILS_Http_Connection_Cancel(manager->downloader->requestBlacklistConnection);
        }
        ARSAL_Mutex_Unlock(&manager->downloader->requestBlacklistLock);

        ARSAL_Mutex_Lock(&manager->downloader->downloadLock);
        if (manager->downloader->downloadConnection != NULL)
        {
            ARUTILS_Http_Connection_Cancel(manager->downloader->downloadConnection);
        }
        ARSAL_Mutex_Unlock(&manager->downloader->downloadLock);

    }

    return error;
}

int ARUPDATER_Downloader_ThreadIsRunning(ARUPDATER_Manager_t* manager, eARUPDATER_ERROR *error)
{
    eARUPDATER_ERROR err = ARUPDATER_OK;
    int isRunning = 0;

    if (manager == NULL)
    {
        err = ARUPDATER_ERROR_BAD_PARAMETER;
    }

    if ((err == ARUPDATER_OK) && (manager->downloader == NULL))
    {
        err = ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
    }

    if (err == ARUPDATER_OK)
    {
        isRunning = manager->downloader->isRunning;
    }

    if (error != NULL)
    {
        *error = err;
    }

    return isRunning;
}

char *ARUPDATER_Downloader_GetPlatformName(eARUPDATER_Downloader_Platforms platform)
{
    char *toReturn = NULL;
    switch (platform)
    {
        case ARUPDATER_DOWNLOADER_ANDROID_PLATFORM :
            toReturn = ARUPDATER_DOWNLOADER_ANDROID_PLATFORM_NAME;
            break;
        case ARUPDATER_DOWNLOADER_IOS_PLATFORM :
            toReturn = ARUPDATER_DOWNLOADER_IOS_PLATFORM_NAME;
            break;
        default:
            break;
    }

    return toReturn;
}

eARUPDATER_ERROR ARUPDATER_Downloader_GetBlacklistedFirmwareVersionsSync(ARUPDATER_Manager_t* manager, int alsoCheckRemote, ARUPDATER_Manager_BlacklistedFirmware_t ***blacklistedFirmwares)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    ARSAL_Sem_t requestSem;
    char *platform = NULL;
    uint32_t dataSize;
    char *dataPtr = NULL;
    char *data;
    eARUTILS_ERROR utilsError = ARUTILS_OK;
    json_object *jsonObj = NULL;
    array_list *blacklistedRemoteList = NULL;
    char *device = NULL;

    if (manager == NULL)
    {
        return ARUPDATER_ERROR_BAD_PARAMETER;
    }
    else if (manager->downloader == NULL)
    {
        return ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
    }

    if (alsoCheckRemote != 0)
    {
        if (error == ARUPDATER_OK)
        {
            platform = ARUPDATER_Downloader_GetPlatformName(manager->downloader->appPlatform);
            if (platform == NULL)
            {
                error = ARUPDATER_ERROR_DOWNLOADER_PLATFORM_ERROR;
            }
        }

        // init the request semaphore
        ARSAL_Mutex_Lock(&manager->downloader->requestBlacklistLock);
        if (error == ARUPDATER_OK)
        {
            int resultSys = ARSAL_Sem_Init(&requestSem, 0, 0);
            if (resultSys != 0)
            {
                error = ARUPDATER_ERROR_SYSTEM;
            }
        }
        // init the connection
        if (error == ARUPDATER_OK)
        {
            manager->downloader->requestBlacklistConnection = ARUTILS_Http_Connection_New(&requestSem, ARUPDATER_DOWNLOADER_SERVER_URL, 80, HTTPS_PROTOCOL_FALSE, NULL, NULL, &utilsError);
            if (utilsError != ARUTILS_OK)
            {
                ARUTILS_Http_Connection_Delete(&manager->downloader->requestBlacklistConnection);
                manager->downloader->requestBlacklistConnection = NULL;
                ARSAL_Sem_Destroy(&requestSem);
                error = ARUPDATER_ERROR_DOWNLOADER_ARUTILS_ERROR;
            }
        }
        ARSAL_Mutex_Unlock(&manager->downloader->requestBlacklistLock);

        // request the php
        if (error == ARUPDATER_OK)
        {
            // create the url params
            char *params = malloc(ARUPDATER_DOWNLOADER_PARAM_MAX_LENGTH);

            strcpy(params, ARUPDATER_DOWNLOADER_APP_PLATFORM_PARAM_BEGIN);
            strcat(params, platform);

            strcat(params, ARUPDATER_DOWNLOADER_APP_VERSION_PARAM);
            strcat(params, manager->downloader->appVersion);

            if (manager->downloader->variant != NULL) {
                strcat(params, ARUPDATER_DOWNLOADER_VARIANT_PARAM);
                strcat(params, manager->downloader->variant);
            }

            char *endUrl = malloc(strlen(ARUPDATER_DOWNLOADER_BEGIN_URL) + strlen(ARUPDATER_DOWNLOADER_PHP_BLACKLIST_FIRM_URL) + strlen(params) + 1);
            strcpy(endUrl, ARUPDATER_DOWNLOADER_BEGIN_URL);
            strcat(endUrl, ARUPDATER_DOWNLOADER_PHP_BLACKLIST_FIRM_URL);
            strcat(endUrl, params);

            utilsError = ARUTILS_Http_Get_WithBuffer(manager->downloader->requestBlacklistConnection, endUrl, (uint8_t**)&dataPtr, &dataSize, NULL, NULL);
            if (utilsError != ARUTILS_OK)
            {
                ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_DOWNLOADER_TAG, "Error : %d", utilsError);
                error = ARUPDATER_ERROR_DOWNLOADER_ARUTILS_ERROR;
            }
            ARSAL_Mutex_Lock(&manager->downloader->requestBlacklistLock);
            if (manager->downloader->requestBlacklistConnection != NULL)
            {
                ARUTILS_Http_Connection_Delete(&manager->downloader->requestBlacklistConnection);
                manager->downloader->requestBlacklistConnection = NULL;
                ARSAL_Sem_Destroy(&requestSem);
            }
            ARSAL_Mutex_Unlock(&manager->downloader->requestBlacklistLock);
            free(endUrl);
            endUrl = NULL;
            free(params);
            params = NULL;
        }

        // use blacklist info from server
        if (error == ARUPDATER_OK)
        {
            data = dataPtr;
            char *result = NULL;
            char *svg = NULL;
            result = strtok_r(data, "|", &svg);

            // if the server has no error
            if(strcmp(result, ARUPDATER_DOWNLOADER_PHP_ERROR_OK) == 0)
            {
                char *jsonAsStr = strtok_r(NULL, "|", &svg);
                if (jsonAsStr != NULL)
                {
                    jsonObj = json_tokener_parse(jsonAsStr);
                }

                if ((jsonObj == NULL) || is_error(jsonObj))
                {
                    jsonObj = NULL;
                    error = ARUPDATER_ERROR_DOWNLOADER_PHP_ERROR;
                    ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_DOWNLOADER_TAG, "Blacklist json is null");
                }
            }
            else
            {
                error = ARUPDATER_ERROR_DOWNLOADER_PHP_ERROR;
            }
        }

        // add all blacklisted versions from server to the blacklisted versions
        if (error == ARUPDATER_OK)
        {
            // for all products
            int i = 0;
            for (i = 0; (error == ARUPDATER_OK) && (i < ARDISCOVERY_PRODUCT_MAX); i++)
            {
                json_object *productJsonObj = NULL;
                uint16_t productId = ARDISCOVERY_getProductID(manager->downloader->blacklistedVersions[i]->product);

                device = malloc(ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE);
                snprintf(device, ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE, "%04x", productId);

                if (json_object_is_type(jsonObj, json_type_object))
                {
                    json_object_object_get_ex (jsonObj, device, &productJsonObj);
                }
                if ((productJsonObj != NULL) && !is_error(productJsonObj) && (json_object_is_type(productJsonObj, json_type_array)))
                {
                    blacklistedRemoteList = json_object_get_array(productJsonObj);
                }
                else
                {
                    blacklistedRemoteList = NULL;
                }

                // if it exists blacklisted version for this product
                if ((blacklistedRemoteList != NULL) && !is_error(blacklistedRemoteList))
                {
                    // add each blacklisted version to the existing list
                    int j = 0;
                    for (j = 0; (error == ARUPDATER_OK) && (j < blacklistedRemoteList->length); j++)
                    {
                        json_object *valueJsonObj = array_list_get_idx (blacklistedRemoteList, j);
                        if ((valueJsonObj != NULL) && !is_error(valueJsonObj) && (json_object_is_type(valueJsonObj, json_type_string)))
                        {
                            const char *blacklistedVersion = json_object_get_string(valueJsonObj);
                            // add it to the existing blacklist
                            // if versions has not enough allocated place
                            if (manager->downloader->blacklistedVersions[i]->nbVersionBlacklisted >= manager->downloader->blacklistedVersions[i]->nbVersionAllocated)
                            {
                                char** oldVersions = manager->downloader->blacklistedVersions[i]->versions;
                                char** newVersions = realloc(manager->downloader->blacklistedVersions[i]->versions, ARUPDATER_DOWNLOADER_FIRST_BLACKLIST_ALLOC * sizeof(char*));
                                if (newVersions != NULL)
                                {
                                    manager->downloader->blacklistedVersions[i]->versions = newVersions;
                                    manager->downloader->blacklistedVersions[i]->nbVersionAllocated += ARUPDATER_DOWNLOADER_FIRST_BLACKLIST_ALLOC;
                                }
                                else
                                {
                                    manager->downloader->blacklistedVersions[i]->versions = oldVersions;
                                    error = ARUPDATER_ERROR_ALLOC;
                                }
                            }

                            // recheck if we have enough place
                            if ((error == ARUPDATER_OK) && (manager->downloader->blacklistedVersions[i]->nbVersionBlacklisted <= manager->downloader->blacklistedVersions[i]->nbVersionAllocated))
                            {
                                manager->downloader->blacklistedVersions[i]->versions[manager->downloader->blacklistedVersions[i]->nbVersionBlacklisted] = strdup(blacklistedVersion);
                                manager->downloader->blacklistedVersions[i]->nbVersionBlacklisted++;
                            }
                        }
                    }
                }

                if (device != NULL)
                {
                    free(device);
                    device = NULL;
                }
            }
        }
    }

    if (jsonObj != NULL)
    {
        json_object_put(jsonObj);
    }

    if (manager && manager->downloader && blacklistedFirmwares)
        *blacklistedFirmwares = manager->downloader->blacklistedVersions;

    return error;
}

int ARUPDATER_Downloader_GetUpdatesInfoSync(ARUPDATER_Manager_t *manager, eARUPDATER_ERROR *err, ARUPDATER_DownloadInformation_t*** informations)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    int nbUpdatesToDownload = 0;
    if (manager == NULL)
    {
        error = ARUPDATER_ERROR_BAD_PARAMETER;
    }
    else if (manager->downloader == NULL)
    {
        error = ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
    }

    if (ARUPDATER_OK == error)
    {
        manager->downloader->updateHasBeenChecked = 1;
    }
    int version = 0;
    int edit = 0;
    int ext = 0;
    eARUTILS_ERROR utilsError = ARUTILS_OK;
    char *device = NULL;
    uint32_t dataSize;
    char *dataPtr = NULL;
    char *data;
    ARSAL_Sem_t requestSem;
    char *platform = NULL;

    if (error == ARUPDATER_OK)
    {
        platform = ARUPDATER_Downloader_GetPlatformName(manager->downloader->appPlatform);
        if (platform == NULL)
        {
            error = ARUPDATER_ERROR_DOWNLOADER_PLATFORM_ERROR;
        }
    }
    //int product = 0;
    int productIndex = 0;
    while ((error == ARUPDATER_OK) && (productIndex < manager->downloader->productCount))
    {
        // for each product, check if update is needed
        eARDISCOVERY_PRODUCT product = manager->downloader->productList[productIndex];
    //while ((error == ARUPDATER_OK) && (product < ARDISCOVERY_PRODUCT_MAX))
    //{
        // for each product, check if update is needed
        uint16_t productId = ARDISCOVERY_getProductID(product);

        device = malloc(ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE);
        snprintf(device, ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE, "%04x", productId);

        // init the request semaphore
        ARSAL_Mutex_Lock(&manager->downloader->requestLock);
        if (error == ARUPDATER_OK)
        {
            int resultSys = ARSAL_Sem_Init(&requestSem, 0, 0);
            if (resultSys != 0)
            {
                error = ARUPDATER_ERROR_SYSTEM;
            }
        }
        // init the connection
        if (error == ARUPDATER_OK)
        {
            manager->downloader->requestConnection = ARUTILS_Http_Connection_New(&requestSem, ARUPDATER_DOWNLOADER_SERVER_URL, 80, HTTPS_PROTOCOL_FALSE, NULL, NULL, &utilsError);
            if (utilsError != ARUTILS_OK)
            {
                ARUTILS_Http_Connection_Delete(&manager->downloader->requestConnection);
                manager->downloader->requestConnection = NULL;
                ARSAL_Sem_Destroy(&requestSem);
                error = ARUPDATER_ERROR_DOWNLOADER_ARUTILS_ERROR;
            }
        }
        ARSAL_Mutex_Unlock(&manager->downloader->requestLock);
        // request the php
        if (error == ARUPDATER_OK)
        {
            char buffer[ARUPDATER_DOWNLOADER_VERSION_BUFFER_MAX_LENGHT];
            // create the url params
            char *params = malloc(ARUPDATER_DOWNLOADER_PARAM_MAX_LENGTH);
            strcpy(params, ARUPDATER_DOWNLOADER_PRODUCT_PARAM);
            strcat(params, device);

            strcat(params, ARUPDATER_DOWNLOADER_SERIAL_PARAM);
            strcat(params, ARUPDATER_DOWNLOADER_SERIAL_DEFAULT_VALUE);

            strcat(params, ARUPDATER_DOWNLOADER_VERSION_PARAM);
            sprintf(buffer,"%i",version);
            strncat(params, buffer, strlen(buffer));
            strcat(params, ARUPDATER_DOWNLOADER_VERSION_SEPARATOR);
            sprintf(buffer,"%i",edit);
            strncat(params, buffer, strlen(buffer));
            strcat(params, ARUPDATER_DOWNLOADER_VERSION_SEPARATOR);
            sprintf(buffer,"%i",ext);
            strncat(params, buffer, strlen(buffer));

            strcat(params, ARUPDATER_DOWNLOADER_APP_PLATFORM_PARAM);
            strcat(params, platform);

            strcat(params, ARUPDATER_DOWNLOADER_APP_VERSION_PARAM);
            strcat(params, manager->downloader->appVersion);

            if (manager->downloader->variant != NULL) {
                strcat(params, ARUPDATER_DOWNLOADER_VARIANT_PARAM);
                strcat(params, manager->downloader->variant);
            }

            char *endUrl = malloc(strlen(ARUPDATER_DOWNLOADER_BEGIN_URL) + strlen(device) + strlen(ARUPDATER_DOWNLOADER_PHP_URL) + strlen(params) + 1);
            strcpy(endUrl, ARUPDATER_DOWNLOADER_BEGIN_URL);
            strcat(endUrl, device);
            strcat(endUrl, ARUPDATER_DOWNLOADER_PHP_URL);
            strcat(endUrl, params);
            ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARUPDATER_DOWNLOADER_TAG, "%s", endUrl);
            utilsError = ARUTILS_Http_Get_WithBuffer(manager->downloader->requestConnection, endUrl, (uint8_t**)&dataPtr, &dataSize, NULL, NULL);
            if (utilsError != ARUTILS_OK)
            {
                ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARUPDATER_DOWNLOADER_TAG, "%d", utilsError);
                error = ARUPDATER_ERROR_DOWNLOADER_ARUTILS_ERROR;
            }

            ARSAL_Mutex_Lock(&manager->downloader->requestLock);
            if (manager->downloader->requestConnection != NULL)
            {
                ARUTILS_Http_Connection_Delete(&manager->downloader->requestConnection);
                manager->downloader->requestConnection = NULL;
                ARSAL_Sem_Destroy(&requestSem);
            }
            ARSAL_Mutex_Unlock(&manager->downloader->requestLock);
            free(endUrl);
            endUrl = NULL;
            free(params);
            params = NULL;
        }

        // check if plf file need to be updated
        if (error == ARUPDATER_OK)
        {
            data = dataPtr;
            char *result = NULL;
            char *svg = NULL;
            result = strtok_r(data, "|", &svg);

            // if this plf is not up to date
            if(strcmp(result, ARUPDATER_DOWNLOADER_PHP_ERROR_UPDATE) == 0)
            {
                nbUpdatesToDownload++;
                char *downloadUrl = strtok_r(NULL, "|", &svg);
                char *remoteMD5 = strtok_r(NULL, "|", &svg);
                char *remoteSizeStr = strtok_r(NULL, "|", &svg);
                int remoteSize = 0;
                if (remoteSizeStr != NULL)
                {
                    remoteSize = atoi(remoteSizeStr);
                }
                char *remoteVersion = strtok_r(NULL, "\n", &svg);
                manager->downloader->downloadInfos[productIndex] = ARUPDATER_DownloadInformation_New(downloadUrl, remoteMD5, remoteVersion, remoteSize, product, &error);
            }
            else if(strcmp(result, ARUPDATER_DOWNLOADER_PHP_ERROR_OK) == 0)
            {
                manager->downloader->downloadInfos[productIndex] = NULL;
            }
            else if(strcmp(result, ARUPDATER_DOWNLOADER_PHP_ERROR_APP_VERSION_OUT_TO_DATE) == 0)
            {
                error = ARUPDATER_ERROR_DOWNLOADER_PHP_APP_OUT_TO_DATE_ERROR;
            }
            else
            {
                error = ARUPDATER_ERROR_DOWNLOADER_PHP_ERROR;
            }
        }
        if (device != NULL)
        {
            free(device);
            device = NULL;
        }

        productIndex++;
    }

    if (err != NULL)
    {
        *err = error;
    }

    if (manager && manager->downloader && informations)
        *informations = manager->downloader->downloadInfos;

    return  productIndex;
}
