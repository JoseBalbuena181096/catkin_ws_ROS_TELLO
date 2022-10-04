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
 * @file ARDATATRANSFER_MediasManager.c
 * @brief libARDataTransfer MediasManager c file.
 * @date 19/12/2013
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
#include <libARUtils/ARUTILS_Ftp.h>
#include <libARUtils/ARUTILS_FileSystem.h>
#include <libARDiscovery/ARDISCOVERY_Discovery.h>

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

#define ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG                "MediasDownloader"

#define ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_DCIM           "DCIM"
#define ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_META           ".META/thumb"
#define ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_MEDIA          "media"
#define ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_THUMB          "thumb"

#define ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_JPG            "jpg"
#define ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_MP4            "mp4"
#define ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_MOV            "mov"
#define ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_JPG_CAP        "JPG"
#define ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_MP4_CAP        "MP4"
#define ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_MOV_CAP        "MOV"

/*****************************************
 *
 *             Public implementation:
 *
 *****************************************/

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_New(ARDATATRANSFER_Manager_t *manager, ARUTILS_Manager_t *ftpListManager, ARUTILS_Manager_t *ftpQueueManager, const char *remoteDirectory, const char *localDirectory)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    int resultSys = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if ((manager == NULL) || (ftpListManager == NULL) || (ftpQueueManager == NULL))
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK)
    {
        if (manager->mediasDownloader != NULL)
        {
            result = ARDATATRANSFER_ERROR_ALREADY_INITIALIZED;
        }
        else
        {
            manager->mediasDownloader = (ARDATATRANSFER_MediasDownloader_t *)calloc(1, sizeof(ARDATATRANSFER_MediasDownloader_t));

            if (manager->mediasDownloader == NULL)
            {
                result = ARDATATRANSFER_ERROR_ALLOC;
            }
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        resultSys = ARSAL_Sem_Init(&manager->mediasDownloader->queueSem, 0, 0);

        if (resultSys != 0)
        {
            result = ARDATATRANSFER_ERROR_SYSTEM;
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        resultSys = ARSAL_Sem_Init(&manager->mediasDownloader->threadSem, 0, 0);

        if (resultSys != 0)
        {
            result = ARDATATRANSFER_ERROR_SYSTEM;
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        resultSys = ARSAL_Mutex_Init(&manager->mediasDownloader->mediasLock);

        if (resultSys != 0)
        {
            result = ARDATATRANSFER_ERROR_SYSTEM;
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        manager->mediasDownloader->medias.medias = NULL;
        manager->mediasDownloader->medias.count = 0;
        manager->mediasDownloader->ftpListManager = ftpListManager;
        manager->mediasDownloader->ftpQueueManager = ftpQueueManager;
    }

    if (result == ARDATATRANSFER_OK)
    {
        ARDATATRANSFER_MediasQueue_New(&manager->mediasDownloader->queue);
    }

    if (result == ARDATATRANSFER_OK)
    {
        manager->mediasDownloader->isRunning = 0;
        manager->mediasDownloader->isCanceled = 0;

        result = ARDATATRANSFER_MediasDownloader_Initialize(manager, ftpListManager, ftpQueueManager, remoteDirectory, localDirectory);
    }

    if (result != ARDATATRANSFER_OK && result != ARDATATRANSFER_ERROR_ALREADY_INITIALIZED)
    {
        ARDATATRANSFER_MediasDownloader_Delete(manager);
    }

    return result;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_Delete(ARDATATRANSFER_Manager_t *manager)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if (manager == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }
    else
    {
        if (manager->mediasDownloader == NULL)
        {
            result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
        }
        else
        {
            if (manager->mediasDownloader->isRunning != 0)
            {
                result = ARDATATRANSFER_ERROR_THREAD_PROCESSING;
            }
            else
            {
                ARDATATRANSFER_MediasDownloader_Clear(manager);

                ARSAL_Sem_Destroy(&manager->mediasDownloader->queueSem);
                ARSAL_Sem_Destroy(&manager->mediasDownloader->threadSem);

                ARDATATRANSFER_MediasQueue_Delete(&manager->mediasDownloader->queue);

                ARSAL_Mutex_Destroy(&manager->mediasDownloader->mediasLock);
                ARDATATRANSFER_MediasDownloader_FreeMediaList(&manager->mediasDownloader->medias);

                free(manager->mediasDownloader);
                manager->mediasDownloader = NULL;
            }
        }
    }

    return result;
}

int ARDATATRANSFER_MediasDownloader_GetAvailableMediasSync(ARDATATRANSFER_Manager_t *manager, int withThumbnail, eARDATATRANSFER_ERROR *error)
{
    char remotePath[ARUTILS_FTP_MAX_PATH_SIZE];
    char thumbPath[ARUTILS_FTP_MAX_PATH_SIZE];
    char remoteProduct[ARUTILS_FTP_MAX_PATH_SIZE];
    char productPathName[ARUTILS_FTP_MAX_PATH_SIZE];
    char *productFtpList = NULL;
    uint32_t productFtpListLen = 0;
    char *mediaFtpList = NULL;
    uint32_t mediaFtpListLen = 0;
    char *dcimFtpList = NULL;
    uint32_t dcimFtpListLen = 0;
    char *metaThumbList = NULL;
    uint32_t metaThumbListLen = 0;
    const char *nextDcim = NULL;
    const char *nextProduct = NULL;
    const char *nextMedia = NULL;
    const char *lineItem;
    int lineSize;
    const char *dirName;
    const char *fileName;
    const char *thumbName;
    int product;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR resultUtils = ARUTILS_OK;
    int count = 0;
    int hasDCIM = 0;

    if (manager == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if ((result == ARDATATRANSFER_OK) && (manager->mediasDownloader == NULL))
    {
        result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
    }

    if (result == ARDATATRANSFER_OK)
    {
        ARSAL_Mutex_Lock(&manager->mediasDownloader->mediasLock);

        ARDATATRANSFER_MediasDownloader_FreeMediaList(&manager->mediasDownloader->medias);

        if ((result == ARDATATRANSFER_OK)
            && ((manager->mediasDownloader->medias.medias != NULL) || (manager->mediasDownloader->medias.count != 0)))
        {
            result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
        }

        if (result == ARDATATRANSFER_OK)
        {
            strncpy(remotePath, manager->mediasDownloader->remoteDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
            remotePath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';

            resultUtils = ARUTILS_Manager_Ftp_List(manager->mediasDownloader->ftpListManager, remotePath, &productFtpList, &productFtpListLen);

            if (resultUtils != ARUTILS_OK)
            {
                result = ARDATATRANSFER_ERROR_FTP;
            }
        }

        /* Search for medias in DCIM, by looking at the .META/thumb/ entries */
        if (result == ARDATATRANSFER_OK)
        {
            resultUtils = ARUTILS_Manager_Ftp_Connection_IsCanceled(manager->mediasDownloader->ftpListManager);
            if (resultUtils != ARUTILS_OK)
            {
                result = ARDATATRANSFER_ERROR_CANCELED;
                goto end_search_dcim;
            }

            char lineDataDcim[ARUTILS_FTP_MAX_PATH_SIZE];
            // Find DCIM directory
            nextProduct = NULL;
            fileName = ARUTILS_Ftp_List_GetNextItem(productFtpList, &nextProduct, ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_DCIM, 1, NULL, NULL, lineDataDcim, ARUTILS_FTP_MAX_PATH_SIZE);
            if ((fileName != NULL) && strcmp(fileName, ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_DCIM) == 0)
            {
                // We have it.
                hasDCIM = 1;
                //First, list the thumbnails, we will need them for every file
                strncpy(thumbPath, manager->mediasDownloader->remoteDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
                thumbPath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
                strncat(thumbPath, "/" ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_META "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(thumbPath) - 1);
                if (ARUTILS_Manager_Ftp_List(manager->mediasDownloader->ftpListManager, thumbPath, &metaThumbList, &metaThumbListLen) != ARUTILS_OK)
                {
                    result = ARDATATRANSFER_ERROR_FTP;
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "Unable to list thumbnails");
                    goto end_search_dcim;
                }

                resultUtils = ARUTILS_Manager_Ftp_Connection_IsCanceled(manager->mediasDownloader->ftpListManager);
                if (resultUtils != ARUTILS_OK)
                {
                    result = ARDATATRANSFER_ERROR_CANCELED;
                    goto end_search_dcim;
                }

                // Then look for all DCIM subdirectories
                strncpy(remotePath, manager->mediasDownloader->remoteDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
                remotePath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
                strncat(remotePath, "/" ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_DCIM "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(remotePath) - 1);
                if (ARUTILS_Manager_Ftp_List(manager->mediasDownloader->ftpListManager, remotePath, &dcimFtpList, &dcimFtpListLen) != ARUTILS_OK)
                {
                    result = ARDATATRANSFER_ERROR_FTP;
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "Unable to list DCIM");
                    goto end_search_dcim;
                }

                resultUtils = ARUTILS_Manager_Ftp_Connection_IsCanceled(manager->mediasDownloader->ftpListManager);
                if (resultUtils != ARUTILS_OK)
                {
                    result = ARDATATRANSFER_ERROR_CANCELED;
                    goto end_search_dcim;
                }

                // Iterate for each subdir
                nextDcim = NULL;
                while ((dirName = ARUTILS_Ftp_List_GetNextItem(dcimFtpList, &nextDcim, NULL, 1, NULL, NULL, lineDataDcim, ARUTILS_FTP_MAX_PATH_SIZE))!= NULL)
                {

                    resultUtils = ARUTILS_Manager_Ftp_Connection_IsCanceled(manager->mediasDownloader->ftpListManager);
                    if (resultUtils != ARUTILS_OK)
                    {
                        result = ARDATATRANSFER_ERROR_CANCELED;
                        goto end_search_dcim;
                    }

                    strncpy(remotePath, manager->mediasDownloader->remoteDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
                    remotePath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
                    strncat(remotePath, "/" ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_DCIM "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(remotePath) - 1);
                    strncat(remotePath, dirName, ARUTILS_FTP_MAX_PATH_SIZE - strlen(remotePath) - 1);
                    strncat(remotePath, "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(remotePath) - 1);

                    if (ARUTILS_Manager_Ftp_List(manager->mediasDownloader->ftpListManager, remotePath, &mediaFtpList, &mediaFtpListLen) != ARUTILS_OK)
                    {
                        result = ARDATATRANSFER_ERROR_FTP;
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "Unable to list DCIM/%s", fileName);
                        goto end_search_dcim;
                    }

                    // And now iterate on each media
                    char lineDataMedia[ARUTILS_FTP_MAX_PATH_SIZE];
                    nextMedia = NULL;
                    while ((fileName = ARUTILS_Ftp_List_GetNextItem(mediaFtpList, &nextMedia, NULL, 0, &lineItem, &lineSize, lineDataMedia, ARUTILS_FTP_MAX_PATH_SIZE)) != NULL)
                    {

                        resultUtils = ARUTILS_Manager_Ftp_Connection_IsCanceled(manager->mediasDownloader->ftpListManager);
                        if (resultUtils != ARUTILS_OK)
                        {
                            result = ARDATATRANSFER_ERROR_CANCELED;
                            goto end_search_dcim;
                        }

                        char *index;
                        // Check that the file is a proper media file
                        index = strrchr (fileName, '.');
                        if (index == NULL)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "No dot found in %s", fileName);
                            continue;
                        }

                        index++;
                        const char *ext;
                        if (strcmp(index, ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_JPG_CAP) == 0)
                        {
                            ext = ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_JPG;
                        }
                        else if(strcmp(index, ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_MP4_CAP) == 0)
                        {
                            ext = ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_MP4;
                        }
                        else if(strcmp(index, ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_MOV_CAP) == 0)
                        {
                            ext = ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_MOV;
                        }
                        else
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "File is not media file !");
                            continue;
                        }

                        // Check that we have a proper thumbnail file for this media
                        char lineDataThumb[ARUTILS_FTP_MAX_PATH_SIZE];
                        const char *nextThumb = NULL;
                        char thumbPrefix[ARUTILS_FTP_MAX_PATH_SIZE];
                        strncpy(thumbPrefix, dirName, ARUTILS_FTP_MAX_PATH_SIZE);
                        thumbPrefix[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
                        strncat(thumbPrefix, fileName, ARUTILS_FTP_MAX_PATH_SIZE - strlen(thumbPrefix) - 1);
                        strncat(thumbPrefix, ".", ARUTILS_FTP_MAX_PATH_SIZE - strlen(thumbPrefix) - 1);

                        thumbName = ARUTILS_Ftp_List_GetNextItem(metaThumbList, &nextThumb, thumbPrefix, 0, NULL, NULL, lineDataThumb, ARUTILS_FTP_MAX_PATH_SIZE);
                        if (thumbName == NULL)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "No thumbnail for media %s/%s", dirName, fileName);
                            continue;
                        }

                        // Yes we do, get all infos !

                        // Start with size
                        double fileSize;
                        if (ARUTILS_Ftp_List_GetItemSize(lineItem, lineSize, &fileSize) == NULL)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "Unable to get media size");
                            continue;
                        }

                        ARDATATRANSFER_Media_t *media = calloc(1, sizeof *media);
                        if (media == NULL)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "Unable to allocate media");
                            continue;
                        }

                        // Forge the media

                        // The DCIM prefix for thumbnails is 21 chars long
                        // 8 for the 100DRONE (folder name) +
                        // 8 for the JUMP0001 (media name) +
                        // 4 for the .MOV (media extension) +
                        // 1 for the . (separator)
                        // The len of thumbName is always good because we checked
                        // this prefix in the GetNextItem function.
                        const int dcimHeaderLen = 21;
                        media->product = ARDISCOVERY_getProductFromPathName(&thumbName[dcimHeaderLen]);

                        // Media name is:
                        // thumbnailName without dcim prefix
                        // the extension (last 3 chars) replaced by the lowercase media ext
                        strncpy(media->name, &thumbName[dcimHeaderLen], ARDATATRANSFER_MEDIA_NAME_SIZE);
                        media->name[ARDATATRANSFER_MEDIA_NAME_SIZE - 1] = '\0';
                        strncpy(&media->name[strlen(media->name) - 3], ext, 3);

                        // Media filePath is the local path + the name
                        strncpy(media->filePath, manager->mediasDownloader->localDirectory, ARDATATRANSFER_MEDIA_PATH_SIZE);
                        media->filePath[ARDATATRANSFER_MEDIA_PATH_SIZE - 1] = '\0';
                        strncat(media->filePath, media->name, ARDATATRANSFER_MEDIA_PATH_SIZE - strlen(media->filePath) - 1);

                        // Media UUID is after the last _ but before the last .
                        char *begin = strrchr(media->name, '_');
                        char *end = strrchr(media->name, '.');
                        if (begin == NULL || end == NULL)
                        {
                            media->uuid[0] = '\0';
                        }
                        else
                        {
                            int len = end - begin - 1;
                            int start = begin - media->name + 1;
                            if (len >= ARDATATRANSFER_MEDIA_UUID_SIZE)
                            {
                                len = ARDATATRANSFER_MEDIA_UUID_SIZE - 1;
                            }
                            strncpy(media->uuid, &media->name[start], len);
                            media->uuid[len] = '\0';
                        }

                        // Media date is between the last _ and the previous one
                        end = begin;
                        begin = end - 1;
                        // Find previous "_", limiting at the beginning of the string
                        for (begin = end - 1; begin >= media->name && *begin != '_'; begin--);
                        if (*begin != '_')
                        {
                            media->date[0] = '\0';
                        }
                        else
                        {
                            int len = end - begin - 1;
                            int start = begin - media->name + 1;
                            if (len >= ARDATATRANSFER_MEDIA_UUID_SIZE)
                            {
                                len = ARDATATRANSFER_MEDIA_UUID_SIZE - 1;
                            }
                            strncpy(media->date, &media->name[start], len);
                            media->date[len] = '\0';
                        }

                        // Media size is just fileSize ;)
                        media->size = fileSize;

                        // Remote path is the full path to the file on the FTP
                        strncpy(media->remotePath, manager->mediasDownloader->remoteDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
                        media->remotePath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
                        strncat(media->remotePath, "/" ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_DCIM "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remotePath) -1);
                        strncat(media->remotePath, dirName, ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remotePath) -1);
                        strncat(media->remotePath, "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remotePath) -1);
                        strncat(media->remotePath, fileName, ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remotePath) -1);

                        // Remote thumb is the full path of the thumbnail on the FTP
                        strncpy(media->remoteThumb, manager->mediasDownloader->remoteDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
                        media->remoteThumb[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
                        strncat(media->remoteThumb, "/" ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_META "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remoteThumb) -1);
                        strncat(media->remoteThumb, thumbName, ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remoteThumb) - 1);

                        if (withThumbnail == 1)
                        {
                            ARDATATRANSFER_MediasDownloader_GetThumbnail(manager, media);
                        }

                        ARDATATRANSFER_Media_t **oldMedias = manager->mediasDownloader->medias.medias;
                        manager->mediasDownloader->medias.medias = realloc(oldMedias, (manager->mediasDownloader->medias.count + 1) * sizeof(ARDATATRANSFER_Media_t *));
                        if (manager->mediasDownloader->medias.medias == NULL)
                        {
                            manager->mediasDownloader->medias.medias = oldMedias;
                            result = ARDATATRANSFER_ERROR_ALLOC;
                            free(media);
                            goto end_search_dcim;
                        }
                        manager->mediasDownloader->medias.medias[manager->mediasDownloader->medias.count++] = media;
                    }
                }
            }
        }
    end_search_dcim:
        free (metaThumbList);
        free (dcimFtpList);
        free (mediaFtpList);
        mediaFtpList = NULL;
        mediaFtpListLen = 0;

        /* Search for medias in their product subfolders */
        product = 0;
        while ((result == ARDATATRANSFER_OK) && (product < ARDISCOVERY_PRODUCT_MAX))
        {
            resultUtils = ARUTILS_Manager_Ftp_Connection_IsCanceled(manager->mediasDownloader->ftpListManager);

            if (resultUtils != ARUTILS_OK)
            {
                result = ARDATATRANSFER_ERROR_CANCELED;
            }

            if (result == ARDATATRANSFER_OK)
            {
                char lineDataProduct[ARUTILS_FTP_MAX_PATH_SIZE];
                ARDISCOVERY_getProductPathName(product, productPathName, sizeof(productPathName));
                nextProduct = NULL;
                fileName = ARUTILS_Ftp_List_GetNextItem(productFtpList, &nextProduct, productPathName, 1, NULL, NULL, lineDataProduct,ARUTILS_FTP_MAX_PATH_SIZE);

                if ((fileName != NULL) && strcmp(fileName, productPathName) == 0)
                {
                    strncpy(remoteProduct, manager->mediasDownloader->remoteDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
                    remoteProduct[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
                    strncat(remoteProduct, "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(remoteProduct) - 1);
                    strncat(remoteProduct, productPathName, ARUTILS_FTP_MAX_PATH_SIZE - strlen(remoteProduct) - 1);
                    strncat(remoteProduct, "/" ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_MEDIA "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(remoteProduct) - 1);

                    resultUtils = ARUTILS_Manager_Ftp_List(manager->mediasDownloader->ftpListManager, remoteProduct, &mediaFtpList, &mediaFtpListLen);
                    if (resultUtils == ARUTILS_OK)
                    {
                        char lineDataMedia[ARUTILS_FTP_MAX_PATH_SIZE];
                        int fileType;
                        const char *index;

                        nextMedia = NULL;
                        while ((result == ARDATATRANSFER_OK)
                               && (fileName = ARUTILS_Ftp_List_GetNextItem(mediaFtpList, &nextMedia, NULL, 0, &lineItem, &lineSize, lineDataMedia,ARUTILS_FTP_MAX_PATH_SIZE)) != NULL)
                        {
                            resultUtils = ARUTILS_Manager_Ftp_Connection_IsCanceled(manager->mediasDownloader->ftpListManager);

                            if (resultUtils != ARUTILS_OK)
                            {
                                result = ARDATATRANSFER_ERROR_CANCELED;
                            }

                            //Check file type
                            fileType = 0;
                            index = fileName + strlen(fileName);
                            while (index > fileName && *index != '.')
                            {
                                index--;
                            }
                            if (*index == '.')
                            {
                                index++;
                                if (strcmp(index, ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_JPG) == 0)
                                {
                                    fileType = 1;
                                }
                                else if (strcmp(index, ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_MP4) == 0)
                                {
                                    fileType = 1;
                                }
                                else if (strcmp(index, ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_MOV) == 0)
                                {
                                    fileType = 1;
                                }
                            }

                            if (strncmp(fileName, productPathName, strlen(productPathName)) != 0)
                            {
                                fileType = 0;
                            }

                            if (result == ARDATATRANSFER_OK && (fileType == 1))
                            {
                                char remotePath[ARUTILS_FTP_MAX_PATH_SIZE];
                                ARDATATRANSFER_Media_t **oldMedias;
                                ARDATATRANSFER_Media_t *media = NULL;
                                double fileSize;
                                const char *begin = NULL;;
                                const char *tag = NULL;
                                const char *end = NULL;
                                const char *index;

                                strncpy(remotePath, remoteProduct, ARUTILS_FTP_MAX_PATH_SIZE);
                                remotePath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
                                strncat(remotePath, fileName, ARUTILS_FTP_MAX_PATH_SIZE - strlen(remotePath) - 1);

                                //do not pertorm ARUTILS_Ftp_Size that is too long, prefer decoding the FTP LIST
                                if (ARUTILS_Ftp_List_GetItemSize(lineItem, lineSize, &fileSize) != NULL)
                                {
                                    resultUtils = ARUTILS_OK;
                                }
                                else
                                {
                                    resultUtils = ARUTILS_ERROR_FTP_CODE;
                                }

                                if ((result == ARDATATRANSFER_OK) && (resultUtils == ARUTILS_OK))
                                {
                                    media = (ARDATATRANSFER_Media_t *)calloc(1, sizeof(ARDATATRANSFER_Media_t));

                                    if (media == NULL)
                                    {
                                        result = ARDATATRANSFER_ERROR_ALLOC;
                                    }
                                    else
                                    {
                                        media->product = product;
                                        strncpy(media->name, fileName, ARDATATRANSFER_MEDIA_NAME_SIZE);
                                        media->name[ARDATATRANSFER_MEDIA_NAME_SIZE - 1] = '\0';

                                        strncpy(media->filePath, manager->mediasDownloader->localDirectory, ARDATATRANSFER_MEDIA_PATH_SIZE);
                                        media->filePath[ARDATATRANSFER_MEDIA_PATH_SIZE - 1] = '\0';
                                        strncat(media->filePath, fileName, ARDATATRANSFER_MEDIA_PATH_SIZE - strlen(media->filePath) - 1);

                                        strncpy(media->date, "", ARDATATRANSFER_MEDIA_DATE_SIZE);
                                        media->date[ARDATATRANSFER_MEDIA_DATE_SIZE - 1] = '\0';

                                        strncpy(media->uuid, "", ARDATATRANSFER_MEDIA_UUID_SIZE);
                                        media->uuid[ARDATATRANSFER_MEDIA_UUID_SIZE - 1] = '\0';
                                        //Jumping_Sumo_1970-01-01T000317+0000_3902B87F947BE865A9D137CFA63492B8.mp4

                                        index = media->name;
                                        while ((index = strstr(index, "_")) != NULL)
                                        {
                                            begin = tag;
                                            tag = ++index;
                                        }

                                        if ((begin != NULL) && (tag != NULL))
                                        {
                                            end = strstr(begin, ".");
                                        }

                                        if ((begin != NULL)  && (tag != NULL) && (end != NULL))
                                        {
                                            long len = tag - begin - 1;
                                            len = (len < ARDATATRANSFER_MEDIA_DATE_SIZE) ? len : (ARDATATRANSFER_MEDIA_DATE_SIZE - 1);
                                            strncpy(media->date, begin, len);
                                            media->date[len] = '\0';

                                            len = end - tag;
                                            len = (len < ARDATATRANSFER_MEDIA_UUID_SIZE) ? len : (ARDATATRANSFER_MEDIA_UUID_SIZE - 1);
                                            strncpy(media->uuid, tag, len);
                                            media->uuid[len] = '\0';
                                        }

                                        media->size = fileSize;

                                        // Remote path is the full path to the file on the FTP
                                        strncpy(media->remotePath, manager->mediasDownloader->remoteDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
                                        media->remotePath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
                                        strncat(media->remotePath, "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remotePath) -1);
                                        strncat(media->remotePath, productPathName, ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remotePath) -1);
                                        strncat(media->remotePath, "/" ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_MEDIA "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remotePath) -1);
                                        strncat(media->remotePath, fileName, ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remotePath) -1);

                                        // Remote thumb is the full path of the thumbnail on the FTP
                                        strncpy(media->remoteThumb, manager->mediasDownloader->remoteDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
                                        media->remoteThumb[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
                                        strncat(media->remoteThumb, "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remoteThumb) -1);
                                        strncat(media->remoteThumb, productPathName, ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remoteThumb) -1);
                                        strncat(media->remoteThumb, "/" ARDATATRANSFER_MEDIAS_DOWNLOADER_FTP_THUMB "/", ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remoteThumb) -1);
                                        strncat(media->remoteThumb, fileName, ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remoteThumb) - 1);
                                        // Append ".jpg" to remote thumb is fileName does not already end whith .jpg
                                        if (fileName[strlen(fileName) - 4] != '.' ||
                                            fileName[strlen(fileName) - 3] != 'j' ||
                                            fileName[strlen(fileName) - 2] != 'p' ||
                                            fileName[strlen(fileName) - 1] != 'g')
                                        {
                                            strncat(media->remoteThumb, "." ARDATATRANSFER_MEDIAS_DOWNLOADER_EXT_JPG, ARUTILS_FTP_MAX_PATH_SIZE - strlen(media->remoteThumb) - 1);
                                        }

                                        if (withThumbnail == 1)
                                        {
                                            ARDATATRANSFER_MediasDownloader_GetThumbnail(manager, media);
                                        }
                                    }
                                }

                                if (result == ARDATATRANSFER_OK)
                                {
                                    oldMedias = manager->mediasDownloader->medias.medias;
                                    manager->mediasDownloader->medias.medias = (ARDATATRANSFER_Media_t **)realloc(manager->mediasDownloader->medias.medias, (manager->mediasDownloader->medias.count + 1) * sizeof(ARDATATRANSFER_Media_t *));

                                    if (manager->mediasDownloader->medias.medias == NULL)
                                    {
                                        manager->mediasDownloader->medias.medias = oldMedias;
                                        result = ARDATATRANSFER_ERROR_ALLOC;
                                        free(media);
                                    }
                                    else
                                    {
                                        manager->mediasDownloader->medias.medias[manager->mediasDownloader->medias.count] = media;
                                        manager->mediasDownloader->medias.count++;
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "List of %s failed with error  = %i",
                            remoteProduct, resultUtils);
                        // only set the error if the product has no DCIM folder
                        // if it has a DCIM folder, we assume that the pictures are in it.
                        if (!hasDCIM)
                        {
                            result = ARDATATRANSFER_ERROR_FTP;
                        }
                    }

                    if (mediaFtpList != NULL)
                    {
                        free(mediaFtpList);
                        mediaFtpList = NULL;
                        mediaFtpListLen = 0;
                    }
                }
            }
            product++;
        }

        if (productFtpList != NULL)
        {
            free(productFtpList);
        }

        if (result == ARDATATRANSFER_OK)
        {
            count = manager->mediasDownloader->medias.count;
        }
        else
        {
            ARDATATRANSFER_MediasDownloader_FreeMediaList(&manager->mediasDownloader->medias);
        }

        ARSAL_Mutex_Unlock(&manager->mediasDownloader->mediasLock);
    }

    *error = result;
    return count;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_GetAvailableMediasAsync(ARDATATRANSFER_Manager_t *manager, ARDATATRANSFER_MediasDownloader_AvailableMediaCallback_t availableMediaCallback, void *availableMediaArg)
{
    ARDATATRANSFER_Media_t *media;
    ARDATATRANSFER_Media_t tmpMedia;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR resultUtils = ARUTILS_OK;
    int i;

    if ((manager == NULL) || (availableMediaCallback == NULL))
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if ((result == ARDATATRANSFER_OK) && (manager->mediasDownloader == NULL))
    {
        result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
    }

    if (result == ARDATATRANSFER_OK)
    {
        if (manager->mediasDownloader->medias.count == 0)
        {
            ARDATATRANSFER_MediasDownloader_GetAvailableMediasSync(manager, 0, &result);
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        for (i=manager->mediasDownloader->medias.count-1; (resultUtils == ARUTILS_OK) && (i>=0); i--)
        {
            eARDATATRANSFER_ERROR resultThumbnail;
            resultUtils = ARUTILS_Manager_Ftp_Connection_IsCanceled(manager->mediasDownloader->ftpListManager);

            if (resultUtils != ARUTILS_OK)
            {
                result = ARDATATRANSFER_ERROR_CANCELED;
            }
            else
            {
                ARSAL_Mutex_Lock(&manager->mediasDownloader->mediasLock);

                media = manager->mediasDownloader->medias.medias[i];
                if (media != NULL)
                {
                    memcpy(&tmpMedia, media, sizeof(ARDATATRANSFER_Media_t));
                }
                else
                {
                    memset(&tmpMedia, 0, sizeof(ARDATATRANSFER_Media_t));
                }

                ARSAL_Mutex_Unlock(&manager->mediasDownloader->mediasLock);

                if ((media != NULL) && (tmpMedia.thumbnail == NULL))
                {
                    resultThumbnail = ARDATATRANSFER_MediasDownloader_GetThumbnail(manager, &tmpMedia);
                    if ((resultThumbnail == ARDATATRANSFER_OK) && (tmpMedia.thumbnail != NULL))
                    {
                        ARSAL_Mutex_Lock(&manager->mediasDownloader->mediasLock);

                        media = manager->mediasDownloader->medias.medias[i];
                        if (media != NULL)
                        {
                            media->thumbnail = tmpMedia.thumbnail;
                            media->thumbnailSize = tmpMedia.thumbnailSize;
                        }
                        else
                        {
                            free(tmpMedia.thumbnail);
                            tmpMedia.thumbnail = NULL;
                        }

                        ARSAL_Mutex_Unlock(&manager->mediasDownloader->mediasLock);
                        availableMediaCallback(availableMediaArg, &tmpMedia, i);
                    }
                }
            }
        }
    }

    return result;
}

ARDATATRANSFER_Media_t * ARDATATRANSFER_MediasDownloader_GetAvailableMediaAtIndex(ARDATATRANSFER_Manager_t *manager, int index, eARDATATRANSFER_ERROR *error)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    ARDATATRANSFER_Media_t *media = NULL;

    if (manager == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if ((result == ARDATATRANSFER_OK) && (manager->mediasDownloader == NULL))
    {
        result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
    }

    if (result == ARDATATRANSFER_OK)
    {
        ARSAL_Mutex_Lock(&manager->mediasDownloader->mediasLock);

        if (((index >= 0) && (index < manager->mediasDownloader->medias.count)))
        {
            media = manager->mediasDownloader->medias.medias[index];
        }
        else
        {
            result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
        }

        ARSAL_Mutex_Unlock(&manager->mediasDownloader->mediasLock);
    }

    *error = result;
    return media;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_DeleteMedia(ARDATATRANSFER_Manager_t *manager, ARDATATRANSFER_Media_t *media, ARDATATRANSFER_MediasDownloader_DeleteMediaCallback_t deleteMediaCallBack, void *deleteMediaArg)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR resultUtils = ARUTILS_OK;

    if ((manager == NULL) || (media == NULL))
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK && (manager->mediasDownloader == NULL))
    {
        result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
    }

    if (result == ARDATATRANSFER_OK)
    {
        resultUtils = ARUTILS_Manager_Ftp_Delete(manager->mediasDownloader->ftpQueueManager, media->remotePath);

        if (resultUtils != ARUTILS_OK)
        {
            result = ARDATATRANSFER_ERROR_FTP;
        }

        ARUTILS_Manager_Ftp_Delete(manager->mediasDownloader->ftpQueueManager, media->remoteThumb);

        if (deleteMediaCallBack != NULL)
        {
            deleteMediaCallBack(deleteMediaArg, media, result);
        }

        if (result == ARDATATRANSFER_OK)
        {
            result = ARDATATRANSFER_MediasDownloader_RemoveMediaFromMediaList(manager, media);
        }
    }

    return result;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_AddMediaToQueue(ARDATATRANSFER_Manager_t *manager, ARDATATRANSFER_Media_t *media,  ARDATATRANSFER_MediasDownloader_MediaDownloadProgressCallback_t progressCallback, void *progressArg, ARDATATRANSFER_MediasDownloader_MediaDownloadCompletionCallback_t completionCallback, void *completionArg)
{
    ARDATATRANSFER_FtpMedia_t *newFtpMedia = NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if ((manager == NULL) || (media == NULL))
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK && (manager->mediasDownloader == NULL))
    {
        result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
    }

    if (result == ARDATATRANSFER_OK)
    {
        newFtpMedia = calloc(1, sizeof(ARDATATRANSFER_FtpMedia_t));

        if (newFtpMedia == NULL)
        {
            result = ARDATATRANSFER_ERROR_ALLOC;
        }
        else
        {
            strncpy(newFtpMedia->media.name, media->name, ARDATATRANSFER_MEDIA_NAME_SIZE);
            newFtpMedia->media.name[ARDATATRANSFER_MEDIA_NAME_SIZE - 1] = '\0';
            strncpy(newFtpMedia->media.filePath, media->filePath, ARDATATRANSFER_MEDIA_PATH_SIZE);
            newFtpMedia->media.filePath[ARDATATRANSFER_MEDIA_PATH_SIZE - 1] = '\0';
            strncpy(newFtpMedia->media.date, media->date, ARDATATRANSFER_MEDIA_DATE_SIZE);
            newFtpMedia->media.date[ARDATATRANSFER_MEDIA_DATE_SIZE - 1] = '\0';
            strncpy(newFtpMedia->media.uuid, media->uuid, ARDATATRANSFER_MEDIA_UUID_SIZE);
            newFtpMedia->media.uuid[ARDATATRANSFER_MEDIA_UUID_SIZE - 1] = '\0';
            strncpy(newFtpMedia->media.remotePath, media->remotePath, ARUTILS_FTP_MAX_PATH_SIZE);
            newFtpMedia->media.remotePath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
            strncpy(newFtpMedia->media.remoteThumb, media->remoteThumb, ARUTILS_FTP_MAX_PATH_SIZE);
            newFtpMedia->media.remoteThumb[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
            newFtpMedia->media.size = media->size;
            newFtpMedia->media.product = media->product;

            newFtpMedia->progressCallback = progressCallback;
            newFtpMedia->progressArg = progressArg;
            newFtpMedia->completionCallback = completionCallback;
            newFtpMedia->completionArg = completionArg;
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        result = ARDATATRANSFER_MediasQueue_Add(&manager->mediasDownloader->queue, newFtpMedia);
    }

    if (result == ARDATATRANSFER_OK)
    {
        ARSAL_Sem_Post(&manager->mediasDownloader->queueSem);
    }

    if (result != ARDATATRANSFER_OK)
    {
        free(newFtpMedia);
    }

    return result;
}

void* ARDATATRANSFER_MediasDownloader_QueueThreadRun(void *managerArg)
{
    ARDATATRANSFER_Manager_t *manager = (ARDATATRANSFER_Manager_t *)managerArg;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if (manager == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK && (manager->mediasDownloader == NULL))
    {
        result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
    }

    if (result == ARDATATRANSFER_OK && (manager->mediasDownloader->isCanceled != 0))
    {
        result = ARDATATRANSFER_ERROR_CANCELED;
    }

    if (result == ARDATATRANSFER_OK && (manager->mediasDownloader->isRunning != 0))
    {
        result = ARDATATRANSFER_ERROR_THREAD_ALREADY_RUNNING;
    }

    if (result == ARDATATRANSFER_OK)
    {
        manager->mediasDownloader->isRunning = 1;
    }

    if (result == ARDATATRANSFER_OK)
    {
        eARDATATRANSFER_ERROR error = ARDATATRANSFER_OK;
        ARDATATRANSFER_FtpMedia_t *ftpMedia = NULL;
        int resultSys;

        do
        {
            resultSys = ARSAL_Sem_Wait(&manager->mediasDownloader->queueSem);

            if (resultSys != 0)
            {
                result = ARDATATRANSFER_ERROR_SYSTEM;
            }

            if (result == ARDATATRANSFER_OK)
            {
                ftpMedia = ARDATATRANSFER_MediasQueue_Pop(&manager->mediasDownloader->queue, &error);
            }

            if ((result == ARDATATRANSFER_OK)
                && (error == ARDATATRANSFER_OK)
                && (ftpMedia != NULL)
                && (manager->mediasDownloader->isCanceled == 0))
            {
                error = ARDATATRANSFER_MediasDownloader_DownloadMedia(manager, ftpMedia);
            }

            if (ftpMedia != NULL)
            {
                if ((ftpMedia->completionCallback != NULL) && (manager->mediasDownloader->isCanceled == 0))
                {
                    ftpMedia->completionCallback(ftpMedia->completionArg, &ftpMedia->media, error);
                }

                free(ftpMedia);
                ftpMedia = NULL;
            }
        }
        while (manager->mediasDownloader->isCanceled == 0);
    }

    if (manager != NULL && manager->mediasDownloader != NULL)
    {
        manager->mediasDownloader->isRunning = 0;

        ARDATATRANSFER_MediasDownloader_ResetQueueThread(manager);
    }

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "exit");

    return NULL;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_ResetQueueThread(ARDATATRANSFER_Manager_t *manager)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if (manager == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK && (manager->mediasDownloader == NULL))
    {
        result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
    }

    if (result == ARDATATRANSFER_OK)
    {
        manager->mediasDownloader->isCanceled = 0;
    }

    if (result == ARDATATRANSFER_OK)
    {
        while (ARSAL_Sem_Trywait(&manager->mediasDownloader->threadSem) == 0)
        {
            /* Do nothing*/
        }

        while (ARSAL_Sem_Trywait(&manager->mediasDownloader->queueSem) == 0)
        {
            /* Do nothing*/
        }

        ARUTILS_Manager_Ftp_Connection_Reset(manager->mediasDownloader->ftpQueueManager);
    }

    return result;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_CancelQueueThread(ARDATATRANSFER_Manager_t *manager)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR resultUtils = ARUTILS_OK;
    int resultSys = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if (manager == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK && (manager->mediasDownloader == NULL))
    {
        result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
    }

    if (result == ARDATATRANSFER_OK)
    {
        manager->mediasDownloader->isCanceled = 1;
    }

    if (result == ARDATATRANSFER_OK)
    {
        result = ARDATATRANSFER_MediasQueue_RemoveAll(&manager->mediasDownloader->queue);
    }

    if (result == ARDATATRANSFER_OK)
    {
        resultSys = ARSAL_Sem_Post(&manager->mediasDownloader->threadSem);

        if (resultSys != 0)
        {
            result = ARDATATRANSFER_ERROR_SYSTEM;
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        resultSys = ARSAL_Sem_Post(&manager->mediasDownloader->queueSem);

        if (resultSys != 0)
        {
            result = ARDATATRANSFER_ERROR_SYSTEM;
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        resultUtils = ARUTILS_Manager_Ftp_Connection_Cancel(manager->mediasDownloader->ftpQueueManager);

        if (resultUtils != ARUTILS_OK)
        {
            result = ARDATATRANSFER_ERROR_FTP;
        }
    }

    return result;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_ResetGetAvailableMedias(ARDATATRANSFER_Manager_t *manager)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR resultUtils = ARUTILS_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if (manager == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK && (manager->mediasDownloader == NULL))
    {
        result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
    }

    if (result == ARDATATRANSFER_OK)
    {
        resultUtils = ARUTILS_Manager_Ftp_Connection_Reset(manager->mediasDownloader->ftpListManager);
        if (resultUtils != ARUTILS_OK)
        {
            result = ARDATATRANSFER_ERROR_FTP;
        }
    }

    return result;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_CancelGetAvailableMedias(ARDATATRANSFER_Manager_t *manager)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR resultUtils = ARUTILS_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if (manager == NULL)
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK && (manager->mediasDownloader == NULL))
    {
        result = ARDATATRANSFER_ERROR_NOT_INITIALIZED;
    }

    if (result == ARDATATRANSFER_OK)
    {
        resultUtils = ARUTILS_Manager_Ftp_Connection_Cancel(manager->mediasDownloader->ftpListManager);
        if (resultUtils != ARUTILS_OK)
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

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_Initialize(ARDATATRANSFER_Manager_t *manager, ARUTILS_Manager_t *ftpListManager, ARUTILS_Manager_t *ftpQueueManager, const char *remoteDirectory, const char *localDirectory)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    //eARUTILS_ERROR error = ARUTILS_OK;
    int resultSys = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s, %s", localDirectory ? localDirectory : "null", remoteDirectory ? remoteDirectory : "null");

    if ((manager == NULL) || (ftpListManager == NULL) || (ftpQueueManager == NULL) || (localDirectory == NULL))
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK)
    {
        strncpy(manager->mediasDownloader->remoteDirectory, remoteDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
        manager->mediasDownloader->remoteDirectory[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
        strncpy(manager->mediasDownloader->localDirectory, localDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
        manager->mediasDownloader->localDirectory[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
        strncat(manager->mediasDownloader->localDirectory, "/" , ARUTILS_FTP_MAX_PATH_SIZE - strlen(manager->mediasDownloader->localDirectory) - 1);

        resultSys = mkdir(manager->mediasDownloader->localDirectory, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        if ((resultSys != 0) && (errno != EEXIST))
        {
            result = ARDATATRANSFER_ERROR_SYSTEM;
        }
    }

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "return %d", result);

    return result;
}

void ARDATATRANSFER_MediasDownloader_Clear(ARDATATRANSFER_Manager_t *manager)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if ((manager != NULL) && (manager->mediasDownloader != NULL))
    {
        manager->mediasDownloader->isCanceled = 0;
    }
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_GetThumbnail(ARDATATRANSFER_Manager_t *manager, ARDATATRANSFER_Media_t *media)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR error = ARUTILS_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if ((manager == NULL) || (media == NULL))
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK)
    {
        error = ARUTILS_Manager_Ftp_Get_WithBuffer(manager->mediasDownloader->ftpListManager, media->remoteThumb, &media->thumbnail, &media->thumbnailSize, NULL, NULL);

        if (error != ARUTILS_OK)
        {
            result = ARDATATRANSFER_ERROR_FTP;
        }
    }

    return result;
}

void ARDATATRANSFER_MediasDownloader_FtpProgressCallback(void* arg, float percent)
{
    ARDATATRANSFER_FtpMedia_t *ftpMedia = (ARDATATRANSFER_FtpMedia_t *)arg;

    if (ftpMedia != NULL)
    {
        if (ftpMedia->progressCallback != NULL)
        {
            ftpMedia->progressCallback(ftpMedia->progressArg, &ftpMedia->media, percent);
        }
    }
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_DownloadMedia(ARDATATRANSFER_Manager_t *manager, ARDATATRANSFER_FtpMedia_t *ftpMedia)
{
    char localPath[ARUTILS_FTP_MAX_PATH_SIZE];
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    eARUTILS_ERROR errorResume = ARUTILS_OK;
    eARUTILS_ERROR error = ARUTILS_OK;
    int64_t localSize = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if ((manager  == NULL) || (ftpMedia == NULL))
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK)
    {
        strncpy(localPath, manager->mediasDownloader->localDirectory, ARUTILS_FTP_MAX_PATH_SIZE);
        localPath[ARUTILS_FTP_MAX_PATH_SIZE - 1] = '\0';
        strncat(localPath, ARDATATRANSFER_MANAGER_DOWNLOADER_DOWNLOADING_PREFIX, ARUTILS_FTP_MAX_PATH_SIZE - strlen(localPath) - 1);
        strncat(localPath, ftpMedia->media.name, ARUTILS_FTP_MAX_PATH_SIZE - strlen(localPath) - 1);

        errorResume = ARUTILS_FileSystem_GetFileSize(localPath, &localSize);
    }

    if (result == ARDATATRANSFER_OK)
    {
        error = ARUTILS_Manager_Ftp_Get(manager->mediasDownloader->ftpQueueManager, ftpMedia->media.remotePath, localPath, ARDATATRANSFER_MediasDownloader_FtpProgressCallback, ftpMedia, (errorResume == ARUTILS_OK) ? FTP_RESUME_TRUE : FTP_RESUME_FALSE);

        if (error == ARUTILS_ERROR_FTP_CANCELED)
        {
            result = ARDATATRANSFER_ERROR_CANCELED;
        }
        else if (error != ARUTILS_OK)
        {
            result = ARDATATRANSFER_ERROR_FTP;
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        error = ARUTILS_FileSystem_Rename(localPath, ftpMedia->media.filePath);

        if (error != ARUTILS_OK)
        {
            result = ARDATATRANSFER_ERROR_FILE;
        }
    }
    else
    {
        // Don't remove localPath, keep it for resume, and manage older file in App
        //remove(localPath);
    }

    return result;
}

eARDATATRANSFER_ERROR ARDATATRANSFER_MediasDownloader_RemoveMediaFromMediaList(ARDATATRANSFER_Manager_t *manager, ARDATATRANSFER_Media_t *media)
{
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    ARDATATRANSFER_Media_t *curMedia;
    int foundIndex = -1;
    int i;

    if ((manager == NULL) || (manager->mediasDownloader == NULL) || (manager->mediasDownloader->medias.medias == NULL) || (manager->mediasDownloader->medias.count == 0))
    {
        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    if (result == ARDATATRANSFER_OK)
    {
        ARSAL_Mutex_Lock(&manager->mediasDownloader->mediasLock);

        for (i = 0; (foundIndex == -1) && (i < manager->mediasDownloader->medias.count); i++)
        {
            curMedia = manager->mediasDownloader->medias.medias[i];

            if ((curMedia != NULL) && (strcmp(curMedia->filePath, media->filePath) == 0))
            {
                foundIndex = i;
            }
        }

        if (foundIndex != -1)
        {
            curMedia = manager->mediasDownloader->medias.medias[foundIndex];
            manager->mediasDownloader->medias.medias[foundIndex] = NULL;
            free(curMedia);
        }

        ARSAL_Mutex_Unlock(&manager->mediasDownloader->mediasLock);
    }

    return result;
}

void ARDATATRANSFER_MediasDownloader_FreeMediaList(ARDATATRANSFER_MediaList_t *mediaList)
{
    int i = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_MEDIAS_DOWNLOADER_TAG, "%s", "");

    if (mediaList != NULL)
    {
        if (mediaList->medias != NULL)
        {
            for (i=0; i<mediaList->count; i++)
            {
                ARDATATRANSFER_Media_t *media = mediaList->medias[i];

                if (media != NULL)
                {
                    if (media->thumbnail != NULL)
                    {
                        free(media->thumbnail);
                    }

                    free(media);
                }
            }
            mediaList->medias = NULL;
        }

        mediaList->count = 0;
    }
}
