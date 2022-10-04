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
 * @brief libARUpdater PlfSender header file.
 * @date 23/05/2014
 * @author djavan.bertrand@parrot.com
 **/

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <poll.h>

#if defined BUILD_LIBMUX
#include <libpomp.h>
#include <libmux.h>
#include <libmux-update.h>
#endif

#include <libARSAL/ARSAL_Print.h>
#include <libARUtils/ARUtils.h>
#include <libARSAL/ARSAL_Error.h>
#include "ARUPDATER_Manager.h"

#include "ARUPDATER_Uploader.h"
#include "ARUPDATER_Utils.h"

/* ***************************************
 *
 *             define :
 *
 *****************************************/
#define ARUPDATER_UPLOADER_TAG                   "ARUPDATER_Uploader"
#define ARUPDATER_UPLOADER_REMOTE_FOLDER         "/"
#define ARUPDATER_UPLOADER_MD5_FILENAME          "md5_check.md5"
#define ARUPDATER_UPLOADER_UPLOADED_FILE_SUFFIX  ".tmp"
#define ARUPDATER_UPLOADER_CHUNK_SIZE            32
#define ARUPDATER_UPLOADER_MUX_CHUNK_SIZE        (128*1024)
/* ***************************************
 *
 *             function implementation :
 *
 *****************************************/

eARUPDATER_ERROR ARUPDATER_Uploader_New(ARUPDATER_Manager_t* manager, const char *const rootFolder, struct mux_ctx *mux, ARUTILS_Manager_t *ftpManager, ARSAL_MD5_Manager_t *md5Manager, int isAndroidApp, eARDISCOVERY_PRODUCT product, ARUPDATER_Uploader_PlfUploadProgressCallback_t progressCallback, void *progressArg, ARUPDATER_Uploader_PlfUploadCompletionCallback_t completionCallback, void *completionArg)
{
    ARUPDATER_Uploader_t *uploader = NULL;
    eARUPDATER_ERROR err = ARUPDATER_OK;
    int ret;
    int pipefds[2];
    char *slash = NULL;
    
    // Check parameters
    if ((manager == NULL) || (rootFolder == NULL) || (ftpManager == NULL) || (md5Manager == NULL))
    {
        err = ARUPDATER_ERROR_BAD_PARAMETER;
    }

#if !defined BUILD_LIBMUX
    if (mux != NULL)
        err = ARUPDATER_ERROR_BAD_PARAMETER;
#endif
    
    if(err == ARUPDATER_OK)
    {
        if (manager->uploader != NULL)
        {
            err = ARUPDATER_ERROR_MANAGER_ALREADY_INITIALIZED;
        }
        else
        {
            uploader = malloc (sizeof (ARUPDATER_Uploader_t));
            if (uploader == NULL)
            {
                err = ARUPDATER_ERROR_ALLOC;
            } else {
                manager->uploader = uploader;
            }
        }
    }

    /* Initialize to default values */
    if(err == ARUPDATER_OK)
    {
        int rootFolderLength = strlen(rootFolder) + 1;
        slash = strrchr(rootFolder, ARUPDATER_MANAGER_FOLDER_SEPARATOR[0]);
        if ((slash != NULL) && (strcmp(slash, ARUPDATER_MANAGER_FOLDER_SEPARATOR) != 0))
        {
            rootFolderLength += 1;
        }
        uploader->rootFolder = (char*) malloc(rootFolderLength);
        if (uploader->rootFolder == NULL)
        {
            err = ARUPDATER_ERROR_ALLOC;
        }
    }
    
    if (err == ARUPDATER_OK)
    {
        strcpy(uploader->rootFolder, rootFolder);
        
        if ((slash != NULL) && (strcmp(slash, ARUPDATER_MANAGER_FOLDER_SEPARATOR) != 0))
        {
            strcat(uploader->rootFolder, ARUPDATER_MANAGER_FOLDER_SEPARATOR);
        }
        
        uploader->product = product;
        uploader->isAndroidApp = isAndroidApp;
        uploader->subfolder = NULL;
        uploader->ftpManager = ftpManager;
        uploader->mux = mux;
#if defined BUILD_LIBMUX
        if (uploader->mux)
            mux_ref(uploader->mux);
#endif

        uploader->md5Manager = md5Manager;
        
        uploader->isRunning = 0;
        uploader->isCanceled = 0;
        uploader->isUploadThreadRunning = 0;
        uploader->isDownloadMd5ThreadRunning = 0;
        
        uploader->uploadError = ARDATATRANSFER_OK;
        
        uploader->progressArg = progressArg;
        uploader->completionArg = completionArg;
        
        uploader->progressCallback = progressCallback;
        uploader->completionCallback = completionCallback;
    }
    
    // create the data transfer manager
    if (ARUPDATER_OK == err)
    {
        eARDATATRANSFER_ERROR dataTransferError = ARDATATRANSFER_OK;
        uploader->dataTransferManager = ARDATATRANSFER_Manager_New(&dataTransferError);
        if (ARDATATRANSFER_OK != dataTransferError)
        {
            err = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
        }
    }
    
    if (err == ARUPDATER_OK)
    {
        int resultSys = ARSAL_Mutex_Init(&manager->uploader->uploadLock);
        
        if (resultSys != 0)
        {
            err = ARUPDATER_ERROR_SYSTEM;
        }
    }
    
    if (err == ARUPDATER_OK)
    {
        /* create pair of pipes for mux update */
        ret = pipe(pipefds);
        if (ret < 0) {
             ret = -errno;
             ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
                  "pipe error %s", strerror(-ret));
             err = ARUPDATER_ERROR_SYSTEM;
        }

        /* set pipes non blocking */
        fcntl(pipefds[0], F_SETFL, fcntl(pipefds[0], F_GETFL, 0) | O_NONBLOCK);
        fcntl(pipefds[1], F_SETFL, fcntl(pipefds[1], F_GETFL, 0) | O_NONBLOCK);

        manager->uploader->pipefds[0] = pipefds[0];
        manager->uploader->pipefds[1] = pipefds[1];
    }

    /* delete the uploader if an error occurred */
    if (err != ARUPDATER_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG, "error: %s", ARUPDATER_Error_ToString (err));
        ARUPDATER_Uploader_Delete (manager);
    }
    
    return err;
}

eARUPDATER_ERROR ARUPDATER_Uploader_Delete(ARUPDATER_Manager_t *manager)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    if (manager == NULL)
    {
        error = ARUPDATER_ERROR_BAD_PARAMETER;
    }
    else
    {
        if (manager->uploader == NULL)
        {
            error = ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
        }
        else
        {
            if (manager->uploader->isRunning != 0)
            {
                error = ARUPDATER_ERROR_THREAD_PROCESSING;
            }
            else
            {
                ARSAL_Mutex_Destroy(&manager->uploader->uploadLock);
                free(manager->uploader->rootFolder);
                manager->uploader->rootFolder = NULL;
                
                ARDATATRANSFER_Manager_Delete(&manager->uploader->dataTransferManager);
                close(manager->uploader->pipefds[0]);
                close(manager->uploader->pipefds[1]);

                free(manager->uploader->subfolder);
                manager->uploader->subfolder = NULL;
#if defined BUILD_LIBMUX
                if (manager->uploader->mux) {
                    mux_unref(manager->uploader->mux);
                    manager->uploader->mux = NULL;
                }
#endif
                free(manager->uploader);
                manager->uploader = NULL;
            }
        }
    }
    
    return error;
}

eARUPDATER_ERROR ARUPDATER_Uploader_SetSubfolder(ARUPDATER_Manager_t *manager, const char *subfolder)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    size_t len;
    int res;
    const char *filtered_subfolder;

    if (!manager)
        return ARUPDATER_ERROR_BAD_PARAMETER;
    if (!manager->uploader)
        return ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
    if (manager->uploader->isRunning)
        return ARUPDATER_ERROR_THREAD_PROCESSING;

    /* Handle "subfoler == NULL,empty,or /" case */
    if (!subfolder ||
        strlen(subfolder) == 0 ||
        (strlen(subfolder) == 1 && subfolder[0] == '/')) {
        free(manager->uploader->subfolder);
        manager->uploader->subfolder = NULL;
        return ARUPDATER_OK;
    }

    /* Subfolder is not null & not empty */
    len = strlen(subfolder);

    /* If subfolder starts with '/', strip it */
    if (subfolder[0] == '/')
        filtered_subfolder = &subfolder[1];
    else
        filtered_subfolder = subfolder;


    /* If subfolder don't end with '/', add one */
    if (subfolder[len-1] == '/') {
        manager->uploader->subfolder = strdup(filtered_subfolder);
        if (!manager->uploader->subfolder)
            error = ARUPDATER_ERROR_ALLOC;
    } else {
        res = asprintf(&manager->uploader->subfolder, "%s/", filtered_subfolder);
        if (res < 0) {
            error = ARUPDATER_ERROR_ALLOC;
            manager->uploader->subfolder = NULL;
        }
    }

    return error;
}

void* ARUPDATER_Uploader_ThreadRun(void *managerArg)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    
    ARUPDATER_Manager_t *manager = NULL;
    if (managerArg != NULL)
    {
        manager = (ARUPDATER_Manager_t*)managerArg;
    }
    else
    {
        error = ARUPDATER_ERROR_BAD_PARAMETER;
    }
    
    if ((manager != NULL) && (manager->uploader != NULL))
    {
        if ((manager->uploader->ftpManager->networkType == ARDISCOVERY_NETWORK_TYPE_BLE) &&
            (manager->uploader->isAndroidApp == 1))
        {
            error = ARUPDATER_Uploader_ThreadRunAndroidDelos(manager);
        }
        else if (manager->uploader->mux &&
                 ARDISCOVERY_getProductFamily(manager->uploader->product) == ARDISCOVERY_PRODUCT_FAMILY_SKYCONTROLLER)
        {
           // upload plf over mux
           error = ARUPDATER_Uploader_ThreadRunMux(manager);
        }
        else
        {
            // upload plf the normal way
            error = ARUPDATER_Uploader_ThreadRunNormal(manager);
        }
    }
    else
    {
        error = ARUPDATER_ERROR_BAD_PARAMETER;
    }
    
    return (void*)error;
}

eARUPDATER_ERROR ARUPDATER_Uploader_ThreadRunAndroidDelos(ARUPDATER_Manager_t *manager)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    
    if (manager && manager->uploader)
    {
        manager->uploader->isRunning = 1;
    }
    else
    {
        return ARUPDATER_ERROR_BAD_PARAMETER;
    }

    eARDATATRANSFER_ERROR dataTransferError = ARDATATRANSFER_OK;
    
    char *sourceFileFolder = NULL;
    char *sourceFilePath = NULL;
    char *device = NULL;
    char *fileName = NULL;
    
    uint16_t productId = ARDISCOVERY_getProductID(manager->uploader->product);
    device = malloc(ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE);
    if (device == NULL)
    {
        error = ARUPDATER_ERROR_ALLOC;
    }
    
    if (error == ARUPDATER_OK)
    {
        snprintf(device, ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE, "%04x", productId);
        
        sourceFileFolder = malloc(strlen(manager->uploader->rootFolder) + strlen(ARUPDATER_MANAGER_PLF_FOLDER) + strlen(device) + strlen(ARUPDATER_MANAGER_FOLDER_SEPARATOR) + 1);
        strcpy(sourceFileFolder, manager->uploader->rootFolder);
        strcat(sourceFileFolder, ARUPDATER_MANAGER_PLF_FOLDER);
        strcat(sourceFileFolder, device);
        strcat(sourceFileFolder, ARUPDATER_MANAGER_FOLDER_SEPARATOR);
        
        error = ARUPDATER_Utils_GetPlfInFolder(sourceFileFolder, &fileName);
    }
    
    if (error == ARUPDATER_OK)
    {
        sourceFilePath = malloc(strlen(sourceFileFolder) + strlen(fileName) + 1);
        if (sourceFilePath == NULL)
        {
            error = ARUPDATER_ERROR_ALLOC;
        }
    }
    
    if (ARUPDATER_OK == error)
    {
        strcpy(sourceFilePath, sourceFileFolder);
        strcat(sourceFilePath, fileName);
        
        // by default, do not resume an upload
        eARDATATRANSFER_UPLOADER_RESUME resumeMode = ARDATATRANSFER_UPLOADER_RESUME_FALSE;
        
        ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
        // create a new uploader
        dataTransferError = ARDATATRANSFER_Uploader_New(manager->uploader->dataTransferManager, manager->uploader->ftpManager, "", sourceFilePath, ARUPDATER_Uploader_ProgressCallback, manager, ARUPDATER_Uploader_CompletionCallback, manager, resumeMode);
        if (ARDATATRANSFER_OK != dataTransferError)
        {
            error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
        }
        ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);
    }
    
    if ((ARUPDATER_OK == error) && (manager->uploader->isCanceled == 0))
    {
        manager->uploader->isUploadThreadRunning = 1;
        ARDATATRANSFER_Uploader_ThreadRun(manager->uploader->dataTransferManager);
        manager->uploader->isUploadThreadRunning = 0;
        if (manager->uploader->uploadError != ARDATATRANSFER_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG, "ARDataTransferError = %d", manager->uploader->uploadError);
            error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
        }
    }
    
    ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
    if (ARUPDATER_OK == error)
    {
        dataTransferError = ARDATATRANSFER_Uploader_Delete(manager->uploader->dataTransferManager);
        if (ARDATATRANSFER_OK != dataTransferError)
        {
            error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
        }
    }
    ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);
    
    if (error != ARUPDATER_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG, "error: %s", ARUPDATER_Error_ToString (error));
    }
    
    if (sourceFileFolder != NULL)
    {
        free(sourceFileFolder);
    }
    if (sourceFilePath != NULL)
    {
        free(sourceFilePath);
    }
    if (device != NULL)
    {
        free(device);
    }
    if (fileName != NULL)
    {
        free(fileName);
    }

    manager->uploader->isRunning = 0;

    if (manager->uploader->completionCallback != NULL)
    {
        manager->uploader->completionCallback(manager->uploader->completionArg, error);
    }
    
    return error;
}

#if defined BUILD_LIBMUX
static int updater_mux_write_msg(struct mux_ctx *mux, uint32_t msgid,
		const char *fmt, ...)
{
	int res = 0;
	struct pomp_msg *msg = NULL;
	va_list args;

	msg = pomp_msg_new();
	if (msg == NULL)
		return -ENOMEM;

	va_start(args, fmt);
	res = pomp_msg_writev(msg, msgid, fmt, args);
	va_end(args);
	if (res < 0) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
				"pomp_msg_writev error: %s", strerror(-res));
		goto out;
	}

	res = mux_encode(mux, MUX_UPDATE_CHANNEL_ID_UPDATE,
			pomp_msg_get_buffer(msg));
	if (res < 0 && res != -EPIPE) {

		ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
				"mux_encode error: %s", strerror(-res));
		goto out;
	}

out:
	pomp_msg_destroy(msg);
	return res;
}

static void update_mux_notify_status(ARUPDATER_Uploader_t *up,
		eARUPDATER_ERROR status)
{
	int ret;
	int event = status;

	do {
		ret = write(up->pipefds[1], &event, sizeof(event));
	} while (ret < 0 && errno == EINTR);
}

static void update_mux_notify_progression(ARUPDATER_Uploader_t *up,
		float percent)
{
	int ret;
	int event = (int)percent;

	if (event <= 0)
		return;

	do {
		ret = write(up->pipefds[1], &event, sizeof(event));
	} while (ret < 0 && errno == EINTR);
}

static int updater_mux_send_next_chunk(ARUPDATER_Uploader_t *up)
{
	ssize_t ret;
	uint32_t n_bytes;

	/* read file chunk */
	ret = read(up->fd, up->chunk, ARUPDATER_UPLOADER_MUX_CHUNK_SIZE);
	if (ret < 0) {
		ret = -errno;
		ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
				"read update file error: %s", strerror(-ret));
		return ret;
	}

	if (ret == 0) {
		ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_UPLOADER_TAG,
			"read update file eof");
		return 0;
	}

	/* send chunk over mux */
	n_bytes = ret;

	ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_UPLOADER_TAG,
			"sending chunk: id=%zd size=%d", up->chunk_id, n_bytes);

	ret = updater_mux_write_msg(up->mux, MUX_UPDATE_MSG_ID_CHUNK,
			MUX_UPDATE_MSG_FMT_ENC_CHUNK, up->chunk_id,
			up->chunk, n_bytes);
	if (ret < 0)
		return ret;

	up->n_written += n_bytes;
	return 0;
}

static void updater_mux_channel_recv(ARUPDATER_Manager_t *mngr,
			struct pomp_buffer *buf)
{
	struct pomp_msg *msg = NULL;
	ARUPDATER_Uploader_t *up = mngr->uploader;
	float percent;
	int ret, status;
	unsigned int id;

	/* Create pomp message from buffer */
	msg = pomp_msg_new_with_buffer(buf);
	if (msg == NULL)
		return;

	/* Decode message */
	switch (pomp_msg_get_id(msg)) {
	case MUX_UPDATE_MSG_ID_UPDATE_RESP:
		/* decode status */
		ret = pomp_msg_read(msg, MUX_UPDATE_MSG_FMT_DEC_UPDATE_RESP,
				&status);

		if (ret < 0) {
			ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
				"pomp_msg_read error: %s", strerror(-ret));
			goto error;
		}

		ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_UPLOADER_TAG,
				"update resp: status=%d", status);

		if (status != 0) {
			ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
				"update refused by remote");
			goto error;
		}

		/* update accepted: start sensing file */
		up->n_written = 0;
		up->chunk_id = 0;
		lseek(up->fd, 0, SEEK_SET);

		/* send 1st chunk */
		ret = updater_mux_send_next_chunk(up);
		if (ret < 0)
			goto error;
	break;

	case MUX_UPDATE_MSG_ID_CHUNK_ACK:
		/* decode chunk id */
		ret = pomp_msg_read(msg, MUX_UPDATE_MSG_FMT_DEC_CHUNK_ACK, &id);
		if (ret < 0) {
			ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
				"pomp_msg_read error: %s", strerror(-ret));
			goto error;
		}

		ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_UPLOADER_TAG,
			"chunk ack: id=%d", id);

		if (id != up->chunk_id) {
			ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
				"chunk id mismatch %d != %zd", id, up->chunk_id);
			goto error;
		}

		/* notify progression */
		percent = (double) (100.f * up->n_written) / (double)up->size;
		ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_UPLOADER_TAG,
				"progression: %f%%", percent);
		update_mux_notify_progression(up, percent);

		/* send next chunk */
		if (up->n_written < up->size) {
			up->chunk_id++;
			ret = updater_mux_send_next_chunk(up);
			if (ret < 0)
				goto error;
		}

		/* last chunk sent and ack successfully received
		 * wait for update status from remote */
		ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_UPLOADER_TAG,
			"image sent waiting for status");
	break;

	case MUX_UPDATE_MSG_ID_STATUS:

		/* decode update status */
		ret = pomp_msg_read(msg, MUX_UPDATE_MSG_FMT_DEC_STATUS, &status);
		if (ret < 0) {
			ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
				"pomp_msg_read error: %s", strerror(-ret));
			goto error;
		}

		ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_UPLOADER_TAG,
			"update status: status=%d", status);

		/* check remote update status is ok */
		if (status != 0) {
			ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
				"remote update status %d", status);
			goto error;
		}

		/* notify upload succeed */
		update_mux_notify_status(up, ARUPDATER_OK);
	break;

	default:
		ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
			"unsupported update mux msg %d", pomp_msg_get_id(msg));
		goto error;
	break;
	}

	pomp_msg_destroy(msg);
	return;

error:
	pomp_msg_destroy(msg);
	update_mux_notify_status(up, ARUPDATER_ERROR_UPLOADER);
	return;
}

static void update_mux_channel_cb(struct mux_ctx *ctx, uint32_t chanid,
		enum mux_channel_event event, struct pomp_buffer *buf,
		void *userdata)
{
	ARUPDATER_Manager_t *mngr = userdata;
	ARUPDATER_Uploader_t *up = mngr->uploader;

	/* ignore message if upload has been canceled */
	if (up->isCanceled)
		return;

	switch (event) {
	case MUX_CHANNEL_RESET:
		update_mux_notify_status(up, ARUPDATER_ERROR_UPLOADER);
	break;
	case MUX_CHANNEL_DATA:
		updater_mux_channel_recv(mngr, buf);
	break;
	}
}

static char *md5_to_str(const uint8_t *md5, char *str)
{
	size_t i;
	for (i = 0; i < ARSAL_MD5_LENGTH; i++)
		snprintf(&str[i * 2], 3, "%02x", md5[i]);

	return str;
}
#endif

eARUPDATER_ERROR ARUPDATER_Uploader_ThreadRunMux(ARUPDATER_Manager_t *manager)
{
#if defined BUILD_LIBMUX
	int res;
	ARUPDATER_PlfVersion v;
	eARUPDATER_ERROR ret, status;
	eARSAL_ERROR aret;
	ARUPDATER_Uploader_t *up = manager->uploader;
	uint16_t product;
	struct stat statbuf;
	char version[128];
	uint8_t md5[ARSAL_MD5_LENGTH];
	char md5_str[2*ARSAL_MD5_LENGTH + 1];
	char dirpath[256];
	char filepath[256];
	char *filename = NULL;
	struct pollfd fds[1];
	int event;
	float percent;

	up->isRunning = 1;
	up->fd = -1;
	up->chunk = NULL;
	memset(md5_str, 0, sizeof(md5_str));
	memset(md5, 0, sizeof(md5));

	ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_UPLOADER_TAG,
			"starting update over mux");

	/* first check we have a mux context */
	if (!up->mux) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
			"Could not upload over usb: no mux instance");
		status = ARUPDATER_ERROR_SYSTEM;
		goto out;
	}

	/* get product id */
	product = ARDISCOVERY_getProductID(up->product);

	/* format directory path containing upload image files */
	snprintf(dirpath, sizeof(dirpath), "%s%s%04x%s", up->rootFolder,
			ARUPDATER_MANAGER_PLF_FOLDER, product,
			ARUPDATER_MANAGER_FOLDER_SEPARATOR);

	/* get image file name */
	ret = ARUPDATER_Utils_GetPlfInFolder(dirpath, &filename);
	if (ret != ARUPDATER_OK) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
			"ARUPDATER_Utils_GetPlfInFolder error %d", ret);
		status = ARUPDATER_ERROR_SYSTEM;
		goto out;
	}

	/* format image file path */
	snprintf(filepath, sizeof(filepath), "%s%s", dirpath, filename);

	/* get update file md5 */
	aret = ARSAL_MD5_Manager_Compute(up->md5Manager, filepath, md5,
			sizeof(md5));
	if (aret != ARSAL_OK) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
			"ARSAL_MD5_Manager_Compute error %d", aret);
		status = ARUPDATER_ERROR_SYSTEM;
		goto out;
	}

	/* get update file version */
	ret = ARUPDATER_Utils_ReadPlfVersion(filepath, &v);
	if (ret != ARUPDATER_OK) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
			"ARUPDATER_Utils_ReadPlfVersion error %d", ret);
		status = ret;
		goto out;
	}

	ARUPDATER_Utils_PlfVersionToString(&v, version, sizeof(version));

	/* open image file */
	up->fd = open(filepath, O_RDONLY);
	if (up->fd < 0) {
		res = errno;
		ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
			"can't open mux update file '%s': error %s", filepath,
			strerror(res));
		status = ARUPDATER_ERROR_PLF;
		goto out;
	}

	/* get file size */
	res = fstat(up->fd, &statbuf);
	if (res < 0) {
		res = errno;
		ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
			"can't stat mux update file '%s': error %s", filepath,
			strerror(res));
		status = ARUPDATER_ERROR_SYSTEM;
		goto out;
	}

	up->size = statbuf.st_size;

	/* allocate chunk buffer */
	up->chunk = malloc(ARUPDATER_UPLOADER_MUX_CHUNK_SIZE);
	if (!up->chunk) {
		status = ARUPDATER_ERROR_ALLOC;
		goto out;
	}

	/* open mux update channel */
	res = mux_channel_open(up->mux, MUX_UPDATE_CHANNEL_ID_UPDATE,
			&update_mux_channel_cb, manager);
	if (res < 0) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG,
			"mux_channel_open: error %s", strerror(-res));
		status = ARUPDATER_ERROR_UPLOADER;
		goto out;
	}

	ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_UPLOADER_TAG, "version:%s "
		"md5:%s size:%zd", version, md5_to_str(md5, md5_str), up->size);

	/* send update request */
	res = updater_mux_write_msg(up->mux, MUX_UPDATE_MSG_ID_UPDATE_REQ,
			MUX_UPDATE_MSG_FMT_ENC_UPDATE_REQ, version, md5,
			sizeof(md5), up->size);
	if (res < 0) {
		status = ARUPDATER_ERROR_UPLOADER;
		goto out;
	}

	/* wait pipe events */
	status = ARUPDATER_ERROR_UPLOADER;
	while (1) {

		/* poll pipe fds */
		fds[0].fd = up->pipefds[0];
		fds[0].events = POLLIN;
		do {
			res = poll(fds, 1, -1);
		} while (res < 0 && errno == EINTR);

		if (res < 0)
			break;

		/* check read pipe data available */
		if (fds[0].revents & POLLIN) {
			/* read pipe status */
			do {
				res = read(up->pipefds[0], &event,
						sizeof(event));
			} while (res < 0 && errno == EINTR);

			if (res != sizeof(event))
				continue;

			/* positive event upload progression */
			if (event > 0) {
				percent = (float)event;
				up->progressCallback(up->progressArg, percent);
			} else {
				/* negative event are status (or error) */
				status = (eARUPDATER_ERROR) event;
				break;
			}
		}
	}

out:
	/* close all */
	if (up->mux)
		mux_channel_close(up->mux, MUX_UPDATE_CHANNEL_ID_UPDATE);

	if (up->fd != -1) {
		close(up->fd);
		up->fd = -1;
	}

	free(filename);
	free(up->chunk);
	up->chunk = NULL;

	/* notify status before return */
	up->isRunning = 0;
	up->completionCallback(up->completionArg, status);

	ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_UPLOADER_TAG,
			"update over mux completed status: %d", status);
	return status;
#else
    return ARUPDATER_ERROR_SYSTEM;
#endif
}

eARUPDATER_ERROR ARUPDATER_Uploader_ThreadRunNormal(ARUPDATER_Manager_t *manager)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    
    if ((manager != NULL) && (manager->uploader != NULL))
    {
        manager->uploader->isRunning = 1;
    } else {
        return ARUPDATER_ERROR_BAD_PARAMETER;
    }
    
    eARDATATRANSFER_ERROR dataTransferError = ARDATATRANSFER_OK;
    eARDATATRANSFER_ERROR dataTransferMd5Error = ARDATATRANSFER_OK;
    char *sourceFileFolder = NULL;
    char *sourceFilePath = NULL;
    char *tmpDestFilePath = NULL;
    char *finalDestFilePath = NULL;
    char *plfDestLocalPath = NULL;
    char *device = NULL;
    char *fileName = NULL;
    char *md5Txt = NULL;
    char *md5RemotePath = NULL;
    char *md5LocalPath = NULL;
    uint8_t *md5 = NULL;
    double pflFileSize = 0.f;
    int existingFinalFile = 0;
    
    uint16_t productId = ARDISCOVERY_getProductID(manager->uploader->product);
    device = malloc(ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE);
    if (device == NULL)
    {
        error = ARUPDATER_ERROR_ALLOC;
    }
    
    if (ARUPDATER_OK == error)
    {
        snprintf(device, ARUPDATER_MANAGER_DEVICE_STRING_MAX_SIZE, "%04x", productId);
        
        sourceFileFolder = malloc(strlen(manager->uploader->rootFolder) + strlen(ARUPDATER_MANAGER_PLF_FOLDER) + strlen(device) + strlen(ARUPDATER_MANAGER_FOLDER_SEPARATOR) + 1);
        if (sourceFileFolder == NULL)
        {
            error = ARUPDATER_ERROR_ALLOC;
        }
        else
        {
            strcpy(sourceFileFolder, manager->uploader->rootFolder);
            strcat(sourceFileFolder, ARUPDATER_MANAGER_PLF_FOLDER);
            strcat(sourceFileFolder, device);
            strcat(sourceFileFolder, ARUPDATER_MANAGER_FOLDER_SEPARATOR);
        }
    }
    
    if (ARUPDATER_OK == error)
    {
        error = ARUPDATER_Utils_GetPlfInFolder(sourceFileFolder, &fileName);
    }
    
    if (ARUPDATER_OK == error)
    {
        int res;
        if (manager->uploader->subfolder)
            res = asprintf(&tmpDestFilePath, "%s%s%s%s",
                           ARUPDATER_UPLOADER_REMOTE_FOLDER,
                           manager->uploader->subfolder,
                           fileName,
                           ARUPDATER_UPLOADER_UPLOADED_FILE_SUFFIX);
        else
            res = asprintf(&tmpDestFilePath, "%s%s%s",
                           ARUPDATER_UPLOADER_REMOTE_FOLDER,
                           fileName,
                           ARUPDATER_UPLOADER_UPLOADED_FILE_SUFFIX);
        if (res < 0) {
            tmpDestFilePath = NULL;
            error = ARUPDATER_ERROR_ALLOC;
        }
    }
    
    if (ARUPDATER_OK == error)
    {
        int res;
        if (manager->uploader->subfolder)
            res = asprintf(&finalDestFilePath, "%s%s%s",
                           ARUPDATER_UPLOADER_REMOTE_FOLDER,
                           manager->uploader->subfolder,
                           fileName);
        else
            res = asprintf(&finalDestFilePath, "%s%s",
                           ARUPDATER_UPLOADER_REMOTE_FOLDER,
                           fileName);
        if (res < 0) {
            finalDestFilePath = NULL;
            error = ARUPDATER_ERROR_ALLOC;
        }
    }
    
    if (ARUPDATER_OK == error)
    {
        sourceFilePath = malloc(strlen(sourceFileFolder) + strlen(fileName) + 1);
        if (sourceFilePath == NULL)
        {
            error = ARUPDATER_ERROR_ALLOC;
        }
        else
        {
            strcpy(sourceFilePath, sourceFileFolder);
            strcat(sourceFilePath, fileName);
        }
    }
    
    if (ARUPDATER_OK == error)
    {
        plfDestLocalPath = malloc(strlen(sourceFileFolder) + strlen(fileName) + 1);
        if (plfDestLocalPath == NULL)
        {
            error = ARUPDATER_ERROR_ALLOC;
        }
        else
        {
            strcpy(plfDestLocalPath, sourceFileFolder);
            strcat(plfDestLocalPath, fileName);
        }
    }
    
    if (ARUPDATER_OK == error)
    {
        md5LocalPath = malloc(strlen(sourceFileFolder) + strlen(ARUPDATER_UPLOADER_MD5_FILENAME) + 1);
        if (md5LocalPath == NULL)
        {
            error = ARUPDATER_ERROR_ALLOC;
        }
        else
        {
            strcpy(md5LocalPath, sourceFileFolder);
            strcat(md5LocalPath, ARUPDATER_UPLOADER_MD5_FILENAME);
        }
    }
    
    if (ARUPDATER_OK == error)
    {
        int res;
        if (manager->uploader->subfolder)
            res = asprintf(&md5RemotePath, "%s%s%s",
                           ARUPDATER_UPLOADER_REMOTE_FOLDER,
                           manager->uploader->subfolder,
                           ARUPDATER_UPLOADER_MD5_FILENAME);
        else
            res = asprintf(&md5RemotePath, "%s%s",
                           ARUPDATER_UPLOADER_REMOTE_FOLDER,
                           ARUPDATER_UPLOADER_MD5_FILENAME);
        if (res < 0) {
            md5RemotePath = NULL;
            error = ARUPDATER_ERROR_ALLOC;
        }
    }

    if (error == ARUPDATER_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_INFO, ARUPDATER_UPLOADER_TAG,
                    "Uploading %s into %s",
                    sourceFilePath, finalDestFilePath);
    }
    
    // get md5 of the plf file to upload
    if (error == ARUPDATER_OK)
    {
        md5 = malloc(ARSAL_MD5_LENGTH);
        if (md5 == NULL)
        {
            error = ARUPDATER_ERROR_ALLOC;
        }
    }
    if (error == ARUPDATER_OK)
    {
        eARSAL_ERROR arsalError = ARSAL_MD5_Manager_Compute(manager->uploader->md5Manager, sourceFilePath, md5, ARSAL_MD5_LENGTH);
        if (ARSAL_OK == arsalError)
        {
            // get md5 in text
            md5Txt = malloc(ARSAL_MD5_LENGTH * 2 + 1);
            int i = 0;
            for (i = 0; i < ARSAL_MD5_LENGTH; i++)
            {
                snprintf(&md5Txt[i * 2], 3, "%02x", md5[i]);
            }
        }
        else
        {
            error = ARUPDATER_ERROR_UPLOADER_ARSAL_ERROR;
        }
    }
    
    if (md5 != NULL)
    {
        free(md5);
        md5 = NULL;
    }
    
    // by default, do not resume an upload
    eARDATATRANSFER_UPLOADER_RESUME resumeMode = ARDATATRANSFER_UPLOADER_RESUME_FALSE;
    // delete the potential md5LocalPath file
    if (md5LocalPath)
         unlink(md5LocalPath);
    
    // read distant plf md5
    ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
    if (ARUPDATER_OK == error)
    {
        dataTransferError = ARDATATRANSFER_Downloader_New(manager->uploader->dataTransferManager, manager->uploader->ftpManager, md5RemotePath, md5LocalPath, NULL, NULL, ARUPDATER_Uploader_CompletionCallback, manager, ARDATATRANSFER_DOWNLOADER_RESUME_FALSE);
        if (ARDATATRANSFER_OK != dataTransferError)
        {
            error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
        }
    }
    ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);
    
    if ((ARUPDATER_OK == error) && (manager->uploader->isCanceled == 0))
    {
        manager->uploader->isDownloadMd5ThreadRunning = 1;
        ARDATATRANSFER_Downloader_ThreadRun(manager->uploader->dataTransferManager);
        manager->uploader->isDownloadMd5ThreadRunning = 0;
        if (manager->uploader->uploadError != ARDATATRANSFER_OK)
        {
            dataTransferMd5Error = manager->uploader->uploadError;
        }
    }
    
    ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
    if (ARUPDATER_OK == error)
    {
        dataTransferError = ARDATATRANSFER_Downloader_Delete(manager->uploader->dataTransferManager);
        if (ARDATATRANSFER_OK != dataTransferError)
        {
            error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
        }
    }
    ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);
    
    // check if an upload was in progress
    if (ARUPDATER_OK == error && ARDATATRANSFER_OK == dataTransferMd5Error)
    {
        // an upload should be resumed if and only if the md5 file is present and the md5 in file match with the plf file md5
        FILE *md5File = fopen(md5LocalPath, "rb");
        if (md5File != NULL)
        {
            int size = 0;
            char line[ARUPDATER_UPLOADER_CHUNK_SIZE];
            int allocatedSize = 1;
            char *uploadedMD5 = malloc(allocatedSize);
            if (uploadedMD5 != NULL)
            {
                strcpy(uploadedMD5, "");
            }
            else
            {
                error = ARUPDATER_ERROR_ALLOC;
            }
            
            while ((ARUPDATER_OK == error) && (size = fread(line, 1, ARUPDATER_UPLOADER_CHUNK_SIZE, md5File)) != 0)
            {
                char *uploadedMD5Reallocated = realloc(uploadedMD5, allocatedSize + size + 1);
                if (uploadedMD5Reallocated != NULL)
                {
                    allocatedSize += size;
                    uploadedMD5 = uploadedMD5Reallocated;
                    strncat(uploadedMD5, line, size);
                    uploadedMD5[allocatedSize] = '\0';
                }
                else
                {
                    error = ARUPDATER_ERROR_ALLOC;
                }
            }
            fclose(md5File);
            md5File = NULL;
            
            if (ARUPDATER_OK == error)
            {
                // md5s match, so we can resume the upload
                if (strcmp(md5Txt, uploadedMD5) == 0)
                {
                    resumeMode = ARDATATRANSFER_UPLOADER_RESUME_TRUE;
                } // ELSE md5s don't match, so keep the default value of resumeMode (=> begin a new upload)
            }
            
            free(uploadedMD5);
            uploadedMD5 = NULL;
            
            // delete the md5LocalPath file
            unlink(md5LocalPath);
        }
    }
    
    //check existing plf
    if ((ARUPDATER_OK == error) && (resumeMode == ARDATATRANSFER_UPLOADER_RESUME_TRUE))
    {
        ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
        if (ARUPDATER_OK == error)
        {
            dataTransferError = ARDATATRANSFER_Downloader_New(manager->uploader->dataTransferManager, manager->uploader->ftpManager, finalDestFilePath, plfDestLocalPath, NULL, NULL, ARUPDATER_Uploader_CompletionCallback, manager, ARDATATRANSFER_DOWNLOADER_RESUME_FALSE);
            if (ARDATATRANSFER_OK != dataTransferError)
            {
                error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
            }
        }
        ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);
        
        if (ARUPDATER_OK == error)
        {
            dataTransferError = ARDATATRANSFER_Downloader_GetSize(manager->uploader->dataTransferManager, &pflFileSize);
            if (ARDATATRANSFER_OK == dataTransferError && pflFileSize > 0.f)
            {
                existingFinalFile = 1;
            }
        }
        
        ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
        if (ARUPDATER_OK == error)
        {
            dataTransferError = ARDATATRANSFER_Downloader_Delete(manager->uploader->dataTransferManager);
            if (ARDATATRANSFER_OK != dataTransferError)
            {
                error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
            }
        }
        ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);
    }
    
    // store in md5LocalPath the md5 of the file that will be uploaded if the upload is a new one
    if ((ARUPDATER_OK == error) && (resumeMode == ARDATATRANSFER_UPLOADER_RESUME_FALSE))
    {
        FILE *md5File = fopen(md5LocalPath, "wb");
        if ((md5File != NULL) && (md5Txt != NULL))
        {
            fprintf(md5File, "%s", md5Txt);
            fclose(md5File);
        }
        else
        {
            error = ARUPDATER_ERROR_UPLOADER;
        }
        
        // create an uploader to upload the md5 file
        ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
        if (ARUPDATER_OK == error)
        {
            dataTransferError = ARDATATRANSFER_Uploader_New(manager->uploader->dataTransferManager, manager->uploader->ftpManager, md5RemotePath, md5LocalPath, NULL, NULL, ARUPDATER_Uploader_CompletionCallback, manager, ARDATATRANSFER_UPLOADER_RESUME_FALSE);
            if (ARDATATRANSFER_OK != dataTransferError)
            {
                error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
            }
        }
        ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);
        
        
        if ((ARUPDATER_OK == error) && (manager->uploader->isCanceled == 0))
        {
            manager->uploader->isUploadThreadRunning = 1;
            ARDATATRANSFER_Uploader_ThreadRun(manager->uploader->dataTransferManager);
            manager->uploader->isUploadThreadRunning = 0;
            if (manager->uploader->uploadError != ARDATATRANSFER_OK)
            {
                error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
            }
        }
        
        ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
        if (ARUPDATER_OK == error)
        {
            dataTransferError = ARDATATRANSFER_Uploader_Delete(manager->uploader->dataTransferManager);
            if (ARDATATRANSFER_OK != dataTransferError)
            {
                error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
            }
        }
        ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);
        
        //we need remove this file in all case
        unlink(md5LocalPath);
    }
    
    if (md5Txt != NULL)
    {
        free(md5Txt);
        md5Txt = NULL;
    }
    
    //existing tmp plf with right md5
    if ((ARUPDATER_OK == error) && (existingFinalFile == 0))
    {
        ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
        // create a new uploader
        if (ARUPDATER_OK == error)
        {
            dataTransferError = ARDATATRANSFER_Uploader_New(manager->uploader->dataTransferManager, manager->uploader->ftpManager, tmpDestFilePath, sourceFilePath, ARUPDATER_Uploader_ProgressCallback, manager, ARUPDATER_Uploader_CompletionCallback, manager, resumeMode);
            if (ARDATATRANSFER_OK != dataTransferError)
            {
                error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
            }
        }
        ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);
        
        
        if ((ARUPDATER_OK == error) && (manager->uploader->isCanceled == 0))
        {
            manager->uploader->isUploadThreadRunning = 1;
            ARDATATRANSFER_Uploader_ThreadRun(manager->uploader->dataTransferManager);
            manager->uploader->isUploadThreadRunning = 0;
            if (manager->uploader->uploadError != ARDATATRANSFER_OK)
            {
                error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
            }
        }
        
        // rename the plf file if the operation went well
        if ((ARUPDATER_OK == error) && (manager->uploader->isCanceled == 0))
        {
            ARDATATRANSFER_Uploader_Rename(manager->uploader->dataTransferManager, tmpDestFilePath, finalDestFilePath);
        }
        
        ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
        if (ARUPDATER_OK == error)
        {
            dataTransferError = ARDATATRANSFER_Uploader_Delete(manager->uploader->dataTransferManager);
            if (ARDATATRANSFER_OK != dataTransferError)
            {
                error = ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR;
            }
        }
        ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);
    }
    
    if (error != ARUPDATER_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_UPLOADER_TAG, "error: %s", ARUPDATER_Error_ToString (error));
    }
    
    if (sourceFileFolder != NULL)
    {
        free(sourceFileFolder);
    }
    if (md5LocalPath != NULL)
    {
        free(md5LocalPath);
    }
    if (md5RemotePath != NULL)
    {
        free(md5RemotePath);
    }
    if (sourceFilePath != NULL)
    {
        free(sourceFilePath);
    }
    if (device != NULL)
    {
        free(device);
    }
    if (fileName != NULL)
    {
        free(fileName);
    }
    if (tmpDestFilePath != NULL)
    {
        free(tmpDestFilePath);
    }
    if (finalDestFilePath != NULL)
    {
        free(finalDestFilePath);
    }
    if (plfDestLocalPath != NULL)
    {
        free(plfDestLocalPath);
    }
    
    if ((manager != NULL) && (manager->uploader != NULL))
    {
        manager->uploader->isRunning = 0;
    }
    
    if (manager->uploader->completionCallback != NULL)
    {
        manager->uploader->completionCallback(manager->uploader->completionArg, error);
    }
    
    return error;
}

void ARUPDATER_Uploader_ProgressCallback(void* arg, float percent)
{
    ARUPDATER_Manager_t *manager = (ARUPDATER_Manager_t *)arg;
    if (manager->uploader->progressCallback != NULL)
    {
        manager->uploader->progressCallback(manager->uploader->progressArg, percent);
    }
}

void ARUPDATER_Uploader_CompletionCallback(void* arg, eARDATATRANSFER_ERROR error)
{
    ARUPDATER_Manager_t *manager = (ARUPDATER_Manager_t *)arg;
    if (manager->uploader != NULL)
    {
        manager->uploader->uploadError = error;
    }
}

eARUPDATER_ERROR ARUPDATER_Uploader_CancelThread(ARUPDATER_Manager_t *manager)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;
    
    if (manager == NULL)
    {
        error = ARUPDATER_ERROR_BAD_PARAMETER;
    }
    
    if ((error == ARUPDATER_OK) && (manager->uploader == NULL))
    {
        error = ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
    }
    
    if (error == ARUPDATER_OK)
    {
        manager->uploader->isCanceled = 1;

#if defined BUILD_LIBMUX
        update_mux_notify_status(manager->uploader, ARUPDATER_ERROR_UPLOADER);
#endif

        ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
        if (manager->uploader->isDownloadMd5ThreadRunning == 1)
        {
            ARDATATRANSFER_Downloader_CancelThread(manager->uploader->dataTransferManager);
        }
        ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);
        
        ARSAL_Mutex_Lock(&manager->uploader->uploadLock);
        if (manager->uploader->isUploadThreadRunning == 1)
        {
            ARDATATRANSFER_Uploader_CancelThread(manager->uploader->dataTransferManager);
        }
        ARSAL_Mutex_Unlock(&manager->uploader->uploadLock);

    }
    
    return error;
}

int ARUPDATER_Uploader_ThreadIsRunning(ARUPDATER_Manager_t* manager, eARUPDATER_ERROR *error)
{
    eARUPDATER_ERROR err = ARUPDATER_OK;
    int isRunning = 0;
    
    if (manager == NULL)
    {
        err = ARUPDATER_ERROR_BAD_PARAMETER;
    }
    
    if ((err == ARUPDATER_OK) && (manager->uploader == NULL))
    {
        err = ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED;
    }
    
    if (err == ARUPDATER_OK)
    {
        isRunning = manager->uploader->isRunning;
    }
    
    if (error != NULL)
    {
        *error = err;
    }
    
    return isRunning;
}
