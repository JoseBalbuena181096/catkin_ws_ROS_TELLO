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
 * @file ARDATATRANSFER_MediasQueue.h
 * @brief libARDataTransfer MediasQueue header file.
 * @date 19/12/2013
 * @author david.flattin.ext@parrot.com
 **/

#ifndef _ARDATATRANSFER_MEDIASQUEUE_PRIVATE_H_
#define _ARDATATRANSFER_MEDIASQUEUE_PRIVATE_H_

/**
 * @brief Defines the media queue size
 * @see ARDATATRANSFER_Media_t
 */
#define ARDATATRANSFER_MEDIA_QUEUE_SIZE             16

/**
 * @brief FtpMedia structure
 * @param media The media
 * @param progressCallback The media progress callback
 * @param progressArg The media
 * @param completionCallback
 * @param completionArg
 * @see ARDATATRANSFER_MediasQueue_Add ()
 */
typedef struct _ARDATATRANSFER_FtpMedia_t_
{
    ARDATATRANSFER_Media_t media;
    ARDATATRANSFER_MediasDownloader_MediaDownloadProgressCallback_t progressCallback;
    void *progressArg;
    ARDATATRANSFER_MediasDownloader_MediaDownloadCompletionCallback_t completionCallback;
    void *completionArg;

} ARDATATRANSFER_FtpMedia_t;

/**
 * @brief MediasQueue structure
 * @param medias The medias queue list
 * @param lock The mutex to protect the list access
 * @param sem The semaphore to signal new medias in the list
 * @see ARDATATRANSFER_MediasQueue_New ()
 */
typedef struct _ARDATATRANSFER_MediasQueue_t_
{
    ARDATATRANSFER_FtpMedia_t **medias;
    int count;
    ARSAL_Mutex_t lock;

} ARDATATRANSFER_MediasQueue_t;

/**
 * @brief Create a new ARDataTransfer MediasQueue
 * @warning This function allocates memory
 * @param queue The address of the pointer on the ARDataTransfer MediasQueue
 * @retval On success, returns ARDATATRANSFER_OK. Otherwise, it returns an error number of eARDATATRANSFER_ERROR.
 * @see ARDATATRANSFER_MediasQueue_Delete ()
 */
eARDATATRANSFER_ERROR ARDATATRANSFER_MediasQueue_New(ARDATATRANSFER_MediasQueue_t *queue);

/**
 * @brief Delete an ARDataTransfer MediasQueue
 * @warning This function frees memory
 * @param queue The address of the pointer on the ARDataTransfer MediasQueue
 * @see ARDATATRANSFER_MediasQueue_New ()
 */
void ARDATATRANSFER_MediasQueue_Delete(ARDATATRANSFER_MediasQueue_t *queue);

/**
 * @brief Add a new FtpMedia to the ARDataTransfer MediasQueue
 * @warning This function allocates memory
 * @param queue The address of the pointer on the ARDataTransfer MediasQueue
 * @retval On success, returns ARDATATRANSFER_OK. Otherwise, it returns an error number of eARDATATRANSFER_ERROR.
 * @see ARDATATRANSFER_FtpMedia_t, ARDATATRANSFER_MediasQueue_Pop
 */
eARDATATRANSFER_ERROR ARDATATRANSFER_MediasQueue_Add(ARDATATRANSFER_MediasQueue_t *queue, ARDATATRANSFER_FtpMedia_t *ftpMedia);

/**
 * @brief Remove all FtpMedia from the ARDataTransfer MediasQueue
 * @warning This function frees memory
 * @param queue The address of the pointer on the ARDataTransfer MediasQueue
 * @retval On success, returns ARDATATRANSFER_OK. Otherwise, it returns an error number of eARDATATRANSFER_ERROR.
 * @see ARDATATRANSFER_MediasQueue_Add ()
 */
eARDATATRANSFER_ERROR ARDATATRANSFER_MediasQueue_RemoveAll(ARDATATRANSFER_MediasQueue_t *queue);

/**
 * @brief Pop an FtpMedia from ARDATATRANSFER_MediasQueue_Pop if any else wait new FtpMedia
 * @param queue The address of the pointer on the ARDataTransfer MediasQueue
 * @param[out] error The pointer of the error code: if success ARDATATRANSFER_OK, otherwise an error number of eARDATATRANSFER_ERROR
 * @retval On success, returns an new FtpMedia. Otherwise, it returns null.
 * @see ARDATATRANSFER_FtpMedia_t, ARDATATRANSFER_MediasQueue_Add
 */
ARDATATRANSFER_FtpMedia_t * ARDATATRANSFER_MediasQueue_Pop(ARDATATRANSFER_MediasQueue_t *queue, eARDATATRANSFER_ERROR *error);

/**
 * @brief Get a new ARDataTransfer MediasQueue free index in the list, if not allocate new index
 * @warning This function allocates memory
 * @param queue The address of the pointer on the ARDataTransfer MediasQueue
 * @param[out] freeIndex A free index in the list
 * @retval On success, returns ARDATATRANSFER_OK. Otherwise, it returns an error number of eARDATATRANSFER_ERROR.
 * @see ARDATATRANSFER_MediasQueue_Delete ()
 */
eARDATATRANSFER_ERROR ARDATATRANSFER_MediasQueue_GetFreeIndex(ARDATATRANSFER_MediasQueue_t *queue, int *freeIndex);

#endif /* _ARDATATRANSFER_MEDIASQUEUE_PRIVATE_H_ */
