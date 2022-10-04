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
 * @file ARDATATRANSFER_JNI.h
 * @brief libARDataTransfer JNI header file.
 * @date 19/12/2013
 * @author david.flattin.ext@parrot.com
 **/

#ifndef _ARDATATRANSFER_JNI_H_
#define _ARDATATRANSFER_JNI_H_

#ifndef JNI_OK
#define JNI_OK      0
#endif
#ifndef JNI_FAILED
#define JNI_FAILED  -1
#endif

extern JavaVM* ARDATATRANSFER_JNI_Manager_VM;
extern jclass classMDMedia;

/**
 * @brief DataDownloader Callbacks structure
 * @param jFileCompletionListener The completion Listener
 * @param jFileCompletionArg The completion Arg object
 * @see ARDATATRANSFER_JNI_DataDownloader_FreeDownloaderCallbacks
 */
typedef struct _ARDATATRANSFER_JNI_DataDownloaderCallbacks_t_
{
    jobject jFileCompletionListener;
    jobject jFileCompletionArg;

} ARDATATRANSFER_JNI_DataDownloaderCallbacks_t;

/**
 * @brief DataDownloader Callbacks structure
 * @param nativeManager The ARDataTransfer manager
 * @param jniDataDownloader the JNY Data Downloader
 * @see ARDATATRANSFER_JNI_DataDownloader_FreeDownloaderCallbacks
 */
typedef struct _ARDATATRANSFER_JNI_Manager_t_
{
    ARDATATRANSFER_Manager_t *nativeManager;
    ARDATATRANSFER_JNI_DataDownloaderCallbacks_t *dataDownloaderCallbacks;

} ARDATATRANSFER_JNI_Manager_t;

/**
 * @brief MediasDownloader Callbacks structure
 * @param jMedia The media
 * @param jProgressListener The progress Listener
 * @param jProgressArg The progress Arg object
 * @param jCompletionListener The completion Listener
 * @param jCompletionArg The completion Arg object
 * @see ARDATATRANSFER_JNI_MediasDownloader_FreeMediasDownloaderCallbacks
 */
typedef struct _ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t_
{
    jobject jMedia;
    jobject jProgressListener;
    jobject jProgressArg;
    jobject jCompletionListener;
    jobject jCompletionArg;
    jobject jAvailableMediaListener;
    jobject jAvailableMediaArg;

} ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t;

/**
 * @brief MediasDownloader Callbacks structure
 * @param jProgressListener The progress Listener
 * @param jProgressArg The progress Arg object
 * @param jCompletionListener The completion Listener
 * @param jCompletionArg The completion Arg object
 * @see ARDATATRANSFER_JNI_Downloader_FreeDownloaderCallbacks
 */
typedef struct _ARDATATRANSFER_JNI_DownloaderCallbacks_t_
{
    jobject jProgressListener;
    jobject jProgressArg;
    jobject jCompletionListener;
    jobject jCompletionArg;

} ARDATATRANSFER_JNI_DownloaderCallbacks_t;

/**
 * @brief MediasUploader Callbacks structure
 * @param jProgressListener The progress Listener
 * @param jProgressArg The progress Arg object
 * @param jCompletionListener The completion Listener
 * @param jCompletionArg The completion Arg object
 * @see ARDATATRANSFER_JNI_Uploader_FreeUploaderCallbacks
 */
typedef struct _ARDATATRANSFER_JNI_UploaderCallbacks_t_
{
    jobject jProgressListener;
    jobject jProgressArg;
    jobject jCompletionListener;
    jobject jCompletionArg;

} ARDATATRANSFER_JNI_UploaderCallbacks_t;

/**
 * @brief Throw a new ARDataTransferException
 * @param env The java env
 * @param nativeError The error
 * @retval void
 * @see ARDATATRANSFER_JNI_Manager_NewARDataTransferException
 */
void ARDATATRANSFER_JNI_Manager_ThrowARDataTransferException(JNIEnv *env, eARDATATRANSFER_ERROR nativeError);

/**
 * @brief Create a new ARDataTransferException
 * @param env The java env
 * @param nativeError The error
 * @retval the new ARDataTransferException
 * @see ARDATATRANSFER_JNI_Manager_ThrowARDataTransferException
 */
jobject ARDATATRANSFER_JNI_Manager_NewARDataTransferException(JNIEnv *env, eARDATATRANSFER_ERROR nativeError);

/**
 * @brief Get the ARDataTransferException JNI class
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARDATATRANSFER_JNI_Manager_FreeARDataTransferExceptionJNI
 */
int ARDATATRANSFER_JNI_Manager_NewARDataTransferExceptionJNI(JNIEnv *env);

/**
 * @brief Free the ARDataTransferException JNI class
 * @param env The java env
 * @retval void
 * @see ARDATATRANSFER_JNI_Manager_NewARDataTransferExceptionJNI
 */
void ARDATATRANSFER_JNI_Manager_FreeARDataTransferExceptionJNI(JNIEnv *env);

/**
 * @brief Create a new ARDATATRANSFER_ERROR_ENUM
 * @param env The java env
 * @param nativeError The error
 * @retval the new ARDATATRANSFER_ERROR_ENUM
 * @see ARDATATRANSFER_JNI_Manager_NewERROR_ENUM_JNI
 */
jobject ARDATATRANSFER_JNI_Manager_NewERROR_ENUM(JNIEnv *env, eARDATATRANSFER_ERROR nativeError);

/**
 * @brief Get the ARDATATRANSFER_ERROR_ENUM JNI class
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARDATATRANSFER_JNI_Manager_FreeERROR_ENUM_JNI
 */
int ARDATATRANSFER_JNI_Manager_NewERROR_ENUM_JNI(JNIEnv *env);

/**
 * @brief Free the ARDATATRANSFER_ERROR_ENUM JNI class
 * @param env The java env
 * @retval void
 * @see ARDATATRANSFER_JNI_Manager_NewERROR_ENUM_JNI
 */
void ARDATATRANSFER_JNI_Manager_FreeERROR_ENUM_JNI(JNIEnv *env);

/**
 * @brief Create a new ARDataTransferMedia
 * @param env The java env
 * @param media The native media
 * @retval the new ARDataTransferMedia
 * @see ARDATATRANSFER_JNI_MediasDownloader_AddMedia
 */
jobject ARDATATRANSFER_JNI_MediasDownloader_NewMedia(JNIEnv *env, ARDATATRANSFER_Media_t *media);

/**
 * @brief Get an ARDataTransferMedia from a native media
 * @param env The java env
 * @param media The native media
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARDATATRANSFER_JNI_MediasDownloader_AddMedia
 */
int ARDATATRANSFER_JNI_MediasDownloader_GetMedia(JNIEnv *env, jobject jMedia, ARDATATRANSFER_Media_t *media);

/**
 * @brief Free the ARDataTransferDataDownloaderFileCompletionListener JNI classes
 * @param env The java env
 * @retval void
 * @see ARDATATRANSFER_JNI_DataDownloader_NewListenersJNI
 */
void ARDATATRANSFER_JNI_DataDownloader_FreeListenersJNI(JNIEnv *env);

/**
 * @brief Get the ARDataTransferDataDownloaderFileCompletionListener JNI classes
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARDATATRANSFER_JNI_DataDownloader_FreeListenersJNI
 */
int ARDATATRANSFER_JNI_DataDownloader_NewListenersJNI(JNIEnv *env);

/**
 * @brief Get the ARDataTransferMedia JNI class
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARDATATRANSFER_JNI_MediasDownloader_FreeMediaJNI
 */
int ARDATATRANSFER_JNI_MediasDownloader_NewMediaJNI(JNIEnv *env);

/**
 * @brief Free the ARDataTransferMedia JNI class
 * @param env The java env
 * @retval void
 * @see ARDATATRANSFER_JNI_MediasDownloader_NewMediaJNI
 */
void ARDATATRANSFER_JNI_MediasDownloader_FreeMediaJNI(JNIEnv *env);

/**
 * @brief Get the ARDataTransferMediasDownloaderProgressListener and ARDataTransferMediasDownloaderCompletionListener JNI classes
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARDATATRANSFER_JNI_MediasDownloader_FreeListenersJNI
 */
int ARDATATRANSFER_JNI_MediasDownloader_NewListenersJNI(JNIEnv *env);

/**
 * @brief Free the ARDataTransferMediasDownloaderProgressListener and ARDataTransferMediasDownloaderCompletionListener JNI classes
 * @param env The java env
 * @retval void
 * @see ARDATATRANSFER_JNI_MediasDownloader_NewListenersJNI
 */
void ARDATATRANSFER_JNI_MediasDownloader_FreeListenersJNI(JNIEnv *env);

/**
 * @brief Get the ARDataTransferDownloaderProgressListener and ARDataTransferDownloaderCompletionListener JNI classes
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARDATATRANSFER_JNI_Downloader_FreeListenersJNI
 */
int ARDATATRANSFER_JNI_Downloader_NewListenersJNI(JNIEnv *env);

/**
 * @brief Get the ARDataTransferDownloaderProgressListener and ARDataTransferDownloaderCompletionListener JNI classes
 * @param env The java env
 * @retval void
 * @see ARDATATRANSFER_JNI_Downloader_FreeListenersJNI
 */
void ARDATATRANSFER_JNI_Downloader_FreeListenersJNI(JNIEnv *env);

/**
 * @brief Free the ARDataTransferUploaderProgressListener and ARDataTransferUploaderCompletionListener JNI classes
 * @param env The java env
 * @retval void
 * @see ARDATATRANSFER_JNI_Uploader_NewListenersJNI
 */
void ARDATATRANSFER_JNI_MediasDownloader_FreeListenersJNI(JNIEnv *env);

/**
 * @brief Get the ARDataTransferUploaderProgressListener and ARDataTransferUploaderCompletionListener JNI classes
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARDATATRANSFER_JNI_Uploader_FreeListenersJNI
 */
int ARDATATRANSFER_JNI_Uploader_NewListenersJNI(JNIEnv *env);

/**
 * @brief Free the ARDataTransferUploaderProgressListener and ARDataTransferUploaderCompletionListener JNI classes
 * @param env The java env
 * @retval void
 * @see ARDATATRANSFER_JNI_Uploader_NewListenersJNI
 */
void ARDATATRANSFER_JNI_Uploader_FreeListenersJNI(JNIEnv *env);

/**
 * @brief Callback that give the file data download completion
 * @param arg The arg
 * @param fileName The file name
 * @param nativeError The error status of the file
 * @retval void
 * @see ARDATATRANSFER_JNI_MediasDownloader_FreeListenersJNI
 */
 void ARDATATRANSFER_JNI_DataDownloader_FileCompletionCallback(void* arg, const char *fileName, eARDATATRANSFER_ERROR nativeError);

/**
 * @brief New Callbacks structure
 * @warning This function frees memory
 * @param env The java env
 * @param callbacks The callbacks structure
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARDATATRANSFER_JNI_DataDownloaderCallbacks_t
 */
int ARDATATRANSFER_JNI_DataDownloader_NewDataDownloaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_DataDownloaderCallbacks_t **callbacksAddr, jobject jFileCompletionListener, jobject jFileCompletionArg);

 /**
 * @brief Free Callbacks structure
 * @warning This function frees memory
 * @param env The java env
 * @param callbacks The callbacks structure
 * @retval void
 * @see ARDATATRANSFER_JNI_DataDownloaderCallbacks_t
 */
void ARDATATRANSFER_JNI_DataDownloader_FreeDataDownloaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_DataDownloaderCallbacks_t **callbacks);

/**
 * @brief Callback that give the media download progress percent
 * @param arg The arg
 * @param media The media
 * @param percent The progress percent
 * @retval void
 * @see ARDATATRANSFER_JNI_MediasDownloader_FreeListenersJNI
 */
void ARDATATRANSFER_JNI_MediasDownloader_ProgressCallback(void* arg, ARDATATRANSFER_Media_t *media, float percent);

/**
 * @brief Callback that give the media download completion status
 * @param arg The arg
 * @param media The media
 * @param nativeError The error status of the media download
 * @retval void
 * @see ARDATATRANSFER_JNI_MediasDownloader_FreeListenersJNI
 */
void ARDATATRANSFER_JNI_MediasDownloader_CompletionCallback(void* arg, ARDATATRANSFER_Media_t *media, eARDATATRANSFER_ERROR nativeError);

/**
 * @brief Callback that give e new discoverd media
 * @param arg The arg
 * @param media The media
 * @param index The media index in the internal medias list
 * @retval void
 * @see ARDATATRANSFER_JNI_MediasDownloader_FreeListenersJNI
 */
void ARDATATRANSFER_JNI_MediasDownloader_AvailableMediaCallback(void* arg, ARDATATRANSFER_Media_t *media, int index);

/**
 * @brief Free Callbacks structure
 * @warning This function frees memory
 * @param env The java env
 * @param callbacks The callbacks structure
 * @retval void
 * @see ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t
 */
void ARDATATRANSFER_JNI_MediasDownloader_FreeMediasDownloaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t **callbacks);

/**
 * @brief Callback that give the download progress persent
 * @param arg The arg
 * @param percent The progress percent
 * @retval void
 * @see ARDATATRANSFER_JNI_Downloader_FreeListenersJNI
 */
void ARDATATRANSFER_JNI_Downloader_ProgressCallback(void* arg, float percent);

/**
 * @brief Callback that give the download completion status
 * @param arg The arg
 * @param nativeError The error status of the media download
 * @retval void
 * @see ARDATATRANSFER_JNI_Downloader_FreeListenersJNI
 */
void ARDATATRANSFER_JNI_Downloader_CompletionCallback(void* arg, eARDATATRANSFER_ERROR nativeError);

/**
 * @brief New Callbacks structure
 * @warning This function frees memory
 * @param env The java env
 * @param callbacksAddr The callbacks structure address
 * @param jProgressListener The progress listener
 * @param jProgressArg The progress arg
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARDATATRANSFER_JNI_DownloaderCallbacks_t
 */
int ARDATATRANSFER_JNI_Downloader_NewDownloaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_DownloaderCallbacks_t **callbacksAddr, jobject jProgressListener, jobject jProgressArg, jobject jCompletionListener, jobject jCompletionArg);

/**
 * @brief Free Callbacks structure
 * @warning This function frees memory
 * @param env The java env
 * @param callbacksAddr The callbacks structure address
 * @retval void
 * @see ARDATATRANSFER_JNI_DownloaderCallbacks_t
 */
void ARDATATRANSFER_JNI_Downloader_FreeDownloaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_DownloaderCallbacks_t **callbacksAddr);

/**
 * @brief Callback that give the download progress persent
 * @param arg The arg
 * @param percent The progress percent
 * @retval void
 * @see ARDATATRANSFER_JNI_Uploader_FreeListenersJNI
 */
void ARDATATRANSFER_JNI_Uploader_ProgressCallback(void* arg, float percent);

/**
 * @brief Callback that give the download completion status
 * @param arg The arg
 * @param nativeError The error status of the media download
 * @retval void
 * @see ARDATATRANSFER_JNI_Uploader_FreeListenersJNI
 */
void ARDATATRANSFER_JNI_Uploader_CompletionCallback(void* arg, eARDATATRANSFER_ERROR nativeError);

/**
 * @brief New Callbacks structure
 * @warning This function frees memory
 * @param env The java env
 * @param callbacksAddr The callbacks structure address
 * @param jProgressListener The progress listener
 * @param jProgressArg The progress arg
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARDATATRANSFER_JNI_UploaderCallbacks_t
 */
int ARDATATRANSFER_JNI_Uploader_NewUploaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_UploaderCallbacks_t **callbacksAddr, jobject jProgressListener, jobject jProgressArg,
 jobject jCompletionListener, jobject jCompletionArg);

/**
 * @brief Free Callbacks structure
 * @warning This function frees memory
 * @param env The java env
 * @param callbacksAddr The callbacks structure address
 * @retval void
 * @see ARDATATRANSFER_JNI_UploaderCallbacks_t
 */
void ARDATATRANSFER_JNI_Uploader_FreeUploaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_UploaderCallbacks_t **callbacksAddr);

#endif /* _ARDATATRANSFER_JNI_H_ */

