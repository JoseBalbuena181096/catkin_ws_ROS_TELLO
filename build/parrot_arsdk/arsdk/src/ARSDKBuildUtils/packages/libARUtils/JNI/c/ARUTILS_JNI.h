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
 * @file ARUTILS_JNI.h
 * @brief libARUtils JNI header file.
 * @date 30/12/2013
 * @author david.flattin.ext@parrot.com
 **/

#ifndef _ARUTILS_JNI_H_
#define _ARUTILS_JNI_H_

#ifndef JNI_OK
#define JNI_OK      0
#endif
#ifndef JNI_FAILED
#define JNI_FAILED  -1
#endif

/**
 * @brief JNI JavaVM struct
 */
extern JavaVM* ARUTILS_JNI_Manager_VM;

/**
 * @brief FtpConnection Callbacks structure
 * @param jProgressListener The progress Listener
 * @param jProgressArg The progress Arg object
 * @see Java_com_parrot_arsdk_arutils_ARUtilsFtpConnection_nativeGet
 */
typedef struct _ARUTILS_JNI_FtpConnectionCallbacks_t_
{
    jobject jProgressListener;
    jobject jProgressArg;

} ARUTILS_JNI_FtpConnectionCallbacks_t;

/**
 * @brief JNI HttpConnection structure
 * @param cancelSem The Http cancel semaphore
 * @param httpConnection The Http connection
 * @see Java_com_parrot_arsdk_arutils_ARUtilsHttpConnection_nativeNewHttpConnection
 */
typedef struct _ARUTILS_JNI_HttpConnection_t_
{
    ARSAL_Sem_t cancelSem;
    ARUTILS_Http_Connection_t *httpConnection;

} ARUTILS_JNI_HttpConnection_t;

/**
 * @brief HttpConnection Callbacks structure
 * @param jProgressListener The progress Listener
 * @param jProgressArg The progress Arg object
 * @see Java_com_parrot_arsdk_arutils_ARUtilsHttpConnection_nativeGet
 */
typedef struct _ARUTILS_JNI_HttpConnectionCallbacks_t_
{
    jobject jProgressListener;
    jobject jProgressArg;

} ARUTILS_JNI_HttpConnectionCallbacks_t;

/**
 * @brief Throw a new ARUtilsException
 * @param env The java env
 * @param nativeError The error
 * @retval void
 * @see ARUTILS_JNI_NewARUtilsException
 */
void ARUTILS_JNI_ThrowARUtilsException(JNIEnv *env, eARUTILS_ERROR nativeError);

/**
 * @brief Create a new ARUtilsException
 * @param env The java env
 * @param nativeError The error
 * @retval the new ARUtilsException
 * @see ARUTILS_JNI_ThrowARUtilsException
 */
jobject ARUTILS_JNI_NewARUtilsException(JNIEnv *env, eARUTILS_ERROR nativeError);

/**
 * @brief Get the ARUtilsException JNI class
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARUTILS_JNI_FreeARUtilsExceptionJNI
 */
int ARUTILS_JNI_NewARUtilsExceptionJNI(JNIEnv *env);

/**
 * @brief Free the ARUtilsException JNI class
 * @param env The java env
 * @retval void
 * @see ARDATATRANSFER_JNI_Manager_NewARDataTransferExceptionJNI
 */
void ARUTILS_JNI_FreeARUtilsExceptionJNI(JNIEnv *env);

/**
 * @brief Get the ARUtilsFtpProgressListener JNI class
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARUTILS_JNI_FreeListenersJNI
 */
int ARUTILS_JNI_NewFtpListenersJNI(JNIEnv *env);

/**
 * @brief Free the ARUtilsFtpProgressListener JNI class
 * @param env The java env
 * @retval void
 * @see ARUTILS_JNI_NewListenersJNI
 */
void ARUTILS_JNI_FreeFtpListenersJNI(JNIEnv *env);

/**
 * @brief Get the ARUtilsHttpProgressListener JNI class
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARUTILS_JNI_FreeListenersJNI
 */
int ARUTILS_JNI_NewHttpListenersJNI(JNIEnv *env);

/**
 * @brief Free the ARUtilsHttpProgressListener JNI class
 * @param env The java env
 * @retval void
 * @see ARUTILS_JNI_NewListenersJNI
 */
void ARUTILS_JNI_FreeHttpListenersJNI(JNIEnv *env);

/**
 * @brief Callback that give the file download progress percent
 * @param arg The arg
 * @param percent The progress percent
 * @retval void
 * @see ARDATATRANSFER_JNI_MediasDownloader_FreeListenersJNI
 */
void ARUTILS_JNI_FtpConnection_ProgressCallback(void* arg, float percent);

/**
 * @brief Callback that give the file download progress percent
 * @param arg The arg
 * @param percent The progress percent
 * @retval void
 * @see ARDATATRANSFER_JNI_MediasDownloader_FreeListenersJNI
 */
void ARUTILS_JNI_HttpConnection_ProgressCallback(void* arg, float percent);

#endif /* _ARUTILS_JNI_H_ */
