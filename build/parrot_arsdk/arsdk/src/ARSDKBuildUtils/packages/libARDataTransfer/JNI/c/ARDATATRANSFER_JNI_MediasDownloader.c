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
 * @file ARDATATRANSFER_Manager.c
 * @brief libARDataTransfer JNI_MediasDownloader c file.
 * @date 19/12/2013
 * @author david.flattin.ext@parrot.com
 **/

#ifdef NDEBUG
/* Android ndk-build NDK_DEBUG=0 */
#else
/* Android ndk-build NDK_DEBUG=1 */
#ifndef DEBUG
#define DEBUG
#endif
#endif

#include <jni.h>
#include <inttypes.h>
#include <stdlib.h>

#include <libARSAL/ARSAL_Sem.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARUtils/ARUTILS_Error.h>

#include "libARDataTransfer/ARDATATRANSFER_Error.h"
#include "libARDataTransfer/ARDATATRANSFER_Manager.h"
#include "libARDataTransfer/ARDATATRANSFER_DataDownloader.h"
#include "libARDataTransfer/ARDATATRANSFER_MediasDownloader.h"

#include "ARDATATRANSFER_JNI.h"

#define ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG       "JNI"

jclass classMDMedia = NULL;
jmethodID methodId_MDMedia_init = NULL;
jmethodID methodId_MDMedia_getProductValue = NULL;
jmethodID methodId_MDMedia_getName = NULL;
jmethodID methodId_MDMedia_getFilePath = NULL;
jmethodID methodId_MDMedia_getDate = NULL;
jmethodID methodId_MDMedia_getUuid = NULL;
jmethodID methodId_MDMedia_getRemotePath = NULL;
jmethodID methodId_MDMedia_getRemoteThumb = NULL;
jmethodID methodId_MDMedia_getSize = NULL;
jmethodID methodId_MDMedia_getThumbnail = NULL;

jmethodID methodId_MDListener_didMediaProgress = NULL;
jmethodID methodId_MDListener_didMediaComplete = NULL;
jmethodID methodId_MDListener_didMediaAvailable = NULL;

JNIEXPORT jboolean JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeStaticInit(JNIEnv *env, jclass jClass)
{
    jboolean jret = JNI_FALSE;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_MediasDownloader_NewListenersJNI(env);
    }

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_MediasDownloader_NewMediaJNI(env);
    }

    if (error == JNI_OK)
    {
        jret = JNI_TRUE;
    }

    return jret;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeNew(JNIEnv *env, jobject jThis, jlong jManager, jlong jftpListManager, jlong jftpQueueManager, jstring jRemoteDirectory, jstring jLocalDirectory)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    ARUTILS_Manager_t *nativeFtpListManager = (ARUTILS_Manager_t *)(intptr_t)jftpListManager;
    ARUTILS_Manager_t *nativeFtpQueueManager = (ARUTILS_Manager_t *)(intptr_t)jftpQueueManager;
    const char *nativeRemoteDirectory = (*env)->GetStringUTFChars(env, jRemoteDirectory, 0);
    const char *nativeLocalDirectory = (*env)->GetStringUTFChars(env, jLocalDirectory, 0);
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    int error = JNI_OK;

    result = ARDATATRANSFER_MediasDownloader_New(nativeManager, nativeFtpListManager, nativeFtpQueueManager, nativeRemoteDirectory, nativeLocalDirectory);

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_MediasDownloader_NewListenersJNI(env);
    }

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_MediasDownloader_NewMediaJNI(env);
    }

    if (error != JNI_OK)
    {
        result = ARDATATRANSFER_ERROR_SYSTEM;
    }

    //cleanup
    if (nativeRemoteDirectory != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jRemoteDirectory, nativeRemoteDirectory);
    }

    if (nativeLocalDirectory != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jLocalDirectory, nativeLocalDirectory);
    }

    return result;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeDelete(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    result = ARDATATRANSFER_MediasDownloader_Delete(nativeManager);

    return result;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeGetAvailableMediasSync(JNIEnv *env, jobject jThis, jlong jManager, jboolean jWithThumbnail)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    int count = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    count = ARDATATRANSFER_MediasDownloader_GetAvailableMediasSync(nativeManager, (jWithThumbnail == JNI_TRUE) ? 1 : 0, &result);

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "return %d, count %d", result, count);

    if (result != ARDATATRANSFER_OK)
    {
        ARDATATRANSFER_JNI_Manager_ThrowARDataTransferException(env, result);
    }

    return count;
}

JNIEXPORT jobject JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeGetAvailableMediaAtIndex(JNIEnv *env, jobject jThis, jlong jManager, jint jIndex)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    int nativeIndex = (int)jIndex;
    ARDATATRANSFER_Media_t *nativeMedia = NULL;
    jobject jMedia = NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    nativeMedia = ARDATATRANSFER_MediasDownloader_GetAvailableMediaAtIndex(nativeManager, nativeIndex, &result);

    if (result == ARDATATRANSFER_OK)
    {
        jMedia = ARDATATRANSFER_JNI_MediasDownloader_NewMedia(env, nativeMedia);
    }

    if (jMedia == NULL)
    {
        result = ARDATATRANSFER_ERROR_ALLOC;
    }

    if (result != ARDATATRANSFER_OK)
    {
        ARDATATRANSFER_JNI_Manager_ThrowARDataTransferException(env, result);
    }

    return jMedia;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeGetAvailableMediasAsync(JNIEnv *env, jobject jThis, jlong jManager, jobject jAvailableMediaListener, jobject jAvailableMediaArg)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t *callbacks = NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    callbacks = (ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t*)calloc(1, sizeof(ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t));

    if (callbacks == NULL)
    {
        error = JNI_FAILED;
    }
    else
    {
        if (jAvailableMediaListener != NULL)
        {
            callbacks->jAvailableMediaListener = (*env)->NewGlobalRef(env, jAvailableMediaListener);
        }

        if (jAvailableMediaArg != NULL)
        {
            callbacks->jAvailableMediaArg = (*env)->NewGlobalRef(env, jAvailableMediaArg);
        }
    }

    result = ARDATATRANSFER_MediasDownloader_GetAvailableMediasAsync(nativeManager, ARDATATRANSFER_JNI_MediasDownloader_AvailableMediaCallback, callbacks);

    if (error != JNI_OK)
    {
        ARDATATRANSFER_JNI_MediasDownloader_FreeMediasDownloaderCallbacks(env, &callbacks);

        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    return result;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeCancelGetAvailableMedias(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    result = ARDATATRANSFER_MediasDownloader_CancelGetAvailableMedias(nativeManager);

    return result;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeDeleteMedia(JNIEnv *env, jobject jThis, jlong jManager, jobject jMedia)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    ARDATATRANSFER_Media_t nativeMedia;
    int error = JNI_OK;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    error = ARDATATRANSFER_JNI_MediasDownloader_GetMedia(env, jMedia, &nativeMedia);

    if (error != JNI_OK)
    {
        result = ARDATATRANSFER_ERROR_SYSTEM;
    }

    if (result == ARDATATRANSFER_OK)
    {
        result = ARDATATRANSFER_MediasDownloader_DeleteMedia(nativeManager, &nativeMedia, NULL, NULL);
    }

    return result;
}

JNIEXPORT jbyteArray JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeGetMediaThumbnail(JNIEnv *env, jobject jThis, jlong jManager, jobject jMedia)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    ARDATATRANSFER_Media_t nativeMedia;
    jbyteArray jThumbnail = NULL;
    int error = JNI_OK;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    error = ARDATATRANSFER_JNI_MediasDownloader_GetMedia(env, jMedia, &nativeMedia);

    if (error != JNI_OK)
    {
        result = ARDATATRANSFER_ERROR_SYSTEM;
    }

    if (result == ARDATATRANSFER_OK)
    {
        result = ARDATATRANSFER_MediasDownloader_GetThumbnail(nativeManager, &nativeMedia);
    }
    if (result == ARDATATRANSFER_OK)
    {
        
        jThumbnail = (*env)->NewByteArray(env, (&nativeMedia)->thumbnailSize);

        if (jThumbnail == NULL)
        {
            error = JNI_FAILED;
        }
    
        if ((error == JNI_OK) && ((&nativeMedia)->thumbnail != NULL))
        {
            (*env)->SetByteArrayRegion(env, jThumbnail, 0, (&nativeMedia)->thumbnailSize, (jbyte*)(&nativeMedia)->thumbnail);
        }
    }

    return jThumbnail;
}

void ARDATATRANSFER_JNI_MediasDownloader_FreeMediasDownloaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t **callbacksAddr)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%p", callbacksAddr ? *callbacksAddr : 0);

    if (callbacksAddr != NULL)
    {
        ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t *callbacks = *callbacksAddr;

        if (callbacks != NULL)
        {
            if (env != NULL)
            {
                if (callbacks->jMedia != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jMedia);
                }

                if (callbacks->jProgressListener != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jProgressListener);
                }

                if (callbacks->jProgressArg != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jProgressArg);
                }

                if (callbacks->jCompletionListener != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jCompletionListener);
                }

                if (callbacks->jCompletionArg != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jCompletionArg);
                }
            }

            free(callbacks);
        }

        *callbacksAddr = NULL;
    }
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeAddMediaToQueue(JNIEnv *env, jobject jThis, jlong jManager, jobject jMedia, jobject jProgressListener, jobject jProgressArg, jobject jCompletionListener, jobject jCompletionArg)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t *callbacks = NULL;
    ARDATATRANSFER_Media_t nativeMedia;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%x, %x", (int)nativeManager, (int)jMedia);

    callbacks = (ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t*)calloc(1, sizeof(ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t));

    if (callbacks == NULL)
    {
        error = JNI_FAILED;
    }
    else
    {
        if (jMedia != NULL)
        {
            callbacks->jMedia = (*env)->NewGlobalRef(env, jMedia);
        }

        if (jProgressListener != NULL)
        {
            callbacks->jProgressListener = (*env)->NewGlobalRef(env, jProgressListener);
        }

        if (jProgressArg != NULL)
        {
            callbacks->jProgressArg = (*env)->NewGlobalRef(env, jProgressArg);
        }

        if (jCompletionListener != NULL)
        {
            callbacks->jCompletionListener = (*env)->NewGlobalRef(env, jCompletionListener);
        }

        if (jCompletionArg != NULL)
        {
            callbacks->jCompletionArg = (*env)->NewGlobalRef(env, jCompletionArg);
        }
    }

    if (error == JNI_OK)
    {
        memset(&nativeMedia, 0, sizeof(ARDATATRANSFER_Media_t));

        error = ARDATATRANSFER_JNI_MediasDownloader_GetMedia(env, jMedia, &nativeMedia);
    }

    if (error == JNI_OK)
    {
        result = ARDATATRANSFER_MediasDownloader_AddMediaToQueue(nativeManager, &nativeMedia, ARDATATRANSFER_JNI_MediasDownloader_ProgressCallback, callbacks, ARDATATRANSFER_JNI_MediasDownloader_CompletionCallback, callbacks);
    }

    if (error != JNI_OK)
    {
        ARDATATRANSFER_JNI_MediasDownloader_FreeMediasDownloaderCallbacks(env, &callbacks);

        result = ARDATATRANSFER_ERROR_BAD_PARAMETER;
    }

    return result;
}

JNIEXPORT void JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeQueueThreadRun(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    ARDATATRANSFER_MediasDownloader_QueueThreadRun(nativeManager);

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "exit");
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferMediasDownloader_nativeCancelQueueThread(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    result = ARDATATRANSFER_MediasDownloader_CancelQueueThread(nativeManager);

    return result;
}

void ARDATATRANSFER_JNI_MediasDownloader_ProgressCallback(void* arg, ARDATATRANSFER_Media_t *media, float percent)
{
    ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t *callbacks = (ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t*)arg;

    if (callbacks != NULL)
    {
        if ((ARDATATRANSFER_JNI_Manager_VM != NULL) && (callbacks->jProgressListener != NULL) && (methodId_MDListener_didMediaProgress != NULL))
        {
            JNIEnv *env = NULL;
            jfloat jPercent = 0;
            jint jResultEnv = 0;
            int error = JNI_OK;

            jResultEnv = (*ARDATATRANSFER_JNI_Manager_VM)->GetEnv(ARDATATRANSFER_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);

            if (jResultEnv == JNI_EDETACHED)
            {
                 (*ARDATATRANSFER_JNI_Manager_VM)->AttachCurrentThread(ARDATATRANSFER_JNI_Manager_VM, &env, NULL);
            }

            if (env == NULL)
            {
                error = JNI_FAILED;
            }

            if ((error == JNI_OK) && (methodId_MDListener_didMediaProgress != NULL))
            {
                jPercent = percent;

                (*env)->CallVoidMethod(env, callbacks->jProgressListener, methodId_MDListener_didMediaProgress, callbacks->jProgressArg, callbacks->jMedia, jPercent);
            }

            if ((jResultEnv == JNI_EDETACHED) && (env != NULL))
            {
                 (*ARDATATRANSFER_JNI_Manager_VM)->DetachCurrentThread(ARDATATRANSFER_JNI_Manager_VM);
            }
        }
    }
}

void ARDATATRANSFER_JNI_MediasDownloader_CompletionCallback(void* arg, ARDATATRANSFER_Media_t *media, eARDATATRANSFER_ERROR nativeError)
{
    ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t *callbacks = (ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t*)arg;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%x, %s, %d", (int)arg, media ? media->name : "null", nativeError);

    if (callbacks != NULL)
    {
        if (ARDATATRANSFER_JNI_Manager_VM != NULL)
        {
            JNIEnv *env = NULL;
            jobject jError = NULL;
			jint jResultEnv = 0;

			jResultEnv = (*ARDATATRANSFER_JNI_Manager_VM)->GetEnv(ARDATATRANSFER_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);

			if (jResultEnv == JNI_EDETACHED)
			{
				 (*ARDATATRANSFER_JNI_Manager_VM)->AttachCurrentThread(ARDATATRANSFER_JNI_Manager_VM, &env, NULL);
			}

			if (env == NULL)
			{
				//error = JNI_FAILED;
				ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "error no env");
			}

			if ((env != NULL) && (callbacks->jCompletionListener != NULL) && (methodId_MDListener_didMediaComplete != NULL))
			{
				int error = JNI_OK;


				if (error == JNI_OK)
				{
					jError = ARDATATRANSFER_JNI_Manager_NewERROR_ENUM(env, nativeError);

					if (jError == NULL)
					{
						error = JNI_FAILED;
						ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "error %d, %p", error, jError);
					}
				}

				if ((error == JNI_OK) && (methodId_MDListener_didMediaComplete != NULL))
				{
					 (*env)->CallVoidMethod(env, callbacks->jCompletionListener, methodId_MDListener_didMediaComplete, callbacks->jCompletionArg, callbacks->jMedia, jError);
				}
			}

			if (env != NULL)
			{
			    if (jError != NULL)
			    {
			        (*env)->DeleteLocalRef(env, jError);
			    }

				ARDATATRANSFER_JNI_MediasDownloader_FreeMediasDownloaderCallbacks(env, &callbacks);
			}

			if ((jResultEnv == JNI_EDETACHED) && (env != NULL))
			{
				(*ARDATATRANSFER_JNI_Manager_VM)->DetachCurrentThread(ARDATATRANSFER_JNI_Manager_VM);
			}
        }

		if (callbacks != NULL)
		{
        	free(callbacks);
        }
    }
}

void ARDATATRANSFER_JNI_MediasDownloader_AvailableMediaCallback(void* arg, ARDATATRANSFER_Media_t *media, int index)
{
    ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t *callbacks = (ARDATATRANSFER_JNI_MediasDownloaderCallbacks_t*)arg;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%d %x, %s", index, (int)arg, media ? media->name : "null");

    if (callbacks != NULL)
    {
        if (ARDATATRANSFER_JNI_Manager_VM != NULL)
        {
            JNIEnv *env = NULL;
			jint jResultEnv = 0;
			jobject jMedia = NULL;
			int error = JNI_OK;

			jResultEnv = (*ARDATATRANSFER_JNI_Manager_VM)->GetEnv(ARDATATRANSFER_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);

			if (jResultEnv == JNI_EDETACHED)
			{
				 (*ARDATATRANSFER_JNI_Manager_VM)->AttachCurrentThread(ARDATATRANSFER_JNI_Manager_VM, &env, NULL);
			}

			if (env == NULL)
			{
				error = JNI_FAILED;
				ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "error no env");
			}

			if (error == JNI_OK)
			{
			    jMedia = ARDATATRANSFER_JNI_MediasDownloader_NewMedia(env, media);

			    if (jMedia == NULL)
			    {
                    error = JNI_FAILED;
			    }
			}

			if ((error == JNI_OK) && (env != NULL) && (callbacks->jAvailableMediaListener != NULL) && (methodId_MDListener_didMediaAvailable != NULL))
			{
				(*env)->CallVoidMethod(env, callbacks->jAvailableMediaListener, methodId_MDListener_didMediaAvailable, callbacks->jAvailableMediaArg, jMedia, (jint)index);
			}

	        if ((env != NULL) && (jMedia != NULL))
            {
                (*env)->DeleteLocalRef(env, jMedia);
            }

			if ((jResultEnv == JNI_EDETACHED) && (env != NULL))
			{
				(*ARDATATRANSFER_JNI_Manager_VM)->DetachCurrentThread(ARDATATRANSFER_JNI_Manager_VM);
			}
        }
    }
}

int ARDATATRANSFER_JNI_MediasDownloader_NewMediaJNI(JNIEnv *env)
{
    jclass locClassMDMedia = NULL;
    int error = JNI_OK;

    if (classMDMedia == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

        if (env == NULL)
        {
            error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            locClassMDMedia = (*env)->FindClass(env, "com/parrot/arsdk/ardatatransfer/ARDataTransferMedia");

            if (locClassMDMedia == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "ARDataTransferMedia class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            classMDMedia = (*env)->NewGlobalRef(env, locClassMDMedia);

            if (classMDMedia == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "ARDataTransferMedia global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDMedia_init = (*env)->GetMethodID(env, classMDMedia, "<init>", "(ILjava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;F[B)V");

            if (methodId_MDMedia_init == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Media <init> method not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDMedia_getProductValue = (*env)->GetMethodID(env, classMDMedia, "getProductValue", "()I");

            if (methodId_MDMedia_getProductValue == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Media getProductValue method not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDMedia_getName = (*env)->GetMethodID(env, classMDMedia, "getName", "()Ljava/lang/String;");

            if (methodId_MDMedia_getName == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Media getName method not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDMedia_getFilePath = (*env)->GetMethodID(env, classMDMedia, "getFilePath", "()Ljava/lang/String;");

            if (methodId_MDMedia_getFilePath == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Media getFilePath method not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDMedia_getDate = (*env)->GetMethodID(env, classMDMedia, "getDate", "()Ljava/lang/String;");

            if (methodId_MDMedia_getDate == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Media getDate method not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDMedia_getUuid = (*env)->GetMethodID(env, classMDMedia, "getUUID", "()Ljava/lang/String;");

            if (methodId_MDMedia_getUuid == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Media getUUID method not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDMedia_getRemotePath = (*env)->GetMethodID(env, classMDMedia, "getRemotePath", "()Ljava/lang/String;");

            if (methodId_MDMedia_getRemotePath == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Media getRemotePath method not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDMedia_getRemoteThumb = (*env)->GetMethodID(env, classMDMedia, "getRemoteThumb", "()Ljava/lang/String;");

            if (methodId_MDMedia_getRemoteThumb == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Media getRemoteThumb method not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDMedia_getSize = (*env)->GetMethodID(env, classMDMedia, "getSize", "()F");

            if (methodId_MDMedia_getSize == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Media getSize method not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDMedia_getThumbnail = (*env)->GetMethodID(env, classMDMedia, "getThumbnail", "()[B");

            if (methodId_MDMedia_getThumbnail == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Media getThumbnail method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARDATATRANSFER_JNI_MediasDownloader_FreeMediaJNI(JNIEnv *env)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    if (env != NULL)
    {
        if (classMDMedia != NULL)
        {
            (*env)->DeleteGlobalRef(env, classMDMedia);
            classMDMedia = NULL;
        }

        methodId_MDMedia_init = NULL;
        methodId_MDMedia_getProductValue = NULL;
        methodId_MDMedia_getName = NULL;
        methodId_MDMedia_getFilePath = NULL;
        methodId_MDMedia_getDate = NULL;
        methodId_MDMedia_getUuid = NULL;
        methodId_MDMedia_getRemotePath = NULL;
        methodId_MDMedia_getRemoteThumb = NULL;
        methodId_MDMedia_getSize = NULL;
        methodId_MDMedia_getThumbnail = NULL;
    }
}

int ARDATATRANSFER_JNI_MediasDownloader_GetMedia(JNIEnv *env, jobject jMedia, ARDATATRANSFER_Media_t *media)
{
    jint jProduct = 0;
    jstring jName = NULL;
    jstring jFilePath = NULL;
    jstring jDate = NULL;
    jstring jUuid = NULL;
    jstring jRemotePath = NULL;
    jstring jRemoteThumb = NULL;
    jfloat jSize = 0.f;
    const char *nativeName = NULL;
    const char *nativeFilePath = NULL;
    const char *nativeDate = NULL;
    const char *nativeUuid = NULL;
    const char *nativeRemotePath = NULL;
    const char *nativeRemoteThumb = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    if ((env == NULL) || (jMedia == NULL) || (media == NULL))
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "wong parameters");
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        if ((classMDMedia == NULL) ||
            (methodId_MDMedia_getProductValue == NULL) ||
            (methodId_MDMedia_getName == NULL) ||
            (methodId_MDMedia_getFilePath == NULL) ||
            (methodId_MDMedia_getDate == NULL) ||
            (methodId_MDMedia_getUuid == NULL) ||
            (methodId_MDMedia_getRemotePath == NULL) ||
            (methodId_MDMedia_getRemoteThumb == NULL) ||
            (methodId_MDMedia_getSize == NULL))
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "wrong JNI parameters");
            error = JNI_FAILED;
        }
    }

    if (error == JNI_OK)
    {
        jProduct = (*env)->CallIntMethod(env, jMedia, methodId_MDMedia_getProductValue);
        jName = (*env)->CallObjectMethod(env, jMedia, methodId_MDMedia_getName);
        jFilePath = (*env)->CallObjectMethod(env, jMedia, methodId_MDMedia_getFilePath);
        jDate = (*env)->CallObjectMethod(env, jMedia, methodId_MDMedia_getDate);
        jUuid = (*env)->CallObjectMethod(env, jMedia, methodId_MDMedia_getUuid);
        jRemotePath = (*env)->CallObjectMethod(env, jMedia, methodId_MDMedia_getRemotePath);
        jRemoteThumb = (*env)->CallObjectMethod(env, jMedia, methodId_MDMedia_getRemoteThumb);
        jSize = (*env)->CallFloatMethod(env, jMedia, methodId_MDMedia_getSize);
    }

    if ((error == JNI_OK) && (jName != NULL))
    {
        nativeName = (*env)->GetStringUTFChars(env, jName, 0);
    }

    if ((error == JNI_OK) && (jFilePath != NULL))
    {
        nativeFilePath = (*env)->GetStringUTFChars(env, jFilePath, 0);
    }

    if ((error == JNI_OK) && (jDate != NULL))
    {
        nativeDate = (*env)->GetStringUTFChars(env, jDate, 0);
    }

    if ((error == JNI_OK) && (jUuid != NULL))
    {
        nativeUuid = (*env)->GetStringUTFChars(env, jUuid, 0);
    }

    if ((error == JNI_OK) && (jRemotePath != NULL))
    {
        nativeRemotePath = (*env)->GetStringUTFChars(env, jRemotePath, 0);
    }

    if ((error == JNI_OK) && (jRemoteThumb != NULL))
    {
        nativeRemoteThumb = (*env)->GetStringUTFChars(env, jRemoteThumb, 0);
    }


    if (error == JNI_OK)
    {
        media->product = jProduct;
        strcpy(media->name, nativeName ? nativeName : "");
        strcpy(media->filePath, nativeFilePath ? nativeFilePath : "");
        strcpy(media->date, nativeDate ? nativeDate : "");
        strcpy(media->uuid, nativeUuid ? nativeUuid : "");
        strcpy(media->remotePath, nativeRemotePath ? nativeRemotePath : "");
        strcpy(media->remoteThumb, nativeRemoteThumb ? nativeRemoteThumb : "");
        media->size = (double)jSize;
    }

    //cleanup
    if (nativeName != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jName, nativeName);
    }

    if (nativeFilePath != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jFilePath, nativeFilePath);
    }

    if (nativeDate != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jDate, nativeDate);
    }

    if (nativeUuid != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jUuid, nativeUuid);
    }

    if (nativeRemotePath != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jRemotePath, nativeRemotePath);
    }

    if (nativeRemoteThumb != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jRemoteThumb, nativeRemoteThumb);
    }

    return error;
}

jobject ARDATATRANSFER_JNI_MediasDownloader_NewMedia(JNIEnv *env, ARDATATRANSFER_Media_t *media)
{
    jobject jMedia = NULL;
    jstring jName = NULL;
    jstring jFilePath = NULL;
    jstring jDate = NULL;
    jstring jUuid = NULL;
    jstring jRemotePath = NULL;
    jstring jRemoteThumb = NULL;
    jbyteArray jThumbnail = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", media->name);

    if ((env == NULL) || (media == NULL))
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        if ((classMDMedia == NULL) || (methodId_MDMedia_init == NULL))
        {
            error = JNI_FAILED;
        }
    }

    if (error == JNI_OK)
    {
        jName = (*env)->NewStringUTF(env, media->name);

        if (jName == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if (error == JNI_OK)
    {
        jFilePath = (*env)->NewStringUTF(env, media->filePath);

        if (jFilePath == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if (error == JNI_OK)
    {
        jDate = (*env)->NewStringUTF(env, media->date);

        if (jDate == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if (error == JNI_OK)
    {
        jUuid = (*env)->NewStringUTF(env, media->uuid);

        if (jUuid == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if (error == JNI_OK)
    {
        jRemotePath = (*env)->NewStringUTF(env, media->remotePath);

        if (jRemotePath == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if (error == JNI_OK)
    {
        jRemoteThumb = (*env)->NewStringUTF(env, media->remoteThumb);

        if (jRemoteThumb == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if (error == JNI_OK)
    {
        jThumbnail = (*env)->NewByteArray(env, media->thumbnailSize);

        if (jThumbnail == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if ((error == JNI_OK) && (media->thumbnail != NULL))
    {
        (*env)->SetByteArrayRegion(env, jThumbnail, 0, media->thumbnailSize, (jbyte*)media->thumbnail);
    }

    if (error == JNI_OK)
    {
        jMedia = (*env)->NewObject(env, classMDMedia, methodId_MDMedia_init, (jint)media->product, jName, jFilePath, jDate, jUuid, jRemotePath, jRemoteThumb, (jfloat)media->size, jThumbnail);
    }

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "return jMedia %d", (int)jMedia);

    // clean local refs

    if (jName != NULL)
    {
        (*env)->DeleteLocalRef(env, jName);
    }

    if (jFilePath != NULL)
    {
        (*env)->DeleteLocalRef(env, jFilePath);
    }

    if (jDate != NULL)
    {
        (*env)->DeleteLocalRef(env, jDate);
    }

    if (jUuid != NULL)
    {
        (*env)->DeleteLocalRef(env, jUuid);
    }

    if (jRemotePath != NULL)
    {
        (*env)->DeleteLocalRef(env, jRemotePath);
    }

    if (jRemoteThumb != NULL)
    {
        (*env)->DeleteLocalRef(env, jRemoteThumb);
    }

    if (jThumbnail != NULL)
    {
        (*env)->DeleteLocalRef(env, jThumbnail);
    }

    return jMedia;
}

int ARDATATRANSFER_JNI_MediasDownloader_NewListenersJNI(JNIEnv *env)
{
    jclass classMDProgressListener = NULL;
    jclass classMDCompletionListener = NULL;
    jclass classMDAvailableListener = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (methodId_MDListener_didMediaProgress == NULL)
    {
        if (error == JNI_OK)
        {
            classMDProgressListener = (*env)->FindClass(env, "com/parrot/arsdk/ardatatransfer/ARDataTransferMediasDownloaderProgressListener");

            if (classMDProgressListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "ARDataTransferMediasDownloaderProgressListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDListener_didMediaProgress = (*env)->GetMethodID(env, classMDProgressListener, "didMediaProgress", "(Ljava/lang/Object;Lcom/parrot/arsdk/ardatatransfer/ARDataTransferMedia;F)V");

            if (methodId_MDListener_didMediaProgress == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Listener didProgress method not found");
                error = JNI_FAILED;
            }
        }
    }

    if (methodId_MDListener_didMediaComplete == NULL)
    {
        if (error == JNI_OK)
        {
            classMDCompletionListener = (*env)->FindClass(env, "com/parrot/arsdk/ardatatransfer/ARDataTransferMediasDownloaderCompletionListener");

            if (classMDCompletionListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "ARDataTransferMediasDownloaderCompletionListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDListener_didMediaComplete = (*env)->GetMethodID(env, classMDCompletionListener, "didMediaComplete", "(Ljava/lang/Object;Lcom/parrot/arsdk/ardatatransfer/ARDataTransferMedia;Lcom/parrot/arsdk/ardatatransfer/ARDATATRANSFER_ERROR_ENUM;)V");

            if (methodId_MDListener_didMediaComplete == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Listener didComplete method not found");
                error = JNI_FAILED;
            }
        }
    }

    if (methodId_MDListener_didMediaAvailable == NULL)
    {
        if (error == JNI_OK)
        {
            classMDAvailableListener = (*env)->FindClass(env, "com/parrot/arsdk/ardatatransfer/ARDataTransferMediasDownloaderAvailableMediaListener");

            if (classMDAvailableListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "ARDataTransferMediasDownloaderAvailableMediaListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_MDListener_didMediaAvailable = (*env)->GetMethodID(env, classMDAvailableListener, "didMediaAvailable", "(Ljava/lang/Object;Lcom/parrot/arsdk/ardatatransfer/ARDataTransferMedia;I)V");

            if (methodId_MDListener_didMediaAvailable == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "Listener didMediaAvailable method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARDATATRANSFER_JNI_MediasDownloader_FreeListenersJNI(JNIEnv *env)
{
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MEDIADOWNLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        methodId_MDListener_didMediaProgress = NULL;
        methodId_MDListener_didMediaComplete = NULL;
        methodId_MDListener_didMediaAvailable = NULL;
    }
}

