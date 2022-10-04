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
 * @file ARDATATRANSFER_JNI_Uploader.c
 * @brief libARDataTransfer JNI_Uploader c file.
 * @date 09/06/2014
 * @author david.flattin.ext@parrot.com
 **/
#ifdef NDEBUG
/* Android ndk-build NDK_DEBUG=0*/
#else
/* Android ndk-build NDK_DEBUG=1*/
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
#include <libARUtils/ARUTILS_Manager.h>

#include "libARDataTransfer/ARDATATRANSFER_Error.h"
#include "libARDataTransfer/ARDATATRANSFER_Manager.h"
#include "libARDataTransfer/ARDATATRANSFER_DataDownloader.h"
#include "libARDataTransfer/ARDATATRANSFER_MediasDownloader.h"
#include "libARDataTransfer/ARDATATRANSFER_Downloader.h"
#include "libARDataTransfer/ARDATATRANSFER_Uploader.h"

#include "ARDATATRANSFER_JNI.h"

#define ARDATATRANSFER_JNI_UPLOADER_TAG       "JNI"

jmethodID methodId_DListener_didUploadProgress = NULL;
jmethodID methodId_DListener_didUploadComplete = NULL;


JNIEXPORT jboolean JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferUploader_nativeStaticInit(JNIEnv *env, jclass jClass)
{
    jboolean jret = JNI_FALSE;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_Uploader_NewListenersJNI(env);
    }

    if (error == JNI_OK)
    {
        jret = JNI_TRUE;
    }

    return jret;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferUploader_nativeNew(JNIEnv *env, jobject jThis, jlong jManager, jlong jFtpManager, jstring jRemotePath, jstring jLocalPath, jobject jProgressListener, jobject jProgressArg, jobject jCompletionListener, jobject jCompletionArg, jint jResume)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    ARUTILS_Manager_t *nativeFtpManager = (ARUTILS_Manager_t *)(intptr_t)jFtpManager;
    ARDATATRANSFER_JNI_UploaderCallbacks_t *callbacks = NULL;
    const char *nativeRemotePath = (*env)->GetStringUTFChars(env, jRemotePath, 0);
    const char *nativeLocalPath = (*env)->GetStringUTFChars(env, jLocalPath, 0);
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "%s, %s", nativeRemotePath ? nativeRemotePath : "null", nativeLocalPath ? nativeLocalPath : "null");

    error = ARDATATRANSFER_JNI_Uploader_NewListenersJNI(env);

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_Uploader_NewUploaderCallbacks(env, &callbacks, jProgressListener, jProgressArg, jCompletionListener, jCompletionArg);
    }

    if (error != JNI_OK)
    {
        result = ARDATATRANSFER_ERROR_ALLOC;
    }

    if (result == ARDATATRANSFER_OK)
    {
        result = ARDATATRANSFER_Uploader_New(nativeManager, nativeFtpManager, nativeRemotePath, nativeLocalPath, ARDATATRANSFER_JNI_Uploader_ProgressCallback, callbacks, ARDATATRANSFER_JNI_Uploader_CompletionCallback, callbacks, jResume);
    }

    //cleanup
    if (error != JNI_OK)
    {
        ARDATATRANSFER_JNI_Uploader_FreeUploaderCallbacks(env, &callbacks);
    }

    if (nativeRemotePath != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jRemotePath, nativeRemotePath);
    }

    if (nativeLocalPath != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jLocalPath, nativeLocalPath);
    }

    return result;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferUploader_nativeDelete(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "%s", "");

    result = ARDATATRANSFER_Uploader_Delete(nativeManager);

    ARDATATRANSFER_JNI_Uploader_FreeListenersJNI(env);

    return result;
}

JNIEXPORT void JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferUploader_nativeThreadRun(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "%s", "");

    ARDATATRANSFER_Uploader_ThreadRun(nativeManager);

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "exiting");
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferUploader_nativeCancelThread(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "%s", "");

    result = ARDATATRANSFER_Uploader_CancelThread(nativeManager);

    return result;
}

void ARDATATRANSFER_JNI_Uploader_ProgressCallback(void* arg, float percent)
{
    ARDATATRANSFER_JNI_UploaderCallbacks_t *callbacks = (ARDATATRANSFER_JNI_UploaderCallbacks_t*)arg;

    if (callbacks != NULL)
    {
        if ((ARDATATRANSFER_JNI_Manager_VM != NULL) && (callbacks->jProgressListener != NULL) && (methodId_DListener_didUploadProgress != NULL))
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

            if ((error == JNI_OK) && (methodId_DListener_didUploadProgress != NULL))
            {
                jPercent = percent;

                (*env)->CallVoidMethod(env, callbacks->jProgressListener, methodId_DListener_didUploadProgress, callbacks->jProgressArg, jPercent);
            }

            if ((jResultEnv == JNI_EDETACHED) && (env != NULL))
            {
                 (*ARDATATRANSFER_JNI_Manager_VM)->DetachCurrentThread(ARDATATRANSFER_JNI_Manager_VM);
            }
        }
    }
}

void ARDATATRANSFER_JNI_Uploader_CompletionCallback(void* arg, eARDATATRANSFER_ERROR nativeError)
{
    ARDATATRANSFER_JNI_UploaderCallbacks_t *callbacks = (ARDATATRANSFER_JNI_UploaderCallbacks_t*)arg;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "%x, %d", (int)arg, nativeError);

    if (callbacks != NULL)
    {
        if (ARDATATRANSFER_JNI_Manager_VM != NULL)
        {
            JNIEnv *env = NULL;
            jobject jError = NULL;
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
				ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "error no env");
			}

			if ((env != NULL) && (callbacks->jCompletionListener != NULL) && (methodId_DListener_didUploadComplete != NULL))
			{
				if (error == JNI_OK)
				{
					jError = ARDATATRANSFER_JNI_Manager_NewERROR_ENUM(env, nativeError);

					if (jError == NULL)
					{
						error = JNI_FAILED;
						ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "error %d, %p", error, jError);
					}
				}

				if ((error == JNI_OK) && (methodId_DListener_didUploadComplete != NULL))
				{
					 (*env)->CallVoidMethod(env, callbacks->jCompletionListener, methodId_DListener_didUploadComplete, callbacks->jCompletionArg, jError);
				}
			}

			if (env != NULL)
			{
			    if (jError != NULL)
			    {
			        (*env)->DeleteLocalRef(env, jError);
			    }

				ARDATATRANSFER_JNI_Uploader_FreeUploaderCallbacks(env, &callbacks);
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

int ARDATATRANSFER_JNI_Uploader_NewUploaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_UploaderCallbacks_t **callbacksAddr, jobject jProgressListener, jobject jProgressArg, jobject jCompletionListener, jobject jCompletionArg)
{
    int error = JNI_OK;

    if (callbacksAddr != NULL)
    {
        ARDATATRANSFER_JNI_UploaderCallbacks_t *callbacks = calloc(1, sizeof(ARDATATRANSFER_JNI_UploaderCallbacks_t));
        if (callbacks == NULL)
        {
            error = JNI_FAILED;
        }

        if ((error == JNI_OK) && (jProgressListener != NULL))
        {
            callbacks->jProgressListener = (*env)->NewGlobalRef(env, jProgressListener);
            if (callbacks->jProgressListener == NULL)
            {
                error = JNI_FAILED;
            }
        }

        if ((error == JNI_OK) && (jProgressArg != NULL))
        {
            callbacks->jProgressArg = (*env)->NewGlobalRef(env, jProgressArg);
            if (callbacks->jProgressArg == NULL)
            {
                error = JNI_FAILED;
            }
        }

        if ((error == JNI_OK) && (jCompletionListener != NULL))
        {
            callbacks->jCompletionListener = (*env)->NewGlobalRef(env, jCompletionListener);
            if (callbacks->jCompletionListener == NULL)
            {
                error = JNI_FAILED;
            }
        }

        if ((error == JNI_OK) && (jCompletionArg != NULL))
        {
            callbacks->jCompletionArg = (*env)->NewGlobalRef(env, jCompletionArg);
            if (callbacks->jCompletionArg == NULL)
            {
                error = JNI_FAILED;
            }
        }

        *callbacksAddr = callbacks;
    }
    else
    {
        error = JNI_FAILED;
    }

    return error;
}

void ARDATATRANSFER_JNI_Uploader_FreeUploaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_UploaderCallbacks_t **callbacksAddr)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "%p", callbacksAddr ? *callbacksAddr : 0);

    if (callbacksAddr != NULL)
    {
        ARDATATRANSFER_JNI_UploaderCallbacks_t *callbacks = *callbacksAddr;

        if (callbacks != NULL)
        {
            if (env != NULL)
            {
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

int ARDATATRANSFER_JNI_Uploader_NewListenersJNI(JNIEnv *env)
{
    jclass classDProgressListener = NULL;
    jclass classDCompletionListener = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (methodId_DListener_didUploadProgress == NULL)
    {
        if (error == JNI_OK)
        {
            classDProgressListener = (*env)->FindClass(env, "com/parrot/arsdk/ardatatransfer/ARDataTransferUploaderProgressListener");

            if (classDProgressListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "ARDataTransferUploaderProgressListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DListener_didUploadProgress = (*env)->GetMethodID(env, classDProgressListener, "didUploadProgress", "(Ljava/lang/Object;F)V");

            if (methodId_DListener_didUploadProgress == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "Listener didUploadProgress method not found");
                error = JNI_FAILED;
            }
        }
    }

    if (methodId_DListener_didUploadComplete == NULL)
    {
        if (error == JNI_OK)
        {
            classDCompletionListener = (*env)->FindClass(env, "com/parrot/arsdk/ardatatransfer/ARDataTransferUploaderCompletionListener");

            if (classDCompletionListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "ARDataTransferUploaderCompletionListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DListener_didUploadComplete = (*env)->GetMethodID(env, classDCompletionListener, "didUploadComplete", "(Ljava/lang/Object;Lcom/parrot/arsdk/ardatatransfer/ARDATATRANSFER_ERROR_ENUM;)V");

            if (methodId_DListener_didUploadComplete == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "Listener didUploadComplete method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARDATATRANSFER_JNI_Uploader_FreeListenersJNI(JNIEnv *env)
{
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_UPLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        methodId_DListener_didUploadProgress = NULL;
        methodId_DListener_didUploadComplete = NULL;
    }
}
