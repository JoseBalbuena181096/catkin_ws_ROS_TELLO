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
 * @brief libARDataTransfer JNI_DataDownloader c file.
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

#include "libARDataTransfer/ARDATATRANSFER_Error.h"
#include "libARDataTransfer/ARDATATRANSFER_Manager.h"
#include "libARDataTransfer/ARDATATRANSFER_DataDownloader.h"
#include "libARDataTransfer/ARDATATRANSFER_MediasDownloader.h"

#include "ARDATATRANSFER_JNI.h"

#define ARDATATRANSFER_JNI_DATADOWNLOADER_TAG       "JNI"


jmethodID methodId_DDListener_didDataDownloaderFileComplete = NULL;

JNIEXPORT jboolean Java_com_parrot_arsdk_ardatatransfer_ARDataTransferDataDownloader_nativeStaticInit(JNIEnv *env, jclass jClass)
{
    jboolean jret = JNI_FALSE;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_DataDownloader_NewListenersJNI(env);
    }

    if (error == JNI_OK)
    {
        jret = JNI_TRUE;
    }

    return jret;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferDataDownloader_nativeNew(JNIEnv *env, jobject jThis, jlong jManager, jlong jUtilsListManager, jlong jUtilsDataManager, jstring jRemoteDirectory, jstring jLocalDirectory, jobject fileCompletionListener, jobject fileCompletionArg)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    ARUTILS_Manager_t *nativeUtilsListManager = (ARUTILS_Manager_t*)(intptr_t)jUtilsListManager;
    ARUTILS_Manager_t *nativeUtilsDataManager = (ARUTILS_Manager_t*)(intptr_t)jUtilsDataManager;
    const char *nativeRemoteDirectory = (*env)->GetStringUTFChars(env, jRemoteDirectory, 0);
    const char *nativeLocalDirectory = (*env)->GetStringUTFChars(env, jLocalDirectory, 0);
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%s, %s", nativeRemoteDirectory ? nativeRemoteDirectory : "null", nativeLocalDirectory ? nativeLocalDirectory : "null");

    error = ARDATATRANSFER_JNI_DataDownloader_NewListenersJNI(env);

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_DataDownloader_NewDataDownloaderCallbacks(env, &nativeJniManager->dataDownloaderCallbacks, fileCompletionListener, fileCompletionArg);
    }

    if (error != JNI_OK)
    {
        result = ARDATATRANSFER_ERROR_ALLOC;
    }

    if (result == ARDATATRANSFER_OK)
    {
        result = ARDATATRANSFER_DataDownloader_New(nativeManager, nativeUtilsListManager, nativeUtilsDataManager, nativeRemoteDirectory, nativeLocalDirectory, ARDATATRANSFER_JNI_DataDownloader_FileCompletionCallback, nativeJniManager->dataDownloaderCallbacks);
    }

    //cleanup
    if (error != JNI_OK)
    {
        ARDATATRANSFER_JNI_DataDownloader_FreeDataDownloaderCallbacks(env, &nativeJniManager->dataDownloaderCallbacks);
    }

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

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferDataDownloader_nativeDelete(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%s", "");

    result = ARDATATRANSFER_DataDownloader_Delete(nativeManager);

    ARDATATRANSFER_JNI_DataDownloader_FreeDataDownloaderCallbacks(env, &nativeJniManager->dataDownloaderCallbacks);
    ARDATATRANSFER_JNI_DataDownloader_FreeListenersJNI(env);

    return result;
}

JNIEXPORT jlong JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferDataDownloader_nativeGetAvailableFiles(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    long filesNumber = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%s", "");

    result = ARDATATRANSFER_DataDownloader_GetAvailableFiles(nativeManager, &filesNumber);

    if (result != ARDATATRANSFER_OK)
    {
        ARDATATRANSFER_JNI_Manager_ThrowARDataTransferException(env, result);
    }

    return (jlong)filesNumber;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferDataDownloader_nativeCancelAvailableFiles(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%s", "");

    result = ARDATATRANSFER_DataDownloader_CancelAvailableFiles(nativeManager);

    return result;
}

JNIEXPORT void JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferDataDownloader_nativeThreadRun(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%s", "");

    ARDATATRANSFER_DataDownloader_ThreadRun(nativeManager);

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "exit");
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferDataDownloader_nativeCancelThread(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;
    ARDATATRANSFER_Manager_t *nativeManager = (nativeJniManager->nativeManager) ? nativeJniManager->nativeManager : NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%s", "");

    result = ARDATATRANSFER_DataDownloader_CancelThread(nativeManager);

    return result;
}

/*****************************************
 *
 *             Private implementation:
 *
 *****************************************/

void ARDATATRANSFER_JNI_DataDownloader_FileCompletionCallback(void* arg, const char *fileName, eARDATATRANSFER_ERROR nativeError)
{
    ARDATATRANSFER_JNI_DataDownloaderCallbacks_t *callbacks = (ARDATATRANSFER_JNI_DataDownloaderCallbacks_t*)arg;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%x, %s, %d", (int)arg, (fileName) ? fileName : "null", nativeError);

    if (callbacks != NULL)
    {
        if (ARDATATRANSFER_JNI_Manager_VM != NULL)
        {
            JNIEnv *env = NULL;
            jstring jFileName = NULL;
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
				ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "error no env");
			}

			if ((env != NULL) && (callbacks->jFileCompletionListener != NULL) && (methodId_DDListener_didDataDownloaderFileComplete != NULL))
			{
				int error = JNI_OK;

				if ((error == JNI_OK) && (fileName != NULL))
				{
                    jFileName = (*env)->NewStringUTF(env, fileName);
                    if (jFileName == NULL)
                    {
                        error = JNI_FAILED;
                        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "error %d", error);
                    }
                }

				if (error == JNI_OK)
				{
					jError = ARDATATRANSFER_JNI_Manager_NewERROR_ENUM(env, nativeError);

					if (jError == NULL)
					{
						error = JNI_FAILED;
						ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "error %d", error);
					}
				}

				if ((error == JNI_OK) && (methodId_DDListener_didDataDownloaderFileComplete != NULL))
				{
					 (*env)->CallVoidMethod(env, callbacks->jFileCompletionListener, methodId_DDListener_didDataDownloaderFileComplete, callbacks->jFileCompletionArg, jFileName, jError);
				}
			}

			if (env != NULL)
			{
                if (jFileName != NULL)
                {
                    (*env)->DeleteLocalRef(env, jFileName);
                }
			    if (jError != NULL)
			    {
			        (*env)->DeleteLocalRef(env, jError);
			    }
			}

			if ((jResultEnv == JNI_EDETACHED) && (env != NULL))
			{
				(*ARDATATRANSFER_JNI_Manager_VM)->DetachCurrentThread(ARDATATRANSFER_JNI_Manager_VM);
			}
        }
    }
}

int ARDATATRANSFER_JNI_DataDownloader_NewDataDownloaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_DataDownloaderCallbacks_t **callbacksAddr, jobject jFileCompletionListener, jobject jFileCompletionArg)
{
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%p", callbacksAddr ? *callbacksAddr : 0);

    if (callbacksAddr != NULL)
    {
        ARDATATRANSFER_JNI_DataDownloaderCallbacks_t *callbacks = calloc(1, sizeof(ARDATATRANSFER_JNI_DataDownloaderCallbacks_t));
        if (callbacks == NULL)
        {
            error = JNI_FAILED;
        }

        if ((error == ARDATATRANSFER_OK) && (jFileCompletionListener != NULL))
        {
            callbacks->jFileCompletionListener = (*env)->NewGlobalRef(env, jFileCompletionListener);
            if (callbacks->jFileCompletionListener == NULL)
            {
                error = JNI_FAILED;
            }
        }
        if ((error == ARDATATRANSFER_OK) && (jFileCompletionArg != NULL))
        {
            callbacks->jFileCompletionArg = (*env)->NewGlobalRef(env, jFileCompletionArg);
            if (callbacks->jFileCompletionArg == NULL)
            {
                error = JNI_FAILED;
            }
        }

        *callbacksAddr = callbacks;
    }

    return error;
}

void ARDATATRANSFER_JNI_DataDownloader_FreeDataDownloaderCallbacks(JNIEnv *env, ARDATATRANSFER_JNI_DataDownloaderCallbacks_t **callbacksAddr)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%p", callbacksAddr ? *callbacksAddr : 0);

    if (callbacksAddr != NULL)
    {
        ARDATATRANSFER_JNI_DataDownloaderCallbacks_t *callbacks = *callbacksAddr;

        if (callbacks != NULL)
        {
            if (env != NULL)
            {
                if (callbacks->jFileCompletionListener != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jFileCompletionListener);
                }
                if (callbacks->jFileCompletionArg != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jFileCompletionArg);
                }
            }
            free(callbacks);
        }
        *callbacksAddr = NULL;
    }
}

int ARDATATRANSFER_JNI_DataDownloader_NewListenersJNI(JNIEnv *env)
{
    jclass classDDCompletionListener = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%s", "");

     if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (methodId_DDListener_didDataDownloaderFileComplete == NULL)
    {
        if (error == JNI_OK)
        {
            classDDCompletionListener = (*env)->FindClass(env, "com/parrot/arsdk/ardatatransfer/ARDataTransferDataDownloaderFileCompletionListener");

            if (classDDCompletionListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "ARDataTransferDataDownloaderFileCompletionListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DDListener_didDataDownloaderFileComplete = (*env)->GetMethodID(env, classDDCompletionListener, "didDataDownloaderFileComplete", "(Ljava/lang/Object;Ljava/lang/String;Lcom/parrot/arsdk/ardatatransfer/ARDATATRANSFER_ERROR_ENUM;)V");

            if (methodId_DDListener_didDataDownloaderFileComplete == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "Listener didDataDownloaderFileComplete method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARDATATRANSFER_JNI_DataDownloader_FreeListenersJNI(JNIEnv *env)
{
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_DATADOWNLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        methodId_DDListener_didDataDownloaderFileComplete = NULL;
    }
}
