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
 * @brief libARUpdater JNI_Uploader c file.
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

#include "libARUpdater/ARUPDATER_Error.h"
#include "libARUpdater/ARUPDATER_Manager.h"
#include "libARUpdater/ARUPDATER_Uploader.h"
#include "ARUPDATER_JNI.h"

#define ARUPDATER_JNI_UPLOADER_TAG       "JNI"

jmethodID methodId_UploaderListener_onPlfUploadProgress = NULL;
jmethodID methodId_UploaderListener_onPlfUploadComplete = NULL;

JNIEXPORT jboolean JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterUploader_nativeStaticInit(JNIEnv *env, jclass jClass)
{
    jboolean jret = JNI_FALSE;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARUPDATER_JNI_Uploader_NewListenersJNI(env);
    }

    if (error == JNI_OK)
    {
        jret = JNI_TRUE;
    }

    return jret;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterUploader_nativeNew(JNIEnv *env, jobject jThis, jlong jManager, jstring jRootFolder, jlong jMuxCtx, jlong jUtilsManager, jlong jMD5Manager, jint jIsAndroidApp, jint jProduct, jobject jProgressListener, jobject jProgressArgs, jobject jCompletionListener, jobject jCompletionArgs)
{
    struct mux_ctx *nativeMux = (struct mux_ctx *)(intptr_t)jMuxCtx;
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    ARUTILS_Manager_t *nativeFtpManager = (ARUTILS_Manager_t *)(intptr_t)jUtilsManager;
    ARSAL_MD5_Manager_t *nativeMD5Manager = (ARSAL_MD5_Manager_t *)(intptr_t)jMD5Manager;
    ARUPDATER_JNI_UploaderCallbacks_t *callbacks = NULL;
    eARUPDATER_ERROR result = ARUPDATER_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "%x, %x, %x", (int)nativeManager, (int)nativeFtpManager, (int)nativeMux);

    callbacks = (ARUPDATER_JNI_UploaderCallbacks_t*)calloc(1, sizeof(ARUPDATER_JNI_UploaderCallbacks_t));

    if (callbacks == NULL)
    {
        error = JNI_FAILED;
    }
    else
    {
        if (jProgressListener != NULL)
        {
            callbacks->jProgressListener = (*env)->NewGlobalRef(env, jProgressListener);
        }
        if (jProgressArgs != NULL)
        {
            callbacks->jProgressArgs = (*env)->NewGlobalRef(env, jProgressArgs);
        }

        if (jCompletionListener != NULL)
        {
            callbacks->jCompletionListener = (*env)->NewGlobalRef(env, jCompletionListener);
        }
        if (jCompletionArgs != NULL)
        {
            callbacks->jCompletionArgs = (*env)->NewGlobalRef(env, jCompletionArgs);
        }
    }

    if (error == JNI_OK)
    {
        error = ARUPDATER_JNI_Uploader_NewListenersJNI(env);
    }
    
    const char *rootFolder = (*env)->GetStringUTFChars(env, jRootFolder, 0);

    if (error != JNI_OK)
    {
        result = ARUPDATER_ERROR_SYSTEM;
    }

    if (result == ARUPDATER_OK)
    {
        result = ARUPDATER_Uploader_New(nativeManager, rootFolder, nativeMux, nativeFtpManager, nativeMD5Manager, jIsAndroidApp, (eARDISCOVERY_PRODUCT)jProduct, ARUPDATER_JNI_Uploader_ProgressCallback, callbacks, ARUPDATER_JNI_Uploader_CompletionCallback, callbacks);
    }

    if ((result != ARUPDATER_OK) && (callbacks != NULL))
    {
        ARUPDATER_JNI_Uploader_FreeUploaderCallbacks(env, &callbacks);
    }

    if (rootFolder != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jRootFolder, rootFolder);
    }

    return result;
}



JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterUploader_nativeDelete(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "%s", "");

    result = ARUPDATER_Uploader_Delete(nativeManager);

    return result;
}


JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterUploader_nativeSetSubfolder(JNIEnv *env, jobject jThis, jlong jManager, jstring jSubfolder)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "%s", "");

    const char *subfolder = (*env)->GetStringUTFChars(env, jSubfolder, 0);

    result = ARUPDATER_Uploader_SetSubfolder(nativeManager, subfolder);

    if (subfolder != NULL)
        (*env)->ReleaseStringUTFChars(env, jSubfolder, subfolder);

    return result;
}



JNIEXPORT void JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterUploader_nativeThreadRun(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "%s", "");

    ARUPDATER_Uploader_ThreadRun(nativeManager);

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "exit");
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterUploader_nativeCancelThread(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "%s", "");

    result = ARUPDATER_Uploader_CancelThread(nativeManager);

    return result;
}


/**
 * @brief Get the ARUpdaterShouldUploadPlfListener, ARUpdaterPlfUploadProgressListener and ARUpdaterPlfUploadCompletionListener JNI classes
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARUPDATER_JNI_Uploader_FreeListenersJNI
 */
int ARUPDATER_JNI_Uploader_NewListenersJNI(JNIEnv *env)
{
    jclass classUploaderProgressListener = NULL;
    jclass classUploaderCompletionListener = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (methodId_UploaderListener_onPlfUploadProgress == NULL)
    {
        if (error == JNI_OK)
        {
            classUploaderProgressListener = (*env)->FindClass(env, "com/parrot/arsdk/arupdater/ARUpdaterPlfUploadProgressListener");

            if (classUploaderProgressListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "ARUpdaterPlfUploadProgressListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_UploaderListener_onPlfUploadProgress = (*env)->GetMethodID(env, classUploaderProgressListener, "onPlfUploadProgress", "(Ljava/lang/Object;F)V");

            if (methodId_UploaderListener_onPlfUploadProgress == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "Listener didProgress method not found");
                error = JNI_FAILED;
            }
        }
    }

    if (methodId_UploaderListener_onPlfUploadComplete == NULL)
    {
        if (error == JNI_OK)
        {
            classUploaderCompletionListener = (*env)->FindClass(env, "com/parrot/arsdk/arupdater/ARUpdaterPlfUploadCompletionListener");

            if (classUploaderCompletionListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "ARUpdaterPlfUploadCompletionListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_UploaderListener_onPlfUploadComplete = (*env)->GetMethodID(env, classUploaderCompletionListener, "onPlfUploadComplete", "(Ljava/lang/Object;Lcom/parrot/arsdk/arupdater/ARUPDATER_ERROR_ENUM;)V");

            if (methodId_UploaderListener_onPlfUploadComplete == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "Listener didComplete method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

/**
 * @brief Callback that give the download progress percent
 * @param arg The arg
 * @param percent The progress percent
 * @retval void
 * @see ARUPDATER_JNI_Uploader_FreeListenersJNI
 */
void ARUPDATER_JNI_Uploader_ProgressCallback(void* arg, float percent)
{
    ARUPDATER_JNI_UploaderCallbacks_t *callbacks = (ARUPDATER_JNI_UploaderCallbacks_t*)arg;

    if (callbacks != NULL)
    {
        if ((ARUPDATER_JNI_Manager_VM != NULL) && (callbacks->jProgressListener != NULL) && (methodId_UploaderListener_onPlfUploadProgress != NULL))
        {
            JNIEnv *env = NULL;
            jfloat jPercent = 0;
            jint jResultEnv = 0;
            int error = JNI_OK;

            jResultEnv = (*ARUPDATER_JNI_Manager_VM)->GetEnv(ARUPDATER_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);

            if (jResultEnv == JNI_EDETACHED)
            {
                 (*ARUPDATER_JNI_Manager_VM)->AttachCurrentThread(ARUPDATER_JNI_Manager_VM, &env, NULL);
            }

            if (env == NULL)
            {
                error = JNI_FAILED;
            }

            if ((error == JNI_OK) && (methodId_UploaderListener_onPlfUploadProgress != NULL))
            {
                jPercent = percent;

                (*env)->CallVoidMethod(env, callbacks->jProgressListener, methodId_UploaderListener_onPlfUploadProgress, callbacks->jProgressArgs, jPercent);
            }

            if ((jResultEnv == JNI_EDETACHED) && (env != NULL))
            {
                 (*ARUPDATER_JNI_Manager_VM)->DetachCurrentThread(ARUPDATER_JNI_Manager_VM);
            }
        }
    }
}


/**
 * @brief Callback that give the download completion status
 * @param arg The arg
 * @param nativeError The error status of the plf download
 * @retval void
 * @see ARUPDATER_JNI_Uploader_FreeListenersJNI
 */
void ARUPDATER_JNI_Uploader_CompletionCallback(void* arg, eARUPDATER_ERROR nativeError)
{
    ARUPDATER_JNI_UploaderCallbacks_t *callbacks = (ARUPDATER_JNI_UploaderCallbacks_t*)arg;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "%x, %d", (int)arg, nativeError);

    if (callbacks != NULL)
    {
        if (ARUPDATER_JNI_Manager_VM != NULL)
        {
            JNIEnv *env = NULL;
            jobject jError = NULL;
            jint jResultEnv = 0;

            jResultEnv = (*ARUPDATER_JNI_Manager_VM)->GetEnv(ARUPDATER_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);

            if (jResultEnv == JNI_EDETACHED)
            {
                 (*ARUPDATER_JNI_Manager_VM)->AttachCurrentThread(ARUPDATER_JNI_Manager_VM, &env, NULL);
            }

            if (env == NULL)
            {
                //error = JNI_FAILED;
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "error no env");
            }

            if ((env != NULL) && (callbacks->jCompletionListener != NULL) && (methodId_UploaderListener_onPlfUploadComplete != NULL))
            {
                int error = JNI_OK;


                if (error == JNI_OK)
                {
                    jError = ARUPDATER_JNI_Manager_NewERROR_ENUM(env, nativeError);
                    if (jError == NULL)
                    {
                        error = JNI_FAILED;
                        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "error %d, %p", error, jError);
                    }
                }

                if ((error == JNI_OK) && (methodId_UploaderListener_onPlfUploadComplete != NULL))
                {
                     (*env)->CallVoidMethod(env, callbacks->jCompletionListener, methodId_UploaderListener_onPlfUploadComplete, callbacks->jCompletionArgs, jError);
                }
            }

            if (env != NULL)
            {
                if (jError != NULL)
                {
                    (*env)->DeleteLocalRef(env, jError);
                }

                ARUPDATER_JNI_Uploader_FreeUploaderCallbacks(env, &callbacks);
            }

            if ((jResultEnv == JNI_EDETACHED) && (env != NULL))
            {
                (*ARUPDATER_JNI_Manager_VM)->DetachCurrentThread(ARUPDATER_JNI_Manager_VM);
            }
        }

        if (callbacks != NULL)
        {
            free(callbacks);
        }
    }
}




/**
 * @brief Free the ARUpdaterShouldUploadPlfListener, ARUpdaterPlfUploadProgressListener and ARUpdaterPlfUploadCompletionListener JNI classes
 * @param env The java env
 * @retval void
 * @see ARUPDATER_JNI_Uploader_NewListenersJNI
 */
void ARUPDATER_JNI_Uploader_FreeListenersJNI(JNIEnv *env)
{
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        methodId_UploaderListener_onPlfUploadProgress = NULL;
        methodId_UploaderListener_onPlfUploadComplete = NULL;
    }
}


/**
 * @brief Free Callbacks structure
 * @warning This function frees memory
 * @param env The java env
 * @param callbacks The callbacks structure
 * @retval void
 * @see ARUPDATER_JNI_UploaderCallbacks_t
 */
void ARUPDATER_JNI_Uploader_FreeUploaderCallbacks(JNIEnv *env, ARUPDATER_JNI_UploaderCallbacks_t **callbacksParam)
{
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_UPLOADER_TAG, "%p", callbacksParam ? *callbacksParam : 0);

    if (callbacksParam != NULL)
    {
        ARUPDATER_JNI_UploaderCallbacks_t *callbacks = *callbacksParam;

        if (callbacks != NULL)
        {
            if (env != NULL)
            {
                if (callbacks->jProgressListener != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jProgressListener);
                }

                if (callbacks->jProgressArgs != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jProgressArgs);
                }

                if (callbacks->jCompletionListener != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jCompletionListener);
                }

                if (callbacks->jCompletionArgs != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jCompletionArgs);
                }

            }

            free(callbacks);
        }

        *callbacksParam = NULL;
    }
}
