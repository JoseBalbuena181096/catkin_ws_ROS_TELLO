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
 * @brief libARUpdater JNI_Downloader c file.
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

#include "libARUpdater/ARUPDATER_Error.h"
#include "libARUpdater/ARUPDATER_Manager.h"
#include "libARUpdater/ARUPDATER_Downloader.h"
#include "ARUPDATER_JNI.h"

#define ARUPDATER_JNI_DOWNLOADER_TAG       "JNI"

jmethodID methodId_DownloaderListener_willDownloadPlf = NULL;
jmethodID methodId_DownloaderListener_onPlfDownloadProgress = NULL;
jmethodID methodId_DownloaderListener_onPlfDownloadComplete = NULL;
jmethodID methodId_DownloaderListener_downloadPlf = NULL;

jclass classDownloadInfo;
jmethodID methodId_DownloadInfo_init;

jobject ARUPDATER_JNI_Downloader_NewDownloadInfo(JNIEnv *env, ARUPDATER_DownloadInformation_t *info);

JNIEXPORT jboolean JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterDownloader_nativeStaticInit(JNIEnv *env, jclass jClass)
{
    jboolean jret = JNI_FALSE;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARUPDATER_JNI_Downloader_NewListenersJNI(env);
    }

    if (error == JNI_OK)
    {
        jret = JNI_TRUE;
    }

    return jret;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterDownloader_nativeNew(JNIEnv *env, jobject jThis, jlong jManager, jstring jRootFolder, jlong jMD5Manager, jint jPlatform, jstring jAppVersion, jobject jDownloadListener, jobject jDownloadArgs, jobject jWillDownloadPlfListener, jobject jWillDownloadPlfArgs, jobject jProgressListener, jobject jProgressArgs, jobject jCompletionListener, jobject jCompletionArgs)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    ARSAL_MD5_Manager_t *nativeMD5Manager = (ARSAL_MD5_Manager_t*)(intptr_t)jMD5Manager;
    ARUPDATER_JNI_DownloaderCallbacks_t *callbacks = NULL;
    eARUPDATER_ERROR result = ARUPDATER_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%x", (int)nativeManager);

    callbacks = (ARUPDATER_JNI_DownloaderCallbacks_t*)calloc(1, sizeof(ARUPDATER_JNI_DownloaderCallbacks_t));

    if (callbacks == NULL)
    {
        error = JNI_FAILED;
    }
    else
    {
        if (jDownloadListener != NULL)
        {
            callbacks->jDownloadListener = (*env)->NewGlobalRef(env, jDownloadListener);
        }
        if (jDownloadArgs != NULL)
        {
            callbacks->jDownloadArgs = (*env)->NewGlobalRef(env, jDownloadArgs);
        }

	if (jWillDownloadPlfListener != NULL)
        {
            callbacks->jWillDownloadPlfListener = (*env)->NewGlobalRef(env, jWillDownloadPlfListener);
        }
        if (jWillDownloadPlfArgs != NULL)
        {
            callbacks->jWillDownloadPlfArgs = (*env)->NewGlobalRef(env, jWillDownloadPlfArgs);
        }

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
        error = ARUPDATER_JNI_Downloader_NewListenersJNI(env);
    }

    const char *rootFolder = (*env)->GetStringUTFChars(env, jRootFolder, 0);

    const char *appVersion = (*env)->GetStringUTFChars(env, jAppVersion, 0);

    if (error != JNI_OK)
    {
        result = ARUPDATER_ERROR_SYSTEM;
    }

    if (result == ARUPDATER_OK)
    {
        result = ARUPDATER_Downloader_New(nativeManager, rootFolder, nativeMD5Manager, jPlatform, appVersion, ARUPDATER_JNI_Downloader_ShouldDownloadCallback, callbacks, ARUPDATER_JNI_Downloader_WillDownloadPlfCallback, callbacks, ARUPDATER_JNI_Downloader_ProgressCallback, callbacks, ARUPDATER_JNI_Downloader_CompletionCallback, callbacks);
    }

    if ((result != ARUPDATER_OK) && (callbacks != NULL))
    {
        ARUPDATER_JNI_Downloader_FreeDownloaderCallbacks(env, &callbacks);
    }

    if (rootFolder != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jRootFolder, rootFolder);
    }

    if (appVersion != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jAppVersion, appVersion);
    }

    return result;
}



JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterDownloader_nativeDelete(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");

    result = ARUPDATER_Downloader_Delete(nativeManager);

    return result;
}



JNIEXPORT void JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterDownloader_nativeThreadRun(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");

    ARUPDATER_Downloader_ThreadRun(nativeManager);

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "exit");
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterDownloader_nativeCancelThread(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");

    result = ARUPDATER_Downloader_CancelThread(nativeManager);

    return result;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterDownloader_nativeSetVariant(JNIEnv *env, jobject jThis, jlong jManager, jstring jVariant)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");

    if (jVariant != NULL)
    {
        const char *variant = (*env)->GetStringUTFChars(env, jVariant, 0);
        if (variant != NULL)
        {
            result = ARUPDATER_Downloader_SetVariant(nativeManager, variant);
            (*env)->ReleaseStringUTFChars(env, jVariant, variant);
        }
        else
        {
            result = ARUPDATER_ERROR_SYSTEM;
        }
    }
    else
    {
        result = ARUPDATER_ERROR_BAD_PARAMETER;
    }

    return result;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterDownloader_nativeSetUpdatesProductList(JNIEnv *env, jobject jThis, jlong jManager, jintArray jProductArray)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;
    jint *productIntArray = NULL;
    jsize productArrayCount = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");

    if (jProductArray != NULL)
    {
        productArrayCount = (*env)->GetArrayLength(env, jProductArray);
        productIntArray = (*env)->GetIntArrayElements(env, jProductArray, NULL);
        if (productIntArray == NULL)
        {
            result = ARUPDATER_ERROR_ALLOC;
        }
    }

    if (result == ARUPDATER_OK)
    {
        result = ARUPDATER_Downloader_SetUpdatesProductList(nativeManager, (eARDISCOVERY_PRODUCT *)productIntArray, productArrayCount);
    }

    if ((jProductArray != NULL) && (productIntArray != NULL))
    {
        (*env)->ReleaseIntArrayElements(env, jProductArray, productIntArray, 0);
    }

    return result;
}

/**
 * @brief Check if updates are available asynchrounously
 * @post call ARUPDATER_Downloader_ShouldDownloadPlfCallback_t at the end of the execution
 * @param managerArg : thread data of type ARUPDATER_Manager_t*
 * @return ARUPDATER_OK int value if operation went well, a description of the error (error value) otherwise.
 */
JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterDownloader_nativeCheckUpdatesAsync(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");
    result = (eARUPDATER_ERROR)ARUPDATER_Downloader_CheckUpdatesAsync(nativeManager);
    return result;
}

/**
 * @brief Check if updates are available synchrounously
 * @param manager : pointer on the manager
 * @param[out] err : The error status. Can be null.
 * @return The number of plf file chich need to be updated
 */
JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterDownloader_nativeCheckUpdatesSync(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;
    int nbPlfToBeUploaded = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");

    nbPlfToBeUploaded = ARUPDATER_Downloader_CheckUpdatesSync(nativeManager, &result);
    if (result != ARUPDATER_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "error during ARUPDATER_Downloader_CheckUpdatesSync: %d", result);
        ARUPDATER_JNI_Manager_ThrowARUpdaterException(env, result);
    }
    return nbPlfToBeUploaded;
}

/**
 * @brief Get blacklisted versions synchonoulsy
 */
JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterDownloader_nativeGetBlacklistedFirmwareVersionsSync(JNIEnv *env, jobject jThis, jlong jManager, jint jAlsoCheckRemote, jintArray jProductArray, jobjectArray jBlacklistedVersionArray)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;
    ARUPDATER_Manager_BlacklistedFirmware_t** blacklistedVersions = NULL;
    
    if (jProductArray == NULL || jBlacklistedVersionArray == NULL)
    {
        result = ARUPDATER_ERROR_BAD_PARAMETER;
    }
    
    result = ARUPDATER_Downloader_GetBlacklistedFirmwareVersionsSync(nativeManager, jAlsoCheckRemote, &blacklistedVersions);
    
    if (result == ARUPDATER_OK)
    {
        // create an array that contains all the blacklisted firmware.
        // at the same id in the jProductArray is the product
        int i = 0;
        for (i = 0; i < ARDISCOVERY_PRODUCT_MAX; i++)
        {
            if (blacklistedVersions[i] != NULL)
            {
                eARDISCOVERY_PRODUCT product = blacklistedVersions[i]->product;
                int nbBlacklistedVersionsForThisProduct = blacklistedVersions[i]->nbVersionBlacklisted;
                
                jobjectArray blacklistedVersionsForThisProduct = (*env)->NewObjectArray(env, nbBlacklistedVersionsForThisProduct, (*env)->FindClass(env, "java/lang/String")
, NULL);
                if (blacklistedVersionsForThisProduct != NULL)
                {
                    int blacklistedVersionIdx = 0;
                    for (blacklistedVersionIdx = 0; blacklistedVersionIdx < nbBlacklistedVersionsForThisProduct; blacklistedVersionIdx++)
                    {
                        jstring jBlacklistedVersion = (*env)->NewStringUTF(env, blacklistedVersions[i]->versions[blacklistedVersionIdx]);
                        
                        if (jBlacklistedVersion == NULL)
                        {
                            result = JNI_FAILED;
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_JNI_DOWNLOADER_TAG, "error : failed to initiate a jstring : %d", result);
                        }
                        
                        if (result == ARUPDATER_OK)
                        {
                            (*env)->SetObjectArrayElement(env, blacklistedVersionsForThisProduct, blacklistedVersionIdx, jBlacklistedVersion);
                        }
                        
                        if (jBlacklistedVersion != NULL)
                        {
                            (*env)->DeleteLocalRef(env, jBlacklistedVersion);
                        }
                    }
                }
                (*env)->SetObjectArrayElement(env, jBlacklistedVersionArray, i, blacklistedVersionsForThisProduct);
                (*env)->SetIntArrayRegion(env, jProductArray, i, 1, (jint *)&product);
            }
        }
    }
    
    if (result != ARUPDATER_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_JNI_DOWNLOADER_TAG, "error during ARUPDATER_JNI_Downloader_GetBlacklistedFirmwareVersions: %d", result);
        ARUPDATER_JNI_Manager_ThrowARUpdaterException(env, result);
    }
    
    return result;
}

/**
 * @brief Get update information from server synchrounously
 */
JNIEXPORT jobjectArray JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterDownloader_nativeGetUpdatesInfoSync(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;
    ARUPDATER_DownloadInformation_t** informations = NULL;
    int nbInformation = 0;
    jobjectArray ret = NULL;

    int error = ARUPDATER_JNI_Downloader_NewDownloadInfoJNI(env);

    if (error != JNI_OK)
    {
        result = ARUPDATER_ERROR_SYSTEM;
    }

    nbInformation = ARUPDATER_Downloader_GetUpdatesInfoSync(nativeManager, &result, &informations);
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%p, %d", informations, nbInformation);
    if (result == ARUPDATER_OK)
    {
        ret = (*env)->NewObjectArray(env, nbInformation, classDownloadInfo, NULL);
        if (ret != NULL)
        {   int i = 0;
            for (i = 0; i < nbInformation; i++)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%p", informations[i]);
                jobject downloadInfo = NULL;
                if (informations[i] != NULL)
                {
                    downloadInfo = ARUPDATER_JNI_Downloader_NewDownloadInfo(env, informations[i]);
                }
                (*env)->SetObjectArrayElement(env, ret, i, downloadInfo);
            }
        }
        
    }    

    ARUPDATER_JNI_Downloader_FreeDownloadInfoJNI(env);

    if (result != ARUPDATER_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUPDATER_JNI_DOWNLOADER_TAG, "error during ARUPDATER_Downloader_GetUpdatesInfoSync: %d", result);
        ARUPDATER_JNI_Manager_ThrowARUpdaterException(env, result);
    }

    return ret;
}



/**
 * @brief Get the ARUpdaterShouldDownloadPlfListener, ARUpdaterPlfDownloadProgressListener and ARUpdaterPlfDownloadCompletionListener JNI classes
 * @param env The java env
 * @retval JNI_TRUE if Success, else JNI_FALSE
 * @see ARUPDATER_JNI_Downloader_FreeListenersJNI
 */
int ARUPDATER_JNI_Downloader_NewListenersJNI(JNIEnv *env)
{
    jclass classWillDownloadPlfListener = NULL;
    jclass classDownloaderProgressListener = NULL;
    jclass classDownloaderCompletionListener = NULL;
    jclass classDownloaderShouldDownloadListener = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (methodId_DownloaderListener_willDownloadPlf == NULL)
    {
        if (error == JNI_OK)
        {
            classWillDownloadPlfListener = (*env)->FindClass(env, "com/parrot/arsdk/arupdater/ARUpdaterWillDownloadPlfListener");

            if (classWillDownloadPlfListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "ARUpdaterWillDownloadPlfListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DownloaderListener_willDownloadPlf = (*env)->GetMethodID(env, classWillDownloadPlfListener, "onWillDownloadPlf", "(Ljava/lang/Object;Lcom/parrot/arsdk/ardiscovery/ARDISCOVERY_PRODUCT_ENUM;Ljava/lang/String;)V");
            if (methodId_DownloaderListener_willDownloadPlf == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "Listener willDownload method not found");
                error = JNI_FAILED;
            }
        }
    }

    if (methodId_DownloaderListener_onPlfDownloadProgress == NULL)
    {
        if (error == JNI_OK)
        {
            classDownloaderProgressListener = (*env)->FindClass(env, "com/parrot/arsdk/arupdater/ARUpdaterPlfDownloadProgressListener");

            if (classDownloaderProgressListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "ARUpdaterPlfDownloadProgressListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DownloaderListener_onPlfDownloadProgress = (*env)->GetMethodID(env, classDownloaderProgressListener, "onPlfDownloadProgress", "(Ljava/lang/Object;F)V");

            if (methodId_DownloaderListener_onPlfDownloadProgress == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "Listener didProgress method not found");
                error = JNI_FAILED;
            }
        }
    }

    if (methodId_DownloaderListener_onPlfDownloadComplete == NULL)
    {
        if (error == JNI_OK)
        {
            classDownloaderCompletionListener = (*env)->FindClass(env, "com/parrot/arsdk/arupdater/ARUpdaterPlfDownloadCompletionListener");

            if (classDownloaderCompletionListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "ARUpdaterPlfDownloadCompletionListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DownloaderListener_onPlfDownloadComplete = (*env)->GetMethodID(env, classDownloaderCompletionListener, "onPlfDownloadComplete", "(Ljava/lang/Object;Lcom/parrot/arsdk/arupdater/ARUPDATER_ERROR_ENUM;)V");

            if (methodId_DownloaderListener_onPlfDownloadComplete == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "Listener didComplete method not found");
                error = JNI_FAILED;
            }
        }
    }

    if (methodId_DownloaderListener_downloadPlf == NULL)
    {
        if (error == JNI_OK)
        {
            classDownloaderShouldDownloadListener = (*env)->FindClass(env, "com/parrot/arsdk/arupdater/ARUpdaterShouldDownloadPlfListener");

            if (classDownloaderShouldDownloadListener == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "ARUpdaterShouldDownloadPlfListener class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DownloaderListener_downloadPlf = (*env)->GetMethodID(env, classDownloaderShouldDownloadListener, "downloadPlf", "(Ljava/lang/Object;ILcom/parrot/arsdk/arupdater/ARUPDATER_ERROR_ENUM;)V");

            if (methodId_DownloaderListener_downloadPlf == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "Listener downloadPlf method not found");
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
 * @see ARUPDATER_JNI_Downloader_FreeListenersJNI
 */
void ARUPDATER_JNI_Downloader_ProgressCallback(void* arg, float percent)
{
    ARUPDATER_JNI_DownloaderCallbacks_t *callbacks = (ARUPDATER_JNI_DownloaderCallbacks_t*)arg;

    if (callbacks != NULL)
    {
        if ((ARUPDATER_JNI_Manager_VM != NULL) && (callbacks->jProgressListener != NULL) && (methodId_DownloaderListener_onPlfDownloadProgress != NULL))
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

            if ((error == JNI_OK) && (methodId_DownloaderListener_onPlfDownloadProgress != NULL))
            {
                jPercent = percent;

                (*env)->CallVoidMethod(env, callbacks->jProgressListener, methodId_DownloaderListener_onPlfDownloadProgress, callbacks->jProgressArgs, jPercent);
            }

            if ((jResultEnv == JNI_EDETACHED) && (env != NULL))
            {
                 (*ARUPDATER_JNI_Manager_VM)->DetachCurrentThread(ARUPDATER_JNI_Manager_VM);
            }
        }
    }
}

/**
 * @brief Fired just before the uploading of a plf file
 * @param arg The pointer of the user custom argument
 * @param product : Description of the product targeted by the plf downloaded
 * @param remotePlfVersion : The version of the plf file that will be downloaded
 */
void ARUPDATER_JNI_Downloader_WillDownloadPlfCallback(void* arg, eARDISCOVERY_PRODUCT product, const char *const remotePlfVersion)
{
    ARUPDATER_JNI_DownloaderCallbacks_t *callbacks = (ARUPDATER_JNI_DownloaderCallbacks_t*)arg;

    if (callbacks != NULL)
    {
        if ((ARUPDATER_JNI_Manager_VM != NULL) && (callbacks->jWillDownloadPlfListener != NULL) && (methodId_DownloaderListener_willDownloadPlf != NULL))
        {
            JNIEnv *env = NULL;
            jobject jProduct = NULL;
            jstring jRemotePlfVersion = NULL;
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

            if (error == JNI_OK)
            {
                jProduct = ARUPDATER_JNI_Manager_NewDISCOVERY_PRODUCT_ENUM(env, product);

                if (jProduct == NULL)
                {
                    error = JNI_FAILED;
                    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "error %d", error);
                }
            }

            if (error == JNI_OK)
            {
                jRemotePlfVersion = (*env)->NewStringUTF(env, remotePlfVersion);

                if (jRemotePlfVersion == NULL)
                {
                    error = JNI_FAILED;
                    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "error %d", error);
                }
            }

            if ((error == JNI_OK) && (methodId_DownloaderListener_willDownloadPlf != NULL))
            {

                (*env)->CallVoidMethod(env, callbacks->jWillDownloadPlfListener, methodId_DownloaderListener_willDownloadPlf, callbacks->jWillDownloadPlfArgs, jProduct, jRemotePlfVersion);
            }

            if (jRemotePlfVersion != NULL)
            {
                (*env)->DeleteLocalRef(env, jRemotePlfVersion);
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
 * @see ARUPDATER_JNI_Downloader_FreeListenersJNI
 */
void ARUPDATER_JNI_Downloader_CompletionCallback(void* arg, eARUPDATER_ERROR nativeError)
{
    ARUPDATER_JNI_DownloaderCallbacks_t *callbacks = (ARUPDATER_JNI_DownloaderCallbacks_t*)arg;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%x, %d", (int)arg, nativeError);

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
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "error no env");
            }

            if ((env != NULL) && (callbacks->jCompletionListener != NULL) && (methodId_DownloaderListener_onPlfDownloadComplete != NULL))
            {
                int error = JNI_OK;


                if (error == JNI_OK)
                {
                    jError = ARUPDATER_JNI_Manager_NewERROR_ENUM(env, nativeError);

                    if (jError == NULL)
                    {
                        error = JNI_FAILED;
                        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "error %d, %p", error, jError);
                    }
                }

                if ((error == JNI_OK) && (methodId_DownloaderListener_onPlfDownloadComplete != NULL))
                {
                     (*env)->CallVoidMethod(env, callbacks->jCompletionListener, methodId_DownloaderListener_onPlfDownloadComplete, callbacks->jCompletionArgs, jError);
                }
            }

            if (env != NULL)
            {
                if (jError != NULL)
                {
                    (*env)->DeleteLocalRef(env, jError);
                }

                ARUPDATER_JNI_Downloader_FreeDownloaderCallbacks(env, &callbacks);
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
 * @brief Callback that indicates if a plf should be download
 * @param arg The arg
 * @param nbPlfToBeUploaded : number of plf which are out to date
 * @retval void
 * @see ARUPDATER_JNI_Downloader_FreeListenersJNI
 */
void ARUPDATER_JNI_Downloader_ShouldDownloadCallback(void* arg, int nbPlfToBeUploaded, eARUPDATER_ERROR nativeError)
{
    ARUPDATER_JNI_DownloaderCallbacks_t *callbacks = (ARUPDATER_JNI_DownloaderCallbacks_t*)arg;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%d %x", nbPlfToBeUploaded, (int)arg);

    if (callbacks != NULL)
    {
        if (ARUPDATER_JNI_Manager_VM != NULL)
        {
            JNIEnv *env = NULL;
            jint jResultEnv = 0;
            jobject jMedia = NULL;
            jobject jError = NULL;
            int error = JNI_OK;

            jResultEnv = (*ARUPDATER_JNI_Manager_VM)->GetEnv(ARUPDATER_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);

            if (jResultEnv == JNI_EDETACHED)
            {
                 (*ARUPDATER_JNI_Manager_VM)->AttachCurrentThread(ARUPDATER_JNI_Manager_VM, &env, NULL);
            }

            if (env == NULL)
            {
                error = JNI_FAILED;
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "error no env");
            }

            if (error == JNI_OK)
            {
                jError = ARUPDATER_JNI_Manager_NewERROR_ENUM(env, nativeError);

                if (jError == NULL)
                {
                    error = JNI_FAILED;
                    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "error %d, %p", error, jError);
                }
            }

            if ((error == JNI_OK) && (env != NULL) && (callbacks->jDownloadListener != NULL) && (methodId_DownloaderListener_downloadPlf != NULL))
            {
                (*env)->CallVoidMethod(env, callbacks->jDownloadListener, methodId_DownloaderListener_downloadPlf, callbacks->jDownloadArgs, nbPlfToBeUploaded, jError);
            }

            if ((env != NULL) && (jError != NULL))
            {
                (*env)->DeleteLocalRef(env, jError);
            }


            if ((env != NULL) && (jMedia != NULL))
            {
                (*env)->DeleteLocalRef(env, jMedia);
            }

            if ((jResultEnv == JNI_EDETACHED) && (env != NULL))
            {
                (*ARUPDATER_JNI_Manager_VM)->DetachCurrentThread(ARUPDATER_JNI_Manager_VM);
            }
        }
    }
}


/**
 * @brief Free the ARUpdaterShouldDownloadPlfListener, ARUpdaterPlfDownloadProgressListener and ARUpdaterPlfDownloadCompletionListener JNI classes
 * @param env The java env
 * @retval void
 * @see ARUPDATER_JNI_Downloader_NewListenersJNI
 */
void ARUPDATER_JNI_Downloader_FreeListenersJNI(JNIEnv *env)
{
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        methodId_DownloaderListener_willDownloadPlf = NULL;
        methodId_DownloaderListener_onPlfDownloadProgress = NULL;
        methodId_DownloaderListener_onPlfDownloadComplete = NULL;
        methodId_DownloaderListener_downloadPlf = NULL;
    }
}

void ARUPDATER_JNI_Downloader_FreeDownloadInfoJNI(JNIEnv *env)
{
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK && classDownloadInfo != NULL)
    {
        (*env)->DeleteGlobalRef(env, classDownloadInfo);
        classDownloadInfo = NULL;
    }
}



/**
 * @brief Free Callbacks structure
 * @warning This function frees memory
 * @param env The java env
 * @param callbacks The callbacks structure
 * @retval void
 * @see ARUPDATER_JNI_DownloaderCallbacks_t
 */
void ARUPDATER_JNI_Downloader_FreeDownloaderCallbacks(JNIEnv *env, ARUPDATER_JNI_DownloaderCallbacks_t **callbacksParam)
{
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%p", callbacksParam ? *callbacksParam : 0);

    if (callbacksParam != NULL)
    {
        ARUPDATER_JNI_DownloaderCallbacks_t *callbacks = *callbacksParam;

        if (callbacks != NULL)
        {
            if (env != NULL)
            {
                if (callbacks->jWillDownloadPlfListener != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jWillDownloadPlfListener);
                }

                if (callbacks->jWillDownloadPlfArgs != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jWillDownloadPlfArgs);
                }

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

                if (callbacks->jDownloadListener != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jDownloadListener);
                }

                if (callbacks->jDownloadArgs != NULL)
                {
                    (*env)->DeleteGlobalRef(env, callbacks->jDownloadArgs);
                }


            }

            free(callbacks);
        }

        *callbacksParam = NULL;
    }
}



int ARUPDATER_JNI_Downloader_NewDownloadInfoJNI(JNIEnv *env)
{
    jclass locClassDownloadInfo = NULL;
    int error = JNI_OK;

    if (classDownloadInfo == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", "");

        if (env == NULL)
        {
            error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            locClassDownloadInfo = (*env)->FindClass(env, "com/parrot/arsdk/arupdater/ARUpdaterDownloadInfo");

            if (locClassDownloadInfo == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "ARUpdaterDownloadInfo class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            classDownloadInfo = (*env)->NewGlobalRef(env, locClassDownloadInfo);

            if (classDownloadInfo == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "ARUpdaterDownloadInfo global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DownloadInfo_init = (*env)->GetMethodID(env, classDownloadInfo, "<init>", "(Ljava/lang/String;Ljava/lang/String;I)V");

            if (methodId_DownloadInfo_init == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "ARUpdaterDownloadInfo <init> method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}


jobject ARUPDATER_JNI_Downloader_NewDownloadInfo(JNIEnv *env, ARUPDATER_DownloadInformation_t *info)
{
    jobject jInfo = NULL;
    jstring jDownloadUrl = NULL;
    jstring jPlfVersion = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_DOWNLOADER_TAG, "%s", (info->plfVersion != NULL) ? info->plfVersion : "null");

    if ((env == NULL) || (info == NULL))
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        if ((classDownloadInfo == NULL) || (methodId_DownloadInfo_init == NULL))
        {
            error = JNI_FAILED;
        }
    }

    if ((error == JNI_OK) && (info->downloadUrl != NULL))
    {
        jDownloadUrl = (*env)->NewStringUTF(env, info->downloadUrl);

        if (jDownloadUrl == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if ((error == JNI_OK) && (info->plfVersion != NULL))
    {
        jPlfVersion = (*env)->NewStringUTF(env, info->plfVersion);

        if (jPlfVersion == NULL)
        {
            error = JNI_FAILED;
        }
    }


    if (error == JNI_OK)
    {
        jInfo = (*env)->NewObject(env, classDownloadInfo, methodId_DownloadInfo_init, jDownloadUrl, jPlfVersion, (jint)ARDISCOVERY_getProductID(info->product));
    }

    // clean local refs

    if (jDownloadUrl != NULL)
    {
        (*env)->DeleteLocalRef(env, jDownloadUrl);
    }

    if (jPlfVersion != NULL)
    {
        (*env)->DeleteLocalRef(env, jPlfVersion);
    }

    return jInfo;
}
