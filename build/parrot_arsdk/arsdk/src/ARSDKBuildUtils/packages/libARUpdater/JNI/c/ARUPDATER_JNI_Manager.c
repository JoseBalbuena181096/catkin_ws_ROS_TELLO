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
 * @brief libARUpdater JNI_Manager c file.
 * @date 19/12/2013
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

#include "libARUpdater/ARUPDATER_Error.h"
#include "libARUpdater/ARUPDATER_Manager.h"
#include "libARUpdater/ARUPDATER_Downloader.h"
#include "libARUpdater/ARUPDATER_Uploader.h"
#include "libARUpdater/ARUPDATER_Utils.h"

#include "ARUPDATER_JNI.h"

#define ARUPDATER_JNI_MANAGER_TAG       "JNI"

JavaVM* ARUPDATER_JNI_Manager_VM = NULL;
int ARUPDATER_Manager_Count = 0;

jclass classUPDATER_Exception = NULL;
jmethodID methodId_UPDATER_Exception_Init = NULL;

jclass classUPDATER_ERROR_ENUM = NULL;
jmethodID methodId_UPDATER_ERROR_ENUM_getFromValue = NULL;

jclass classDTERROR_ENUM = NULL;
jmethodID methodId_DTERROR_ENUM_getFromValue = NULL;

jclass classDISCOVERY_PRODUCT_ENUM = NULL;
jmethodID methodId_DISCOVERY_PRODUCT_ENUM_getFromValue = NULL;

/*****************************************
 *
 *             Private implementation:
 *
 *****************************************/

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *VM, void *reserved)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "Library has been loaded: (arupdater_android.so) %x", (int)VM);

    ARUPDATER_JNI_Manager_VM = VM;

    return JNI_VERSION_1_6;
}

JNIEXPORT jboolean JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterManager_nativeStaticInit(JNIEnv *env, jclass jClass)
{
    jboolean jret = JNI_FALSE;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%x", (int)env);

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARUPDATER_JNI_Manager_NewARUpdaterExceptionJNI(env);
    }

    if (error == JNI_OK)
    {
        error = ARUPDATER_JNI_Manager_NewERROR_ENUM_JNI(env);
    }

    if (error == JNI_OK)
    {
        jret = JNI_TRUE;
    }

    return jret;
}

JNIEXPORT jlong JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterManager_nativeNew(JNIEnv *env, jobject jThis)
{
    ARUPDATER_Manager_t *nativeManager = NULL;
    eARUPDATER_ERROR result = ARUPDATER_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%s", "");

    nativeManager = ARUPDATER_Manager_New(&result);

    if (error == JNI_OK)
    {
        error = ARUPDATER_JNI_Manager_NewARUpdaterExceptionJNI(env);
    }

    if (error == JNI_OK)
    {
        error = ARUPDATER_JNI_Manager_NewERROR_ENUM_JNI(env);
    }

    if (error != JNI_OK)
    {
        result = ARUPDATER_ERROR_SYSTEM;
    }

    if (result == ARUPDATER_OK)
    {
        ARUPDATER_Manager_Count++;
    }
    else
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUPDATER_JNI_MANAGER_TAG, "error: %d occurred", result);

        ARUPDATER_JNI_Manager_ThrowARUpdaterException(env, result);
    }

    return (jlong)(intptr_t)nativeManager;
}

JNIEXPORT void JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterManager_nativeDelete(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*) (intptr_t) jManager;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%s", "");

    ARUPDATER_Manager_Delete (&nativeManager);

    if (ARUPDATER_Manager_Count > 0)
    {
        ARUPDATER_Manager_Count--;

        if (ARUPDATER_Manager_Count == 0)
        {
            ARUPDATER_JNI_Manager_FreeARUpdaterExceptionJNI(env);
            ARUPDATER_JNI_Manager_FreeERROR_ENUM_JNI(env);
            ARUPDATER_JNI_Manager_FreeDATATRANSFER_ERROR_ENUM_JNI(env);
            ARUPDATER_JNI_Downloader_FreeListenersJNI(env);
        }
    }
}

JNIEXPORT jboolean JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterManager_nativePlfVersionIsUpToDate(JNIEnv *env, jobject jThis, jlong jManager, jint jProduct, jstring jRemoteVersion, jstring jRootFolder)
{
    ARUPDATER_Manager_t *nativeManager = (ARUPDATER_Manager_t*)(intptr_t)jManager;
    eARUPDATER_ERROR result = ARUPDATER_OK;
    const char *rootFolder = (*env)->GetStringUTFChars(env, jRootFolder, 0);
    const char *remoteVersion = (*env)->GetStringUTFChars(env, jRemoteVersion, 0);

    jclass thisClass = (*env)->GetObjectClass(env, jThis);
    jfieldID localVersionField = (*env)->GetFieldID(env, thisClass, "localVersion", "Ljava/lang/String;");
    (*env)->DeleteLocalRef(env, thisClass);

    int bufferSize = 16;
    char *localVersionBuffer = malloc(bufferSize * sizeof(char));

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%d %s %s", jProduct, remoteVersion, rootFolder);

    int isUpToDate = ARUPDATER_Manager_PlfVersionIsUpToDate(nativeManager, (eARDISCOVERY_PRODUCT)jProduct, remoteVersion, rootFolder, localVersionBuffer, bufferSize, &result);

    if (rootFolder != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jRootFolder, rootFolder);
    }

    if (remoteVersion != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jRemoteVersion, remoteVersion);
    }

    if (localVersionBuffer != NULL && result == ARUPDATER_OK)
    {
        jstring jLocalVersion = (*env)->NewStringUTF(env, localVersionBuffer);
        free(localVersionBuffer);
        (*env)->SetObjectField(env, jThis, localVersionField, jLocalVersion);
    }
    else
    {
        (*env)->SetObjectField(env, jThis, localVersionField, NULL);
    }

    if (result != ARUPDATER_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARUPDATER_JNI_MANAGER_TAG, "error while trying to call ARUPDATER_Manager_PlfVersionIsUpToDate: [%d]", result);
        ARUPDATER_JNI_Manager_ThrowARUpdaterException(env, result);
    }

    return isUpToDate;
}

JNIEXPORT jstring JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterManager_nativeReadPlfVersion(JNIEnv *env, jclass jClass, jstring jPlfFilePath)
{
    eARUPDATER_ERROR result = ARUPDATER_OK;
    jstring jversion = NULL;

    const char *plfFilePath = (*env)->GetStringUTFChars(env, jPlfFilePath, 0);
    ARUPDATER_PlfVersion version;

    if (plfFilePath != NULL)
    {
        result = ARUPDATER_Utils_ReadPlfVersion(plfFilePath, &version);
        (*env)->ReleaseStringUTFChars(env, jPlfFilePath, plfFilePath);
    }

    if (result == ARUPDATER_OK)
    {
        char buffer[128];
        result = ARUPDATER_Utils_PlfVersionToString(&version, buffer, sizeof(buffer));
        if (result == ARUPDATER_OK)
        {
            jversion = (*env)->NewStringUTF(env, buffer);
        }
    }

    return jversion;

}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterManager_nativeComparePlfVersions(JNIEnv *env, jclass jClass, jstring jVersion1, jstring jVersion2)
{
    eARUPDATER_ERROR result = ARUPDATER_OK;

    const char *version1 = (*env)->GetStringUTFChars(env, jVersion1, 0);
    const char *version2 = (*env)->GetStringUTFChars(env, jVersion2, 0);

    ARUPDATER_PlfVersion plfVersion1;
    ARUPDATER_PlfVersion plfVersion2;
    int ret = 0;

    if (version1 != NULL)
    {
        result = ARUPDATER_Utils_PlfVersionFromString(version1, &plfVersion1);
    }

    if (result == ARUPDATER_OK && version2 != NULL)
    {
        result = ARUPDATER_Utils_PlfVersionFromString(version2, &plfVersion2);
    }

    if (result == ARUPDATER_OK)
    {
        ret = ARUPDATER_Utils_PlfVersionCompare(&plfVersion1, &plfVersion2);
    }

    if (version1 != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jVersion1, version1);
    }

    if (version2 != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jVersion2, version2);
    }

    return ret;

}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arupdater_ARUpdaterManager_nativeExtractUnixFileFromPlf(JNIEnv *env, jclass jClass, jstring jplfFileName,
												     jstring joutFolder, jstring junixFileName)
{
    eARUPDATER_ERROR result = ARUPDATER_ERROR_BAD_PARAMETER;

    const char *plfFileName  = (*env)->GetStringUTFChars(env, jplfFileName, 0);
    const char *outFolder    = (*env)->GetStringUTFChars(env, joutFolder, 0);
    const char *unixFileName = (*env)->GetStringUTFChars(env, junixFileName, 0);

    if ((plfFileName != NULL) && (outFolder != NULL) && (unixFileName != NULL))
    {
        result = ARUPDATER_Utils_ExtractUnixFileFromPlf(plfFileName, outFolder, unixFileName);
    }

    if (plfFileName)
	(*env)->ReleaseStringUTFChars(env, jplfFileName, plfFileName);

    if (outFolder)
	(*env)->ReleaseStringUTFChars(env, joutFolder, outFolder);

    if (unixFileName)
	(*env)->ReleaseStringUTFChars(env, junixFileName, unixFileName);

    return result;
}


/*****************************************
 *
 *             Private implementation:
 *
 *****************************************/

int ARUPDATER_JNI_Manager_NewARUpdaterExceptionJNI(JNIEnv *env)
{
    jclass locClassUPDATER_Exception = NULL;
    int error = JNI_OK;

    if (classUPDATER_Exception == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%s", "");

        if (env == NULL)
        {
           error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            locClassUPDATER_Exception  = (*env)->FindClass(env, "com/parrot/arsdk/arupdater/ARUpdaterException");

            if (locClassUPDATER_Exception == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "ARUpdaterException class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            classUPDATER_Exception = (*env)->NewGlobalRef(env,locClassUPDATER_Exception);

            if (classUPDATER_Exception == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "ARUpdaterException global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_UPDATER_Exception_Init = (*env)->GetMethodID(env, classUPDATER_Exception, "<init>", "(I)V");

            if (methodId_UPDATER_Exception_Init == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "init method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARUPDATER_JNI_Manager_FreeARUpdaterExceptionJNI(JNIEnv *env)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%s", "");

    if (env != NULL)
    {
        if (classUPDATER_Exception != NULL)
        {
            (*env)->DeleteGlobalRef(env, classUPDATER_Exception);
            classUPDATER_Exception = NULL;
        }

        methodId_UPDATER_Exception_Init = NULL;
    }
}

jobject ARUPDATER_JNI_Manager_NewARUpdaterException(JNIEnv *env, eARUPDATER_ERROR nativeError)
{
    jobject jException = NULL;
    jint jError = JNI_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%d", nativeError);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARUPDATER_JNI_Manager_NewARUpdaterExceptionJNI(env);
    }

    if (error == JNI_OK)
    {
         jError = nativeError;

        jException = (*env)->NewObject(env, classUPDATER_Exception, methodId_UPDATER_Exception_Init, jError);
    }

    return jException;
}

void ARUPDATER_JNI_Manager_ThrowARUpdaterException(JNIEnv *env, eARUPDATER_ERROR nativeError)
{
    jthrowable jThrowable = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%d", error);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        jThrowable = ARUPDATER_JNI_Manager_NewARUpdaterException(env, nativeError);

        if (jThrowable == NULL)
        {
           error = JNI_FAILED;
        }
    }

    if (error == JNI_OK)
    {
        (*env)->Throw(env, jThrowable);
    }
}

int ARUPDATER_JNI_Manager_NewERROR_ENUM_JNI(JNIEnv *env)
{
    jclass locClassUPDATER_ERROR_ENUM = NULL;
    int error = JNI_OK;

    if (classUPDATER_ERROR_ENUM == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%s", "");

        if (env == NULL)
        {
            error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            locClassUPDATER_ERROR_ENUM = (*env)->FindClass(env, "com/parrot/arsdk/arupdater/ARUPDATER_ERROR_ENUM");

            if (locClassUPDATER_ERROR_ENUM == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "ARUPDATER_ERROR_ENUM class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            classUPDATER_ERROR_ENUM = (*env)->NewGlobalRef(env, locClassUPDATER_ERROR_ENUM);

            if (classUPDATER_ERROR_ENUM == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "ARUPDATER_ERROR_ENUM global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_UPDATER_ERROR_ENUM_getFromValue = (*env)->GetStaticMethodID(env, classUPDATER_ERROR_ENUM, "getFromValue", "(I)Lcom/parrot/arsdk/arupdater/ARUPDATER_ERROR_ENUM;");

            if (methodId_UPDATER_ERROR_ENUM_getFromValue == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "getFromValue method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARUPDATER_JNI_Manager_FreeERROR_ENUM_JNI(JNIEnv *env)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%s", "");

    if (env != NULL)
    {
        if (classUPDATER_ERROR_ENUM != NULL)
        {
            (*env)->DeleteGlobalRef(env, classUPDATER_ERROR_ENUM);
            classUPDATER_ERROR_ENUM = NULL;
        }

        methodId_UPDATER_ERROR_ENUM_getFromValue = NULL;
    }
}

jobject ARUPDATER_JNI_Manager_NewERROR_ENUM(JNIEnv *env, eARUPDATER_ERROR nativeError)
{
    jobject jERROR_ENUM = NULL;
    jint jError;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%d", nativeError);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARUPDATER_JNI_Manager_NewERROR_ENUM_JNI(env);
    }

    if (error == JNI_OK)
    {
        jError = nativeError;

        jERROR_ENUM = (*env)->CallStaticObjectMethod(env, classUPDATER_ERROR_ENUM, methodId_UPDATER_ERROR_ENUM_getFromValue, jError);
    }

    return jERROR_ENUM;
}


/*************************************************
    ARDATATRANSFER enum generator
**************************************************/

int ARUPDATER_JNI_Manager_NewDATATRANSFER_ERROR_ENUM_JNI(JNIEnv *env)
{
    jclass locClassDTERROR_ENUM = NULL;
    int error = JNI_OK;

    if (classDTERROR_ENUM == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%s", "");

        if (env == NULL)
        {
            error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            locClassDTERROR_ENUM = (*env)->FindClass(env, "com/parrot/arsdk/ardatatransfer/ARDATATRANSFER_ERROR_ENUM");

            if (locClassDTERROR_ENUM == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "ARDATATRANSFER_ERROR_ENUM class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            classDTERROR_ENUM = (*env)->NewGlobalRef(env, locClassDTERROR_ENUM);

            if (classDTERROR_ENUM == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "ARDATATRANSFER_ERROR_ENUM global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DTERROR_ENUM_getFromValue = (*env)->GetStaticMethodID(env, classDTERROR_ENUM, "getFromValue", "(I)Lcom/parrot/arsdk/ardatatransfer/ARDATATRANSFER_ERROR_ENUM;");

            if (methodId_DTERROR_ENUM_getFromValue == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "getFromValue method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARUPDATER_JNI_Manager_FreeDATATRANSFER_ERROR_ENUM_JNI(JNIEnv *env)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%s", "");

    if (env != NULL)
    {
        if (classDTERROR_ENUM != NULL)
        {
            (*env)->DeleteGlobalRef(env, classDTERROR_ENUM);
            classDTERROR_ENUM = NULL;
        }

        methodId_DTERROR_ENUM_getFromValue = NULL;
    }
}

jobject ARUPDATER_JNI_Manager_NewDATATRANSFER_ERROR_ENUM(JNIEnv *env, eARDATATRANSFER_ERROR nativeError)
{
    jobject jERROR_ENUM = NULL;
    jint jError;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%d", nativeError);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARUPDATER_JNI_Manager_NewDATATRANSFER_ERROR_ENUM_JNI(env);
    }

    if (error == JNI_OK)
    {
        jError = nativeError;

        jERROR_ENUM = (*env)->CallStaticObjectMethod(env, classDTERROR_ENUM, methodId_DTERROR_ENUM_getFromValue, jError);
    }

    return jERROR_ENUM;
}

/*************************************************
    ARDISCOVERY Product enum generator
**************************************************/

int ARUPDATER_JNI_Manager_NewDISCOVERY_PRODUCT_ENUM_JNI(JNIEnv *env)
{
    jclass locClassDISCOVERY_PRODUCT_ENUM = NULL;
    int error = JNI_OK;

    if (classDISCOVERY_PRODUCT_ENUM == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%s", "");

        if (env == NULL)
        {
            error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            locClassDISCOVERY_PRODUCT_ENUM = (*env)->FindClass(env, "com/parrot/arsdk/ardiscovery/ARDISCOVERY_PRODUCT_ENUM");

            if (locClassDISCOVERY_PRODUCT_ENUM == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "ARDISCOVERY_PRODUCT_ENUM class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            classDISCOVERY_PRODUCT_ENUM = (*env)->NewGlobalRef(env, locClassDISCOVERY_PRODUCT_ENUM);

            if (classDISCOVERY_PRODUCT_ENUM == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "ARDISCOVERY_PRODUCT_ENUM global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DISCOVERY_PRODUCT_ENUM_getFromValue = (*env)->GetStaticMethodID(env, classDISCOVERY_PRODUCT_ENUM, "getFromValue", "(I)Lcom/parrot/arsdk/ardiscovery/ARDISCOVERY_PRODUCT_ENUM;");

            if (methodId_DISCOVERY_PRODUCT_ENUM_getFromValue == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "getFromValue method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARUPDATER_JNI_Manager_FreeDISCOVERY_PRODUCT_ENUM_JNI(JNIEnv *env)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%s", "");

    if (env != NULL)
    {
        if (classDISCOVERY_PRODUCT_ENUM != NULL)
        {
            (*env)->DeleteGlobalRef(env, classDISCOVERY_PRODUCT_ENUM);
            classDISCOVERY_PRODUCT_ENUM = NULL;
        }

        methodId_DISCOVERY_PRODUCT_ENUM_getFromValue = NULL;
    }
}

jobject ARUPDATER_JNI_Manager_NewDISCOVERY_PRODUCT_ENUM(JNIEnv *env, eARDISCOVERY_PRODUCT product)
{
    jobject jPRODUCT_ENUM = NULL;
    jint jProduct;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUPDATER_JNI_MANAGER_TAG, "%d", product);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARUPDATER_JNI_Manager_NewDISCOVERY_PRODUCT_ENUM_JNI(env);
    }

    if (error == JNI_OK)
    {
        jProduct = product;

        jPRODUCT_ENUM = (*env)->CallStaticObjectMethod(env, classDISCOVERY_PRODUCT_ENUM, methodId_DISCOVERY_PRODUCT_ENUM_getFromValue, jProduct);
    }

    return jPRODUCT_ENUM;
}
