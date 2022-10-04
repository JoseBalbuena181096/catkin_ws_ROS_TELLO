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
 * @brief libARDataTransfer JNI_Manager c file.
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

#include "libARDataTransfer/ARDATATRANSFER_Error.h"
#include "libARDataTransfer/ARDATATRANSFER_Manager.h"
#include "libARDataTransfer/ARDATATRANSFER_DataDownloader.h"
#include "libARDataTransfer/ARDATATRANSFER_MediasDownloader.h"

#include "ARDATATRANSFER_JNI.h"

#define ARDATATRANSFER_JNI_MANAGER_TAG       "JNI"

JavaVM* ARDATATRANSFER_JNI_Manager_VM = NULL;
int ARDATATRANSFER_Manager_Count = 0;

jclass classDTException = NULL;
jmethodID methodId_DTException_Init = NULL;

jclass classDTERROR_ENUM = NULL;
jmethodID methodId_DTERROR_ENUM_getFromValue = NULL;

/*****************************************
 *
 *             Private implementation:
 *
 *****************************************/

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *VM, void *reserved)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "Library has been loaded: (ardatatransfer_android.so) %x", (int)VM);

    ARDATATRANSFER_JNI_Manager_VM = VM;

    return JNI_VERSION_1_6;
}

JNIEXPORT jboolean JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferManager_nativeStaticInit(JNIEnv *env, jclass jClass)
{
    jboolean jret = JNI_FALSE;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "%x", (int)env);

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_Manager_NewARDataTransferExceptionJNI(env);
    }

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_Manager_NewERROR_ENUM_JNI(env);
    }

    if (error == JNI_OK)
    {
        jret = JNI_TRUE;
    }

    return jret;
}

JNIEXPORT jlong JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferManager_nativeNew(JNIEnv *env, jobject jThis)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = NULL;
    eARDATATRANSFER_ERROR result = ARDATATRANSFER_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "%s", "");

    nativeJniManager = calloc(1, sizeof(ARDATATRANSFER_JNI_Manager_t));
    if (nativeJniManager == NULL)
    {
        result = ARDATATRANSFER_ERROR_ALLOC;
    }

    if (result == ARDATATRANSFER_OK)
    {
        nativeJniManager->nativeManager = ARDATATRANSFER_Manager_New(&result);
    }

    if (result == ARDATATRANSFER_OK)
    {
        if (error == JNI_OK)
        {
            error = ARDATATRANSFER_JNI_Manager_NewARDataTransferExceptionJNI(env);
        }

        if (error == JNI_OK)
        {
            error = ARDATATRANSFER_JNI_Manager_NewERROR_ENUM_JNI(env);
        }

        if (error != JNI_OK)
        {
            result = ARDATATRANSFER_ERROR_SYSTEM;
        }
    }

    if (result == ARDATATRANSFER_OK)
    {
        ARDATATRANSFER_Manager_Count++;
    }
    else
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARDATATRANSFER_JNI_MANAGER_TAG, "error: %d occurred", result);

        if (nativeJniManager != NULL)
        {
            ARDATATRANSFER_Manager_Delete(&nativeJniManager->nativeManager);
            free(nativeJniManager);
            nativeJniManager = NULL;
        }

        ARDATATRANSFER_JNI_Manager_ThrowARDataTransferException(env, result);
    }

    return (long)nativeJniManager;
}

JNIEXPORT void JNICALL Java_com_parrot_arsdk_ardatatransfer_ARDataTransferManager_nativeDelete(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARDATATRANSFER_JNI_Manager_t *nativeJniManager = (ARDATATRANSFER_JNI_Manager_t*)(intptr_t)jManager;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "%s", "");

    if (nativeJniManager != NULL)
    {
        ARDATATRANSFER_Manager_Delete(&nativeJniManager->nativeManager);

        if (ARDATATRANSFER_Manager_Count > 0)
        {
            ARDATATRANSFER_Manager_Count--;

            if (ARDATATRANSFER_Manager_Count == 0)
            {
                ARDATATRANSFER_JNI_Manager_FreeARDataTransferExceptionJNI(env);
                ARDATATRANSFER_JNI_Manager_FreeERROR_ENUM_JNI(env);
            }
        }

        free(nativeJniManager);
    }
}

/*****************************************
 *
 *             Private implementation:
 *
 *****************************************/

int ARDATATRANSFER_JNI_Manager_NewARDataTransferExceptionJNI(JNIEnv *env)
{
    jclass locClassDTException = NULL;
    int error = JNI_OK;

    if (classDTException == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "%s", "");

        if (env == NULL)
        {
           error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            locClassDTException  = (*env)->FindClass(env, "com/parrot/arsdk/ardatatransfer/ARDataTransferException");

            if (locClassDTException == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "ARDataTransferException class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            classDTException = (*env)->NewGlobalRef(env,locClassDTException);

            if (classDTException == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "ARDataTransferException global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DTException_Init = (*env)->GetMethodID(env, classDTException, "<init>", "(I)V");

            if (methodId_DTException_Init == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "init method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARDATATRANSFER_JNI_Manager_FreeARDataTransferExceptionJNI(JNIEnv *env)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "%s", "");

    if (env != NULL)
    {
        if (classDTException != NULL)
        {
            (*env)->DeleteGlobalRef(env, classDTException);
            classDTException = NULL;
        }

        methodId_DTException_Init = NULL;
    }
}

jobject ARDATATRANSFER_JNI_Manager_NewARDataTransferException(JNIEnv *env, eARDATATRANSFER_ERROR nativeError)
{
    jobject jException = NULL;
    jint jError = JNI_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "%d", nativeError);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_Manager_NewARDataTransferExceptionJNI(env);
    }

    if (error == JNI_OK)
    {
         jError = nativeError;

        jException = (*env)->NewObject(env, classDTException, methodId_DTException_Init, jError);
    }

    return jException;
}

void ARDATATRANSFER_JNI_Manager_ThrowARDataTransferException(JNIEnv *env, eARDATATRANSFER_ERROR nativeError)
{
    jthrowable jThrowable = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "%d", error);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        jThrowable = ARDATATRANSFER_JNI_Manager_NewARDataTransferException(env, nativeError);

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

int ARDATATRANSFER_JNI_Manager_NewERROR_ENUM_JNI(JNIEnv *env)
{
    jclass locClassDTERROR_ENUM = NULL;
    int error = JNI_OK;

    if (classDTERROR_ENUM == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "%s", "");

        if (env == NULL)
        {
            error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            locClassDTERROR_ENUM = (*env)->FindClass(env, "com/parrot/arsdk/ardatatransfer/ARDATATRANSFER_ERROR_ENUM");

            if (locClassDTERROR_ENUM == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "ARDATATRANSFER_ERROR_ENUM class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            classDTERROR_ENUM = (*env)->NewGlobalRef(env, locClassDTERROR_ENUM);

            if (classDTERROR_ENUM == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "ARDATATRANSFER_ERROR_ENUM global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DTERROR_ENUM_getFromValue = (*env)->GetStaticMethodID(env, classDTERROR_ENUM, "getFromValue", "(I)Lcom/parrot/arsdk/ardatatransfer/ARDATATRANSFER_ERROR_ENUM;");

            if (methodId_DTERROR_ENUM_getFromValue == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "getFromValue method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARDATATRANSFER_JNI_Manager_FreeERROR_ENUM_JNI(JNIEnv *env)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "%s", "");

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

jobject ARDATATRANSFER_JNI_Manager_NewERROR_ENUM(JNIEnv *env, eARDATATRANSFER_ERROR nativeError)
{
    jobject jERROR_ENUM = NULL;
    jint jError;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDATATRANSFER_JNI_MANAGER_TAG, "%d", nativeError);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARDATATRANSFER_JNI_Manager_NewERROR_ENUM_JNI(env);
    }

    if (error == JNI_OK)
    {
        jError = nativeError;

        jERROR_ENUM = (*env)->CallStaticObjectMethod(env, classDTERROR_ENUM, methodId_DTERROR_ENUM_getFromValue, jError);
    }

    return jERROR_ENUM;
}

