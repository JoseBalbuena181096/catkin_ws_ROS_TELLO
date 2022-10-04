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
 * @file ARASL_Manager.c
 * @brief libARSAL JNI_Manager c file.
 * @date 03/06/2014
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
#include <libARSAL/ARSAL_Error.h>


#include "ARSAL_JNI.h"

#define ARSAL_JNI_MANAGER_TAG       "JNI"

JavaVM* ARSAL_JNI_Manager_VM = NULL;

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
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "Library has been loaded: (arsal_android.so) %x", (int)VM);

    ARSAL_JNI_Manager_VM = VM;

    return JNI_VERSION_1_6;
}

/*****************************************
 *
 *             Private implementation:
 *
 *****************************************/

int ARSAL_JNI_Manager_NewARSALExceptionJNI(JNIEnv *env)
{
    jclass locClassDTException = NULL;
    int error = JNI_OK;

    if (classDTException == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "%s", "");

        if (env == NULL)
        {
           error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            locClassDTException  = (*env)->FindClass(env, "com/parrot/arsdk/arsal/ARSALException");

            if (locClassDTException == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "ARSALException class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            classDTException = (*env)->NewGlobalRef(env,locClassDTException);

            if (classDTException == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "ARSALException global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DTException_Init = (*env)->GetMethodID(env, classDTException, "<init>", "(I)V");

            if (methodId_DTException_Init == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "init method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARSAL_JNI_Manager_FreeARSALExceptionJNI(JNIEnv *env)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "%s", "");

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

jobject ARSAL_JNI_Manager_NewARSALException(JNIEnv *env, eARSAL_ERROR nativeError)
{
    jobject jException = NULL;
    jint jError = JNI_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "%d", nativeError);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARSAL_JNI_Manager_NewARSALExceptionJNI(env);
    }

    if (error == JNI_OK)
    {
         jError = nativeError;

        jException = (*env)->NewObject(env, classDTException, methodId_DTException_Init, jError);
    }

    return jException;
}

void ARSAL_JNI_Manager_ThrowARSALException(JNIEnv *env, eARSAL_ERROR nativeError)
{
    jthrowable jThrowable = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "%d", error);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        jThrowable = ARSAL_JNI_Manager_NewARSALException(env, nativeError);

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

int ARSAL_JNI_Manager_NewERROR_ENUM_JNI(JNIEnv *env)
{
    jclass locClassDTERROR_ENUM = NULL;
    int error = JNI_OK;

    if (classDTERROR_ENUM == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "%s", "");

        if (env == NULL)
        {
            error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            locClassDTERROR_ENUM = (*env)->FindClass(env, "com/parrot/arsdk/arsal/ARSAL_ERROR_ENUM");

            if (locClassDTERROR_ENUM == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "ARSAL_ERROR_ENUM class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            classDTERROR_ENUM = (*env)->NewGlobalRef(env, locClassDTERROR_ENUM);

            if (classDTERROR_ENUM == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "ARSAL_ERROR_ENUM global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_DTERROR_ENUM_getFromValue = (*env)->GetStaticMethodID(env, classDTERROR_ENUM, "getFromValue", "(I)Lcom/parrot/arsdk/arsal/ARSAL_ERROR_ENUM;");

            if (methodId_DTERROR_ENUM_getFromValue == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "getFromValue method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

void ARSAL_JNI_Manager_FreeERROR_ENUM_JNI(JNIEnv *env)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "%s", "");

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

jobject ARSAL_JNI_Manager_NewERROR_ENUM(JNIEnv *env, eARSAL_ERROR nativeError)
{
    jobject jERROR_ENUM = NULL;
    jint jError;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MANAGER_TAG, "%d", nativeError);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARSAL_JNI_Manager_NewERROR_ENUM_JNI(env);
    }

    if (error == JNI_OK)
    {
        jError = nativeError;

        jERROR_ENUM = (*env)->CallStaticObjectMethod(env, classDTERROR_ENUM, methodId_DTERROR_ENUM_getFromValue, jError);
    }

    return jERROR_ENUM;
}

