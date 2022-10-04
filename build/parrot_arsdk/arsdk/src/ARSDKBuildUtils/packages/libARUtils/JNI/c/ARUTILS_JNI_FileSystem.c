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
 * @file ARUTILS_JNI_FileSystem.c
 * @brief libARUtils JNI_FileSystem c file.
 * @date 30/12/2013
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

#include "libARUtils/ARUTILS_Error.h"
#include "libARUtils/ARUTILS_Http.h"
#include "libARUtils/ARUTILS_Ftp.h"
#include "libARUtils/ARUTILS_FileSystem.h"

#include "ARUTILS_JNI.h"

#define ARUTILS_JNI_FILESYSTEM_TAG       "JNI"

/*****************************************
 *
 *             Public implementation:
 *
 *****************************************/

JNIEXPORT jboolean JNICALL Java_com_parrot_arsdk_arutils_ARUtilsFileSystem_nativeStaticInit(JNIEnv *env, jclass jClass)
{
    jboolean jret = JNI_FALSE;
    int error = JNI_OK;

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARUTILS_JNI_NewARUtilsExceptionJNI(env);
    }

    if (error == JNI_OK)
    {
        jret = JNI_TRUE;
    }

    return jret;
}

JNIEXPORT jlong JNICALL Java_com_parrot_arsdk_arutils_ARUtilsFileSystem_nativeGetFileSize(JNIEnv *env, jobject jThis, jstring jNamePath)
{
    const char *nativeNamePath = (*env)->GetStringUTFChars(env, jNamePath, 0);
    int64_t nativeSize = 0;
    jlong jSize = 0;
    eARUTILS_ERROR result = ARUTILS_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_FILESYSTEM_TAG, "%s", "");

    result = ARUTILS_FileSystem_GetFileSize(nativeNamePath, &nativeSize);

    if (result == ARUTILS_OK)
    {
        jSize = (jlong)nativeSize;
    }
    else
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUTILS_JNI_FILESYSTEM_TAG, "error: %d occurred", result);

        ARUTILS_JNI_ThrowARUtilsException(env, result);
    }

    if (nativeNamePath != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jNamePath, nativeNamePath);
    }

    return jSize;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arutils_ARUtilsFileSystem_nativeRename(JNIEnv *env, jobject jThis, jstring jOldName, jstring jNewName)
{
    const char *nativeOldName = (*env)->GetStringUTFChars(env, jOldName, 0);
    const char *nativeNewName = (*env)->GetStringUTFChars(env, jNewName, 0);
    eARUTILS_ERROR result = ARUTILS_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_FILESYSTEM_TAG, "%s", "");

    result = ARUTILS_FileSystem_Rename(nativeOldName, nativeNewName);

    if (nativeOldName != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jOldName, nativeOldName);
    }

    if (nativeNewName != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jNewName, nativeNewName);
    }

    return result;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arutils_ARUtilsFileSystem_nativeRemoveFile(JNIEnv *env, jobject jThis, jstring jLocalPath)
{
    const char *nativeLocalPath = (*env)->GetStringUTFChars(env, jLocalPath, 0);
    eARUTILS_ERROR result = ARUTILS_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_FILESYSTEM_TAG, "%s", "");

    result = ARUTILS_FileSystem_RemoveFile(nativeLocalPath);

    if (nativeLocalPath != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jLocalPath, nativeLocalPath);
    }

    return result;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arutils_ARUtilsFileSystem_nativeRemoveDir(JNIEnv *env, jobject jThis, jstring jLocalPath)
{
    const char *nativeLocalPath = (*env)->GetStringUTFChars(env, jLocalPath, 0);
    eARUTILS_ERROR result = ARUTILS_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_FILESYSTEM_TAG, "%s", "");

    result = ARUTILS_FileSystem_RemoveDir(nativeLocalPath);

    if (nativeLocalPath != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jLocalPath, nativeLocalPath);
    }

    return result;
}

JNIEXPORT jdouble JNICALL Java_com_parrot_arsdk_arutils_ARUtilsFileSystem_nativeGetFreeSpace(JNIEnv *env, jobject jThis, jstring jLocalPath)
{
    const char *nativeLocalPath = (*env)->GetStringUTFChars(env, jLocalPath, 0);
    jdouble jFreeSpace = 0.f;
    double nativeFreeSpace = 0.f;
    eARUTILS_ERROR result = ARUTILS_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_FILESYSTEM_TAG, "%s", "");

    result = ARUTILS_FileSystem_GetFreeSpace(nativeLocalPath, &nativeFreeSpace);

    if (result == ARUTILS_OK)
    {
        jFreeSpace = (jdouble)nativeFreeSpace;
    }
    else
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARUTILS_JNI_FILESYSTEM_TAG, "error: %d occurred", result);

        ARUTILS_JNI_ThrowARUtilsException(env, result);
    }

    if (nativeLocalPath != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jLocalPath, nativeLocalPath);
    }

    return jFreeSpace;
}

/*****************************************
 *
 *             Private implementation:
 *
 *****************************************/



