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
 * @file ARSAL_Md5Manager.c
 * @brief libARSAL JNI_Md5Manager c file.
 * @date 03/06/2014
 * @author david.flattin.ext@parrot.com
 **/
#define DEBUG

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

#include <libARSAL/ARSAL_MD5_Manager.h>

#include "ARSAL_JNI.h"

#define ARSAL_JNI_MD5_MANAGER_TAG   "Jni"

jclass classARSALMd5 = NULL;
jmethodID methodId_ARSALMd5_check = NULL;
jmethodID methodId_ARSALMd5_compute = NULL;


int ARSAL_JNI_Md5Manager_NewARSALMd5_JNI(JNIEnv *env)
{
    jclass localClassARSALMd5 = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%s", "");

    if (classARSALMd5 == NULL)
    {
        localClassARSALMd5 = (*env)->FindClass(env, "com/parrot/arsdk/arsal/ARSALMd5");

        if (localClassARSALMd5 == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "ARSALMd5 class not found");
            error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            classARSALMd5 = (*env)->NewGlobalRef(env, localClassARSALMd5);

            if (classARSALMd5 == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "ARSALMd5 global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_ARSALMd5_check = (*env)->GetMethodID(env, classARSALMd5, "check", "(Ljava/lang/String;Ljava/lang/String;)Z");

            if (methodId_ARSALMd5_check == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "check method not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            methodId_ARSALMd5_compute = (*env)->GetMethodID(env, classARSALMd5, "compute", "(Ljava/lang/String;)[B");

            if (methodId_ARSALMd5_compute == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "compute method not found");
                error = JNI_FAILED;
            }
        }
    }

    return error;
}

eARSAL_ERROR ARSAL_JNI_MD5_Check(void *md5Object, const char *filePath, const char *md5Txt)
{
    jobject jARSALMd5Object = (jobject)md5Object;
    eARSAL_ERROR result = ARSAL_OK;
    JNIEnv *env = NULL;
    jstring jFilePath = NULL;
    jstring jMd5Txt = NULL;
    jint jResultEnv = 0;
    jboolean jCheckResult = JNI_FALSE;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%s", "");

    jResultEnv = (*ARSAL_JNI_Manager_VM)->GetEnv(ARSAL_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);

    if (jResultEnv == JNI_EDETACHED)
    {
         (*ARSAL_JNI_Manager_VM)->AttachCurrentThread(ARSAL_JNI_Manager_VM, &env, NULL);
    }

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if ((error == JNI_OK) && (filePath != NULL))
    {
        jFilePath = (*env)->NewStringUTF(env, filePath);

        if (jFilePath == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if ((error == JNI_OK) && (md5Txt != NULL))
    {
        jMd5Txt = (*env)->NewStringUTF(env, md5Txt);

        if (jMd5Txt == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if ((error == JNI_OK) && (jARSALMd5Object != NULL) && (methodId_ARSALMd5_check != NULL))
    {
        jCheckResult = (*env)->CallBooleanMethod(env, jARSALMd5Object, methodId_ARSALMd5_check, jFilePath, jMd5Txt);

        if (jCheckResult != JNI_TRUE)
        {
            result = ARSAL_ERROR_MD5;
        }
    }

    if (jFilePath != NULL)
    {
        (*env)->DeleteLocalRef(env, jFilePath);
    }

    if (jMd5Txt != NULL)
    {
        (*env)->DeleteLocalRef(env, jMd5Txt);
    }

    if ((jResultEnv == JNI_EDETACHED) && (env != NULL))
    {
         (*ARSAL_JNI_Manager_VM)->DetachCurrentThread(ARSAL_JNI_Manager_VM);
    }

    return result;
}

eARSAL_ERROR ARSAL_JNI_MD5_Compute(void *md5Object, const char *filePath, uint8_t *md5, int md5Size)
{
    jobject jARSALMd5Object = (jobject)md5Object;
    eARSAL_ERROR result = ARSAL_OK;
    JNIEnv *env = NULL;
    jstring jFilePath = NULL;
    jobject  jMd5 = NULL;
    jint jResultEnv = 0;
    int md5ArrayLen;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%s", "");

    jResultEnv = (*ARSAL_JNI_Manager_VM)->GetEnv(ARSAL_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);

    if (jResultEnv == JNI_EDETACHED)
    {
         (*ARSAL_JNI_Manager_VM)->AttachCurrentThread(ARSAL_JNI_Manager_VM, &env, NULL);
    }

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if ((error == JNI_OK) && (filePath != NULL))
    {
        jFilePath = (*env)->NewStringUTF(env, filePath);

        if (jFilePath == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if ((error == JNI_OK) && (jARSALMd5Object != NULL) && (methodId_ARSALMd5_compute != NULL))
    {
        jMd5 = (*env)->CallObjectMethod(env, jARSALMd5Object, methodId_ARSALMd5_compute, jFilePath);

        if (jMd5 == NULL)
        {
            error = JNI_FAILED;
        }
    }

    if ((error == JNI_OK) && (jMd5 != NULL))
    {
        md5ArrayLen = (*env)->GetArrayLength(env, jMd5);
    }

    if (md5ArrayLen > md5Size)
    {
        error = JNI_FAILED;
    }

    if ((error == JNI_OK) && (jMd5 != NULL))
    {
        (*env)->GetByteArrayRegion(env, jMd5, 0, md5ArrayLen, (jbyte *)md5);
    }

    if (jFilePath != NULL)
    {
        (*env)->DeleteLocalRef(env, jFilePath);
    }

    if (jMd5 != NULL)
    {
        (*env)->DeleteLocalRef(env, jMd5);
    }

    if ((jResultEnv == JNI_EDETACHED) && (env != NULL))
    {
         (*ARSAL_JNI_Manager_VM)->DetachCurrentThread(ARSAL_JNI_Manager_VM);
    }

    return result;
}

JNIEXPORT jboolean JNICALL Java_com_parrot_arsdk_arsal_ARSALMd5Manager_nativeStaticInit(JNIEnv *env, jclass jClass)
{
    jboolean jret = JNI_FALSE;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%x", (int)env);

    if (env == NULL)
    {
        error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = ARSAL_JNI_Md5Manager_NewARSALMd5_JNI(env);
    }

    if (error == JNI_OK)
    {
        error = ARSAL_JNI_Manager_NewARSALExceptionJNI(env);
    }

    if (error == JNI_OK)
    {
        error = ARSAL_JNI_Manager_NewERROR_ENUM_JNI(env);
    }

    if (error == JNI_OK)
    {
        jret = JNI_TRUE;
    }

    return jret;
}

JNIEXPORT jlong JNICALL Java_com_parrot_arsdk_arsal_ARSALMd5Manager_nativeNew(JNIEnv *env, jobject jThis)
{
    ARSAL_MD5_Manager_t *nativeManager = NULL;
    eARSAL_ERROR result = ARSAL_OK;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%s", "");

    nativeManager = ARSAL_MD5_Manager_New(&result);

    if (error == JNI_OK)
    {
        error = ARSAL_JNI_Manager_NewARSALExceptionJNI(env);
    }

    if (error == JNI_OK)
    {
        error = ARSAL_JNI_Manager_NewERROR_ENUM_JNI(env);
    }

    if (error != JNI_OK)
    {
        result = ARSAL_ERROR_SYSTEM;
    }

    if (result != ARSAL_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARSAL_JNI_MD5_MANAGER_TAG, "error: %d occurred", result);

        ARSAL_JNI_Manager_ThrowARSALException(env, result);
    }

    return (long)nativeManager;
}

JNIEXPORT void JNICALL Java_com_parrot_arsdk_arsal_ARSALMd5Manager_nativeDelete(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARSAL_MD5_Manager_t *nativeManager = (ARSAL_MD5_Manager_t*) (intptr_t) jManager;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%s", "");

    ARSAL_MD5_Manager_Delete (&nativeManager);
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arsal_ARSALMd5Manager_nativeInit(JNIEnv *env, jobject jThis, jlong jManager, jobject jARSALMd5)
{
    ARSAL_MD5_Manager_t *nativeManager = (ARSAL_MD5_Manager_t*) (intptr_t) jManager;
    eARSAL_ERROR result = ARSAL_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%s", "");

    nativeManager->md5Object = (void*)(*env)->NewGlobalRef(env, jARSALMd5);
    if (nativeManager->md5Object == NULL)
    {
        result = ARSAL_ERROR_ALLOC;
    }

    if (result == ARSAL_OK)
    {
        nativeManager->md5Check = ARSAL_JNI_MD5_Check;
        nativeManager->md5Compute = ARSAL_JNI_MD5_Compute;
    }

    return result;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arsal_ARSALMd5Manager_nativeClose(JNIEnv *env, jobject jThis, jlong jManager)
{
    ARSAL_MD5_Manager_t *nativeManager = (ARSAL_MD5_Manager_t*) (intptr_t) jManager;
    eARSAL_ERROR result = ARSAL_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%s", "");

    if (nativeManager->md5Object != NULL)
    {
        (*env)->DeleteGlobalRef(env, (jobject)nativeManager->md5Object);
        nativeManager->md5Object = NULL;
    }

    return result;
}

JNIEXPORT jint JNICALL Java_com_parrot_arsdk_arsal_ARSALMd5Manager_nativeCheck(JNIEnv *env, jobject jThis, jlong jManager, jstring jFilePath, jstring jMd5Txt)
{
    ARSAL_MD5_Manager_t *nativeManager = (ARSAL_MD5_Manager_t*) (intptr_t) jManager;
    const char *nativeFilePath = NULL;
    const char *nativeMd5Txt = NULL;
    eARSAL_ERROR result = ARSAL_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%d", (int)nativeManager);

    if ((nativeManager == NULL) || (nativeManager->md5Check == NULL))
    {
        result = ARSAL_ERROR_BAD_PARAMETER;
    }

    if (result == ARSAL_OK)
    {
        nativeFilePath = (*env)->GetStringUTFChars(env, jFilePath, 0);
        if (nativeFilePath == NULL)
        {
            result = ARSAL_ERROR_ALLOC;
        }
    }

    if (result == ARSAL_OK)
    {
        nativeMd5Txt = (*env)->GetStringUTFChars(env, jMd5Txt, 0);

        if (nativeMd5Txt == NULL)
        {
            result = ARSAL_ERROR_ALLOC;
        }
    }

    //ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%d, %s, %s", result, nativeFilePath, nativeMd5Txt);

    if (result == ARSAL_OK)
    {
        result = nativeManager->md5Check(nativeManager->md5Object,  nativeFilePath, nativeMd5Txt);
    }

    if (nativeFilePath != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jFilePath, nativeFilePath);
    }

    if (nativeMd5Txt != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jMd5Txt, nativeMd5Txt);
    }

    return result;
}

JNIEXPORT jbyteArray JNICALL Java_com_parrot_arsdk_arsal_ARSALMd5Manager_nativeCompute(JNIEnv *env, jobject jThis, jlong jManager, jstring jFilePath)
{
    ARSAL_MD5_Manager_t *nativeManager = (ARSAL_MD5_Manager_t*) (intptr_t) jManager;
    const char *nativeFilePath = NULL;
    uint8_t md5Hex[ARSAL_MD5_LENGTH];
    eARSAL_ERROR result = ARSAL_OK;
    jbyteArray jMd5 = NULL;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%s", "");

    if ((nativeManager == NULL) || (nativeManager->md5Compute == NULL))
    {
        result = ARSAL_ERROR_BAD_PARAMETER;
    }

    //ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%d", result);

    if (result == ARSAL_OK)
    {
        nativeFilePath = (*env)->GetStringUTFChars(env, jFilePath, 0);
        if (nativeFilePath == NULL)
        {
            result = ARSAL_ERROR_ALLOC;
        }
    }

    //ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSAL_JNI_MD5_MANAGER_TAG, "%d %s", result, nativeFilePath);

    if (result == ARSAL_OK)
    {
        result = nativeManager->md5Compute(nativeManager->md5Object,  nativeFilePath, md5Hex, sizeof(md5Hex));
    }

    if (result == ARSAL_OK)
    {
        jMd5 = (*env)->NewByteArray(env, sizeof(md5Hex));

        if (jMd5 == NULL)
        {
            result = ARSAL_ERROR_ALLOC;
        }
    }

    if (result == ARSAL_OK)
    {
        (*env)->SetByteArrayRegion(env, jMd5, 0, sizeof(md5Hex), (jbyte*)md5Hex);
    }

    if (nativeFilePath != NULL)
    {
        (*env)->ReleaseStringUTFChars(env, jFilePath, nativeFilePath);
    }

    return jMd5;
}
