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
#include <jni.h>
#include <libARStream/ARSTREAM_Reader.h>
#include <libARSAL/ARSAL_Print.h>

#define JNI_READER_TAG "ARSTREAM_JNIReader"

static jmethodID g_cbWrapper_id = 0;
static JavaVM *g_vm = NULL;

static uint8_t* internalCallback (eARSTREAM_READER_CAUSE cause, uint8_t *framePointer, uint32_t frameSize, int numberOfSkippedFrames, int isFlushFrame, uint32_t *newBufferCapacity, void *thizz)
{
    JNIEnv *env = NULL;
    int wasAlreadyAttached = 1;
    int envStatus = (*g_vm)->GetEnv(g_vm, (void **)&env, JNI_VERSION_1_6);
    if (envStatus == JNI_EDETACHED)
    {
        wasAlreadyAttached = 0;
        if ((*g_vm)->AttachCurrentThread(g_vm, &env, NULL) != 0)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, JNI_READER_TAG, "Unable to attach thread to VM");
            *newBufferCapacity = 0;
            return NULL;
        }
    }
    else if (envStatus != JNI_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, JNI_READER_TAG, "Error %d while getting JNI Environment", envStatus);
        *newBufferCapacity = 0;
        return NULL;
    }

    jboolean isFlush = (isFlushFrame == 1) ? JNI_TRUE : JNI_FALSE;
    jlongArray newNativeDataInfos = (*env)->CallObjectMethod(env, (jobject)thizz, g_cbWrapper_id, (jint)cause, (jlong)(intptr_t)framePointer, (jint)frameSize, isFlush, (jint)numberOfSkippedFrames, (jint)*newBufferCapacity);

    uint8_t *retVal = NULL;
    *newBufferCapacity = 0;
    if (newNativeDataInfos != NULL)
    {
        jlong *array = (*env)->GetLongArrayElements(env, newNativeDataInfos, NULL);
        retVal = (uint8_t *)(intptr_t)array[0];
        *newBufferCapacity = (int)array[1];
        (*env)->ReleaseLongArrayElements(env, newNativeDataInfos, array, 0);

        (*env)->DeleteLocalRef (env, newNativeDataInfos);
    }

    if (wasAlreadyAttached == 0)
    {
        (*g_vm)->DetachCurrentThread(g_vm);
    }

    return retVal;
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arstream_ARStreamReader_nativeInitClass (JNIEnv *env, jclass clazz)
{
    jint res = (*env)->GetJavaVM(env, &g_vm);
    if (res < 0)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, JNI_READER_TAG, "Unable to get JavaVM pointer");
    }
    g_cbWrapper_id = (*env)->GetMethodID (env, clazz, "callbackWrapper", "(IJIZII)[J");
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_arstream_ARStreamReader_nativeGetDefaultMaxAckInterval (JNIEnv *env, jclass clazz)
{
    return ARSTREAM_READER_MAX_ACK_INTERVAL_DEFAULT;
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arstream_ARStreamReader_nativeSetDataBufferParams (JNIEnv *env, jclass clazz, jlong cParams, jint id, jint maxFragmentSize, jint maxNumberOfFragment)
{
    ARSTREAM_Reader_InitStreamDataBuffer ((ARNETWORK_IOBufferParam_t *)(intptr_t)cParams, id, maxFragmentSize, maxNumberOfFragment);
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arstream_ARStreamReader_nativeSetAckBufferParams (JNIEnv *env, jclass clazz, jlong cParams, jint id)
{
    ARSTREAM_Reader_InitStreamAckBuffer ((ARNETWORK_IOBufferParam_t *)(intptr_t)cParams, id);
}

JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_arstream_ARStreamReader_nativeConstructor (JNIEnv *env, jobject thizz, jlong cNetManager, jint dataBufferId, jint ackBufferId, jlong frameBuffer, jint frameBufferSize, jint maxFragmentSize, jint maxAckInterval)
{
    eARSTREAM_ERROR err = ARSTREAM_OK;
    jobject g_thizz = (*env)->NewGlobalRef(env, thizz);
    ARSTREAM_Reader_t *retReader = ARSTREAM_Reader_New ((ARNETWORK_Manager_t *)(intptr_t)cNetManager, dataBufferId, ackBufferId, internalCallback, (uint8_t *)(intptr_t)frameBuffer, frameBufferSize, maxFragmentSize, maxAckInterval, (void *)g_thizz, &err);

    if (err != ARSTREAM_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, JNI_READER_TAG, "Error while creating reader : %s", ARSTREAM_Error_ToString (err));
    }
    return (jlong)(intptr_t)retReader;
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arstream_ARStreamReader_nativeRunDataThread (JNIEnv *env, jobject thizz, jlong cReader)
{
    ARSTREAM_Reader_RunDataThread ((void *)(intptr_t)cReader);
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arstream_ARStreamReader_nativeRunAckThread (JNIEnv *env, jobject thizz, jlong cReader)
{
    ARSTREAM_Reader_RunAckThread ((void *)(intptr_t)cReader);
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arstream_ARStreamReader_nativeStop (JNIEnv *env, jobject thizz, jlong cReader)
{
    ARSTREAM_Reader_StopReader ((ARSTREAM_Reader_t *)(intptr_t)cReader);
}

JNIEXPORT jboolean JNICALL
Java_com_parrot_arsdk_arstream_ARStreamReader_nativeDispose (JNIEnv *env, jobject thizz, jlong cReader)
{
    jboolean retVal = JNI_TRUE;
    ARSTREAM_Reader_t *reader = (ARSTREAM_Reader_t *)(intptr_t)cReader;
    void *ref = ARSTREAM_Reader_GetCustom(reader);
    eARSTREAM_ERROR err = ARSTREAM_Reader_Delete (&reader);
    if (err != ARSTREAM_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, JNI_READER_TAG, "Unable to delete reader : %s", ARSTREAM_Error_ToString (err));
        retVal = JNI_FALSE;
    }
    if (retVal == JNI_TRUE && ref != NULL)
    {
        (*env)->DeleteGlobalRef(env, (jobject)ref);
        if ((*env)->ExceptionOccurred(env) != NULL)
        {
            (*env)->ExceptionDescribe(env);
        }
    }
    return retVal;
}

JNIEXPORT jfloat JNICALL
Java_com_parrot_arsdk_arstream_ARStreamReader_nativeGetEfficiency (JNIEnv *env, jobject thizz, jlong cReader)
{
    return ARSTREAM_Reader_GetEstimatedEfficiency ((ARSTREAM_Reader_t *)(intptr_t)cReader);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_arstream_ARStreamReader_nativeAddFilter (JNIEnv *env, jobject thizz, jlong cReader, jlong cFilter)
{
    ARSTREAM_Reader_t *reader = (ARSTREAM_Reader_t *)(intptr_t)cReader;
    ARSTREAM_Filter_t *filter = (ARSTREAM_Filter_t *)(intptr_t)cFilter;
    return ARSTREAM_Reader_AddFilter (reader, filter);
}
