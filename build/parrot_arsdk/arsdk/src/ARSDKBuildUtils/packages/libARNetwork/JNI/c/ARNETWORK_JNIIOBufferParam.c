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
 * @file IOBufferParam_wrapper.c
 * @brief JNI into between the ARNETWORK_IOBufferParam.h and NetworkIOBufferParam.java
 * @date 01/18/2013
 * @author maxime.maitre@parrot.com
 **/

#include <libARNetwork/ARNETWORK_Error.h>
#include <libARNetwork/ARNETWORK_IOBufferParam.h>
#include <libARNetworkAL/ARNETWORKAL_Frame.h>
#include <jni.h>
#include <stdlib.h>

JNIEXPORT int JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeStaticGetInfiniteNumber (JNIEnv *env, jclass class)
{
    return ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER;
}

JNIEXPORT int JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeStaticGetDataCopyMaxSizeUseMax (JNIEnv *env, jclass class)
{
    return ARNETWORK_IOBUFFERPARAM_DATACOPYMAXSIZE_USE_MAX;
}

JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeNew(JNIEnv *env, jobject obj)
{
    /** local declarations */
    ARNETWORK_IOBufferParam_t* ioBufferParam = malloc (sizeof (ARNETWORK_IOBufferParam_t));

    ARNETWORK_IOBufferParam_DefaultInit (ioBufferParam);

    return (long) ioBufferParam;
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeDelete(JNIEnv *env, jobject obj, jlong jIOBufferParamPtr)
{
    /** local declarations */
    ARNETWORK_IOBufferParam_t* ioBufferParamPtr = (ARNETWORK_IOBufferParam_t*) (intptr_t) jIOBufferParamPtr;

    free(ioBufferParamPtr);
}


JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeSetId(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr, jint ID)
{
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        param->ID = ID;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeSetDataType(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr, jint type)
{
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        param->dataType = type;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeSetTimeBetweenSend(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr, jint time)
{
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        param->sendingWaitTimeMs = time;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeSetAckTimeoutMs(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr, jint time)
{
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        param->ackTimeoutMs = time;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeSetNumberOfRetry(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr, jint nb)
{
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        param->numberOfRetry = nb;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeSetNumberOfCell(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr, jint nb)
{
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        param->numberOfCell = nb;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeSetDataCopyMaxSize(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr, jint size)
{
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        param->dataCopyMaxSize = size;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeSetIsOverwriting(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr, jboolean over)
{
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        param->isOverwriting = (over == JNI_TRUE) ? 1 : 0;
    }
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeGetId(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr)
{
    jint retVal = -1;
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        retVal = param->ID;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeGetDataType(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr)
{
    jint retVal = -1;
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        retVal = param->dataType;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeGetTimeBetweenSend(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr)
{
    jint retVal = -1;
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        retVal = param->sendingWaitTimeMs;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeGetAckTimeoutMs(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr)
{
    jint retVal = -1;
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        retVal = param->ackTimeoutMs;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeGetNumberOfRetry(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr)
{
    jint retVal = -1;
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        retVal = param->numberOfRetry;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeGetNumberOfCell(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr)
{
    jint retVal = -1;
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        retVal = param->numberOfCell;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeGetDataCopyMaxSize(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr)
{
    jint retVal = -1;
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        retVal = param->dataCopyMaxSize;
    }
    return retVal;
}

JNIEXPORT jboolean JNICALL
Java_com_parrot_arsdk_arnetwork_ARNetworkIOBufferParam_nativeGetIsOverwriting(JNIEnv *env, jobject thizz, jlong jIOBufferParamPtr, jboolean over)
{
    jboolean retVal = JNI_FALSE;
    if (jIOBufferParamPtr != 0)
    {
        ARNETWORK_IOBufferParam_t *param = (ARNETWORK_IOBufferParam_t *) (intptr_t) jIOBufferParamPtr;
        retVal = (param->isOverwriting == 1) ? JNI_TRUE : JNI_FALSE;
    }
    return retVal;
}
