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
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>

JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_arsal_ARNativeData_allocateData (JNIEnv *env, jobject thizz, jint capacity)
{
    void *ptr = 0;
    /* Alloc only if capacity is positive */
    if (capacity > 0)
    {
        ptr = malloc (capacity);
    }
    return (jlong)(intptr_t)ptr;
}

JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_arsal_ARNativeData_reallocateData (JNIEnv *env, jobject thizz, jlong pointer, jint capacity)
{
    void *ptr = 0;
    /* Realloc only if capacity is positive */
    if (capacity > 0)
    {
      ptr = realloc ((void *)(intptr_t)pointer, capacity);
    }
    return (jlong)(intptr_t)ptr;
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arsal_ARNativeData_freeData (JNIEnv *env, jobject thizz, jlong data)
{
    void *ptr = (void *)(intptr_t)data;
    free (ptr);
}

JNIEXPORT jbyteArray JNICALL
Java_com_parrot_arsdk_arsal_ARNativeData_generateByteArray (JNIEnv *env, jobject thizz, jlong data, jint capacity, jint used)
{
    jbyteArray retArray = NULL;
    if (used <= capacity)
    {
        retArray = (*env)->NewByteArray (env, used);
        if (retArray != NULL)
        {
            (*env)->SetByteArrayRegion (env, retArray, 0, used, (jbyte *) (intptr_t)data);
        }
    }
    return retArray;
}

JNIEXPORT jboolean JNICALL
Java_com_parrot_arsdk_arsal_ARNativeData_copyData (JNIEnv *env, jobject thizz, jlong dst, jint dstCapacity, jlong src, jint srcLen)
{
    jboolean retVal = JNI_FALSE;
    if (srcLen <= dstCapacity)
    {
        void *srcPtr = (void *)(intptr_t)src;
        void *dstPtr = (void *)(intptr_t)dst;
        memcpy (dstPtr, srcPtr, srcLen);
        retVal = JNI_TRUE;
    }
    return retVal;
}

JNIEXPORT jboolean JNICALL
Java_com_parrot_arsdk_arsal_ARNativeData_copyJavaData (JNIEnv *env, jobject thizz, jlong dst, jint dstCapacity, jbyteArray src, jint srcLen)
{
    jboolean retVal = JNI_FALSE;
    if (srcLen <= dstCapacity)
    {
        jbyte* bufferPtr = (*env)->GetByteArrayElements(env, src, NULL);

        void *srcPtr = (void *)(intptr_t)bufferPtr;
        void *dstPtr = (void *)(intptr_t)dst;
        memcpy (dstPtr, srcPtr, srcLen);
        retVal = JNI_TRUE;

        (*env)->ReleaseByteArrayElements(env, src, bufferPtr, 0);
    }
    return retVal;
}
