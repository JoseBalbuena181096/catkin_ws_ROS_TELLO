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
#include <libARSAL/ARSAL_Print.h>

static void ARSAL_Java_Log(eARSAL_PRINT_LEVEL level, const char *tag, const char *message)
{
    ARSAL_PRINT(level, tag, "%s", message);
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arsal_ARSALPrint_nativePrint(JNIEnv *env, jclass clazz, jint level, jstring tag, jstring message)
{
    const char *c_tag = (*env)->GetStringUTFChars(env, tag, NULL);
    const char *c_message = (*env)->GetStringUTFChars(env, message, NULL);
    ARSAL_Java_Log(level, c_tag, c_message);
    (*env)->ReleaseStringUTFChars(env, tag, c_tag);
    (*env)->ReleaseStringUTFChars(env, message, c_message);
}

JNIEXPORT jboolean JNICALL
Java_com_parrot_arsdk_arsal_ARSALPrint_nativeSetMinLevel(JNIEnv *env, jclass clazz, jint level)
{
    int ret = ARSAL_Print_SetMinimumLevel(level);
    return (ret == 0) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_arsal_ARSALPrint_nativeGetMinLevel(JNIEnv *env, jclass clazz)
{
    return ARSAL_Print_GetMinimumLevel();
}
