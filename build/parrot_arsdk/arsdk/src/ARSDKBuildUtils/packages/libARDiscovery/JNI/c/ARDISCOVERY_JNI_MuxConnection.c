/*
    Copyright (C) 2016 Parrot SA

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

#include <libARSAL/ARSAL_Print.h>
#include <libARDiscovery/ARDISCOVERY_MuxDiscovery.h>

#define TAG "ARDISCOVERY_JNI_MuxDiscovery"

static jmethodID g_onDeviceConnected;

struct jni_ctx {
    struct MuxConnectionCtx     *connection_ctx;
    jobject                     *thizz;
    struct mux_ctx              *muxctx;
};

static JavaVM* g_jvm;

static void device_conn_resp_cb(uint32_t status, const char* json, void *userdata)
{
    struct jni_ctx* ctx = (struct jni_ctx*)userdata;
    JNIEnv* env = NULL;
    if ((*g_jvm)->GetEnv(g_jvm, (void **)&env, JNI_VERSION_1_6) != JNI_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "thread not attached to JVM");
        return;
    }

    jstring jjson = (*env)->NewStringUTF(env, json);

    (*env)->CallVoidMethod(env, ctx->thizz, g_onDeviceConnected, (jint)status, jjson);

    if (jjson) {
        (*env)->DeleteLocalRef(env, jjson);
    }

}


static void cleanup(JNIEnv *env, struct jni_ctx *ctx)
{
    if (ctx != NULL) {
        if (ctx->connection_ctx != NULL) {
            ARDiscovery_MuxConnection_dispose(ctx->connection_ctx);
        }
        if (ctx->thizz != NULL) {
            (*env)->DeleteGlobalRef(env, ctx->thizz);
        }
        free (ctx);
        ctx = NULL;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryMux_nativeClInitConnection (JNIEnv *env, jclass clazz)
{
    jint res = (*env)->GetJavaVM(env, &g_jvm);
    if (res < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Unable to get JavaVM pointer");
    }

    g_onDeviceConnected = (*env)->GetMethodID (env, clazz, "onDeviceConnected", "(ILjava/lang/String;)V");
    if (!g_onDeviceConnected)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Unable to find method onDeviceConnected");
    }
}


JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryMux_nativeNewConnection (JNIEnv *env, jobject thizz, jlong jMuxCtxPtr)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Creating new ARDiscoveryMux");
    struct jni_ctx *ctx = calloc(1, sizeof(*ctx));
    if (ctx == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error allocating ARMuxDiscover context");
        goto fail;
    }

    struct mux_ctx* muxctx = (struct mux_ctx*) (intptr_t) jMuxCtxPtr;
    ctx->thizz = (*env)->NewGlobalRef(env, thizz);
    if (ctx->thizz == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error creating object global ref");
        goto fail;
    }

    ctx->muxctx = muxctx;
    ctx->connection_ctx = ARDiscovery_MuxConnection_new(muxctx, &device_conn_resp_cb, ctx);

    if (ctx == NULL) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error creating MuxDiscovery");
        goto fail;
    }

    return (jlong)(intptr_t)ctx;

fail:
    cleanup(env, ctx);
    return (jlong)(intptr_t)NULL;
}


JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryMux_nativeSendConnectRequest(JNIEnv *env, jobject thizz, jlong jCtx,
        jstring controllerName, jstring controllerType, jstring deviceId, jstring json)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Send connection request");
    struct jni_ctx* ctx = (struct jni_ctx*) (intptr_t) jCtx;

    const char *cControlerName = (*env)->GetStringUTFChars(env, controllerName, 0);
    const char *cControllerType = (*env)->GetStringUTFChars(env, controllerType, 0);
    const char *cDeviceId = (*env)->GetStringUTFChars(env, deviceId, 0);
    const char *cJson = (*env)->GetStringUTFChars(env, json, 0);

    int ret = ARDiscovery_MuxConnection_sendConnReq(ctx->connection_ctx, cControlerName,
            cControllerType, cDeviceId, cJson);

    (*env)->ReleaseStringUTFChars(env, controllerName, cControlerName);
    (*env)->ReleaseStringUTFChars(env, controllerType, cControllerType);
    (*env)->ReleaseStringUTFChars(env, deviceId, cDeviceId);
    (*env)->ReleaseStringUTFChars(env, json, cJson);

    return (jint)ret;
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryMux_nativeDisposeConnection (JNIEnv *env, jobject thizz, jlong jCtx)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disposing ARDiscoveryMux");
    struct jni_ctx* ctx = (struct jni_ctx*) (intptr_t) jCtx;
    cleanup(env, ctx);
}
