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

static jmethodID g_onDeviceAdded;
static jmethodID g_onDeviceRemoved;
static jmethodID g_onReset;

struct jni_ctx {
    struct MuxDiscoveryCtx      *discovery_ctx;
    jobject                     *thizz;
    struct mux_ctx              *muxctx;
};

static JavaVM* g_jvm;

static void device_added_cb(const char *name, uint32_t type, const char *id, void *userdata)
{
    struct jni_ctx* ctx = (struct jni_ctx*)userdata;
    JNIEnv* env = NULL;
    if ((*g_jvm)->GetEnv(g_jvm, (void **)&env, JNI_VERSION_1_6) != JNI_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "thread not attached to JVM");
        return;
    }

    jstring jname = (*env)->NewStringUTF(env, name);
    jstring jid = (*env)->NewStringUTF(env, id);

    (*env)->CallVoidMethod(env, ctx->thizz, g_onDeviceAdded, jname, type, jid);

    if (jname) {
        (*env)->DeleteLocalRef(env, jname);
    }
    if (jid) {
        (*env)->DeleteLocalRef(env, jid);
    }
}

static void device_removed_cb(const char *name, uint32_t type, const char *id, void *userdata)
{
    struct jni_ctx* ctx = (struct jni_ctx*)userdata;
    JNIEnv* env = NULL;
    if ((*g_jvm)->GetEnv(g_jvm, (void **)&env, JNI_VERSION_1_6) != JNI_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "thread not attached to JVM");
        return;
    }

    jstring jname = (*env)->NewStringUTF(env, name);
    jstring jid = (*env)->NewStringUTF(env, id);

    (*env)->CallVoidMethod(env, ctx->thizz, g_onDeviceRemoved, jname, type, jid);

    if (jname) {
        (*env)->DeleteLocalRef(env, jname);
    }
    if (jid) {
        (*env)->DeleteLocalRef(env, jid);
    }
}

static void eof_cb(void *userdata)
{
    struct jni_ctx* ctx = (struct jni_ctx*)userdata;
    JNIEnv* env = NULL;
    if ((*g_jvm)->GetEnv(g_jvm, (void **)&env, JNI_VERSION_1_6) != JNI_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "thread not attached to JVM");
        return;
    }
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "mux eof: restart discovery");

    (*env)->CallVoidMethod(env, ctx->thizz, g_onReset);

    if (ctx->discovery_ctx)
        ARDiscovery_MuxDiscovery_dispose(ctx->discovery_ctx);

    ctx->discovery_ctx = ARDiscovery_MuxDiscovery_new(ctx->muxctx,
                                    &device_added_cb, &device_removed_cb,
                                    &eof_cb, ctx);
    if (!ctx->discovery_ctx)
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error creating MuxDiscovery");
}


static void cleanup(JNIEnv *env, struct jni_ctx *ctx)
{
    if (ctx != NULL) {
        if (ctx->discovery_ctx != NULL) {
            ARDiscovery_MuxDiscovery_dispose(ctx->discovery_ctx);
        }
        if (ctx->thizz != NULL) {
            (*env)->DeleteGlobalRef(env, ctx->thizz);
        }
        free (ctx);
        ctx = NULL;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryMux_nativeClInit (JNIEnv *env, jclass clazz)
{
    jint res = (*env)->GetJavaVM(env, &g_jvm);
    if (res < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Unable to get JavaVM pointer");
    }

    g_onDeviceAdded = (*env)->GetMethodID (env, clazz, "onDeviceAdded", "(Ljava/lang/String;ILjava/lang/String;)V");
    if (!g_onDeviceAdded)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Unable to find method onDeviceAdded");
    }
    g_onDeviceRemoved = (*env)->GetMethodID (env, clazz, "onDeviceRemoved", "(Ljava/lang/String;ILjava/lang/String;)V");
    if (!g_onDeviceAdded)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Unable to find method onDeviceRemoved");
    }
    g_onReset = (*env)->GetMethodID (env, clazz, "onReset", "()V");
    if (!g_onReset)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Unable to find method onReset");
    }
}


JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryMux_nativeNew (JNIEnv *env, jobject thizz, jlong jMuxCtxPtr)
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
    ctx->discovery_ctx = ARDiscovery_MuxDiscovery_new(muxctx, &device_added_cb, &device_removed_cb, &eof_cb, ctx);

    if (ctx == NULL) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error creating MuxDiscovery");
        goto fail;
    }

    return (jlong)(intptr_t)ctx;

fail:
    cleanup(env, ctx);
    return (jlong)(intptr_t)NULL;
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryMux_nativeDispose (JNIEnv *env, jobject thizz, jlong jCtx)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disposing ARDiscoveryMux");
    struct jni_ctx* ctx = (struct jni_ctx*) (intptr_t) jCtx;
    cleanup(env, ctx);
}
