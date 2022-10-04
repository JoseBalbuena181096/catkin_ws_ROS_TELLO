LOCAL_PATH := $(call my-dir)

# JNI Wrapper
include $(CLEAR_VARS)

LOCAL_CFLAGS := -g
LOCAL_MODULE := libarstream2_android
LOCAL_SRC_FILES := JNI/c/arstream2_stream_receiver_jni.c
LOCAL_LDLIBS := -llog -lz
LOCAL_SHARED_LIBRARIES := \
	libARStream2 \
	libARSAL

include $(BUILD_SHARED_LIBRARY)
