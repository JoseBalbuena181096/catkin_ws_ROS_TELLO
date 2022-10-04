LOCAL_PATH := $(call my-dir)

# JNI Wrapper
include $(CLEAR_VARS)

LOCAL_CFLAGS := -g
LOCAL_MODULE := libarstream_android
LOCAL_SRC_FILES := JNI/c/ARSTREAM_JNIReader.c JNI/c/ARSTREAM_JNISender.c
LOCAL_LDLIBS := -llog -lz
LOCAL_SHARED_LIBRARIES := \
	libARStream \
	libARSAL

include $(BUILD_SHARED_LIBRARY)
