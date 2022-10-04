LOCAL_PATH := $(call my-dir)

# JNI Wrapper
include $(CLEAR_VARS)

LOCAL_CFLAGS := -g
LOCAL_MODULE := libarnetwork_android
LOCAL_SRC_FILES := JNI/c/ARNETWORK_JNIIOBufferParam.c JNI/c/ARNETWORK_JNIManager.c
LOCAL_LDLIBS := -llog -lz
LOCAL_SHARED_LIBRARIES := \
	libARNetwork \
	libARSAL

include $(BUILD_SHARED_LIBRARY)
