LOCAL_PATH := $(call my-dir)

# JNI Wrapper
include $(CLEAR_VARS)

LOCAL_CFLAGS := -g
LOCAL_MODULE := libarupdater_android
LOCAL_SRC_FILES := JNI/c/ARUPDATER_JNI_Uploader.c JNI/c/ARUPDATER_JNI_Downloader.c JNI/c/ARUPDATER_JNI_Manager.c
LOCAL_LDLIBS := -llog -lz
LOCAL_SHARED_LIBRARIES := \
	libARUpdater \
	libARDiscovery \
	libARSAL

include $(BUILD_SHARED_LIBRARY)
