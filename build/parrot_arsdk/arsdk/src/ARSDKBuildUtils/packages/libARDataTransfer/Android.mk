LOCAL_PATH := $(call my-dir)

# JNI Wrapper
include $(CLEAR_VARS)

LOCAL_CFLAGS := -g
LOCAL_MODULE := libardatatransfer_android
LOCAL_SRC_FILES := \
	JNI/c/ARDATATRANSFER_JNI_Downloader.c \
	JNI/c/ARDATATRANSFER_JNI_Uploader.c \
	JNI/c/ARDATATRANSFER_JNI_Manager.c \
	JNI/c/ARDATATRANSFER_JNI_MediasDownloader.c \
	JNI/c/ARDATATRANSFER_JNI_DataDownloader.c
LOCAL_LDLIBS := -llog -lz
LOCAL_SHARED_LIBRARIES := \
	libARDataTransfer \
	libARSAL

include $(BUILD_SHARED_LIBRARY)
