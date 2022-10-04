LOCAL_PATH := $(call my-dir)

# JNI Wrapper
include $(CLEAR_VARS)

LOCAL_CFLAGS := -g
LOCAL_MODULE := libarsal_android
LOCAL_SRC_FILES := JNI/c/ARSAL_JNI_Manager.c JNI/c/ARSAL_JNI_Print.c JNI/c/ARSAL_JNINativeData.c JNI/c/ARSAL_JNI_Md5Manager.c
LOCAL_LDLIBS := -llog -lz
LOCAL_SHARED_LIBRARIES := libARSAL
include $(BUILD_SHARED_LIBRARY)
