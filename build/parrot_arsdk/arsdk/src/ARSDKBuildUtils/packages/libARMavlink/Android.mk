LOCAL_PATH := $(call my-dir)

# JNI Wrapper
include $(CLEAR_VARS)

LOCAL_CFLAGS := -g
LOCAL_MODULE := libarmavlink_android
LOCAL_SRC_FILES := JNI/c/ARMAVLINK_JNIListUtils.c JNI/c/ARMAVLINK_JNIFileParser.c JNI/c/ARMAVLINK_JNIFileGenerator.c JNI/c/ARMAVLINK_JNIMissionItemUtils.c
LOCAL_LDLIBS := -llog -lz
LOCAL_SHARED_LIBRARIES := libARMavlink libARSAL
include $(BUILD_SHARED_LIBRARY)
