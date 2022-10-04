LOCAL_PATH := $(call my-dir)

# JNI Wrapper
include $(CLEAR_VARS)

LOCAL_CFLAGS := -g -I$(LOCAL_PATH)/Sources
LOCAL_MODULE := libardiscovery_android
LOCAL_SRC_FILES := \
    JNI/c/ARDISCOVERY_JNI_Connection.c \
    JNI/c/ARDISCOVERY_JNI_Device.c \
    JNI/c/ARDISCOVERY_JNI_Discovery.c \
    JNI/c/ARDISCOVERY_JNI_DEVICE_Ble.c \
    JNI/c/ARDISCOVERY_JNI_MuxDiscovery.c \
    JNI/c/ARDISCOVERY_JNI_MuxConnection.c

LOCAL_LDLIBS := -llog -lz
LOCAL_SHARED_LIBRARIES := \
	libARDiscovery \
	json \
	libARSAL \
	libARNetworkAL \
	libmux

include $(BUILD_SHARED_LIBRARY)
