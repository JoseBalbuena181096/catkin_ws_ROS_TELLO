LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libpuf
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Helper library for accessing parrot firmware files
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -fvisibility=hidden
LOCAL_SRC_FILES := src/libpuf.c \
	src/libpuf_plf.c \
	src/libpuf_tar.c

LOCAL_LIBRARIES := zlib libtar
LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:libplfng \
	OPTIONAL:libulog

include $(BUILD_LIBRARY)
