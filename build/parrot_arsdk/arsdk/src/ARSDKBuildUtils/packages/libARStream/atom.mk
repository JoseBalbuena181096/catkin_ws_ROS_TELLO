LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libARStream
LOCAL_DESCRIPTION := ARSDK Stream library
LOCAL_CATEGORY_PATH := dragon/libs

LOCAL_MODULE_FILENAME := libarstream.so

LOCAL_LIBRARIES := libARSAL libARNetworkAL libARNetwork

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/Includes \
	$(LOCAL_PATH)/Sources

LOCAL_CFLAGS := \
	-DHAVE_CONFIG_H

LOCAL_SRC_FILES := \
	Sources/ARSTREAM_Buffers.c \
	Sources/ARSTREAM_NetworkHeaders.c \
	Sources/ARSTREAM_Reader.c \
	Sources/ARSTREAM_Sender.c \
	gen/Sources/ARSTREAM_Error.c

LOCAL_INSTALL_HEADERS := \
	Includes/libARStream/ARStream.h:usr/include/libARStream/ \
	Includes/libARStream/ARSTREAM_Error.h:usr/include/libARStream/ \
	Includes/libARStream/ARSTREAM_Filter.h:usr/include/libARStream/ \
	Includes/libARStream/ARSTREAM_Reader.h:usr/include/libARStream/  \
	Includes/libARStream/ARSTREAM_Sender.h:usr/include/libARStream/ \

include $(BUILD_LIBRARY)
