LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libARNetwork
LOCAL_DESCRIPTION := ARSDK Network Control Library
LOCAL_CATEGORY_PATH := dragon/libs

LOCAL_MODULE_FILENAME := libarnetwork.so

LOCAL_LIBRARIES := libARSAL libARNetworkAL

LOCAL_CONFIG_FILES := Config.in
$(call load-config)

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/Includes \
	$(LOCAL_PATH)/Sources

LOCAL_CFLAGS := \
	-DHAVE_CONFIG_H

LOCAL_SRC_FILES := \
	Sources/ARNETWORK_IOBuffer.c \
	Sources/ARNETWORK_IOBufferParam.c \
	Sources/ARNETWORK_Manager.c \
	Sources/ARNETWORK_Receiver.c \
	Sources/ARNETWORK_RingBuffer.c \
	Sources/ARNETWORK_Sender.c \
	gen/Sources/ARNETWORK_Error.c

LOCAL_INSTALL_HEADERS := \
	Includes/libARNetwork/ARNetwork.h:usr/include/libARNetwork/ \
	Includes/libARNetwork/ARNETWORK_Error.h:usr/include/libARNetwork/ \
	Includes/libARNetwork/ARNETWORK_IOBufferParam.h:usr/include/libARNetwork/ \
	Includes/libARNetwork/ARNETWORK_Manager.h:usr/include/libARNetwork/

include $(BUILD_LIBRARY)
