LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libARUtils
LOCAL_DESCRIPTION := ARSDK Utils
LOCAL_CATEGORY_PATH := dragon/libs

LOCAL_MODULE_FILENAME := libarutils.so

LOCAL_LIBRARIES := \
	libARSAL \
	libARDiscovery \
	libARCommands \
	curl

LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:libmux

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/Includes \
	$(LOCAL_PATH)/Sources

LOCAL_CFLAGS := \
	-DHAVE_CONFIG_H

LOCAL_SRC_FILES := \
	Sources/ARUTILS_FileSystem.c \
	Sources/ARUTILS_Http.c \
	Sources/ARUTILS_Manager.c \
	Sources/ARUTILS_WifiFtp.c \
	gen/Sources/ARUTILS_Error.c

LOCAL_INSTALL_HEADERS := \
	Includes/libARUtils/ARUtils.h:usr/include/libARUtils/ \
	Includes/libARUtils/ARUTILS_Error.h:usr/include/libARUtils/ \
	Includes/libARUtils/ARUTILS_FileSystem.h:usr/include/libARUtils/ \
	Includes/libARUtils/ARUTILS_Ftp.h:usr/include/libARUtils/ \
	Includes/libARUtils/ARUTILS_Http.h:usr/include/libARUtils/ \
	Includes/libARUtils/ARUTILS_Manager.h:usr/include/libARUtils/

ifeq ("$(TARGET_OS)","darwin")
  ifneq ("$(TARGET_OS_FLAVOUR)","native")
LOCAL_SRC_FILES += \
	Sources/ARUTILS_BLEFtp.m
  else
LOCAL_SRC_FILES += \
	Sources/ARUTILS_BLEFtp_stub.c
  endif
else
LOCAL_SRC_FILES += \
	Sources/ARUTILS_BLEFtp_stub.c
endif

include $(BUILD_LIBRARY)
