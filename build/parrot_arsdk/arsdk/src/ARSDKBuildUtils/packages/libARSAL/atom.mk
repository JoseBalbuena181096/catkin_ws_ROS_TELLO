LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libARSAL
LOCAL_DESCRIPTION := ARSDK Software Abstraction Layer
LOCAL_CATEGORY_PATH := dragon/libs

LOCAL_MODULE_FILENAME := libarsal.so

LOCAL_LIBRARIES :=

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/Includes \
	$(LOCAL_PATH)/Sources

# The config.h that was previousliy generated is now committed in Config dir
# Select correct one to export for all other modules
# TODO: either use an alchemy generated one or re-create a single configure
# just for that.
ifeq ("$(TARGET_OS)","linux")
  ifeq ("$(TARGET_OS_FLAVOUR)","android")
    LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/Config/android
  else
    LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/Config/linux
  endif
else ifeq ("$(TARGET_OS)","darwin")
  ifeq ("$(TARGET_OS_FLAVOUR)","native")
    LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/Config/darwin
  else
    LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/Config/ios
  endif
endif

LOCAL_CFLAGS := \
	-DHAVE_CONFIG_H

LOCAL_SRC_FILES := \
	Sources/ARSAL_Ftw.c \
	Sources/ARSAL_MD5.c \
	Sources/ARSAL_MD5_Manager.c \
	Sources/ARSAL_Mutex.c \
	Sources/ARSAL_Print.c \
	Sources/ARSAL_Sem.c \
	Sources/ARSAL_Socket.c \
	Sources/ARSAL_Time.c \
	Sources/ARSAL_Thread.c \
	Sources/md5.c \
	gen/Sources/ARSAL_Error.c

LOCAL_INSTALL_HEADERS := \
	Includes/libARSAL/ARSAL.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_Endianness.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_Error.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_Ftw.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_MD5_Manager.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_Mutex.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_Print.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_Sem.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_Singleton.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_Socket.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_Thread.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_Time.h:usr/include/libARSAL/

ifeq ("$(TARGET_OS_FLAVOUR)","android")
LOCAL_LDLIBS += -llog
endif

ifeq ("$(TARGET_OS)","darwin")
LOCAL_SRC_FILES += \
	Sources/ARSAL_BLEManager.m \
	Sources/ARSAL_CentralManager.m
LOCAL_INSTALL_HEADERS += \
	Includes/libARSAL/ARSAL_BLEManager.h:usr/include/libARSAL/ \
	Includes/libARSAL/ARSAL_CentralManager.h:usr/include/libARSAL/
LOCAL_LDLIBS += \
	-framework Foundation \
	-framework CoreBluetooth
endif

include $(BUILD_LIBRARY)
