LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libARDataTransfer
LOCAL_DESCRIPTION := ARSDK DataTransfer
LOCAL_CATEGORY_PATH := dragon/libs

LOCAL_MODULE_FILENAME := libardatatransfer.so

LOCAL_LIBRARIES := libARSAL libARDiscovery libARCommands libARUtils

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/Includes \
	$(LOCAL_PATH)/Sources

LOCAL_CFLAGS := \
	-DHAVE_CONFIG_H

LOCAL_SRC_FILES := \
	Sources/ARDATATRANSFER_DataDownloader.c \
	Sources/ARDATATRANSFER_Downloader.c \
	Sources/ARDATATRANSFER_Manager.c \
	Sources/ARDATATRANSFER_MediasDownloader.c \
	Sources/ARDATATRANSFER_MediasQueue.c \
	Sources/ARDATATRANSFER_Uploader.c \
	gen/Sources/ARDATATRANSFER_Error.c

LOCAL_INSTALL_HEADERS := \
	Includes/libARDataTransfer/ARDataTransfer.h:usr/include/libARDataTransfer/ \
	Includes/libARDataTransfer/ARDATATRANSFER_DataDownloader.h:usr/include/libARDataTransfer/ \
	Includes/libARDataTransfer/ARDATATRANSFER_Downloader.h:usr/include/libARDataTransfer/ \
	Includes/libARDataTransfer/ARDATATRANSFER_Error.h:usr/include/libARDataTransfer/ \
	Includes/libARDataTransfer/ARDATATRANSFER_Manager.h:usr/include/libARDataTransfer/ \
	Includes/libARDataTransfer/ARDATATRANSFER_MediasDownloader.h:usr/include/libARDataTransfer/ \
	Includes/libARDataTransfer/ARDATATRANSFER_Uploader.h:usr/include/libARDataTransfer/

include $(BUILD_LIBRARY)
