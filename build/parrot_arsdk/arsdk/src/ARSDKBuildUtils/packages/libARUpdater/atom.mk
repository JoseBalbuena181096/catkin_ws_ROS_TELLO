LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libARUpdater
LOCAL_DESCRIPTION := ARSDK Updater
LOCAL_CATEGORY_PATH := dragon/libs

LOCAL_MODULE_FILENAME := libarupdater.so

LOCAL_LIBRARIES := \
	libARSAL \
	libARDiscovery \
	libARCommands \
	libARUtils \
	libARDataTransfer \
	json \
	libpuf

LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:libmux \
	OPTIONAL:libpomp

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/Includes \
	$(LOCAL_PATH)/Sources

LOCAL_CFLAGS := \
	-DHAVE_CONFIG_H

LOCAL_SRC_FILES := \
	Sources/ARUPDATER_Downloader.c \
	Sources/ARUPDATER_DownloadInformation.c \
	Sources/ARUPDATER_Manager.c \
	Sources/ARUPDATER_Uploader.c \
	Sources/ARUPDATER_Utils.c \
	gen/Sources/ARUPDATER_Error.c

LOCAL_INSTALL_HEADERS := \
	Includes/libARUpdater/ARUpdater.h:usr/include/libARUpdater/ \
	Includes/libARUpdater/ARUPDATER_Downloader.h:usr/include/libARUpdater/ \
	Includes/libARUpdater/ARUPDATER_Error.h:usr/include/libARUpdater/ \
	Includes/libARUpdater/ARUPDATER_Manager.h:usr/include/libARUpdater/ \
	Includes/libARUpdater/ARUPDATER_Uploader.h:usr/include/libARUpdater/ \
	Includes/libARUpdater/ARUPDATER_Utils.h:usr/include/libARUpdater/

include $(BUILD_LIBRARY)
