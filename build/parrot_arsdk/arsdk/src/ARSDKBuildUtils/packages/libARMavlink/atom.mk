LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libARMavlink
LOCAL_DESCRIPTION := ARSDK Mavlink
LOCAL_CATEGORY_PATH := dragon/libs

LOCAL_MODULE_FILENAME := libarmavlink.so

LOCAL_LIBRARIES := libARSAL

# the two following lines are for generating mavlink C headers
LOCAL_CUSTOM_MACROS := \
	mavgen-macro:C,generated,$(LOCAL_PATH)/message_definitions/parrot.xml

LOCAL_EXPORT_C_INCLUDES += \
	$(call local-get-build-dir)/generated

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/Includes \
	$(LOCAL_PATH)/Sources

LOCAL_CFLAGS := \
	-DHAVE_CONFIG_H

LOCAL_SRC_FILES := \
	Sources/ARMAVLINK_FileGenerator.c \
	Sources/ARMAVLINK_FileParser.c \
	Sources/ARMAVLINK_ListUtils.c \
	Sources/ARMAVLINK_Manager.c \
	Sources/ARMAVLINK_MissionItemUtils.c \
	gen/Sources/ARMAVLINK_Error.c

LOCAL_INSTALL_HEADERS := \
	Includes/libARMavlink/libARMavlink.h:usr/include/libARMavlink/ \
	Includes/libARMavlink/ARMAVLINK_Error.h:usr/include/libARMavlink/ \
	Includes/libARMavlink/ARMAVLINK_FileGenerator.h:usr/include/libARMavlink/ \
	Includes/libARMavlink/ARMAVLINK_FileParser.h:usr/include/libARMavlink/ \
	Includes/libARMavlink/ARMAVLINK_ListUtils.h:usr/include/libARMavlink/ \
	Includes/libARMavlink/ARMAVLINK_Manager.h:usr/include/libARMavlink/ \
	Includes/libARMavlink/ARMAVLINK_MissionItemUtils.h:usr/include/libARMavlink/

# Install generated mavlink headers
define LOCAL_CMD_PRE_INSTALL
    $(Q) cp -Raf $(TARGET_OUT_BUILD)/libARMavlink/generated/* $(TARGET_OUT_STAGING)/usr/include/
endef

include $(BUILD_LIBRARY)