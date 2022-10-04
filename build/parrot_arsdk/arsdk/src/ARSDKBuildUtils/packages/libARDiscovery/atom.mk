LOCAL_PATH := $(call my-dir)

###############################################################################
#  libARDiscovery headers
###############################################################################
include $(CLEAR_VARS)

LOCAL_MODULE := libARDiscovery_headers
LOCAL_DESCRIPTION := ARSDK Discovery and Connection Management Layer (Headers only)
LOCAL_CATEGORY_PATH := dragon/libs

LOCAL_INSTALL_HEADERS := \
	Includes/libARDiscovery/ARDiscovery.h:usr/include/libARDiscovery/ \
	Includes/libARDiscovery/ARDISCOVERY_MuxDiscovery.h:usr/include/libARDiscovery/ \
	Includes/libARDiscovery/ARDISCOVERY_Connection.h:usr/include/libARDiscovery/ \
	Includes/libARDiscovery/ARDISCOVERY_Discovery.h:usr/include/libARDiscovery/ \
	Includes/libARDiscovery/ARDISCOVERY_NetworkConfiguration.h:usr/include/libARDiscovery/ \
	Includes/libARDiscovery/ARDISCOVERY_Device.h:usr/include/libARDiscovery/ \
	Includes/libARDiscovery/ARDISCOVERY_Error.h:usr/include/libARDiscovery/

ifndef ARSDK_BUILD_FOR_APP

LOCAL_INSTALL_HEADERS += \
	Includes/libARDiscovery/ARDISCOVERY_AvahiDiscovery.h:usr/include/libARDiscovery/

else ifeq ("$(TARGET_OS)","linux")

ifneq ("$(TARGET_OS_FLAVOUR)","android")

# Linux: use avahi with dbus if available
LOCAL_INSTALL_HEADERS += \
	Includes/libARDiscovery/ARDISCOVERY_AvahiDiscovery.h:usr/include/libARDiscovery/

endif
# ifneq ("$(TARGET_OS_FLAVOUR)","android")

else ifeq ("$(TARGET_OS)","darwin")

# Darwin: use bonjour
LOCAL_INSTALL_HEADERS += \
	Includes/libARDiscovery/ARDISCOVERY_BonjourDiscovery.h:usr/include/libARDiscovery/

ifneq ("$(TARGET_OS_FLAVOUR)","native")

LOCAL_INSTALL_HEADERS += \
	Includes/libARDiscovery/USBAccessoryManager.h:usr/include/libARDiscovery/

endif
# ifneq ("$(TARGET_OS_FLAVOUR)","native")

endif
# ifndef ARSDK_BUILD_FOR_APP

include $(BUILD_CUSTOM)

###############################################################################
#  default libARDiscovery (contains ExternalAccessory framework on iOS)
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := libARDiscovery
LOCAL_DESCRIPTION := ARSDK Discovery and Connection Management Layer
LOCAL_CATEGORY_PATH := dragon/libs

LOCAL_MODULE_FILENAME := libardiscovery.so

LOCAL_LIBRARIES := \
	libARDiscovery_headers \
	libARSAL \
	libARNetwork \
	libARNetworkAL \
	libARCommands \
	json

LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:libmux \
	OPTIONAL:libpomp


LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/Includes \
	$(LOCAL_PATH)/Sources

LOCAL_CFLAGS := \
	-DHAVE_CONFIG_H

LOCAL_SRC_FILES := \
	Sources/ARDISCOVERY_Connection.c \
	Sources/ARDISCOVERY_Discovery.c \
	Sources/ARDISCOVERY_Device.c \
	Sources/ARDISCOVERY_MuxDiscovery.c \
	Sources/Wifi/ARDISCOVERY_DEVICE_Wifi.c \
	Sources/BLE/ARDISCOVERY_DEVICE_Ble.c \
	Sources/Usb/ARDISCOVERY_DEVICE_Usb.c \
	gen/Sources/ARDISCOVERY_Error.c

ifndef ARSDK_BUILD_FOR_APP

# Embedded: Use avahi without dbus
LOCAL_SRC_FILES += \
	Sources/ARDISCOVERY_AvahiDiscovery_nodbus.c

else ifeq ("$(TARGET_OS)","linux")

ifneq ("$(TARGET_OS_FLAVOUR)","android")

# Linux: use avahi with dbus if available
LOCAL_LIBRARIES += avahi

LOCAL_SRC_FILES += \
	Sources/ARDISCOVERY_AvahiDiscovery.c

endif
# ifneq ("$(TARGET_OS_FLAVOUR)","android")

else ifeq ("$(TARGET_OS)","darwin")

# Darwin: use bonjour
LOCAL_SRC_FILES += \
	Sources/ARDISCOVERY_BonjourDiscovery.m

LOCAL_LDLIBS += \
	-framework Foundation \
	-framework CoreBluetooth

ifneq ("$(TARGET_OS_FLAVOUR)","native")

LOCAL_SRC_FILES += \
	Sources/USBAccessoryManager.m \
	Sources/USBAccessoryManagerEA.m \
	Sources/USBAccessoryManagerProxy.m

LOCAL_LDLIBS +=	\
	-framework ExternalAccessory

LOCAL_CFLAGS += -DUSE_USB_ACCESSORY=1

endif
# ifneq ("$(TARGET_OS_FLAVOUR)","native")

endif
# ifndef ARSDK_BUILD_FOR_APP


include $(BUILD_LIBRARY)


###############################################################################
#  libARDiscovery without ExternalAccessory (only for iOS iphoneos)
###############################################################################

ifeq ("$(TARGET_OS)","darwin")
ifneq ("$(TARGET_OS_FLAVOUR)","native")

include $(CLEAR_VARS)

LOCAL_MODULE := libARDiscoveryWithoutEAccessory
LOCAL_DESCRIPTION := ARSDK Discovery and Connection Management Layer \
                     without External Accessory framework
LOCAL_CATEGORY_PATH := dragon/libs

LOCAL_MODULE_FILENAME := libardiscoverywithouteacc.so

LOCAL_LIBRARIES := \
	libARDiscovery_headers \
	libARSAL \
	libARNetwork \
	libARNetworkAL \
	libARCommands \
	json

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/Includes \
	$(LOCAL_PATH)/Sources

LOCAL_CFLAGS := \
	-DHAVE_CONFIG_H

LOCAL_SRC_FILES := \
	Sources/ARDISCOVERY_Connection.c \
	Sources/ARDISCOVERY_Discovery.c \
	Sources/ARDISCOVERY_Device.c \
	Sources/ARDISCOVERY_MuxDiscovery.c \
	Sources/Wifi/ARDISCOVERY_DEVICE_Wifi.c \
	Sources/BLE/ARDISCOVERY_DEVICE_Ble.c \
	Sources/Usb/ARDISCOVERY_DEVICE_Usb.c \
	gen/Sources/ARDISCOVERY_Error.c

# Darwin: use bonjour
LOCAL_SRC_FILES += \
	Sources/ARDISCOVERY_BonjourDiscovery.m \
	Sources/USBAccessoryManager.m \

LOCAL_LDLIBS += \
	-framework Foundation \
	-framework CoreBluetooth

include $(BUILD_LIBRARY)
endif
endif
