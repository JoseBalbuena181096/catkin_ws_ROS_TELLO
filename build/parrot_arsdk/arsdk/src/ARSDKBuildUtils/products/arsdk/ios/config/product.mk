
ARSDK_IOS_CONFIG_DIR := $(call my-dir)

# Include common product.mk
include $(ARSDK_IOS_CONFIG_DIR)/../../common/config/product.mk

# Override alchemy default AR
TARGET_AR := $(shell xcrun --find --sdk iphoneos ar)

# Setup TARGET_OS
TARGET_OS := darwin
TARGET_OS_FLAVOUR := iphoneos
TARGET_ARCH := arm

TARGET_FORCE_STATIC := 1
TARGET_IPHONE_VERSION := 7.0

TARGET_GLOBAL_CFLAGS += -fembed-bitcode
TARGET_GLOBAL_OBJCFLAGS += -fobjc-arc
