
ARSDK_IOS_SIM_CONFIG_DIR := $(call my-dir)

# Include common product.mk
include $(ARSDK_IOS_SIM_CONFIG_DIR)/../../common/config/product.mk

CONFIG_GLOBAL_FILE := $(ARSDK_IOS_SIM_CONFIG_DIR)/../../ios/config/global.config

# Override alchemy default AR
TARGET_AR := $(shell xcrun --find --sdk iphonesimulator ar)

# Setup TARGET_OS
TARGET_OS := darwin
TARGET_OS_FLAVOUR := iphonesimulator
TARGET_ARCH := x64

TARGET_FORCE_STATIC := 1
TARGET_IPHONE_VERSION := 7.0

TARGET_GLOBAL_OBJCFLAGS += -fobjc-arc
