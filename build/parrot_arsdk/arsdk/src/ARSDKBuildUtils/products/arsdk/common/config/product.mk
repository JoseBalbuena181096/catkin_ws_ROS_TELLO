
ARSDK_COMMON_CONFIG_DIR := $(call my-dir)

# Select application build for ARSDK (in case the drone firmware defaults is not
# wanted)
ARSDK_BUILD_FOR_APP := 1

TARGET_GLOBAL_CFLAGS += -std=gnu99

custom.libARNetwork.config :=$(ARSDK_COMMON_CONFIG_DIR)/libARNetwork.config
