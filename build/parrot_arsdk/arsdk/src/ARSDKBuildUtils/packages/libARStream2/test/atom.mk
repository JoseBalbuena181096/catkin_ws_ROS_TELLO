ifeq ("$(TARGET_OS_FLAVOUR)","native")

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_CATEGORY_PATH := test
LOCAL_MODULE := ARStream2StreamReceiverTest
LOCAL_DESCRIPTION := Parrot Streaming Library - Stream Receiver test program

LOCAL_LIBRARIES := libARSAL libARCommands libARNetwork libARNetworkAL libARDiscovery libARStream2 json

LOCAL_SRC_FILES := arstream2_stream_receiver_test.c

include $(BUILD_EXECUTABLE)

endif
