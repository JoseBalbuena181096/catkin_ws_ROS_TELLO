LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libARStream2
LOCAL_DESCRIPTION := Parrot Streaming Library
LOCAL_CATEGORY_PATH := dragon/libs

LOCAL_MODULE_FILENAME := libarstream2.so

LOCAL_LIBRARIES := \
	libARSAL

LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:libmux \
	OPTIONAL:libpomp \
	OPTIONAL:libARMedia

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/Includes \
	$(LOCAL_PATH)/src

LOCAL_CFLAGS := \
	-DHAVE_CONFIG_H

LOCAL_SRC_FILES := \
	gen/Sources/arstream2_error.c \
	src/arstream2_h264_filter.c \
	src/arstream2_h264_filter_error.c \
	src/arstream2_h264_parser.c \
	src/arstream2_h264_sei.c \
	src/arstream2_h264_writer.c \
	src/arstream2_h264.c \
	src/arstream2_rtp_receiver.c \
	src/arstream2_rtp_sender.c \
	src/arstream2_rtp.c \
	src/arstream2_rtp_h264.c \
	src/arstream2_rtcp.c \
	src/arstream2_stream_recorder.c \
	src/arstream2_stream_stats.c \
	src/arstream2_stream_sender.c \
	src/arstream2_stream_receiver.c

LOCAL_INSTALL_HEADERS := \
	Includes/libARStream2/arstream2_error.h:usr/include/libARStream2/ \
	Includes/libARStream2/arstream2_h264_parser.h:usr/include/libARStream2/ \
	Includes/libARStream2/arstream2_h264_sei.h:usr/include/libARStream2/ \
	Includes/libARStream2/arstream2_h264_writer.h:usr/include/libARStream2/ \
	Includes/libARStream2/arstream2_stream_sender.h:usr/include/libARStream2/ \
	Includes/libARStream2/arstream2_stream_receiver.h:usr/include/libARStream2/ \
	Includes/libARStream2/arstream2_stream_stats.h:usr/include/libARStream2/ \
	Includes/libARStream2/arstream2_stream_metadata.h:usr/include/libARStream2/


ifeq ("$(TARGET_OS)","linux")
  ifeq ("$(TARGET_OS_FLAVOUR)","android")
    ANDROID_API_HAS_MMSG = $(shell test $(TARGET_ANDROID_APILEVEL) -ge 21 && echo 1)
    ifeq ("$(ANDROID_API_HAS_MMSG)","1")
      LOCAL_CFLAGS += -DHAS_MMSG
    endif
  else
    LOCAL_CFLAGS += -DHAS_MMSG
  endif
endif

include $(BUILD_LIBRARY)


ifeq ("$(TARGET_OS_FLAVOUR)","native")

include $(LOCAL_PATH)/test/atom.mk

endif
