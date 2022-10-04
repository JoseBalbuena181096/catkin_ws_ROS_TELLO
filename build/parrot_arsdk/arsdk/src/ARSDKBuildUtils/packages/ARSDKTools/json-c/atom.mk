LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := json
LOCAL_DESCRIPTION := JSON-C
LOCAL_CATEGORY_PATH := libs

LOCAL_AUTOTOOLS_VERSION := 0.12.1
LOCAL_AUTOTOOLS_ARCHIVE := json-c-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := json-c-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_EXPORT_LDLIBS := -ljson-c

# Compatibility link
LOCAL_CREATE_LINKS := \
	usr/include/json:json-c \
	usr/lib/libjson.a:libjson-c.a

ifneq ("$(TARGET_FORCE_STATIC)","1")
LOCAL_CREATE_LINKS += \
	usr/lib/libjson.so:libjson-c.so
endif

# Do not error out on implicit fallthrough in switch statements. json-c is
# full of them, and they are all intentional. There is no easy way to disable
# this specific type of warnings, so we stop treating all warnings as errors.
LOCAL_CFLAGS += -Wno-error

# Remove so version for android shared libraries
#ifeq ("$(TARGET_OS_FLAVOUR)","android")
LOCAL_AUTOTOOLS_PATCHES := 0001-android_avoid_so_version.patch

# If targetting an API level before 21, also apply the following patches
ifneq ("$(firstword $(sort $(TARGET_ANDROID_APILEVEL) 21))", "21")
LOCAL_AUTOTOOLS_PATCHES += 0002-android_avoid_isinf.patch
endif

#endif


# Don't redefine alloc functions
LOCAL_AUTOTOOLS_CONFIGURE_ENV := \
	ac_cv_func_malloc_0_nonnull=yes \
	ac_cv_func_realloc_0_nonnull=yes \

include $(BUILD_AUTOTOOLS)
