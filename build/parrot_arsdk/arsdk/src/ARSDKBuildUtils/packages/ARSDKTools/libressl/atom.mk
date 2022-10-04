LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libressl
LOCAL_DESCRIPTION := LibreSSL
LOCAL_CATEGORY_PATH := libs

LOCAL_AUTOTOOLS_VERSION := 2.2.1
LOCAL_AUTOTOOLS_ARCHIVE := libressl-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := libressl-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_PATCHES := \
	0001-libressl-2.2.1-maintainer-mode.patch \
	0002-libressl-2.2.1-android-compilation.patch \
	0003-libressl-2.2.1-ios-compilation.patch \
	0004-libressl-2.2.1-armv5.patch \
	0005-libressl-2.2.1-android_unified_headers.patch \
	0006-libressl-2.2.1-avoid_version.patch

ifneq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-native")
LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
	--enable-static \
	--disable-shared
endif

LOCAL_EXPORT_LDLIBS := -lssl -lcrypto

# Don't redefine alloc functions
LOCAL_AUTOTOOLS_CONFIGURE_ENV := \
	ac_cv_func_malloc_0_nonnull=yes \
	ac_cv_func_realloc_0_nonnull=yes \

include $(BUILD_AUTOTOOLS)
