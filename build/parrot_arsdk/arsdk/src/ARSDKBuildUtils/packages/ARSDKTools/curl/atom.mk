ifdef ARSDK_BUILD_FOR_APP

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := curl
LOCAL_DESCRIPTION := cURL
LOCAL_CATEGORY_PATH := libs

LOCAL_AUTOTOOLS_VERSION := 7.36.0
LOCAL_AUTOTOOLS_ARCHIVE := curl-$(LOCAL_AUTOTOOLS_VERSION).tar.bz2
LOCAL_AUTOTOOLS_SUBDIR := curl-$(LOCAL_AUTOTOOLS_VERSION)
LOCAL_AUTOTOOLS_PATCHES := \
	0001-ARSDK-Allow-mixed-32-64bits-builds.patch \
	0002-ARSDK-Allow-iOS-x86_64-builds-for-simulators.patch \
	avoid-version.patch

ifeq ("$(TARGET_OS)","darwin")
LOCAL_AUTOTOOLS_PATCHES += \
	0003-ARSDK-fix-clock_gettime-ios.patch
endif

LOCAL_EXPORT_LDLIBS := -lcurl
LOCAL_LIBRARIES := libressl

LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
	--enable-ipv6 \
	--disable-ares \
	--disable-file \
	--disable-ldap \
	--disable-ldaps \
	--disable-rtsp \
	--disable-proxy \
	--disable-dict \
	--disable-telnet \
	--disable-tftp \
	--disable-pop3 \
	--disable-imap \
	--disable-smtp \
	--disable-gopher \
	--disable-manual \
	--disable-verbose \
	--disable-sspi \
	--disable-ntlm-wb \
	--disable-tls-srp \
	--without-winssl \
	--without-gnutls \
	--without-polarssl \
	--without-cyassl \
	--without-nss \
	--without-axtls \
	--without-ca-bundle \
	--without-libmetalink \
	--without-libssh2 \
	--without-librtmp \
	--without-winidn

ifeq ("$(TARGET_OS_FLAVOUR)","android")
LOCAL_AUTOTOOLS_CONFIGURE_ARGS += \
	--disable-so-version
endif

# User define command to be launch before configure step.
# Generates files used by configure
define LOCAL_AUTOTOOLS_CMD_POST_UNPACK
	$(Q) rm -f $(PRIVATE_SRC_DIR)/src/tool_hugehelp.c
endef

include $(BUILD_AUTOTOOLS)

endif
