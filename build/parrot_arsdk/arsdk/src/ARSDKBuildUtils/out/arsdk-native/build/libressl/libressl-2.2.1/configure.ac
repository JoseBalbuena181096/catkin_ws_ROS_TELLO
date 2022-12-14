# Copyright (c) 2014-2015 Brent Cook
#
# Permission to use, copy, modify, and distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

AC_INIT([libressl], m4_esyscmd([tr -d '\n' < VERSION]))
AC_SUBST([LIBCRYPTO_VERSION], m4_esyscmd([tr -d '\n' < crypto/VERSION]))
AC_SUBST([LIBSSL_VERSION], m4_esyscmd([tr -d '\n' < ssl/VERSION]))
AC_SUBST([LIBTLS_VERSION], m4_esyscmd([tr -d '\n' < tls/VERSION]))

AC_CANONICAL_HOST
AM_INIT_AUTOMAKE([subdir-objects foreign])
AC_CONFIG_MACRO_DIR([m4])

AM_MAINTAINER_MODE([enable])

m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

# This must be saved before AC_PROG_CC
USER_CFLAGS="$CFLAGS"

AC_PROG_CC
AC_PROG_CC_STDC
AM_PROG_CC_C_O
AC_PROG_LIBTOOL
LT_INIT

CHECK_OS_OPTIONS

CHECK_C_HARDENING_OPTIONS

DISABLE_AS_EXECUTABLE_STACK
AM_PROG_AS

DISABLE_COMPILER_WARNINGS

# Check if the certhash command should be built
AC_CHECK_FUNCS([symlink])
AM_CONDITIONAL([BUILD_CERTHASH], [test "x$ac_cv_func_symlink" = xyes])

# Check if funopen exists
AC_CHECK_FUNC([funopen])

CHECK_LIBC_COMPAT
CHECK_LIBC_CRYPTO_COMPAT
CHECK_VA_COPY

AC_CHECK_HEADERS([err.h])

AC_ARG_WITH([openssldir],
	AS_HELP_STRING([--with-openssldir],
		       [Set the default openssl directory]),
	AC_DEFINE_UNQUOTED(OPENSSLDIR, "$withval")
)

AC_ARG_WITH([enginesdir],
	AS_HELP_STRING([--with-enginesdir],
		       [Set the default engines directory (use with openssldir)]),
	AC_DEFINE_UNQUOTED(ENGINESDIR, "$withval")
)

AC_ARG_ENABLE([extratests],
	AS_HELP_STRING([--enable-extratests], [Enable extra tests that may be unreliable on some platforms]))
AM_CONDITIONAL([ENABLE_EXTRATESTS], [test "x$enable_extratests" = xyes])

# Add CPU-specific alignment flags
old_cflags=$CFLAGS
CFLAGS="$CFLAGS -I$srcdir/include"
AC_MSG_CHECKING([if BSWAP4 builds without __STRICT_ALIGNMENT])
AC_TRY_COMPILE([#include "$srcdir/crypto/modes/modes_lcl.h"],
	       [int a = 0; BSWAP4(a);],
	       AC_MSG_RESULT([yes])
	       BSWAP4=yes,
	       AC_MSG_RESULT([no])
	       BSWAP4=no)
CFLAGS="$old_cflags"

case $host_cpu in
	*sparc*)
		CPPFLAGS="$CPPFLAGS -D__STRICT_ALIGNMENT"
		;;
	*arm*)
		AS_IF([test "x$BSWAP4" = "xyes"],,
		    CPPFLAGS="$CPPFLAGS -D__STRICT_ALIGNMENT")
		;;
esac

AC_ARG_ENABLE([asm],
	AS_HELP_STRING([--disable-asm], [Disable assembly]))
AM_CONDITIONAL([OPENSSL_NO_ASM], [test "x$enable_asm" = "xno"])

# Conditionally enable assembly by default
AM_CONDITIONAL([HOST_ASM_ELF_X86_64],
    [test "x$HOST_ABI" = "xelf" -a "$host_cpu" = "x86_64" -a "x$enable_asm" != "xno"])
AM_CONDITIONAL([HOST_ASM_MACOSX_X86_64],
    [test "x$HOST_ABI" = "xmacosx" -a "$host_cpu" = "x86_64" -a "x$enable_asm" != "xno"])

AC_CONFIG_FILES([
	Makefile
	include/Makefile
	include/openssl/Makefile
	crypto/Makefile
	ssl/Makefile
	tls/Makefile
	tests/Makefile
	apps/Makefile
	man/Makefile
	libcrypto.pc
	libssl.pc
	libtls.pc
	openssl.pc
])

AC_OUTPUT
