AC_DEFUN([CHECK_LIBC_COMPAT], [
# Check for general libc functions
AC_CHECK_FUNCS([asprintf inet_pton memmem poll reallocarray])
AC_CHECK_FUNCS([strlcat strlcpy strndup strnlen strsep strtonum])
AM_CONDITIONAL([HAVE_ASPRINTF], [test "x$ac_cv_func_asprintf" = xyes])
AM_CONDITIONAL([HAVE_INET_PTON], [test "x$ac_cv_func_inet_pton" = xyes])
AM_CONDITIONAL([HAVE_MEMMEM], [test "x$ac_cv_func_memmem" = xyes])
AM_CONDITIONAL([HAVE_POLL], [test "x$ac_cv_func_poll" = xyes])
AM_CONDITIONAL([HAVE_REALLOCARRAY], [test "x$ac_cv_func_reallocarray" = xyes])
AM_CONDITIONAL([HAVE_STRLCAT], [test "x$ac_cv_func_strlcat" = xyes])
AM_CONDITIONAL([HAVE_STRLCPY], [test "x$ac_cv_func_strlcpy" = xyes])
AM_CONDITIONAL([HAVE_STRNDUP], [test "x$ac_cv_func_strndup" = xyes])
AM_CONDITIONAL([HAVE_STRNLEN], [test "x$ac_cv_func_strnlen" = xyes])
AM_CONDITIONAL([HAVE_STRSEP], [test "x$ac_cv_func_strsep" = xyes])
AM_CONDITIONAL([HAVE_STRTONUM], [test "x$ac_cv_func_strtonum" = xyes])
])

AC_DEFUN([CHECK_LIBC_CRYPTO_COMPAT], [
# Check crypto-related libc functions
AC_CHECK_FUNCS([arc4random_buf explicit_bzero getauxval getentropy])
AC_CHECK_FUNCS([timingsafe_bcmp timingsafe_memcmp])
AM_CONDITIONAL([HAVE_ARC4RANDOM_BUF], [test "x$ac_cv_func_arc4random_buf" = xyes])
AM_CONDITIONAL([HAVE_EXPLICIT_BZERO], [test "x$ac_cv_func_explicit_bzero" = xyes])
AM_CONDITIONAL([HAVE_GETENTROPY], [test "x$ac_cv_func_getentropy" = xyes])
AM_CONDITIONAL([HAVE_TIMINGSAFE_BCMP], [test "x$ac_cv_func_timingsafe_bcmp" = xyes])
AM_CONDITIONAL([HAVE_TIMINGSAFE_MEMCMP], [test "x$ac_cv_func_timingsafe_memcmp" = xyes])

# Check for getentropy fallback dependencies
AC_CHECK_FUNC([getauxval])
AC_CHECK_FUNC([clock_gettime],, [AC_SEARCH_LIBS([clock_gettime],[rt posix4])])
AC_CHECK_FUNC([dl_iterate_phdr],, [AC_SEARCH_LIBS([dl_iterate_phdr],[dl])])
])

AC_DEFUN([CHECK_VA_COPY], [
AC_CACHE_CHECK([whether va_copy exists], ac_cv_have_va_copy, [
	AC_LINK_IFELSE([AC_LANG_PROGRAM([[
#include <stdarg.h>
va_list x,y;
		]], [[ va_copy(x,y); ]])],
	[ ac_cv_have_va_copy="yes" ],
	[ ac_cv_have_va_copy="no"
	])
])
if test "x$ac_cv_have_va_copy" = "xyes" ; then
	AC_DEFINE([HAVE_VA_COPY], [1], [Define if va_copy exists])
fi

AC_CACHE_CHECK([whether __va_copy exists], ac_cv_have___va_copy, [
	AC_LINK_IFELSE([AC_LANG_PROGRAM([[
#include <stdarg.h>
va_list x,y;
		]], [[ __va_copy(x,y); ]])],
	[ ac_cv_have___va_copy="yes" ], [ ac_cv_have___va_copy="no"
	])
])
if test "x$ac_cv_have___va_copy" = "xyes" ; then
	AC_DEFINE([HAVE___VA_COPY], [1], [Define if __va_copy exists])
fi
])
