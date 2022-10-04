# libtar/Makefile.  Generated from Makefile.in by configure.

### Path settings
srcdir		= /home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/packages/libtar/libtar
top_srcdir	= /home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/packages/libtar
top_builddir    = ..
prefix		= /home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/usr
exec_prefix	= ${prefix}
bindir		= ${exec_prefix}/bin
libdir		= ${exec_prefix}/lib
includedir	= ${prefix}/include
datarootdir	= ${prefix}/share

PACKAGE_NAME	= libtar
PACKAGE_VERSION	= 1.2.20



SHELL           = /bin/bash

### Installation programs and flags
INSTALL		= /usr/bin/install -p
INSTALL_PROGRAM	= ${INSTALL} -s
INSTALL_DATA	= ${INSTALL} -m 644
LN_S		= ln -s
MKDIR		= mkdir -p -m 755

### Compiler and link options
CC		=  cc
LIBTOOL         = $(SHELL) $(top_builddir)/libtool
CPPFLAGS	= -I.. \
		  -I../lib \
		  -I../listhash \
		  -I${top_srcdir}/lib \
		  -I${top_srcdir}/compat \
		  -isystem/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/usr/include/x86_64-linux-gnu -isystem/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/usr/include
CFLAGS		= -isystem/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/usr/include/x86_64-linux-gnu -isystem/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/usr/include -std=gnu99 -pipe -O2 -g -ffunction-sections -DALCHEMY_BUILD  -m64 -fPIC -msse -msse2 -mssse3 
LDFLAGS		=  -O2  -m64  -L/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/lib/x86_64-linux-gnu -L/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/lib -L/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/usr/lib/x86_64-linux-gnu -L/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/usr/lib -Wl,-rpath-link=/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/lib/x86_64-linux-gnu -Wl,-rpath-link=/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/lib -Wl,-rpath-link=/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/usr/lib/x86_64-linux-gnu -Wl,-rpath-link=/home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/out/arsdk-native/staging/usr/lib  -pthread -lrt
LIBS		=  -ltar
LIBOBJS		=  ${LIBOBJDIR}strlcpy$U.o ${LIBOBJDIR}basename$U.o ${LIBOBJDIR}dirname$U.o ${LIBOBJDIR}strmode$U.o
RANLIB		= ranlib

VPATH		= /home/jose/catkin_ws/build/parrot_arsdk/arsdk/src/ARSDKBuildUtils/packages/libtar/libtar


### Makefile rules - no user-servicable parts below

LIBTAR_OBJS	= libtar.lo
LIBTAR_HDRS	= ../config.h \
		  ${top_srcdir}/compat/compat.h \
		  ${top_srcdir}/lib/libtar.h \
		  ../listhash/libtar_listhash.h
LIBTAR_LIBS	= $(top_builddir)/lib
ALL		= libtar


all: ${ALL}

.PHONY: clean distclean install

libtar: ${LIBTAR_OBJS} ${LIBTAR_LIBS} ${LIBTAR_HDRS}
	$(LIBTOOL) --mode=link ${CC} ${CFLAGS} ${LDFLAGS} -o libtar $(LIBTAR_OBJS) -L${LIBTAR_LIBS} ${LIBS}

${LIBTAR_OBJS}: ${LIBTAR_HDRS}

%.lo: %.c
	$(LIBTOOL) --mode=compile ${CC} ${CFLAGS} ${CPPFLAGS} -c -o $@ $<

clean:
	rm -rf *~ *.o ${ALL} $(LIBTAR_OBJS) core .libs

distclean: clean
	rm -f Makefile

install: ${ALL}
	${MKDIR} ${DESTDIR}${bindir}
	$(LIBTOOL) --mode=install ${INSTALL_PROGRAM} libtar ${DESTDIR}${bindir}

