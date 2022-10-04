/**
* Copyright (c) 2017 Parrot Drones SAS
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*   * Neither the name of the Parrot Drones SAS Company nor the
*     names of its contributors may be used to endorse or promote products
*     derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _LIBPUF_PRIVATE_H
#define _LIBPUF_PRIVATE_H

#include <libpuf.h>

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>

#include "libpuf_plf.h"
#include "libpuf_tar.h"


#define ULOG_TAG libpuf

#if defined(BUILD_LIBULOG)

#include <ulog.h>
#define PUF_LOGD(_fmt, ...) ULOGD(_fmt, ##__VA_ARGS__)
#define PUF_LOGI(_fmt, ...) ULOGI(_fmt, ##__VA_ARGS__)
#define PUF_LOGW(_fmt, ...) ULOGW(_fmt, ##__VA_ARGS__)
#define PUF_LOGE(_fmt, ...) ULOGE(_fmt, ##__VA_ARGS__)

#else /* !BUILD_LIBULOG */

#define PUF_STRINGIFY(x) #x
#define PUF_LOG(_fmt, ...) fprintf(stderr, PUF_STRINGIFY(ULOG_TAG) \
				   "\t"_fmt "\n", ##__VA_ARGS__)
#define PUF_LOGD(_fmt, ...) PUF_LOG("[D] " _fmt, ##__VA_ARGS__)
#define PUF_LOGI(_fmt, ...) PUF_LOG("[I] " _fmt, ##__VA_ARGS__)
#define PUF_LOGW(_fmt, ...) PUF_LOG("[W] " _fmt, ##__VA_ARGS__)
#define PUF_LOGE(_fmt, ...) PUF_LOG("[E] " _fmt, ##__VA_ARGS__)

#endif /* !BUILD_LIBULOG */


enum puf_file_type {
	FILE_TYPE_PLF,
	FILE_TYPE_TAR,
	FILE_TYPE_GZIP,
	FILE_TYPE_UNKNOWN,
};

struct puf {
	struct puf_plf *plf;
	struct puf_tar *tar;
};

#ifndef EXPORT_SYMBOL
#define EXPORT_SYMBOL __attribute__((visibility("default")))
#endif
#endif /* _LIBPUF_PRIVATE_H */
