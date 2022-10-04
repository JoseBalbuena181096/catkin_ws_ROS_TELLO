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

#include "libpuf_private.h"

#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <zlib.h>


#define TAR_HEADER_FILE "header"

static intptr_t gzopen_frontend(char *pathname, int oflags, int mode)
{
	char *gzoflags;
	gzFile gzf;
	int fd;

	switch (oflags & O_ACCMODE) {
	case O_WRONLY:
		gzoflags = "wb";
		break;
	case O_RDONLY:
		gzoflags = "rb";
		break;
	default:
	case O_RDWR:
		errno = EINVAL;
		return -1;
	}

	fd = open(pathname, oflags, mode);
	if (fd == -1)
		return -1;

	if ((oflags & O_CREAT) && fchmod(fd, mode)) {
		close(fd);
		return -1;
	}

	gzf = gzdopen(fd, gzoflags);
	if (!gzf) {
		errno = ENOMEM;
		return -1;
	}

	return (intptr_t)gzf;
}

/* Can't be const because libtar expects a non-const tartype_t, but will not
   modify it */
static tartype_t gztype = { (openfunc_t) gzopen_frontend,
			    (closefunc_t) gzclose,
			    (readfunc_t) gzread,
			    (writefunc_t) gzwrite
};

struct puf_tar *puf_tar_new(const char *path, int is_gzip)
{
	int res;
	struct puf_tar *puf_tar = NULL;
	if (!path)
		return NULL;

	puf_tar = calloc(1, sizeof(*puf_tar));
	if (!puf_tar)
		return NULL;

	puf_tar->path = strdup(path);
	if (!puf_tar->path)
		goto error;
	puf_tar->tartype = is_gzip ? &gztype : NULL;

	res = puf_tar_extract_to_buf(puf_tar, TAR_HEADER_FILE,
				     (uint8_t *)&puf_tar->header,
				     sizeof(puf_tar->header));
	if (res < 0)
		goto error;

	return puf_tar;

error:
	puf_tar_destroy(puf_tar);
	return NULL;
}

void puf_tar_destroy(struct puf_tar *puf_tar)
{
	if (!puf_tar)
		return;
	if (puf_tar->path)
		free(puf_tar->path);
	free(puf_tar);
}

int puf_tar_get_version(struct puf_tar *puf_tar, struct puf_version *version)
{
	if (!puf_tar || !version)
		return -EINVAL;

	return puf_plf_get_version_from_buffer((uint8_t *)&puf_tar->header,
					       sizeof(puf_tar->header),
					       version);
}

int puf_tar_get_app_id(struct puf_tar *puf_tar, uint32_t *app_id)
{
	if (!puf_tar || !app_id)
		return -EINVAL;

	*app_id = puf_tar->header.p_app;
	return 0;
}

int puf_tar_get_target_id(struct puf_tar *puf_tar, uint32_t *target_id)
{
	if (!puf_tar || !target_id)
		return -EINVAL;

	*target_id = puf_tar->header.p_targ;
	return 0;
}

int puf_tar_get_file_size(struct puf_tar *puf_tar, const char *fname)
{
	int ret = -ENOENT;
	TAR *t;

	if (!puf_tar || !fname)
		return -EINVAL;

	if (tar_open(&t, puf_tar->path, puf_tar->tartype, O_RDONLY, 0, 0) == -1)
		return -EIO;

	while (th_read(t) == 0) {
		char *pathname = th_get_pathname(t);
		/* strip the leading ./ in pathname if present */
		if (strlen(pathname) > 2 &&
		    pathname[0] == '.' &&
		    pathname[1] == '/')
			pathname = &pathname[2];
		if (strcmp(pathname, fname) == 0) {
			ret = th_get_size(t);
			break;
		}
		if (TH_ISREG(t) && tar_skip_regfile(t) != 0) {
			ret = -EIO;
			break;
		}
	}

	tar_close(t);

	return ret;
}

int puf_tar_extract_to_buf(struct puf_tar *puf_tar, const char *fname,
			   uint8_t *buf, size_t len)
{
	int ret = -ENOENT;
	TAR *t;

	if (!puf_tar || !fname || !buf)
		return -EINVAL;

	if (tar_open(&t, puf_tar->path, puf_tar->tartype, O_RDONLY, 0, 0) == -1)
		return -EIO;

	while (th_read(t) == 0) {
		char *pathname = th_get_pathname(t);
		/* strip the leading ./ in pathname if present */
		if (strlen(pathname) > 2 &&
		    pathname[0] == '.' &&
		    pathname[1] == '/')
			pathname = &pathname[2];
		if (strcmp(pathname, fname) == 0) {
			size_t copied_size = 0;
			size_t tar_size = th_get_size(t);
			if (tar_size > len) {
				ret = -EINVAL;
				break;
			}
			while (copied_size < tar_size) {
				size_t remaining = tar_size - copied_size;
				uint8_t rbuf[T_BLOCKSIZE];
				tar_block_read(t, rbuf);
				if (remaining > T_BLOCKSIZE) {
					memcpy(&buf[copied_size],
					       rbuf,
					       T_BLOCKSIZE);
					copied_size += T_BLOCKSIZE;
				} else {
					memcpy(&buf[copied_size],
					       rbuf,
					       remaining);
					copied_size += remaining;
				}
			}
			ret = tar_size;
			break;
		}

		if (TH_ISREG(t) && tar_skip_regfile(t) != 0) {
			ret = -EIO;
			break;
		}
	}

	tar_close(t);

	return ret;
}

int puf_tar_extract_to_file(struct puf_tar *puf_tar,
			    const char *fname, const char *oname)
{
	int ret = -ENOENT;
	TAR *t;

	if (!puf_tar || !fname || !oname)
		return -EINVAL;

	if (tar_open(&t, puf_tar->path, puf_tar->tartype, O_RDONLY, 0, 0) == -1)
		return -EIO;

	while (th_read(t) == 0) {
		char *pathname = th_get_pathname(t);
		/* strip the leading ./ in pathname if present */
		if (strlen(pathname) > 2 &&
		    pathname[0] == '.' &&
		    pathname[1] == '/')
			pathname = &pathname[2];
		if (strcmp(pathname, fname) == 0) {
			ret = tar_extract_file(t, (char *)oname);
			break;
		}

		if (TH_ISREG(t) && tar_skip_regfile(t) != 0) {
			ret = -EIO;
			break;
		}
	}

	tar_close(t);

	return ret;
}
