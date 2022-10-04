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

#include <stdlib.h>
#include <string.h>

static int puf_get_type_version(uint32_t p_lang)
{
	uint8_t c1, c2;
	if (p_lang == 0)
		return 0;

	c1 = (p_lang >> 8);
	c2 = (p_lang >> 16);

	if (('0' <= c1) && (c1 <= '9') &&
	    ('0' <= c2) && (c2 <= '9'))
		return (c1-'0')*10 + (c2-'0');
	return 0;
}

static int puf_fill_version_from_header(plf_phdr *header,
					struct puf_version *version)
{
	if (!header || !version)
		return -EINVAL;

	version->major = header->p_ver;
	version->minor = header->p_edit;
	version->patch = header->p_ext;

	if (version->major == 0 &&
	    version->minor == 0 &&
	    version->patch == 0) {
		version->type = PUF_VERSION_TYPE_DEV;
		version->build = 0;
		return 0;
	}

	version->build = puf_get_type_version(header->p_lang);

	switch (header->p_lang & 0xff) {
	case 'A':
		version->type = PUF_VERSION_TYPE_ALPHA;
		break;
	case 'B':
		version->type = PUF_VERSION_TYPE_BETA;
		break;
	case 'R':
		version->type = PUF_VERSION_TYPE_RC;
		break;
	default:
		version->type = PUF_VERSION_TYPE_RELEASE;
		break;
	}
	return 0;
}

#if BUILD_LIBPLFNG

struct puf_plf *puf_plf_new(const char *path)
{
	int res;
	struct puf_plf *puf_plf;

	if (!path)
		return NULL;

	puf_plf = calloc(1, sizeof(*puf_plf));
	if (!puf_plf)
		return NULL;

	puf_plf->plf_file = fopen(path, "rb");
	if (!puf_plf->plf_file)
		goto error;

	puf_plf->plf = plfng_new_from_file(puf_plf->plf_file);
	if (!puf_plf->plf)
		goto error;

	res = plfng_get_header(puf_plf->plf, &puf_plf->header);
	if (res < 0)
		goto error;

	return puf_plf;

error:
	puf_plf_destroy(puf_plf);
	return NULL;
}

void puf_plf_destroy(struct puf_plf *puf_plf)
{
	if (!puf_plf)
		return;

	if (puf_plf->plf)
		plfng_destroy(puf_plf->plf);
	if (puf_plf->plf_file)
		fclose(puf_plf->plf_file);
	free(puf_plf);
}

int puf_plf_get_version(struct puf_plf *puf_plf, struct puf_version *version)
{
	if (!puf_plf || !version)
		return -EINVAL;

	return puf_fill_version_from_header(&puf_plf->header, version);
}

int puf_plf_get_app_id(struct puf_plf *puf_plf, uint32_t *app_id)
{
	if (!puf_plf || !app_id)
		return -EINVAL;

	*app_id = puf_plf->header.p_app;
	return 0;
}

int puf_plf_get_target_id(struct puf_plf *puf_plf, uint32_t *target_id)
{
	if (!puf_plf || !target_id)
		return -EINVAL;

	*target_id = puf_plf->header.p_targ;
	return 0;
}

int puf_plf_get_file_size(struct puf_plf *puf_plf, const char *fname)
{
	return -ENOSYS;
}

int puf_plf_extract_to_buf(struct puf_plf *puf_plf, const char *fname,
			   uint8_t *buf, size_t len)
{
	return -ENOSYS;
}

int puf_plf_extract_to_file(struct puf_plf *puf_plf,
			    const char *fname, const char *oname)
{
	int ret = 0, i, count, pos = -1;
	plf_unixhdr hdr;
	char buf[128];

	count = plfng_get_section_count(puf_plf->plf);
	if (count < 0) {
		ret = count;
		goto exit;
	}

	for (i = 0; i < count; i++) {
		/* probe section */
		ret = plfng_get_unixfile_path(puf_plf->plf, i, &hdr, buf,
					      sizeof(buf));
		if ((ret < 0) || !S_ISREG((mode_t)hdr.s_mode))
			/* probably not a U_UNIXFILE regular file */
			continue;

		if (strcmp(buf, fname) == 0) {
			/* we have a match */
			pos = i;
			break;
		}
	}

	if (pos < 0) {
		ret = -ENOENT;
		goto exit;
	}

	/* now do the extraction */
	ret = plfng_extract_unixfile(puf_plf->plf, pos, oname);
	if (ret < 0)
		goto exit;

exit:
	return ret;
}

#else

struct puf_plf *puf_plf_new(const char *path)
{
	FILE *plf_file;
	struct puf_plf *puf_plf = NULL;

	if (!path)
		return NULL;

	plf_file = fopen(path, "rb");
	if (!plf_file)
		return NULL;

	puf_plf = calloc(1, sizeof(*puf_plf));
	if (!puf_plf)
		goto exit;

	if (fread(&puf_plf->header, sizeof(puf_plf->header), 1, plf_file) != 1)
		goto exit;

exit:
	fclose(plf_file);
	return puf_plf;
}

void puf_plf_destroy(struct puf_plf *puf_plf)
{
	if (!puf_plf)
		return;
	free(puf_plf);
}

int puf_plf_get_version(struct puf_plf *puf_plf, struct puf_version *version)
{
	if (!puf_plf || !version)
		return -EINVAL;

	return puf_fill_version_from_header(&puf_plf->header, version);
}

int puf_plf_get_app_id(struct puf_plf *puf_plf, uint32_t *app_id)
{
	if (!puf_plf || !app_id)
		return -EINVAL;

	*app_id = puf_plf->header.p_app;
	return 0;
}

int puf_plf_get_target_id(struct puf_plf *puf_plf, uint32_t *target_id)
{
	if (!puf_plf || !target_id)
		return -EINVAL;

	*target_id = puf_plf->header.p_targ;
	return 0;
}

int puf_plf_get_file_size(struct puf_plf *puf_plf, const char *fname)
{
	return -ENOSYS;
}

int puf_plf_extract_to_buf(struct puf_plf *puf_plf, const char *fname,
			   uint8_t *buf, size_t len)
{
	return -ENOSYS;
}

int puf_plf_extract_to_file(struct puf_plf *puf_plf,
			    const char *fname, const char *oname)
{
	return -ENOSYS;
}

#endif

int puf_plf_get_version_from_buffer(uint8_t *buffer, size_t len,
				    struct puf_version *version)
{
	plf_phdr *header;

	if (!buffer || !version || len < sizeof(plf_phdr))
		return -EINVAL;

	header = (plf_phdr *)buffer;
	if (header->p_magic != PLF_HEADER_MAGIC)
		return -EINVAL;
	return puf_fill_version_from_header(header, version);
}
