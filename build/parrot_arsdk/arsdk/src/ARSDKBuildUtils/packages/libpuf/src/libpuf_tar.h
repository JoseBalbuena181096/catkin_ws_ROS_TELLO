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

#ifndef _LIBPUF_TAR_H
#define _LIBPUF_TAR_H

#include <stdint.h>
#include <libtar.h>

struct puf_tar {
	char *path;
	tartype_t *tartype;

	plf_phdr header;
};

struct puf_tar *puf_tar_new(const char *path, int is_gzip);
void puf_tar_destroy(struct puf_tar *fw_tar);
int puf_tar_get_version(struct puf_tar *fw_tar, struct puf_version *version);
int puf_tar_get_app_id(struct puf_tar *fw_tar, uint32_t *app_id);
int puf_tar_get_target_id(struct puf_tar *fw_tar, uint32_t *target_id);
int puf_tar_get_file_size(struct puf_tar *puf_tar, const char *fname);
int puf_tar_extract_to_buf(struct puf_tar *puf_tar, const char *fname,
			   uint8_t *buf, size_t len);
int puf_tar_extract_to_file(struct puf_tar *puf_tar,
			    const char *fname, const char *oname);

#endif /* _LIBPUF_TAR_H */
