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

#ifndef _LIBPUF_PLF_H
#define _LIBPUF_PLF_H

#include <stdint.h>

#if BUILD_LIBPLFNG

#include <libplfng.h>

struct puf_plf {
	FILE *plf_file;
	struct plfng *plf;

	plf_phdr header;
};

#else
/* codecheck_ignore_file[NEW_TYPEDEFS] */

/* Minmimal plf header informations */
#define PLF_HEADER_MAGIC        0x21464c50    /* PLF magic number */

typedef uint32_t Plf_Word;                    /* Unsigned 32 bits integer */
typedef uint16_t Plf_Half;                    /* Unsigned 16 bits integer */
typedef uint32_t Plf_Add;                     /* 32 bits address */

/* PLF file header */
typedef struct __attribute__ ((__packed__)) {
	Plf_Word    p_magic;                  /* PLF magic number */
	Plf_Word    p_plfversion;             /* PLF format version */
	Plf_Word    p_phdrsize;               /* File header size */
	Plf_Word    p_shdrsize;               /* Section header size */
	Plf_Word    p_type;                   /* File type */
	Plf_Add     p_entry;                  /* Executable entry point */
	Plf_Word    p_targ;                   /* Target platform */
	Plf_Word    p_app;                    /* Target application */
	Plf_Word    p_crc;                    /* Headers checksum */
	Plf_Word    p_ver;                    /* Version */
	Plf_Word    p_edit;                   /* Edition */
	Plf_Word    p_ext;                    /* Extension */
	Plf_Word    p_lang;                   /* Language zone */
	Plf_Word    p_size;                   /* File size in bytes */
} plf_phdr;

struct puf_plf {
	plf_phdr header;
};

#endif

struct puf_plf *puf_plf_new(const char *path);
void puf_plf_destroy(struct puf_plf *fw_plf);
int puf_plf_get_version(struct puf_plf *fw_plf, struct puf_version *version);
int puf_plf_get_app_id(struct puf_plf *fw_plf, uint32_t *app_id);
int puf_plf_get_target_id(struct puf_plf *fw_plf, uint32_t *target_id);
int puf_plf_get_file_size(struct puf_plf *puf_plf, const char *fname);
int puf_plf_extract_to_buf(struct puf_plf *puf_plf, const char *fname,
			   uint8_t *buf, size_t len);
int puf_plf_extract_to_file(struct puf_plf *puf_plf,
			    const char *fname, const char *oname);
int puf_plf_get_version_from_buffer(uint8_t *buffer, size_t len,
				    struct puf_version *version);

#endif /* _LIBPUF_PLF_H */
