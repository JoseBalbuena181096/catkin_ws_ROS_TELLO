/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/
/**
 * @file ARUPDATER_Utils.h
 * @brief libARUpdater Utils c file.
 * @date 23/05/2014
 * @author djavan.bertrand@parrot.com
 **/

#ifndef WIN32
#include <sys/types.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <stdint.h>
#include <syslog.h>
#include <ctype.h>
#include <errno.h>
#include <sys/stat.h>

#include <libpuf.h>

#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Endianness.h>

#include "ARUPDATER_Utils.h"
#include "ARUPDATER_Manager.h"

#define ARUPDATER_UTILS_TAG                   "ARUPDATER_Utils"

static eARUPDATER_ERROR ARUPDATER_Utils_PufVersionToARUpdaterVersion(const struct puf_version *pv, ARUPDATER_PlfVersion *v)
{
	static const eARUPDATER_PLF_TYPE puf_to_arup[] = {
		[PUF_VERSION_TYPE_DEV] = ARUPDATER_PLF_TYPE_PROD,
		[PUF_VERSION_TYPE_ALPHA] = ARUPDATER_PLF_TYPE_ALPHA,
		[PUF_VERSION_TYPE_BETA] = ARUPDATER_PLF_TYPE_BETA,
		[PUF_VERSION_TYPE_RC] = ARUPDATER_PLF_TYPE_RC,
		[PUF_VERSION_TYPE_RELEASE] = ARUPDATER_PLF_TYPE_PROD,
	};

	if (!v || !pv)
		return ARUPDATER_ERROR_BAD_PARAMETER;

	v->ver = pv->major;
	v->edit = pv->minor;
	v->ext = pv->patch;
	v->patch = pv->build;
	if (pv->type <= PUF_VERSION_TYPE_RELEASE)
		v->type = puf_to_arup[pv->type];
	else
		v->type = ARUPDATER_PLF_TYPE_ALPHA;

	return ARUPDATER_OK;
}

static eARUPDATER_ERROR ARUPDATER_Utils_ARUpdaterVersionToPufVersion(const ARUPDATER_PlfVersion *v,struct puf_version *pv)
{
	static const enum puf_version_type arup_to_puf[] = {
		[ARUPDATER_PLF_TYPE_ALPHA] = PUF_VERSION_TYPE_ALPHA,
		[ARUPDATER_PLF_TYPE_BETA] = PUF_VERSION_TYPE_BETA,
		[ARUPDATER_PLF_TYPE_RC] = PUF_VERSION_TYPE_RC,
		[ARUPDATER_PLF_TYPE_PROD] = PUF_VERSION_TYPE_RELEASE,
	};

	if (!v || !pv)
		return ARUPDATER_ERROR_BAD_PARAMETER;

	pv->major = v->ver;
	pv->minor = v->edit;
	pv->patch = v->ext;
	pv->build = v->patch;
	if (v->type <= ARUPDATER_PLF_TYPE_PROD)
		pv->type = arup_to_puf[v->type];
	else
		pv->type = PUF_VERSION_TYPE_ALPHA;

	if (v->ver == 0 && v->edit == 0 && v->ext == 0)
		pv->type = PUF_VERSION_TYPE_DEV;

	return ARUPDATER_OK;
}

eARUPDATER_ERROR ARUPDATER_Utils_PlfVersionFromString(const char *str, ARUPDATER_PlfVersion *v)
{
	struct puf_version pv;
	int ret;

	if (!str || !v)
		return ARUPDATER_ERROR_BAD_PARAMETER;

	ret = puf_version_fromstring(str, &pv);
	if (ret < 0)
		return ARUPDATER_ERROR_BAD_PARAMETER;

	return ARUPDATER_Utils_PufVersionToARUpdaterVersion(&pv, v);
}

eARUPDATER_ERROR ARUPDATER_Utils_PlfVersionToString(const ARUPDATER_PlfVersion *v,
		char *buf, size_t size)
{
	struct puf_version pv;
	eARUPDATER_ERROR ret;

	if (!v || !buf)
		return ARUPDATER_ERROR_BAD_PARAMETER;

	ret = ARUPDATER_Utils_ARUpdaterVersionToPufVersion(v, &pv);
	if (ret != ARUPDATER_OK)
		return ret;

	if (puf_version_tostring(&pv, buf, size) != 0)
		return ARUPDATER_ERROR;
	return ARUPDATER_OK;
}


eARUPDATER_ERROR ARUPDATER_Utils_ReadPlfVersion(const char *plfFilePath, ARUPDATER_PlfVersion *v)
{
	struct puf *puf;
	struct puf_version pv;
	eARUPDATER_ERROR ret = ARUPDATER_ERROR;

	if (!plfFilePath || !v)
		return ARUPDATER_ERROR_BAD_PARAMETER;

	puf = puf_new(plfFilePath);
	if (!puf)
		return ARUPDATER_ERROR;

	if (puf_get_version(puf, &pv) != 0)
		goto exit;

	ret = ARUPDATER_Utils_PufVersionToARUpdaterVersion(&pv, v);

exit:
	puf_destroy(puf);
	return ret;
}

int ARUPDATER_Utils_PlfVersionCompare(const ARUPDATER_PlfVersion *v1, const ARUPDATER_PlfVersion *v2)
{
	struct puf_version pv1, pv2;
	if (!v1 || !v2)
		return 0;

	if (ARUPDATER_Utils_ARUpdaterVersionToPufVersion(v1, &pv1) != ARUPDATER_OK)
		return 0;
	if (ARUPDATER_Utils_ARUpdaterVersionToPufVersion(v2, &pv2) != ARUPDATER_OK)
		return 0;

	return puf_compare_version(&pv1, &pv2);
}

eARUPDATER_ERROR ARUPDATER_Utils_GetPlfInFolder(const char *const plfFolder, char **plfFileName)
{
    eARUPDATER_ERROR error = ARUPDATER_OK;

    if ((plfFolder == NULL) || (plfFileName == NULL))
    {
        return ARUPDATER_ERROR_BAD_PARAMETER;
    }

    *plfFileName = NULL;
    if (ARUPDATER_OK == error)
    {
        DIR *dir = opendir(plfFolder);

        struct dirent *entry = NULL;
        int found = 0;
        if (dir)
        {
            while((found == 0) && (entry = readdir(dir)))
            {
                char *filename = entry->d_name;
                char *extension = strrchr(filename, '.');
                if ((extension != NULL) &&
                    ((strcmp(extension, ARUPDATER_MANAGER_PLF_EXTENSION) == 0) ||
                     (strcmp(extension, ARUPDATER_MANAGER_TAR_EXTENSION) == 0) ||
                     (strcmp(extension, ARUPDATER_MANAGER_GZ_EXTENSION) == 0)))
                {
                    *plfFileName = strdup(filename);
                    if (*plfFileName == NULL)
                    {
                        error = ARUPDATER_ERROR_ALLOC;
                    }
                    found = 1;
                }
            }
            closedir(dir);
        }

        if (found == 0)
        {
            error = ARUPDATER_ERROR_PLF_FILE_NOT_FOUND;
        }
    }

    return error;
}

eARUPDATER_ERROR ARUPDATER_Utils_ExtractUnixFileFromPlf(const char *plfFileName, const char *outFolder, const char *unixFileName)
{
	eARUPDATER_ERROR err = ARUPDATER_OK;
	int ret;
	struct puf *puf;
	char *oname = NULL;

	if (!plfFileName || !outFolder || !unixFileName)
		return ARUPDATER_ERROR_BAD_PARAMETER;

	puf = puf_new(plfFileName);
	if (!puf)
		return ARUPDATER_ERROR;

	asprintf(&oname, "%s/%s", outFolder, unixFileName);
	if (!oname) {
		err = ARUPDATER_ERROR_ALLOC;
		goto exit;
	}

	ret = puf_extract_to_file(puf, unixFileName, oname);
	if (ret < 0)
		err = ARUPDATER_ERROR;

exit:
	free(oname);
	puf_destroy(puf);
	return err;
}
