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
 * @file ARSAL_MD5.m
 * @brief MD5 manager allow compute and check md5.
 * @date 02/06/2014
 * @author david.flattin.ext@parrot.com
 */

#include <stdio.h>

#include "md5.h"
#include "libARSAL/ARSAL_Error.h"
#include "libARSAL/ARSAL_Print.h"
#include "libARSAL/ARSAL_MD5_Manager.h"
#include "ARSAL_MD5.h"
//#include "ARSAL_Singleton.h"

#define ARUTILS_MD5_TAG         "Md5"

eARSAL_ERROR ARSAL_MD5_Manager_Init(ARSAL_MD5_Manager_t *manager)
{
    eARSAL_ERROR result = ARSAL_OK;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_MD5_TAG, "%s", "");
    
    if (manager == NULL)
    {
        result = ARSAL_ERROR_BAD_PARAMETER;
    }
    
    if (result == ARSAL_OK)
    {
        manager->md5Check = ARSAL_MD5_Check;
        manager->md5Compute = ARSAL_MD5_Compute;
    }
    
    return result;
}

void ARSAL_MD5_Manager_Close(ARSAL_MD5_Manager_t *manager)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_MD5_TAG, "%s", "");
}

eARSAL_ERROR ARSAL_MD5_Check(void *md5Object, const char *filePath, const char *md5Txt)
{
    eARSAL_ERROR result = ARSAL_OK;
    uint8_t md5[MD5_DIGEST_LENGTH];
    char md5Src[(MD5_DIGEST_LENGTH * 2) + 1];
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_MD5_TAG, "%s", "");
    
    if ((filePath == NULL) || (md5Txt == NULL))
    {
        result = ARSAL_ERROR_BAD_PARAMETER;
    }
    
    if (result == ARSAL_OK)
    {
        result = ARSAL_MD5_Compute(md5Object, filePath, md5, sizeof(md5));
    }
    
    if (result == ARSAL_OK)
    {
        result = ARSAL_MD5_GetMd5AsTxt(md5, sizeof(md5), md5Src, sizeof(md5Src));
    }
    
    if (result == ARSAL_OK)
    {
        if (strcmp(md5Txt, md5Src) != 0)
        {
            result = ARSAL_ERROR_MD5;
        }
    }
    
    return result;
}

eARSAL_ERROR ARSAL_MD5_Compute(void *md5Object, const char *filePath, uint8_t *md5, int md5Len)
{
    eARSAL_ERROR result = ARSAL_OK;
    uint8_t block[1024];
    MD5_CTX ctx;
    FILE *file = NULL;
    size_t count;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_MD5_TAG, "%s", "");
    
    if ((filePath == NULL) || (md5 == NULL) || (md5Len < MD5_DIGEST_LENGTH))
    {
        result = ARSAL_ERROR_BAD_PARAMETER;
    }
    
    if (result == ARSAL_OK)
    {
        AR_MD5_Init(&ctx);
        file = fopen(filePath, "rb");
        if (file == NULL)
        {
            result = ARSAL_ERROR_FILE;
        }
    }
    
    if (result == ARSAL_OK)
    {
        while ((count = fread(block, sizeof(uint8_t), sizeof(block), file)) > 0)
        {
            AR_MD5_Update(&ctx, block, count);
        }
        
        AR_MD5_Final(md5, &ctx);
    }
    
    if (file != NULL)
    {
        fclose(file);
    }
    
    return result;
}

eARSAL_ERROR ARSAL_MD5_GetMd5AsTxt(const uint8_t *md5, int md5Len, char *md5Txt, int md5TxtLen)
{
    eARSAL_ERROR result = ARSAL_OK;
    int i;

    if ((md5 == NULL) || (md5Len < MD5_DIGEST_LENGTH) || (md5Txt == NULL) || (md5TxtLen < ((MD5_DIGEST_LENGTH *2) + 1)))
    {
        result = ARSAL_ERROR_BAD_PARAMETER;
    }
    else
    {
        for (i= 0; i<MD5_DIGEST_LENGTH; i++)
        {
            sprintf(&md5Txt[i * 2], "%02x", md5[i]);
        }

        md5Txt[MD5_DIGEST_LENGTH * 2] = '\0';
    }

    return result;
}
