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
 * @file ARSAL_MD5.c
 * @brief MD5 manager allow compute and check md5.
 * @date 02/06/2014
 * @author david.flattin.ext@parrot.com
 */
 
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "libARSAL/ARSAL_Error.h"
#include "libARSAL/ARSAL_Print.h"
#include "libARSAL/ARSAL_MD5_Manager.h"  

#define ARUTILS_MD5_TAG "Md5"

ARSAL_MD5_Manager_t* ARSAL_MD5_Manager_New(eARSAL_ERROR *error)
{
    ARSAL_MD5_Manager_t* newManager = NULL;
    eARSAL_ERROR result = ARSAL_OK;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_MD5_TAG, "%s", "");
    
    newManager = calloc(1, sizeof(ARSAL_MD5_Manager_t));
    
    if (newManager == NULL)
    {
        result = ARSAL_ERROR_ALLOC;
    }
    
    *error = result;
    return newManager;
}

void ARSAL_MD5_Manager_Delete(ARSAL_MD5_Manager_t **managerAddr)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_MD5_TAG, "%s", "");
    
    if (managerAddr != NULL)
    {
        ARSAL_MD5_Manager_t* manager = *managerAddr;
        
        if (manager != NULL)
        {
            free(manager);
        }
        
        *managerAddr = NULL;
    }
}

eARSAL_ERROR ARSAL_MD5_Manager_Check(ARSAL_MD5_Manager_t *manager, const char *filePath, const char *md5Txt)
{
    eARSAL_ERROR result = ARSAL_OK;
    
    if ((manager == NULL) || (manager->md5Check == NULL))
    {
        result = ARSAL_ERROR_BAD_PARAMETER;
    }
    
    if (result == ARSAL_OK)
    {
        result = manager->md5Check(manager->md5Object, filePath, md5Txt);
    }
    
    return result;
}

eARSAL_ERROR ARSAL_MD5_Manager_Compute(ARSAL_MD5_Manager_t *manager, const char *filePath, uint8_t *md5, int md5Len)
{
    eARSAL_ERROR result = ARSAL_OK;
    
    if ((manager == NULL) || (manager->md5Compute == NULL))
    {
        result = ARSAL_ERROR_BAD_PARAMETER;
    }
    
    if (result == ARSAL_OK)
    {
        result = manager->md5Compute(manager->md5Object, filePath, md5, md5Len);
    }
    
    return result;
}

