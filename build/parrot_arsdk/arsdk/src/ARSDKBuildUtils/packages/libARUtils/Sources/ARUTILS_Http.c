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
 * @file ARUTILS_Http.c
 * @brief libARUtils Http c file.
 * @date 26/12/2013
 * @author david.flattin.ext@parrot.com
 **/

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <libARSAL/ARSAL_Sem.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_MD5_Manager.h>
#include <curl/curl.h>

#include "libARUtils/ARUTILS_Error.h"
#include "libARUtils/ARUTILS_Http.h"
#include "libARUtils/ARUTILS_FileSystem.h"
#include "ARUTILS_Http.h"

#define ARUTILS_HTTP_TAG              "Http"

#define ARUTILS_HTTP_LOW_SPEED_TIME   5
#define ARUTILS_HTTP_LOW_SPEED_LIMIT  1
#define ARUTILS_HTTP_TIMEOUT 3

#ifdef DEBUG
#define ARUTILS_HTTP_CURL_VERBOSE         1
#endif

// Doc
//http://curl.haxx.se/libcurl/c/libcurl-easy.html

/*****************************************
 *
 *             Public implementation:
 *
 *****************************************/

ARUTILS_Http_Connection_t * ARUTILS_Http_Connection_New(ARSAL_Sem_t *cancelSem, const char *server, int port, eARUTILS_HTTPS_PROTOCOL security, const char *username, const char* password, eARUTILS_ERROR *error)
{
    ARUTILS_Http_Connection_t *newConnection = NULL;
    eARUTILS_ERROR result = ARUTILS_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_HTTP_TAG, "%s, %d, %s", server ? server : "null", port, username ? username : "null");

    if (server == NULL)
    {
        result = ARUTILS_ERROR_BAD_PARAMETER;
    }

    newConnection = (ARUTILS_Http_Connection_t *)calloc(1, sizeof(ARUTILS_Http_Connection_t));

    if (newConnection == NULL)
    {
        result = ARUTILS_ERROR_ALLOC;
    }

    if (result == ARUTILS_OK)
    {
        newConnection->curlSocket = -1;
        newConnection->cancelSem = cancelSem;
    }

    if (result == ARUTILS_OK)
    {
        if (security == HTTPS_PROTOCOL_FALSE)
        {
            sprintf(newConnection->serverUrl, "http://%s:%d", server, port);
        }
        else
        {
            sprintf(newConnection->serverUrl, "https://%s:%d", server, port);
        }
    }

    if ((result == ARUTILS_OK) && (username != NULL))
    {
        strncpy(newConnection->username, username, ARUTILS_HTTP_MAX_USER_SIZE);
        newConnection->username[ARUTILS_HTTP_MAX_USER_SIZE - 1] = '\0';
    }

    if ((result == ARUTILS_OK) && (password != NULL))
    {
        strncpy(newConnection->password, password, ARUTILS_HTTP_MAX_USER_SIZE);
        newConnection->password[ARUTILS_HTTP_MAX_USER_SIZE - 1] = '\0';
    }

    if (result == ARUTILS_OK)
    {
        newConnection->curl = curl_easy_init();

        if (newConnection->curl == NULL)
        {
            result = ARUTILS_ERROR_CURL_ALLOC;
        }
    }

    if (result != ARUTILS_OK)
    {
        ARUTILS_Http_Connection_Delete(&newConnection);
    }

    *error = result;
    return newConnection;
}

void ARUTILS_Http_Connection_Delete(ARUTILS_Http_Connection_t **connectionAddr)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_HTTP_TAG, "%s", "");

    if (connectionAddr != NULL)
    {
        ARUTILS_Http_Connection_t *connection = *connectionAddr;

        if (connection != NULL)
        {
            if (connection->curl != NULL)
            {
                curl_easy_cleanup(connection->curl);
            }

            ARUTILS_Http_FreeCallbackData(&connection->cbdata);

            free(connection);
            *connectionAddr = NULL;
        }
    }
}

eARUTILS_ERROR ARUTILS_Http_Connection_Cancel(ARUTILS_Http_Connection_t *connection)
{
    eARUTILS_ERROR result = ARUTILS_OK;
    int resutlSys = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_HTTP_TAG, "%s", "");

    if ((connection == NULL) || (connection->cancelSem == NULL))
    {
        result = ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (result == ARUTILS_OK)
    {
        resutlSys = ARSAL_Sem_Post(connection->cancelSem);

        if (resutlSys != 0)
        {
            result = ARUTILS_ERROR_SYSTEM;
        }
    }

    if (result == ARUTILS_OK)
    {
        if (connection->curlSocket != -1)
        {
            shutdown(connection->curlSocket, SHUT_RDWR);
            connection->curlSocket = -1;
        }
    }

    return result;
}

eARUTILS_ERROR ARUTILS_Http_Get(ARUTILS_Http_Connection_t *connection, const char *namePath, const char *dstFile, ARUTILS_Http_ProgressCallback_t progressCallback, void* progressArg)
{
    return ARUTILS_Http_Get_Internal(connection, namePath, dstFile, NULL, NULL, progressCallback, progressArg);
}

eARUTILS_ERROR ARUTILS_Http_Get_WithBuffer(ARUTILS_Http_Connection_t *connection, const char *namePath, uint8_t **data, uint32_t *dataLen, ARUTILS_Http_ProgressCallback_t progressCallback, void* progressArg)
{
    return ARUTILS_Http_Get_Internal(connection, namePath, NULL, data, dataLen, progressCallback, progressArg);
}

eARUTILS_ERROR ARUTILS_Http_Get_Internal(ARUTILS_Http_Connection_t *connection, const char *namePath, const char *dstFile, uint8_t **data, uint32_t *dataLen, ARUTILS_Http_ProgressCallback_t progressCallback, void* progressArg)
{
    char fileUrl[ARUTILS_HTTP_MAX_URL_SIZE];
    eARUTILS_ERROR result = ARUTILS_OK;
    CURLcode code = CURLE_OK;
    long httpCode = 0L;
    double remoteSize = 0.f;
    int64_t localSize = 0;
    char *splitter = "";

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_HTTP_TAG, "%s, %s", namePath ? namePath : "null", dstFile ? dstFile : "null");

    if ((connection == NULL) || (connection->curl == NULL) || (namePath == NULL))
    {
        result =  ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (dstFile != NULL)
    {
        if ((data != NULL) || (dataLen != NULL))
        {
            result =  ARUTILS_ERROR_BAD_PARAMETER;
        }
    }
    else
    {
        if ((data == NULL) || (dataLen == NULL))
        {
            result =  ARUTILS_ERROR_BAD_PARAMETER;
        }
    }

    if (result == ARUTILS_OK)
    {
        result = ARUTILS_Http_IsCanceled(connection);
    }

    if (result == ARUTILS_OK)
    {
        result = ARUTILS_Http_ResetOptions(connection);
    }

    if (result == ARUTILS_OK)
    {
        if ((namePath[0] != '/') &&
            (namePath[0] != '?'))
        {
            splitter = "/";
        }
        snprintf(fileUrl, ARUTILS_HTTP_MAX_URL_SIZE, "%s%s%s",
                connection->serverUrl,
                splitter,
                namePath);

        code = curl_easy_setopt(connection->curl, CURLOPT_URL, fileUrl);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if ((result == ARUTILS_OK) && (dstFile != NULL))
    {
        connection->cbdata.writeFile = fopen(dstFile, "wb");

        if (connection->cbdata.writeFile == NULL)
        {
            result = ARUTILS_ERROR_SYSTEM;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_WRITEDATA, connection);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_WRITEFUNCTION, ARUTILS_Http_WriteDataCallback);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (progressCallback != NULL)
    {
        if (result == ARUTILS_OK)
        {
            connection->cbdata.progressCallback = progressCallback;
            connection->cbdata.progressArg = progressArg;

            code = curl_easy_setopt(connection->curl, CURLOPT_PROGRESSDATA, connection);

            if (code != CURLE_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }

        if (result == ARUTILS_OK)
        {
            code = curl_easy_setopt(connection->curl, CURLOPT_PROGRESSFUNCTION, ARUTILS_Http_ProgressCallback);

            if (code != CURLE_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }

        if (result == ARUTILS_OK)
        {
            code = curl_easy_setopt(connection->curl, CURLOPT_NOPROGRESS, 0L);

            if (code != CURLE_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }
    }

    //libcurl process
    if (result == ARUTILS_OK)
    {
        code = curl_easy_perform(connection->curl);

        if (code != CURLE_OK)
        {
            result = ARUTILS_Http_GetErrorFromCode(connection, code);
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_getinfo(connection->curl, CURLINFO_RESPONSE_CODE, &httpCode);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_GETINFO;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_getinfo(connection->curl, CURLINFO_CONTENT_LENGTH_DOWNLOAD, &remoteSize);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_GETINFO;
        }
    }

    //result checking
    if ((result == ARUTILS_OK) && (connection->cbdata.error != ARUTILS_OK))
    {
        result = connection->cbdata.error;
    }

    if (result == ARUTILS_OK)
    {
        // GET OK (200)
        if (httpCode == 200)
        {
            if (dstFile != NULL)
            {
                fflush(connection->cbdata.writeFile);
            }
        }
        else if (httpCode == 401)
        {
            result = ARUTILS_ERROR_HTTP_AUTHORIZATION_REQUIRED;
        }
        else if (httpCode == 403)
        {
            result = ARUTILS_ERROR_HTTP_ACCESS_DENIED;
        }
        else
        {
            result = ARUTILS_ERROR_HTTP_CODE;
        }
    }

    if ((result == ARUTILS_OK) && (dstFile != NULL))
    {
        result = ARUTILS_FileSystem_GetFileSize(dstFile, &localSize);

        if (result == ARUTILS_OK)
        {
            if (localSize != (int64_t)remoteSize)
            {
                result = ARUTILS_ERROR_HTTP_SIZE;
            }
        }
    }

    if ((result == ARUTILS_OK) && (dstFile == NULL))
    {
        // -1 when no Content-Length available
        if ((((int32_t)remoteSize) > 0)
            && (((uint32_t)remoteSize) != connection->cbdata.writeDataSize))
        {
            result = ARUTILS_ERROR_HTTP_SIZE;
        }
    }

    if ((result == ARUTILS_OK) && (dstFile == NULL) && (connection->cbdata.writeData != NULL))
    {
        // add trailing \0 to secure unchecked string used
        if (connection->cbdata.writeData[connection->cbdata.writeDataSize - 1] != '\0')
        {
            uint8_t *oldData = connection->cbdata.writeData;
            connection->cbdata.writeData = (uint8_t*)realloc(connection->cbdata.writeData, (connection->cbdata.writeDataSize + 1) * sizeof(uint8_t));
            if (connection->cbdata.writeData == NULL)
            {
                connection->cbdata.writeData = oldData;
                result = ARUTILS_ERROR_ALLOC;
            }
            else
            {
                connection->cbdata.writeData[connection->cbdata.writeDataSize] = '\0';
            }
        }
    }

    if ((result == ARUTILS_OK) && (data != NULL) && (dataLen != NULL))
    {
        if (result == ARUTILS_OK)
        {
            *data = connection->cbdata.writeData;
            connection->cbdata.writeData = NULL;
            *dataLen = connection->cbdata.writeDataSize;
        }
    }

    //cleanup
    if (connection != NULL)
    {
        ARUTILS_Http_FreeCallbackData(&connection->cbdata);
    }

    return result;
}

eARUTILS_ERROR ARUTILS_Http_SetSeverCertificate(ARUTILS_Http_Connection_t *connection, const char* certPath)
{
    eARUTILS_ERROR result = ARUTILS_OK;

    if ((strlen(certPath) + 1) > ARUTILS_HTTP_MAX_PATH_SIZE)
    {
        result = ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (result == ARUTILS_OK)
    {
        strncpy(connection->serverCert, certPath, ARUTILS_HTTP_MAX_PATH_SIZE);
        connection->serverCert[ARUTILS_HTTP_MAX_PATH_SIZE -1] = '\0';
    }

    return result;
}

eARUTILS_ERROR ARUTILS_Http_Post_WithFiles(ARUTILS_Http_Connection_t *connection, const char *namePath, ARUTILS_Http_File_t *fileList, int fileListCount, ARUTILS_Http_ProgressCallback_t progressCallback, void* progressArg)
{
    struct curl_slist *headers = NULL;
    struct curl_httppost *formItems = NULL;
    struct curl_httppost *lastFormItem = NULL;
    char fileUrl[ARUTILS_HTTP_MAX_URL_SIZE];
    CURLFORMcode formCode = CURL_FORMADD_OK;
    CURLcode code = CURLE_OK;
    long httpCode = 0L;
    eARUTILS_ERROR result = ARUTILS_OK;
    int i;
    char *splitter = "";

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_HTTP_TAG, "%d", fileListCount);

    if ((connection == NULL) || (connection->curl == NULL) || (fileList == NULL))
    {
        result =  ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (result == ARUTILS_OK)
    {
        result = ARUTILS_Http_IsCanceled(connection);
    }

    if (result == ARUTILS_OK)
    {
        result = ARUTILS_Http_ResetOptions(connection);
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_CUSTOMREQUEST, "POST");

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        if ((namePath[0] != '/') &&
            (namePath[0] != '?'))
        {
            splitter = "/";
        }
        snprintf(fileUrl, ARUTILS_HTTP_MAX_URL_SIZE, "%s%s%s",
                connection->serverUrl,
                splitter,
                namePath);

        code = curl_easy_setopt(connection->curl, CURLOPT_URL, fileUrl);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        headers = curl_slist_append(headers, "Expect:");
        if (headers == NULL)
        {
            result = ARUTILS_ERROR_ALLOC;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_HTTPHEADER, headers);
        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        char fileName[ARUTILS_HTTP_MAX_NAME_SIZE];

        for (i = 0; (result == ARUTILS_OK) && (i<fileListCount); i++)
        {
            ARUTILS_Http_File_t *fileItem = &fileList[i];
            sprintf(fileName, "file%d", i);

            formCode = curl_formadd(&formItems, &lastFormItem,
                                CURLFORM_COPYNAME, fileName,
                                CURLFORM_FILENAME, fileItem->name,
                                CURLFORM_FILE, fileItem->path,
                                CURLFORM_CONTENTTYPE, "application/octet-stream",
                                CURLFORM_END);
            if (formCode != CURL_FORMADD_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_HTTPPOST, formItems);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (progressCallback != NULL)
    {
        if (result == ARUTILS_OK)
        {
            connection->cbdata.progressCallback = progressCallback;
            connection->cbdata.progressArg = progressArg;

            code = curl_easy_setopt(connection->curl, CURLOPT_PROGRESSDATA, connection);

            if (code != CURLE_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }

        if (result == ARUTILS_OK)
        {
            code = curl_easy_setopt(connection->curl, CURLOPT_PROGRESSFUNCTION, ARUTILS_Http_ProgressCallback);

            if (code != CURLE_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }

        if (result == ARUTILS_OK)
        {
            code = curl_easy_setopt(connection->curl, CURLOPT_NOPROGRESS, 0L);

            if (code != CURLE_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }
    }

    //libcurl process
    if (result == ARUTILS_OK)
    {
        code = curl_easy_perform(connection->curl);

        if (code != CURLE_OK)
        {
            result = ARUTILS_Http_GetErrorFromCode(connection, code);
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_getinfo(connection->curl, CURLINFO_RESPONSE_CODE, &httpCode);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_GETINFO;
        }
    }

    //result checking
    if ((result == ARUTILS_OK) && (connection->cbdata.error != ARUTILS_OK))
    {
        result = connection->cbdata.error;
    }

    if (result == ARUTILS_OK)
    {
        // GET OK (200)
        if ((httpCode == 200) || (httpCode == 201))
        {
        }
        else if (httpCode == 401)
        {
            result = ARUTILS_ERROR_HTTP_AUTHORIZATION_REQUIRED;
        }
        else if (httpCode == 403)
        {
            result = ARUTILS_ERROR_HTTP_ACCESS_DENIED;
        }
        else
        {
            result = ARUTILS_ERROR_HTTP_CODE;
        }
    }

    //cleanup
    if (headers != NULL)
    {
        curl_slist_free_all(headers);
    }

    if (formItems != NULL)
    {
        curl_formfree(formItems);
    }

    if (connection != NULL)
    {
        ARUTILS_Http_FreeCallbackData(&connection->cbdata);
    }

    return result;
}

eARUTILS_ERROR ARUTILS_Http_Post_WithRange(ARUTILS_Http_Connection_t *connection, const char *namePath, const char *srcFile, const char *md5Txt, uint32_t startRange, uint32_t endRange, uint8_t **outData, uint32_t *outDataLen, ARUTILS_Http_ProgressCallback_t progressCallback, void* progressArg)
{
    char contentRange[ARUTILS_HTTP_MAX_PATH_SIZE];
    struct curl_slist *headers = NULL;
    struct curl_httppost *formItems = NULL;
    struct curl_httppost *lastFormItem = NULL;
    char fileUrl[ARUTILS_HTTP_MAX_URL_SIZE];
    eARUTILS_ERROR result = ARUTILS_OK;
    CURLFORMcode formCode = CURL_FORMADD_OK;
    CURLcode code = CURLE_OK;
    long httpCode = 0L;
    int64_t localSize = 0;
    char *splitter = "";

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_HTTP_TAG, "%s, %s", namePath ? namePath : "null", srcFile ? srcFile : "null");

    if ((connection == NULL) || (connection->curl == NULL) || (namePath == NULL) || (srcFile == NULL) || (md5Txt == NULL))
    {
        result =  ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (result == ARUTILS_OK)
    {
        result = ARUTILS_Http_IsCanceled(connection);
    }

    if (result == ARUTILS_OK)
    {
        result = ARUTILS_Http_ResetOptions(connection);
    }

    if ((result == ARUTILS_OK) && (srcFile != NULL))
    {
        result = ARUTILS_FileSystem_GetFileSize(srcFile, &localSize);
    }

    if (result == ARUTILS_OK)
    {
        if (endRange > (((uint32_t)localSize) - 1))
        {
            result =  ARUTILS_ERROR_BAD_PARAMETER;
        }
        else
        {
            connection->cbdata.readMaxSize = endRange + 1;
            connection->cbdata.readDataSize = startRange;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_CUSTOMREQUEST, "POST");

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        if ((namePath[0] != '/') &&
            (namePath[0] != '?'))
        {
            splitter = "/";
        }
        snprintf(fileUrl, ARUTILS_HTTP_MAX_URL_SIZE, "%s%s%s",
                connection->serverUrl,
                splitter,
                namePath);

        code = curl_easy_setopt(connection->curl, CURLOPT_URL, fileUrl);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if ((result == ARUTILS_OK) && (srcFile != NULL))
    {
        connection->cbdata.readFile = fopen(srcFile, "rb");

        if (startRange > 0)
        {
            int fileResult = fseek(connection->cbdata.readFile, (long)startRange, SEEK_SET);
            if (fileResult != 0)
            {
                result = ARUTILS_ERROR_SYSTEM;
            }
        }

        if (connection->cbdata.readFile == NULL)
        {
            result = ARUTILS_ERROR_SYSTEM;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_READDATA, connection);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_READFUNCTION, ARUTILS_Http_ReadDataCallback);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_WRITEDATA, connection);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_WRITEFUNCTION, ARUTILS_Http_WriteDataCallback);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        //Content-Range: bytes 0-6000/7913
        sprintf(contentRange, "Content-Range: bytes %d-%d/%d", startRange, endRange, ((uint32_t)localSize) - 1);
        headers = curl_slist_append(headers, contentRange);
        if (headers == NULL)
        {
            result = ARUTILS_ERROR_ALLOC;
        }
    }

    if (result == ARUTILS_OK)
    {
        headers = curl_slist_append(headers, "Expect:");
        if (headers == NULL)
        {
            result = ARUTILS_ERROR_ALLOC;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_HTTPHEADER, headers);
        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        formCode = curl_formadd(&formItems, &lastFormItem,
                                CURLFORM_COPYNAME, "md5",
                                CURLFORM_COPYCONTENTS, md5Txt,
                                CURLFORM_END);
        if (formCode != CURL_FORMADD_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        if (connection->cbdata.readFile != NULL)
        {
            const char *fileName = srcFile + strlen(srcFile);
            while ((fileName > srcFile) && (*fileName != '/'))
            {
                fileName--;
            }
            if (*fileName == '/')
            {
                fileName++;
            }

            formCode = curl_formadd(&formItems, &lastFormItem,
                                    CURLFORM_COPYNAME, "file",
                                    CURLFORM_STREAM, connection,
                                    CURLFORM_FILENAME, fileName,
                                    //CURLFORM_CONTENTSLENGTH, (long)localSize,
                                    CURLFORM_CONTENTSLENGTH, (long)(endRange + 1 - startRange),
                                    CURLFORM_CONTENTTYPE, "application/octet-stream",
                                    CURLFORM_END);
        }
        /*else
        {
            formCode = curl_formadd(&formItems, &lastFormItem,
                                     CURLFORM_COPYNAME, "file",//"userdata",
                                     //CURLFORM_FILENAME, "data.bin",
                                     CURLFORM_COPYCONTENTS, data,
                                     CURLFORM_CONTENTSLENGTH, (long)dataLen,
                                     CURLFORM_CONTENTTYPE, "application/octet-stream",
                                     CURLFORM_END);
        }*/

        if (formCode != CURL_FORMADD_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_HTTPPOST, formItems);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (progressCallback != NULL)
    {
        if (result == ARUTILS_OK)
        {
            connection->cbdata.progressCallback = progressCallback;
            connection->cbdata.progressArg = progressArg;

            code = curl_easy_setopt(connection->curl, CURLOPT_PROGRESSDATA, connection);

            if (code != CURLE_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }

        if (result == ARUTILS_OK)
        {
            code = curl_easy_setopt(connection->curl, CURLOPT_PROGRESSFUNCTION, ARUTILS_Http_ProgressCallback);

            if (code != CURLE_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }

        if (result == ARUTILS_OK)
        {
            code = curl_easy_setopt(connection->curl, CURLOPT_NOPROGRESS, 0L);

            if (code != CURLE_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }
    }

    //libcurl process
    if (result == ARUTILS_OK)
    {
        code = curl_easy_perform(connection->curl);

        if (code != CURLE_OK)
        {
            result = ARUTILS_Http_GetErrorFromCode(connection, code);
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_getinfo(connection->curl, CURLINFO_RESPONSE_CODE, &httpCode);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_GETINFO;
        }
    }

    //result checking
    if ((result == ARUTILS_OK) && (connection->cbdata.error != ARUTILS_OK))
    {
        result = connection->cbdata.error;
    }

    if (result == ARUTILS_OK)
    {
        // GET OK (200)
        if ((httpCode == 200) || (httpCode == 201))
        {
        }
        else if (httpCode == 401)
        {
            result = ARUTILS_ERROR_HTTP_AUTHORIZATION_REQUIRED;
        }
        else if (httpCode == 403)
        {
            result = ARUTILS_ERROR_HTTP_ACCESS_DENIED;
        }
        else
        {
            result = ARUTILS_ERROR_HTTP_CODE;
        }
    }

    if (result == ARUTILS_OK)
    {
        if ((outData != NULL) && (outDataLen != NULL))
        {
            *outData = connection->cbdata.writeData;
            connection->cbdata.writeData = NULL;
            *outDataLen = connection->cbdata.writeDataSize;
            connection->cbdata.writeDataSize = 0;
        }
    }

    //cleanup
    if (headers != NULL)
    {
        curl_slist_free_all(headers);
    }

    if (formItems != NULL)
    {
        curl_formfree(formItems);
    }

    if (connection != NULL)
    {
        ARUTILS_Http_FreeCallbackData(&connection->cbdata);
    }

    return result;
}

/*****************************************
 *
 *             Private implementation:
 *
 *****************************************/

eARUTILS_ERROR ARUTILS_Http_ResetOptions(ARUTILS_Http_Connection_t *connection)
{
    eARUTILS_ERROR result = ARUTILS_OK;
    CURLcode code = CURLE_OK;

    if ((connection == NULL) || (connection->curl == NULL))
    {
        result = ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (result == ARUTILS_OK)
    {
        ARUTILS_Http_FreeCallbackData(&connection->cbdata);
    }

    if (result == ARUTILS_OK)
    {
        curl_easy_reset(connection->curl);
    }

    if (result == ARUTILS_OK)
    {
#if (ARUTILS_HTTP_CURL_VERBOSE)
        code = curl_easy_setopt(connection->curl, CURLOPT_VERBOSE, 1L);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
#endif
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_URL, connection->serverUrl);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if ((result == ARUTILS_OK) && (strlen(connection->username) != 0))
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_USERNAME, connection->username);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if ((result == ARUTILS_OK) && (strlen(connection->password) != 0))
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_PASSWORD, connection->password);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_NOSIGNAL, 1);

        if ((code != CURLE_OK) && (code != CURLE_UNKNOWN_OPTION))
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_LOW_SPEED_LIMIT, ARUTILS_HTTP_LOW_SPEED_LIMIT);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_LOW_SPEED_TIME, ARUTILS_HTTP_LOW_SPEED_TIME);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_OPENSOCKETFUNCTION, ARUTILS_Http_OpensocketCallback);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_OPENSOCKETDATA, connection);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_CLOSESOCKETFUNCTION,ARUTILS_Http_ClosesocketCallback);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_CLOSESOCKETDATA, connection);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        code = curl_easy_setopt(connection->curl, CURLOPT_CONNECTTIMEOUT, ARUTILS_HTTP_TIMEOUT);

        if (code != CURLE_OK)
        {
            result = ARUTILS_ERROR_CURL_SETOPT;
        }
    }

    if (result == ARUTILS_OK)
    {
        if (connection->serverCert[0] == '\0')
        {
            code = curl_easy_setopt(connection->curl, CURLOPT_SSL_VERIFYPEER, 0);

            if (code != CURLE_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }
        else
        {
            code =  curl_easy_setopt(connection->curl, CURLOPT_CAINFO, connection->serverCert);

            if (code != CURLE_OK)
            {
                result = ARUTILS_ERROR_CURL_SETOPT;
            }
        }
    }

    return result;
}

size_t ARUTILS_Http_ReadDataCallback(void *ptr, size_t size, size_t nmemb, void *userData)
{
    ARUTILS_Http_Connection_t *connection = (ARUTILS_Http_Connection_t *)userData;
    size_t readSize = 0;
    size_t retSize = 0;

    //ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_HTTP_TAG, "%d, %d", (int)size, (int)nmemb);

    if (connection != NULL)
    {
        connection->cbdata.error = ARUTILS_Http_IsCanceled(connection);

        if (connection->cbdata.error == ARUTILS_OK)
        {
            if (connection->cbdata.readFile != NULL)
            {
                do
                {
                    if ((connection->cbdata.readDataSize + nmemb) > connection->cbdata.readMaxSize)
                    {
                        nmemb = connection->cbdata.readMaxSize - connection->cbdata.readDataSize;
                    }

                    readSize = fread(ptr, size, nmemb, connection->cbdata.readFile);

                    if (readSize == 0)
                    {
                        int err = ferror(connection->cbdata.readFile);
                        if (err != 0)
                        {
                            connection->cbdata.error = ARUTILS_ERROR_SYSTEM;
                        }
                    }
                    else
                    {
                        retSize = readSize;
                    }

                    connection->cbdata.readDataSize += retSize;
                }
                while ((connection->cbdata.error == ARUTILS_OK)
                       && (connection->cbdata.readDataSize < connection->cbdata.readMaxSize)
                       && (readSize == 0)
                       && !feof(connection->cbdata.readFile));
            }
        }

        if (connection->cbdata.error != ARUTILS_OK)
        {
            retSize = CURL_READFUNC_ABORT;
        }
    }
    else
    {
        retSize = 0;
    }

    return retSize;
}

size_t ARUTILS_Http_WriteDataCallback(void *ptr, size_t size, size_t nmemb, void *userData)
{
    ARUTILS_Http_Connection_t *connection = (ARUTILS_Http_Connection_t *)userData;
    u_char *olddata = NULL;
    size_t retSize = 0;

    //ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_HTTP_TAG, "%d, %d", (int)size, (int)nmemb);

    if (connection != NULL)
    {
        connection->cbdata.error = ARUTILS_Http_IsCanceled(connection);

        if (connection->cbdata.error == ARUTILS_OK)
        {
            if (connection->cbdata.writeFile == NULL)
            {
                olddata = connection->cbdata.writeData;
                connection->cbdata.writeData = (u_char *)realloc(connection->cbdata.writeData, connection->cbdata.writeDataSize + (size * nmemb));
                if (connection->cbdata.writeData == NULL)
                {
                    connection->cbdata.writeData = olddata;
                    connection->cbdata.error = ARUTILS_ERROR_ALLOC;
                }
            }
            else
            {
                size_t len = fwrite(ptr, size, nmemb, connection->cbdata.writeFile);
                if (len != size * nmemb)
                {
                    connection->cbdata.error = ARUTILS_ERROR_SYSTEM;
                }
                else
                {
                    connection->cbdata.writeDataSize += size * nmemb;
                    retSize = nmemb;
                }
            }
        }

        if ((connection->cbdata.error == ARUTILS_OK) && (connection->cbdata.writeFile == NULL) && (connection->cbdata.writeData != NULL))
        {
            memcpy(&connection->cbdata.writeData[connection->cbdata.writeDataSize], ptr, size * nmemb);
            connection->cbdata.writeDataSize += size * nmemb;
            retSize = nmemb;
        }

        if (connection->cbdata.error != ARUTILS_OK)
        {
            retSize = 0;
        }
    }
    else
    {
        retSize = 0;
    }

    return retSize;
}

int ARUTILS_Http_ProgressCallback(void *userData, double dltotal, double dlnow, double ultotal, double ulnow)
{
    ARUTILS_Http_Connection_t *connection = (ARUTILS_Http_Connection_t *)userData;
    float percent;

    //ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_HTTP_TAG, "%.0f, %.0f, %.0f, %.0f", dltotal, dlnow, ultotal, ulnow);

    if (connection != NULL)
    {
        if (connection->cbdata.progressCallback != NULL)
        {
            if (connection->cbdata.isUploading == 0)
            {
                // when 0, uploading isn't started
                if (dltotal != 0.f)
                {
                    percent = (dlnow / dltotal) * 100.f;
                    connection->cbdata.progressCallback(connection->cbdata.progressArg, percent);
                }
            }
            else
            {
                // when 0, downloading isn't started
                if (ultotal != 0.f)
                {
                    percent = ulnow / ultotal;
                    connection->cbdata.progressCallback(connection->cbdata.progressArg, percent);
                }
            }
        }
    }

    return 0;
}

curl_socket_t ARUTILS_Http_OpensocketCallback(void *clientp, curlsocktype purpose, struct curl_sockaddr *address)
{
    ARUTILS_Http_Connection_t *connection = clientp;
    curl_socket_t sock = 0;
    //ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_HTTP_TAG, "%x", clientp);

    if ((address != NULL) && (purpose == CURLSOCKTYPE_IPCXN))
    {
        sock = socket(address->family, address->socktype, address->protocol);

        if (connection != NULL)
        {
            connection->curlSocket = sock;
        }
    }

    return sock;
}

void ARUTILS_Http_ClosesocketCallback(void *clientp, curl_socket_t sock)
{
    ARUTILS_Http_Connection_t *connection = clientp;

    close(sock);
    if (connection != NULL)
    {
        connection->curlSocket = -1;
    }
}

void ARUTILS_Http_FreeCallbackData(ARUTILS_Http_CallbackData_t *cbdata)
{
    //ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_HTTP_TAG, "%x", cbdata);

    if (cbdata != NULL)
    {
        if (cbdata->readFile != NULL)
        {
            if (cbdata->readFile != NULL)
            {
                fclose(cbdata->readFile);
                cbdata->readFile = NULL;
            }
        }

        if (cbdata->writeFile != NULL)
        {
            if (cbdata->writeFile != NULL)
            {
                fclose(cbdata->writeFile);
                cbdata->writeFile = NULL;
            }
        }

        if (cbdata->readData != NULL)
        {
            free(cbdata->readData);
            cbdata->readData = NULL;
        }

        if (cbdata->writeData != NULL)
        {
            free(cbdata->writeData);
            cbdata->writeData = NULL;
        }

        memset(cbdata, 0, sizeof(ARUTILS_Http_CallbackData_t));
    }
}

eARUTILS_ERROR ARUTILS_Http_IsCanceled(ARUTILS_Http_Connection_t *connection)
{
    eARUTILS_ERROR result = ARUTILS_OK;

    if (connection == NULL)
    {
        result = ARUTILS_ERROR_BAD_PARAMETER;
    }

    if ((connection != NULL) && (connection->cancelSem != NULL))
    {
        int resultSys = ARSAL_Sem_Trywait(connection->cancelSem);

        if (resultSys == 0)
        {
            result = ARUTILS_ERROR_HTTP_CANCELED;

            //give back the signal state lost from trywait
            ARSAL_Sem_Post(connection->cancelSem);
        }
        else if (errno != EAGAIN)
        {
            result = ARUTILS_ERROR_SYSTEM;
        }
    }

    return result;
}

eARUTILS_ERROR ARUTILS_Http_GetErrorFromCode(ARUTILS_Http_Connection_t *connection, CURLcode code)
{
    eARUTILS_ERROR result = ARUTILS_ERROR;
    //long httpCode = 0L;
    //CURLcode codeInfo;

    switch (code)
    {
        case CURLE_WRITE_ERROR://write callback error
            result = ARUTILS_ERROR_HTTP_CODE;
            if (connection->cbdata.error != 0)
            {
                result = connection->cbdata.error;
            }
            break;

        case CURLE_COULDNT_RESOLVE_HOST:
            result = ARUTILS_ERROR_HTTP_CONNECT;
            break;

        /*case CURLE_QUOTE_ERROR: //(file or directory doesn't exist)
            result = ARUTILS_ERROR_CURL_PERFORM;
            codeInfo = curl_easy_getinfo(connection->curl, CURLINFO_RESPONSE_CODE, &httpCode);

            if (codeInfo == CURLE_OK)
            {
                if (httpCode != 200)
                {
                    result = ARUTILS_ERROR_HTTP_CODE;
                }
            }
            break;*/

        default:
            result = ARUTILS_ERROR_CURL_PERFORM;
            break;
    }

    return result;
}
