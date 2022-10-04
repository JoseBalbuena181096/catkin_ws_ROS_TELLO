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
 * @file ARUTILS_Ftp.h
 * @brief libARUtils Ftp header file.
 * @date 19/12/2013
 * @author david.flattin.ext@parrot.com
 **/

#ifndef _ARUTILS_WIFI_FTP_PRIVATE_H_
#define _ARUTILS_WIFI_FTP_PRIVATE_H_

#include "libARUtils/ARUTILS_Manager.h"

/**
 * @brief Ftp command Delete file
 */
#define FTP_CMD_DELE    "DELE "

/**
 * @brief Ftp command List directory
 */
#define FTP_CMD_LIST    "LIST "

/**
 * @brief Ftp command Name List directory
 */
#define FTP_CMD_NLST    "NLST "

/**
 * @brief Ftp command Rename From origine name
 */
#define FTP_CMD_RNFR    "RNFR "

/**
 * @brief Ftp command Rename To destination name
 */
#define FTP_CMD_RNTO    "RNTO "

/**
 * @brief Ftp command Remove Directory
 */
#define FTP_CMD_RMD     "RMD "

/**
 * @brief Ftp command Make Directory
 */
#define FTP_CMD_MKD     "MKD "

/**
 * @brief Ftp command Size get file size
 */
#define FTP_CMD_SIZE    "SIZE "

/**
 * @brief Ftp command Change working directory
 */
#define FTP_CMD_CWD     "CWD "

/**
 * @brief Ftp max user name string size
 */
#define ARUTILS_FTP_MAX_USER_SIZE     64

/**
 * @brief Ftp max socket opened
 */
#define ARUTILS_FTP_MAX_SOCKET_SIZE    4

/**
 * @brief Ftp CallbackData structure
 * @param isUploading Is set to 1 if upploading else 0 of downloading
 * @param data The byte buffer data if data mode else null
 * @param dataSize The byte buffer data size if data mode else 0
 * @param file The file data if file mode else null
 * @param error The last error
 * @param progressCallback The progress callback
 * @param progressArg The progress arg
 * @see ARUTILS_WifiFtp_ReadDataCallback (), ARUTILS_WifiFtp_WriteDataCallback ()
 */
typedef struct _ARUTILS_WifiFtp_CallbackData_t_
{
    int isUploading;
    uint8_t *data;
    uint32_t dataSize;
    double resumeSize;
    double totalSize;
    int fileFd;
    eARUTILS_ERROR error;
    ARUTILS_Ftp_ProgressCallback_t progressCallback;
    void *progressArg;

} ARUTILS_WifiFtp_CallbackData_t;

/**
 * @brief Ftp Connection structure
 * @param cancelSem The semaphore to cancel Ftp command
 * @param curl The cURL connection
 * @param serverUrl The Ftp url connection string
 * @param username The Ftp connection user name
 * @param passwordThe Ftp connection user password
 * @param cbdata The Ftp connection data for callbacks
 * @see ARUTILS_WifiFtp_ReadDataCallback (), ARUTILS_WifiFtp_WriteDataCallback ()
 */
//typedef struct _ARUTILS_WifiFtp_Connection_t_
struct ARUTILS_WifiFtp_Connection_t
{
    ARSAL_Sem_t *cancelSem;
    CURL *curl;
    int curlSocket[ARUTILS_FTP_MAX_SOCKET_SIZE];
    struct mux_ctx *mux; //<< set when the ftp is using the mux
    uint32_t mux_channel;    //<< set when the ftp is using the mux
    char serverUrl[ARUTILS_FTP_MAX_URL_SIZE];
    char username[ARUTILS_FTP_MAX_USER_SIZE];
    char password[ARUTILS_FTP_MAX_USER_SIZE];
    ARUTILS_WifiFtp_CallbackData_t cbdata;
};
//} ARUTILS_WifiFtp_Connection_t;

typedef struct ARUTILS_WifiFtp_Connection_t ARUTILS_WifiFtp_Connection_t;


/**
 * @brief Create a new Ftp Connection
 * @warning This function allocates memory
 * @param cancelSem The pointer of the Ftp get/put cancel semaphore or null
 * @param server The Ftp server IP address
 * @param port The Ftp server port
 * @param mux the mux if connected on a mux channel, null else
 * @param username The Ftp server account name
 * @param password The Ftp server account password
 * @param[out] error The pointer of the error code: if success ARUTILS_OK, otherwise an error number of eARUTILS_ERROR
 * @retval On success, returns an ARUTILS_WifiFtp_Connection_t. Otherwise, it returns null.
 * @see ARUTILS_WifiFtp_DeleteConnection ()
 */
ARUTILS_WifiFtp_Connection_t * ARUTILS_WifiFtp_Connection_New(ARSAL_Sem_t *cancelSem, const char *server, int port,
		struct mux_ctx* mux, const char *username, const char* password, eARUTILS_ERROR *error);

/**
 * @brief Delete an Ftp Connection
 * @warning This function frees memory
 * @param connection The address of the pointer on the Ftp Connection
 * @see ARUTILS_WifiFtp_NewConnection ()
 */
void ARUTILS_WifiFtp_Connection_Delete(ARUTILS_WifiFtp_Connection_t **connection);

/**
 * @brief Cancel an Ftp Connection command in progress (get, put, list etc)
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_WifiFtp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_WifiFtp_Connection_Cancel(ARUTILS_WifiFtp_Connection_t *connection);

/**
 * @brief Check if the connection has received a cancel to it's semaphore
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see cURL
 */
eARUTILS_ERROR ARUTILS_WifiFtp_IsCanceled(ARUTILS_WifiFtp_Connection_t *connection);

/**
 * @brief Reset an Ftp Connection command in progress (get, put, list etc)
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_WifiFtp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_WifiFtp_Connection_Reset(ARUTILS_WifiFtp_Connection_t *connection);

/**
 * @brief Execute Ftp List command to retrieve directory content
 * @warning This function allocates memory
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the directory path on the remote Ftp server
 * @param resultList The pointer of the string of the directory content null terminated
 * @param resultListLen The pointer of the lenght of the resultList string including null terminated
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_WifiFtp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_WifiFtp_List(ARUTILS_WifiFtp_Connection_t *connection, const char *namePath, char **resultList, uint32_t *resultListLen);

/**
 * @brief Rename an remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param oldNamePath The string of the old file name path on the remote Ftp server
 * @param newNamePath The string of the new file name path on the remote Ftp server
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_WifiFtp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_WifiFtp_Rename(ARUTILS_WifiFtp_Connection_t *connection, const char *oldNamePath, const char *newNamePath);

/**
 * @brief Get the size of an remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_WifiFtp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_WifiFtp_Size(ARUTILS_WifiFtp_Connection_t *connection, const char *namePath, double *size);

/**
 * @brief Delete an remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_WifiFtp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_WifiFtp_Delete(ARUTILS_WifiFtp_Connection_t *connection, const char *namePath);

/**
 * @brief Get an remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param dstFile The string of the local file name path to be get
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @param resume The resume capability requested
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_WifiFtp_NewConnection (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_WifiFtp_Get(ARUTILS_WifiFtp_Connection_t *connection, const char *namePath, const char *dstFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume);

/**
 * @brief Get an remote Ftp server file
 * @warning This function allocates memory
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param data The pointer of the data buffer of the file data
 * @param dataLen The pointer of the length of the data buffer
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_WifiFtp_NewConnection (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_WifiFtp_Get_WithBuffer(ARUTILS_WifiFtp_Connection_t *connection, const char *namePath, uint8_t **data, uint32_t *dataLen,  ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg);

/**
 * @brief Put an remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param srcFile The string of the local file name path to be put
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @param resume The resume capability requested
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_WifiFtp_NewConnection (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_WifiFtp_Put(ARUTILS_WifiFtp_Connection_t *connection, const char *namePath, const char *srcFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume);


/**
 * @brief ReadData callback of cURL connection
 * @param ptr The pointer of read data
 * @param size The size of the read data type (byte)
 * @param nmemb The number of read data present
 * @param userData The pointer of the user custom argument
 * @retval On success, returns nmemb. Otherwise, it returns an error code.
 * @see cURL
 */
size_t ARUTILS_WifiFtp_ReadDataCallback(void *ptr, size_t size, size_t nmemb, void *userData);

/**
 * @brief WriteData callback of cURL connection
 * @param ptr The pointer of write data
 * @param size The size of the write data type (byte)
 * @param nmemb The number of write data present
 * @param userData The pointer of the user custom argument
 * @retval On success, returns nmemb. Otherwise, it returns an error code.
 * @see cURL
 */
size_t ARUTILS_WifiFtp_WriteDataCallback(void *ptr, size_t size, size_t nmemb, void *userData);

/**
 * @brief Progress callback of cURL connection
 * @param userData The pointer of the user custom argument
 * @param dltotal The total size to be downloaded
 * @param dlnow The current size already donloaded
 * @param ultotal The total size to be uploaded
 * @param ulnow The current size already uploaded
 * @retval On success, returns 0. Otherwise, it returns an error code.
 * @see cURL
 */
int ARUTILS_WifiFtp_ProgressCallback(void *userData, double dltotal, double dlnow, double ultotal, double ulnow);

/**
 * @brief Opensocket callback of cURL connection
 * @param clientp
 * @param purpose
 * @param address
 * @retval On success, returns socket. Otherwise, it returns 0.
 * @see cURL
 */
curl_socket_t ARUTILS_WifiFtp_OpensocketCallback(void *clientp, curlsocktype purpose, struct curl_sockaddr *address);

/**
 * @brief Closesocket callback of cURL connection
 * @param clientp
 * @param sock
 * @see cURL
 */
void ARUTILS_WifiFtp_ClosesocketCallback(void *clientp, curl_socket_t sock);

/**
 * @brief Execute a command on remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path
 * @param command The string of the Ftp command
 * @param[out] ftpCode The resule code of the Ftp command
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see cURL
 */
eARUTILS_ERROR ARUTILS_WifiFtp_Command(ARUTILS_WifiFtp_Connection_t *connection, const char *namePath, const char *command, long *ftpCode);

/**
 * @brief Reset the Ftp connection values
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see cURL
 */
eARUTILS_ERROR ARUTILS_WifiFtp_ResetOptions(ARUTILS_WifiFtp_Connection_t *connection);

/**
 * @brief Execute Cd command on remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see cURL
 */
eARUTILS_ERROR ARUTILS_WifiFtp_Cd(ARUTILS_WifiFtp_Connection_t *connection, const char *namePath);

/**
 * @brief Execute Get command on remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path
 * @param dstFile The destination string file name path
 * @param[out] data Returns byte buffer data address if data mode else give null pointer
 * @param[out] dataLen Returns byte buffer data length else give null pointer
 * @param progressCallback The progress callback
 * @param progressArg The progress callback arg
 * @param resume The resune enun requested action
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see cURL
 */
eARUTILS_ERROR ARUTILS_WifiFtp_GetInternal(ARUTILS_WifiFtp_Connection_t *connection, const char *namePath, const char *dstFile, uint8_t **data, uint32_t *dataLen, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume);

/**
 * @brief Free CallbackData structure
 * @warning This function frees memory
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path
 * @see cURL
 */
void ARUTILS_WifiFtp_FreeCallbackData(ARUTILS_WifiFtp_CallbackData_t *cbdata);

/**
 * @brief Translate cURL error code to an eARUTILS_ERROR enum error
 * @param connection The address of the pointer on the Ftp Connection
 * @param code The cURL error code
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see cURL
 */
eARUTILS_ERROR ARUTILS_WifiFtp_GetErrorFromCode(ARUTILS_WifiFtp_Connection_t *connection, CURLcode code);

/**
 * @brief Disconnect an remote Ftp server connection
 * @param connection The address of the pointer on the Ftp Manager
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewWifiFtp ()
 */
eARUTILS_ERROR ARUTILS_WifiFtpAL_Connection_Disconnect(ARUTILS_Manager_t *manager);

/**
 * @brief Reconnect an remote Ftp server connection
 * @param connection The address of the pointer on the Ftp Manager
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewWifiFtp ()
 */
eARUTILS_ERROR ARUTILS_WifiFtpAL_Connection_Reconnect(ARUTILS_Manager_t *manager);

/**
* @brief Cancel an Ftp Connection command in progress (get, put, list etc)
* @param manager The address of the pointer on the Ftp Connection
* @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
* @see ARUTILS_Manager_NewWifiFtp ()
*/
eARUTILS_ERROR ARUTILS_WifiFtpAL_Connection_Cancel(ARUTILS_Manager_t *manager);

/**
 * @brief Check if the connection has received a cancel to it's semaphore
 * @param manager The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see cURL
 */
eARUTILS_ERROR ARUTILS_WifiFtpAL_Connection_IsCanceled(ARUTILS_Manager_t *manager);

/**
* @brief Reset an Ftp Connection command in progress (get, put, list etc)
* @param manager The address of the pointer on the Ftp Connection
* @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
* @see ARUTILS_Manager_NewWifiFtp ()
*/
eARUTILS_ERROR ARUTILS_WifiFtpAL_Connection_Reset(ARUTILS_Manager_t *manager);

/**
 * @brief Execute Ftp List command to retrieve directory content
 * @warning This function allocates memory
 * @param manager The address of the pointer on the Ftp Connection
 * @param namePath The string of the directory path on the remote Ftp server
 * @param resultList The pointer of the string of the directory content null terminated
 * @param resultListLen The pointer of the lenght of the resultList string including null terminated
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewWifiFtp ()
 */
eARUTILS_ERROR ARUTILS_WifiFtpAL_List(ARUTILS_Manager_t *manager, const char *namePath, char **resultList, uint32_t *resultListLen);

/**
 * @brief Execute Ftp Size command to retrieve file size
 * @warning This function allocates memory
 * @param manager The address of the pointer on the Ftp Connection
 * @param namePath The string of the directory path on the remote Ftp server
 * @param fileSize The returned file size
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewWifiFtp ()
 */
eARUTILS_ERROR ARUTILS_WifiFtpAL_Size(ARUTILS_Manager_t *manager, const char *namePath, double *fileSize);

/**
 * @brief Get an remote Ftp server file
 * @warning This function allocates memory
 * @param manager The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param data The pointer of the data buffer of the file data
 * @param dataLen The pointer of the length of the data buffer
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewWifiFtp (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_WifiFtpAL_Get_WithBuffer(ARUTILS_Manager_t *manager, const char *namePath, uint8_t **data, uint32_t *dataLen,  ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg);

/**
 * @brief Get an remote Ftp server file
 * @param manager The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param dstFile The string of the local file name path to be get
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @param resume The resume capability requested
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewWifiFtp (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_WifiFtpAL_Get(ARUTILS_Manager_t *manager, const char *namePath, const char *dstFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume);

/**
 * @brief Put an remote Ftp server file
 * @param manager The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param srcFile The string of the local file name path to be put
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @param resume The resume capability requested
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewWifiFtp (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_WifiFtpAL_Put(ARUTILS_Manager_t *manager, const char *namePath, const char *srcFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume);

/**
 * @brief Delete an remote Ftp server file
 * @param manger The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewWifiFtp ()
 */
eARUTILS_ERROR ARUTILS_WifiFtpAL_Delete(ARUTILS_Manager_t *manager, const char *namePath);

/**
 * @brief Delete an remote Ftp server directory
 * @param manger The address of the pointer on the Ftp Connection
 * @param namePath The string of the directory name path on the remote Ftp server
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewWifiFtp ()
 */
eARUTILS_ERROR ARUTILS_WifiFtpAL_RemoveDir(ARUTILS_Manager_t *manager, const char *namePath);

/**
 * @brief Rename an remote Ftp server file
 * @param manager The address of the pointer on the Ftp Connection
 * @param oldNamePath The string of the old file name path on the remote Ftp server
 * @param newNamePath The string of the new file name path on the remote Ftp server
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewWifiFtp ()
 */
eARUTILS_ERROR ARUTILS_WifiFtpAL_Rename(ARUTILS_Manager_t *manager, const char *oldNamePath, const char *newNamePath);

#endif /* _ARUTILS_WIFI_FTP_PRIVATE_H_ */
