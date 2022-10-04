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
 * @file  ARUTILS_JNIRFCommNetwork.h
 * @brief private headers of RFComm network manager allow to send over RFComm network.
 * @date
 * @author
 */

#ifndef _ARUTILS_JNI_RFCOMMFTP_PRIVATE_H_
#define _ARUTILS_JNI_RFCOMMFTP_PRIVATE_H_

#include <libARUtils/ARUTILS_Manager.h>


/**
 * @brief RFCommFtp Connection structure
 * @param cancelSem The semaphore to cancel Ftp command
 * @param RFCommFtpObject The RFComm manager
 * @see ARUTILS_RFCommFtp_Connection_New
 */
typedef struct _ARUTILS_RFCommFtp_Connection_t_
{
    jobject rfcommFtpObject;
    jobject cancelSemObject;

} ARUTILS_RFCommFtp_Connection_t;

typedef struct _ARUTILS_RFCommFtp_Command_t_
{
    ARUTILS_Ftp_ProgressCallback_t rfcommFtpProgressCallback;
    void *rfcommProgressArg;

} ARUTILS_RFCommFtp_Command_t;

/**
 * @brief Create a new Ftp Connection
 * @warning This function allocates memory
 * @param cancelSem The pointer of the Ftp get/put cancel semaphore or null
 * @param device The RFComm Ftp device
 * @param[out] error The pointer of the error code: if success ARUTILS_OK, otherwise an error number of eARUTILS_ERROR
 * @retval On success, returns an ARUTILS_FtpAL_Connection_t. Otherwise, it returns null.
 * @see ARUTILS_FtpAL_DeleteConnection ()
 */
ARUTILS_RFCommFtp_Connection_t * ARUTILS_RFCommFtp_Connection_New(jobject rfcommFtp, jobject cancelSem, eARUTILS_ERROR *error);

/**
 * @brief Delete an Ftp Connection
 * @warning This function frees memory
 * @param connection The address of the pointer on the Ftp Connection
 * @see ARUTILS_FtpAL_NewConnection ()
 */
void ARUTILS_RFCommFtp_Connection_Delete(ARUTILS_RFCommFtp_Connection_t **connection);

/**
 * @brief Cancel an Ftp Connection command in progress (get, put, list etc)
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Ftp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_RFCommFtp_Connection_Cancel(ARUTILS_RFCommFtp_Connection_t *connection);

/**
 * @brief Check if the connection has received a cancel to it's semaphore
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see cURL
 */
eARUTILS_ERROR ARUTILS_RFCommFtp_IsCanceled(ARUTILS_RFCommFtp_Connection_t *connection);

/**
 * @brief Reset an Ftp Connection command in progress (get, put, list etc)
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Ftp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_RFCommFtp_Connection_Reset(ARUTILS_RFCommFtp_Connection_t *connection);


/**
 * @brief Put an remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param srcFile The string of the local file name path to be put
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @param resume The resume capability requested
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_RFCommFtp_NewConnection (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_RFCommFtp_Put(ARUTILS_RFCommFtp_Connection_t *connection, const char *remotePath, const char *srcFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume);

/**
 * @brief Disconnect an remote Ftp server connection
 * @param connection The address of the pointer on the Ftp Manager
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_InitWifiFtp (), ARUTILS_Manager_InitBLEFtp (), ARUTILS_Manager_InitRFCommFtp ()
 */
eARUTILS_ERROR ARUTILS_RFCommFtpAL_Connection_Disconnect(ARUTILS_Manager_t *manager);

/**
 * @brief Reconnect an remote Ftp server connection
 * @param connection The address of the pointer on the Ftp Manager
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_InitWifiFtp (), ARUTILS_Manager_InitBLEFtp (), ARUTILS_Manager_InitRFCommFtp ()
 */
eARUTILS_ERROR ARUTILS_RFCommFtpAL_Connection_Reconnect(ARUTILS_Manager_t *manager);

/**
 * @brief Cancel an Ftp Connection command in progress (get, put, list etc)
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewRFCommFtp ()
 */
eARUTILS_ERROR ARUTILS_RFCommFtpAL_Connection_Cancel(ARUTILS_Manager_t *manager);

 /**
 * @brief Check if the Ftp Connection has received a cancel to it's semaphore
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see cURL
 */
 eARUTILS_ERROR ARUTILS_RFCommFtpAL_Connection_IsCanceled(ARUTILS_Manager_t *manager);

/**
 * @brief Reset an Ftp Connection command in progress (get, put, list etc)
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewRFCommFtp ()
 */
eARUTILS_ERROR ARUTILS_RFCommFtpAL_Connection_Reset(ARUTILS_Manager_t *manager);

/**
 * @brief Put an remote Ftp server file
 * @param manager The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param srcFile The string of the local file name path to be put
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @param resume The resume capability requested
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Manager_NewRFCommFtp (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_RFCommFtpAL_Put(ARUTILS_Manager_t *manager, const char *namePath, const char *srcFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume);

#endif /** _ARUTILS_JNI_RFCOMMNETWORK_PRIVATE_H_ */
