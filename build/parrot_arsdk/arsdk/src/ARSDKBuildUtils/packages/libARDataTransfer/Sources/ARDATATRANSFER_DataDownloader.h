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
 * @file ARDATATRANSFER_DataDownloader.h
 * @brief libARDataTransfer DataDownloader header file.
 * @date 19/12/2013
 * @author david.flattin.ext@parrot.com
 **/

#ifndef _ARDATATRANSFER_DATA_DOWNLOADER_PRIVATE_H_
#define _ARDATATRANSFER_DATA_DOWNLOADER_PRIVATE_H_

/**
 * @brief DataDownloader structure
 * @param isInitialized Is set to 1 if DataDownloader initilized else 0
 * @param isCanceled Is set to 1 if DataDownloader Thread is canceled else 0
 * @param isRunning Is set to 1 if DataDownloader Thread is running else 0
 * @param ftp The FTP DataDownloader connection
 * @param localDirectory The local directory where DataDownloader download files
 * @param sem The semaphore to cancel the DataDownloader Thread and its FTP connection
 * @see ARDATATRANSFER_DataDownloader_New ()
 */
typedef struct
{
    int isCanceled;
    int isRunning;
    ARUTILS_Manager_t *ftpListManager;
    ARUTILS_Manager_t *ftpDataManager;
    char remoteDirectory[ARUTILS_FTP_MAX_PATH_SIZE];
    char localDataDirectory[ARUTILS_FTP_MAX_PATH_SIZE];
    char localCrashReportsDirectory[ARUTILS_FTP_MAX_PATH_SIZE];
    ARSAL_Sem_t threadSem;
    ARDATATRANSFER_DataDownloader_FileCompletionCallback_t fileCompletionCallback;
    void *fileCompletionArg;

} ARDATATRANSFER_DataDownloader_t;

/**
 * @brief DataDownloader structure
 * @param sum The current sum
 * @param allowedSpace The maximum allowed space
 * @param dir The directory to parse
 * @see ARDATATRANSFER_DataDownloader_New ()
 */
typedef struct
{
    double sum;
    double allowedSpace;
    char dir[ARUTILS_FTP_MAX_PATH_SIZE];

} ARDATATRANSFER_DataDownloader_Fwt_t;

/**
 * @brief Initialize the device DataDownloader (flights data or ride data)
 * @warning This function allocates memory
 * @param manager The pointer of the ADataTransfer Manager
 * @param ftpListManager The ftp list manager
 * @param ftpDataManager The ftp data manager
 * @param localDirectory The path of the local directory where to store data
 * @retval On success, returns ARDATATRANSFER_OK. Otherwise, it returns an error number of eARDATATRANSFER_ERROR.
 * @see ARDATATRANSFER_DataDownloader_ThreadRun ()
 */
eARDATATRANSFER_ERROR ARDATATRANSFER_DataDownloader_Initialize(ARDATATRANSFER_Manager_t *manager, ARUTILS_Manager_t *ftpListManager, ARUTILS_Manager_t *ftpDataManager, const char *remoteDirectory, const char *localDirectory);

/**
 * @brief Delete an ARDataTransfer DataDownloader connection data
 * @warning This function frees memory
 * @param manager The address of the pointer on the ARDataTransfer Manager
 * @see ARDATATRANSFER_DataDownloader_New ()
 */
void ARDATATRANSFER_DataDownloader_Clear(ARDATATRANSFER_Manager_t *manager);

/**
 * @brief Remove older data file when exceed the free precent available space allowed
 * @param localPath The local directory path where to check available space
 * @param spacePercent The percent of available space allowed to ride data consuming
 * @see ARDATATRANSFER_DataDownloader_New ()
 */
eARDATATRANSFER_ERROR ARDATATRANSFER_DataDownloader_CheckUsedMemory(const char *localPath, float spacePercent);

/**
 * @brief Compare file extension
 * @param fileName The file name or path to compare with
 * @param ext The extension to compare with
 * @retval 0, -1 or lower -1 or 1 or upper 1
 * @see strcmp C lib
 */
int ARDATATRANSFER_DataDownloader_CompareFileExtension(const char* fileName, const char* ext);

/**
 * @brief Download PUD files from ftp
 * @param manager The pointer of the ADataTransfer Manager
 * @param errorUtils The pointer of the ftp error
 * @retval On success, returns ARDATATRANSFER_OK. Otherwise, it returns an error number of eARDATATRANSFER_ERROR.
 * @see ARDATATRANSFER_DataDownloader_ThreadRun ()
 */
eARDATATRANSFER_ERROR ARDATATRANSFER_DataDownloader_DownloadPudFiles(ARDATATRANSFER_Manager_t *manager, eARUTILS_ERROR *errorUtils);

/**
 * @brief Download CrashReport files from ftp
 * @param manager The pointer of the ADataTransfer Manager
 * @param errorUtils The pointer of the ftp error
 * @retval On success, returns ARDATATRANSFER_OK. Otherwise, it returns an error number of eARDATATRANSFER_ERROR.
 * @see ARDATATRANSFER_DataDownloader_ThreadRun ()
 */
eARDATATRANSFER_ERROR ARDATATRANSFER_DataDownloader_DownloadCrashReports(ARDATATRANSFER_Manager_t *managere, eARUTILS_ERROR *errorUtils);

/**
 * @brief Download CrashReport files from ftp
 * @param manager The pointer of the ADataTransfer Manager
 * @param remoteDirPath The remote ftp server directory
 * @param errorUtils The pointer of the ftp error
 * @retval On success, returns ARDATATRANSFER_OK. Otherwise, it returns an error number of eARDATATRANSFER_ERROR.
 * @see ARDATATRANSFER_DataDownloader_DownloadCrashReports ()
 */
eARDATATRANSFER_ERROR ARDATATRANSFER_DataDownloader_RemoveRemoteDir(ARDATATRANSFER_Manager_t *manager, const char* remoteDirPath, eARUTILS_ERROR *errorUtils);

/**
 * @brief Get the current CrashReport local report diretory index
 * @param reportDir The local report directory
 * @retval Returns the index number if any else 0
 * @see ARDATATRANSFER_DataDownloader_DownloadCrashReports ()
 */
unsigned int ARDATATRANSFER_DataDownloader_GetCrashReportIndex(const char *reportDir);

#endif /* _ARDATATRANSFER_DATA_DOWNLOADER_PRIVATE_H_ */
