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
 * @file ARDATATRANSFER_Manager.h
 * @brief libARDataTransfer Manager header file.
 * @date 19/12/2013
 * @author david.flattin.ext@parrot.com
 **/

#ifndef _ARDATATRANSFER_MANAGER_PRIVATE_H_
#define _ARDATATRANSFER_MANAGER_PRIVATE_H_

#define ARDATATRANSFER_MANAGER_DOWNLOADER_DOWNLOADING_PREFIX      "downloading_"
#define ARDATATRANSFER_MANAGER_DOWNLOADER_PROCESSING_PREFIX       "processing_"
#define ARDATATRANSFER_MANAGER_DOWNLOADER_PRODUCT_ID_MAX_SIZE   10

/**
 * @brief Manager structure
 * @param dataDownloader The DataDownloader
 * @param mediasDownloader The MediasDownloader
 * @see
 */
struct ARDATATRANSFER_Manager_t
{
    ARDATATRANSFER_Downloader_t *downloader;
    ARDATATRANSFER_Uploader_t *uploader;
    ARDATATRANSFER_DataDownloader_t *dataDownloader;
    ARDATATRANSFER_MediasDownloader_t *mediasDownloader;
};

#endif /* _ARDATATRANSFER_MANAGER_PRIVATE_H_ */