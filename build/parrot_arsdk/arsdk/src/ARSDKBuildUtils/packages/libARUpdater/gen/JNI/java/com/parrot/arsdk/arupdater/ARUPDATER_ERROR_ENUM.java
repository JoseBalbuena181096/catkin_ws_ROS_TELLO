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
/*
 * GENERATED FILE
 *  Do not modify this file, it will be erased during the next configure run 
 */

package com.parrot.arsdk.arupdater;

import java.util.HashMap;

/**
 * Java copy of the eARUPDATER_ERROR enum
 */
public enum ARUPDATER_ERROR_ENUM {
   /** Dummy value for all unknown cases */
    eARUPDATER_ERROR_UNKNOWN_ENUM_VALUE (Integer.MIN_VALUE, "Dummy value for all unknown cases"),
   /** No error */
    ARUPDATER_OK (0, "No error"),
   /** Unknown generic error */
    ARUPDATER_ERROR (-1000, "Unknown generic error"),
   /** Memory allocation error */
    ARUPDATER_ERROR_ALLOC (-999, "Memory allocation error"),
   /** Bad parameters error */
    ARUPDATER_ERROR_BAD_PARAMETER (-998, "Bad parameters error"),
   /** System error */
    ARUPDATER_ERROR_SYSTEM (-997, "System error"),
   /** Thread processing error */
    ARUPDATER_ERROR_THREAD_PROCESSING (-996, "Thread processing error"),
   /** Generic manager error */
    ARUPDATER_ERROR_MANAGER (-2000, "Generic manager error"),
   /** The uploader or downloader is already initilized in the manager */
    ARUPDATER_ERROR_MANAGER_ALREADY_INITIALIZED (-1999, "The uploader or downloader is already initilized in the manager"),
   /** The uploader or downloader is not initialized in the manager */
    ARUPDATER_ERROR_MANAGER_NOT_INITIALIZED (-1998, "The uploader or downloader is not initialized in the manager"),
   /** The given buffer is too small */
    ARUPDATER_ERROR_MANAGER_BUFFER_TOO_SMALL (-1997, "The given buffer is too small"),
   /** Generic PLF error */
    ARUPDATER_ERROR_PLF (-3000, "Generic PLF error"),
   /** Plf File not found */
    ARUPDATER_ERROR_PLF_FILE_NOT_FOUND (-2999, "Plf File not found"),
   /** Generic Updater error */
    ARUPDATER_ERROR_DOWNLOADER (-4000, "Generic Updater error"),
   /** error on a ARUtils operation */
    ARUPDATER_ERROR_DOWNLOADER_ARUTILS_ERROR (-3999, "error on a ARUtils operation"),
   /** error downloading a file */
    ARUPDATER_ERROR_DOWNLOADER_DOWNLOAD (-3998, "error downloading a file"),
   /** error on a platform name */
    ARUPDATER_ERROR_DOWNLOADER_PLATFORM_ERROR (-3997, "error on a platform name"),
   /** This app version is out to date */
    ARUPDATER_ERROR_DOWNLOADER_PHP_APP_OUT_TO_DATE_ERROR (-3996, "This app version is out to date"),
   /** error given by the PHP script on server */
    ARUPDATER_ERROR_DOWNLOADER_PHP_ERROR (-3995, "error given by the PHP script on server"),
   /** error when renaming files */
    ARUPDATER_ERROR_DOWNLOADER_RENAME_FILE (-3994, "error when renaming files"),
   /** Plf file not found in the downloader */
    ARUPDATER_ERROR_DOWNLOADER_FILE_NOT_FOUND (-3993, "Plf file not found in the downloader"),
   /** MD5 checksum does not match with the remote file */
    ARUPDATER_ERROR_DOWNLOADER_MD5_DONT_MATCH (-3992, "MD5 checksum does not match with the remote file"),
   /** Generic Uploader error */
    ARUPDATER_ERROR_UPLOADER (-5000, "Generic Uploader error"),
   /** error on a ARUtils operation in uploader */
    ARUPDATER_ERROR_UPLOADER_ARUTILS_ERROR (-4999, "error on a ARUtils operation in uploader"),
   /** error on a ARDataTransfer operation in uploader */
    ARUPDATER_ERROR_UPLOADER_ARDATATRANSFER_ERROR (-4998, "error on a ARDataTransfer operation in uploader"),
   /** error on a ARSAL operation in uploader */
    ARUPDATER_ERROR_UPLOADER_ARSAL_ERROR (-4997, "error on a ARSAL operation in uploader");

    private final int value;
    private final String comment;
    static HashMap<Integer, ARUPDATER_ERROR_ENUM> valuesList;

    ARUPDATER_ERROR_ENUM (int value) {
        this.value = value;
        this.comment = null;
    }

    ARUPDATER_ERROR_ENUM (int value, String comment) {
        this.value = value;
        this.comment = comment;
    }

    /**
     * Gets the int value of the enum
     * @return int value of the enum
     */
    public int getValue () {
        return value;
    }

    /**
     * Gets the ARUPDATER_ERROR_ENUM instance from a C enum value
     * @param value C value of the enum
     * @return The ARUPDATER_ERROR_ENUM instance, or null if the C enum value was not valid
     */
    public static ARUPDATER_ERROR_ENUM getFromValue (int value) {
        if (null == valuesList) {
            ARUPDATER_ERROR_ENUM [] valuesArray = ARUPDATER_ERROR_ENUM.values ();
            valuesList = new HashMap<Integer, ARUPDATER_ERROR_ENUM> (valuesArray.length);
            for (ARUPDATER_ERROR_ENUM entry : valuesArray) {
                valuesList.put (entry.getValue (), entry);
            }
        }
        ARUPDATER_ERROR_ENUM retVal = valuesList.get (value);
        if (retVal == null) {
            retVal = eARUPDATER_ERROR_UNKNOWN_ENUM_VALUE;
        }
        return retVal;    }

    /**
     * Returns the enum comment as a description string
     * @return The enum description
     */
    public String toString () {
        if (this.comment != null) {
            return this.comment;
        }
        return super.toString ();
    }
}
