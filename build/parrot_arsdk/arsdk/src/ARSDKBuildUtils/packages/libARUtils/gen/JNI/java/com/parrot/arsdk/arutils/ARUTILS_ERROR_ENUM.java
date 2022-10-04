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

package com.parrot.arsdk.arutils;

import java.util.HashMap;

/**
 * Java copy of the eARUTILS_ERROR enum
 */
public enum ARUTILS_ERROR_ENUM {
   /** Dummy value for all unknown cases */
    eARUTILS_ERROR_UNKNOWN_ENUM_VALUE (Integer.MIN_VALUE, "Dummy value for all unknown cases"),
   /** No error */
    ARUTILS_OK (0, "No error"),
   /** Unknown generic error */
    ARUTILS_ERROR (-1000, "Unknown generic error"),
   /** Memory allocation error */
    ARUTILS_ERROR_ALLOC (-999, "Memory allocation error"),
   /** Bad parameters error */
    ARUTILS_ERROR_BAD_PARAMETER (-998, "Bad parameters error"),
   /** System error */
    ARUTILS_ERROR_SYSTEM (-997, "System error"),
   /** Function not implemented */
    ARUTILS_ERROR_NOT_IMPLEMENTED (-996, "Function not implemented"),
   /** curl allocation error */
    ARUTILS_ERROR_CURL_ALLOC (-2000, "curl allocation error"),
   /** curl set option error */
    ARUTILS_ERROR_CURL_SETOPT (-1999, "curl set option error"),
   /** curl get info error */
    ARUTILS_ERROR_CURL_GETINFO (-1998, "curl get info error"),
   /** curl perform error */
    ARUTILS_ERROR_CURL_PERFORM (-1997, "curl perform error"),
   /** file not found error */
    ARUTILS_ERROR_FILE_NOT_FOUND (-3000, "file not found error"),
   /** ftp connect error */
    ARUTILS_ERROR_FTP_CONNECT (-4000, "ftp connect error"),
   /** ftp code error */
    ARUTILS_ERROR_FTP_CODE (-3999, "ftp code error"),
   /** ftp file size error */
    ARUTILS_ERROR_FTP_SIZE (-3998, "ftp file size error"),
   /** ftp resume error */
    ARUTILS_ERROR_FTP_RESUME (-3997, "ftp resume error"),
   /** ftp user canceled error */
    ARUTILS_ERROR_FTP_CANCELED (-3996, "ftp user canceled error"),
   /** ftp file error */
    ARUTILS_ERROR_FTP_FILE (-3995, "ftp file error"),
   /** ftp md5 error */
    ARUTILS_ERROR_FTP_MD5 (-3994, "ftp md5 error"),
   /** http connect error */
    ARUTILS_ERROR_HTTP_CONNECT (-5000, "http connect error"),
   /** http code error */
    ARUTILS_ERROR_HTTP_CODE (-4999, "http code error"),
   /** http authorization required */
    ARUTILS_ERROR_HTTP_AUTHORIZATION_REQUIRED (-4998, "http authorization required"),
   /** http access denied */
    ARUTILS_ERROR_HTTP_ACCESS_DENIED (-4997, "http access denied"),
   /** http file size error */
    ARUTILS_ERROR_HTTP_SIZE (-4996, "http file size error"),
   /** http resume error */
    ARUTILS_ERROR_HTTP_RESUME (-4995, "http resume error"),
   /** http user canceled error */
    ARUTILS_ERROR_HTTP_CANCELED (-4994, "http user canceled error"),
   /** BLE ftp failed error */
    ARUTILS_ERROR_BLE_FAILED (-6000, "BLE ftp failed error"),
   /** Network type, not available for the platform error */
    ARUTILS_ERROR_NETWORK_TYPE (-7000, "Network type, not available for the platform error"),
   /** RFComm ftp failed error */
    ARUTILS_ERROR_RFCOMM_FAILED (-8000, "RFComm ftp failed error");

    private final int value;
    private final String comment;
    static HashMap<Integer, ARUTILS_ERROR_ENUM> valuesList;

    ARUTILS_ERROR_ENUM (int value) {
        this.value = value;
        this.comment = null;
    }

    ARUTILS_ERROR_ENUM (int value, String comment) {
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
     * Gets the ARUTILS_ERROR_ENUM instance from a C enum value
     * @param value C value of the enum
     * @return The ARUTILS_ERROR_ENUM instance, or null if the C enum value was not valid
     */
    public static ARUTILS_ERROR_ENUM getFromValue (int value) {
        if (null == valuesList) {
            ARUTILS_ERROR_ENUM [] valuesArray = ARUTILS_ERROR_ENUM.values ();
            valuesList = new HashMap<Integer, ARUTILS_ERROR_ENUM> (valuesArray.length);
            for (ARUTILS_ERROR_ENUM entry : valuesArray) {
                valuesList.put (entry.getValue (), entry);
            }
        }
        ARUTILS_ERROR_ENUM retVal = valuesList.get (value);
        if (retVal == null) {
            retVal = eARUTILS_ERROR_UNKNOWN_ENUM_VALUE;
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
