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

package com.parrot.arsdk.ardatatransfer;

import java.util.HashMap;

/**
 * Java copy of the eARDATATRANSFER_ERROR enum
 */
public enum ARDATATRANSFER_ERROR_ENUM {
   /** Dummy value for all unknown cases */
    eARDATATRANSFER_ERROR_UNKNOWN_ENUM_VALUE (Integer.MIN_VALUE, "Dummy value for all unknown cases"),
   /** No error */
    ARDATATRANSFER_OK (0, "No error"),
   /** Unknown generic error */
    ARDATATRANSFER_ERROR (-1000, "Unknown generic error"),
   /** Memory allocation error */
    ARDATATRANSFER_ERROR_ALLOC (-999, "Memory allocation error"),
   /** Bad parameters error */
    ARDATATRANSFER_ERROR_BAD_PARAMETER (-998, "Bad parameters error"),
   /** Not initialized error */
    ARDATATRANSFER_ERROR_NOT_INITIALIZED (-997, "Not initialized error"),
   /** Already initialized error */
    ARDATATRANSFER_ERROR_ALREADY_INITIALIZED (-996, "Already initialized error"),
   /** Thread already running error */
    ARDATATRANSFER_ERROR_THREAD_ALREADY_RUNNING (-995, "Thread already running error"),
   /** Thread processing error */
    ARDATATRANSFER_ERROR_THREAD_PROCESSING (-994, "Thread processing error"),
   /** Canceled received */
    ARDATATRANSFER_ERROR_CANCELED (-993, "Canceled received"),
   /** System error */
    ARDATATRANSFER_ERROR_SYSTEM (-992, "System error"),
   /** Ftp error */
    ARDATATRANSFER_ERROR_FTP (-991, "Ftp error"),
   /** File error */
    ARDATATRANSFER_ERROR_FILE (-990, "File error");

    private final int value;
    private final String comment;
    static HashMap<Integer, ARDATATRANSFER_ERROR_ENUM> valuesList;

    ARDATATRANSFER_ERROR_ENUM (int value) {
        this.value = value;
        this.comment = null;
    }

    ARDATATRANSFER_ERROR_ENUM (int value, String comment) {
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
     * Gets the ARDATATRANSFER_ERROR_ENUM instance from a C enum value
     * @param value C value of the enum
     * @return The ARDATATRANSFER_ERROR_ENUM instance, or null if the C enum value was not valid
     */
    public static ARDATATRANSFER_ERROR_ENUM getFromValue (int value) {
        if (null == valuesList) {
            ARDATATRANSFER_ERROR_ENUM [] valuesArray = ARDATATRANSFER_ERROR_ENUM.values ();
            valuesList = new HashMap<Integer, ARDATATRANSFER_ERROR_ENUM> (valuesArray.length);
            for (ARDATATRANSFER_ERROR_ENUM entry : valuesArray) {
                valuesList.put (entry.getValue (), entry);
            }
        }
        ARDATATRANSFER_ERROR_ENUM retVal = valuesList.get (value);
        if (retVal == null) {
            retVal = eARDATATRANSFER_ERROR_UNKNOWN_ENUM_VALUE;
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
