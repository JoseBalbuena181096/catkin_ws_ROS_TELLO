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

package com.parrot.arsdk.arsal;

import java.util.HashMap;

/**
 * Java copy of the eARSAL_ERROR enum
 */
public enum ARSAL_ERROR_ENUM {
   /** Dummy value for all unknown cases */
    eARSAL_ERROR_UNKNOWN_ENUM_VALUE (Integer.MIN_VALUE, "Dummy value for all unknown cases"),
   /** No error */
    ARSAL_OK (0, "No error"),
   /** ARSAL Generic error */
    ARSAL_ERROR (-1000, "ARSAL Generic error"),
   /** ARSAL alloc error */
    ARSAL_ERROR_ALLOC (-999, "ARSAL alloc error"),
   /** ARSAL system error */
    ARSAL_ERROR_SYSTEM (-998, "ARSAL system error"),
   /** ARSAL bad parameter error */
    ARSAL_ERROR_BAD_PARAMETER (-997, "ARSAL bad parameter error"),
   /** ARSAL file error */
    ARSAL_ERROR_FILE (-996, "ARSAL file error"),
   /** ARSAL md5 error */
    ARSAL_ERROR_MD5 (-2000, "ARSAL md5 error"),
   /** BLE connection generic error */
    ARSAL_ERROR_BLE_CONNECTION (-5000, "BLE connection generic error"),
   /** BLE is not connected */
    ARSAL_ERROR_BLE_NOT_CONNECTED (-4999, "BLE is not connected"),
   /** BLE disconnection error */
    ARSAL_ERROR_BLE_DISCONNECTION (-4998, "BLE disconnection error"),
   /** BLE network services discovering error */
    ARSAL_ERROR_BLE_SERVICES_DISCOVERING (-4997, "BLE network services discovering error"),
   /** BLE network characteristics discovering error */
    ARSAL_ERROR_BLE_CHARACTERISTICS_DISCOVERING (-4996, "BLE network characteristics discovering error"),
   /** BLE network characteristic configuring error */
    ARSAL_ERROR_BLE_CHARACTERISTIC_CONFIGURING (-4995, "BLE network characteristic configuring error"),
   /** BLE stack generic error */
    ARSAL_ERROR_BLE_STACK (-4994, "BLE stack generic error"),
   /** BLE timeout */
    ARSAL_ERROR_BLE_TIMEOUT (-4993, "BLE timeout"),
   /** BLE no data */
    ARSAL_ERROR_BLE_NO_DATA (-4992, "BLE no data");

    private final int value;
    private final String comment;
    static HashMap<Integer, ARSAL_ERROR_ENUM> valuesList;

    ARSAL_ERROR_ENUM (int value) {
        this.value = value;
        this.comment = null;
    }

    ARSAL_ERROR_ENUM (int value, String comment) {
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
     * Gets the ARSAL_ERROR_ENUM instance from a C enum value
     * @param value C value of the enum
     * @return The ARSAL_ERROR_ENUM instance, or null if the C enum value was not valid
     */
    public static ARSAL_ERROR_ENUM getFromValue (int value) {
        if (null == valuesList) {
            ARSAL_ERROR_ENUM [] valuesArray = ARSAL_ERROR_ENUM.values ();
            valuesList = new HashMap<Integer, ARSAL_ERROR_ENUM> (valuesArray.length);
            for (ARSAL_ERROR_ENUM entry : valuesArray) {
                valuesList.put (entry.getValue (), entry);
            }
        }
        ARSAL_ERROR_ENUM retVal = valuesList.get (value);
        if (retVal == null) {
            retVal = eARSAL_ERROR_UNKNOWN_ENUM_VALUE;
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
