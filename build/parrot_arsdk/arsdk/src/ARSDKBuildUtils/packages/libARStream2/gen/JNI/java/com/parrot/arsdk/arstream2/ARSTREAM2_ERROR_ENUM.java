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

package com.parrot.arsdk.arstream2;

import java.util.HashMap;

/**
 * Java copy of the eARSTREAM2_ERROR enum
 */
public enum ARSTREAM2_ERROR_ENUM {
   /** Dummy value for all unknown cases */
    eARSTREAM2_ERROR_UNKNOWN_ENUM_VALUE (Integer.MIN_VALUE, "Dummy value for all unknown cases"),
   /** No error */
    ARSTREAM2_OK (0, "No error"),
   /** Bad parameters */
    ARSTREAM2_ERROR_BAD_PARAMETERS (-1, "Bad parameters"),
   /** Unable to allocate resource */
    ARSTREAM2_ERROR_ALLOC (-2, "Unable to allocate resource"),
   /** Object is busy and can not be deleted yet */
    ARSTREAM2_ERROR_BUSY (-3, "Object is busy and can not be deleted yet"),
   /** Queue is full */
    ARSTREAM2_ERROR_QUEUE_FULL (-4, "Queue is full"),
   /** Waiting for synchronization */
    ARSTREAM2_ERROR_WAITING_FOR_SYNC (-5, "Waiting for synchronization"),
   /** Re-synchronization required */
    ARSTREAM2_ERROR_RESYNC_REQUIRED (-6, "Re-synchronization required"),
   /** Resource unavailable */
    ARSTREAM2_ERROR_RESOURCE_UNAVAILABLE (-7, "Resource unavailable"),
   /** Not found */
    ARSTREAM2_ERROR_NOT_FOUND (-8, "Not found"),
   /** Invalid state */
    ARSTREAM2_ERROR_INVALID_STATE (-9, "Invalid state"),
   /** Unsupported */
    ARSTREAM2_ERROR_UNSUPPORTED (-10, "Unsupported");

    private final int value;
    private final String comment;
    static HashMap<Integer, ARSTREAM2_ERROR_ENUM> valuesList;

    ARSTREAM2_ERROR_ENUM (int value) {
        this.value = value;
        this.comment = null;
    }

    ARSTREAM2_ERROR_ENUM (int value, String comment) {
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
     * Gets the ARSTREAM2_ERROR_ENUM instance from a C enum value
     * @param value C value of the enum
     * @return The ARSTREAM2_ERROR_ENUM instance, or null if the C enum value was not valid
     */
    public static ARSTREAM2_ERROR_ENUM getFromValue (int value) {
        if (null == valuesList) {
            ARSTREAM2_ERROR_ENUM [] valuesArray = ARSTREAM2_ERROR_ENUM.values ();
            valuesList = new HashMap<Integer, ARSTREAM2_ERROR_ENUM> (valuesArray.length);
            for (ARSTREAM2_ERROR_ENUM entry : valuesArray) {
                valuesList.put (entry.getValue (), entry);
            }
        }
        ARSTREAM2_ERROR_ENUM retVal = valuesList.get (value);
        if (retVal == null) {
            retVal = eARSTREAM2_ERROR_UNKNOWN_ENUM_VALUE;
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
