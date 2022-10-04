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

package com.parrot.arsdk.arnetworkal;

import java.util.HashMap;

/**
 * Java copy of the eARNETWORKAL_MANAGER_RETURN enum
 */
public enum ARNETWORKAL_MANAGER_RETURN_ENUM {
   /** Dummy value for all unknown cases */
    eARNETWORKAL_MANAGER_RETURN_UNKNOWN_ENUM_VALUE (Integer.MIN_VALUE, "Dummy value for all unknown cases"),
   /** Default return value (no error) */
    ARNETWORKAL_MANAGER_RETURN_DEFAULT (0, "Default return value (no error)"),
   /** Impossible to push a frame : network buffer is full */
    ARNETWORKAL_MANAGER_RETURN_BUFFER_FULL (1, "Impossible to push a frame : network buffer is full"),
   /** Impossible to pop a frame, no frame in the buffer */
    ARNETWORKAL_MANAGER_RETURN_BUFFER_EMPTY (2, "Impossible to pop a frame, no frame in the buffer"),
   /** Impossible to pop a frame, frame is corrupted */
    ARNETWORKAL_MANAGER_RETURN_BAD_FRAME (3, "Impossible to pop a frame, frame is corrupted"),
   /** Impossible to read data from the network, no data available */
    ARNETWORKAL_MANAGER_RETURN_NO_DATA_AVAILABLE (4, "Impossible to read data from the network, no data available"),
   /** Parameters given to the callback were not good */
    ARNETWORKAL_MANAGER_RETURN_BAD_PARAMETERS (5, "Parameters given to the callback were not good"),
   /** Network error while reading or sending data */
    ARNETWORKAL_MANAGER_RETURN_NETWORK_ERROR (6, "Network error while reading or sending data");

    private final int value;
    private final String comment;
    static HashMap<Integer, ARNETWORKAL_MANAGER_RETURN_ENUM> valuesList;

    ARNETWORKAL_MANAGER_RETURN_ENUM (int value) {
        this.value = value;
        this.comment = null;
    }

    ARNETWORKAL_MANAGER_RETURN_ENUM (int value, String comment) {
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
     * Gets the ARNETWORKAL_MANAGER_RETURN_ENUM instance from a C enum value
     * @param value C value of the enum
     * @return The ARNETWORKAL_MANAGER_RETURN_ENUM instance, or null if the C enum value was not valid
     */
    public static ARNETWORKAL_MANAGER_RETURN_ENUM getFromValue (int value) {
        if (null == valuesList) {
            ARNETWORKAL_MANAGER_RETURN_ENUM [] valuesArray = ARNETWORKAL_MANAGER_RETURN_ENUM.values ();
            valuesList = new HashMap<Integer, ARNETWORKAL_MANAGER_RETURN_ENUM> (valuesArray.length);
            for (ARNETWORKAL_MANAGER_RETURN_ENUM entry : valuesArray) {
                valuesList.put (entry.getValue (), entry);
            }
        }
        ARNETWORKAL_MANAGER_RETURN_ENUM retVal = valuesList.get (value);
        if (retVal == null) {
            retVal = eARNETWORKAL_MANAGER_RETURN_UNKNOWN_ENUM_VALUE;
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
