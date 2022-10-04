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

package com.parrot.arsdk.ardiscovery;

import java.util.HashMap;

/**
 * Java copy of the eARDISCOVERY_NETWORK_TYPE enum
 */
public enum ARDISCOVERY_NETWORK_TYPE_ENUM {
   /** Dummy value for all unknown cases */
    eARDISCOVERY_NETWORK_TYPE_UNKNOWN_ENUM_VALUE (Integer.MIN_VALUE, "Dummy value for all unknown cases"),
   /** unknown network */
    ARDISCOVERY_NETWORK_TYPE_UNKNOWN (0, "unknown network"),
   /** IP (e.g. wifi) network */
    ARDISCOVERY_NETWORK_TYPE_NET (1, "IP (e.g. wifi) network"),
   /** BLE network */
    ARDISCOVERY_NETWORK_TYPE_BLE (2, "BLE network"),
   /** libmux over USB network */
    ARDISCOVERY_NETWORK_TYPE_USBMUX (3, "libmux over USB network");

    private final int value;
    private final String comment;
    static HashMap<Integer, ARDISCOVERY_NETWORK_TYPE_ENUM> valuesList;

    ARDISCOVERY_NETWORK_TYPE_ENUM (int value) {
        this.value = value;
        this.comment = null;
    }

    ARDISCOVERY_NETWORK_TYPE_ENUM (int value, String comment) {
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
     * Gets the ARDISCOVERY_NETWORK_TYPE_ENUM instance from a C enum value
     * @param value C value of the enum
     * @return The ARDISCOVERY_NETWORK_TYPE_ENUM instance, or null if the C enum value was not valid
     */
    public static ARDISCOVERY_NETWORK_TYPE_ENUM getFromValue (int value) {
        if (null == valuesList) {
            ARDISCOVERY_NETWORK_TYPE_ENUM [] valuesArray = ARDISCOVERY_NETWORK_TYPE_ENUM.values ();
            valuesList = new HashMap<Integer, ARDISCOVERY_NETWORK_TYPE_ENUM> (valuesArray.length);
            for (ARDISCOVERY_NETWORK_TYPE_ENUM entry : valuesArray) {
                valuesList.put (entry.getValue (), entry);
            }
        }
        ARDISCOVERY_NETWORK_TYPE_ENUM retVal = valuesList.get (value);
        if (retVal == null) {
            retVal = eARDISCOVERY_NETWORK_TYPE_UNKNOWN_ENUM_VALUE;
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
