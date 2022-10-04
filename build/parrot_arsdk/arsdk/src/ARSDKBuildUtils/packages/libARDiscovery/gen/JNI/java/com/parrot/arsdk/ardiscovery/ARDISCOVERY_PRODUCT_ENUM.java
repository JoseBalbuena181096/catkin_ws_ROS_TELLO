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
 * Java copy of the eARDISCOVERY_PRODUCT enum
 */
public enum ARDISCOVERY_PRODUCT_ENUM {
   /** Dummy value for all unknown cases */
    eARDISCOVERY_PRODUCT_UNKNOWN_ENUM_VALUE (Integer.MIN_VALUE, "Dummy value for all unknown cases"),
   /** Bebop Drone product */
    ARDISCOVERY_PRODUCT_ARDRONE (0, "Bebop Drone product"),
   /** JUMPING SUMO product */
    ARDISCOVERY_PRODUCT_JS (1, "JUMPING SUMO product"),
   /** Sky controller product */
    ARDISCOVERY_PRODUCT_SKYCONTROLLER (2, "Sky controller product"),
   /** Jumping Sumo EVO Light product */
    ARDISCOVERY_PRODUCT_JS_EVO_LIGHT (3, "Jumping Sumo EVO Light product"),
   /** Jumping Sumo EVO Race product */
    ARDISCOVERY_PRODUCT_JS_EVO_RACE (4, "Jumping Sumo EVO Race product"),
   /** Bebop drone 2.0 product */
    ARDISCOVERY_PRODUCT_BEBOP_2 (5, "Bebop drone 2.0 product"),
   /** Power up product */
    ARDISCOVERY_PRODUCT_POWER_UP (6, "Power up product"),
   /** Evinrude product */
    ARDISCOVERY_PRODUCT_EVINRUDE (7, "Evinrude product"),
   /** Unknownproduct_4 product */
    ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_4 (8, "Unknownproduct_4 product"),
   /** Sky controller product (2.0 & newer versions) */
    ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG (9, "Sky controller product (2.0 & newer versions)"),
   /** Unknownproduct_5 product */
    ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_5 (10, "Unknownproduct_5 product"),
   /** Chimera product */
    ARDISCOVERY_PRODUCT_CHIMERA (11, "Chimera product"),
   /** DELOS product */
    ARDISCOVERY_PRODUCT_MINIDRONE (12, "DELOS product"),
   /** Delos EVO Light product */
    ARDISCOVERY_PRODUCT_MINIDRONE_EVO_LIGHT (13, "Delos EVO Light product"),
   /** Delos EVO Brick product */
    ARDISCOVERY_PRODUCT_MINIDRONE_EVO_BRICK (14, "Delos EVO Brick product"),
   /** Delos EVO Hydrofoil product */
    ARDISCOVERY_PRODUCT_MINIDRONE_EVO_HYDROFOIL (15, "Delos EVO Hydrofoil product"),
   /** Delos3 product */
    ARDISCOVERY_PRODUCT_MINIDRONE_DELOS3 (16, "Delos3 product"),
   /** WingX product */
    ARDISCOVERY_PRODUCT_MINIDRONE_WINGX (17, "WingX product"),
   /** Sky controller 2 product */
    ARDISCOVERY_PRODUCT_SKYCONTROLLER_2 (18, "Sky controller 2 product"),
   /** Sky controller 2P product */
    ARDISCOVERY_PRODUCT_SKYCONTROLLER_2P (19, "Sky controller 2P product"),
   /** Tinos product */
    ARDISCOVERY_PRODUCT_TINOS (20, "Tinos product"),
   /** Sequoia product */
    ARDISCOVERY_PRODUCT_SEQUOIA (21, "Sequoia product"),
   /** Max of products */
    ARDISCOVERY_PRODUCT_MAX (22, "Max of products");

    private final int value;
    private final String comment;
    static HashMap<Integer, ARDISCOVERY_PRODUCT_ENUM> valuesList;

    ARDISCOVERY_PRODUCT_ENUM (int value) {
        this.value = value;
        this.comment = null;
    }

    ARDISCOVERY_PRODUCT_ENUM (int value, String comment) {
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
     * Gets the ARDISCOVERY_PRODUCT_ENUM instance from a C enum value
     * @param value C value of the enum
     * @return The ARDISCOVERY_PRODUCT_ENUM instance, or null if the C enum value was not valid
     */
    public static ARDISCOVERY_PRODUCT_ENUM getFromValue (int value) {
        if (null == valuesList) {
            ARDISCOVERY_PRODUCT_ENUM [] valuesArray = ARDISCOVERY_PRODUCT_ENUM.values ();
            valuesList = new HashMap<Integer, ARDISCOVERY_PRODUCT_ENUM> (valuesArray.length);
            for (ARDISCOVERY_PRODUCT_ENUM entry : valuesArray) {
                valuesList.put (entry.getValue (), entry);
            }
        }
        ARDISCOVERY_PRODUCT_ENUM retVal = valuesList.get (value);
        if (retVal == null) {
            retVal = eARDISCOVERY_PRODUCT_UNKNOWN_ENUM_VALUE;
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
