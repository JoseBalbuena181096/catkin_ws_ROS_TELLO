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

package com.parrot.arsdk.armavlink;

import java.util.HashMap;

/**
 * Java copy of the eARMAVLINK_ERROR enum
 */
public enum ARMAVLINK_ERROR_ENUM {
   /** Dummy value for all unknown cases */
    eARMAVLINK_ERROR_UNKNOWN_ENUM_VALUE (Integer.MIN_VALUE, "Dummy value for all unknown cases"),
   /** No error */
    ARMAVLINK_OK (0, "No error"),
   /** Unknown generic error */
    ARMAVLINK_ERROR (-1000, "Unknown generic error"),
   /** Memory allocation error */
    ARMAVLINK_ERROR_ALLOC (-999, "Memory allocation error"),
   /** Bad parameters */
    ARMAVLINK_ERROR_BAD_PARAMETER (-998, "Bad parameters"),
   /** Unknown ARMAVLINK_Manager error */
    ARMAVLINK_ERROR_MANAGER (-2000, "Unknown ARMAVLINK_Manager error"),
   /** Unknown ARMAVLINK_FileGenerator error */
    ARMAVLINK_ERROR_FILE_GENERATOR (-3000, "Unknown ARMAVLINK_FileGenerator error"),
   /** Unknown ARMAVLINK_ListUtils error */
    ARMAVLINK_ERROR_LIST_UTILS (-4000, "Unknown ARMAVLINK_ListUtils error"),
   /** Unknown ARMAVLINK_MissionItemUtils error */
    ARMAVLINK_ERROR_MISSION_ITEM_UTILS (-5000, "Unknown ARMAVLINK_MissionItemUtils error"),
   /** Command not linked with Mavlink commands */
    ARMAVLINK_ERROR_MISSION_ITEM_UTILS_NOT_LINKED_COMMAND (-4999, "Command not linked with Mavlink commands"),
   /** Unknown ARMAVLINK_FileParser error */
    ARMAVLINK_ERROR_FILE_PARSER (-6000, "Unknown ARMAVLINK_FileParser error"),
   /** File to parse not found */
    ARMAVLINK_ERROR_FILE_PARSER_FILE_NOT_FOUND (-5999, "File to parse not found"),
   /** A word was not expected during parsing */
    ARMAVLINK_ERROR_FILE_PARSER_WORD_NOT_EXPTECTED (-5998, "A word was not expected during parsing");

    private final int value;
    private final String comment;
    static HashMap<Integer, ARMAVLINK_ERROR_ENUM> valuesList;

    ARMAVLINK_ERROR_ENUM (int value) {
        this.value = value;
        this.comment = null;
    }

    ARMAVLINK_ERROR_ENUM (int value, String comment) {
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
     * Gets the ARMAVLINK_ERROR_ENUM instance from a C enum value
     * @param value C value of the enum
     * @return The ARMAVLINK_ERROR_ENUM instance, or null if the C enum value was not valid
     */
    public static ARMAVLINK_ERROR_ENUM getFromValue (int value) {
        if (null == valuesList) {
            ARMAVLINK_ERROR_ENUM [] valuesArray = ARMAVLINK_ERROR_ENUM.values ();
            valuesList = new HashMap<Integer, ARMAVLINK_ERROR_ENUM> (valuesArray.length);
            for (ARMAVLINK_ERROR_ENUM entry : valuesArray) {
                valuesList.put (entry.getValue (), entry);
            }
        }
        ARMAVLINK_ERROR_ENUM retVal = valuesList.get (value);
        if (retVal == null) {
            retVal = eARMAVLINK_ERROR_UNKNOWN_ENUM_VALUE;
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
