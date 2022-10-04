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

package com.parrot.arsdk.arnetwork;

import java.util.HashMap;

/**
 * Java copy of the eARNETWORK_ERROR enum
 */
public enum ARNETWORK_ERROR_ENUM {
   /** Dummy value for all unknown cases */
    eARNETWORK_ERROR_UNKNOWN_ENUM_VALUE (Integer.MIN_VALUE, "Dummy value for all unknown cases"),
   /** No error */
    ARNETWORK_OK (0, "No error"),
   /** Unknown generic error */
    ARNETWORK_ERROR (-1000, "Unknown generic error"),
   /** Memory allocation error */
    ARNETWORK_ERROR_ALLOC (-999, "Memory allocation error"),
   /** Bad parameters */
    ARNETWORK_ERROR_BAD_PARAMETER (-998, "Bad parameters"),
   /** Given IOBuffer identifier is unknown */
    ARNETWORK_ERROR_ID_UNKNOWN (-997, "Given IOBuffer identifier is unknown"),
   /** Insufficient free space in the buffer */
    ARNETWORK_ERROR_BUFFER_SIZE (-996, "Insufficient free space in the buffer"),
   /** Buffer is empty, nothing was read */
    ARNETWORK_ERROR_BUFFER_EMPTY (-995, "Buffer is empty, nothing was read"),
   /** Error when using a semaphore */
    ARNETWORK_ERROR_SEMAPHORE (-994, "Error when using a semaphore"),
   /** Error when using a mutex */
    ARNETWORK_ERROR_MUTEX (-993, "Error when using a mutex"),
   /** A mutex is already locked by the same thread */
    ARNETWORK_ERROR_MUTEX_DOUBLE_LOCK (-992, "A mutex is already locked by the same thread"),
   /** Unknown ARNETWORK_Manager error */
    ARNETWORK_ERROR_MANAGER (-2000, "Unknown ARNETWORK_Manager error"),
   /** IOBuffer creation error */
    ARNETWORK_ERROR_MANAGER_NEW_IOBUFFER (-1999, "IOBuffer creation error"),
   /** Sender creation error */
    ARNETWORK_ERROR_MANAGER_NEW_SENDER (-1998, "Sender creation error"),
   /** Receiver creation error */
    ARNETWORK_ERROR_MANAGER_NEW_RECEIVER (-1997, "Receiver creation error"),
   /** Buffer creation error */
    ARNETWORK_ERROR_NEW_BUFFER (-1996, "Buffer creation error"),
   /** RingBuffer creation error */
    ARNETWORK_ERROR_NEW_RINGBUFFER (-1995, "RingBuffer creation error"),
   /** Unknown IOBuffer error */
    ARNETWORK_ERROR_IOBUFFER (-3000, "Unknown IOBuffer error"),
   /** Bad sequence number for the acknowledge */
    ARNETWORK_ERROR_IOBUFFER_BAD_ACK (-2999, "Bad sequence number for the acknowledge"),
   /** Unknown Receiver error */
    ARNETWORK_ERROR_RECEIVER (-5000, "Unknown Receiver error"),
   /** Receiver buffer too small */
    ARNETWORK_ERROR_RECEIVER_BUFFER_END (-4999, "Receiver buffer too small"),
   /** Bad frame content on network */
    ARNETWORK_ERROR_RECEIVER_BAD_FRAME (-4998, "Bad frame content on network");

    private final int value;
    private final String comment;
    static HashMap<Integer, ARNETWORK_ERROR_ENUM> valuesList;

    ARNETWORK_ERROR_ENUM (int value) {
        this.value = value;
        this.comment = null;
    }

    ARNETWORK_ERROR_ENUM (int value, String comment) {
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
     * Gets the ARNETWORK_ERROR_ENUM instance from a C enum value
     * @param value C value of the enum
     * @return The ARNETWORK_ERROR_ENUM instance, or null if the C enum value was not valid
     */
    public static ARNETWORK_ERROR_ENUM getFromValue (int value) {
        if (null == valuesList) {
            ARNETWORK_ERROR_ENUM [] valuesArray = ARNETWORK_ERROR_ENUM.values ();
            valuesList = new HashMap<Integer, ARNETWORK_ERROR_ENUM> (valuesArray.length);
            for (ARNETWORK_ERROR_ENUM entry : valuesArray) {
                valuesList.put (entry.getValue (), entry);
            }
        }
        ARNETWORK_ERROR_ENUM retVal = valuesList.get (value);
        if (retVal == null) {
            retVal = eARNETWORK_ERROR_UNKNOWN_ENUM_VALUE;
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
