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
 * Java copy of the eARDISCOVERY_ERROR enum
 */
public enum ARDISCOVERY_ERROR_ENUM {
   /** Dummy value for all unknown cases */
    eARDISCOVERY_ERROR_UNKNOWN_ENUM_VALUE (Integer.MIN_VALUE, "Dummy value for all unknown cases"),
   /** No error */
    ARDISCOVERY_OK (0, "No error"),
   /** Unknown generic error */
    ARDISCOVERY_ERROR (-1, "Unknown generic error"),
   /** Avahi failed to create simple poll object */
    ARDISCOVERY_ERROR_SIMPLE_POLL (-1000, "Avahi failed to create simple poll object"),
   /** Avahi failed to create simple poll object */
    ARDISCOVERY_ERROR_BUILD_NAME (-999, "Avahi failed to create simple poll object"),
   /** Avahi failed to create client */
    ARDISCOVERY_ERROR_CLIENT (-998, "Avahi failed to create client"),
   /** Failed to create config file */
    ARDISCOVERY_ERROR_CREATE_CONFIG (-997, "Failed to create config file"),
   /** Failed to delete config file */
    ARDISCOVERY_ERROR_DELETE_CONFIG (-996, "Failed to delete config file"),
   /** Avahi failed to create entry group */
    ARDISCOVERY_ERROR_ENTRY_GROUP (-995, "Avahi failed to create entry group"),
   /** Avahi failed to add service */
    ARDISCOVERY_ERROR_ADD_SERVICE (-994, "Avahi failed to add service"),
   /** Avahi failed to commit group */
    ARDISCOVERY_ERROR_GROUP_COMMIT (-993, "Avahi failed to commit group"),
   /** Avahi failed to allocate desired number of browsers */
    ARDISCOVERY_ERROR_BROWSER_ALLOC (-992, "Avahi failed to allocate desired number of browsers"),
   /** Avahi failed to create one browser */
    ARDISCOVERY_ERROR_BROWSER_NEW (-991, "Avahi failed to create one browser"),
   /** Failed to allocate connection resources */
    ARDISCOVERY_ERROR_ALLOC (-2000, "Failed to allocate connection resources"),
   /** Wrong type to connect as */
    ARDISCOVERY_ERROR_INIT (-1999, "Wrong type to connect as"),
   /** Socket creation error */
    ARDISCOVERY_ERROR_SOCKET_CREATION (-1998, "Socket creation error"),
   /** Socket access permission denied */
    ARDISCOVERY_ERROR_SOCKET_PERMISSION_DENIED (-1997, "Socket access permission denied"),
   /** Socket is already connected */
    ARDISCOVERY_ERROR_SOCKET_ALREADY_CONNECTED (-1996, "Socket is already connected"),
   /** Socket accept failed */
    ARDISCOVERY_ERROR_ACCEPT (-1995, "Socket accept failed"),
   /** Failed to write frame to socket */
    ARDISCOVERY_ERROR_SEND (-1994, "Failed to write frame to socket"),
   /** Failed to read frame from socket */
    ARDISCOVERY_ERROR_READ (-1993, "Failed to read frame from socket"),
   /** Failed to select sets */
    ARDISCOVERY_ERROR_SELECT (-1992, "Failed to select sets"),
   /** timeout error */
    ARDISCOVERY_ERROR_TIMEOUT (-1991, "timeout error"),
   /** Aborted by the user */
    ARDISCOVERY_ERROR_ABORT (-1990, "Aborted by the user"),
   /** Failed to intitialize a pipe */
    ARDISCOVERY_ERROR_PIPE_INIT (-1989, "Failed to intitialize a pipe"),
   /** Bad parameters */
    ARDISCOVERY_ERROR_BAD_PARAMETER (-1988, "Bad parameters"),
   /** discovery is busy */
    ARDISCOVERY_ERROR_BUSY (-1987, "discovery is busy"),
   /** host or net is not reachable */
    ARDISCOVERY_ERROR_SOCKET_UNREACHABLE (-1986, "host or net is not reachable"),
   /** the length of the output is to small */
    ARDISCOVERY_ERROR_OUTPUT_LENGTH (-1985, "the length of the output is to small"),
   /** JNI error */
    ARDISCOVERY_ERROR_JNI (-3000, "JNI error"),
   /** JNI virtual machine, not initialized */
    ARDISCOVERY_ERROR_JNI_VM (-2999, "JNI virtual machine, not initialized"),
   /** null JNI environment */
    ARDISCOVERY_ERROR_JNI_ENV (-2998, "null JNI environment"),
   /** null jni callback listener */
    ARDISCOVERY_ERROR_JNI_CALLBACK_LISTENER (-2997, "null jni callback listener"),
   /** Connection error */
    ARDISCOVERY_ERROR_CONNECTION (-4000, "Connection error"),
   /** Product already connected */
    ARDISCOVERY_ERROR_CONNECTION_BUSY (-3999, "Product already connected"),
   /** Product not ready to connect */
    ARDISCOVERY_ERROR_CONNECTION_NOT_READY (-3998, "Product not ready to connect"),
   /** It is not the good Product */
    ARDISCOVERY_ERROR_CONNECTION_BAD_ID (-3997, "It is not the good Product"),
   /** Device generic error */
    ARDISCOVERY_ERROR_DEVICE (-5000, "Device generic error"),
   /** The current device does not support this operation */
    ARDISCOVERY_ERROR_DEVICE_OPERATION_NOT_SUPPORTED (-4999, "The current device does not support this operation"),
   /** Json generic error */
    ARDISCOVERY_ERROR_JSON (-6000, "Json generic error"),
   /** Json parssing error */
    ARDISCOVERY_ERROR_JSON_PARSSING (-5999, "Json parssing error"),
   /** The size of the buffer storing the Json is too small */
    ARDISCOVERY_ERROR_JSON_BUFFER_SIZE (-5998, "The size of the buffer storing the Json is too small");

    private final int value;
    private final String comment;
    static HashMap<Integer, ARDISCOVERY_ERROR_ENUM> valuesList;

    ARDISCOVERY_ERROR_ENUM (int value) {
        this.value = value;
        this.comment = null;
    }

    ARDISCOVERY_ERROR_ENUM (int value, String comment) {
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
     * Gets the ARDISCOVERY_ERROR_ENUM instance from a C enum value
     * @param value C value of the enum
     * @return The ARDISCOVERY_ERROR_ENUM instance, or null if the C enum value was not valid
     */
    public static ARDISCOVERY_ERROR_ENUM getFromValue (int value) {
        if (null == valuesList) {
            ARDISCOVERY_ERROR_ENUM [] valuesArray = ARDISCOVERY_ERROR_ENUM.values ();
            valuesList = new HashMap<Integer, ARDISCOVERY_ERROR_ENUM> (valuesArray.length);
            for (ARDISCOVERY_ERROR_ENUM entry : valuesArray) {
                valuesList.put (entry.getValue (), entry);
            }
        }
        ARDISCOVERY_ERROR_ENUM retVal = valuesList.get (value);
        if (retVal == null) {
            retVal = eARDISCOVERY_ERROR_UNKNOWN_ENUM_VALUE;
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
