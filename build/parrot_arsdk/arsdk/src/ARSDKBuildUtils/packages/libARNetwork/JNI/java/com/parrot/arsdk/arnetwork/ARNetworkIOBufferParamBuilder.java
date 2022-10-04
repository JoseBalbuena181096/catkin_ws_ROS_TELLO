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
package com.parrot.arsdk.arnetwork;

import com.parrot.arsdk.arnetworkal.ARNETWORKAL_FRAME_TYPE_ENUM;
import com.parrot.arsdk.arsal.ARSALPrint;

public class ARNetworkIOBufferParamBuilder {

    private static final String TAG = ARNetworkIOBufferParamBuilder.class.getSimpleName();

    /** Identifier of the buffer */
    private int ID;
    /** Type of the buffer */
    private ARNETWORKAL_FRAME_TYPE_ENUM dataType;
    /** Minimum time (ms) between two send */
    private int timeBetweenSendMs;
    /** Timeout (ms) of acknowledges */
    private int ackTimeoutMs;
    /** Maximum number or retries (only for acknowledged data) */
    private int numberOfRetry;
    /** Capacity of the buffer */
    private int numberOfCell;
    /** Maximum size (bytes) of a data to copy */
    private int dataCopyMaxSize;
    /** Overwriting flag */
    private boolean isOverwriting;

    public ARNetworkIOBufferParamBuilder (int id) {
        this.ID = id;
        this.dataType = ARNETWORKAL_FRAME_TYPE_ENUM.ARNETWORKAL_FRAME_TYPE_DATA;
        this.timeBetweenSendMs = 1;
        this.ackTimeoutMs = ARNetworkIOBufferParam.ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER;
        this.numberOfRetry = ARNetworkIOBufferParam.ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER;
        this.numberOfCell = 1;
        this.dataCopyMaxSize = 0;
        this.isOverwriting = false;
    }

    public ARNetworkIOBufferParamBuilder setDataType (ARNETWORKAL_FRAME_TYPE_ENUM type) {
        this.dataType = type;
        return this;
    }

    public ARNetworkIOBufferParamBuilder setTimeBetweenSendMs (int time) {
        this.timeBetweenSendMs = time;
        return this;
    }

    public ARNetworkIOBufferParamBuilder setAckTimeoutMs (int time) {
        this.ackTimeoutMs = time;
        return this;
    }

    public ARNetworkIOBufferParamBuilder setNumberOfRetry (int number) {
        this.numberOfRetry = number;
        return this;
    }

    public ARNetworkIOBufferParamBuilder setNumberOfCell (int number) {
        this.numberOfCell = number;
        return this;
    }

    public ARNetworkIOBufferParamBuilder setDataCopyMaxSize (int size) {
        this.dataCopyMaxSize = size;
        return this;
    }

    public ARNetworkIOBufferParamBuilder setOverwriting (boolean enable) {
        this.isOverwriting = enable;
        return this;
    }

    public String toString () {
        return "{\n" +
            "    ID = " + ID + "\n" +
            "    dataType = " + dataType + "\n" +
            "    timeBetweenSendMs = " + timeBetweenSendMs + "\n" +
            "    ackTimeoutMs = " + ackTimeoutMs + "\n" +
            "    numberOfRetry = " + numberOfRetry + "\n" +
            "    numberOfCell = " + numberOfCell + "\n" +
            "    dataCopyMaxSize = " + dataCopyMaxSize + "\n" +
            "    isOverwriting = " + isOverwriting + "\n" +
            "}";
    }

    public ARNetworkIOBufferParam build () {
        return new ARNetworkIOBufferParam (ID,
                                           dataType,
                                           timeBetweenSendMs,
                                           ackTimeoutMs,
                                           numberOfRetry,
                                           numberOfCell,
                                           dataCopyMaxSize,
                                           isOverwriting);
    }
}
