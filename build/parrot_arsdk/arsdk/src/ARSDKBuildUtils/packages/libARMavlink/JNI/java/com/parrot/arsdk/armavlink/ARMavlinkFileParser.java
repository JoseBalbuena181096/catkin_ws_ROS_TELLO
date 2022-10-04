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

package com.parrot.arsdk.armavlink;

import com.parrot.arsdk.arsal.ARSALPrint;

public class ARMavlinkFileParser
{
    private static String TAG = ARMavlinkFileGenerator.class.getSimpleName();

    private native long nativeNew() throws ARMavlinkException;
    private native void nativeDelete(long nativeFileParser);
    private native void nativeParse(long nativeFileParser, String filePath, long nativeMissionList);

    private long nativeFileParser = 0;

    /**
     * ARMavlinkFileGenerator constructor
     */
    public ARMavlinkFileParser () throws ARMavlinkException
    {
        nativeFileParser = nativeNew();
    }

    /**
     * Parse a Mavlink file and store it in a list
     * @param filePath the path of the file to parse
     * @return missionItemList a list of all mission items
     */
    public ARMavlinkMissionItemList parseFile(String filePath) throws ARMavlinkException
    {
        ARMavlinkMissionItemList missionList = new ARMavlinkMissionItemList();
        nativeParse(nativeFileParser, filePath, missionList.getNativePointre());
        return missionList;
    }

    /**
     * Dispose ARMavlinkFileGenerator
     */
    public void dispose() 
    {
        if(nativeFileParser != 0) {
            nativeDelete(nativeFileParser);
            nativeFileParser = 0;
        }
    }

    /**
     * Destructor
     */
    public void finalize () throws Throwable 
    {
        try {
            dispose ();
        } finally {
            super.finalize ();
        }
    }

}
