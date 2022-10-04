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

public class ARMavlinkFileGenerator
{
    private static String TAG = ARMavlinkFileGenerator.class.getSimpleName();

    private native long nativeNew() throws ARMavlinkException;
    private native void nativeDelete(long nativeFileGenerator);
    private native int nativeAddMissionItem(long nativeFileGenerator, long nativeMissionItem);
    private native int nativeReplaceMissionItem(long nativeFileGenerator, long nativeMissionItem, int index);
    private native int nativeInsertMissionItem(long nativeFileGenerator, long nativeMissionItem, int index);
    private native int nativeDeleteMissionItem(long nativeFileGenerator, int index);
    private native void nativeCreateMavlinkFile(long nativeFileGenerator, String filePath);
    private native long nativeGetCurrentMissionItemList(long nativeFileGenerator) throws ARMavlinkException;

    private long nativeFileGenerator = 0;

    /**
     * ARMavlinkFileGenerator constructor
     */
    public ARMavlinkFileGenerator () throws ARMavlinkException
    {
        nativeFileGenerator = nativeNew();
    }

    public ARMAVLINK_ERROR_ENUM addMissionItemList(ARMavlinkMissionItemList missionList)
    {
        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK;

        for(int i = 0; i < missionList.getSize(); i++)
        {
            error = addMissionItem(missionList.getMissionItem(i));
            if(error != ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
                break;
        }

        return error;
    }

    public ARMAVLINK_ERROR_ENUM removeAllMissionItem()
    {
        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK;
        int size = 0;
        try 
        {
            size = GetCurrentMissionItemList().getSize();
        }
        catch (ARMavlinkException e) 
        {
            e.printStackTrace();
        }
        if(size > 0)
        {
            for(int i = 0; i < size; i++)
            {
                error = deleteMissionItem(0);
                if(error != ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
                break;
            }
        }
        return error;
    }

    /**
     * Add a {@link ARMavlinkMissionItem} to the File Generator list
     * @param missionItem the mission item to add
     * @return ARMAVLINK_OK if operation went well, the enum description of the error otherwise
     */
    public ARMAVLINK_ERROR_ENUM addMissionItem(ARMavlinkMissionItem missionItem)
    {
        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK;
        if(nativeFileGenerator != 0 && missionItem.getNativePointer() != 0)
        {
            int value = nativeAddMissionItem(nativeFileGenerator, missionItem.getNativePointer());
            error = ARMAVLINK_ERROR_ENUM.getFromValue(value);
        }
        else
        {
            error = ARMAVLINK_ERROR_ENUM.ARMAVLINK_ERROR;
        }
        return error;
    }

    /**
     * Replace a {@link ARMavlinkMissionItem} in the File Generator list
     * @param missionItem the mission item to replace with
     * @param index the index of the mission item to replace
     * @return ARMAVLINK_OK if operation went well, the enum description of the error otherwise
     */
    public ARMAVLINK_ERROR_ENUM replaceMissionItem(ARMavlinkMissionItem missionItem, int index)
    {
        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK;
        if(nativeFileGenerator != 0 &&  missionItem.getNativePointer() != 0)
        {
            int value = nativeReplaceMissionItem(nativeFileGenerator, missionItem.getNativePointer(), index);
            error = ARMAVLINK_ERROR_ENUM.getFromValue(value);
        }
        else
        {
            error = ARMAVLINK_ERROR_ENUM.ARMAVLINK_ERROR;
        }
        return error;
    }

    /**
     * Insert a {@link ARMavlinkMissionItem} in the File Generator list
     * @param missionItem the mission item to insert.
     * @param index the index of the mission item to replace
     * @return ARMAVLINK_OK if operation went well, the enum description of the error otherwise
     */
    public ARMAVLINK_ERROR_ENUM insertMissionItem(ARMavlinkMissionItem missionItem, int index)
    {
        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK;
        if(nativeFileGenerator != 0 && missionItem.getNativePointer() != 0)
        {
            int value = nativeInsertMissionItem(nativeFileGenerator, missionItem.getNativePointer(), index);
            error = ARMAVLINK_ERROR_ENUM.getFromValue(value);
        }
        else
        {
            error = ARMAVLINK_ERROR_ENUM.ARMAVLINK_ERROR;
        }
        return error;
    }

    /**
     * Delete a {@link ARMavlinkMissionItem} in the File Generator list
     * @param index the index of the mission item to delete
     * @return ARMAVLINK_OK if operation went well, the enum description of the error otherwise
     */
    public ARMAVLINK_ERROR_ENUM deleteMissionItem(int index)
    {
        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK;
        if(nativeFileGenerator != 0)
        {
            int value = nativeDeleteMissionItem(nativeFileGenerator, index);
            error = ARMAVLINK_ERROR_ENUM.getFromValue(value);
        }
        else
        {
            error = ARMAVLINK_ERROR_ENUM.ARMAVLINK_ERROR;
        }
        return error;
    }

    /**
     * Write a mavlink file with all the mission items in the list
     * @param filePath path to write the file
     */
    public void CreateMavlinkFile(String filePath)
    {
        if(nativeFileGenerator != 0 )
        {
            nativeCreateMavlinkFile(nativeFileGenerator, filePath);
        }
    }

    /**
     * Get the current mission item list
     * @return {@link ARMavlinkMissionItemList} the mission item list
    */
    public ARMavlinkMissionItemList GetCurrentMissionItemList() throws ARMavlinkException
    {
        if(nativeFileGenerator != 0 )
        {
            long itemListPtr = nativeGetCurrentMissionItemList(nativeFileGenerator);
            return new ARMavlinkMissionItemList(itemListPtr);
        }else{
            return null;
        }
    }

    /**
     * Dispose ARMavlinkFileGenerator
     */
    public void dispose() 
    {
        if(nativeFileGenerator != 0) {
            nativeDelete(nativeFileGenerator);
            nativeFileGenerator = 0;
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
