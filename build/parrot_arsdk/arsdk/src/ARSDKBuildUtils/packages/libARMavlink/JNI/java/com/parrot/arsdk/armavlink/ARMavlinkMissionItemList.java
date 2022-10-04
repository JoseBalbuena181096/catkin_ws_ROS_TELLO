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

public class ARMavlinkMissionItemList
{
    private static String TAG = ARMavlinkMissionItemList.class.getSimpleName();

    private native long nativeMissionItemListNew();
    private native void nativeMissionItemListDelete(long nativeList);
    private native int nativeMissionItemListGetSize(long nativeList);
    private native long nativeMissionItemListGet(long nativeList, int index);

    private long nativeMissionItemList = 0;

    /**
     * ARMavlinkMissionItemList constructor
     * Can't be Instantiated by user
     */
    public ARMavlinkMissionItemList ()
    {
        nativeMissionItemList = nativeMissionItemListNew();
    }

    /**
     * ARMavlinkMissionItemList constructor
     * @param itemPtr ARMavlinkMissionItemList native Pointre
     */
    public ARMavlinkMissionItemList (long itemListPtr)
    {
        if(itemListPtr != 0){
            nativeMissionItemList = itemListPtr;
        } 
    }

    public long getNativePointre()
    {
        return nativeMissionItemList;
    }

    /**
     * Get a {@link ARMavlinkMissionItem} in the list according to its index
     * @param index the index of the mission item to return
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public ARMavlinkMissionItem getMissionItem(int index)
    {
        ARMavlinkMissionItem missionItem = null;
        if(nativeMissionItemList != 0){
            long missionItemPtr = nativeMissionItemListGet(nativeMissionItemList, index);
            missionItem = new ARMavlinkMissionItem(missionItemPtr);
            missionItem.updateCommandCode();
        }
        return missionItem;
    }

    /**
     * Get the current size of the list (the number of mission item which are actually in the list)
     * @return the size of the list
     */
    public int getSize(){
        if(nativeMissionItemList != 0){
            return nativeMissionItemListGetSize(nativeMissionItemList);
        }else{
            return -1;
        }
    }

    /**
     * Dispose
     */
    public void dispose()
    {
        if(nativeMissionItemList != 0) {
            nativeMissionItemListDelete(nativeMissionItemList);
            nativeMissionItemList = 0;
        }
    }

}
