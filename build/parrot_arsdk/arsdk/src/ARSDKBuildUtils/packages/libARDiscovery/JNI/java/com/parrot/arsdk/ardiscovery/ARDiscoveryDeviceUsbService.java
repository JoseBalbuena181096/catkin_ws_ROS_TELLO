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

package com.parrot.arsdk.ardiscovery;

import android.os.Parcel;
import android.os.Parcelable;

public class ARDiscoveryDeviceUsbService implements Parcelable
{
    private static String TAG = "ARDiscoveryDeviceUsbService";

    private final String serial;

    public ARDiscoveryDeviceUsbService(String serial)
    {
        this.serial = serial;
    }

    @Override
    public boolean equals(Object other)
    {
        boolean isEqual = true;

        if ( (other == null) || !(other instanceof ARDiscoveryDeviceUsbService) )
        {
            isEqual = false;
        }
        else if (other == this)
        {
            isEqual = true;
        }
        else
        {
            /* check */
            ARDiscoveryDeviceUsbService otherDevice = (ARDiscoveryDeviceUsbService) other;

            if (!this.serial.equals(otherDevice.serial))
            {
                isEqual = false;
            }
        }

        return isEqual;
    }

    @Override
    public int hashCode() {
        return serial != null ? serial.hashCode() : 0;
    }

    public String getSerial() {
        return serial;
    }

    @Override
    public int describeContents()
    {
        return Parcelable.CONTENTS_FILE_DESCRIPTOR;
    }

    protected ARDiscoveryDeviceUsbService(Parcel source)
    {
        serial = source.readString();
    }

    @Override
    public void writeToParcel(Parcel dest, int flags)
    {
        dest.writeString(serial);
    }

    public static final Creator<ARDiscoveryDeviceUsbService> CREATOR = new Creator<ARDiscoveryDeviceUsbService>()
    {
        @Override
        public ARDiscoveryDeviceUsbService createFromParcel(Parcel source)
        {
            return new ARDiscoveryDeviceUsbService(source);
        }

        @Override
        public ARDiscoveryDeviceUsbService[] newArray(int size)
        {
            return new ARDiscoveryDeviceUsbService[size];
        }
    };
};

