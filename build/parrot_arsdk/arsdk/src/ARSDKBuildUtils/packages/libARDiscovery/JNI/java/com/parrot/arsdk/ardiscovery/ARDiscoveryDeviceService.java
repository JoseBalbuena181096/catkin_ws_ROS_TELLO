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
 * ARDiscoveryService.java
 *
 *  Created on: 
 *  Author:
 */

package com.parrot.arsdk.ardiscovery;

import android.os.Parcel;
import android.os.Parcelable;

import com.parrot.arsdk.arsal.ARSALPrint;

public class ARDiscoveryDeviceService implements Parcelable
{
    /**
     * 
     */

    private static String TAG = "ARDiscoveryDevice";
    
    private String name; /**< Name of the device */
    private int productID; /**< Specific product ID */
    private ARDISCOVERY_NETWORK_TYPE_ENUM networkType;
    private Object device; /**< can by ARDiscoveryDeviceNetService or ARDiscoveryDeviceBLEService or ARDiscoveryDeviceUsbService*/
    
    public static final Parcelable.Creator<ARDiscoveryDeviceService> CREATOR = new Parcelable.Creator<ARDiscoveryDeviceService>()
    {
        @Override
        public ARDiscoveryDeviceService createFromParcel(Parcel source)
        {
            return new ARDiscoveryDeviceService(source);
        }

        @Override
        public ARDiscoveryDeviceService[] newArray(int size)
        {
            return new ARDiscoveryDeviceService[size];
        }
    };
    
    public ARDiscoveryDeviceService ()
    {
        name = "";
        setDevice(null);
        productID = 0;
        networkType = ARDISCOVERY_NETWORK_TYPE_ENUM.ARDISCOVERY_NETWORK_TYPE_UNKNOWN;
    }
    
    public ARDiscoveryDeviceService (String name, Object device, int productID)
    {
        this.name = name;
        this.setDevice(device);
        this.productID = productID;
    }
    
    // Parcelling part
    public ARDiscoveryDeviceService(Parcel in)
    {
        this.name = in.readString();
        this.productID = in.readInt();
        this.networkType = ARDISCOVERY_NETWORK_TYPE_ENUM.getFromValue(in.readInt());

        switch(this.networkType)
        {
            case ARDISCOVERY_NETWORK_TYPE_NET:
                this.device = in.readParcelable(ARDiscoveryDeviceNetService.class.getClassLoader());
                break;
            case ARDISCOVERY_NETWORK_TYPE_BLE:
                this.device = in.readParcelable(ARDiscoveryDeviceBLEService.class.getClassLoader());
                break;
            case ARDISCOVERY_NETWORK_TYPE_USBMUX:
                this.device = in.readParcelable(ARDiscoveryDeviceUsbService.class.getClassLoader());
                break;
            case ARDISCOVERY_NETWORK_TYPE_UNKNOWN:
            default:
                this.device = null;
                break;
        }
    }
    
    @Override
    public boolean equals(Object other) 
    {
        boolean isEqual = true;
            
        if ( (other == null) || !(other instanceof ARDiscoveryDeviceService) )
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
            ARDiscoveryDeviceService otherDevice = (ARDiscoveryDeviceService) other;
            
            if (this.getDevice() != otherDevice.getDevice())
            {
                if((this.getDevice() != null) && (otherDevice.getDevice() != null))
                {
                    /* check if the devices are of same class */
                    if (this.networkType == otherDevice.networkType)
                    {
                        switch (this.networkType) {
                            case ARDISCOVERY_NETWORK_TYPE_NET:
                                /* if it is a NetDevice */
                                ARDiscoveryDeviceNetService deviceNetService = (ARDiscoveryDeviceNetService) this.getDevice();
                                ARDiscoveryDeviceNetService otherDeviceNetService = (ARDiscoveryDeviceNetService) otherDevice.getDevice();

                                if (!deviceNetService.equals(otherDeviceNetService)) {
                                    isEqual = false;
                                }
                                break;
                            case ARDISCOVERY_NETWORK_TYPE_BLE:
                            /* if it is a BLEDevice */
                                ARDiscoveryDeviceBLEService deviceBLEService = (ARDiscoveryDeviceBLEService) this.getDevice();
                                ARDiscoveryDeviceBLEService otherDeviceBLEService = (ARDiscoveryDeviceBLEService) otherDevice.getDevice();

                                if (!deviceBLEService.equals(otherDeviceBLEService)) {
                                    isEqual = false;
                                }
                                break;
                            case ARDISCOVERY_NETWORK_TYPE_USBMUX:
                            /* if it is a BLEDevice */
                                ARDiscoveryDeviceUsbService deviceUsbService = (ARDiscoveryDeviceUsbService) this.getDevice();
                                ARDiscoveryDeviceUsbService otherDeviceUsbService = (ARDiscoveryDeviceUsbService) otherDevice.getDevice();

                                if (!deviceUsbService.equals(otherDeviceUsbService)) {
                                    isEqual = false;
                                }
                                break;
                        }

                    }
                    else
                    {
                        isEqual = false;
                    }
                }
                else
                {
                    isEqual = false;
                }
            }
            else
            {
                isEqual = false;
            }
        }
        
        return isEqual;
    }
    
    public String getName ()
    {
        return name;
    }
    
    public void setName (String name)
    {
        this.name = name;
    }

    public Object getDevice ()
    {
        return device;
    }

    public void setDevice (Object device)
    {
        this.device = device;
        if (device instanceof ARDiscoveryDeviceNetService) {
            this.networkType = ARDISCOVERY_NETWORK_TYPE_ENUM.ARDISCOVERY_NETWORK_TYPE_NET;
        } else if (device instanceof ARDiscoveryDeviceBLEService) {
            this.networkType = ARDISCOVERY_NETWORK_TYPE_ENUM.ARDISCOVERY_NETWORK_TYPE_BLE;
        } else if (device instanceof ARDiscoveryDeviceUsbService) {
            this.networkType = ARDISCOVERY_NETWORK_TYPE_ENUM.ARDISCOVERY_NETWORK_TYPE_USBMUX;
        } else {
            this.networkType = ARDISCOVERY_NETWORK_TYPE_ENUM.ARDISCOVERY_NETWORK_TYPE_UNKNOWN;
        }
    }

    public ARDISCOVERY_NETWORK_TYPE_ENUM getNetworkType() {
        return networkType;
    }
    
    public int getProductID ()
    {
        return productID;
    }

    public void setProductID (int productID)
    {
        this.productID = productID;
    }

    @Override
    public int describeContents()
    {
        return 0;
    }

    @Override
    public String toString()
    {
        return "name="+name+", productID="+productID+", networkType="+networkType;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags)
    {    
        dest.writeString(this.name);
        dest.writeInt(this.productID);
        dest.writeInt(this.networkType.getValue());

        if(this.networkType != ARDISCOVERY_NETWORK_TYPE_ENUM.ARDISCOVERY_NETWORK_TYPE_UNKNOWN)
        {
            dest.writeParcelable((Parcelable) this.device, flags);
        }
    }
};

