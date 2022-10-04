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

import com.parrot.arsdk.arsal.ARSALPrint;

import org.json.JSONException;
import org.json.JSONObject;

public class ARDiscoveryDeviceNetService implements  Parcelable
{
    /**
     * 
     */

    private static String TAG = "ARDiscoveryDeviceNetService";
    
    private String name;
    private String type;
    private String ip;
    private int port;
    private String txtRecord;
    
    public static final Parcelable.Creator<ARDiscoveryDeviceNetService> CREATOR = new Parcelable.Creator<ARDiscoveryDeviceNetService>()
    {
        @Override
        public ARDiscoveryDeviceNetService createFromParcel(Parcel source)
        {
            return new ARDiscoveryDeviceNetService(source);
        }

        @Override
        public ARDiscoveryDeviceNetService[] newArray(int size)
        {
        return new ARDiscoveryDeviceNetService[size];
        }
    };
    
    public ARDiscoveryDeviceNetService ()
    {
        name = "";
        type = "";
        ip = "";
        port = 0;
    }
    
    public ARDiscoveryDeviceNetService (String name, String type, String ip, int port, String txtRecord)
    {
        this.name = name;
        this.type = type;
        this.ip = ip;
        this.port = port;
        this.txtRecord = txtRecord;
    }
    
    /* Parcelling part */
    public ARDiscoveryDeviceNetService(Parcel in)
    {
        this.name = in.readString();
        this.type = in.readString();
        this.ip = in.readString();
        this.port = in.readInt();
        this.txtRecord = in.readString();
    }
    
    @Override
    public boolean equals(Object other) 
    {
        boolean isEqual = true;
            
        if ((other == null) || !(other instanceof ARDiscoveryDeviceNetService))
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
            ARDiscoveryDeviceNetService otherDevice = (ARDiscoveryDeviceNetService) other;
            
            if ((txtRecord != null) || (otherDevice.getTxtRecord() != null))
            {
                // compare txtRecord
                
                if (txtRecord == otherDevice.getTxtRecord())
                {
                    isEqual = true;
                }
                else if ((txtRecord == null) || (otherDevice.getTxtRecord() == null))
                {
                    isEqual = false;
                }
                else
                {
                    String discoveryID = null;
                    String otherDiscoveryID = null;
                    
                    // get discoveryID
                    try
                    {
                        JSONObject jsonObject = new JSONObject(txtRecord);

                        if (!jsonObject.isNull(ARDiscoveryConnection.ARDISCOVERY_CONNECTION_JSON_DEVICE_ID_KEY))
                        {
                            discoveryID = jsonObject.getString(ARDiscoveryConnection.ARDISCOVERY_CONNECTION_JSON_DEVICE_ID_KEY);
                        }
                    }
                    catch (JSONException e)
                    {
                        //e.printStackTrace();
                        //Do Nothing
                    }
                    
                    // get otherDiscoveryID
                    try
                    {
                        JSONObject otherJsonObject = new JSONObject(otherDevice.getTxtRecord());

                        if (!otherJsonObject.isNull(ARDiscoveryConnection.ARDISCOVERY_CONNECTION_JSON_DEVICE_ID_KEY))
                        {
                            otherDiscoveryID = otherJsonObject.getString(ARDiscoveryConnection.ARDISCOVERY_CONNECTION_JSON_DEVICE_ID_KEY);
                        }
                    }
                    catch (JSONException e)
                    {
                        //e.printStackTrace();
                        //Do Nothing
                    }
                    
                    // compare the device ID
                    
                    // Because nsdManager, used on skeController, can't set TxtRecord in its services
                    // and an application workaround set the TxtRecord,
                    // two services with TxtRecord and without TxtRecord can be the same (one before the workaround and the other after).
                    // Compare the discoveryID only if the TWO services have a discoveryID, otherwise compare the name.
                    if ((discoveryID != null) && (otherDiscoveryID != null)) 
                    {
                        if (discoveryID != null)
                        {
                            isEqual = discoveryID.equals(otherDiscoveryID);
                        }
                        else
                        {
                            isEqual = false;
                        }
                    }
                    else
                    {
                        // no device ID : compare the name
                        isEqual = this.name.equals(otherDevice.name);
                    }
                }
            }
            else if (!this.name.equals(otherDevice.name)) // compare the name
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

    public String getType()
    {
        return type;
    }

    public void setType(String type)
    {
        this.type = type;
    }
    
    public String getIp()
    {
        return ip;
    }
    
    public void setIp (String ip)
    {
        this.ip = ip;
    }
    
    public int getPort()
    {
        return port;
    }
    
    public void setPort (int port)
    {
        this.port = port;
    }
    
    public String getTxtRecord()
    {
        return txtRecord;
    }
    
    public void setTxtRecord (String txtRecord)
    {
        this.txtRecord = txtRecord;
    }

    @Override
    public int describeContents()
    {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags)
    {    
        dest.writeString(this.name);
        dest.writeString(this.type);
        dest.writeString(this.ip);
        dest.writeInt(this.port);
        dest.writeString(this.txtRecord);
    }

};

