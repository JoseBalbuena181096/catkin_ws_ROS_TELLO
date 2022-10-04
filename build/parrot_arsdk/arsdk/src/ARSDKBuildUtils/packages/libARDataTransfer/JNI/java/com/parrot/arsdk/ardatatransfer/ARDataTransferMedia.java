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

package com.parrot.arsdk.ardatatransfer;

import com.parrot.arsdk.ardiscovery.ARDISCOVERY_PRODUCT_ENUM;
import com.parrot.arsdk.arsal.ARSALPrint;

import android.os.Parcel;
import android.os.Parcelable;

import java.util.Arrays;

/**
 * ARDataTransfer Media
 * @author david.flattin.ext@parrot.com
 * @date 19/12/2013
 */
public class ARDataTransferMedia implements Parcelable
{
    /*  Members  */
    private static final int HASH_START = 17;
    private static final int HASH_FIELD = 31;
    
    private static final String TAG = "ARDataTransferMedia";
    private ARDISCOVERY_PRODUCT_ENUM product = null;
    private String name = null;
    private String filePath = null;
    private String date = null;
    private String uuid = null;
    private String remotePath = null;
    private String remoteThumb = null;
    private float size = 0.f;
    private byte[] thumbnail = null;

    /*  Java Methods */

    /**
     * Private ARDataTransfer Media constructor
     * @note private ARDataTransfer Media constructor
     * @param name String Media Name
     * @param filePath String Media Path
     * @param date String Media Date
     * @param uuid String Media UUID
     * @param size float Media Size
     * @param thumbnail byte[] Media Thumbnail
     * @return void
     */
    protected ARDataTransferMedia(int productValue, String name, String filePath, String date, String uuid, String remotePath, String remoteThumb, float size, byte[] thumbnail)
    {
        this.product = ARDISCOVERY_PRODUCT_ENUM.getFromValue(productValue);
        this.name = name;
        this.filePath = filePath;
        this.date = date;
        this.uuid = uuid;
        this.remotePath = remotePath;
        this.remoteThumb = remoteThumb;
        this.size = size;
        this.thumbnail = thumbnail;
    }

    private ARDataTransferMedia(Parcel source)
    {
        this.product = ARDISCOVERY_PRODUCT_ENUM.getFromValue(source.readInt());
        this.name = readString(source);
        this.filePath = readString(source);
        this.date = readString(source);
        this.uuid = readString(source);
        this.remotePath = readString(source);
        this.remoteThumb = readString(source);
        this.size = source.readFloat();
        int thumbnailSize = source.readInt();
        if (thumbnailSize > 0)
        {
            this.thumbnail = new byte[thumbnailSize];
            source.readByteArray(this.thumbnail);
        }
    }

    @Override
    public void writeToParcel(Parcel dest, int flags)
    {
        dest.writeInt(this.product.getValue());
        writeString(dest, this.name);
        writeString(dest, this.filePath);
        writeString(dest, this.date);
        writeString(dest, this.uuid);
        writeString(dest, this.remotePath);
        writeString(dest, this.remoteThumb);
        dest.writeFloat(this.size);
        int thumbnailSize = (this.thumbnail != null) ? this.thumbnail.length : 0;
        dest.writeInt(thumbnailSize);
        if (thumbnailSize > 0)
        {
            dest.writeByteArray(this.thumbnail);
        }
    }

    public static final Parcelable.Creator<ARDataTransferMedia> CREATOR = new Creator<ARDataTransferMedia>()
    {
        public ARDataTransferMedia createFromParcel(Parcel source)
        {
            return new ARDataTransferMedia(source);
        }

        @Override
        public ARDataTransferMedia[] newArray(int size)
        {
            return new ARDataTransferMedia[size];
        }
    };

    @Override
    public int describeContents()
    {
        // TODO Auto-generated method stub
        return 0;
    }

    private static String readString(Parcel source)
    {
        byte presentByte = source.readByte();
        if (presentByte > 0)
        {
            return source.readString();
        }
        return null;
    }

    private static void writeString(Parcel dest, String value)
    {
        dest.writeByte((value != null) ? (byte)1 : (byte)0);
        if (value != null)
        {
            dest.writeString(value);
        }
    }

    /**
     * Gets the Media product it belong to
     * @note get the Media Product
     * @return String media Product
     */
    public ARDISCOVERY_PRODUCT_ENUM getProduct()
    {
        return this.product;
    }

    /**
     * Gets the Media product it belong to
     * @note get the Media Product
     * @return String media Product
     */
    public int getProductValue()
    {
        return this.product.getValue();
    }

    /**
     * Gets the Media Name
     * @note get the Media Name
     * @return String media Name
     */
    public String getName()
    {
        return this.name;
    }

    /**
     * Gets the Media file Path
     * @note get the Media file Path
     * @return String media file Path
     */
    public String getFilePath()
    {
        return this.filePath;
    }

    /**
     * Gets the Media Date
     * @note get the Media Date
     * @return String media Date
     */
    public String getDate()
    {
        return this.date;
    }

    /**
     * Gets the Media UUID
     * @note get the Media UUID
     * @return String media UUID
     */
    public String getUUID()
    {
        return this.uuid;
    }

    /**
     * Gets the Media remote path
     * @note get the Media remote path
     * @return String media remote path
     */
    public String getRemotePath()
    {
        return this.remotePath;
    }

    /**
     * Gets the Media remote thumbnail path
     * @note get the Media remote thumbnail path
     * @return String media remote thumbnail path
     */
    public String getRemoteThumb()
    {
        return this.remoteThumb;
    }

    /**
     * Gets the Media Size
     * @note get the Media Size
     * @return float media Size
     */
    public float getSize()
    {
        return this.size;
    }

    /**
     * Gets the Media Thumbnail
     * @note get the Media Thumbnail
     * @return byte[] media Thumbnail
     */
    public byte[] getThumbnail()
    {
        return this.thumbnail;
    }

    /**
     * Sets the Media Thumbnail
     * Data are copied into a new array
     * @note set the Media Thumbnail
     */
    public void setThumbail(byte[] rawData)
    {
        this.thumbnail = Arrays.copyOf(rawData, rawData.length);
    }
    
    @Override
    public boolean equals(Object other)
    {
        boolean isEqual = false;
        if (other == null)
        {
            isEqual = false;
        }
        else if (other == this)
        {
            isEqual = true;
        }
        else if (!(other instanceof ARDataTransferMedia))
        {
            isEqual = false;
        }
        else
        {
            ARDataTransferMedia otherMedia = (ARDataTransferMedia)other;
            
            if ((otherMedia.getProductValue() == getProductValue()) &&
                (((name == null) && (otherMedia.getName() == null)) || ((name != null) && (name.equals(otherMedia.getName())))) &&
                (((filePath == null) && (otherMedia.getFilePath() == null)) || ((filePath != null) && (filePath.equals(otherMedia.getFilePath())))) &&
                (((remotePath == null) && (otherMedia.getRemotePath() == null)) || (remotePath != null) && (remotePath.equals(otherMedia.getRemotePath()))) &&
                (((remoteThumb == null) && (otherMedia.getRemoteThumb() == null)) || (remoteThumb != null) && (remoteThumb.equals(otherMedia.getRemoteThumb()))) &&
                (((date == null) && (otherMedia.getDate() == null)) || ((date != null) && (date.equals(otherMedia.getDate())))) &&
                (((uuid == null) && (otherMedia.getUUID() == null)) || ((uuid != null) && (uuid.equals(otherMedia.getUUID())))))
            {
                isEqual = true;
            }
            //NO ELSE ; isEqual = false
        }
        
        return isEqual;
    }
    
    @Override public int hashCode()
    {
        // Start with a non-zero constant.
        int result = HASH_START;

        // Include a hash for each field.
        result = HASH_FIELD * result + product.getValue();
        result = HASH_FIELD * result + (name == null ? 0 : name.hashCode());
        result = HASH_FIELD * result + (filePath == null ? 0 : filePath.hashCode());
        result = HASH_FIELD * result + (date == null ? 0 : date.hashCode());
        result = HASH_FIELD * result + (uuid == null ? 0 : uuid.hashCode());
        result = HASH_FIELD * result + (remotePath == null ? 0 : remotePath.hashCode());
        result = HASH_FIELD * result + (remoteThumb == null ? 0 : remoteThumb.hashCode());

        return result;
   }
}
