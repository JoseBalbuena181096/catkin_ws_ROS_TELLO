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

package com.parrot.arsdk.arsal;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

import android.util.Log;

public class ARSALMd5
{
    public final static int MD5_LENGTH = 16;
	
    private MessageDigest digest = null;
    private static final String TAG = "Md5";
    
    public ARSALMd5()
    {
        initialize();
    }
    
    private boolean initialize()
    {
        boolean ret = true;
        try
        {
            digest = java.security.MessageDigest.getInstance("MD5");
        }
        catch (NoSuchAlgorithmException e)
        {
            Log.d(TAG, e.toString());
            ret = false;
        }
        
        return ret;
    }
    
    public String getTextDigest(byte[] hash, int index, int len)
	{
		StringBuffer txt = new StringBuffer();
		
		for (int i=0; i<len; i++)
		{
			String val = String.format("%02x", hash[index + i] & 0x00FF);
			txt.append(val);
		}
		return txt.toString();
	}
    
    public boolean check(String filePath, String md5Txt)
    {
        byte[] md5 = null;
        boolean result = false;
        
        md5 = compute(filePath);
        
        if (md5 != null)
        {            
            if (getTextDigest(md5, 0, md5.length).compareTo(md5Txt) == 0)
            {
                result = true;
            }
        }
        
        return result;
    }
    
    public byte [] compute(String filePath)
    {
        FileInputStream src = null;
        byte[] block = new byte[1024];
        byte[] md5 = null;
        int count;
        
        try
        {
            initialize();
            src = new FileInputStream(filePath);
             
            while ((count = src.read(block, 0, block.length)) > 0)
            {
                digest.update(block, 0, count);
            }
            
            md5 = digest.digest();
            src.close();
            
            //Log.d("DBG", "ARSALMd5.compute: " + getTextDigest(md5, 0, md5.length));
        }
        catch (FileNotFoundException e)
        {
            Log.d("DBG", e.toString());
        }
        catch (IOException e)
        {
            Log.d("DBG", e.toString());
        }
        
        return md5;
    }

    public byte [] compute(byte[] data)
    {
        byte[] md5 = null;
        initialize();
        digest.update(data);
        md5 = digest.digest();
        //Log.d("DBG", "ARSALMd5.compute: " + getTextDigest(md5, 0, md5.length));
        return md5;
    }
}


