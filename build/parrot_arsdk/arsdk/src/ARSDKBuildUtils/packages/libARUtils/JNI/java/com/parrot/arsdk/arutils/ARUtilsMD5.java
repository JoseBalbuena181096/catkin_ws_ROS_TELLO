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
package com.parrot.arsdk.arutils;

import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

import com.parrot.arsdk.arsal.ARSALPrint;

public class ARUtilsMD5 
{

	private static final String TAG = "ARUtilsMD5";

	public final static int MD5_LENGTH = 16;
	
	MessageDigest digest = null;
	
	public ARUtilsMD5()
	{
		initialize();
	}
	
	public boolean initialize()
	{
		boolean ret = true;
		try
		{
			digest = java.security.MessageDigest.getInstance("MD5");
		}
		catch (NoSuchAlgorithmException e)
		{
			ARSALPrint.d(TAG, e.toString());
			ret = false;
		}
		
		return ret;
	}
	
	public void update(byte[] buffer, int index, int len)
	{
		digest.update(buffer, index, len);
	}
	
	static public String getTextDigest(byte[] hash, int index, int len)
	{
		StringBuffer txt = new StringBuffer();
		
		for (int i=0; i<len; i++)
		{
			String val = String.format("%02x", hash[index + i] & 0x00FF);
			txt.append(val);
		}
		return txt.toString();
	}
	
	public String digest()
	{
		byte[] hash = digest.digest();
		return getTextDigest(hash, 0, hash.length);
	}
}
