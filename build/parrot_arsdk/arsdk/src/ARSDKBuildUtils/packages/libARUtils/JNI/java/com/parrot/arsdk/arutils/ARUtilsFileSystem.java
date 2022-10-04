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

/**
 * ARUtils FileSystem module
 * @author david.flattin.ext@parrot.com
 * @date 30/12/2013
 */
public class ARUtilsFileSystem
{
    /* Native Functions */
    private native static boolean nativeStaticInit();
    private native long nativeGetFileSize(String namePath) throws ARUtilsException;
    private native int nativeRename(String oldName, String newName);
    private native int nativeRemoveFile(String localPath);
    private native int nativeRemoveDir(String localPath);
    private native double nativeGetFreeSpace(String localPath) throws ARUtilsException;
    
    /*  Java Methods */
    
    /**
     * Gets file size
     * @param namePath The file path name
     * @return the file size
     * @throws ARUtilsException if error
     */
    public long getFileSize(String namePath) throws ARUtilsException
    {
        return nativeGetFileSize(namePath);
    }
    
    /**
     * Renames file
     * @param oldName The old file path name
     * @param newName The new file path name
     * @return void
     * @throws ARUtilsException if error
     */
    public void rename(String oldName, String newName)  throws ARUtilsException
    {
        int result = nativeRename(oldName, newName);
        
        ARUTILS_ERROR_ENUM error = ARUTILS_ERROR_ENUM.getFromValue(result);
        
        if (error != ARUTILS_ERROR_ENUM.ARUTILS_OK)
        {
            throw new ARUtilsException(error);
        }
    }
    
    /**
     * Remove file
     * @param localPath The local file path name
     * @return void
     * @throws ARUtilsException if error
     */
    public void removeFile(String localPath) throws ARUtilsException
    {
        int result = nativeRemoveFile(localPath);
        
        ARUTILS_ERROR_ENUM error = ARUTILS_ERROR_ENUM.getFromValue(result);
        
        if (error != ARUTILS_ERROR_ENUM.ARUTILS_OK)
        {
            throw new ARUtilsException(error);
        }
    }
    
    /**
     * Removes directory and its content recursively
     * @param localPath The local file path name
     * @return void
     * @throws ARUtilsException if error
     */
    public void removeDir(String localPath) throws ARUtilsException
    {
        int result = nativeRemoveDir(localPath);
        
        ARUTILS_ERROR_ENUM error = ARUTILS_ERROR_ENUM.getFromValue(result);
        
        if (error != ARUTILS_ERROR_ENUM.ARUTILS_OK)
        {
            throw new ARUtilsException(error);
        }
    }
    
    /**
     * Gets file memory free space available
     * @param localPath The local file path name
     * @return void
     * @throws ARUtilsException if error
     */
    public double getFreeSpace(String localPath) throws ARUtilsException
    {
        return nativeGetFreeSpace(localPath);
    }
    
    /*  Static Block */
    static
    {
        nativeStaticInit();
    }
}

