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

package com.parrot.arsdk.arupdater;

import java.lang.Runnable;
import com.parrot.arsdk.arsal.ARSALPrint;
import com.parrot.arsdk.ardiscovery.ARDISCOVERY_PRODUCT_ENUM;
import com.parrot.arsdk.arutils.ARUtilsManager;


public class ARUpdaterManager
{

	private static final String TAG = "ARUpdaterManager";

	/* Native Functions */
	private static native void nativeStaticInit();
    private native long nativeNew() throws ARUpdaterException;
    private native int nativeDelete(long manager);
    private native boolean nativePlfVersionIsUpToDate(long manager, int discoveryProduct, String remoteVersion, String rootFolder) throws ARUpdaterException;
    private native boolean nativePlfVersionIsBlacklisted(int discoveryProduct, int version, int edition, int extension);
    private static native String nativeReadPlfVersion(String pflPath);
    private static native int nativeComparePlfVersions(String version1, String version2);
    private static native int nativeExtractUnixFileFromPlf(String plfFileName, String outFolder, String unixFileName);

    private long nativeManager = 0;
    private String localVersion = null;
    private boolean isInit = false;

    // Uploader from Android device to product
    ARUpdaterUploader uploader;

    // Downloader from Internet to Android device
    ARUpdaterDownloader downloader;

    /*  Static Block */
    static
    {
        nativeStaticInit();
    }  

    public ARUpdaterManager() throws ARUpdaterException
    {
        if (!isInit)
        {
            nativeManager = nativeNew();
        }
        if (nativeManager != 0)
        {
            isInit = true;
        }
    }


	/**
     * Deletes the ARUpdater Manager
     */
    public void dispose()
    {
        if (nativeManager != 0)
        {
            if (downloader != null)
            {
                downloader.dispose();
            }

            if (uploader != null)
            {
                uploader.dispose();
            }

            nativeDelete(nativeManager);
            nativeManager = 0;
            isInit = false;
        }
        
    }

    /**
     * Destructor<br>
     * This destructor tries to avoid leaks if the object was not disposed
     */
    protected void finalize () throws Throwable
    {
        try
        {
            if (isInit)
            {
                ARSALPrint.e (TAG, "Object " + this + " was not disposed !");
                dispose();
            }
        }
        finally
        {
            super.finalize ();
        }
    }

    /**
     * Gets the Manager status Initialized or not
     * @return true if the Manager is already created else false
     */     
    public boolean isInitialized()
    {
        return isInit;
    }


    /**
     * Gets the ARUpdater Downloader object {@link ARUpdaterDownloader}
     * @return an ARUpdater Downloader object, null if Manager not initialized
     */     
    public ARUpdaterDownloader getARUpdaterDownloader()
    {
        if (isInit == false)
        {
            return null;
        }
        
        if (downloader == null)
        {
            downloader = new ARUpdaterDownloader(nativeManager);
        }
        
        return downloader;
    }
    
    /**
     * Gets the ARUpdater Uploader object {@link ARUpdaterUploader}
     * @return an ARUpdater Uploader object, null if Manager not initialized
     */
    public ARUpdaterUploader getARUpdaterUploader()
    {
        if (isInit == false)
        {
            return null;
        }
        
        if (uploader == null)
        {
            uploader = new ARUpdaterUploader(nativeManager);
        }
        
        return uploader;
    }

    /**
     * Test if the plf in the root folder is up to date compare to the given product information
     * @return true is the plf is up to date, false otherwise
     * @throws ARUpdaterException throws ARUpdaterException if there is a bad parameter or if there is no plf file in the root folder
     */
    public ARUpdaterUploadPlfVersionInfo isPlfVersionUpToDate(ARDISCOVERY_PRODUCT_ENUM product, String remoteVersion, String rootFolder) throws ARUpdaterException
    {
        boolean upToDate = nativePlfVersionIsUpToDate(nativeManager, product.getValue(), remoteVersion, rootFolder);
        ARUpdaterUploadPlfVersionInfo toReturn = new ARUpdaterUploadPlfVersionInfo(upToDate, this.localVersion);
        return toReturn;
    }

    public boolean isPlfVersionBlacklisted(ARDISCOVERY_PRODUCT_ENUM product, int version, int edition, int extension)
    {
        return nativePlfVersionIsBlacklisted(product.getValue(), version, edition, extension);
    }

    public static String readPlfVersion(String plfPath)
    {
        return nativeReadPlfVersion(plfPath);
    }

    /**
    * Compare two plf versions
    * @return It returns an integer less than 0 if version1 is lower than version2, 0 if versions are equal,
    * greater than zero if version1 is greater than version2
    */
    public static int comparePlfVersions(String version1, String version2)
    {
        return nativeComparePlfVersions(version1, version2);
    }

    /**
     * Extract a U_UNIFXILE regular file from a PLF file
     *
     * Example: a U_UNIXFILE section (regular file) with path "data/foo/config.txt" should be extracted
     * by providing the last path component ("config.txt") as parameter unixFileName.
     *
     * @return ARUPDATER_OK if operation went well, the description of the error otherwise
     */
    public static ARUPDATER_ERROR_ENUM extractUnixFileFromPlf(String plfFileName, String outFolder, String unixFileName)
    {
        return ARUPDATER_ERROR_ENUM.getFromValue(nativeExtractUnixFileFromPlf(plfFileName, outFolder, unixFileName));
    }
}
