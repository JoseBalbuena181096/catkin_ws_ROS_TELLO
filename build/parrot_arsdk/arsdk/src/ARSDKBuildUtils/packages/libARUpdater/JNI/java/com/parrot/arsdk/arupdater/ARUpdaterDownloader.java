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
import com.parrot.arsdk.arsal.ARSALMd5Manager;
import com.parrot.arsdk.ardiscovery.ARDISCOVERY_PRODUCT_ENUM;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.Collections;

public class ARUpdaterDownloader
{

	private static final String TAG = "ARUpdaterDownloader";

	/* Native Functions */
	private static native void nativeStaticInit();
    private native int nativeNew(long manager, String rootFolder, long md5Manager, int platform, String appVersion, ARUpdaterShouldDownloadPlfListener shouldDownloadPlfListener, Object willDownloadPlfArgs, 
        ARUpdaterWillDownloadPlfListener willDownloadPlfListener, Object downloadArgs, 
    	ARUpdaterPlfDownloadProgressListener plfDownloadProgressListener, Object progressArgs, 
    	ARUpdaterPlfDownloadCompletionListener plfDownloadCompletionListener, Object completionArgs);
    private native int nativeDelete(long manager);
    private native void nativeThreadRun (long manager);
    private native int nativeCancelThread (long manager);
    private native int nativeSetVariant (long manager, String variant);
    private native int nativeSetUpdatesProductList (long manager, int[] productArray);
    private native int nativeCheckUpdatesAsync(long manager);
    private native int nativeCheckUpdatesSync(long manager) throws ARUpdaterException;
    private native ARUpdaterDownloadInfo[] nativeGetUpdatesInfoSync(long manager) throws ARUpdaterException;
    private native int nativeGetBlacklistedFirmwareVersionsSync(long manager, int alsoCheckRemote, int[] productArray, Object[] blacklistedVersions) throws ARUpdaterException;

    private long nativeManager = 0;
    private Runnable downloaderRunnable = null;
    private boolean isInit = false;

    /*  Static Block */
    static
    {
        nativeStaticInit();
    }  

    protected ARUpdaterDownloader(long _nativeManager)
    {
    	this.nativeManager = _nativeManager;
    	this.downloaderRunnable = new Runnable() {
    		public void run() {
    			nativeThreadRun(nativeManager);
    		}
    	};
    }


	/**
     * Creates the ARUpdater Downloader
     * @param rootFolder The root folder
     * @param shouldDownloadPlfListener The available download listener
     * @param downloadArgs The available download listener arg
     * @param[in] willDownloadPlfListener : The available will download listener
     * @param[in|out] willDownloadArg : The available will download listener arg
     * @param plfDownloadProgressListener The available progress listener
     * @param progressArgs The available progress listener arg
     * @param completionArgs The available completion listener
     * @param progressArgs The available completion listener arg
     * @return void
     * @throws ARUpdaterException if error
     */
    public void createUpdaterDownloader(String rootFolder, ARSALMd5Manager md5Manager, String appVersion, 
        ARUpdaterShouldDownloadPlfListener shouldDownloadPlfListener, Object downloadArgs, 
        ARUpdaterWillDownloadPlfListener willDownloadPlfListener, Object willDownloadPlfArgs,
    	ARUpdaterPlfDownloadProgressListener plfDownloadProgressListener, Object progressArgs, 
    	ARUpdaterPlfDownloadCompletionListener plfDownloadCompletionListener, Object completionArgs) throws ARUpdaterException
    {
    	int result = nativeNew(nativeManager, rootFolder, md5Manager.getNativeManager(),  ARUPDATER_Downloader_Platforms_ENUM.ARUPDATER_DOWNLOADER_ANDROID_PLATFORM.getValue(), appVersion, shouldDownloadPlfListener, downloadArgs, willDownloadPlfListener, willDownloadPlfArgs, plfDownloadProgressListener, progressArgs, plfDownloadCompletionListener, completionArgs);

    	ARUPDATER_ERROR_ENUM error = ARUPDATER_ERROR_ENUM.getFromValue(result);

    	if (error != ARUPDATER_ERROR_ENUM.ARUPDATER_OK)
    	{
    		throw new ARUpdaterException();
    	}
    	else
    	{
    		this.isInit = true;
    	}
    }

	/**
     * Deletes the ARUpdater Downloader
     * @return ARUPDATER_OK if success, else an {@link ARUPDATER_ERROR_ENUM} error code
     */
    public ARUPDATER_ERROR_ENUM dispose()
    {
        ARUPDATER_ERROR_ENUM error = ARUPDATER_ERROR_ENUM.ARUPDATER_OK;
        
        if (isInit)
        {
            int result = nativeDelete(nativeManager);
            
            error = ARUPDATER_ERROR_ENUM.getFromValue(result);
            if (error == ARUPDATER_ERROR_ENUM.ARUPDATER_OK)
            {
                isInit = false;
            }
        }
        
        return error;
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
                ARUPDATER_ERROR_ENUM error = dispose ();
                if(error != ARUPDATER_ERROR_ENUM.ARUPDATER_OK)
                {
                    ARSALPrint.e (TAG, "Unable to dispose object " + this + " ... leaking memory !");
                }
            }
        }
        finally
        {
            super.finalize ();
        }
    }

    public ARUPDATER_ERROR_ENUM cancel()
    {
    	int result = nativeCancelThread(nativeManager);

    	ARUPDATER_ERROR_ENUM error = ARUPDATER_ERROR_ENUM.getFromValue(result);

    	return error;
    }

    public Runnable getDownloaderRunnable()
    {
        Runnable runnable = null;

        if (isInit == true)
        {
            runnable = this.downloaderRunnable;
        }

        return runnable;
    }

    public ARUPDATER_ERROR_ENUM setVariant(String variant)
    {
        int result = nativeSetVariant(nativeManager, variant);

        ARUPDATER_ERROR_ENUM error = ARUPDATER_ERROR_ENUM.getFromValue(result);

        return error;
    }

    public ARUPDATER_ERROR_ENUM setUpdatesProductList(ARDISCOVERY_PRODUCT_ENUM[] productEnumArray)
    {
        int[] productArray = new int[productEnumArray.length];
        for (int i=0; i<productEnumArray.length; i++)
        {
            productArray[i] = productEnumArray[i].getValue();
        }
        int result = nativeSetUpdatesProductList(nativeManager, productArray);

        ARUPDATER_ERROR_ENUM error = ARUPDATER_ERROR_ENUM.getFromValue(result);

        return error;
    }

    /**
     * Use this to check asynchronously update from internet (must be called from a background thread)
     * The ARUpdaterPlfShouldDownloadPlfListener callback set in the 'createUpdaterDownloader' method will be called
     */
    public ARUPDATER_ERROR_ENUM checkUpdatesAsync()
    {
        int result = nativeCheckUpdatesAsync(nativeManager);

        ARUPDATER_ERROR_ENUM error = ARUPDATER_ERROR_ENUM.getFromValue(result);

        return error;
    }

    /**
     * Use this to check synchronously update from internet
     */
    public int checkUpdatesSync() throws ARUpdaterException
    {
        int nbPlfToBeUpdated = nativeCheckUpdatesSync(nativeManager);

        return nbPlfToBeUpdated;
    }

    /**
     * Use this to get synchronously update info from internet
     */
    public ARUpdaterDownloadInfo[] getUpdatesInfoSync() throws ARUpdaterException
    {
        ARUpdaterDownloadInfo[] infos = nativeGetUpdatesInfoSync(nativeManager);

        return infos;
    }
    
    public HashMap<ARDISCOVERY_PRODUCT_ENUM, Set<String>> getBlacklistedVersionSync(boolean alsoCheckRemote) throws ARUpdaterException
    {
        HashMap<ARDISCOVERY_PRODUCT_ENUM, Set<String>> blacklistDict = null;
        int[] productArray = new int[ARDISCOVERY_PRODUCT_ENUM.ARDISCOVERY_PRODUCT_MAX.getValue()];
        
        Object[] blacklistedVersionArray = new Object[ARDISCOVERY_PRODUCT_ENUM.ARDISCOVERY_PRODUCT_MAX.getValue()];
        
        int alsoCheckRemoteInt = (alsoCheckRemote) ? 1 : 0;
        
        int result = nativeGetBlacklistedFirmwareVersionsSync(nativeManager, alsoCheckRemoteInt, productArray, blacklistedVersionArray);
        
        ARUPDATER_ERROR_ENUM error = ARUPDATER_ERROR_ENUM.getFromValue(result);
        
        if (error == ARUPDATER_ERROR_ENUM.ARUPDATER_OK)
        {
            blacklistDict = new HashMap<ARDISCOVERY_PRODUCT_ENUM, Set<String>>();
            for (int i = 0; i < ARDISCOVERY_PRODUCT_ENUM.ARDISCOVERY_PRODUCT_MAX.getValue(); i++)
            {
                ARDISCOVERY_PRODUCT_ENUM product = ARDISCOVERY_PRODUCT_ENUM.getFromValue(productArray[i]);
                String[] blacklistedVersionsForThisProduct = (String[])blacklistedVersionArray[i];
                if (blacklistedVersionsForThisProduct != null)
                {
                    Set<String> blacklistedStringArr = new HashSet<String>();
                    Collections.addAll(blacklistedStringArr, blacklistedVersionsForThisProduct);
                                        
                    blacklistDict.put(product, blacklistedStringArr);
                }
            }
        }
        
        return blacklistDict;
    }
}
