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

import com.parrot.arsdk.arsal.ARSALPrint;

/**
 * ARDataTransfer Manager
 * @author david.flattin.ext@parrot.com
 * @date 19/12/2013
 */
public class ARDataTransferManager
{
    /* Native Functions */
    private native static boolean nativeStaticInit();
    private native long nativeNew() throws ARDataTransferException;
    private native void nativeDelete(long manager);

    /*  Members  */
    private static final String TAG = ARDataTransferManager.class.getSimpleName ();
    private long nativeManager = 0;
    private boolean isInit = false;
    ARDataTransferDataDownloader dataDownloader = null;
    ARDataTransferMediasDownloader mediasDownloader = null;
    ARDataTransferDownloader downloader = null;
    ARDataTransferUploader uploader = null;

    /*  Java Methods */

    /**
     * Creates ARDataTransfer Manager
     * @return void
     * @throws ARDataTransferException if error
     */
    public ARDataTransferManager() throws ARDataTransferException
    {
        if (isInit == false)
        {
            nativeManager = nativeNew();
        }

        if (nativeManager != 0)
        {
            isInit = true;
        }
    }

    /**
     * Deletes the ARDataTransferManager.<br>
     */
    public void dispose()
    {
        if (nativeManager != 0)
        {
            if (dataDownloader != null)
            {
                dataDownloader.dispose();
            }

            if(mediasDownloader != null)
            {
                mediasDownloader.dispose();
            }

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
                dispose ();
            }
        }
        finally
        {
            super.finalize ();
        }
    }

    /**
     * Gets the ARDataTransfer Manager status Initialized or not
     * @return true if the Manager is already created else false
     */
    public boolean isInitialized()
    {
        return isInit;
    }

    /**
     * Gets the ARDataTransfer DataDownloader object {@link ARDataTransferDataDownloader}
     * @return an ARDataTransfer DataDownloader object, null if Manager not initialized
     */
    public ARDataTransferDataDownloader getARDataTransferDataDownloader()
    {
        if (isInit == false)
        {
            return null;
        }

        if (dataDownloader == null)
        {
            dataDownloader = new ARDataTransferDataDownloader(nativeManager);
        }

        return dataDownloader;
    }

    /**
     * Gets the ARDataTransfer MediasDownloader object {@link ARDataTransferMediasDownloader}
     * @return an ARDataTransfer MediasDownloader object, null if Manager not initialized
     */
    public ARDataTransferMediasDownloader getARDataTransferMediasDownloader()
    {
        if (isInit == false)
        {
            return null;
        }

        if (mediasDownloader == null)
        {
            mediasDownloader = new ARDataTransferMediasDownloader(nativeManager);
        }

        return mediasDownloader;
    }

    /**
     * Gets the ARDataTransfer Downloader object {@link ARDataTransferDownloader}
     * @return an ARDataTransfer Downloader object, null if Manager not initialized
     */
    public ARDataTransferDownloader getARDataTransferDownloader()
    {
        if (isInit == false)
        {
            return null;
        }

        if (downloader == null)
        {
            downloader = new ARDataTransferDownloader(nativeManager);
        }

        return downloader;
    }

    /**
     * Gets the ARDataTransfer Uploader object {@link ARDataTransferUploader}
     * @return an ARDataTransfer Uploader object, null if Manager not initialized
     */
    public ARDataTransferUploader getARDataTransferUploader()
    {
        if (isInit == false)
        {
            return null;
        }

        if (uploader == null)
        {
            uploader = new ARDataTransferUploader(nativeManager);
        }

        return uploader;
    }

    /*  Static Block */
    static
    {
        nativeStaticInit();
    }
}
