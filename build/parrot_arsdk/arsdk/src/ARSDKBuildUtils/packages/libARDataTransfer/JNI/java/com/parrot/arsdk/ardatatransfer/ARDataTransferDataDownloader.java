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

import java.lang.Runnable;
import com.parrot.arsdk.arsal.ARSALPrint;
import com.parrot.arsdk.arutils.ARUtilsManager;

/**
 * ARDataTransfer DataDownloader module
 * @author david.flattin.ext@parrot.com
 * @date 19/12/2013
 */
public class ARDataTransferDataDownloader
{
    /* Native Functions */
    private native static boolean nativeStaticInit();
    private native int nativeNew(long manager, long utilsListManager, long utilsDataManager, String remoteDirectory, String localDirectory, ARDataTransferDataDownloaderFileCompletionListener fileCompletionListener, Object fileCompletionArg);
    private native int nativeDelete(long manager);
    private native long nativeGetAvailableFiles(long manager) throws ARDataTransferException;
    private native int nativeCancelAvailableFiles(long manager);
    private native void nativeThreadRun (long manager);
    private native int nativeCancelThread (long manager);

    /*  Members  */
    private static final String TAG = ARDataTransferDataDownloader.class.getSimpleName ();
    private boolean isInit = false;
    private long nativeManager = 0;
    private Runnable downloaderRunnable = null;

    /*  Java Methods */

    /**
     * Private ARDataTransfer DataDownloader constructor
     * @return void
     */
    protected ARDataTransferDataDownloader(long _nativeManager)
    {
        this.nativeManager = _nativeManager;

        this.downloaderRunnable = new Runnable () {
            public void run() {
                nativeThreadRun(nativeManager);
            }
        };
    }

    /**
     * Creates a new ARDataTransfer DataDownloader
     * @return void
     * @throws ARDataTransferException if error
     */
    public void createDataDownloader(ARUtilsManager utilsListManager, ARUtilsManager utilsDataManager, String remoteDirectory, String localDirectory, ARDataTransferDataDownloaderFileCompletionListener fileCompletionListener, Object fileCompletionArg) throws ARDataTransferException
    {
        int result = nativeNew(nativeManager, utilsListManager.getManager(), utilsDataManager.getManager(), remoteDirectory, localDirectory, fileCompletionListener, fileCompletionArg);

        ARDATATRANSFER_ERROR_ENUM error = ARDATATRANSFER_ERROR_ENUM.getFromValue(result);

        if (error != ARDATATRANSFER_ERROR_ENUM.ARDATATRANSFER_OK)
        {
            throw new ARDataTransferException(error);
        }
        else
        {
            isInit = true;
        }
    }

    /**
     * Deletes an ARDataTransfer DataDownloader
     * @return ARDATATRANSFER_OK if success, else an {@link ARDATATRANSFER_ERROR_ENUM} error code
     */
    public ARDATATRANSFER_ERROR_ENUM dispose()
    {
        ARDATATRANSFER_ERROR_ENUM error = ARDATATRANSFER_ERROR_ENUM.ARDATATRANSFER_OK;

        if (isInit)
        {
            int result = nativeDelete(nativeManager);

            error = ARDATATRANSFER_ERROR_ENUM.getFromValue(result);
            if (error == ARDATATRANSFER_ERROR_ENUM.ARDATATRANSFER_OK)
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
                ARDATATRANSFER_ERROR_ENUM error = dispose ();
                if(error != ARDATATRANSFER_ERROR_ENUM.ARDATATRANSFER_OK)
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

   /**
     * Gets ARDataTransfer DataDownloader available files
     * @return the available files count
     */
    public long getAvailableFiles()  throws ARDataTransferException
    {
        long result = nativeGetAvailableFiles(nativeManager);

        return result;
    }

    /**
     * Gets ARDataTransfer DataDownloader available files
     * @return DataDownloader Runnable
     */
    public ARDATATRANSFER_ERROR_ENUM cancelAvailableFiles()
    {
         int result = nativeCancelAvailableFiles(nativeManager);

        ARDATATRANSFER_ERROR_ENUM error = ARDATATRANSFER_ERROR_ENUM.getFromValue(result);
        return error;
    }

    /**
     * Gets the ARDataTransfer DataDownloader {@link Runnable} to start as new {@link Thread}
     * @return DataDownloader Runnable
     */
    public Runnable getDownloaderRunnable()
    {
        Runnable runnable = null;

        if (isInit == true)
        {
            runnable = this.downloaderRunnable;
        }

        return runnable;
    }

    /**
     * Cancels the ARDataTransfer DataDownloader Runnable Thread
     * @return ARDATATRANSFER_OK if success, else an {@link ARDATATRANSFER_ERROR_ENUM} error code
     */
    public ARDATATRANSFER_ERROR_ENUM cancelThread()
    {
        int result = nativeCancelThread(nativeManager);

        ARDATATRANSFER_ERROR_ENUM error = ARDATATRANSFER_ERROR_ENUM.getFromValue(result);
        return error;
    }

    /*  Static Block */
    static
    {
        nativeStaticInit();
    }
}
