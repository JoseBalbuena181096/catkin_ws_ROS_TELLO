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
import java.util.Vector;
import java.util.List;
import com.parrot.arsdk.arsal.ARSALPrint;
import com.parrot.arsdk.arutils.ARUtilsManager;

/**
 * ARDataTransfer MediasDownloader module
 * @author david.flattin.ext@parrot.com
 * @date 19/12/2013
 */
public class ARDataTransferMediasDownloader
{
    /* Native Functions */
    private native static boolean nativeStaticInit();
    private native int nativeNew(long manager, long utilsListManager, long utilsQueueManager, String remoteDirectory, String localDirectory);
    private native int nativeDelete(long manager);    
    private native int nativeGetAvailableMediasSync(long manager, boolean withThumbnail);
    private native ARDataTransferMedia nativeGetAvailableMediaAtIndex(long manager, int index);
    private native int nativeGetAvailableMediasAsync(long manager, ARDataTransferMediasDownloaderAvailableMediaListener availableMediaListener, Object availableMediaArg);
    private native int nativeAddMediaToQueue(long manager, ARDataTransferMedia media, ARDataTransferMediasDownloaderProgressListener progressListener, Object progressArg, ARDataTransferMediasDownloaderCompletionListener completionListener, Object completionArg);
    private native int nativeDeleteMedia(long manager, ARDataTransferMedia media);
    private native void nativeQueueThreadRun(long manager);
    private native int nativeCancelQueueThread(long manager);
    private native int nativeCancelGetAvailableMedias(long manager);
    private native byte[] nativeGetMediaThumbnail(long manager, ARDataTransferMedia media);
    
    /*  Members  */
    private static final String TAG = ARDataTransferMediasDownloader.class.getSimpleName ();
    private boolean isInit = false;
    private long nativeManager = 0;
    private Runnable downloaderRunnable = null;
    
    /*  Java Methods */
    
    /**
     * Private ARDataTransfer MediasDownloader constructor
     * @return void
     */
    protected ARDataTransferMediasDownloader(long _nativeManager)
    {
        this.nativeManager = _nativeManager;
        
        this.downloaderRunnable = new Runnable () {
            public void run() {
                nativeQueueThreadRun(nativeManager);    
            }
        };
    }
    
    /**
     * Creates the ARDataTransfer MediasDownloader
     * @param deviceIP The FTP server ip address
     * @param port The FTP server port
     * @param remoteDirectory The FTP Server local directory
     * @param localDirectory The local system directory
     * @return void
     * @throws ARDataTransferException if error
     */
    public void createMediasDownloader(ARUtilsManager utilsListManager, ARUtilsManager utilsQueueManager, String remoteDirectory, String localDirectory) throws ARDataTransferException
    {
        int result = nativeNew(nativeManager, utilsListManager.getManager(), utilsQueueManager.getManager(), remoteDirectory, localDirectory);
        
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
     * Deletes the ARDataTransfer MediasDownloader
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
     * Gets the {@link List} of available {@link ARDataTransferMedia} medias in the ARDataTransfer MediasDownloader
     * @param withThumbnail The true to return thumnail, else false
     * @return the count of available ARDataTransferMedia medias List
     * @throws ARDataTransferException if error
     */
    public int getAvailableMediasSync(boolean withThumbnail) throws ARDataTransferException
    {
        return nativeGetAvailableMediasSync(nativeManager, withThumbnail);
    }
    
    public ARDataTransferMedia getAvailableMediaAtIndex(int index) throws ARDataTransferException
    {
        return nativeGetAvailableMediaAtIndex(nativeManager, index);
    }
    
    /**
     * Gets the availables medias in the ARDataTransfer MediasDownloader and signal each Media found with the ARDataTransferMediasDownloaderAvailableMediaListener listener
     * @param availableMediaListener The available Media listener
     * @param availableMediaArg The availale Media listener arg 
     * @return void
     * @throws ARDataTransferException if error
     */
    public void getAvailableMediasAsync(ARDataTransferMediasDownloaderAvailableMediaListener availableMediaListener, Object availableMediaArg)  throws ARDataTransferException
    {
        int result = nativeGetAvailableMediasAsync(nativeManager, availableMediaListener, availableMediaArg);
        
        ARDATATRANSFER_ERROR_ENUM error = ARDATATRANSFER_ERROR_ENUM.getFromValue(result);

        if (error != ARDATATRANSFER_ERROR_ENUM.ARDATATRANSFER_OK)
        {
            throw new ARDataTransferException(error);
        }
    }

    /**
     * Cancels get available Medias getAvailableMediasSync or getAvailableMediasAsync of the ARDataTransfer MediasDownloader
     * @return ARDATATRANSFER_OK if success, else an {@link ARDATATRANSFER_ERROR_ENUM} error code
     */    
    public ARDATATRANSFER_ERROR_ENUM cancelGetAvailableMedias()
    {
         int result = nativeCancelGetAvailableMedias(nativeManager);
    
        ARDATATRANSFER_ERROR_ENUM error = ARDATATRANSFER_ERROR_ENUM.getFromValue(result);
        
        return error;
    }
    
    /**
     * Deletes a remote media of the ARDataTransfer MediasDownloader
     * @param media The media to delete
     * @return ARDATATRANSFER_OK if success, else an {@link ARDATATRANSFER_ERROR_ENUM} error code
     */
    public ARDATATRANSFER_ERROR_ENUM deleteMedia(ARDataTransferMedia media)
    {
        int result = nativeDeleteMedia(nativeManager, media);
        
        ARDATATRANSFER_ERROR_ENUM error = ARDATATRANSFER_ERROR_ENUM.getFromValue(result);
        
        return error;
    }

    /**
     * Gets a remote media thumbnail. The {@link ARDataTransferMedia} object will be updated with the downloaded thumbnail
     * @param media The media to get the thumbnail
     * @return The thumbnail raw bytes if success, null otherwise
     */
    public byte[] getMediaThumbnail(ARDataTransferMedia media)
    {
        byte[] result = nativeGetMediaThumbnail(nativeManager, media);

        if (result != null)
        {
            media.setThumbail(result);
        }

        return media.getThumbnail();
    }
    
    /**
     * Adds an {@link ARDataTransferMedia} media to the ARDataTransfer MediasDownloader Runnable Queue to start as new Thread
     * @param media ARDataTransferMedia media to add
     * @param progressListener ARDataTransferMediasDownloaderProgressListener progress Listener
     * @param progressArg Object progress Listener arg
     * @param completionListener ARDataTransferMediasDownloaderCompletionListener completion Listener
     * @param completionArg Object completion Listener arg
     * @return void
     * @throws ARDataTransferException if error
     */
    public void addMediaToQueue(ARDataTransferMedia media, ARDataTransferMediasDownloaderProgressListener progressListener, Object progressArg, ARDataTransferMediasDownloaderCompletionListener completionListener, Object completionArg) throws ARDataTransferException
    {
        int result = nativeAddMediaToQueue(nativeManager, media, progressListener, progressArg, completionListener, completionArg);
        
        ARDATATRANSFER_ERROR_ENUM error = ARDATATRANSFER_ERROR_ENUM.getFromValue(result);
        
        if (error != ARDATATRANSFER_ERROR_ENUM.ARDATATRANSFER_OK)
        {
            throw new ARDataTransferException(error);
        }
    }
        
    /**
     * Gets the ARDataTransfer MediasDownloader {@link Runnable} Queue to start as new {@link Thread}
     * @return MediasDownloader Runnable
     */
    public Runnable getDownloaderQueueRunnable()
    {
        Runnable runnable = null;
        
        if (isInit == true)
        {
            runnable = this.downloaderRunnable;
        }
        
        return runnable;
    }
        
    /**
     * Cancels the ARDataTransfer MediasDownloader Runnable Queue Thread
     * @return ARDATATRANSFER_OK if success, else an {@link ARDATATRANSFER_ERROR_ENUM} error code
     */
    public ARDATATRANSFER_ERROR_ENUM cancelQueueThread()
    {
        int result = nativeCancelQueueThread(nativeManager);
        
        ARDATATRANSFER_ERROR_ENUM error = ARDATATRANSFER_ERROR_ENUM.getFromValue(result);
                
        return error;
    }
    
    /*  Static Block */
    static
    {
        nativeStaticInit();
    }        
}

