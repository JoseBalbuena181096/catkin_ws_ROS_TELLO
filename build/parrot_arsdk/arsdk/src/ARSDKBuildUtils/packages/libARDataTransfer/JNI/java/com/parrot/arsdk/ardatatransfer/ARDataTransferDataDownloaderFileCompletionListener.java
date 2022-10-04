
package com.parrot.arsdk.ardatatransfer;

/**
 * ARDataTransfer Uploader CompletionListener
 * @author david.flattin.ext@parrot.com
 * @date 18/11/2014
 */


public interface ARDataTransferDataDownloaderFileCompletionListener
{
    /**
     * Gives the ARDataTransferDataDownloader file complete status
     * @param arg Object complete Listener arg
     * @param fileName file name
     * @param error ARDATATRANSFER_OK if success, else an {@link ARDATATRANSFER_ERROR_ENUM} error code
     * @return void
     */
    void didDataDownloaderFileComplete(Object arg, String fileName, ARDATATRANSFER_ERROR_ENUM error);
}
