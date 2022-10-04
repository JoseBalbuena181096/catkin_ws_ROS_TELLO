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
package com.parrot.arsdk.arstream;

import java.util.Map;
import java.util.HashMap;

import com.parrot.arsdk.arsal.ARNativeData;
import com.parrot.arsdk.arsal.ARSALPrint;

import com.parrot.arsdk.arnetwork.ARNetworkManager;
import com.parrot.arsdk.arnetwork.ARNetworkIOBufferParam;
import com.parrot.arsdk.arnetwork.ARNetworkIOBufferParamBuilder;

/**
 * Wrapper class for the ARSTREAM_Sender C object.<br>
 * <br>
 * To create an ARStreamSender, the application must provide a suitable
 * <code>ARStreamSenderListener</code> to handle the events.<br>
 * <br>
 * The two ARStreamSender Runnables must be run in independant threads<br>
 */
public class ARStreamSender
{
    private static final String TAG = ARStreamSender.class.getSimpleName ();

    /* *********************** */
    /* INTERNAL REPRESENTATION */
    /* *********************** */

    /**
     * Storage of the C pointer
     */
    private long cSender;

    /**
     * Current frames buffer storage
     */
    private Map<Long, ARNativeData> frames;

    /**
     * Event listener
     */
    private ARStreamSenderListener eventListener;

    /**
     * Check validity before all function calls
     */
    private boolean valid;

    /**
     * Runnable of the data part
     */
    private Runnable dataRunnable;

    /**
     * Runnable of the ack part
     */
    private Runnable ackRunnable;

    /*
     * C #defined constants
     */
    public static final int DEFAULT_MINIMUM_TIME_BETWEEN_RETRIES_MS = nativeGetDefaultMinTimeBetweenRetries();
    public static final int DEFAULT_MAXIUMU_TIME_BETWEEN_RETRIES_MS = nativeGetDefaultMaxTimeBetweenRetries();
    public static final int INFINITE_TIME_BETWEEN_RETRIES = nativeGetInfiniteTimeBetweenRetries();

    /* **************** */
    /* STATIC FUNCTIONS */
    /* **************** */

    /**
     * Static builder for ARNetworkIOBufferParam of data buffers
     * @param bufferId Id of the new buffer
     * @param  maxFragmentSize Maximum size of the fragment to send
     * @param  maxNumberOfFragment Maximum number of the fragment to send
     * @return A new buffer, configured to be an ARStream data buffer
     */
    public static ARNetworkIOBufferParam newDataARNetworkIOBufferParam (int bufferId, int maxFragmentSize, int maxNumberOfFragment) {
        ARNetworkIOBufferParam retParam = new ARNetworkIOBufferParamBuilder (bufferId).build ();
        nativeSetDataBufferParams (retParam.getNativePointer (), bufferId, maxFragmentSize, maxNumberOfFragment);
        retParam.updateFromNative();
        return retParam;
    }

    /**
     * Static builder for ARNetworkIOBufferParam of ack buffers
     * @param bufferId Id of the new buffer
     * @return A new buffer, configured to be an ARStream ack buffer
     */
    public static ARNetworkIOBufferParam newAckARNetworkIOBufferParam (int bufferId) {
        ARNetworkIOBufferParam retParam = new ARNetworkIOBufferParamBuilder (bufferId).build ();
        nativeSetAckBufferParams (retParam.getNativePointer (), bufferId);
        retParam.updateFromNative();
        return retParam;
    }

    /* *********** */
    /* CONSTRUCTOR */
    /* *********** */

    /**
     * Constructor for ARStreamSender object<br>
     * Create a new instance of an ARStreamSender for a given ARNetworkManager,
     * and given buffer ids within the ARNetworkManager.
     * @param netManager The ARNetworkManager to use (must be initialized and valid)
     * @param dataBufferId The id to use for data transferts on network (must be a valid buffer id in <code>netManager</code>
     * @param ackBufferId The id to use for ack transferts on network (must be a valid buffer id in <code>netManager</code>
     * @param theEventListener The event listener to use for this instance
     * @param frameBufferSize Capacity of the internal frameBuffer
     * @param maxFragmentSize Maximum size of the fragment to send
     * @param maxNumberOfFragment Maximum number of the fragment to send
     */
    public ARStreamSender (ARNetworkManager netManager, int dataBufferId, int ackBufferId, ARStreamSenderListener theEventListener, int frameBufferSize,  int maxFragmentSize, int maxNumberOfFragment)
    {
        this.cSender = nativeConstructor (netManager.getManager (), dataBufferId, ackBufferId, frameBufferSize, maxFragmentSize, maxNumberOfFragment);
        if (this.cSender != 0) {
            this.valid = true;
            this.eventListener = theEventListener;
            // HashMap will realloc when over 75% of its capacity, so put a greater capacity than needed to avoid that
            this.frames = new HashMap<Long, ARNativeData> (frameBufferSize * 2);
            this.dataRunnable = new Runnable () {
                public void run () {
                    nativeRunDataThread (ARStreamSender.this.cSender);
                }
            };
            this.ackRunnable = new Runnable () {
                public void run () {
                    nativeRunAckThread (ARStreamSender.this.cSender);
                }
            };
        } else {
            this.valid = false;
            this.dataRunnable = null;
            this.ackRunnable = null;
        }
    }

    /* ********** */
    /* DESTRUCTOR */
    /* ********** */

    /**
     * Destructor<br>
     * This destructor tries to avoid leaks if the object was not disposed
     */
    protected void finalize () throws Throwable {
        try {
            if (valid) {
                ARSALPrint.e (TAG, "Object " + this + " was not disposed !");
                stop ();
                if (! dispose ()) {
                    ARSALPrint.e (TAG, "Unable to dispose object " + this + " ... leaking memory !");
                }
            }
        } finally {
            super.finalize ();
        }
    }

    /* **************** */
    /* PUBLIC FUNCTIONS */
    /* **************** */

    /**
     * Sets the minimum and maximum time between retries.<br>
     * Setting a small retry time might increase reliability, at the cost of network and cpu loads.<br>
     * Setting a high retry time might decrease reliability, but also reduce the network and cpu loads.<br>
     * These rules apply to both the minimum and the maximum time.<br>
     * @note To reset to default values, use DEFAULT_MINIMUM_TIME_BETWEEN_RETRIES_MS and DEFAULT_MAXIMUM_TIME_BETWEEN_RETRIES_MS.<br>
     * @note To disable retries, use INFINITE_TIME_BETWEEN_RETRIES as both minimum and maximum time.<br>
     * @warning Setting a too low maximum wait time might create a very high network bandwidth, and a very high cpu load.
     * @warning Setting a too big minimum wait time (i.e. greater than the time between two flush frames) will effectively disable retries.
     * @param minTimeMs The minimum time between two retries, in miliseconds.
     * @param maxTimeMs The maximum time between two retries, in miliseconds.
     */
    public ARSTREAM_ERROR_ENUM setTimeBetweenRetries(int minTimeMs, int maxTimeMs)
    {
        int err = nativeSetTimeBetweenRetries(cSender, minTimeMs, maxTimeMs);
        return ARSTREAM_ERROR_ENUM.getFromValue(err);
    }

    /**
     * Checks if the current manager is valid.<br>
     * A valid manager is a manager which can be used to send video frames.
     * @return The validity of the manager.
     */
    public boolean isValid()
    {
        return this.valid;
    }

    /**
     * Stops the internal threads of the ARStreamSender.<br>
     * Calling this function allow the ARStreamSender Runnables to end
     */
    public void stop () {
        nativeStop (cSender);
    }

    /**
     * Deletes the ARStreamSender.<br>
     * This function should only be called after <code>stop()</code><br>
     * <br>
     * Warning: If this function returns <code>false</code>, then the ARStreamSender was not disposed !
     * @return <code>true</code> if the Runnables are not running.<br><code>false</code> if the ARStreamSender could not be disposed now.
     */
    public boolean dispose () {
        boolean ret = nativeDispose (cSender);
        if (ret) {
            this.valid = false;
        }
        return ret;
    }

    /**
     * Sends a new frame with the ARStreamSender.<br>
     * The application should not modify <code>frame</code> until an event
     * is called for it (either cancel or sent).<br>
     * Modifying <code>frame</code> before that leads to undefined behavior.
     * @param frame The frame to send
     * @param flush If active, the ARStreamSender will cancel any remaining prevous frame, and start sending this one immediately
     */
    public ARSTREAM_ERROR_ENUM sendNewFrame (ARNativeData frame, boolean flush) {
        ARSTREAM_ERROR_ENUM err = ARSTREAM_ERROR_ENUM.ARSTREAM_ERROR_BAD_PARAMETERS;
        synchronized (this)
        {
            frames.put(frame.getData(), frame);
        }
        int intErr = nativeSendNewFrame(cSender, frame.getData(), frame.getDataSize(), flush);
        err = ARSTREAM_ERROR_ENUM.getFromValue(intErr);
        if (err != ARSTREAM_ERROR_ENUM.ARSTREAM_OK)
        {
            synchronized (this)
            {
                frames.remove(frame.getData());
            }
        }
        return err;
    }

    /**
     * Flushes all currently queued frames on the ARStreamSender.
     */
    public ARSTREAM_ERROR_ENUM flushFrameQueue ()
    {
        int intErr = nativeFlushFrameQueue(cSender);
        return ARSTREAM_ERROR_ENUM.getFromValue(intErr);
    }

    /**
     * Gets the Data Runnable<br>
     * Each runnable must be run exactly ONE time
     * @return The Data Runnable
     */
    public Runnable getDataRunnable () {
        return dataRunnable;
    }

    /**
     * Gets the Ack Runnable<br>
     * Each runnable must be run exactly ONE time
     * @return The Ack Runnable
     */
    public Runnable getAckRunnable () {
        return ackRunnable;
    }

    /**
     * Gets the estimated efficiency of the network link<br>
     * This methods gives the percentage of useful data packets in the
     * data stream.
     * @return Estimated network link efficiency (0.0-1.0)
     */
    public float getEstimatedEfficiency () {
        return nativeGetEfficiency (cSender);
    }

    /**
     * Adds a new ARStreamFilter to the filter chain (at the end).<br>
     * This function can only be called on non-started instances.
     * @param filter The filter to add.
     * @return ARSTREAM_OK if the filter was added.
     */
    public ARSTREAM_ERROR_ENUM addFilter (ARStreamFilter filter) {
        if (filter != null) {
            return ARSTREAM_ERROR_ENUM.getFromValue(nativeAddFilter(cSender, filter.getFilterPointer()));
        }
        return ARSTREAM_ERROR_ENUM.ARSTREAM_ERROR_BAD_PARAMETERS;
    }

    /* ***************** */
    /* PRIVATE FUNCTIONS */
    /* ***************** */

    /**
     * Callback wrapper for the listener
     */
    private void callbackWrapper (int istatus, long ndPointer, int ndSize) {
        ARSTREAM_SENDER_STATUS_ENUM status = ARSTREAM_SENDER_STATUS_ENUM.getFromValue (istatus);
        if (status == null) {
            return;
        }
        switch (status) {
            case ARSTREAM_SENDER_STATUS_FRAME_SENT:
            case ARSTREAM_SENDER_STATUS_FRAME_CANCEL:
                ARNativeData data = null;
                synchronized (this)
                {
                    data = frames.remove (ndPointer);
                }
                eventListener.didUpdateFrameStatus(status, data);
                break;

            default:
                ARSALPrint.e (TAG, "Unknown status :" + status);
                break;
        }
    }

    /* **************** */
    /* NATIVE FUNCTIONS */
    /* **************** */

    /**
     * Sets an ARNetworkIOBufferParams internal values to represent an
     * ARStream data buffer.
     * @param cParams C-Pointer to the ARNetworkIOBufferParams internal representation
     * @param id The id of the param
     * @param maxFragmentSize Maximum size of the fragment to send
     * @param maxNumberOfFragment Maximum number of the fragment to send
     */
    private native static void nativeSetDataBufferParams (long cParams, int id, int maxFragmentSize, int maxNumberOfFragment);

    /**
     * Sets an ARNetworkIOBufferParams internal values to represent an
     * ARStream ack buffer.
     * @param cParams C-Pointer to the ARNetworkIOBufferParams internal representation
     * @param id The id of the param
     */
    private native static void nativeSetAckBufferParams (long cParams, int id);

    /**
     * Constructor in native memory space<br>
     * This function created a C-Managed ARSTREAM_Sender object
     * @param cNetManager C-Pointer to the ARNetworkManager internal object
     * @param dataBufferId id of the data buffer to use
     * @param ackBufferId id of the ack buffer to use
     * @param nbFramesToBuffer number of frames that the object can keep in its internal buffer
     * @param maxNumberOfFragment Maximum number of the fragment to send
     * @return C-Pointer to the ARSTREAM_Sender object (or null if any error occured)
     */
    private native long nativeConstructor (long cNetManager, int dataBufferId, int ackBufferId, int nbFramesToBuffer, int maxFragmentSize, int maxNumberOfFragment);

    /**
     * Entry point for the data thread<br>
     * This function never returns until <code>stop</code> is called
     * @param cSender C-Pointer to the ARSTREAM_Sender C object
     */
    private native void nativeRunDataThread (long cSender);

    /**
     * Entry point for the ack thread<br>
     * This function never returns until <code>stop</code> is called
     * @param cSender C-Pointer to the ARSTREAM_Sender C object
     */
    private native void nativeRunAckThread (long cSender);

    /**
     * Stops the internal thread loops
     * @param cSender C-Pointer to the ARSTREAM_Sender C object
     */
    private native void nativeStop (long cSender);

    /**
     * Marks the sender as invalid and frees it if needed
     * @param cSender C-Pointer to the ARSTREAM_Sender C object
     */
    private native boolean nativeDispose (long cSender);

    /**
     * Gets the estimated efficiency of the sender
     * @param cSender C-Pointer to the ARSTREAM_Sender C object
     */
    private native float nativeGetEfficiency (long cSender);

    /**
     * Tries to send a new frame.
     * @param cSender C-Pointer to the ARSTREAM_Sender C object
     * @param frameBuffer The c pointer to the frame.
     * @param frameSize The size in bytes of the frame.
     * @param flushPreviousFrame Whether to flush any queued frames or not.
     */
    private native int nativeSendNewFrame (long cSender, long frameBuffer, int frameSize, boolean flushPreviousFrame);

    /**
     * Flushes the frames queue.
     * @param cSender C-Pointer to the ARSTREAM_Sender C object
     */
    private native int nativeFlushFrameQueue(long cSender);

    /**
     * Sets the min/max times between retries.
     * @param cSender C-Pointer to the ARSTREAM_Sender C object
     * @param min minimum time between retries, in ms
     * @param max maximum time between retries, in ms
     */
    private native int nativeSetTimeBetweenRetries(long cSender, int min, int max);

    /**
     * Adds a filter to the Sender
     * @param cSender C-Pointer to the ARSTREAM_Sender C object
     * @param cFilter C-Pointer to the ARSTREAM_Filter C object
     */
    private native int nativeAddFilter (long cSender, long cFilter);

    /**
     * Initializes global static references in native code
     */
    private native static void nativeInitClass ();

    /*
     * Getters for C #defines
     */
    private native static int nativeGetDefaultMinTimeBetweenRetries();
    private native static int nativeGetDefaultMaxTimeBetweenRetries();
    private native static int nativeGetInfiniteTimeBetweenRetries();

    /* *********** */
    /* STATIC BLOC */
    /* *********** */
    static {
        nativeInitClass ();
    }
}
