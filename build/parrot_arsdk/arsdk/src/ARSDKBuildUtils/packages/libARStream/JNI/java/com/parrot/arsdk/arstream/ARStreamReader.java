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

import com.parrot.arsdk.arsal.ARNativeData;
import com.parrot.arsdk.arsal.ARSALPrint;

import com.parrot.arsdk.arnetwork.ARNetworkManager;
import com.parrot.arsdk.arnetwork.ARNetworkIOBufferParam;
import com.parrot.arsdk.arnetwork.ARNetworkIOBufferParamBuilder;

/**
 * Wrapper class for the ARSTREAM_Reader C object.<br>
 * <br>
 * To create an ARStreamReader, the application must provide a suitable
 * <code>ARStreamReaderListener</code> to handle the events.<br>
 * <br>
 * The two ARStreamReader Runnables must be run in independant threads<br>
 */
public class ARStreamReader
{
    private static final String TAG = ARStreamReader.class.getSimpleName ();

    /* *********************** */
    /* INTERNAL REPRESENTATION */
    /* *********************** */

    /**
     * Storage of the C pointer
     */
    private long cReader;

    /**
     * Current frame buffer storage
     */
    private ARNativeData currentFrameBuffer;

    /**
     * Previous frame buffer storage (only for "too small" events)
     */
    private ARNativeData previousFrameBuffer;

    /**
     * Event listener
     */
    private ARStreamReaderListener eventListener;

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

    /* **************** */
    /* PUBLIC CONSTANTS */
    /* **************** */
    public static final int DEFAULT_MAX_ACK_INTERVAL = nativeGetDefaultMaxAckInterval();

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
     * Constructor for ARStreamReader object<br>
     * Create a new instance of an ARStreamReader for a given ARNetworkManager,
     * and given buffer ids within the ARNetworkManager.
     * @param netManager The ARNetworkManager to use (must be initialized and valid)
     * @param dataBufferId The id to use for data transferts on network (must be a valid buffer id in <code>netManager</code>
     * @param ackBufferId The id to use for ack transferts on network (must be a valid buffer id in <code>netManager</code>
     * @param initialFrameBuffer The first frame buffer to use
     * @param theEventListener The event listener to use for this instance
     * @param maxFragmentSize Maximum allowed size for a video data fragment. Video frames larger that will be fragmented.
     */
    public ARStreamReader (ARNetworkManager netManager, int dataBufferId, int ackBufferId, ARNativeData initialFrameBuffer, ARStreamReaderListener theEventListener, int maxFragmentSize, int maxAckInterval)
    {
        this.cReader = nativeConstructor (netManager.getManager (), dataBufferId, ackBufferId, initialFrameBuffer.getData (), initialFrameBuffer.getCapacity (), maxFragmentSize, maxAckInterval);
        if (this.cReader != 0) {
            this.valid = true;
            this.eventListener = theEventListener;
            this.currentFrameBuffer = initialFrameBuffer;
            this.dataRunnable = new Runnable () {
                    public void run () {
                        nativeRunDataThread (ARStreamReader.this.cReader);
                    }
                };
            this.ackRunnable = new Runnable () {
                    public void run () {
                        nativeRunAckThread (ARStreamReader.this.cReader);
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
     * Checks if the current manager is valid.<br>
     * A valid manager is a manager which can be used to receive video frames.
     * @return The validity of the manager.
     */
    public boolean isValid()
    {
        return this.valid;
    }

    /**
     * Stops the internal threads of the ARStreamReader.<br>
     * Calling this function allow the ARStreamReader Runnables to end
     */
    public void stop () {
        nativeStop (cReader);
    }

    /**
     * Deletes the ARStreamReader.<br>
     * This function should only be called after <code>stop()</code><br>
     * <br>
     * Warning: If this function returns <code>false</code>, then the ARStreamReader was not disposed !
     * @return <code>true</code> if the Runnables are not running.<br><code>false</code> if the ARStreamReader could not be disposed now.
     */
    public boolean dispose () {
        boolean ret = nativeDispose (cReader);
        if (ret) {
            this.valid = false;
        }
        return ret;
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
        return nativeGetEfficiency (cReader);
    }

    /**
     * Adds a new ARStreamFilter to the filter chain (at the end).<br>
     * This function can only be called on non-started instances.
     * @param filter The filter to add.
     * @return ARSTREAM_OK if the filter was added.
     */
    public ARSTREAM_ERROR_ENUM addFilter (ARStreamFilter filter) {
        if (filter != null) {
            return ARSTREAM_ERROR_ENUM.getFromValue(nativeAddFilter(cReader, filter.getFilterPointer()));
        }
        return ARSTREAM_ERROR_ENUM.ARSTREAM_ERROR_BAD_PARAMETERS;
    }

    /* ***************** */
    /* PRIVATE FUNCTIONS */
    /* ***************** */

    /**
     * Callback wrapper for the listener
     */
    private long[] callbackWrapper (int icause, long ndPointer, int ndSize, boolean isFlush, int nbSkip, int newBufferCapacity) {
        ARSTREAM_READER_CAUSE_ENUM cause = ARSTREAM_READER_CAUSE_ENUM.getFromValue (icause);
        if (cause == null) {
            ARSALPrint.e (TAG, "Bad cause : " + icause);
            return null;
        }

        if ((currentFrameBuffer == null || ndPointer != currentFrameBuffer.getData()) &&
            (previousFrameBuffer == null || ndPointer != previousFrameBuffer.getData())) {
            ARSALPrint.e (TAG, "Bad frame buffer");
            return null;
        }

        switch (cause) {
        case ARSTREAM_READER_CAUSE_FRAME_COMPLETE:
            currentFrameBuffer.setUsedSize(ndSize);
            currentFrameBuffer = eventListener.didUpdateFrameStatus (cause, currentFrameBuffer, isFlush, nbSkip, newBufferCapacity);
            break;
        case ARSTREAM_READER_CAUSE_FRAME_TOO_SMALL:
            previousFrameBuffer = currentFrameBuffer;
            currentFrameBuffer = eventListener.didUpdateFrameStatus (cause, currentFrameBuffer, isFlush, nbSkip, newBufferCapacity);
            break;
        case ARSTREAM_READER_CAUSE_COPY_COMPLETE:
            eventListener.didUpdateFrameStatus (cause, previousFrameBuffer, isFlush, nbSkip, newBufferCapacity);
            previousFrameBuffer = null;
            break;
        case ARSTREAM_READER_CAUSE_CANCEL:
            eventListener.didUpdateFrameStatus (cause, currentFrameBuffer, isFlush, nbSkip, newBufferCapacity);
            currentFrameBuffer = null;
            break;
        default:
            ARSALPrint.e (TAG, "Unknown cause :" + cause);
            break;
        }
        if (currentFrameBuffer != null) {
            long retVal[] = { currentFrameBuffer.getData(), currentFrameBuffer.getCapacity() };
            return retVal;
        } else {
            long retVal[] = { 0, 0 };
            return retVal;
        }
    }

    /* **************** */
    /* NATIVE FUNCTIONS */
    /* **************** */

    /**
     * Get the default value for the maxAckInterval contructor parameter.
     */
    private native static int nativeGetDefaultMaxAckInterval ();

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
     * This function created a C-Managed ARSTREAM_Reader object
     * @param cNetManager C-Pointer to the ARNetworkManager internal object
     * @param dataBufferId id of the data buffer to use
     * @param ackBufferId id of the ack buffer to use
     * @param frameBuffer C-Pointer to the initial frame buffer
     * @param frameBufferSize size of the initial frame buffer
     * @param maxFragmentSize Maximum size of the fragment to send
     * @param maxAckInterval Maximum duration without sending an ACK.
     * @return C-Pointer to the ARSTREAM_Reader object (or null if any error occured)
     */
    private native long nativeConstructor (long cNetManager, int dataBufferId, int ackBufferId, long frameBuffer, int frameBufferSize, int maxFragmentSize, int maxAckInterval);

    /**
     * Entry point for the data thread<br>
     * This function never returns until <code>stop</code> is called
     * @param cReader C-Pointer to the ARSTREAM_Reader C object
     */
    private native void nativeRunDataThread (long cReader);

    /**
     * Entry point for the ack thread<br>
     * This function never returns until <code>stop</code> is called
     * @param cReader C-Pointer to the ARSTREAM_Reader C object
     */
    private native void nativeRunAckThread (long cReader);

    /**
     * Stops the internal thread loops
     * @param cReader C-Pointer to the ARSTREAM_Reader C object
     */
    private native void nativeStop (long cReader);

    /**
     * Marks the reader as invalid and frees it if needed
     * @param cReader C-Pointer to the ARSTREAM_Reader C object
     */
    private native boolean nativeDispose (long cReader);

    /**
     * Gets the estimated efficiency of the reader
     * @param cReader C-Pointer to the ARSTREAM_Reader C object
     */
    private native float nativeGetEfficiency (long cReader);

    /**
     * Adds a filter to the Reader
     * @param cReader C-Pointer to the ARSTREAM_Reader C object
     * @param cFilter C-Pointer to the ARSTREAM_Filter C object
     */
    private native int nativeAddFilter (long cReader, long cFilter);

    /**
     * Initializes global static references in native code
     */
    private native static void nativeInitClass ();

    /* *********** */
    /* STATIC BLOC */
    /* *********** */
    static {
        nativeInitClass ();
    }
}
