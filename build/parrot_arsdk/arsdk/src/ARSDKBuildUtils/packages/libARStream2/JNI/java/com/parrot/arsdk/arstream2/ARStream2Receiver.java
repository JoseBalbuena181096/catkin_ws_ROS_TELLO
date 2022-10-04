package com.parrot.arsdk.arstream2;

import android.util.Log;

import com.parrot.arsdk.arsal.ARSALPrint;

import java.nio.ByteBuffer;

public class ARStream2Receiver
{
    private static final String TAG = ARStream2Receiver.class.getSimpleName();

    private final long arstream2ManagerNativeRef;
    private final long nativeRef;
    private final ARStream2ReceiverListener listener;
    private ByteBuffer[] buffers;

    public ARStream2Receiver(ARStream2Manager manager, ARStream2ReceiverListener listener)
    {
        this.listener = listener;
        this.arstream2ManagerNativeRef = manager.getNativeRef();
        this.nativeRef = nativeInit();
    }

    public boolean isValid()
    {
        return arstream2ManagerNativeRef != 0;
    }

    public void start()
    {
        if (isValid())
        {
            nativeStart(arstream2ManagerNativeRef, nativeRef);
        }
        else
        {
            Log.e(TAG, "unable to start, resender is not valid! ");
        }

    }

    public void stop()
    {
        if (isValid())
        {
            nativeStop(arstream2ManagerNativeRef);
        }
    }

    public void dispose()
    {
        buffers = null;
        nativeFree(nativeRef);
    }

    /**
     * spsPpsCallback wrapper for the listener
     */
    private int onSpsPpsReady(ByteBuffer sps, ByteBuffer pps)
    {
        try
        {
            this.buffers = listener.onSpsPpsReady(sps, pps);
        } catch (Throwable t)
        {
            ARSALPrint.e(TAG, "Exception in onSpsPpsReady" + t.getMessage());
            return -1;
        }
        if (this.buffers != null)
        {
            return 0;
        }
        return -1;
    }

    private int getFreeBufferIdx()
    {
        try
        {
            int bufferIdx = listener.getFreeBuffer();
            if (bufferIdx >= 0)
            {
                return bufferIdx;
            }
            ARSALPrint.e(TAG, "\tNo more free buffers");
        } catch (Throwable t)
        {
            ARSALPrint.e(TAG, "Exception in getFreeBufferIdx" + t.getMessage());
        }
        return -1;
    }

    private ByteBuffer getBuffer(int bufferIdx)
    {
        try
        {
            return buffers[bufferIdx];
        } catch (Throwable t)
        {
            ARSALPrint.e(TAG, "Exception in getBuffer" + t.getMessage());
        }

        return null;
    }

    private int onBufferReady(int bufferIdx, int auSize, long metadata, int metadataSize,
                              long userdata, int userdataSize,
                              long auTimestamp, long auTimestampRaw, long auTimestampLocal,
                              int iAuSyncType, int isComplete, int hasErrors, int isRef)
    {
        ARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE_ENUM auSyncType = ARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE_ENUM.getFromValue(iAuSyncType);
        if (auSyncType == null)
        {
            ARSALPrint.e(TAG, "Bad au sync type : " + iAuSyncType);
            return -1;
        }

        try
        {
            ByteBuffer buffer = this.buffers[bufferIdx];
            //buffer.limit(auSize);
            buffer.position(auSize);
            listener.onBufferReady(bufferIdx, metadata, metadataSize, userdata, userdataSize,
                                   auTimestamp, auTimestampRaw, auTimestampLocal,
                                   auSyncType, isComplete, hasErrors, isRef);
            return 0;
        } catch (Throwable t)
        {
            ARSALPrint.e(TAG, "Exception in onBufferReady" + t.getMessage());
        }
        return -1;
    }

    private native long nativeInit();
    private native void nativeFree(long nativeRef);
    private native boolean nativeStart(long arstream2ManagerNativeRef, long nativeRef);
    private native boolean nativeStop(long arstream2ManagerNativeRef);
    private native static void nativeInitClass();
    static
    {
        nativeInitClass();
    }
 }
