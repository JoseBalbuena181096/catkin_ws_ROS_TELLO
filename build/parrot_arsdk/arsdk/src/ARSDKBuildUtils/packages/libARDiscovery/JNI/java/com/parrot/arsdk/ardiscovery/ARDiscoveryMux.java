/*
    Copyright (C) 2016 Parrot SA

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

package com.parrot.arsdk.ardiscovery;

import android.util.Log;

import com.parrot.mux.Mux;

public class ARDiscoveryMux
{

    public interface Listener {
        void onDeviceAdded(String name, int type, String deviceId);

        void onDeviceRemoved();
    }
    public interface ConnectCallback {
        void onConnected(int status, String json);
    }

    private final String TAG = "ARDiscoveryMux";

    private long cPtr;
    private Listener listener;
    private final Object connectLock = new Object();
    private String deviceName;
    private int deviceType;
    private String deviceId;
    private ConnectCallback connectCallback;
    private Mux.Ref muxRef;

    static {
        nativeClInit();
        nativeClInitConnection();
    }

    public ARDiscoveryMux(Mux mux) {
        this.muxRef = mux.newMuxRef();
        this.cPtr = nativeNew(muxRef.getCPtr());
    }

    public boolean isValid() {
        return cPtr != 0;
    }

    public void setListener(Listener listener) {
        this.listener = listener;
        if (listener != null && deviceName != null) {
            listener.onDeviceAdded(deviceName, deviceType, deviceId);
        }
    }


    public int connect(String device, String model, String id, String json, ConnectCallback connectCallback) {
        int ret = -1;
        if (isValid()) {
            synchronized (connectLock) {
                if (this.connectCallback != null) {
                    throw new RuntimeException("Connection already in progress!");
                }
                this.connectCallback = connectCallback;
                long muxConn = nativeNewConnection(muxRef.getCPtr());
                if (nativeSendConnectRequest(muxConn, device, model, id, json) == 0) {
                    try {
                        connectLock.wait();
                    } catch (InterruptedException e) {
                    }
                    ret = 0;
                }
                nativeDisposeConnection(muxConn);
            }
        }
        return ret;
    }

    public void cancelConnect() {
        synchronized (connectLock) {
            connectLock.notifyAll();
        }
    }

    public void destroy() {
        Log.d(TAG, "ARDiscoveryMux destroy");
        synchronized (connectLock) {
            connectLock.notifyAll();
        }
        nativeDispose(cPtr);
        cPtr = 0;
        deviceName = null;
        if (listener != null) {
            listener.onDeviceRemoved();
        }
        listener = null;
        muxRef.release();
        Log.d(TAG, "ARDiscoveryMux destroy done");
    }

    /**
     * Called from native code,
     */
    protected void onDeviceAdded(String name, int type, String id) {
        try {
            Log.d(TAG, "onDeviceAdded " + name + " " + type + " " + id);
            deviceType = type;
            deviceName = name;
            deviceId = id;
            Listener l = listener;
            if (l != null) {
                l.onDeviceAdded(deviceName, deviceType, deviceId);
            }
        } catch (Throwable t) {
            // catch all before returning to native code
            Log.e(TAG, "exception in onDeviceRemoved", t);
        }
    }

    /**
     * Called from native code,
     */
    protected void onDeviceRemoved(String name, int type, String id) {
        try {
            Log.d(TAG, "onDeviceRemoved " + name + "  " + type + " " + id);
            synchronized (connectLock) {
                connectLock.notify();
            }
            deviceName = null;
            Listener l = listener;
            if (l != null) {
                listener.onDeviceRemoved();
            }
        } catch (Throwable t) {
            // catch all before returning to native code
            Log.e(TAG, "exception in onDeviceRemoved", t);
        }
    }

    /**
     * Called from native code,
     */
    protected void onDeviceConnected(int status, String json) {
        try {
            synchronized (connectLock) {
                if (this.connectCallback != null) {
                    Log.d(TAG, "onDeviceConnected " + status + "  " + json);
                    connectCallback.onConnected(status, json);
                    connectCallback = null;
                    connectLock.notify();
                }
            }
        } catch (Throwable t) {
            // catch all before returning to native code
            Log.e(TAG, "exception in onDeviceConnected", t);
        }
    }

    /**
     * Called from native code,
     */
    protected void onReset() {
        try {
            Log.d(TAG, "onReset");
            synchronized (connectLock) {
                connectLock.notify();
            }
            deviceName = null;
            Listener l = listener;
            if (l != null) {
                listener.onDeviceRemoved();
            }
        } catch (Throwable t) {
            // catch all before returning to native code
            Log.e(TAG, "exception in onReset", t);
        }
    }

    private static native void nativeClInit();
    private static native void nativeClInitConnection();

    private native long nativeNew(long muxCPtr);
    private native long nativeNewConnection(long muxCPtr);

    private native int nativeSendConnectRequest(long cPtr, String controllerName, String controllerTpe,
                                                String deviceId, String json);

    private native void nativeDispose(long cPtr);
    private native void nativeDisposeConnection(long cPtr);
};

