package com.parrot.arsdk.arstream2;

import com.parrot.arsdk.arsal.ARSAL_SOCKET_CLASS_SELECTOR_ENUM;

import com.parrot.mux.Mux;
import android.os.Process;
import android.os.Build;
import android.util.Log;

public class ARStream2Manager
{
    private static final String TAG = ARStream2Manager.class.getSimpleName();
    private final long nativeRef;
    private final Thread networkThread;
    private final Thread outputThread;

    public ARStream2Manager(Mux mux, String canonicalName, int maxPacketSize, int ardiscoveryProductType)
    {
        String friendlyName = Build.MODEL + " " + Build.DEVICE + " " + canonicalName;
        Mux.Ref muxRef = mux.newMuxRef();
        this.nativeRef = nativeMuxInit(muxRef.getCPtr(), canonicalName, friendlyName, maxPacketSize, ardiscoveryProductType);
        muxRef.release();
        this.networkThread = new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_DISPLAY);
                nativeRunNetworkThread(nativeRef);
            }
        }, "ARStream2Stream");
        this.outputThread = new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                Process.setThreadPriority(Process.THREAD_PRIORITY_DISPLAY);
                nativeRunOutputThread(nativeRef);
            }
        }, "ARStream2Filter");
    }

    public ARStream2Manager(String serverAddress, int serverStreamPort, int serverControlPort, int clientStreamPort, int clientControlPort,
                         String canonicalName, int maxPacketSize, ARSAL_SOCKET_CLASS_SELECTOR_ENUM classSelector, int ardiscoveryProductType)
    {
        String friendlyName = Build.MODEL + " " + Build.DEVICE + " " + canonicalName;
        this.nativeRef = nativeNetInit(serverAddress, serverStreamPort, serverControlPort, clientStreamPort, clientControlPort,
                canonicalName, friendlyName, maxPacketSize, classSelector.getValue(), ardiscoveryProductType);
        this.networkThread = new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_DISPLAY);
                nativeRunNetworkThread(nativeRef);
            }
        }, "ARStream2Stream");
        this.outputThread = new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                Process.setThreadPriority(Process.THREAD_PRIORITY_DISPLAY);
                nativeRunOutputThread(nativeRef);
            }
        }, "ARStream2Filter");
    }

    public boolean isValid()
    {
        return nativeRef != 0;
    }

    public void start()
    {
        if (isValid())
        {
            networkThread.start();
            outputThread.start();
        }
        else
        {
            Log.e(TAG, "unable to start, arstream2 manager is not valid! ");
        }
    }

    public boolean stop()
    {
        if (isValid())
        {
            return nativeStop(nativeRef);
        }
        else
        {
            return false;
        }
    }

    public boolean dispose()
    {
        if (isValid())
        {
            try
            {
                networkThread.join();
                outputThread.join();
            } catch (InterruptedException e)
            {
            }
            return nativeFree(nativeRef);
        }
        else
        {
            return false;
        }
    }

    long getNativeRef()
    {
        return nativeRef;
    }

    private native long nativeNetInit(String serverAddress, int serverStreamPort, int serverControlPort,
                                      int clientStreamPort, int clientControlPort,
                                      String canonicalName, String friendlyName, int maxPacketSize,
                                      int classSelector, int ardiscoveryProductType);

    private native long nativeMuxInit(long mux, String canonicalName, String friendlyName,
                                      int maxPacketSize, int ardiscoveryProductType);

    private native boolean nativeStop(long nativeRef);

    private native boolean nativeFree(long nativeRef);

    private native void nativeRunNetworkThread(long nativeRef);

    private native void nativeRunOutputThread(long nativeRef);
}
