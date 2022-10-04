package com.parrot.arsdk.arstream2;

import com.parrot.arsdk.arsal.ARSAL_SOCKET_CLASS_SELECTOR_ENUM;

import android.os.Build;
import android.util.Log;

public class ARStream2Resender
{
    private static final String TAG = ARStream2Resender.class.getSimpleName();
    private final long nativeRef;
    private final ARStream2Manager manager;

    public ARStream2Resender(ARStream2Manager manager, String clientAddress,
                          String mcastAddress, String mcastIfaceAddress,
                          int serverStreamPort, int serverControlPort,
                          int clientStreamPort, int clientControlPort,
                          String canonicalName, ARSAL_SOCKET_CLASS_SELECTOR_ENUM classSelector, int maxNetworkLatency)
    {
        this.manager = manager;
        String friendlyName = Build.MODEL + " " + Build.DEVICE + " " + canonicalName;
        this.nativeRef = nativeInit(manager.getNativeRef(), clientAddress, mcastAddress, mcastIfaceAddress,
                serverStreamPort, serverControlPort, clientStreamPort, clientControlPort,
                canonicalName, friendlyName, classSelector.getValue(), maxNetworkLatency);
    }

    private boolean isValid()
    {
        return nativeRef != 0;
    }

    public void stop()
    {
        if (isValid())
        {
            nativeStop(manager.getNativeRef(), nativeRef);
        }
    }

    private native long nativeInit(long arstream2ManagerNativeRef, String clientAddress,
                                   String mcastAddress, String mcastIfaceAddress,
                                   int serverStreamPort, int serverControlPort,
                                   int clientStreamPort, int clientControlPort,
                                   String canonicalName, String friendlyName,
                                   int classSelector, int maxNetworkLatency);

    private native boolean nativeStop(long arstream2ManagerNativeRef, long nativeRef);

}
