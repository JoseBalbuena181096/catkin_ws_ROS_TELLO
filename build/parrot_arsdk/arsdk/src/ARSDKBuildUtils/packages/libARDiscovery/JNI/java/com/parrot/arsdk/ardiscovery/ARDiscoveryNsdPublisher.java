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
package com.parrot.arsdk.ardiscovery;

import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.net.InetAddress;

import com.parrot.arsdk.arsal.ARSALPrint;

import android.net.nsd.NsdServiceInfo;
import android.net.nsd.NsdManager;
import android.content.Context;
import android.util.Log;

/**
 * Android Nsd implementation of the wifi part of ARDiscoveryService
 */
public class ARDiscoveryNsdPublisher implements ARDiscoveryWifiPublisher
{
    private static final String TAG = ARDiscoveryNsdPublisher.class.getSimpleName();

    // NsdManager
    private NsdManager mNsdManager;
    
    // Listeners
    private NsdManager.RegistrationListener mRegistrationListener;

    // Published service info
    private String mServiceName;
    private boolean published;

    private boolean opened;
    private ARDiscoveryService broadcaster;
    private Context context;

    public ARDiscoveryNsdPublisher()
    {
        opened = false;

        initializeRegistrationListener();
    }

    public synchronized void open(ARDiscoveryService broadcaster, Context c)
    {
        this.broadcaster = broadcaster;
        this.context = c;
        if (opened)
        {
            return;
        }

        mNsdManager = (NsdManager) context.getSystemService(Context.NSD_SERVICE);

        opened = true;
        
    }

    public synchronized void close()
    {
        if (! opened)
        {
            return;
        }

        this.broadcaster = null;
        this.context = null;

        opened = false;
    }


    public void update()
    {
        // Nothing here, we always are publishing, regardless of the network type
    }

    /**
     * @brief Publishes a service of the given product
     * This function unpublishes any previously published service
     * @param product The product to publish
     * @param name The name of the service
     * @param port The port of the service
     */
    public boolean publishService(ARDISCOVERY_PRODUCT_ENUM product, String name, int port)
    {
        return publishService(ARDiscoveryService.getProductID(product), name, port);
    }

    /**
     * @brief Publishes a service of the given product_id
     * This function unpublishes any previously published service
     * @param product_id The product ID to publish
     * @param name The name of the service
     * @param port The port of the service
     */
    public boolean publishService(final int product_id, final String name, final int port)
    {
        if (opened)
        {
            unpublishService();

            NsdServiceInfo serviceInfo = new NsdServiceInfo();

            serviceInfo.setServiceName(name);
            serviceInfo.setPort(port);
            String type = String.format(ARDiscoveryService.ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, product_id);
            serviceInfo.setServiceType(type);

            mNsdManager.registerService(serviceInfo, NsdManager.PROTOCOL_DNS_SD, mRegistrationListener);
            
            published = true;
        }

        return published;
    }

    /**
     * @brief Unpublishes any published service
     */
    public void unpublishService()
    {
        if (published)
        {
            mNsdManager.unregisterService(mRegistrationListener);
            mServiceName = null;
            published = false;
        }
    }

    private void initializeRegistrationListener()
    {
        mRegistrationListener = new NsdManager.RegistrationListener()
        {

            @Override
            public void onServiceRegistered(NsdServiceInfo NsdServiceInfo)
            {
                // Save the service name.  Android may have changed it in order to
                // resolve a conflict, so update the name you initially requested
                // with the name Android actually used.
                mServiceName = NsdServiceInfo.getServiceName();
            }

            @Override
            public void onRegistrationFailed(NsdServiceInfo serviceInfo, int errorCode)
            {
                // Registration failed!  Put debugging code here to determine why.
            }

            @Override
            public void onServiceUnregistered(NsdServiceInfo arg0)
            {
                // Service has been unregistered.  This only happens when you call
                // NsdManager.unregisterService() and pass in this listener.
            }

            @Override
            public void onUnregistrationFailed(NsdServiceInfo serviceInfo, int errorCode)
            {
                // Unregistration failed.  Put debugging code here to determine why.
            }
        };
    }
}
