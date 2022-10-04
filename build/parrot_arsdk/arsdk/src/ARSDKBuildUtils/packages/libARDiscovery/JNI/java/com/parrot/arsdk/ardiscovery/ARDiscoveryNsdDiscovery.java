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
import java.util.Map;
import java.util.Set;

import com.parrot.arsdk.arsal.ARSALPrint;

import android.net.nsd.NsdServiceInfo;
import android.net.nsd.NsdManager;
import android.content.Context;

/**
 * Android Nsd implementation of the wifi part of ARDiscoveryService
 */
public class ARDiscoveryNsdDiscovery implements ARDiscoveryWifiDiscovery
{
    private static final String TAG = ARDiscoveryNsdDiscovery.class.getSimpleName();

    /** Map of device services string to enum */
    private final Map<String, ARDISCOVERY_PRODUCT_ENUM> devicesServices;
    private final HashMap<String, ARDiscoveryDeviceService> netDeviceServicesHmap;

    // Listeners
    private HashMap<String, NsdManager.DiscoveryListener> mDiscoveryListeners;
    private NsdManager.ResolveListener mResolveListener;

    // NsdManager
    private NsdManager mNsdManager;

    private Boolean isNetDiscovering = false;

    private boolean opened;
    private ARDiscoveryService broadcaster;
    private Context context;

    public ARDiscoveryNsdDiscovery(Set<ARDISCOVERY_PRODUCT_ENUM> supportedProducts)
    {
        opened = false;

        /**
         * devicesServiceArray init
         */
        devicesServices = new HashMap<String, ARDISCOVERY_PRODUCT_ENUM>();
        for (ARDISCOVERY_PRODUCT_ENUM product : ARDISCOVERY_PRODUCT_ENUM.values())
        {
            if (product == ARDISCOVERY_PRODUCT_ENUM.ARDISCOVERY_PRODUCT_MAX ||
                    product == ARDISCOVERY_PRODUCT_ENUM.eARDISCOVERY_PRODUCT_UNKNOWN_ENUM_VALUE)
                continue;
            if (supportedProducts.contains(product))
            {
                String devicesService = String.format(ARDiscoveryService.ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDiscoveryService.getProductID(product));
                devicesServices.put(devicesService, product);
            }
        }
        netDeviceServicesHmap = new HashMap<String, ARDiscoveryDeviceService> ();

        initializeDiscoveryListeners();
        initializeResolveListener();
    }

    @Override
    public synchronized void open(ARDiscoveryService broadcaster, Context c)
    {
        this.broadcaster = broadcaster;
        this.context = c;
        if (opened)
        {
            return;
        }

        netDeviceServicesHmap.clear();
        mNsdManager = (NsdManager) context.getSystemService(Context.NSD_SERVICE);

        opened = true;
    }

    @Override
    public synchronized void close()
    {
        if (! opened)
        {
            return;
        }

        for (String type : devicesServices.keySet())
        {
            ARSALPrint.i(TAG, "Will stop searching for devices of type <" + type + ">");
            mNsdManager.stopServiceDiscovery(mDiscoveryListeners.get(type));
        }

        this.broadcaster = null;
        this.context = null;

        opened = false;
    }


    @Override
    synchronized public void start()
    {
        if (!isNetDiscovering)
        {
            if (mNsdManager != null && mDiscoveryListeners != null)
            {
                
                for (String type : devicesServices.keySet())
                {
                    ARSALPrint.i(TAG, "Will start searching for devices of type <" + type + ">");
                    ARSALPrint.i(TAG, "NsdManager.PROTOCOL_DNS_SD:" + NsdManager.PROTOCOL_DNS_SD +" mDiscoveryListeners.get(type):" + mDiscoveryListeners.get(type));
                    mNsdManager.discoverServices(type, NsdManager.PROTOCOL_DNS_SD, mDiscoveryListeners.get(type));
                }
                isNetDiscovering = true;
            }
        }
    }

    @Override
    synchronized public void stop()
    {
        if (isNetDiscovering)
        {
            if (mNsdManager != null && mDiscoveryListeners != null)
            {
                for (String type : devicesServices.keySet())
                {
                    ARSALPrint.i(TAG, "Will stop searching for devices of type <" + type + ">");
                    mNsdManager.stopServiceDiscovery(mDiscoveryListeners.get(type));
                }
                isNetDiscovering = false;
            }
        }
    }

    @Override
    public void wifiAvailable(boolean wifiAvailable)
    {}

    private void initializeDiscoveryListeners()
    {
        mDiscoveryListeners = new HashMap<String, NsdManager.DiscoveryListener> ();
        for (String type : devicesServices.keySet())
        {
            // Instantiate a new DiscoveryListener
            NsdManager.DiscoveryListener dl = new NsdManager.DiscoveryListener()
                {

                    //  Called as soon as service discovery begins.
                    @Override
                    public void onDiscoveryStarted(String regType)
                    {
                        ARSALPrint.i(TAG, "Service discovery started");
                    }

                    @Override
                    public void onServiceFound(NsdServiceInfo service)
                    {
                        // A service was found!  Do something with it.
                        ARSALPrint.i(TAG, "Service discovery success" + service);
                        boolean validType = devicesServices.containsKey(service.getServiceType());
                        if (validType)
                        {
                            mNsdManager.resolveService(service, mResolveListener);
                        }
                    }

                    @Override
                    public void onServiceLost(NsdServiceInfo service)
                    {
                        // When the network service is no longer available.
                        // Internal bookkeeping code goes here.
                        ARSALPrint.i(TAG, "service lost" + service);

                        /* remove from the deviceServicesHmap */
                        ARDiscoveryDeviceService deviceServiceRemoved = netDeviceServicesHmap.remove(service.getServiceName());

                        if(deviceServiceRemoved != null)
                        {
                            /* broadcast the new deviceServiceList */
                            broadcaster.broadcastDeviceServiceArrayUpdated ();
                        }
                        else
                        {
                            ARSALPrint.e(TAG, "service: "+ service.getServiceName() + " not known");
                        }
                    }

                    @Override
                    public void onDiscoveryStopped(String serviceType)
                    {
                        ARSALPrint.i(TAG, "Discovery stopped: " + serviceType);
                    }

                    @Override
                    public void onStartDiscoveryFailed(String serviceType, int errorCode)
                    {
                        ARSALPrint.e(TAG, "onStartDiscoveryFailed ... Discovery failed: Error code:" + errorCode);
                    }

                    @Override
                    public void onStopDiscoveryFailed(String serviceType, int errorCode)
                    {
                        ARSALPrint.e(TAG, "onStopDiscoveryFailed ... Discovery failed: Error code:" + errorCode);
                    }
                };
            mDiscoveryListeners.put(type, dl);
        }
    }

    private void initializeResolveListener()
    {
        mResolveListener = new NsdManager.ResolveListener()
        {

            @Override
            public void onResolveFailed(NsdServiceInfo serviceInfo, int errorCode)
            {
                // Called when the resolve fails.  Use the error code to debug.
                ARSALPrint.e(TAG, "Resolve failed " + errorCode);
                // if (errorCode == NsdManager.FAILURE_ALREADY_ACTIVE)
                // {
                //     onServiceResolved(serviceInfo);
                // }
            }

            @Override
            public void onServiceResolved(NsdServiceInfo serviceInfo)
            {
                ARSALPrint.i(TAG, "Resolve Succeeded. " + serviceInfo);

                int port = serviceInfo.getPort();
                InetAddress host = serviceInfo.getHost();
                String ip = host.getHostAddress();

                boolean known = netDeviceServicesHmap.containsKey(serviceInfo.getServiceName());

                ARSALPrint.i (TAG, "IP = " + ip + ", Port = " + port + ", Known ? " + known);

                /* add the service if it not known yet*/
                if ((ip != null) && (!known))
                {
                    String serviceInfoType = serviceInfo.getServiceType().substring(1, serviceInfo.getServiceType().length()) + ".";
                    
                    ARDiscoveryDeviceNetService deviceNetService = new ARDiscoveryDeviceNetService(serviceInfo.getServiceName(), serviceInfoType, ip, port, null);

                    ARDISCOVERY_PRODUCT_ENUM product = devicesServices.get(deviceNetService.getType());
                    if (product != null)
                    {
                        /* add the service in the array */
                        int productID = ARDiscoveryService.nativeGetProductID(product.getValue());
                        ARDiscoveryDeviceService deviceService = new ARDiscoveryDeviceService (serviceInfo.getServiceName(), deviceNetService, productID);
                        netDeviceServicesHmap.put(deviceService.getName(), deviceService);

                        /* broadcast the new deviceServiceList */
                        broadcaster.broadcastDeviceServiceArrayUpdated ();
                    }
                    else
                    {
                        ARSALPrint.e(TAG,"Found an unknown service : " + deviceNetService);
                    }
                }
            }
        };
    }

    @Override
    public List<ARDiscoveryDeviceService> getDeviceServicesArray()
    {
        return new ArrayList<ARDiscoveryDeviceService> (netDeviceServicesHmap.values());
    }


}
