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

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;

import com.parrot.arsdk.ardiscovery.mdnssdmin.MdnsSdMin;
import com.parrot.arsdk.arsal.ARSALPrint;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Custom mdns-sd implementation of the wifi part of ARDiscoveryService
 */
public class ARDiscoveryMdnsSdMinDiscovery implements ARDiscoveryWifiDiscovery
{

    private static final String TAG = ARDiscoveryMdnsSdMinDiscovery.class.getSimpleName();
    private final MdnsSdMin mdnsSd;
    private ConnectivityManager mConnectivityManager;
    private WifiManager mWifiManager;
    private final IntentFilter networkStateChangedFilter;
    private final IntentFilter wifiStateChangedFilter;
    private final Map<String, ARDiscoveryDeviceService> netDeviceServicesHmap;
    /** Map of device services string to enum */
    private final Map<String, ARDISCOVERY_PRODUCT_ENUM> devicesServices;
    private ARDiscoveryService broadcaster;
    private Context context;
    private WifiManager.MulticastLock multicastLock;
    private boolean started;
    private boolean mWifiAvailable;

    public ARDiscoveryMdnsSdMinDiscovery(Set<ARDISCOVERY_PRODUCT_ENUM> supportedProducts)
    {
        ARSALPrint.d(TAG, "Creating MdsnSd based ARDiscovery");
        // build the list of services to look for
        devicesServices = new HashMap<String, ARDISCOVERY_PRODUCT_ENUM>();
        for (ARDISCOVERY_PRODUCT_ENUM product : ARDISCOVERY_PRODUCT_ENUM.values())
        {
            if (product == ARDISCOVERY_PRODUCT_ENUM.ARDISCOVERY_PRODUCT_MAX ||
                    product == ARDISCOVERY_PRODUCT_ENUM.eARDISCOVERY_PRODUCT_UNKNOWN_ENUM_VALUE)
                continue;
            if (supportedProducts.contains(product))
            {
                String devicesService = String.format(ARDiscoveryService.ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDiscoveryService.getProductID(product));
                devicesService += ARDiscoveryService.ARDISCOVERY_SERVICE_NET_DEVICE_DOMAIN;
                devicesServices.put(devicesService, product);
            }
        }

        netDeviceServicesHmap = new HashMap<String, ARDiscoveryDeviceService>();
        mdnsSd = new MdnsSdMin(devicesServices.keySet().toArray(new String[devicesServices.keySet().size()]), mdsnSdListener);

        // create filter for connectivity change receiver
        networkStateChangedFilter = new IntentFilter();
        networkStateChangedFilter.addAction(ConnectivityManager.CONNECTIVITY_ACTION);
        // create filter for WiFi state change receiver
        wifiStateChangedFilter = new IntentFilter();
        wifiStateChangedFilter.addAction(WifiManager.NETWORK_STATE_CHANGED_ACTION);
    }

    public synchronized void open(ARDiscoveryService broadcaster, Context c)
    {
        ARSALPrint.d(TAG, "Opening MdsnSd based ARDiscovery");
        this.broadcaster = broadcaster;
        this.context = c;
        mConnectivityManager = (ConnectivityManager) context.getSystemService(Context.CONNECTIVITY_SERVICE);
        mWifiManager = (WifiManager) context.getSystemService(android.content.Context.WIFI_SERVICE);
        // create a multicast lock
        multicastLock = mWifiManager.createMulticastLock("ARDiscovery");
    }

    public synchronized void close()
    {
        ARSALPrint.d(TAG, "Closing MdsnSd based ARDiscovery");
        if (started)
        {
            stop();
        }
        mdnsSd.stop();
        this.broadcaster = null;
        this.context = null;
    }

    public synchronized void start()
    {
        if (!started)
        {
            ARSALPrint.d(TAG, "Starting MdsnSd based ARDiscovery");
            if (!multicastLock.isHeld())
            {
                multicastLock.acquire();
            }
            // this is sticky intent, receiver will be called asap
            context.registerReceiver(networkStateIntentReceiver, networkStateChangedFilter);
            context.registerReceiver(wifiStateIntentReceiver, wifiStateChangedFilter);
            started = true;
        }
    }

    public synchronized void stop()
    {
        if (started)
        {
            ARSALPrint.d(TAG, "Stopping MdsnSd based ARDiscovery");
            started = false;
            if (multicastLock.isHeld())
            {
                multicastLock.release();
            }
            context.unregisterReceiver(networkStateIntentReceiver);
            context.unregisterReceiver(wifiStateIntentReceiver);
            mdnsSd.stop();
            netDeviceServicesHmap.clear();
            broadcaster.broadcastDeviceServiceArrayUpdated();
        }
    }

    @Override
    public void wifiAvailable(boolean wifiAvailable)
    {
        ARSALPrint.d(TAG, "wifiAvailable " + wifiAvailable);
        if (mWifiAvailable != wifiAvailable)
        {
            mWifiAvailable = wifiAvailable;
            startOrStopDiscovery();
        }
    }

    @Override
    public List<ARDiscoveryDeviceService> getDeviceServicesArray()
    {
        return new ArrayList<ARDiscoveryDeviceService>(netDeviceServicesHmap.values());
    }

    private final BroadcastReceiver wifiStateIntentReceiver = new BroadcastReceiver()
    {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (intent.getAction().equals(WifiManager.NETWORK_STATE_CHANGED_ACTION))
            {
                Bundle extras = intent.getExtras();
                NetworkInfo networkInfo = (NetworkInfo)extras.get(WifiManager.EXTRA_NETWORK_INFO);
                if (networkInfo != null)
                {
                    NetworkInfo.State state = networkInfo.getState();
                    ARSALPrint.d(TAG, "Receive NETWORK_STATE_CHANGED_ACTION intent, state: " + state);
                    if ((state == NetworkInfo.State.CONNECTED)
                            || ((state == NetworkInfo.State.DISCONNECTED))) {
                        startOrStopDiscovery();
                    }
                }
            }
        }
    };

    private final BroadcastReceiver networkStateIntentReceiver = new BroadcastReceiver()
    {
        @Override
        public void onReceive(Context context, Intent intent)
        {
            if (!mWifiAvailable && intent.getAction().equals(ConnectivityManager.CONNECTIVITY_ACTION))
            {
                ARSALPrint.d(TAG, "Receive CONNECTIVITY_ACTION intent, extras are :");
                Bundle extras = intent.getExtras();
                for (String key : extras.keySet()) {
                    ARSALPrint.d(TAG, "Key : " + key + ", value = " + (extras.get(key) != null ? extras.get(key).toString() : "NULL"));
                }
                ARSALPrint.d(TAG, "End of extras");
                startOrStopDiscovery();
            }
        }
    };

    private void startOrStopDiscovery()
    {
        if ((mConnectivityManager == null)
                || (mWifiManager == null)) {
            // Invalid state
            return;
        }

        NetworkInfo wifi = mConnectivityManager.getNetworkInfo(ConnectivityManager.TYPE_WIFI);
        NetworkInfo eth = mConnectivityManager.getNetworkInfo(ConnectivityManager.TYPE_ETHERNET);
        NetworkInterface netInterface = null;

        // search the network interface with the ip address returned by the wifi manager
        if (mWifiAvailable || ((wifi != null) && wifi.isConnected()))
        {
            WifiInfo wifiInfo = mWifiManager.getConnectionInfo();
            int ipAddressInt = wifiInfo.getIpAddress();
            String  ipAddress = String.format(Locale.US, "%d.%d.%d.%d",
                    (ipAddressInt & 0xff), (ipAddressInt >> 8 & 0xff),
                    (ipAddressInt >> 16 & 0xff), (ipAddressInt >> 24 & 0xff));
            try
            {
                InetAddress addr = InetAddress.getByName(ipAddress);
                Enumeration<NetworkInterface> intfs = NetworkInterface.getNetworkInterfaces();
                while (netInterface == null && intfs.hasMoreElements())
                {
                    NetworkInterface intf = intfs.nextElement();
                    Enumeration<InetAddress> interfaceAddresses = intf.getInetAddresses();
                    while (netInterface == null && interfaceAddresses.hasMoreElements())
                    {
                        InetAddress interfaceAddr =  interfaceAddresses.nextElement();
                        if (interfaceAddr.equals(addr))
                        {
                            netInterface = intf;
                        }
                    }
                }
            }
            catch (Exception e)
            {
                ARSALPrint.e(TAG, "Unable to get the wifi network interface", e);
            }
        }

        // for ethernet, it's not possible to find the correct netInterface. Assume there is
        // a default route don't specify the netinterface
        if (mWifiAvailable
                || ((wifi != null) && wifi.isConnected())
                || ((eth != null) && eth.isConnected()))
        {
            ARSALPrint.d(TAG, "Restaring MdsnSd");
            mdnsSd.stop();
            mdnsSd.start(netInterface);
        }
        else
        {
            ARSALPrint.d(TAG, "Not connected to either wifi or ethernet, need flush list");
            stopWifi();
        }
    }

    private void stopWifi()
    {
        ARSALPrint.d(TAG, "Clearing devices list");
        mdnsSd.stop();
        netDeviceServicesHmap.clear();
        broadcaster.broadcastDeviceServiceArrayUpdated();
    }

    private final MdnsSdMin.Listener mdsnSdListener = new MdnsSdMin.Listener()
    {
        @Override
        public void onServiceAdded(String name, String serviceType, String ipAddress, int port, String[] txts)
        {
            ARSALPrint.d(TAG, "onServiceAdded : " + name + " |type : " + serviceType + " |ip : " + ipAddress + " |port : " + port + " |txts = " + txts);
            String serialNumber = null;
            if (txts != null && txts.length >=1)
            {
                serialNumber = txts[0];
            }
            ARDiscoveryDeviceNetService deviceNetService = new ARDiscoveryDeviceNetService(name, serviceType, ipAddress,
                    port, serialNumber);

            ARDISCOVERY_PRODUCT_ENUM product = devicesServices.get(deviceNetService.getType());
            if (product != null)
            {
                int productID = ARDiscoveryService.nativeGetProductID(product.getValue());
                ARDiscoveryDeviceService deviceService = new ARDiscoveryDeviceService(name, deviceNetService,productID);
                netDeviceServicesHmap.put(deviceService.getName(), deviceService);
                if (broadcaster != null)
                {
                    // got an answer, stop sending queries
                    mdnsSd.cancelSendQueries();
                    // broadcast the new deviceServiceList
                    broadcaster.broadcastDeviceServiceArrayUpdated();
                }
            }
        }

        @Override
        public void onServiceRemoved(String name, String serviceType)
        {
            ARSALPrint.d(TAG, "onServiceRemoved : " + name + " |type : " + serviceType);
            ARDiscoveryDeviceService deviceServiceRemoved = netDeviceServicesHmap.remove(name);
            if (deviceServiceRemoved != null)
            {
                broadcaster.broadcastDeviceServiceArrayUpdated();
            }
        }
    };

}
