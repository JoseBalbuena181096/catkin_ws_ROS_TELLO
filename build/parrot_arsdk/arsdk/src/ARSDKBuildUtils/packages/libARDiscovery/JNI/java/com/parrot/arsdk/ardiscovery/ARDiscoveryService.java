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
/*
 * DroneControlService
 *
 *  Created on: 
 *  Author:
 */

package com.parrot.arsdk.ardiscovery;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

import com.parrot.arsdk.arsal.ARSALPrint;

import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;
import android.support.v4.content.LocalBroadcastManager;
import android.os.Build;

public class ARDiscoveryService extends Service
{
    private static final String TAG = ARDiscoveryService.class.getSimpleName();

    /* Native Functions */
    public static native int nativeGetProductID (int product);
    private static native String nativeGetProductName(int product);
    private static native String nativeGetProductPathName(int product);
    private static native int nativeGetProductFromName(String name);
    private static native int nativeGetProductFromProductID(int productID);
    private static native int nativeGetProductFamily(int product);
    
    public enum eARDISCOVERY_SERVICE_EVENT_STATUS
    {
        ARDISCOVERY_SERVICE_EVENT_STATUS_ADD, 
        ARDISCOVERY_SERVICE_EVENT_STATUS_REMOVED,
        ARDISCOVERY_SERVICE_EVENT_STATUS_RESOLVED 
    }
    
    public enum ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_ENUM
    {
        /** jmdns based implementation */
        ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_JMDNS,
        /** Android Network Service Discovery Manager implementation, only available on API 16 */
        ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_NSD,
        /** Custom MDNS-SD implementation */
        ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_MDSNSDMIN;
    }


    /**
     * Constant for devices services list updates notification
     */
    public static final String kARDiscoveryServiceNotificationServicesDevicesListUpdated = "kARDiscoveryServiceNotificationServicesDevicesListUpdated";

    public static String ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT;
    public static String ARDISCOVERY_SERVICE_NET_DEVICE_DOMAIN;
    
    private static native String nativeGetDefineNetDeviceDomain ();
    private static native String nativeGetDefineNetDeviceFormat ();
    
    private HashMap<String, Intent> intentCache;
    
    private ARDiscoveryBLEDiscovery bleDiscovery;
    private ARDiscoveryWifiDiscovery wifiDiscovery;
    private ARDiscoveryNsdPublisher wifiPublisher;
    private ARDiscoveryUsbDiscovery usbDiscovery;
    private final IBinder binder = new LocalBinder();

    private boolean mWifiAvailable;

    /**
     * Selected wifi discovery implementation. Can be set using
     * {@link #setWifiPreferredWifiDiscoveryType(ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_ENUM)}
     */
    private static ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_ENUM wifiDiscoveryType;
    private static Set<ARDISCOVERY_PRODUCT_ENUM> supportedProducts;

    private static boolean usePublisher;

    static
    {
        ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT = nativeGetDefineNetDeviceFormat () + ".";
        ARDISCOVERY_SERVICE_NET_DEVICE_DOMAIN = nativeGetDefineNetDeviceDomain () + ".";
    }

    /**
     * Configuration: set the preferred wifi discovery type.
     * This method must be called before starting ARDiscoveryService.
     * @param discoveryType preferred wifi discovery type
     */
    public static void setWifiPreferredWifiDiscoveryType(ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_ENUM discoveryType)
    {
        synchronized (ARDiscoveryService.class)
        {
            if (wifiDiscoveryType != null)
            {
                throw new RuntimeException("setWifiPreferredWifiDiscoveryType must be called before stating ARDiscoveryService");
            }
            wifiDiscoveryType = discoveryType;
        }
    }

    public static void setSupportedProducts(Set<ARDISCOVERY_PRODUCT_ENUM> products)
    {
        synchronized (ARDiscoveryService.class)
        {
            if (supportedProducts != null)
            {
                throw new RuntimeException("setWifiPreferredWifiDiscoveryType must be called before stating ARDiscoveryService");
            }
            supportedProducts = products;
        }
    }

    public static void setUsePublisher(boolean use)
    {
        usePublisher = use;
    }

    @Override
    public IBinder onBind(Intent intent)
    {
        ARSALPrint.d(TAG, "onBind");

        return binder;
    }  
    
    @Override
    public void onCreate() 
    {
        initIntents();

        synchronized (this.getClass())
        {
            if (wifiDiscoveryType == null)
            {
                wifiDiscoveryType = ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_ENUM.ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_MDSNSDMIN;
            }
            if (supportedProducts == null)
            {
                supportedProducts = EnumSet.allOf(ARDISCOVERY_PRODUCT_ENUM.class);
            }
        }
        // create  and open BLE discovery
        bleDiscovery = new ARDiscoveryBLEDiscoveryImpl(supportedProducts);
        bleDiscovery.open(this, this);
        // create and open wifi discovery and publisher
        initWifiDiscovery();
        // create and open UsbDiscovery
        usbDiscovery = new ARDiscoveryUsbDiscovery();
        usbDiscovery.open(this, this);
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) 
    {
        if (intent == null)
        {
            ARSALPrint.w(TAG, "recreated by the system, don't need! stop it");
            stopSelf();
        }
        return START_STICKY;
    }

    @Override
    public void onDestroy()
    {
        super.onDestroy();

        if (usbDiscovery != null)
        {
            usbDiscovery.close();
            usbDiscovery = null;
        }

        if(bleDiscovery != null)
        {
            bleDiscovery.close();
            bleDiscovery = null;
        }
        
        if(wifiDiscovery != null)
        {
            wifiDiscovery.close();
            wifiDiscovery = null;
        }
        
        if(wifiPublisher != null)
        {
            wifiPublisher.close();
            wifiPublisher = null;
        }

    }
    
    public class LocalBinder extends Binder
    {
        public ARDiscoveryService getService()
        {
            ARSALPrint.d(TAG,"getService");
            return ARDiscoveryService.this;
        }
    }
    
    private void initIntents()
    {
        ARSALPrint.d(TAG,"initIntents");
        
        intentCache = new HashMap<String, Intent>();
        intentCache.put(kARDiscoveryServiceNotificationServicesDevicesListUpdated, new Intent(kARDiscoveryServiceNotificationServicesDevicesListUpdated));
    }
    
    @Override
    public boolean onUnbind(Intent intent)
    {
        ARSALPrint.d(TAG, "onUnbind");
        return true; /* ensures onRebind is called */
    }
    
    @Override
    public void onRebind(Intent intent)
    {
        ARSALPrint.d(TAG, "onRebind");
    }
    
    private synchronized void initWifiDiscovery()
    {
        switch (wifiDiscoveryType)
        {
            case ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_NSD:
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN)
                {
                    wifiDiscovery = new ARDiscoveryNsdDiscovery(supportedProducts);
                    if (usePublisher)
                    {
                        wifiPublisher = new ARDiscoveryNsdPublisher();
                    }
                    ARSALPrint.d(TAG, "Wifi discovery asked is nsd and it will be ARDiscoveryNsdDiscovery");
                }
                else
                {
                    ARSALPrint.w(TAG, "NSD can't run on " + Build.VERSION.SDK_INT + " MdnsSdMin will be used");
                    wifiDiscovery = new ARDiscoveryMdnsSdMinDiscovery(supportedProducts);
                    wifiPublisher = null;
                    ARSALPrint.d(TAG, "Wifi discovery asked is nsd and it will be ARDiscoveryMdnsSdMinDiscovery");
                }
                break;
            case ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_MDSNSDMIN:
                wifiDiscovery = new ARDiscoveryMdnsSdMinDiscovery(supportedProducts);
                ARSALPrint.d(TAG, "Wifi discovery asked is MDSNSDMIN and it will be ARDiscoveryMdnsSdMinDiscovery");
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN)
                {   // uses NSD to publish as MdnsSdMin doesn't supports publishing yet
                    if (usePublisher)
                    {
                        wifiPublisher = new ARDiscoveryNsdPublisher();
                    }
                }
                else
                {
                    wifiPublisher = null;
                }
                break;

            case ARDISCOVERYSERVICE_WIFI_DISCOVERY_TYPE_JMDNS:
            default:
                wifiDiscovery = new ARDiscoveryJmdnsDiscovery(supportedProducts);
                ARSALPrint.d(TAG, "Wifi discovery asked is " + wifiDiscoveryType + " and it will be ARDiscoveryJmdnsDiscovery");
                wifiPublisher = null;
                break;
            
        }

        ARSALPrint.v(TAG, "Opening wifi discovery");
        wifiDiscovery.open(this, this);

        if (wifiPublisher != null)
        {
            ARSALPrint.v(TAG, "Opening wifi publisher");
            wifiPublisher.open(this, this);
        }
        else
        {
            ARSALPrint.i(TAG, "No wifi publisher available");
        }
    }

    /**
     * Tells the service that wifi is present, even though it may not be seeable by connectivity manager.
     * This is usefull for android >= 7 where wifi is not always accessible via connectivity manager (when mobile data is
     * available for exemple), but wifi is present and accessible anyway.
     *
     * @param wifiAvailable : True if we must consider wifi is present and accessible, false if we must check via ConnectivityManager.
     */
    public void wifiAvailable(boolean wifiAvailable) {
        mWifiAvailable = wifiAvailable;
        if (wifiDiscovery != null) {
            wifiDiscovery.wifiAvailable(mWifiAvailable);
        }
    }

    /**
     * Starts discovering all wifi and bluetooth devices
     */
    public synchronized void start()
    {
        ARSALPrint.d(TAG, "Start discoveries");
        bleDiscovery.start();
        wifiDiscovery.start();
        usbDiscovery.start();
    }

    public synchronized void stop()
    {
        ARSALPrint.d(TAG, "Stop discoveries");
        bleDiscovery.stop();
        wifiDiscovery.stop();
        usbDiscovery.stop();
    }
    
    public synchronized void startWifiDiscovering()
    {
        if (wifiDiscovery != null)
        {
            ARSALPrint.d(TAG, "Start wifi discovery");
            wifiDiscovery.start();
        }
    }
    
    public synchronized void stopWifiDiscovering()
    {
        if (wifiDiscovery != null)
        {
            ARSALPrint.d(TAG, "Stop wifi discovery");
            wifiDiscovery.stop();
        }
    }

    public synchronized void startUsbDiscovering() {
        if (usbDiscovery != null) {
            ARSALPrint.d(TAG, "Start Usb discovery");
            usbDiscovery.start();
        }
    }

    public synchronized void stopUsbDiscovering() {
        if (usbDiscovery != null) {
            ARSALPrint.d(TAG, "Stop Usb discovery");
            usbDiscovery.stop();
        }
    }
    
    public synchronized void startBLEDiscovering()
    {
        if (bleDiscovery != null)
        {
            ARSALPrint.d(TAG, "Start BLE discovery");
            bleDiscovery.start();
        }
    }
    
    public synchronized void stopBLEDiscovering()
    {
        if (bleDiscovery != null)
        {
            ARSALPrint.d(TAG, "Stop BLE discovery");
            bleDiscovery.stop();
        }
    }
    
    /* broadcast the deviceServicelist Updated */
    public void broadcastDeviceServiceArrayUpdated ()
    {
        /* broadcast the service add */
        Intent intent = intentCache.get(kARDiscoveryServiceNotificationServicesDevicesListUpdated);
        
        LocalBroadcastManager.getInstance(getApplicationContext()).sendBroadcast(intent);
    }
    
    public List<ARDiscoveryDeviceService> getDeviceServicesArray()
    {
        List<ARDiscoveryDeviceService> deviceServicesArray = new ArrayList<ARDiscoveryDeviceService>();
        if (wifiDiscovery != null)
        {
            deviceServicesArray.addAll(wifiDiscovery.getDeviceServicesArray());
        }
        if (bleDiscovery != null)
        {
            deviceServicesArray.addAll(bleDiscovery.getDeviceServicesArray());
        }
        if (usbDiscovery != null)
        {
            ARDiscoveryDeviceService deviceService = usbDiscovery.getDeviceService();
            if (deviceService != null)
            {
                deviceServicesArray.add(deviceService);
            }
        }
        ARSALPrint.d(TAG,"getDeviceServicesArray: " + deviceServicesArray);

        return deviceServicesArray;
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
        boolean ret = false;
        
        if (wifiPublisher != null)
        {
            ret = wifiPublisher.publishService(product, name, port);
        }
        
        return ret;
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
        boolean ret = false;
        
        if (wifiPublisher != null)
        {
            ret = wifiPublisher.publishService(product_id, name, port);
        }
        
        return ret;
    }

    /**
     * @brief Unpublishes any published service
     */
    public void unpublishServices()
    {
        if (wifiPublisher != null)
        {
            wifiPublisher.unpublishService();
        }
    }

    /**
     * @brief Converts a product enumerator in product ID
     * This function is the only one knowing the correspondance
     * between the enumerator and the products' IDs.
     * @param product The product's enumerator
     * @return The corresponding product ID
     */
    public static int getProductID (ARDISCOVERY_PRODUCT_ENUM product)
    {
        if (product != ARDISCOVERY_PRODUCT_ENUM.eARDISCOVERY_PRODUCT_UNKNOWN_ENUM_VALUE) {
            return nativeGetProductID(product.getValue());
        } else {
            ARSALPrint.e(TAG, "getProductID:Unknown product : " + product);
            return 0;
        }
    }

    /**
     * @brief Converts a product ID in product enumerator
     * This function is the only one knowing the correspondance
     * between the enumerator and the products' IDs.
     * @param productID The product's ID
     * @return The corresponding product enumerator
     */
    public static ARDISCOVERY_PRODUCT_ENUM getProductFromProductID (int productID)
    {
        return ARDISCOVERY_PRODUCT_ENUM.getFromValue (nativeGetProductFromProductID (productID));
    }
    
    /**
     * @brief Converts a product ID in product name
     * This function is the only one knowing the correspondance
     * between the products' IDs and the product names.
     * @param product The product ID
     * @return The corresponding product name
     */
    public static String getProductName (ARDISCOVERY_PRODUCT_ENUM product)
    {
        if (product != ARDISCOVERY_PRODUCT_ENUM.eARDISCOVERY_PRODUCT_UNKNOWN_ENUM_VALUE) {
            return nativeGetProductName(product.getValue());
        } else {
            ARSALPrint.e(TAG, "getProductName:Unknown product : " + product);
            return "UNKNOWN";
        }
    }
    
    /**
     * @brief Converts a product ID in product path name
     * This function is the only one knowing the correspondance
     * between the products' IDs and the product path names.
     * @param product The product ID
     * @return The corresponding product path name
     */
    public static String getProductPathName (ARDISCOVERY_PRODUCT_ENUM product)
    {
        return nativeGetProductPathName (product.getValue());
    }
    
    /**
     * @brief Converts a product product name in product ID
     * This function is the only one knowing the correspondance
     * between the product names and the products' IDs.
     * @param name The product's product name
     * @return The corresponding product ID
     */
    public static ARDISCOVERY_PRODUCT_ENUM getProductFromName(String name)
    {
        int product = nativeGetProductFromName(name);
        
        return ARDISCOVERY_PRODUCT_ENUM.getFromValue(product);
    }
    
    /**
     * @brief Converts a product family in product
     * This function is the only one knowing the correspondance
     * between the products and the products' families.
     * @param product The product
     * @return The corresponding product family
     */
    public static ARDISCOVERY_PRODUCT_FAMILY_ENUM getProductFamily(ARDISCOVERY_PRODUCT_ENUM product)
    {
        int family = nativeGetProductFamily (product.getValue());
        
        return ARDISCOVERY_PRODUCT_FAMILY_ENUM.getFromValue(family);
    }
};

