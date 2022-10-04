package com.parrot.arsdk.ardiscovery;

import android.bluetooth.BluetoothDevice;
import android.content.Context;

import com.parrot.arsdk.arnetworkal.ARNETWORKAL_ERROR_ENUM;
import com.parrot.arsdk.arnetworkal.ARNetworkALManager;
import com.parrot.arsdk.arsal.ARSALPrint;
import com.parrot.mux.Mux;

public class ARDiscoveryDevice
{
    private static String TAG = "ARDiscoveryDevice";

    public static int ROLLINGSPIDER_CONTROLLER_TO_DEVICE_NONACK_ID = 0;
    public static int ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID = 0;
    public static int ROLLINGSPIDER_CONTROLLER_TO_DEVICE_EMERGENCY_ID = 0;
    public static int ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID = 0;
    public static int ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID = 0;

    private static native void nativeStaticInit ();

    private static native int nativeGetCToDNonAckId ();
    private static native int nativeGetCToDAckId ();
    private static native int nativeGetCToDEmergencyId ();
    private static native int nativeGetDToCNavDataId ();
    private static native int nativeGetDToCEventId ();


    private native long nativeNew() throws ARDiscoveryException;
    private native void nativeDelete(long jARDiscoveryDevice);
    
    private native int nativeInitWifi(long jARDiscoveryDevice, int product, String name, String address, int port);
    private native int nativeInitBLE(long jARDiscoveryDevice, int product, BLEPart blePart);
    private native int nativeInitUsb(long jARDiscoveryDevice, int product, long mux);
    
    private long nativeARDiscoveryDevice;
    private boolean initOk;

    BLEPart blePart;

    static
    {
        nativeStaticInit();

        ROLLINGSPIDER_CONTROLLER_TO_DEVICE_NONACK_ID = nativeGetCToDNonAckId ();
        ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID = nativeGetCToDAckId ();
        ROLLINGSPIDER_CONTROLLER_TO_DEVICE_EMERGENCY_ID = nativeGetCToDEmergencyId ();
        ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID = nativeGetDToCNavDataId ();
        ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID = nativeGetDToCEventId ();
    }


    /**
     * Constructor
     */
    public ARDiscoveryDevice() throws ARDiscoveryException
    {
        ARSALPrint.d(TAG,"ARDiscoveryDevice ...");
        
        initOk = false;
        nativeARDiscoveryDevice = nativeNew();
        if (nativeARDiscoveryDevice != 0)
        {
            initOk = true;
        }
    }

    public ARDiscoveryDevice(Context ctx, ARDiscoveryDeviceService service) throws ARDiscoveryException
    {
        this();
        ARDISCOVERY_PRODUCT_ENUM product = ARDiscoveryService.getProductFromProductID(service.getProductID());
        ARDISCOVERY_ERROR_ENUM err;
        switch (service.getNetworkType()) {
            case ARDISCOVERY_NETWORK_TYPE_NET:
                ARDiscoveryDeviceNetService ns = (ARDiscoveryDeviceNetService)service.getDevice();
                err = initWifi(product, service.getName(), ns.getIp(), ns.getPort());
                break;
            case ARDISCOVERY_NETWORK_TYPE_BLE:
                ARDiscoveryDeviceBLEService bs = (ARDiscoveryDeviceBLEService)service.getDevice();
                err = initBLE(product, ctx.getApplicationContext(), bs.getBluetoothDevice());
                break;
            case ARDISCOVERY_NETWORK_TYPE_USBMUX:
                err = initUSB(product, UsbAccessoryMux.get(ctx.getApplicationContext()).getMux());
                break;
            default:
                err = ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_ERROR_BAD_PARAMETER;
                break;
        }
        if (err != ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_OK) {
            throw new ARDiscoveryException(err.getValue());
        }
    }


    /**
     * Dispose
     */
    public void dispose()
    {
        ARSALPrint.d(TAG,"dispose ...");
        
        ARDISCOVERY_ERROR_ENUM error = ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_OK;
        synchronized (this)
        {
            if(initOk == true)
            {
                nativeDelete(nativeARDiscoveryDevice);
                nativeARDiscoveryDevice = 0;
                initOk = false;
            }
        }
    }

    /**
     * Destructor
     */
    public void finalize () throws Throwable
    {
        try
        {
            dispose ();
        }
        finally
        {
            super.finalize ();
        }
    }
    
    
    public ARDISCOVERY_ERROR_ENUM initWifi (ARDISCOVERY_PRODUCT_ENUM product, String name, String address, int port)
    {
        ARSALPrint.d(TAG,"initWifi ...");
        
        ARDISCOVERY_ERROR_ENUM error = ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_OK;
        synchronized (this)
        {
            if (initOk == true)
            {
                if (product != null)
                {
                    int nativeError = nativeInitWifi(nativeARDiscoveryDevice, product.getValue() , name, address, port);
                    error = ARDISCOVERY_ERROR_ENUM.getFromValue(nativeError);
                }
                else
                {
                    error = ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_ERROR_BAD_PARAMETER;
                }
            }
        }

        return error;
    }

    public ARDISCOVERY_ERROR_ENUM initBLE (ARDISCOVERY_PRODUCT_ENUM product, Context context, BluetoothDevice bleDevice)
    {
        ARDISCOVERY_ERROR_ENUM error = ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_OK;
        synchronized (this)
        {
            if (initOk == true)
            {
                if (product != null)
                {
                    //ARNetworkALManager alManager = new ARNetworkALManager();

                    blePart = new BLEPart();

                    int ackOffset = (ARNetworkALManager.ARNETWORKAL_MANAGER_BLE_ID_MAX / 2);
                    blePart.bleNotificationIDs = new int[]{ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID, ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID, (ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID + ackOffset) ,(ROLLINGSPIDER_CONTROLLER_TO_DEVICE_EMERGENCY_ID + ackOffset)};
                    blePart.context = context;
                    blePart.bleDevice = bleDevice;

                    int nativeError = nativeInitBLE(nativeARDiscoveryDevice, product.getValue(), blePart);
                    error = ARDISCOVERY_ERROR_ENUM.getFromValue(nativeError);
                }
                else
                {
                    error = ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_ERROR_BAD_PARAMETER;
                }
            }
        }

        return error;
    }

    public ARDISCOVERY_ERROR_ENUM initUSB (ARDISCOVERY_PRODUCT_ENUM product, Mux mux)
    {
        ARDISCOVERY_ERROR_ENUM error = ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_OK;
        synchronized (this)
        {
            if (initOk == true)
            {
                if (product != null)
                {
                    Mux.Ref muxRef = mux.newMuxRef();
                    int nativeError = nativeInitUsb(nativeARDiscoveryDevice, product.getValue(), muxRef.getCPtr());
                    muxRef.release();
                    error = ARDISCOVERY_ERROR_ENUM.getFromValue(nativeError);
                }
                else
                {
                    error = ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_ERROR_BAD_PARAMETER;
                }
            }
        }

        return error;
    }

    public long getNativeDevice ()
    {
        return nativeARDiscoveryDevice;
    }

    private class BLEPart
    {
        Context context;
        public int bleNotificationIDs[];
        public BluetoothDevice bleDevice;
        public ARNetworkALManager alManager;

        public BLEPart()
        {

        }

        public long newARNetworkAL()
        {
            alManager = new ARNetworkALManager();
            ARNETWORKAL_ERROR_ENUM netALError = alManager.initBLENetwork(context.getApplicationContext(), bleDevice, 1, bleNotificationIDs);
            return alManager.getManager();
        }

        public int deleteARNetworkAL()
        {
            ARDISCOVERY_ERROR_ENUM error = ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_OK;

            if (alManager != null)
            {
                alManager.unlock();
                alManager.closeBLENetwork(context);
                alManager.dispose();
                alManager = null;
            }
            else
            {
                error = ARDISCOVERY_ERROR_ENUM.ARDISCOVERY_ERROR_BAD_PARAMETER;
            }

            return error.getValue();
        }
    }
}
