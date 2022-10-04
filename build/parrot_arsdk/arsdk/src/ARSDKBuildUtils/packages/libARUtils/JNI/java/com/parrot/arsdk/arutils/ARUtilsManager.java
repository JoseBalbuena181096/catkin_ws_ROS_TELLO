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

package com.parrot.arsdk.arutils;

import android.bluetooth.BluetoothGatt;
import android.content.Context;
import android.content.pm.PackageManager;
import android.text.TextUtils;
import android.util.Log;

import com.parrot.arsdk.ardiscovery.ARDISCOVERY_NETWORK_TYPE_ENUM;
import com.parrot.arsdk.ardiscovery.ARDISCOVERY_PRODUCT_ENUM;
import com.parrot.arsdk.ardiscovery.ARDISCOVERY_PRODUCT_FAMILY_ENUM;
import com.parrot.arsdk.ardiscovery.ARDiscoveryDeviceNetService;
import com.parrot.arsdk.ardiscovery.ARDiscoveryDeviceService;
import com.parrot.arsdk.ardiscovery.ARDiscoveryService;
import com.parrot.arsdk.ardiscovery.UsbAccessoryMux;
import com.parrot.arsdk.arsal.ARSALBLEManager;
import com.parrot.mux.Mux;

import java.util.concurrent.Semaphore;

/**
 * ARUtils Manager module
 */
public class ARUtilsManager
{
    private static final String TAG = "ARUtilsManager";
    public final static String FTP_ANONYMOUS = "anonymous";

    private final static int FTP_GENERIC = 21;
    private final static int FTP_GENERIC_SKY = 121;
    private final static int FTP_UPDATE = 51;
    private final static int FTP_UPDATE_SKY = 151;
    private final static int FTP_FLIGHTPLAN = 61;
    private final static int FTP_FLIGHTPLAN_SKY = 161;

    /* Native Functions */
    private native static boolean nativeStaticInit();
    private native long nativeNew() throws ARUtilsException;
    private native int nativeDelete(long jManager);
    private native int nativeInitWifiFtp(long jManager, String jserver, int port, String jusername, String jpassword);
    private native int nativeInitWifiFtpOverMux(long jManager, String jserver, int port, long muxCtxCPtr, String jusername, String jpassword);
    private native int nativeCloseWifiFtp(long jManager);
    private native int nativeInitBLEFtp(long jManager, ARUtilsBLEFtp bleFtp, Semaphore cancelSem);
    private native void nativeCloseBLEFtp(long jManager);
    private native int nativeInitRFCommFtp(long jManager, ARUtilsRFCommFtp rfcommFtp, Semaphore cancelSem);
    private native void nativeCloseRFCommFtp(long jManager);

    /*Test Methods*/
    private native int nativeBLEFtpConnectionDisconnect(long jManager);
    private native int nativeBLEFtpConnectionReconnect(long jManager);
    private native int nativeBLEFtpConnectionCancel(long jManager);
    private native int nativeBLEFtpIsConnectionCanceled(long jManager);
    private native int nativeBLEFtpConnectionReset(long jManager);
    private native String nativeBLEFtpList(long jManager, String remotePath) throws ARUtilsException;
    private native double nativeBLEFtpSize(long jManager, String remotePath) throws ARUtilsException;
    private native byte[] nativeBLEFtpGetWithBuffer(long jManager, String remotePath, ARUtilsFtpProgressListener progressListener, Object progressArg);
    private native int nativeBLEFtpGet(long jManager, String remotePath, String destFile, ARUtilsFtpProgressListener progressListener, Object progressArg, boolean resume);
    private native int nativeBLEFtpPut(long jManager, String remotePath, String srcFile, ARUtilsFtpProgressListener progressListener, Object progressArg, boolean resume);
    private native int nativeBLEFtpDelete(long jManager, String remotePath);
    private native int nativeBLEFtpRename(long jManager, String oldNamePath, String newNamePath);
    
    private native int nativeRFCommFtpConnectionDisconnect(long jManager);
    private native int nativeRFCommFtpConnectionReconnect(long jManager);
    private native int nativeRFCommFtpConnectionCancel(long jManager);
    private native int nativeRFCommFtpIsConnectionCanceled(long jManager);
    private native int nativeRFCommFtpConnectionReset(long jManager);
    private native int nativeRFCommFtpPut(long jManager, String remotePath, String srcFile, ARUtilsFtpProgressListener progressListener, Object progressArg, boolean resume);

    private long m_managerPtr;
    private boolean m_initOk;

    private boolean mIsWifiFtpInited = false;
    private boolean mIsBLEFtpInited = false;
    private boolean mIsRFCommFtpInited = false;

    static
    {
        nativeStaticInit();
    }

    /*  Java Methods */

    /**
     * Constructor
     */
    public ARUtilsManager() throws ARUtilsException
    {
        m_initOk = false;
        m_managerPtr = nativeNew();

        if( m_managerPtr != 0 )
        {
            m_initOk = true;
        }
    }

    /**
     * Dispose
     */
    public void dispose()
    {
        if(m_initOk)
        {
            nativeDelete(m_managerPtr);
            m_managerPtr = 0;
            m_initOk = false;
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

    /**
     * Get the pointer C on the network manager
     * @return  Pointer C on the network manager
     */
    public long getManager ()
    {
        return m_managerPtr;
    }

    /**
     * Get is the Manager is correctly initialized and if it is usable
     * @return true is the Manager is usable
     */
    public boolean isCorrectlyInitialized ()
    {
        return m_initOk;
    }

    /**
     * Initialize ftp for the given device
     */
    public ARUTILS_ERROR_ENUM initFtp(Context ctx, ARDiscoveryDeviceService device, ARUTILS_DESTINATION_ENUM destination, ARUTILS_FTP_TYPE_ENUM type)
    {
        if (device == null || ctx == null) {
            return ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }
        ARDISCOVERY_PRODUCT_ENUM product = ARDiscoveryService.getProductFromProductID(device.getProductID());
        boolean sky = ARDiscoveryService.getProductFamily(product) == ARDISCOVERY_PRODUCT_FAMILY_ENUM.ARDISCOVERY_PRODUCT_FAMILY_SKYCONTROLLER;
        boolean old_sky = (product == ARDISCOVERY_PRODUCT_ENUM.ARDISCOVERY_PRODUCT_SKYCONTROLLER);
        boolean new_sky = (sky && !old_sky);
        boolean wifi = device.getNetworkType() == ARDISCOVERY_NETWORK_TYPE_ENUM.ARDISCOVERY_NETWORK_TYPE_NET;


        if (!new_sky && destination != ARUTILS_DESTINATION_ENUM.ARUTILS_DESTINATION_DRONE) {
            return ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }
        if (destination == ARUTILS_DESTINATION_ENUM.ARUTILS_DESTINATION_SKYCONTROLLER &&
                type == ARUTILS_FTP_TYPE_ENUM.ARUTILS_FTP_TYPE_FLIGHTPLAN) {
            return ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }

        int port;

        switch (type) {
            case ARUTILS_FTP_TYPE_GENERIC:
                port = (new_sky && wifi && destination == ARUTILS_DESTINATION_ENUM.ARUTILS_DESTINATION_DRONE) ? FTP_GENERIC_SKY : FTP_GENERIC;
                break;
            case ARUTILS_FTP_TYPE_UPDATE:
                port = (new_sky && wifi && destination == ARUTILS_DESTINATION_ENUM.ARUTILS_DESTINATION_DRONE) ? FTP_UPDATE_SKY : FTP_UPDATE;
                break;
            case ARUTILS_FTP_TYPE_FLIGHTPLAN:
                port = (new_sky && wifi) ? FTP_FLIGHTPLAN_SKY : FTP_FLIGHTPLAN;
                break;
            default:
                return ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }

        ARUTILS_ERROR_ENUM ret;

        switch(device.getNetworkType()) {
            case ARDISCOVERY_NETWORK_TYPE_NET:
                ARDiscoveryDeviceNetService ns = (ARDiscoveryDeviceNetService)device.getDevice();
                ret = initWifiFtp(ns.getIp(), port, ARUtilsManager.FTP_ANONYMOUS, "");
                break;
            case ARDISCOVERY_NETWORK_TYPE_BLE:
                if (destination == ARUTILS_DESTINATION_ENUM.ARUTILS_DESTINATION_DRONE &&
                        type == ARUTILS_FTP_TYPE_ENUM.ARUTILS_FTP_TYPE_UPDATE)
                    ret = initRFCommFtp(ctx, ARSALBLEManager.getInstance(ctx).getGatt(), port);
                else
                    ret = initBLEFtp(ctx, ARSALBLEManager.getInstance(ctx).getGatt(), port);
                break;
            case ARDISCOVERY_NETWORK_TYPE_USBMUX:
                Mux mux = UsbAccessoryMux.get(ctx.getApplicationContext()).getMux();
                if (mux != null) {
                    Mux.Ref muxref = mux.newMuxRef();
                    String dest = (destination == ARUTILS_DESTINATION_ENUM.ARUTILS_DESTINATION_DRONE) ? "drone" : "skycontroller";
                    ret = initWifiFtp(muxref, dest, port, ARUtilsManager.FTP_ANONYMOUS, "");
                    muxref.release();
                }
                else {
                    Log.w(TAG, "initFtp ARDISCOVERY_NETWORK_TYPE_USB failed, mux is null");
                    ret = ARUTILS_ERROR_ENUM.ARUTILS_ERROR;
                }
                break;
            default:
                ret = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
                break;
        }

        return ret;
    }

    public ARUTILS_ERROR_ENUM closeFtp(Context ctx, ARDiscoveryDeviceService device)
    {
        if (device == null) {
            return ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }

        ARUTILS_ERROR_ENUM ret;

        switch(device.getNetworkType()) {
            case ARDISCOVERY_NETWORK_TYPE_NET:
                ret = closeWifiFtp();
                break;
            case ARDISCOVERY_NETWORK_TYPE_BLE:
                if (mIsBLEFtpInited)
                    ret = closeBLEFtp(ctx);
                else if (mIsRFCommFtpInited)
                    ret = closeRFCommFtp(ctx);
                else
                    ret = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
                break;
            case ARDISCOVERY_NETWORK_TYPE_USBMUX:
                ret = closeWifiFtp();
                break;
            default:
                ret = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
                break;
        }

        return ret;
    }

    /**
     * Initialize Wifi network to send and receive data
     */
    public ARUTILS_ERROR_ENUM initWifiFtp(String addr, int port, String username, String password)
    {
        Log.i(TAG, "initWifiFtp on ip:port " + addr+":" +port);
        ARUTILS_ERROR_ENUM error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR;

        if(TextUtils.isEmpty(addr))
        {
            error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }
        else if(! mIsWifiFtpInited)
        {
            int intError = nativeInitWifiFtp(m_managerPtr, addr, port, username, password);
            error =  ARUTILS_ERROR_ENUM.getFromValue(intError);

            mIsWifiFtpInited = (error == ARUTILS_ERROR_ENUM.ARUTILS_OK);
        }
        else
        {
            Log.e(TAG, "wifi FTP is already inited");
        }

        return error;
    }

    /**
     * Initialize Wifi network to send and receive data
     */
    public ARUTILS_ERROR_ENUM initWifiFtp(Mux.Ref muxRef, String dest, int port, String username, String password)
    {
        Log.i(TAG, "initWifiFtp on mux, port " + port);
        ARUTILS_ERROR_ENUM error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR;

        if(! mIsWifiFtpInited) {
            muxRef.getCPtr();
            int intError = nativeInitWifiFtpOverMux(m_managerPtr, dest, port, muxRef.getCPtr(), username, password);
            error = ARUTILS_ERROR_ENUM.getFromValue(intError);
            mIsWifiFtpInited = (error == ARUTILS_ERROR_ENUM.ARUTILS_OK);
        }
        else
        {
            Log.e(TAG, "wifi FTP is already inited");
        }
        return error;
    }

    /**
     * Closes Wifi network
     */
    public ARUTILS_ERROR_ENUM closeWifiFtp()
    {
        ARUTILS_ERROR_ENUM error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR;
        if(mIsWifiFtpInited)
        {
            int intError = nativeCloseWifiFtp(m_managerPtr);
            error = ARUTILS_ERROR_ENUM.getFromValue(intError);
            mIsWifiFtpInited = false;
        }
        else
        {
            Log.e(TAG, "we haven't successfully initWifiFtp");
        }

        return error;
    }

    /**
     * Initialize BLE network to send and receive data
     */
    public ARUTILS_ERROR_ENUM initBLEFtp(Context context, BluetoothGatt deviceGatt, int port)
    {
        if(mIsBLEFtpInited)
        {
            Log.e(TAG, "BLE FTP is already inited");
            return ARUTILS_ERROR_ENUM.ARUTILS_ERROR;
        }

        ARUTILS_ERROR_ENUM error = ARUTILS_ERROR_ENUM.ARUTILS_OK;
        
        ARUtilsBLEFtp bleFtp = null;

        /* check parameters */
        if (context == null)
        {
            error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }

        if (deviceGatt == null)
        {
            error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }

        if ((port == 0) || ((port % 10) != 1))
        {
            error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }

        if (error == ARUTILS_ERROR_ENUM.ARUTILS_OK)
        {
            /* check if the BLE is available*/
            if (! context.getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE))
            {
                error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_NETWORK_TYPE;
            }
        }
        if (error == ARUTILS_ERROR_ENUM.ARUTILS_OK)
        {
            // No need to connect because we will always be connected before reaching this step (in FreeFlight!)
            //if (bleManager.connect(device) != ARSAL_ERROR_ENUM.ARSAL_OK) {
            //    error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BLE_FAILED;
            //}
        }
        if (error == ARUTILS_ERROR_ENUM.ARUTILS_OK)
        {
            bleFtp = ARUtilsBLEFtp.getInstance(context);
            boolean registered = bleFtp.registerDevice(deviceGatt, port);
            if (!registered)
            {
                error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BLE_FAILED;
            }
        }

        mIsBLEFtpInited = (error == ARUTILS_ERROR_ENUM.ARUTILS_OK);
        
        if (mIsBLEFtpInited)
        {
            Semaphore cancelSem = new Semaphore(0);
            nativeInitBLEFtp(m_managerPtr, bleFtp, cancelSem);
        }

        return error;
    }

    /**
     * cancel BLE network connection
     */
    /*public ARUTILS_ERROR_ENUM cancelBLEFtp()
    {
        ARUTILS_ERROR_ENUM error = ARUTILS_ERROR_ENUM.ARUTILS_OK;

        if(m_initOk == true)
        {
            error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR;
        }

        if (error == ARUTILS_ERROR_ENUM.ARUTILS_OK)
        {
            //int intError = nativeCancelBLEFtp(m_managerPtr);
            //error = ARUTILS_ERROR_ENUM.getFromValue(intError);
        }

        return error;
    }*/

    /**
     * Closes BLE network
     */
    public ARUTILS_ERROR_ENUM closeBLEFtp(Context context)
    {
        ARUTILS_ERROR_ENUM error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR;

        if(context == null)
        {
            error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }
        else if(mIsBLEFtpInited)
        {
            ARUtilsBLEFtp bleFtp = ARUtilsBLEFtp.getInstance(context);
            //we unregister Device only when the BLE FTP is correctly initialised
            if(bleFtp.unregisterDevice())
            {
                error = ARUTILS_ERROR_ENUM.ARUTILS_OK;
            }
            nativeCloseBLEFtp(m_managerPtr);
            mIsBLEFtpInited = false;
        }
        else
        {
            Log.e(TAG, "we haven't successfully initBLEFtp");
        }

        return error;
    }

    public ARUTILS_ERROR_ENUM BLEFtpConnectionDisconnect()
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeBLEFtpConnectionDisconnect(m_managerPtr));
    }

    public ARUTILS_ERROR_ENUM BLEFtpConnectionReconnect()
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeBLEFtpConnectionReconnect(m_managerPtr));
    }

    public ARUTILS_ERROR_ENUM BLEFtpConnectionCancel()
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeBLEFtpConnectionCancel(m_managerPtr));
    }

    public ARUTILS_ERROR_ENUM BLEFtpIsConnectionCanceled()
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeBLEFtpIsConnectionCanceled(m_managerPtr));
    }

    public ARUTILS_ERROR_ENUM BLEFtpConnectionReset()
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeBLEFtpConnectionReset(m_managerPtr));
    }

    public String BLEFtpListFile(String remotePath) throws ARUtilsException
    {
        return nativeBLEFtpList(m_managerPtr, remotePath);
    }
    
    public double BLEFtpSize(String remotePath) throws ARUtilsException
    {
        return nativeBLEFtpSize(m_managerPtr, remotePath);
    }

    public ARUTILS_ERROR_ENUM BLEFtpPut(String remotePath, String srcFile, ARUtilsFtpProgressListener progressListener, Object progressArg, boolean resume)
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeBLEFtpPut(m_managerPtr, remotePath, srcFile, progressListener, progressArg, resume));
    }

    public ARUTILS_ERROR_ENUM BLEFtpGet(String remotePath, String destFile, ARUtilsFtpProgressListener progressListener, Object progressArg, boolean resume)
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeBLEFtpGet(m_managerPtr, remotePath, destFile, progressListener, progressArg, resume));
    }

    public byte[] BLEFtpGetWithBuffer(String remotePath, ARUtilsFtpProgressListener progressListener, Object progressArg)
    {
        return nativeBLEFtpGetWithBuffer(m_managerPtr, remotePath, progressListener, progressArg);
    }

    public ARUTILS_ERROR_ENUM BLEFtpDelete(String remotePath)
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeBLEFtpDelete(m_managerPtr, remotePath));
    }

    public ARUTILS_ERROR_ENUM BLEFtpRename(String oldNamePath, String newNamePath)
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeBLEFtpRename(m_managerPtr, oldNamePath, newNamePath));
    }
    
    
    /**
     * Initialize RFComm network to send and receive data
     */
    public ARUTILS_ERROR_ENUM initRFCommFtp(Context context, BluetoothGatt deviceGatt, int port)
    {
        if(mIsRFCommFtpInited)
        {
            Log.e(TAG, "RFComm FTP is already inited");
            return ARUTILS_ERROR_ENUM.ARUTILS_ERROR;
        }

        ARUTILS_ERROR_ENUM error = ARUTILS_ERROR_ENUM.ARUTILS_OK;
        ARUtilsRFCommFtp rfcommFtp = null;
        /* check parameters */
        if (context == null)
        {
            error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }
        
        if (deviceGatt == null)
        {
            error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }
        
        if ((port == 0) || ((port % 10) != 1))
        {
            error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }
        
        if (error == ARUTILS_ERROR_ENUM.ARUTILS_OK)
        {
            /* check if the RFComm is available*/
            if (! context.getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH))
            {
                error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_NETWORK_TYPE;
            }
        }
        if (error == ARUTILS_ERROR_ENUM.ARUTILS_OK)
        {
            // No need to connect because we will always be connected before reaching this step (in FreeFlight!)
            //if (bleManager.connect(device) != ARSAL_ERROR_ENUM.ARSAL_OK) {
            //    error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BLE_FAILED;
            //}
        }
        if (error == ARUTILS_ERROR_ENUM.ARUTILS_OK)
        {
            rfcommFtp = ARUtilsRFCommFtp.getInstance(context);
            boolean registered = rfcommFtp.registerDevice(deviceGatt, port);
            if (!registered)
            {
                error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_RFCOMM_FAILED;
            }
        }

        mIsRFCommFtpInited = (error == ARUTILS_ERROR_ENUM.ARUTILS_OK);
        
        if (mIsRFCommFtpInited)
        {
            Semaphore cancelSem = new Semaphore(0);
            nativeInitRFCommFtp(m_managerPtr, rfcommFtp, cancelSem);
        }
        
        return error;
    }
    
    /**
     * Closes BLE network
     */
    public ARUTILS_ERROR_ENUM closeRFCommFtp(Context context)
    {
        ARUTILS_ERROR_ENUM error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR;

        if(context == null)
        {
            error = ARUTILS_ERROR_ENUM.ARUTILS_ERROR_BAD_PARAMETER;
        }
        else if(mIsRFCommFtpInited)
        {
            ARUtilsRFCommFtp rfcommFtp = ARUtilsRFCommFtp.getInstance(context);
            if (rfcommFtp.unregisterDevice())
            {
                error = ARUTILS_ERROR_ENUM.ARUTILS_OK;
            }
            nativeCloseRFCommFtp(m_managerPtr);
            mIsRFCommFtpInited = false;
        }
        else
        {
            Log.e(TAG, "we haven't successfully initRFCommFtp");
        }

        return error;
    }
    
    public ARUTILS_ERROR_ENUM RFCommFtpConnectionDisconnect()
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeRFCommFtpConnectionDisconnect(m_managerPtr));
    }
    
    public ARUTILS_ERROR_ENUM RFCommFtpConnectionReconnect()
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeRFCommFtpConnectionReconnect(m_managerPtr));
    }
    
    public ARUTILS_ERROR_ENUM RFCommFtpConnectionCancel()
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeRFCommFtpConnectionCancel(m_managerPtr));
    }
    
    public ARUTILS_ERROR_ENUM RFCommFtpIsConnectionCanceled()
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeRFCommFtpIsConnectionCanceled(m_managerPtr));
    }
    
    public ARUTILS_ERROR_ENUM RFCommFtpConnectionReset()
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeRFCommFtpConnectionReset(m_managerPtr));
    }
    
    public ARUTILS_ERROR_ENUM RFCommFtpPut(String remotePath, String srcFile, ARUtilsFtpProgressListener progressListener, Object progressArg, boolean resume)
    {
        return ARUTILS_ERROR_ENUM.getFromValue(nativeRFCommFtpPut(m_managerPtr, remotePath, srcFile, progressListener, progressArg, resume));
    }

}

