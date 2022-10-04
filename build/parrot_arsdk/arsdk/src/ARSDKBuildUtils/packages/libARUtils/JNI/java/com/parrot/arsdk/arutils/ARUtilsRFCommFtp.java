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

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.InputMismatchException;
import java.util.Iterator;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Scanner;
import java.util.concurrent.Semaphore;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.AsyncTask;
import android.os.Environment;
import android.util.Log;
import android.content.Context;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import android.os.Build;
import java.util.UUID;
import java.io.InputStream;
import java.io.OutputStream;

import com.parrot.arsdk.arsal.ARSALBLEManager;
import com.parrot.arsdk.arsal.ARSALBLEManager.ARSALManagerNotificationData;
import com.parrot.arsdk.arsal.ARSALPrint;
import com.parrot.arsdk.arsal.ARSAL_ERROR_ENUM;
import com.parrot.arsdk.arsal.ARUUID;
import com.parrot.arsdk.arnetworkal.ARNetworkALBLENetwork;
import java.nio.ByteBuffer;

public class ARUtilsRFCommFtp
{
    private static final String LOG_TAG = "ARUtilsRFCommFTP.java";

    private static final String RFCOMM_UPDATE_KEY = "UPD";

    public final static String RFCOMM_GETTING_KEY = "kARUTILS_BLERFComm_Getting";

    private ARSALBLEManager bleManager = null;
    private BluetoothGatt gattDevice = null;
    private int port;
    private int connectionCount = 0;
    private Lock connectionLock = new ReentrantLock();

    private ArrayList<BluetoothGattCharacteristic> arrayGetting = null;

    private native void nativeProgressCallback(long nativeCallbackObject, float percent);
    private native static void nativeJNIInit();

    private BluetoothGattCharacteristic rfCommWriteCharac;
    private BluetoothGattCharacteristic rfCommReadCharac;
    
    // Type of message
    protected static final int ST_NOT_CONNECTED = 0;
    protected static final int ST_CONNECTING = 1;
    protected static final int ST_CONNECTED = 2;

    public static final byte TYPE_MES_OPEN_SESSION = 0x00;
    public static final byte TYPE_MES_CLOSE_SESSION = 0x01;
    private static final byte TYPE_MES_ACKNOWLEDGT = 0x02;
    public static final byte TYPE_MES_DATA = (byte) 0x80; // get or set request

    public static final String SOFTWARE_DOWNLOAD_SIZE_SET = "/api/software/download_size/set";
    private static final UUID MY_UUID = UUID.fromString("8b6814d3-6ce7-4498-9700-9312c1711f63");    // TODO change this name
    private static final Integer RFCOMM_CHANNEL = 21;
    private BluetoothSocket mSocket;
    private InputStream mInStream;
    private OutputStream mOutStream;
    private boolean mIsOpeningSession = false;
    private int mState = ST_NOT_CONNECTED;
    private BluetoothDevice mDevice;

    static
    {
        nativeJNIInit();
    }

    private ARUtilsRFCommFtp()
    {
    }

    private static class ARUtilsRFCommFtpHolder
    {
        private final static ARUtilsRFCommFtp instance = new ARUtilsRFCommFtp();
    }

    public static ARUtilsRFCommFtp getInstance(Context context)
    {
        ARUtilsRFCommFtp instance = ARUtilsRFCommFtpHolder.instance;
        if (context == null)
        {
            throw new IllegalArgumentException("Context must not be null");
        }
        instance.setBLEManager(context);
        return instance;
    }

    private synchronized void setBLEManager(Context context)
    {
        if (this.bleManager == null)
        {
            if (context == null)
            {
                throw new IllegalArgumentException("Context must not be null");
            }
            this.bleManager = ARSALBLEManager.getInstance(context);
        }
    }


    public boolean registerDevice(BluetoothGatt gattDevice, int port)
    {
        ARSALPrint.d(LOG_TAG, "registerDevice " + gattDevice.toString() + " port : " + port);
        boolean ret = true;

        /*if (connectionCount == 0)
        {*/
        if ((this.gattDevice != gattDevice) || (this.port != port))
        {
            this.gattDevice = gattDevice;
            this.port = port;
            connectionCount++;

            searchForInterestingCharacs();

            ret = registerCharacteristics();
        }
        else if (this.gattDevice == null)
        {
            ARSALPrint.e(LOG_TAG, "registerDevice : Bad parameters");
            ret = false;
        }
        else
        {
            ARSALPrint.e(LOG_TAG, "already on good device");
        }
        /*}
        else if ((this.gattDevice == gattDevice) && (this.port == port))
        {
            connectionCount++;
        }
        else
        {
            ARSALPrint.e(LOG_TAG, "Bad parameters");
            ret = false;
        }*/

        return ret;
    }

    public boolean unregisterDevice()
    {
        boolean ret = true;

        if (connectionCount > 0)
        {
            if (connectionCount == 1)
            {
                this.gattDevice = null;
                this.port = 0;

                unregisterCharacteristics();
            }

            connectionCount--;
        }
        else
        {
            ARSALPrint.e(LOG_TAG, "Bad parameters");
            ret = false;
        }

        return ret;
    }

    @SuppressLint("NewApi")
    public void searchForInterestingCharacs()
    {
        List<BluetoothGattService> services = gattDevice.getServices();
        ARSAL_ERROR_ENUM error = ARSAL_ERROR_ENUM.ARSAL_OK;
        boolean ret = true;

        ARSALPrint.d(LOG_TAG, "registerCharacteristics");

        // store in variables the characteristics we will need
        Iterator<BluetoothGattService> servicesIterator = services.iterator();
        while (servicesIterator.hasNext())
        {
            BluetoothGattService service = servicesIterator.next();
            String serviceUuid = ARUUID.getShortUuid(service.getUuid());
            String name = ARUUID.getShortUuid(service.getUuid());
            ARSALPrint.e(LOG_TAG, "service " + name);

            if (serviceUuid.startsWith(ARNetworkALBLENetwork.ARNETWORKAL_BLENETWORK_PARROT_SERVICE_PREFIX_UUID_RFCOMM))
            {
                List<BluetoothGattCharacteristic> characteristics = service.getCharacteristics();
                Iterator<BluetoothGattCharacteristic> characteristicsIterator = characteristics.iterator();

                while (characteristicsIterator.hasNext())
                {
                    BluetoothGattCharacteristic characteristic = characteristicsIterator.next();
                    String characteristicUuid = ARUUID.getShortUuid(characteristic.getUuid());
                    ARSALPrint.e(LOG_TAG, "characteristic " + characteristicUuid);

                    if (characteristicUuid.startsWith(ARNetworkALBLENetwork.ARNETWORKAL_BLENETWORK_PARROT_CHARACTERISTIC_PREFIX_UUID_RFCOMM_READ))
                    {
                        this.rfCommReadCharac = characteristic;
                    }
                    else if (characteristicUuid.startsWith(ARNetworkALBLENetwork.ARNETWORKAL_BLENETWORK_PARROT_CHARACTERISTIC_PREFIX_UUID_RFCOMM_WRITE))
                    {
                        this.rfCommWriteCharac = characteristic;
                        this.rfCommReadCharac = characteristic;
                    }
                }
            }
        }
    }

    public boolean registerCharacteristics()
    {
        boolean ret = false;

        ARSALPrint.d(LOG_TAG, "registerCharacteristics");

        arrayGetting = null;

        if (this.rfCommReadCharac != null)
        {
            this.arrayGetting = new ArrayList<BluetoothGattCharacteristic>();
            this.arrayGetting.add(rfCommReadCharac);
            bleManager.registerNotificationCharacteristics(this.arrayGetting, RFCOMM_GETTING_KEY);
            ret = true;
        }

        return ret;
    }

    public boolean unregisterCharacteristics()
    {
        boolean ret = true;

        ARSALPrint.d(LOG_TAG, "unregisterCharacteristics");

        ret = bleManager.unregisterNotificationCharacteristics(RFCOMM_GETTING_KEY);

        return ret;
    }
    
    /* ************************************************************* */
    /* Public API
    /* ************************************************************* */
    public boolean putFileAL(String remotePath, String localFile, long nativeCallbackObject, boolean resume, Semaphore cancelSem)
    {
        boolean ret = false;
        ARSALPrint.e(LOG_TAG, "putFileAL");
        connectionLock.lock();
        ret =  putFile(remotePath, localFile, nativeCallbackObject, resume, cancelSem);
        connectionLock.unlock();
        
        return ret;
    }
    
    public boolean cancelFileAL(Semaphore cancelSem)
    {
        return cancelFile(cancelSem);
    }
    
    public boolean isConnectionCanceledAL(Semaphore cancelSem)
    {
        return isConnectionCanceled(cancelSem);
    }
    
    public boolean resetConnectionAL(Semaphore cancelSem)
    {
        return resetConnection(cancelSem);
    }
    /* ************************************************************* */
    /* End of Public API
    /* ************************************************************* */
    
    private boolean cancelFile(Semaphore cancelSem)
	{
		boolean ret = true;

		cancelSem.release();

		return ret;
	}
    
    private boolean isConnectionCanceled(Semaphore cancelSem)
    {
        boolean ret = false;
        
        if (cancelSem != null)
        {
            ret = cancelSem.tryAcquire();
            if (ret  == true)
            {
                cancelSem.release();
            }
        }
        
        return ret;
    }
    
    private boolean resetConnection(Semaphore cancelSem)
    {
        boolean ret = true;
        
        if (cancelSem != null)
        {
            while (cancelSem.tryAcquire())
            {
                /* Do nothing*/
            }
        }
        
        return ret;
    }

    private boolean putFile(String remoteFile, String localFile, long nativeCallbackObject, boolean resume, Semaphore cancelSem)
    {
        boolean processIsOK = true;
        ARSALPrint.d(LOG_TAG, "putFile Begin");
        File fileToUpload = new File(localFile);
        if ((fileToUpload == null) || !(fileToUpload.exists()))
        {
            processIsOK = false;
        }
        
        // connect to the rfcomm device through BLE
        if (processIsOK)
        {
            ARSALPrint.d(LOG_TAG, "Will send file with size = " + fileToUpload.length());
            connectToRFCommDevice(fileToUpload.length(), cancelSem);
            if (mState != ST_CONNECTING)
            {
                processIsOK = false;
            }
        }
        
        // tell the rfcomm chip that we open the session
        if (processIsOK)
        {
            openSession(); 
            if (mState != ST_CONNECTED)
            {
                processIsOK = false;
            }  
        }
        
        // send plf file
        if (processIsOK) 
        {
            processIsOK = sendFile(fileToUpload, nativeCallbackObject, cancelSem);
        }
        
        // close the session
        if (processIsOK && !isConnectionCanceled(cancelSem))
        {
            // wait a second to delay the close session
            try
            {
                Thread.sleep(5000);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        
        if (processIsOK)
        {
            closeSession();
            if (mState != ST_NOT_CONNECTED)
            {
                processIsOK = false;
            }
        }
        
        // close the rfcomm connection
        closeConnection();
        //if (processIsOK)
        //{
        //    closeConnection();
        //}
        
        return processIsOK;
    }

    private void connectToRFCommDevice(long fileSize, Semaphore cancelSem)
    {
        // first get the mac address of the rfcomm chip
        String rfCommMacAddress = askRFCommMacAdress(fileSize, cancelSem);
        ARSALPrint.d(LOG_TAG, "rfCommMacAddress = " + rfCommMacAddress);
        if (rfCommMacAddress != null)
        {
            // connect to the rfcomm chip with its mac address
            connectToBluetoothDevice(rfCommMacAddress);
        }
    }

    private String askRFCommMacAdress(long fileSize, Semaphore cancelSem)
    {
        String macAddress = null;
        if (rfCommWriteCharac != null)
        {
            // write UPDxxxx (xxx = nb bytes that will be written) on the rfCommWriteCharac
            String key = RFCOMM_UPDATE_KEY + fileSize;
            byte[] keyAsByteArr = key.getBytes();
            
            ARSALPrint.d(LOG_TAG, "Write in charac " + key);
            bleManager.writeData(keyAsByteArr, rfCommWriteCharac);
            if (!isConnectionCanceled(cancelSem))
            {
                // read mac address on the rfCommReadCharac
                macAddress = readRFCommMacAdress(cancelSem);
            }
            else
            {
                ARSALPrint.e(LOG_TAG, "Canceled received after having written in the BLE characteristic to get rfcomm mac address");
            }
        }

        return macAddress;
    }
    
    private String readRFCommMacAdress(Semaphore cancelSem)
    {
        ArrayList<ARSALManagerNotificationData> receivedNotifications = new ArrayList<ARSALManagerNotificationData>();

        boolean readDataSucceed = false;
        String rfcommMacAddress = null;


        if (receivedNotifications.size() == 0)
        {
            readDataSucceed = bleManager.readDataNotificationData(receivedNotifications, 1, RFCOMM_GETTING_KEY);
            ARSALPrint.d(LOG_TAG, "Data has been read");
        }
        
        if (isConnectionCanceled(cancelSem))
        {
            ARSALPrint.d(LOG_TAG, "Canceled received after having read the rfcomm mac address");
            readDataSucceed = false;
        }

        if ((readDataSucceed == true) && (receivedNotifications.size() > 0))
        {
            ARSALManagerNotificationData notificationData = null;
            notificationData = receivedNotifications.get(0);
            byte[] block = notificationData.value;
            if (block != null)
            {
                StringBuilder strBld = new StringBuilder();
                // don't take first byte, because it's an header
                for (int i = block.length - 1; i > 0 ; i--)
                {
                    strBld.append(String.format("%02X", block[i]));
                    if (i > 1)
                    {
                        strBld.append(":");
                    }
                }
                rfcommMacAddress = strBld.toString();
            }
        }

        return rfcommMacAddress;
    }

    private void connectToBluetoothDevice(String macAddress)
    {
        mSocket = null;
        ARSALPrint.d(LOG_TAG, "Try to connect to bluetooth device");
        BluetoothAdapter bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (bluetoothAdapter.isEnabled())
        {
            BluetoothDevice device = bluetoothAdapter.getRemoteDevice(macAddress);
            try
            {
                // this should not happen because BLE support begins in 18
                if (Build.VERSION.SDK_INT < 17)
                {
                    Method createInsecureRfcommSocketToServiceRecord = device.getClass().getMethod("createInsecureRfcommSocketToServiceRecord",
                            UUID.class);
                    mSocket = (BluetoothSocket) createInsecureRfcommSocketToServiceRecord.invoke(device, MY_UUID);
                }
                else
                {
                    Method createInsecureRfcommSocket = device.getClass().getMethod("createInsecureRfcommSocket", new Class[] { int.class });
                    mSocket = (BluetoothSocket) createInsecureRfcommSocket.invoke(device, RFCOMM_CHANNEL);
                }
            }
            catch (NoSuchMethodException e)
            {
                e.printStackTrace();
            }
            catch (InvocationTargetException e)
            {
                e.printStackTrace();
            }
            catch (IllegalAccessException e)
            {
                e.printStackTrace();
            }

            try
            {
                mSocket.connect();
            }
            catch (IOException e)
            {
                mSocket = null;
                e.printStackTrace();
            }

            if (mSocket != null)
            {
                mDevice = device;
                // Start the thread to manage the connection and perform transmissions
                try {
                    mInStream = mSocket.getInputStream();
                    mOutStream = mSocket.getOutputStream();
                    mState = ST_CONNECTING;
                } catch (IOException e) {
                    closeConnection();
                }
            }
        }
        else
        {
            ARSALPrint.e(LOG_TAG, "Bluetooth adapter is not enabled");
        }

    }

    /**
     * Open session
     */
    private void openSession() {
        ARSALPrint.d(LOG_TAG, "open RFComm session");
        mIsOpeningSession = true;
        write(getHeaderFirst(0, TYPE_MES_OPEN_SESSION));
        byte[] readArray = new byte[4096];
        try {
            // wait for the answer
            int readLength = mInStream.read(readArray);
        }
        catch (IOException e) {
            closeConnection();
            return;
        }
        
        mIsOpeningSession = false;
        mState = ST_CONNECTED;
    }
    
    /**
     * Close session
     */
    private void closeSession() {
        ARSALPrint.d(LOG_TAG, "close RFComm session");
        mIsOpeningSession = true;
        write(getHeaderFirst(0, TYPE_MES_CLOSE_SESSION));
        byte[] readArray = new byte[4096];
        try {
            // wait for the answer
            int readLength = mInStream.read(readArray);
        }
        catch (IOException e) {
            closeConnection();
            return;
        }
        mState = ST_NOT_CONNECTED;
    }

    private void unpairDevice(BluetoothDevice device) {
        try 
        {
            Method m = device.getClass()
                .getMethod("removeBond", (Class[]) null);
            m.invoke(device, (Object[]) null);
        } 
        catch (Exception e) 
        {
            ARSALPrint.e(LOG_TAG, e.getMessage());
        }
    }
    
    private boolean sendFile(File file, long nativeCallbackObject, Semaphore cancelSem) 
    {
        int nbSstoredBytes = 0;     // for now, send the entire file (=> no resume)
        boolean ret = true;
        FileInputStream f = null;
        try {
            f = new FileInputStream(file);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return false;
        }

        byte[] buffer = new byte[978];
        int len = 0;
        long total = 0;
        int id = 0;
        float percent = 0.f;
        float lastPercent = 0.f;
        
        try {
            // Skip bytes that already saved on divice
            if(nbSstoredBytes > 0) {
                while (nbSstoredBytes > 0) {
                    int nbSkipped = (int) f.skip(nbSstoredBytes);
                    nbSstoredBytes -= nbSkipped ;
                }
            }

            long fileSize = file.length();

            // Send to device
            while ( ret && (len = f.read(buffer)) > 0 ) {

                total += len;
                byte[] request = new byte[len];
                System.arraycopy(buffer, 0, request, 0, len);
                //long time = System.currentTimeMillis();
                if(!sendFirmwareOnDevice(request, id)){
                    ARSALPrint.e(LOG_TAG, "upload firmware, task was canceled");
                    f.close();
                    return false;
                }
                /*if (System.currentTimeMillis() - time > 500) {
                    try {
                        Log.e(LOG_TAG, "fixIssuesOnHtcDevices");
                        //fixIssuesOnHtcDevices();
                    } catch (Exception e) {
                    }
                }*/
                
                percent = ((float)total / (float)fileSize) * 100.f;
                if (nativeCallbackObject != 0)
                {
                    
                    // no need of this block for now, because we have no resume for now
                    /*if ((resume == true) && (totalPacket < resumeIndex))
                    {
                        if ((percent - lastPercent) > 1.f)
                        {
                            lastPercent = percent;
                            nativeProgressCallback(nativeCallbackObject, percent);
                        }
                    }
                    else
                    {*/
                        nativeProgressCallback(nativeCallbackObject, percent);
                    //}
                }
                
                if (isConnectionCanceled(cancelSem))
                {
                    ARSALPrint.d(LOG_TAG, "Canceled received during file upload");
                    
                    ret = false;
                }
                
                id++;
            }
        } catch (Exception e) {
            e.printStackTrace();
            try {
                f.close();
            } catch (IOException e1) {
                e1.printStackTrace();
            }
            return false;
        }

        try {
            f.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        
        ARSALPrint.e(LOG_TAG, "Sending done. Sent " + total + " bytes");

        return ret;
    }
    
    public boolean sendFirmwareOnDevice(byte[] data, int id) 
    {
        boolean success = (mState == ST_CONNECTED);

        if(success)
        {
            byte[] request = getUploadPacket(data, id);
            //long time = System.currentTimeMillis();
            try 
            {
                success = write(request);
            } 
            catch (Exception e) 
            {
                success = false;
            }
            /*if (System.currentTimeMillis() < time + 10000) {
                try {
                    fixIssuesOnHtcDevices();
                } catch (Exception e) {
                }
            }*/
        }
        return success;
    }
    

    private synchronized boolean write(byte[] buffer) 
    {
        boolean success = false;
        try 
        {
            mOutStream.write(buffer);
            Thread.sleep(40);
            success = true;
        } 
        catch (IOException e) 
        {
            ARSALPrint.e(LOG_TAG, "Exception during write" + e.getMessage());
        } 
        catch (InterruptedException e)
        {
            ARSALPrint.e(LOG_TAG, "Exception during sleep" + e.getMessage());
        }
        return success;
    }

    /**
     * Return header of message
     * @param length - size of message
     */
    private static byte[] getHeaderFirst(int length, byte type) {

// ================================= format ======================================
//    	3-byte header : ZZ + T
//    	ZZ - total number of bytes of the packet (including this header)
//		T - type massage
// ===============================================================================

        byte[] zz = sizeIntToByte(length + 3); // 3 bytes of header
        byte[] t = new byte[1];
        t[0] = type;

        byte[] header = new byte[zz.length + t.length];
        System.arraycopy(zz, 0, header, 0, zz.length);
        System.arraycopy(t, 0, header, zz.length, t.length);

        return header;
    }

    /**
     * Return ZZ (2 bytes) - total number of bytes of the messages,
     * (Most Significant Byte first, then Least Significant Byte)
     */
    private static byte[] sizeIntToByte(int length) {

        byte[] zz = new byte[2];
        zz =  new byte[]{(byte)(length >>> 8), (byte)(length)};
        //zz =  new byte[]{(byte)(length), (byte)(length >>> 8)};

        return zz;
    }
    
    /**
     * Return ZZ (2 bytes) - total number of bytes of the messages,
     * (Most Significant Byte first, then Least Significant Byte)
     */
    private static byte[] sizeIntToByte2(int length) {

        byte[] zz = new byte[2];
        zz =  new byte[]{(byte)(length), (byte)(length >>> 8)};

        return zz;
    }

    public synchronized void closeConnection() {
        ARSALPrint.e(LOG_TAG, "closeConnection");
        try 
        {
            if (mInStream != null)
            {
                mInStream.close();
                mInStream = null;
            }
        } 
        catch (IOException e) 
        {
            ARSALPrint.e(LOG_TAG, "Closing of mInStream failed", e);
        }
        try 
        {
            if (mOutStream != null)
            {
                mOutStream.close();
                mOutStream = null;
            }
        } 
        catch (IOException e) 
        {
            ARSALPrint.e(LOG_TAG, "Closing of mOutStream failed", e);
        }
        try 
        {
            if (mSocket != null)
            {
                mSocket.close();
                mSocket = null;
            }
        } 
        catch (IOException e) 
        {
            ARSALPrint.e(LOG_TAG, "Closing of mSocket failed", e);
        }
        if (mDevice != null)
        {
            unpairDevice(mDevice);
            mDevice = null;
        }
        mState = ST_NOT_CONNECTED;
    }
    
    /**
     * Return packet in a specific format for uploading to Zik. Use for updating firmware.
     */
    private static byte[] getUploadPacket(byte[] data, int id) {

// ================================= format ======================================
//    	3-byte header
//    	4-byte header for uploading: XYZZ (see download2.odt)
//    	1-byte packet type (DATA = 0)
//    	2-byte packet identifier (MSB first),
//    	core on several bytes
//    	2-byte trailer (sign)
// ==============================================================================
//    		X current packet number, on 1 byte, value 1
//    		Y total number of packets, on 1 byte, value 1
//    		ZZ total number of bytes of the packet (including this header)
//    	optional trailer (sign) : inclusive OR (1 byte) and exclusive OR (1 byte).
//    	These 2 bytes are computed from start of packet up to last header byte
// ==============================================================================

        int sizePack2 = data.length + 9;

        byte[] header = getHeaderFirst(sizePack2, TYPE_MES_DATA); // ignoring on Zik side, but need
        byte[] xy = {0x01, 0x01};
        
        byte[] zzDesordered = ByteBuffer.allocate(2).putShort((short)(data.length + 9)).array(); 
        byte[] zz = {zzDesordered[1], zzDesordered[0]};
        
        //byte[] zz = sizeIntToByte(data.length + 9); // without length of the first header, 2 byte - trailer (sign) at the end
        byte[] pktType = {0x00};
        byte[] pktIdDesordered = ByteBuffer.allocate(2).putShort((short)id).array();//sizeIntToByte2(id);  
        byte[] pktId = {pktIdDesordered[1], pktIdDesordered[0]};      

        byte[] request = new byte[data.length + 12]; // a;; header and trailer

        System.arraycopy(header, 0, request, 0, header.length);
        System.arraycopy(xy, 0, request, header.length, xy.length);
        System.arraycopy(zz, 0, request, header.length + xy.length, zz.length);
        System.arraycopy(pktType, 0, request, header.length + xy.length + zz.length, pktType.length);
        System.arraycopy(pktId, 0, request, header.length + xy.length + zz.length + pktType.length, pktId.length);
        System.arraycopy(data, 0, request, header.length + xy.length + zz.length + pktType.length + pktId.length, data.length);

        // signing
        byte a = 0x00;
        byte b = 0x00;

        for ( int i=header.length; i<request.length; i++ ) { // ignoring the first header (3 bytes)
        //for ( int i=0; i<request.length - 2; i++ ) { // ignoring the first header (3 bytes)
            a |= request[i];
            b ^= request[i];
        }
        byte[] sign = {(byte) a, (byte) b};

        System.arraycopy(sign, 0, request, header.length + xy.length + zz.length + pktType.length + pktId.length + data.length, sign.length);

        return request;
    }

}

