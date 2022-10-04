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

import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.content.Context;

import com.parrot.arsdk.arsal.ARSALBLEManager;
import com.parrot.arsdk.arsal.ARSALBLEManager.ARSALManagerNotificationData;
import com.parrot.arsdk.arsal.ARSALPrint;
import com.parrot.arsdk.arsal.ARSAL_ERROR_ENUM;
import com.parrot.arsdk.arsal.ARUUID;

public class ARUtilsBLEFtp
{
	private static final String APP_TAG = "BLEFtp ";

	public final static String BLE_GETTING_KEY = "kARUTILS_BLEFtp_Getting";

	public final static String BLE_PACKET_WRITTEN =        "FILE WRITTEN";
	public final static String BLE_PACKET_NOT_WRITTEN =    "FILE NOT WRITTEN";
	public final static String BLE_PACKET_EOF =            "End of Transfer";
	public final static String BLE_PACKET_RENAME_SUCCESS = "Rename successful";
	public final static String BLE_PACKET_RENAME_FROM_SUCCESS = "Rename successful";
	public final static String BLE_PACKET_DELETE_SUCCESS =  "Delete successful";
	public final static int BLE_PACKET_MAX_SIZE = 132;
	public final static int BLE_PACKET_BLOCK_PUTTING_COUNT = 500;
	public final static int BLE_PACKET_BLOCK_GETTING_COUNT = 100;

	public final static long BLE_PACKET_WRITE_SLEEP = 35; /* 20ms or 30ms*/
	public final static int BLE_MTU_SIZE = 20;

	public final static byte BLE_BLOCK_HEADER_START = 0x02;    //Paquet de Start :      ID = 10 (en binaire) + data
	public final static byte BLE_BLOCK_HEADER_CONTINUE = 0x00; //Paquet de "data" :     ID = 00 + data
	public final static byte BLE_BLOCK_HEADER_STOP  = 0x01;    //Paquet de Stop :       ID = 01 + data
	public final static byte BLE_BLOCK_HEADER_SINGLE = 0x03;   //Paquet unique :        ID = 11 + data

	private ARSALBLEManager bleManager = null;
	private BluetoothGatt gattDevice = null;
	private int port;
	private int connectionCount = 0;
	private Lock connectionLock = new ReentrantLock();
	private volatile boolean isListing;

	private BluetoothGattCharacteristic transferring = null;
	private BluetoothGattCharacteristic getting = null;
	private BluetoothGattCharacteristic handling = null;
	private ArrayList<BluetoothGattCharacteristic> arrayGetting = null;

	private native void nativeProgressCallback(long nativeCallbackObject, float percent);

	private native static void nativeJNIInit();

	static
    {
        nativeJNIInit();
    }

	private ARUtilsBLEFtp()
	{
	}

	private static class ARUtilsBLEFtpHolder
    {
        private final static ARUtilsBLEFtp instance = new ARUtilsBLEFtp();
    }

    public static ARUtilsBLEFtp getInstance(Context context)
    {
        ARUtilsBLEFtp instance = ARUtilsBLEFtpHolder.instance;
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
	    boolean ret = true;

        if (connectionCount == 0)
        {
            this.gattDevice = gattDevice;
            this.port = port;
            connectionCount++;

            ret = registerCharacteristics();
        }
        else if ((this.gattDevice == gattDevice) && (this.port == port))
        {
            connectionCount++;
        }
        else
        {
            ARSALPrint.e(APP_TAG, "registerDevice Bad parameters : " + connectionCount + "\nthis.gattDevice = " + this.gattDevice + "\ngattDevice = " + gattDevice + "\nthis.port = " + this.port + "\nport = " + port);
            ret = false;
        }

	    return ret;
	}

	public boolean unregisterDevice()
	{
	    boolean ret = true;

	    if (connectionCount > 0)
	    {
            ARSALPrint.e("DBG", "unregisterDevice : connection count is " + connectionCount);
	        if (connectionCount == 1)
	        {
	            this.gattDevice = null;
	            this.port = 0;

	            this.transferring = null;
	            this.getting = null;
	            this.handling = null;

	            unregisterCharacteristics();
	        }

	        connectionCount--;
	    }
	    else
	    {
	        ARSALPrint.e("DBG", APP_TAG + "Bad parameters");
	        ret = false;
	    }

        return ret;
	}

	public boolean registerCharacteristics()
	{
		List<BluetoothGattService> services = gattDevice.getServices();
		ARSAL_ERROR_ENUM error = ARSAL_ERROR_ENUM.ARSAL_OK;
		boolean ret = true;

		ARSALPrint.d("DBG", APP_TAG + "registerCharacteristics");

		Iterator<BluetoothGattService> servicesIterator = services.iterator();
		while (servicesIterator.hasNext())
		{
			BluetoothGattService service = servicesIterator.next();
			String serviceUuid = ARUUID.getShortUuid(service.getUuid());
			String name = ARUUID.getShortUuid(service.getUuid());
			ARSALPrint.d("DBG", APP_TAG + "service " + name);

			if (serviceUuid.startsWith(String.format("fd%02d", this.port)))
			{
				List<BluetoothGattCharacteristic> characteristics = service.getCharacteristics();
				Iterator<BluetoothGattCharacteristic> characteristicsIterator = characteristics.iterator();

				while (characteristicsIterator.hasNext())
				{
					BluetoothGattCharacteristic characteristic = characteristicsIterator.next();
					String characteristicUuid = ARUUID.getShortUuid(characteristic.getUuid());
					ARSALPrint.d("DBG", APP_TAG + "characteristic " + characteristicUuid);

					if (characteristicUuid.startsWith(String.format("fd%02d", this.port + 1)))
					{
						this.transferring = characteristic;
					}
					else if (characteristicUuid.startsWith(String.format("fd%02d", this.port + 2)))
					{
						this.arrayGetting = new ArrayList<BluetoothGattCharacteristic>();
						this.arrayGetting.add(characteristic);
						this.getting = characteristic;

						ARSALPrint.d("DBG", APP_TAG + "set " + error.toString());

					}
					else if (characteristicUuid.startsWith(String.format("fd%02d", this.port + 3)))
					{
						this.handling = characteristic;
					}
				}
			}
		}

		if ((transferring != null) && (getting != null) && (handling != null))
		{
			/*error = bleManager.setCharacteristicNotification(gettingService, this.getting);
			if (error != ARSAL_ERROR_ENUM.ARSAL_OK)
			{
				ARSALPrint.e("DBG", APP_TAG + "set " + error.toString());
				ret = false;
			}*/

			if (ret == true)
			{
				bleManager.registerNotificationCharacteristics(this.arrayGetting, BLE_GETTING_KEY);
			}
		}

		return ret;
	}

	public boolean unregisterCharacteristics()
	{
		boolean ret = true;

		ARSALPrint.d("DBG", APP_TAG + "unregisterCharacteristics");

		ret = bleManager.unregisterNotificationCharacteristics(BLE_GETTING_KEY);

		return ret;
	}

	/***************************************************************/
	/* Public API
	/***************************************************************/
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

	public boolean listFilesAL(String remotePath, String[] resultList)
	{
	    boolean ret = true;

	    connectionLock.lock();
	    ret = listFiles(remotePath, resultList);
	    connectionLock.unlock();

	    return ret;
	}

	public boolean sizeFileAL(String remoteFile, double[] fileSize)
    {
        boolean ret = true;

        connectionLock.lock();
        ret = sizeFile(remoteFile, fileSize);
        connectionLock.unlock();

        return ret;
    }

	public boolean getFileAL(String remotePath, String localFile, long nativeCallbackObject, boolean wantProgress, Semaphore cancelSem)
    {
        boolean ret = false;

        connectionLock.lock();
        ret = getFile(remotePath, localFile, nativeCallbackObject, wantProgress, cancelSem);
        connectionLock.unlock();

        return ret;
    }

    public boolean getFileWithBufferAL(String remotePath, byte[][] data, long nativeCallbackObject, boolean wantProgress, Semaphore cancelSem)
    {
        boolean ret = false;

        connectionLock.lock();
        ret = getFileWithBuffer(remotePath, data, nativeCallbackObject, wantProgress, cancelSem);
        connectionLock.unlock();

        return ret;
    }

    public boolean putFileAL(String remotePath, String localFile, long nativeCallbackObject, boolean resume, Semaphore cancelSem)
    {
        boolean ret = false;

        connectionLock.lock();
        ret =  putFile(remotePath, localFile, nativeCallbackObject, resume, cancelSem);
        connectionLock.unlock();

        return ret;
    }

    public boolean deleteFileAL(String remoteFile)
    {
        boolean ret = true;

        connectionLock.lock();
        ret = deleteFile(remoteFile);
        connectionLock.unlock();

        return ret;
    }

    public boolean renameFileAL(String oldNamePath, String newNamePath)
    {
        boolean ret = true;

        connectionLock.lock();
        ret = renameFile(oldNamePath, newNamePath);
        connectionLock.unlock();

        return ret;
    }

	/***************************************************************/
    /* Internal Functions
    /***************************************************************/

	private boolean cancelFile(Semaphore cancelSem)
	{
		if (isListing) {
			return true;
		}

		cancelSem.release();

		boolean retVal = bleManager.cancelReadNotification(BLE_GETTING_KEY);

		return retVal;
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

	private boolean sizeFile(String remoteFile, double[] fileSize)
	{
		String[] resultList = new String[1];
		resultList[0] = null;
		String remotePath = null;
		String remoteFileName = null;
		int idx = 0;
		int endIdx = -1;
		boolean found = false;
		boolean ret = true;

		ARSALPrint.d("DBG", APP_TAG + "sizeFile " + remoteFile);

		remoteFile = normalizePathName(remoteFile);

		fileSize[0] = 0;

		while ((idx = remoteFile.indexOf('/', idx)) != -1)
		{
			idx++;
			endIdx = idx;
		}

		if (endIdx != -1)
		{
			remotePath = remoteFile.substring(0, endIdx);
			remoteFileName = remoteFile.substring(endIdx, remoteFile.length());
		}

		ret = listFiles(remotePath, resultList);

		if ((ret == true) && (resultList[0] != null))
		{
			String[] nextItem = new String[1];
			nextItem[0] = null;
			int[] indexItem = new int[1];
			indexItem[0] = 0;
			int[] itemLen = new int[1];
			itemLen[0] = 0;
			String fileName = null;

			while ((found == false) && (fileName = getListNextItem(resultList[0], nextItem, null, false, indexItem, itemLen)) != null)
			{
				ARSALPrint.d("DBG", APP_TAG + "file " + fileName);

				if (remoteFileName.contentEquals(fileName))
				{
					if (getListItemSize(resultList[0], indexItem[0], itemLen[0], fileSize) == null)
					{
						ret = false;
					}
					else
					{
						found = true;
					}
				}
			}
		}

		if (found == true)
	    {
	        ret = true;
	    }
	    else
	    {
	        ret = false;
	    }

		return ret;
	}

	private boolean listFiles(String remotePath, String[] resultList)
	{
		isListing = true;

		boolean ret = true;

		ARSALPrint.d("DBG", APP_TAG + "listFiles " + remotePath);

		remotePath = normalizePathName(remotePath);

		ret = sendCommand("LIS", remotePath, handling);

		if (ret == true)
		{
			byte[][] data = new byte[1][];

			ret = readGetData(0, null, data, 0, null);

			if (data[0] != null)
			{
				ARSALPrint.d("DBG", APP_TAG + "listFiles==" + new String(data[0]) + "==");
			}

			if ((ret == true) && (data[0] != null))
			{
				resultList[0] = new String(data[0]);
			}
		}

		isListing = false;

		return ret;
	}

	private boolean getFile(String remoteFile, String localFile, long nativeCallbackObject, boolean wantProgress, Semaphore cancelSem)
	{
		boolean ret = true;

		ret = getFileInternal(remoteFile, localFile, null, nativeCallbackObject, wantProgress, cancelSem);

		return ret;
	}

	private boolean getFileWithBuffer(String remoteFile, byte[][] data, long nativeCallbackObject, boolean wantProgress, Semaphore cancelSem)
	{
		boolean ret = true;

		ret = getFileInternal(remoteFile, null, data, nativeCallbackObject, wantProgress, cancelSem);

		return ret;
	}

	private boolean getFileInternal(String remoteFile, String localFile, byte[][] data, long nativeCallbackObject, boolean wantProgress, Semaphore cancelSem)
	{
		FileOutputStream dst = null;
		boolean ret = true;
		double[] totalSize = new double[1];
		totalSize[0] = 0.f;

		ARSALPrint.d("DBG", APP_TAG + "getFile " + remoteFile);

		remoteFile = normalizePathName(remoteFile);

		if (wantProgress)
		{
			ret = sizeFile(remoteFile, totalSize);
		}

        if ((ret == true) && (localFile != null))
        {
            try
            {
                dst = new FileOutputStream(localFile);
            }
            catch(FileNotFoundException e)
            {
                ARSALPrint.e("DBG", APP_TAG + e.toString());
                ret = false;
            }
        }

		if (ret == true)
		{
			ret = sendCommand("GET", remoteFile, handling);
		}

		if (ret == true)
		{
			ret = readGetData((int)totalSize[0], dst, data, nativeCallbackObject, cancelSem);
		}

		if (dst != null)
		{
			try { dst.close(); } catch(IOException e) { }
		}

		return ret;
	}

	private boolean abortPutFile(String remoteFile)
	{
		int[] resumeIndex = new int[1];
		resumeIndex[0] = 0;
		int[] remoteSize = new int[1];
		remoteSize[0] = 0;
		boolean resume = false;
		boolean ret = true;

		remoteFile = normalizePathName(remoteFile);

		ret = readPutResumeIndex(remoteFile, resumeIndex, remoteSize);
		if ((ret == true) && (resumeIndex[0] > 0))
		{
			resume = true;
		}
		else
		{
			resume = false;
		}

		if (resume == true)
		{
			ret = sendCommand("PUT", remoteFile, handling);

			if (ret == true)
			{
				ret = sendPutData(0, null, resumeIndex[0], false, true, 0, null);
			}
		}

		deleteFile(remoteFile);

		return ret;
	}

	private boolean putFile(String remoteFile, String localFile, long nativeCallbackObject, boolean resume, Semaphore cancelSem)
	{
		FileInputStream src = null;
		int[] resumeIndex = new int[1];
		resumeIndex[0] = 0;
	    int[] remoteSize = new int[1];
	    remoteSize[0] = 0;
		boolean ret = true;
		int totalSize = 0;

		ARSALPrint.d("DBG", APP_TAG + "putFile " + remoteFile);

		remoteFile = normalizePathName(remoteFile);

		if (resume == false)
		{
			abortPutFile(remoteFile);
		}
		else
		{
			if (ret == true)
			{
				ret = readPutResumeIndex(remoteFile, resumeIndex, remoteSize);
				if (ret == false)
				{
					ret = true;
					resumeIndex[0] = 0;
					resume = false;
				}
			}

			if (resumeIndex[0] > 0)
			{
				resume = true;
			}
		}

		ARUtilsFileSystem fileSys = new ARUtilsFileSystem();
		try
		{
			totalSize = (int)fileSys.getFileSize(localFile);
		}
		catch (ARUtilsException e)
		{
		    ARSALPrint.e("DBG", APP_TAG + e.toString());
			ret = false;
		}

		if ((ret == true) && (resume == true) && (remoteSize[0] == totalSize))
		{
		    ARSALPrint.d("DBG", APP_TAG + "full resume");
		    if (nativeCallbackObject != 0)
            {
                nativeProgressCallback(nativeCallbackObject, 100.f);
            }
		}
		else
		{
    		if (ret == true)
    		{
    			ret = sendCommand("PUT", remoteFile, handling);
    		}

    		if (ret == true)
    		{
    			try
    			{
    				src = new FileInputStream(localFile);
    			}
    			catch(FileNotFoundException e)
    			{
    			    ARSALPrint.e("DBG", APP_TAG + e.toString());
    				ret = false;
    			}
    		}

    		if (ret == true)
    		{
    			ret = sendPutData(totalSize, src, resumeIndex[0], resume, false, nativeCallbackObject, cancelSem);
    		}

    		if (src != null)
    		{
    			try { src.close(); } catch(IOException e) { }
    		}
		}

		return ret;
	}

	private boolean deleteFile(String remoteFile)
	{
		boolean ret = true;

		ret = sendCommand("DEL", remoteFile, handling);

		if (ret == true)
		{
			ret = readDeleteData();
		}

		return ret;
	}

	private boolean renameFile(String oldNamePath, String newNamePath)
	{
        boolean ret = true;
        String cmd = "REN";
		String param = oldNamePath + " " + newNamePath;

		if ((cmd.length() + param.length()) > BLE_PACKET_MAX_SIZE)
		{
            ret = renameLongFile(oldNamePath, newNamePath);
		}
		else
		{
            ret = renameShortFile(oldNamePath, newNamePath);
		}

		return ret;
	}

	private boolean renameShortFile(String oldNamePath, String newNamePath)
	{
		boolean ret = true;
		String param = oldNamePath + " " + newNamePath;

		ret = sendCommand("REN", param, handling);

		if (ret == true)
		{
			ret = readRenameData();
		}

		return ret;
	}

	private boolean renameLongFile(String oldNamePath, String newNamePath)
	{
		boolean ret = true;

		ret = sendCommand("RNFR", oldNamePath, handling);

		if (ret == true)
		{
			ret = readRenameData();
		}

		if (ret == true)
		{
			ret = sendCommand("RNTO", newNamePath, handling);
		}

		if (ret == true)
		{
			ret = readRenameData();
		}

		return ret;
	}

	private boolean sendCommand(String cmd, String param, BluetoothGattCharacteristic characteristic)
	{
		boolean ret = true;
		byte[] bufferParam = null;
		byte[] bufferCmd = null;
		byte[] buffer = null;
		int indexBuffer = 0;

		try
		{
			bufferCmd = cmd.getBytes("UTF8");
		}
		catch (UnsupportedEncodingException e)
		{
		    ARSALPrint.e("DBG", APP_TAG + e.toString());
			ret = false;
		}

		if ((ret == true) && (param != null))
		{
			try
			{
				bufferParam = param.getBytes("UTF8");
			}
			catch (UnsupportedEncodingException e)
			{
			    ARSALPrint.e("DBG", APP_TAG + e.toString());
				ret = false;
			}
		}

		if (ret == true)
		{
			if ((bufferParam != null) && ((cmd.length() + bufferParam.length + 1) > BLE_PACKET_MAX_SIZE))
			{
			    ARSALPrint.e("DBG", APP_TAG + "Block size error");
				ret = false;
			}
		}

		if (ret == true)
		{
			if (bufferParam == null)
			{
				buffer = new byte[bufferCmd.length + 1];
			}
			else
			{
				buffer = new byte[bufferCmd.length + bufferParam.length + 1];
			}

		    System.arraycopy(bufferCmd, 0, buffer, 0, bufferCmd.length);
		    indexBuffer = bufferCmd.length;

		    if (bufferParam != null)
		    {
			    System.arraycopy(bufferParam, 0, buffer, indexBuffer, bufferParam.length);
			    indexBuffer += bufferParam.length;
		    }
		    buffer[indexBuffer] = 0;
		    ret = sendBufferBlocks(buffer, characteristic);
        }

		return ret;
	}

	private boolean sendResponse(String cmd, BluetoothGattCharacteristic characteristic)
	{
		boolean ret = true;
		byte[] bufferCmd = null;
		byte[] buffer = null;
		int indexBuffer = 0;

		try
		{
			try
			{
				bufferCmd = cmd.getBytes("UTF8");
			}
			catch (UnsupportedEncodingException e)
			{
			    ARSALPrint.e("DBG", APP_TAG + e.toString());
				ret = false;
			}

			if (ret == true)
			{
				if (((cmd.length() + 1) > BLE_PACKET_MAX_SIZE))
				{
				    ARSALPrint.e("DBG", APP_TAG + "Block size error");
					ret = false;
				}
			}

			if (ret == true)
			{
				buffer = new byte[bufferCmd.length + 1];

			    System.arraycopy(bufferCmd, 0, buffer, 0, bufferCmd.length);
			    indexBuffer = bufferCmd.length;

			    buffer[indexBuffer] = 0;

			    Thread.sleep(BLE_PACKET_WRITE_SLEEP, 0);
			    ret = bleManager.writeData(buffer, characteristic);
			}
		}
		catch (InterruptedException e)
		{
		}

		return ret;
	}

	private boolean sendResponse(byte[] buffer, BluetoothGattCharacteristic characteristic)
	{
		boolean ret = true;

		try
		{
			Thread.sleep(BLE_PACKET_WRITE_SLEEP, 0);
			ret = bleManager.writeData(buffer, characteristic);
		}
		catch (InterruptedException e)
		{
		}

		return ret;
	}

	private boolean sendBufferBlocks(byte[] buffer, BluetoothGattCharacteristic characteristic)
	{
		boolean ret = true;

		try
		{
			int bufferIndex = 0;
			if (buffer.length == 0)
			{
				byte[] block = new byte[1];
				block[0] = BLE_BLOCK_HEADER_SINGLE;
				Thread.sleep(BLE_PACKET_WRITE_SLEEP, 0);
				ret = bleManager.writeData(block, characteristic);

				ARSALPrint.d("DBG", APP_TAG + "block " + 1 + ", " + 0);
			}
			else
			{
				while ((ret == true) && (bufferIndex < buffer.length))
				{
					int blockSize = BLE_MTU_SIZE;
					if ((buffer.length - bufferIndex) <= (BLE_MTU_SIZE -1))
					{
						blockSize = (buffer.length - bufferIndex) + 1;
					}
					byte[] block = new byte[blockSize];

					if (buffer.length < BLE_MTU_SIZE)
					{
						block[0] = BLE_BLOCK_HEADER_SINGLE;
					}
					else
					{
						if (bufferIndex == 0)
						{
							block[0] = BLE_BLOCK_HEADER_START;
						}
						else if (bufferIndex + (BLE_MTU_SIZE - 1) >= buffer.length)
						{
							block[0] = BLE_BLOCK_HEADER_STOP;
						}
						else
						{
							block[0] = BLE_BLOCK_HEADER_CONTINUE;
						}
					}

					System.arraycopy(buffer, bufferIndex, block, 1, blockSize - 1);
					bufferIndex += blockSize - 1;
					Thread.sleep(BLE_PACKET_WRITE_SLEEP, 0);
					ret = bleManager.writeData(block, characteristic);

					ARSALPrint.d("DBG", APP_TAG + "block " + blockSize + ", " + bufferIndex);
				}
			}
		}
		catch (InterruptedException e)
		{
		    ARSALPrint.e("DBG", APP_TAG + e.toString());
		}

		return ret;
	}

	private boolean readBufferBlocks(byte[][] notificationArray)
	{
		ArrayList<ARSALManagerNotificationData> receivedNotifications = new ArrayList<ARSALManagerNotificationData>();
		boolean ret = true;
		boolean end = false;
		int bufferIndex = 0;
		byte[] buffer = new byte[BLE_PACKET_MAX_SIZE];
		int blockCount = 0;

		do
		{
			if (receivedNotifications.size() == 0)
			{
				ret = bleManager.readDataNotificationData(receivedNotifications, 1, BLE_GETTING_KEY);
				/** we need continue to read notification until the end message, even though it's failed due to cancel operation
				 * (cancel --> semaphore release --> no notification to read --> failed)
				 * This is very useful when a message is cut into small pieces (HEADER_START, HEADER_CONTINUE, HEADER_CONTINUE ..., HEADER_STOP),
				 * we need insure that a complete message is read in this function
				 */
				if (! ret)
				{
					ret = bleManager.isDeviceConnected();
				}
			}

			if ((ret == true) && (receivedNotifications.size() > 0))
			{
				ARSALManagerNotificationData notificationData = null;
				notificationData = receivedNotifications.get(0);
				int blockLen = notificationData.value.length;
				byte[] block = notificationData.value;
				int blockIndex = 0;

				ARSALPrint.d("DBG", APP_TAG + "block length " + blockLen);
				if (blockLen > 0)
				{
					switch(block[0])
					{
					case BLE_BLOCK_HEADER_SINGLE:
					case BLE_BLOCK_HEADER_STOP:
						ARSALPrint.d("DBG", APP_TAG + "this is the last block.");
						end = true;
						blockLen = blockLen - 1;
						blockIndex = 1;
						break;

					case BLE_BLOCK_HEADER_CONTINUE:
					case BLE_BLOCK_HEADER_START:
						blockLen = blockLen - 1;
						blockIndex = 1;
						break;
					default:
					    ARSALPrint.e("DBG", APP_TAG + "Block state error");
						ret = false;
						break;
					}

					if (ret == true)
					{
						/* Check that the buffer is large enough to hold the current
						 * data block. If it isn't, enlarge it by one allocation unit.
						 */
						if ((bufferIndex + blockLen) > buffer.length)
						{
							int minSize = bufferIndex + blockLen;
							int newSize = buffer.length + BLE_PACKET_MAX_SIZE;
							/* Just in the (unlikely) event when block would be larger
							 * than BLE_PACKET_MAX_SIZE. */
							if (newSize < minSize)
							{
								newSize = minSize;
							}
							ARSALPrint.d("DBG", APP_TAG + "buffer alloc size " + buffer.length + " -> " + newSize);
							byte[] oldBuffer = buffer;
							buffer = new byte[newSize];
							System.arraycopy(oldBuffer, 0, buffer, 0, oldBuffer.length);
						}

						/* Copy the current block to the packet buffer. */
						System.arraycopy(block, blockIndex, buffer, bufferIndex, blockLen);
						bufferIndex += blockLen;
						blockCount++;

						ARSALPrint.d("DBG", APP_TAG + "block " + blockCount +", "+ blockLen +", "+ bufferIndex);
					}
				}
				else
				{
					//ret = false;
					ARSALPrint.d("DBG", APP_TAG + "Empty block ");
				}

				receivedNotifications.remove(notificationData);
			}
		}
		while ((ret == true) && (end == false));

		if (bufferIndex > 0)
		{
			/* Allocate a buffer just the right size copy the read packet to it. */
			byte[] finalBuffer = new byte[bufferIndex];
			System.arraycopy(buffer, 0, finalBuffer, 0, bufferIndex);

			if ((notificationArray != null) && (notificationArray.length > 0))
			{
				notificationArray[0] = finalBuffer;
			}
			else
			{
				ret = false;
			}
		}

		return ret;
	}

	private boolean sendPutData(int fileSize, FileInputStream src, int resumeIndex, boolean resume, boolean abort, long nativeCallbackObject, Semaphore cancelSem)
	{
		BufferedInputStream in = new BufferedInputStream(src);
		byte[] buffer = new byte [BLE_PACKET_MAX_SIZE];
		boolean ret = true;
		int packetLen = 0;
		int packetCount = 0;
		int totalPacket = 0;
		int totalSize = 0;
		boolean endFile = false;
		ARUtilsMD5 md5 = new ARUtilsMD5();
		ARUtilsMD5 md5End = new ARUtilsMD5();
		String[] md5Msg = new String[1];
		md5Msg[0] = "";
		String md5Txt = null;
		byte[] send = null;
        float percent = 0.f;
        float lastPercent = 0.f;

		try
		{
			if (abort == true)
		    {
		        endFile = true;
		        resumeIndex = 0;
		        resume = false;
		    }

			do
			{
				if (abort == false)
				{
					packetLen = in.read(buffer, 0, BLE_PACKET_MAX_SIZE);
				}

				if (packetLen > 0)
				{
					packetCount++;
					totalPacket++;
					totalSize += packetLen;

					md5End.update(buffer, 0, packetLen);

					if ((resume == false) || ((resume == true) && (totalPacket > resumeIndex)))
					{
						md5.update(buffer, 0, packetLen);

						send = buffer;
						if (packetLen != BLE_PACKET_MAX_SIZE)
						{
							send = new byte[packetLen];
							System.arraycopy(buffer, 0, send, 0, packetLen);
						}

						ret = sendBufferBlocks(send, transferring);

						ARSALPrint.d("DBG", APP_TAG + "packet " + packetCount + ", " + packetLen);
					}
					else
					{
						ARSALPrint.d("DBG", APP_TAG + "resume " + packetCount);
					}

					if (nativeCallbackObject != 0)
					{
                        percent = ((float)totalSize / (float)fileSize) * 100.f;
                        if ((resume == true) && (totalPacket < resumeIndex))
                        {
                            if ((percent - lastPercent) > 1.f)
                            {
                                lastPercent = percent;
                                nativeProgressCallback(nativeCallbackObject, percent);
                            }
                        }
                        else
                        {
						    nativeProgressCallback(nativeCallbackObject, percent);
                        }
					}

					if (isConnectionCanceled(cancelSem))
					{
					    ARSALPrint.e("DBG", APP_TAG + "Canceled received");
                        send = new byte[0];
                        boolean err = false;

				        err = sendResponse(send, transferring);
                        err = readPutMd5(md5Msg);

						ret = false;
					}
				}
				else
				{
					if (packetLen == -1)
					{
						endFile = true;
					}
				}

				if ((ret == true) && ((packetCount >= BLE_PACKET_BLOCK_PUTTING_COUNT) || ((endFile == true) && (packetCount > 0))))
				{
					packetCount = 0;

					if ((resume == false) || ((resume ==  true) && (totalPacket > resumeIndex)))
					{
						md5Txt = md5.digest();
						md5.initialize();

						ARSALPrint.d("DBG", APP_TAG + "sending md5 " + md5Txt);

						md5Txt = "MD5" + md5Txt;
						send = md5Txt.getBytes("UTF8");
						ret = sendBufferBlocks(send, transferring);

						if (ret == true)
						{
							ret = readPudDataWritten();
						}
					}
				}
			}
			while ((ret == true) && (endFile == false));

			if ((ret == true) && (endFile == true))
			{
				send = new byte[0];
				ret = sendResponse(send, transferring);

				if (ret == true)
				{
					ret = readPutMd5(md5Msg);
				}

				if (ret == true)
				{
					md5Txt = md5End.digest();

					ARSALPrint.d("DBG", APP_TAG + "md5 end" + md5Txt);

					if (md5Msg[0].compareTo(md5Txt) != 0)
					{
						ARSALPrint.e("DBG", APP_TAG + "md5 end Failed");
						ret = false;
					}
					else
					{
						ARSALPrint.d("DBG", APP_TAG + "md5 end ok");
					}
				}
			}
		}
		catch (IOException e)
		{
			ARSALPrint.e("DBG", APP_TAG + e.toString());
			ret = false;
		}

		return ret;
	}

	/*private boolean readPutResumeIndex(int[] resumeIndex)
	{
		ArrayList<ARSALManagerNotificationData> receivedNotifications = new ArrayList<ARSALManagerNotificationData>();
		boolean ret = true;

		bleManager.readData(getting);

		ret = bleManager.readDataNotificationData(receivedNotifications, 1, BLE_GETTING_KEY);
		if (ret = true)
		{
			if (receivedNotifications.size() > 0)
			{
				ARSALManagerNotificationData  notificationData = receivedNotifications.get(0);
				byte[] packet = notificationData.value;
				int packetLen = notificationData.value.length;

				if (packetLen == 3)
				{
					int size = (0xff & packet[0]) | (0xff00 & (packet[1] << 8)) | (0xff0000 & (packet[2] << 16));
					resumeIndex[0] = size;
					ARSALPrint.d("DBG", APP_TAG + "resume index " + size);
				}
				else
				{
				ARSALPrint.e("DBG", APP_TAG + "eee");
					ret = false;
				}
			}
			else
			{
				ret = false;
			}
		}

		return ret;
	}*/
	private boolean readPutResumeIndex(String remoteFile, int[] resumeIndex, int[]totalSize)
	{
		double[] fileSize = new double[1];
		fileSize[0] = 0.f;
		boolean ret = true;

		resumeIndex[0] = 0;
		ret = sizeFile(remoteFile, fileSize);
		if (ret == true)
		{
			resumeIndex[0] = (int)fileSize[0] / BLE_PACKET_MAX_SIZE;
			totalSize[0] = (int)fileSize[0];
		}

		return ret;
	}

	private boolean readGetData(int fileSize, FileOutputStream dst, byte[][] data, long nativeCallbackObject, Semaphore cancelSem)
	{
		byte[][] notificationArray = new byte[1][];
		boolean ret = true;
		int packetCount = 0;
		int totalSize = 0;
		int totalPacket = 0;
		boolean endFile = false;
		boolean endMD5 = false;
		String md5Msg = null;
		String md5Txt = null;
		ARUtilsMD5 md5 = new ARUtilsMD5();
		ARUtilsMD5 md5End = new ARUtilsMD5();

		while ((ret == true) && (endMD5 == false))
		{
			boolean blockMD5 = false;
			md5.initialize();

			do
			{
				ret = readBufferBlocks(notificationArray);
				if (!ret)
				{
					ret = bleManager.isDeviceConnected();
				}
				else if (ret)
				{
					//for (int i=0; (i < receivedNotifications.size()) && (ret == true) && (blockMD5 == false) && (endMD5 == false); i++)
					if ((ret == true) && (notificationArray[0] != null) && (blockMD5 == false) && (endMD5 == false))
					{
						int packetLen = notificationArray[0].length;
						byte[] packet = notificationArray[0];

						packetCount++;
						totalPacket++;
						ARSALPrint.d("DBG", APP_TAG + "== packet " + packetLen +", "+ packetCount + ", " + totalPacket + ", " + totalSize);
						//String s = new String(packet);
						//ARSALPrint.d("DBG", APP_TAG + "packet " + s);

						if (packetLen > 0)
						{
							if (endFile == true)
							{
								endMD5 = true;

								if (packetLen == (ARUtilsMD5.MD5_LENGTH * 2))
								{
									md5Msg = new String(packet, 0, packetLen);
									ARSALPrint.d("DBG", APP_TAG + "md5 end received " + md5Msg);
								}
								else
								{
									ret = false;
									ARSALPrint.d("DBG", APP_TAG + "md5 end failed size " + packetLen);
								}
							}
							else if (compareToString(packet, packetLen, BLE_PACKET_EOF))
							{
								endFile = true;

								if (packetLen == (BLE_PACKET_EOF.length() + 1))
								{
									ARSALPrint.d("DBG", APP_TAG + "End of file received ");
								}
								else
								{
									ARSALPrint.d("DBG", APP_TAG + "End of file failed size " + packetLen);
									ret = false;
								}
							}
							else if (compareToString(packet, packetLen, "MD5"))
							{
								if (packetCount > (BLE_PACKET_BLOCK_GETTING_COUNT + 1))
								{
									ARSALPrint.d("DBG", APP_TAG + "md5 failed packet count " + packetCount);
								}

								if (packetLen == ((ARUtilsMD5.MD5_LENGTH * 2) + 3))
								{
									blockMD5 = true;
									md5Msg = new String(packet, 3, packetLen - 3);
									ARSALPrint.d("DBG", APP_TAG + "md5 received " + md5Msg);
								}
								else
								{
									ret = false;
									ARSALPrint.d("DBG", APP_TAG + "md5 failed size " + packetLen);
								}
							}
							/**
							 * when receiving the error "Error : Local file doesn't exist" message from drone, and if it isn't recognized,
							 * the process will be blocked to waiting data, which means this task never stop
							 */
							else if (compareToStringIgnoreCase(packet, packetLen, "error")) // the error "Error : Local file doesn't exist" should be recognized
							{
								ARSALPrint.e("DBG", APP_TAG + "Error received");
								ret = false;
							}
							else
							{
								totalSize += packetLen;
								md5.update(packet, 0, packetLen);
								md5End.update(packet, 0, packetLen);

								if (dst != null)
								{
									try
									{
										dst.write(packet, 0, packetLen);
									}
									catch (IOException e)
									{
										ARSALPrint.e("DBG", APP_TAG + "failed writting file " + e.toString());
										ret = false;
									}
								}
								else
								{
									byte[] newData = new byte[totalSize];
									if (data[0] != null)
									{
										System.arraycopy(data[0], 0, newData, 0, totalSize - packetLen);
									}
									System.arraycopy(packet, 0, newData, totalSize - packetLen, packetLen);
									data[0] = newData;
								}

								if (nativeCallbackObject != 0 && fileSize != 0)
								{
									nativeProgressCallback(nativeCallbackObject, ((float)totalSize / fileSize) * 100.f);
								}
							}
						}
						else
						{
							//empty packet authorized
						}
					}
				}

				//receivedNotifications.clear();
				notificationArray[0] = null;
			}
			while ((ret == true) && (blockMD5 == false) && (endMD5 == false));

			if ((ret == true) && (blockMD5 == true))
			{
				blockMD5 = false;
				packetCount = 0;
				md5Txt = md5.digest();

				if (md5Msg.contentEquals(md5Txt) == false)
				{
					ARSALPrint.d("DBG", APP_TAG + "md5 block failed");
					//TOFIX some 1st md5 packet failed !!!!
					//ret = false;
				}
				else
				{
					ARSALPrint.d("DBG", APP_TAG + "md5 block ok");
				}

				//firmware 1.0.45 protocol dosen't implement cancel today at the and of 100 packets download
				if (isConnectionCanceled(cancelSem))
		        {
				    ARSALPrint.e("DBG", APP_TAG + "Canceled received");
		            ret = false;
		        }

		        if (ret == false)
		        {
		        	ret = sendResponse("CANCEL", getting);
		        }
		        else
		        {
					ret = sendResponse("MD5 OK", getting);
		        }
			}
		}

		if (endMD5 == true)
		{
			md5Txt = md5End.digest();
			ARSALPrint.d("DBG", APP_TAG + "md5 end computed " + md5Txt);

			if (md5Msg.contentEquals(md5Txt) == false)
			{
				ARSALPrint.d("DBG", APP_TAG + "md5 end Failed");
				ret = false;
			}
			else
			{
				ARSALPrint.d("DBG", APP_TAG + "md5 end OK");
			}
		}
		else
		{
			ret = false;
		}

		if (isConnectionCanceled(cancelSem))
        {
		    ARSALPrint.e("DBG", APP_TAG + "Canceled received");
            ret = false;
        }

		return ret;
	}

	private boolean readPudDataWritten()
	{
		byte[][] notificationArray = new byte[1][];
		boolean ret = false;

		ret = readBufferBlocks(notificationArray);
		if (ret = true)
		{
			if (notificationArray[0] != null)
			{
				int packetLen = notificationArray[0].length;
				byte[] packet = notificationArray[0];

				if (packetLen > 0)
				{
					//String packetTxt = new String(packet, 0, packetLen, "UTF8");

					if (compareToString(packet, packetLen, BLE_PACKET_WRITTEN))
					{
						ARSALPrint.d("DBG", APP_TAG + "Written OK");
						ret = true;
					}
					else if (compareToString(packet, packetLen, BLE_PACKET_NOT_WRITTEN))
					{
						ARSALPrint.e("DBG", APP_TAG + "NOT Written");
						ret = false;
					}
					else
					{
						ARSALPrint.e("DBG", APP_TAG + "UNKNOWN Written");
						ret = false;
					}
				}
			}
			else
			{
				ARSALPrint.e("DBG", APP_TAG + "UNKNOWN Written");
				ret = false;
			}
		}

		return ret;
	}

	private boolean readPutMd5(String[] md5Txt)
	{
		byte[][] notificationArray = new byte[1][];
		boolean ret = false;

		md5Txt[0] = "";

		try
		{
			ret = readBufferBlocks(notificationArray);
			if (ret == true)
			{
				if (notificationArray[0] != null)
				{
					int packetLen = notificationArray[0].length;
					byte[] packet = notificationArray[0];

					if (packetLen > 0)
					{
						if (packetLen == 32)
						{
							String packetTxt = new String(packet, 0, packetLen, "UTF8");
							md5Txt[0] = packetTxt;

							ARSALPrint.d("DBG", APP_TAG + "md5 end received " + md5Txt[0]);
						}
						else
						{
							ARSALPrint.e("DBG", APP_TAG + "md5 size failed");
							ret = false;
						}
					}
					else
					{
						ARSALPrint.e("DBG", APP_TAG + "md5 end failed");
						ret = false;
					}
				}
				else
				{
				    ARSALPrint.e("DBG", APP_TAG + "md5 end size failed");
					ret = false;
				}
			}
		}
		catch (UnsupportedEncodingException e)
		{
			ARSALPrint.e("DBG", APP_TAG + e.toString());
			ret = false;
		}

		return ret;
	}

	private boolean readRenameData()
	{
		byte[][] notificationArray = new byte[1][];
		boolean ret = false;

		ret = readBufferBlocks(notificationArray);
		if (ret == true)
		{
			if (notificationArray[0] != null)
			{
				int packetLen = notificationArray[0].length;
				byte[] packet = notificationArray[0];

				if (packetLen > 0)
				{
					//String packetString = new String(packet);
					if (compareToString(packet, packetLen, BLE_PACKET_RENAME_SUCCESS))
					{
					    ARSALPrint.d("DBG", APP_TAG + "Rename Success");
						ret = true;
					}
					else
					{
					    ARSALPrint.e("DBG", APP_TAG + "Rename Failed");
						ret = false;
					}
				}
				else
				{
				    ARSALPrint.e("DBG", APP_TAG + "Rename Failed");
					ret = false;
				}
			}
			else
			{
			    ARSALPrint.e("DBG", APP_TAG + "Rename Failed");
				ret = false;
			}
		}
		return ret;
	}

	private boolean readDeleteData()
	{
		byte[][] notificationArray = new byte[1][];
		boolean ret = false;

		ret = readBufferBlocks(notificationArray);
		if (ret == true)
		{
			if (notificationArray[0] != null)
			{
				int packetLen = notificationArray[0].length;
				byte[] packet = notificationArray[0];

				if (packetLen > 0)
				{
					//String packetString = new String(packet);
					if (compareToString(packet, packetLen, BLE_PACKET_DELETE_SUCCESS))
					{
					    ARSALPrint.d("DBG", APP_TAG + "Delete Success");
						ret = true;
					}
					else
					{
					    ARSALPrint.e("DBG", APP_TAG + "Delete Failed");
						ret = false;
					}
				}
				else
				{
				    ARSALPrint.e("DBG", APP_TAG + "Delete Failed");
					ret = false;
				}
			}
			else
			{
			    ARSALPrint.e("DBG", APP_TAG + "Delete Failed");
				ret = false;
			}
		}
		return ret;
	}

	public static String getListNextItem(String list, String[] nextItem, String prefix, boolean isDirectory, int[] indexItem, int[] itemLen)
	{
	    String lineData = null;
	    String item = null;
	    String line = null;
	    int fileIdx = 0;
	    int endLine = 0;
	    int ptr;

	    if ((list != null) && (nextItem != null))
	    {
	        if (nextItem[0] == null)
	        {
	            nextItem[0] = list;
	            if (indexItem != null)
	            {
	            	indexItem[0] = 0;
	            }
	        }
	        else
	        {
	        	if (indexItem != null)
	            {
	            	indexItem[0] += itemLen[0];
	            }
	        }

	        ptr = 0;
	        while ((item == null) && (ptr != -1))
	        {
	            line = nextItem[0];
	            endLine =  line.length();
	            ptr = line.indexOf('\n');
	            if (ptr == -1)
	            {
	                ptr = line.indexOf('\r');
	            }

	            if (ptr != -1)
	            {
	                endLine = ptr;
	                if (line.charAt(endLine - 1) == '\r')
	                {
	                    endLine--;
	                }

	                ptr++;
	                nextItem[0] = line.substring(ptr);
	                fileIdx = 0;
	                if (line.charAt(0) == ((isDirectory  == true) ? 'd' : '-'))
	                {
	                    int varSpace = 0;
	                    while (((ptr = line.indexOf('\u0020', fileIdx)) != -1) && (ptr < endLine) && (varSpace < 8))
	                    {
	                        if (line.charAt(ptr + 1) != '\u0020')
	                        {
	                            varSpace++;
	                        }

	                        fileIdx = ++ptr;
	                    }

	                    if ((prefix != null) && (prefix.length() != 0))
	                    {
	                        if (line.indexOf(prefix, fileIdx) != -1)
	                        {
	                            fileIdx = -1;
	                        }
	                    }

	                    if (fileIdx != -1)
	                    {
	                    	int len = endLine - fileIdx;
	                        lineData = line.substring(fileIdx, fileIdx + len);
	                        item = lineData;
	                    }
	                }
	            }
	        }

	        if (itemLen != null)
	        {
	            itemLen[0] = endLine;
	            //ARSALPrint.d("DBG", APP_TAG + "LINE " + list.substring(indexItem[0], indexItem[0] + itemLen[0]));
	        }
	    }

	    return item;
	}

	//-rw-r--r--    1 root     root       1210512 Jan  1 02:46 ckcm.bin
	public static String getListItemSize(String list, int lineIndex, int lineSize, double[] size)
	{
	    int fileIdx;
	    int sizeIdx;
	    int endLine;
	    int ptr;
	    String item = null;
	    int varSpace = 0;

	    if ((list != null) && (size != null))
	    {
	        size[0] = 0.f;
	        endLine = lineIndex + lineSize;
	        sizeIdx = -1;
	        fileIdx = lineIndex;

	        while ((ptr = list.indexOf('\u0020', fileIdx)) != -1 && (ptr < endLine) && (varSpace < 3))
	        {
	            if ((list.charAt(ptr - 1) == '\u0020') && (list.charAt(ptr + 1) != '\u0020'))
	            {
	                varSpace++;
	                if ((list.charAt(0) == '-'))
	                {
	                    if ((varSpace == 3) && (sizeIdx == -1))
	                    {
	                        sizeIdx = ptr + 1;
	                        String subLine = list.substring(sizeIdx);
	                        Scanner scanner = new Scanner(subLine);
	                        try
	                        {
	                        	size[0] = scanner.nextDouble();
	                        }
	                        catch (InputMismatchException e)
	                        {
	                        	size[0] = 0.f;
	                        }
	                        catch (IllegalStateException e)
	                        {
	                        	size[0] = 0.f;
	                        }
	                        catch (NoSuchElementException e)
	                        {
	                        	size[0] = 0.f;
	                        }
	                        scanner.close();
	                        item = subLine;
	                    }
	                }
	            }
	            fileIdx = ++ptr;
	        }
	    }

	    return item;
	}

    private boolean compareToStringIgnoreCase(byte[] buffer, int len, String str)
    {
        try
        {
            byte[] strBytes = str.toLowerCase().getBytes("UTF8");
            if (len >= strBytes.length)
            {
                int delta = 'A' - 'a';
                for (int i = 0; i < strBytes.length; i++)
                {
                    if (buffer[i] != strBytes[i] && buffer[i] != (strBytes[i] + delta))
                    {
                        return false;
                    }
                }
                return true;
            }
        }
        catch (UnsupportedEncodingException e)
        {
            e.printStackTrace();
        }
        return false;
    }

    private boolean compareToString(byte[] buffer, int len, String str)
    {
        boolean ret = false;
        byte[] strBytes = null;

        try
        {
            strBytes = str.getBytes("UTF8");
            if (len >= strBytes.length)
            {
                ret = true;
                for (int i=0; i<strBytes.length; i++)
                {
                    if (buffer[i] != strBytes[i])
                    {
                        ret = false;
                        break;
                    }
                }
            }
            else
            {
                ret = false;
            }
        }
        catch (UnsupportedEncodingException e)
        {
            ARSALPrint.e("DBG", APP_TAG + e.toString());
            ret = false;
        }

        return ret;
    }

    private String normalizePathName(String name)
    {
        String newName = name;
        if (name.charAt(0) != '/')
        {
            newName = "/" + name;
        }

        return newName;
    }
}
