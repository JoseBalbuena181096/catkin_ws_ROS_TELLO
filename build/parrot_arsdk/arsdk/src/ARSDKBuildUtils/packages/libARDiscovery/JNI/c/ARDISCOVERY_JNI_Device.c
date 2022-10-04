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
/**
 * @file ARDISCOVERY_JNI_Device.c
 * @brief libARDiscovery JNI device c file.
 **/

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <jni.h>
#include <stdlib.h>

#include <libARSAL/ARSAL_Print.h>

#include <libARDiscovery/ARDISCOVERY_Error.h>
#include <libARDiscovery/ARDISCOVERY_Device.h>

#include <BLE/ARDISCOVERY_DEVICE_Ble.h>
#include "ARDISCOVERY_JNI_DEVICE_Ble.h"

/*****************************************
 *
 *             define :
 *
 *****************************************/

#define ARDISCOVERY_JNIDEVICE_TAG "JNIDiscoveryDevice"

/*****************************************
 *
 *             private header:
 *
 *****************************************/
eARDISCOVERY_ERROR ARDISCOVERY_JNI_Device_SendJsonCallback (json_object *jsonObj, void *customData);
eARDISCOVERY_ERROR ARDISCOVERY_JNI_Device_ReceiveJsonCallback (json_object *jsonObj, void *customData);
eARDISCOVERY_ERROR ARDISCOVERY_JNI_Device_InitBLE (JNIEnv *env, ARDISCOVERY_Device_t *device, eARDISCOVERY_PRODUCT product, jobject jBLEPart);

/*****************************************
 *
 *             implementation :
 *
 *****************************************/

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryDevice_nativeStaticInit (JNIEnv *env, jclass class)
{
    /* local declarations */
    jclass jBLEPartCls = NULL;

    /* get ARDiscoveryConnection */
    jBLEPartCls = (*env)->FindClass(env, "com/parrot/arsdk/ardiscovery/ARDiscoveryDevice$BLEPart");

    ARDISCOVERY_JNIDEVICE_BLE_METHOD_NEWNETWORKAL = (*env)->GetMethodID (env, jBLEPartCls, "newARNetworkAL", "()J");
    ARDISCOVERY_JNIDEVICE_BLE_METHOD_DELETENETWORKAL = (*env)->GetMethodID (env, jBLEPartCls, "deleteARNetworkAL", "()I");

    /* cleanup */
    (*env)->DeleteLocalRef (env, jBLEPartCls);
}

/**
 * @brief get ROLLINGSPIDER_CONTROLLER_TO_DEVICE_NONACK_ID
 * @return value of ROLLINGSPIDER_CONTROLLER_TO_DEVICE_NONACK_ID
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryDevice_nativeGetCToDNonAckId (JNIEnv *env, jclass class)
{
    return ROLLINGSPIDER_CONTROLLER_TO_DEVICE_NONACK_ID;
}

/**
 * @brief get ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID
 * @return value of ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryDevice_nativeGetCToDAckId (JNIEnv *env, jclass class)
{
    return ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID;
}

/**
 * @brief get ROLLINGSPIDER_CONTROLLER_TO_DEVICE_EMERGENCY_ID
 * @return value of ROLLINGSPIDER_CONTROLLER_TO_DEVICE_EMERGENCY_ID
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryDevice_nativeGetCToDEmergencyId (JNIEnv *env, jclass class)
{
    return ROLLINGSPIDER_CONTROLLER_TO_DEVICE_EMERGENCY_ID;
}

/**
 * @brief get ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID
 * @return value of ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryDevice_nativeGetDToCNavDataId (JNIEnv *env, jclass class)
{
    return ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID;
}

/**
 * @brief get ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID
 * @return value of ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryDevice_nativeGetDToCEventId (JNIEnv *env, jclass class)
{
    return ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID;
}

/**
 * @brief Create and initialize a new device
 * @param env reference to the java environment
 * @param thizz reference to the object calling this function
 * @return new jni device object
 */
JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryDevice_nativeNew (JNIEnv *env, jobject thizz)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, ARDISCOVERY_JNIDEVICE_TAG, "nativeNew  ...");

    // -- Create the Device --

    // local declarations
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_Device_t *device = NULL;

    jclass exceptionCls = NULL;
    jmethodID exceptionMethodInit = NULL;
    jthrowable exception = NULL;

    ARSAL_PRINT(ARSAL_PRINT_INFO, ARDISCOVERY_JNIDEVICE_TAG, "pre ARDISCOVERY_Device_New  ...");

    // allocate the device
    device = ARDISCOVERY_Device_New (&error);

    if (error != ARDISCOVERY_OK)
    {

        ARSAL_PRINT(ARSAL_PRINT_INFO, ARDISCOVERY_JNIDEVICE_TAG, "throw the exception");

        // throw the exception
        exceptionCls = (*env)->FindClass(env, "com/parrot/arsdk/discovery/ARDiscoveryException");
        exceptionMethodInit = (*env)->GetMethodID(env, exceptionCls, "<init>", "(I)V");
        exception = (*env)->NewObject(env, exceptionCls, exceptionMethodInit, error);
        (*env)->Throw(env, exception);
    }

    return (long) device;
}

/**
 * @brief delete the Device
 * @param env reference to the java environment
 * @param thizz reference to the object calling this function
 * @param jDevice the ARDISCOVERY_Device_t to delete
 */
JNIEXPORT void JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryDevice_nativeDelete (JNIEnv *env, jobject thizz, jlong jDevice)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, ARDISCOVERY_JNIDEVICE_TAG, "nativeDelete  ...");

    // -- Delete the Device --

    // local declarations
    ARDISCOVERY_Device_t *device = (ARDISCOVERY_Device_t*) (intptr_t) jDevice;

    // Delete native device
    ARDISCOVERY_Device_Delete (&device);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryDevice_nativeInitWifi(JNIEnv *env, jobject thizz, jlong jDevice, jint product, jstring name, jstring address, jint port)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, ARDISCOVERY_JNIDEVICE_TAG, "nativeInitWifi  ... product: %d", product);


    // -- Initialize the Discovery Device with a wifi device --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_Device_t *device = (ARDISCOVERY_Device_t*) (intptr_t) jDevice;
    const char *nativeName = (*env)->GetStringUTFChars(env, name, 0);
    const char *nativeAddress = (*env)->GetStringUTFChars(env, address, 0);

    error = ARDISCOVERY_Device_InitWifi (device, product, nativeName, nativeAddress, port);

    /* clean up */
    (*env)->ReleaseStringUTFChars(env, name, nativeName);
    (*env)->ReleaseStringUTFChars(env, address, nativeAddress);


    if (error == ARDISCOVERY_OK)
    {
        // Add callbacks for the connection json part
        error = ARDISCOVERY_Device_WifiAddConnectionCallbacks (device, ARDISCOVERY_JNI_Device_SendJsonCallback, ARDISCOVERY_JNI_Device_ReceiveJsonCallback, NULL);
    }

    return error;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryDevice_nativeInitBLE(JNIEnv *env, jobject thizz, jlong jDevice, jint product, jobject jBLEPart)
{
    // -- Initialize the Discovery Device with a BLE device --

    ARDISCOVERY_Device_t *device = (ARDISCOVERY_Device_t*) (intptr_t) jDevice;

    return  ARDISCOVERY_JNI_Device_InitBLE (env, device, product, jBLEPart);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryDevice_nativeInitUsb(JNIEnv *env, jobject thizz, jlong jDevice, jint product, jlong mux)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, ARDISCOVERY_JNIDEVICE_TAG, "nativeInitUSB  ... product: %d", product);

    // -- Initialize the Discovery Device with an USB device --

    ARDISCOVERY_Device_t *device = (ARDISCOVERY_Device_t*) (intptr_t) jDevice;
	eARDISCOVERY_ERROR error;

    error = ARDISCOVERY_Device_InitUSB (device, product, (struct mux_ctx *)(intptr_t)mux);
    if (error == ARDISCOVERY_OK)
    {
        error = ARDISCOVERY_Device_UsbAddConnectionCallbacks (device, ARDISCOVERY_JNI_Device_SendJsonCallback, ARDISCOVERY_JNI_Device_ReceiveJsonCallback, NULL);
    }

    return error;
}

/*****************************************
 *
 *             private implementation:
 *
 *****************************************/

eARDISCOVERY_ERROR ARDISCOVERY_JNI_Device_SendJsonCallback (json_object *jsonObj, void *customData)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, ARDISCOVERY_JNIDEVICE_TAG, "SendJsonCallback  ...");

    // -- Connection callback to receive the Json --

    // Local declarations
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    json_object *valueJsonObj = NULL;

    // Check parameters
    if (jsonObj == NULL)
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }

    if (error == ARDISCOVERY_OK)
    {
        // add ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY
        valueJsonObj = json_object_new_string ("DEFAULT_SDK_CONTROLLER"); //TODO get from the device controller
        json_object_object_add (jsonObj, ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY, valueJsonObj);

        // add ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY
        valueJsonObj = json_object_new_string ("DEFAULT_SDK_TYPE"); //TODO get from the device controller
        json_object_object_add (jsonObj, ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY, valueJsonObj);

    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_JNI_Device_ReceiveJsonCallback (json_object *jsonObj, void *customData)
{

    ARSAL_PRINT(ARSAL_PRINT_INFO, ARDISCOVERY_JNIDEVICE_TAG, "ReceiveJsonCallback   ...");

    // -- Connection callback to receive the Json --

    // Local declarations
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    if (jsonObj == NULL)
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }

    if (error == ARDISCOVERY_OK)
    {
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_JNI_Device_InitBLE (JNIEnv *env, ARDISCOVERY_Device_t *device, eARDISCOVERY_PRODUCT product, jobject jBLEPart)
{
    // -- Initialize the Discovery Device with a BLE device --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) ||
        (jBLEPart == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    //TODO see to check if the device is already initialized !!!!

    if (error == ARDISCOVERY_OK)
    {
        switch (product)
        {
        case ARDISCOVERY_PRODUCT_MINIDRONE:
        case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_LIGHT:
        case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_BRICK:
        case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_HYDROFOIL:
        case ARDISCOVERY_PRODUCT_MINIDRONE_DELOS3:
        case ARDISCOVERY_PRODUCT_MINIDRONE_WINGX:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Ble_InitRollingSpiderNetworkConfiguration;
            break;

        case ARDISCOVERY_PRODUCT_SKYCONTROLLER:
        case ARDISCOVERY_PRODUCT_ARDRONE:
        case ARDISCOVERY_PRODUCT_JS:
        case ARDISCOVERY_PRODUCT_MAX:
            error = ARDISCOVERY_ERROR_BAD_PARAMETER;
            break;

        default:
            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARDISCOVERY_JNIDEVICE_TAG, "Product:%d not known", product);
            error = ARDISCOVERY_ERROR_BAD_PARAMETER;
            break;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        // Initialize common parameters
        device->productID = product;
        device->networkType = ARDISCOVERY_NETWORK_TYPE_BLE;
        device->newNetworkAL = ARDISCOVERY_JNI_DEVICE_Ble_NewARNetworkAL;
        device->deleteNetworkAL = ARDISCOVERY_JNI_DEVICE_Ble_DeleteARNetworkAL;
        device->getCopyOfSpecificParameters = ARDISCOVERY_JNI_DEVICE_Ble_GetCopyOfSpecificParameters;
        device->deleteSpecificParameters = ARDISCOVERY_JNI_DEVICE_Ble_DeleteSpecificParameters;
    }

    if (error == ARDISCOVERY_OK)
    {
        // Initialize BLE specific parameters
        error = ARDISCOVERY_JNI_DEVICE_Ble_CreateSpecificParameters (env, device, jBLEPart);
    }

    return error;
}
