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
 * @file ARDISCOVERY_Device.c
 * @brief Discovery Device contains the informations of a device discovered
 * @date 02/03/2015
 * @author maxime.maitre@parrot.com
 */

#include <stdlib.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetworkAL/ARNETWORKAL_Manager.h>
#include <libARNetworkAL/ARNETWORKAL_Error.h>
#include <libARDiscovery/ARDISCOVERY_Connection.h>
#include <libARDiscovery/ARDISCOVERY_Device.h>

struct mux_ctx;

#include "Wifi/ARDISCOVERY_DEVICE_Wifi.h"
#include "BLE/ARDISCOVERY_DEVICE_Ble.h"
#include "Usb/ARDISCOVERY_DEVICE_Usb.h"

#include "ARDISCOVERY_Device.h"


#define ARDISCOVERY_DEVICE_TAG "ARDISCOVERY_Device"

/*************************
 * Private header
 *************************/



/*************************
 * Implementation
 *************************/

ARDISCOVERY_Device_t *ARDISCOVERY_Device_New (eARDISCOVERY_ERROR *error)
{
    // -- Create a new Discovery Device --

    //local declarations
    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    ARDISCOVERY_Device_t *device = NULL;

    // Allocate the device
    device = malloc (sizeof(ARDISCOVERY_Device_t));
    if (device == NULL)
    {
        localError = ARDISCOVERY_ERROR_ALLOC;
    }

    if (localError == ARDISCOVERY_OK)
    {
        // Initialize the Device
        device->name = NULL;
        device->productID = ARDISCOVERY_PRODUCT_MAX;
        device->networkType = ARDISCOVERY_NETWORK_TYPE_UNKNOWN;
        device->newNetworkAL = NULL;
        device->deleteNetworkAL = NULL;
        device->initNetworkConfiguration = NULL;
        device->specificParameters = NULL;
        device->getCopyOfSpecificParameters = NULL;
        device->deleteSpecificParameters = NULL;
    }
    // No else: skipped by an error

    // delete the Device Controller if an error occurred
    if (localError != ARDISCOVERY_OK)
    {
        ARDISCOVERY_Device_Delete (&device);
    }
    // No else: skipped no error

    // return the error
    if (error != NULL)
    {
        *error = localError;
    }
    // No else: error is not returned

    return device;
}

ARDISCOVERY_Device_t *ARDISCOVERY_Device_NewByCopy (ARDISCOVERY_Device_t *deviceToCopy, eARDISCOVERY_ERROR *error)
{
    // -- Copy a Discovery Device --

    //local declarations
    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    ARDISCOVERY_Device_t *device =  NULL;

    // check parameters
    if (deviceToCopy == NULL)
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (localError == ARDISCOVERY_OK)
    {
        // Create the new device
        device = ARDISCOVERY_Device_New (&localError);
    }
    // No else: skipped by error

    if (localError == ARDISCOVERY_OK)
    {
        // Copy common parameters
        device->productID = deviceToCopy->productID;
        device->networkType = deviceToCopy->networkType;
        device->newNetworkAL = deviceToCopy->newNetworkAL;
        device->deleteNetworkAL = deviceToCopy->deleteNetworkAL;
        device->initNetworkConfiguration = deviceToCopy->initNetworkConfiguration;
        device->getCopyOfSpecificParameters = deviceToCopy->getCopyOfSpecificParameters;
        device->deleteSpecificParameters = deviceToCopy->deleteSpecificParameters;

        if (deviceToCopy->name != NULL)
        {
            device->name = strdup(deviceToCopy->name);
            if (device->name == NULL)
            {
                localError = ARDISCOVERY_ERROR_ALLOC;
            }
        }
    }
    // No else: skipped by error

    if (localError == ARDISCOVERY_OK)
    {
        // Copy specific parameters
        if (deviceToCopy->getCopyOfSpecificParameters != NULL)
        {
            device->specificParameters = device->getCopyOfSpecificParameters (deviceToCopy, &localError);
        }
    }
    // No else: skipped by error

    // delete the Device if an error occurred
    if (localError != ARDISCOVERY_OK)
    {
        ARDISCOVERY_Device_Delete (&device);
    }
    // No else: skipped no error

    // return the error
    if (error != NULL)
    {
        *error = localError;
    }
    // No else: error is not returned

    return device;
}

void ARDISCOVERY_Device_Delete (ARDISCOVERY_Device_t **device)
{
    // -- Delete the Discovery Device --

    if (device != NULL)
    {
        if ((*device) != NULL)
        {
            // cleanup common parameters
            if ((*device)->name != NULL)
            {
                free ((*device)->name);
                (*device)->name = NULL;
            }

            // cleanup specific parameters
            if ((*device)->deleteSpecificParameters != NULL)
            {
                (*device)->deleteSpecificParameters (*device);
            }

            free (*device);
            (*device) = NULL;
        }
    }
}

ARNETWORKAL_Manager_t *ARDISCOVERY_Device_NewARNetworkAL (ARDISCOVERY_Device_t *discoveryDevice, eARDISCOVERY_ERROR *error, eARNETWORKAL_ERROR *errorAL)
{
    // -- Create a new ARNetworkAL --

    //local declarations
    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    eARNETWORKAL_ERROR localErrorAL = ARNETWORKAL_OK;
    ARNETWORKAL_Manager_t *networkALManager = NULL;

    // check parameters
    if (discoveryDevice == NULL)
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets localError to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (localError == ARDISCOVERY_OK)
    {
        if ((discoveryDevice->newNetworkAL != NULL) && (discoveryDevice->deleteNetworkAL != NULL))
        {
            networkALManager = discoveryDevice->newNetworkAL (discoveryDevice, &localError, &localErrorAL);
        }
        else
        {
            localError = ARDISCOVERY_ERROR_DEVICE_OPERATION_NOT_SUPPORTED;
        }
    }

    // delete the NetworkManagerAL if an error occurred
    if ((localError != ARDISCOVERY_OK) || (localErrorAL != ARNETWORKAL_OK))
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARDISCOVERY_DEVICE_TAG, "error: %s", ARDISCOVERY_Error_ToString (localError));

        if (discoveryDevice && discoveryDevice->deleteNetworkAL)
            discoveryDevice->deleteNetworkAL (discoveryDevice, &networkALManager);
    }
    // No else: skipped by an error

    // return the error
    if (error != NULL)
    {
        *error = localError;
    }
    // No else: error is not returned

    if (errorAL != NULL)
    {
        *errorAL = localErrorAL;
    }
    // No else: error is not returned

    return networkALManager;
}

eARDISCOVERY_ERROR ARDISCOVERY_Device_DeleteARNetworkAL (ARDISCOVERY_Device_t *discoveryDevice, ARNETWORKAL_Manager_t **networkALManager)
{
    // -- Delete a ARNetworkAL  --

    //local declarations
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if (discoveryDevice == NULL)
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets localError to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        if (discoveryDevice->deleteNetworkAL != NULL)
        {
            error = discoveryDevice->deleteNetworkAL (discoveryDevice, networkALManager);
        }
        else
        {
            error = ARDISCOVERY_ERROR_DEVICE_OPERATION_NOT_SUPPORTED;
        }
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_Device_InitNetworkConfiguration (ARDISCOVERY_Device_t *discoveryDevice, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initialize the NetworkConfiguration to use with the device  --

    // Local declarations
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // Check parameters
    if ((discoveryDevice == NULL) || (networkConfiguration == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: The checking parameters sets localError to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        if (discoveryDevice->initNetworkConfiguration != NULL)
        {
            discoveryDevice->initNetworkConfiguration (discoveryDevice, networkConfiguration);
        }
        else
        {
            error = ARDISCOVERY_ERROR_DEVICE_OPERATION_NOT_SUPPORTED;
        }
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_Device_DestroyNetworkConfiguration (ARDISCOVERY_Device_t *discoveryDevice, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    if (!discoveryDevice || ! networkConfiguration)
        return ARDISCOVERY_ERROR_BAD_PARAMETER;

    free(networkConfiguration->rtspAddress);
    networkConfiguration->rtspAddress = NULL;

    return ARDISCOVERY_OK;
}

/***********************
 * -- Wifi part --
 ***********************/

eARDISCOVERY_ERROR ARDISCOVERY_Device_InitWifi (ARDISCOVERY_Device_t *device, eARDISCOVERY_PRODUCT product, const char *name, const char *address, int port)
{
    // -- Initialize the Discovery Device with a wifi device --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // Check parameters
    if ((device == NULL) ||
        (name == NULL) ||
        (address == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    //TODO see to check if the device is already initialized !!!!

    if (error == ARDISCOVERY_OK)
    {
        switch (product)
        {
        case ARDISCOVERY_PRODUCT_ARDRONE:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitBebopNetworkConfiguration;
            break;
        case ARDISCOVERY_PRODUCT_BEBOP_2:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitBebop2NetworkConfiguration;
            break;
        case ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_4:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitUnknownproduct_4NetworkConfiguration;
            break;
        case ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_5:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitUnknownproduct_5NetworkConfiguration;
            break;
        case ARDISCOVERY_PRODUCT_CHIMERA:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitChimeraNetworkConfiguration;
            break;
        case ARDISCOVERY_PRODUCT_JS:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitJumpingSumoNetworkConfiguration;
            break;
        case ARDISCOVERY_PRODUCT_JS_EVO_LIGHT:
        case ARDISCOVERY_PRODUCT_JS_EVO_RACE:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitJumpingSumoEvoNetworkConfiguration;
            break;
        case ARDISCOVERY_PRODUCT_POWER_UP:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitPowerUpNetworkConfiguration;
            break;
        case ARDISCOVERY_PRODUCT_EVINRUDE:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitEvinrudeNetworkConfiguration;
            break;
        case ARDISCOVERY_PRODUCT_SKYCONTROLLER:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitSkyControllerNetworkConfiguration;
            break;
        case ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG:
        case ARDISCOVERY_PRODUCT_SKYCONTROLLER_2:
        case ARDISCOVERY_PRODUCT_SKYCONTROLLER_2P:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitSkyController2NetworkConfiguration;
            break;
        case ARDISCOVERY_PRODUCT_MINIDRONE_DELOS3:
            device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Wifi_InitDelos3NetworkConfiguration;
            break;

        case ARDISCOVERY_PRODUCT_MINIDRONE:
        case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_LIGHT:
        case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_BRICK:
        case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_HYDROFOIL:
        case ARDISCOVERY_PRODUCT_MINIDRONE_WINGX:
        case ARDISCOVERY_PRODUCT_TINOS:
        case ARDISCOVERY_PRODUCT_SEQUOIA:
        case ARDISCOVERY_PRODUCT_MAX:
            error = ARDISCOVERY_ERROR_BAD_PARAMETER;
            break;

        default:
            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARDISCOVERY_DEVICE_TAG, "Product:%d not known", product);
            error = ARDISCOVERY_ERROR_BAD_PARAMETER;
            break;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        // Initialize common parameters
        device->productID = product;
        device->networkType = ARDISCOVERY_NETWORK_TYPE_NET;
        device->newNetworkAL = ARDISCOVERY_DEVICE_Wifi_NewARNetworkAL;
        device->deleteNetworkAL = ARDISCOVERY_DEVICE_Wifi_DeleteARNetworkAL;
        device->getCopyOfSpecificParameters = ARDISCOVERY_DEVICE_Wifi_GetCopyOfSpecificParameters;
        device->deleteSpecificParameters = ARDISCOVERY_DEVICE_Wifi_DeleteSpecificParameters;

        device->name = strdup(name);
        if (device->name == NULL)
        {
            error = ARDISCOVERY_ERROR_ALLOC;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        // Initialize wifi specific parameters
        error = ARDISCOVERY_DEVICE_Wifi_CreateSpecificParameters (device, name, address, port);
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_Device_WifiAddConnectionCallbacks (ARDISCOVERY_Device_t *device, ARDISCOVERY_Device_ConnectionJsonCallback_t sendJsonCallback, ARDISCOVERY_Device_ConnectionJsonCallback_t receiveJsonCallback, void *customData)
{
    // -- Wifi Add Connection Callbacks --

    return ARDISCOVERY_DEVICE_Wifi_AddConnectionCallbacks (device, sendJsonCallback, receiveJsonCallback, customData);
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_WifiGetIpAddress (ARDISCOVERY_Device_t *device, char *ipAddress, int length)
{
    // -- Get the IP address of the device  --

    return ARDISCOVERY_DEVICE_Wifi_GetIpAddress (device, ipAddress, length);
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_WifiSetQoSLevel (ARDISCOVERY_Device_t *device, int level)
{
    return ARDISCOVERY_DEVICE_Wifi_SetQoSLevel(device, level);
}

eARDISCOVERY_ERROR ARDISCOVERY_Device_WifiSetDeviceToControllerPort (ARDISCOVERY_Device_t *device, int d2c_port)
{
    return ARDISCOVERY_DEVICE_Wifi_SetDeviceToControllerPort (device, d2c_port);
}


/***********************
 * -- BLE part --
 ***********************/

eARDISCOVERY_ERROR ARDISCOVERY_Device_InitBLE (ARDISCOVERY_Device_t *device, eARDISCOVERY_PRODUCT product, ARNETWORKAL_BLEDeviceManager_t bleDeviceManager, ARNETWORKAL_BLEDevice_t bleDevice)
{
    // -- Initialize the Discovery Device with a wifi device --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) ||
        (bleDeviceManager == NULL) ||
        (bleDevice == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    //TODO see to check fi the device is already initialized !!!!

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
        case ARDISCOVERY_PRODUCT_SKYCONTROLLER_2:
        case ARDISCOVERY_PRODUCT_SKYCONTROLLER_2P:
        case ARDISCOVERY_PRODUCT_ARDRONE:
        case ARDISCOVERY_PRODUCT_BEBOP_2:
        case ARDISCOVERY_PRODUCT_JS:
        case ARDISCOVERY_PRODUCT_JS_EVO_LIGHT:
        case ARDISCOVERY_PRODUCT_JS_EVO_RACE:
        case ARDISCOVERY_PRODUCT_POWER_UP:
        case ARDISCOVERY_PRODUCT_EVINRUDE:
        case ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_4:
        case ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_5:
        case ARDISCOVERY_PRODUCT_CHIMERA:
        case ARDISCOVERY_PRODUCT_TINOS:
        case ARDISCOVERY_PRODUCT_SEQUOIA:
        case ARDISCOVERY_PRODUCT_MAX:
            error = ARDISCOVERY_ERROR_BAD_PARAMETER;
            break;

        default:
            ARSAL_PRINT (ARSAL_PRINT_ERROR, ARDISCOVERY_DEVICE_TAG, "Product:%d not known", product);
            error = ARDISCOVERY_ERROR_BAD_PARAMETER;
            break;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        // Initialize common parameters
        device->productID = product;
        device->networkType = ARDISCOVERY_NETWORK_TYPE_BLE;
        device->newNetworkAL = ARDISCOVERY_DEVICE_Ble_NewARNetworkAL;
        device->deleteNetworkAL = ARDISCOVERY_DEVICE_Ble_DeleteARNetworkAL;
        device->getCopyOfSpecificParameters = ARDISCOVERY_DEVICE_Ble_GetCopyOfSpecificParameters;
        device->deleteSpecificParameters = ARDISCOVERY_DEVICE_Ble_DeleteSpecificParameters;
    }

    if (error == ARDISCOVERY_OK)
    {
        // Initialize BLE specific parameters
        error = ARDISCOVERY_DEVICE_Ble_CreateSpecificParameters (device, bleDeviceManager, bleDevice);
    }

    return error;
}


eARDISCOVERY_ERROR ARDISCOVERY_Device_BLEGetManager(ARDISCOVERY_Device_t *device, ARNETWORKAL_BLEDeviceManager_t **manager)
{
    return ARDISCOVERY_Device_Ble_GetManager(device, manager);
}

eARDISCOVERY_ERROR ARDISCOVERY_Device_BLEGetDevice(ARDISCOVERY_Device_t *device, ARNETWORKAL_BLEDevice_t **bleDevice)
{
    return ARDISCOVERY_Device_Ble_GetDevice(device, bleDevice);
}

/***********************
 * -- USB part --
 ***********************/

eARDISCOVERY_ERROR ARDISCOVERY_Device_InitUSB (ARDISCOVERY_Device_t *device, eARDISCOVERY_PRODUCT product, struct mux_ctx *mux)
{
    // -- Initialize the Discovery Device with an usb device --

    // Check parameters
    if ((device == NULL) ||
        (mux == NULL))
        return ARDISCOVERY_ERROR_BAD_PARAMETER;

    switch (product) {
    case ARDISCOVERY_PRODUCT_SKYCONTROLLER_2:
    case ARDISCOVERY_PRODUCT_SKYCONTROLLER_2P:
    case ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG:
        device->initNetworkConfiguration = ARDISCOVERY_DEVICE_Usb_InitSkyController2NetworkConfiguration;
        break;
    case ARDISCOVERY_PRODUCT_ARDRONE:
    case ARDISCOVERY_PRODUCT_BEBOP_2:
    case ARDISCOVERY_PRODUCT_JS:
    case ARDISCOVERY_PRODUCT_JS_EVO_LIGHT:
    case ARDISCOVERY_PRODUCT_JS_EVO_RACE:
    case ARDISCOVERY_PRODUCT_POWER_UP:
    case ARDISCOVERY_PRODUCT_EVINRUDE:
    case ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_4:
    case ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_5:
    case ARDISCOVERY_PRODUCT_CHIMERA:
    case ARDISCOVERY_PRODUCT_SKYCONTROLLER:
    case ARDISCOVERY_PRODUCT_MINIDRONE:
    case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_LIGHT:
    case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_BRICK:
    case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_HYDROFOIL:
    case ARDISCOVERY_PRODUCT_MINIDRONE_DELOS3:
    case ARDISCOVERY_PRODUCT_MINIDRONE_WINGX:
    case ARDISCOVERY_PRODUCT_TINOS:
    case ARDISCOVERY_PRODUCT_SEQUOIA:
    case ARDISCOVERY_PRODUCT_MAX:
        return ARDISCOVERY_ERROR_BAD_PARAMETER;
        break;

    default:
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARDISCOVERY_DEVICE_TAG, "Product:%d not known", product);
        return ARDISCOVERY_ERROR_BAD_PARAMETER;
        break;
    }

    // Initialize common parameters
    device->productID = product;
    device->networkType = ARDISCOVERY_NETWORK_TYPE_USBMUX;
    device->newNetworkAL = ARDISCOVERY_DEVICE_Usb_NewARNetworkAL;
    device->deleteNetworkAL = ARDISCOVERY_DEVICE_Usb_DeleteARNetworkAL;
    device->getCopyOfSpecificParameters = ARDISCOVERY_DEVICE_Usb_GetCopyOfSpecificParameters;
    device->deleteSpecificParameters = ARDISCOVERY_DEVICE_Usb_DeleteSpecificParameters;

    return ARDISCOVERY_DEVICE_Usb_CreateSpecificParameters (device, mux);
}


eARDISCOVERY_ERROR ARDISCOVERY_Device_UsbAddConnectionCallbacks (ARDISCOVERY_Device_t *device, ARDISCOVERY_Device_ConnectionJsonCallback_t sendJsonCallback, ARDISCOVERY_Device_ConnectionJsonCallback_t receiveJsonCallback, void *customData)
{
    // -- USB Add Connection Callbacks --

    return ARDISCOVERY_DEVICE_Usb_AddConnectionCallbacks (device, sendJsonCallback, receiveJsonCallback, customData);
}

eARDISCOVERY_ERROR ARDISCOVERY_Device_UsbGetMux(ARDISCOVERY_Device_t *device, struct mux_ctx **mux)
{
    return ARDISCOVERY_DEVICE_Usb_GetMux(device, mux);
}

/*************************
 * local Implementation
 *************************/
