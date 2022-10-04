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
 * @file ARDISCOVERY_DEVICE_Ble.c
 * @brief Discovery BLE Device contains the informations of a device discovered
 * @date 02/03/2015
 * @author maxime.maitre@parrot.com
 */

#include <stdlib.h>
#include <json-c/json.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetworkAL/ARNETWORKAL_Manager.h>
#include <libARNetworkAL/ARNETWORKAL_Error.h>
#include <libARDiscovery/ARDISCOVERY_Connection.h>
#include <libARDiscovery/ARDISCOVERY_Discovery.h>
#include <libARDiscovery/ARDISCOVERY_Device.h>

#include "ARDISCOVERY_DEVICE_Ble.h"

#define ARDISCOVERY_DEVICE_BLE_TAG "ARDISCOVERY_DEVICE_BLE"

/*************************
 * Private header
 *************************/


/*************************
 * Implementation
 *************************/

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Ble_CreateSpecificParameters (ARDISCOVERY_Device_t *device, ARNETWORKAL_BLEDeviceManager_t bleDeviceManager, ARNETWORKAL_BLEDevice_t bleDevice)
{
    // Initialize BLE specific parameters
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_BLE_t *specificBLEParam = NULL;

    // Check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        specificBLEParam = malloc(sizeof(ARDISCOVERY_DEVICE_BLE_t));
        if (specificBLEParam != NULL)
        {
            device->specificParameters = specificBLEParam;
            specificBLEParam->deviceManager = bleDeviceManager;
            specificBLEParam->device = bleDevice;
        }
        else
        {
            error = ARDISCOVERY_ERROR_ALLOC;
        }
    }

    // if an error occurred and it is not a bad parameters error
    if (error != ARDISCOVERY_OK)
    {
        // try to delete SpecificParameters
        ARDISCOVERY_DEVICE_Ble_DeleteSpecificParameters (device);
    }
    // No else: skipped no error

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Ble_DeleteSpecificParameters (ARDISCOVERY_Device_t *device)
{
    // -- Delete SpecificParameters allocated by the wifi initialization --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        // free specific parameter
        if (device->specificParameters != NULL)
        {
            free (device->specificParameters);
            device->specificParameters = NULL;
        }
    }

    return error;
}

void *ARDISCOVERY_DEVICE_Ble_GetCopyOfSpecificParameters (ARDISCOVERY_Device_t *device, eARDISCOVERY_ERROR *error)
{
    // -- Copy BLE specificParameters --

    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_BLE_t *specificBLEParamToCopy = NULL;
    ARDISCOVERY_DEVICE_BLE_t *specificBLEParam = NULL;

    // check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE))
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (localError == ARDISCOVERY_OK)
    {
        // cast device->specificBLEParam
        specificBLEParamToCopy = (ARDISCOVERY_DEVICE_BLE_t *)device->specificParameters;

        if (specificBLEParamToCopy != NULL)
        {
            // Copy wifi specific parameters
            specificBLEParam = malloc(sizeof(ARDISCOVERY_DEVICE_BLE_t));
            if (specificBLEParam != NULL)
            {
                specificBLEParam->deviceManager = specificBLEParamToCopy->deviceManager;
                specificBLEParam->device = specificBLEParamToCopy->device;
            }
            else
            {
                localError = ARDISCOVERY_ERROR_ALLOC;
            }

        }
        // NO Else ; No Specific Wifi Parameters To Copy.
    }
    // No else: skipped by error

    // delete the SpecificParameters if an error occurred
    if (localError != ARDISCOVERY_OK)
    {
        ARDISCOVERY_DEVICE_Ble_DeleteSpecificParameters (device);
    }
    // No else: skipped no error

    // return the error
    if (error != NULL)
    {
        *error = localError;
    }
    // No else: error is not returned

    return specificBLEParam;
}

ARNETWORKAL_Manager_t *ARDISCOVERY_DEVICE_Ble_NewARNetworkAL (ARDISCOVERY_Device_t *device, eARDISCOVERY_ERROR *error, eARNETWORKAL_ERROR *errorAL)
{
    //-- Create a new networlAL adapted to the device. --

    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    eARNETWORKAL_ERROR localErrorAL = ARNETWORKAL_OK;
    ARNETWORKAL_Manager_t *networkAL = NULL;
    ARDISCOVERY_DEVICE_BLE_t *specificBLEParam = NULL;

    // check parameters
    if ((device == NULL) ||
        (device->specificParameters == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE))
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (localError == ARDISCOVERY_OK)
    {
        // Cast of device->specificParameters
        specificBLEParam = (ARDISCOVERY_DEVICE_BLE_t *) device->specificParameters;

        // Create the ARNetworkALManager
        networkAL = ARNETWORKAL_Manager_New (&localErrorAL);
    }

    if ((localError == ARDISCOVERY_OK) && (localErrorAL == ARNETWORKAL_OK))
    {
        // init bleNotificationIDs
        int bleNotificationIDs[] = {
            ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID,
            ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID,
            (ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID + (ARNETWORKAL_MANAGER_BLE_ID_MAX / 2)),
            (ROLLINGSPIDER_CONTROLLER_TO_DEVICE_EMERGENCY_ID + (ARNETWORKAL_MANAGER_BLE_ID_MAX / 2)),
        };
        uint32_t numberOfNotificationID = sizeof(bleNotificationIDs) / sizeof(int);

        // Initialize the ARNetworkALManager
        localErrorAL = ARNETWORKAL_Manager_InitBLENetwork(networkAL, specificBLEParam->deviceManager, specificBLEParam->device, 1, bleNotificationIDs, numberOfNotificationID);
    }

    // set localError to ARDISCOVERY_ERROR is an error AL is occured
    if ((localError == ARDISCOVERY_OK) && (localErrorAL != ARNETWORKAL_OK))
    {
        localError = ARDISCOVERY_ERROR;
    }

    // return localErrorAL
    if (errorAL != NULL)
    {
        *errorAL = localErrorAL;
    }

    // return localError
    if (error != NULL)
    {
        *error = localError;
    }

    // delete networkAL if an error occured
    if ((localError != ARDISCOVERY_OK) && (networkAL != NULL))
    {
        ARDISCOVERY_DEVICE_Ble_DeleteARNetworkAL (device, &networkAL);
    }

    return networkAL;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Ble_DeleteARNetworkAL (ARDISCOVERY_Device_t *device, ARNETWORKAL_Manager_t **networkAL)
{
    // -- Delete a networlAL create by ARDISCOVERY_DEVICE_Ble_NewARNetworkAL --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        if (networkAL != NULL)
        {
            if ((*networkAL) != NULL)
            {
                ARNETWORKAL_Manager_Unlock((*networkAL));

                ARNETWORKAL_Manager_CloseBLENetwork((*networkAL));
                ARNETWORKAL_Manager_Delete(networkAL);
            }
        }
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Ble_InitRollingSpiderNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a Rolling Spider. --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) ||
        (ARDISCOVERY_getProductFamily(device->productID) != ARDISCOVERY_PRODUCT_FAMILY_MINIDRONE) ||
        (networkConfiguration == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    memset(networkConfiguration, 0x0, sizeof(*networkConfiguration));

    static ARNETWORK_IOBufferParam_t c2dParams[] = {
        /* Non-acknowledged commands. */
        {
            .ID = ROLLINGSPIDER_CONTROLLER_TO_DEVICE_NONACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 1,
            .dataCopyMaxSize = ARNETWORK_IOBUFFERPARAM_DATACOPYMAXSIZE_USE_MAX,
            .isOverwriting = 1,
        },
        /* Acknowledged commands. */
        {
            .ID = ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = 500,
            .numberOfRetry = 3,
            .numberOfCell = 20,
            .dataCopyMaxSize = ARNETWORK_IOBUFFERPARAM_DATACOPYMAXSIZE_USE_MAX,
            .isOverwriting = 0,
        },
        /* Emergency commands. */
        {
            .ID = ROLLINGSPIDER_CONTROLLER_TO_DEVICE_EMERGENCY_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 1,
            .ackTimeoutMs = 100,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 1,
            .dataCopyMaxSize = ARNETWORK_IOBUFFERPARAM_DATACOPYMAXSIZE_USE_MAX,
            .isOverwriting = 0,
        },
    };
    size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static ARNETWORK_IOBufferParam_t d2cParams[] = {
        {
            .ID = ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 20,
            .dataCopyMaxSize = ARNETWORK_IOBUFFERPARAM_DATACOPYMAXSIZE_USE_MAX,
            .isOverwriting = 0,
        },
        {
            .ID = ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = 500,
            .numberOfRetry = 3,
            .numberOfCell = 20,
            .dataCopyMaxSize = ARNETWORK_IOBUFFERPARAM_DATACOPYMAXSIZE_USE_MAX,
            .isOverwriting = 0,
        },
    };
    size_t numD2cParams = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static int commandBufferIds[] = {
        ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID,
        ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID,
    };

    size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);

    if (error == ARDISCOVERY_OK)
    {
        networkConfiguration->controllerLoopIntervalMs = 50;

        networkConfiguration->controllerToDeviceNotAckId = ROLLINGSPIDER_CONTROLLER_TO_DEVICE_NONACK_ID;
        networkConfiguration->controllerToDeviceAckId = ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID;
        networkConfiguration->controllerToDeviceHightPriority = ROLLINGSPIDER_CONTROLLER_TO_DEVICE_EMERGENCY_ID;
        networkConfiguration->controllerToDeviceARStreamAck = -1;
        networkConfiguration->controllerToDeviceARStreamAudioAck = -1;
        networkConfiguration->controllerToDeviceARStreamAudioData = -1;
        networkConfiguration->deviceToControllerNotAckId = ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerAckId = ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        //int deviceToControllerHightPriority = -1;
        networkConfiguration->deviceToControllerARStreamData = -1;
        networkConfiguration->deviceToControllerARStreamAudioData = -1;
        networkConfiguration->deviceToControllerARStreamAudioAck = -1;

        networkConfiguration->hasVideo = 0;
        networkConfiguration->streamType = ARDISCOVERY_STREAM_STARTSTOP_NONE;

        networkConfiguration->controllerToDeviceParams = c2dParams;
        networkConfiguration->numberOfControllerToDeviceParam = numC2dParams;

        networkConfiguration->deviceToControllerParams = d2cParams;
        networkConfiguration->numberOfDeviceToControllerParam = numD2cParams;

        networkConfiguration->pingDelayMs = -1;

        networkConfiguration->numberOfDeviceToControllerCommandsBufferIds = numOfCommandBufferIds;
        networkConfiguration->deviceToControllerCommandsBufferIds = commandBufferIds;
    }

    return error;
}


eARDISCOVERY_ERROR ARDISCOVERY_Device_Ble_GetManager(ARDISCOVERY_Device_t *device, ARNETWORKAL_BLEDeviceManager_t **manager)
{
    ARDISCOVERY_DEVICE_BLE_t *params;

    if (!device ||
        device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE ||
        !manager)
        return ARDISCOVERY_ERROR_BAD_PARAMETER;

    params = (ARDISCOVERY_DEVICE_BLE_t *)device->specificParameters;

    *manager = params->deviceManager;
    return ARDISCOVERY_OK;
}

eARDISCOVERY_ERROR ARDISCOVERY_Device_Ble_GetDevice(ARDISCOVERY_Device_t *device, ARNETWORKAL_BLEDevice_t **bleDevice)
{
    ARDISCOVERY_DEVICE_BLE_t *params;

    if (!device ||
        device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE ||
        !bleDevice)
        return ARDISCOVERY_ERROR_BAD_PARAMETER;

    params = (ARDISCOVERY_DEVICE_BLE_t *)device->specificParameters;

    *bleDevice = params->device;
    return ARDISCOVERY_OK;
}

/*************************
 * local Implementation
 *************************/
