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
 * @file ARDISCOVERY_DEVICE_Wifi.c
 * @brief Discovery wifi Device contains the informations of a device discovered
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

#include "ARDISCOVERY_DEVICE_Wifi.h"

#define ARDISCOVERY_DEVICE_WIFI_TAG "ARDISCOVERY_DEVICE_WIFI"

/*************************
 * Private header
 *************************/

#define DEVICE_TO_CONTROLLER_PORT 43210

// Bebop
#define BEBOP_CONTROLLER_TO_DEVICE_NONACK_ID 10
#define BEBOP_CONTROLLER_TO_DEVICE_ACK_ID 11
#define BEBOP_CONTROLLER_TO_DEVICE_EMERGENCY_ID 12
#define BEBOP_DEVICE_TO_CONTROLLER_NAVDATA_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 1)
#define BEBOP_DEVICE_TO_CONTROLLER_EVENT_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 2)

// Jumping Sumo
#define JUMPINGSUMO_CONTROLLER_TO_DEVICE_NONACK_ID 10
#define JUMPINGSUMO_CONTROLLER_TO_DEVICE_ACK_ID 11
#define JUMPINGSUMO_CONTROLLER_TO_DEVICE_VIDEO_ACK_ID 13
#define JUMPINGSUMO_CONTROLLER_TO_DEVICE_AUDIO_ACK_ID 14
#define JUMPINGSUMO_CONTROLLER_TO_DEVICE_AUDIO_DATA_ID 15
#define JUMPINGSUMO_DEVICE_TO_CONTROLLER_NAVDATA_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 1)
#define JUMPINGSUMO_DEVICE_TO_CONTROLLER_EVENT_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 2)
#define JUMPINGSUMO_DEVICE_TO_CONTROLLER_VIDEO_DATA_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 3)
#define JUMPINGSUMO_DEVICE_TO_CONTROLLER_AUDIO_DATA_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 4)
#define JUMPINGSUMO_DEVICE_TO_CONTROLLER_AUDIO_ACK_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 5)

// Power Up
#define POWERUP_CONTROLLER_TO_DEVICE_NONACK_ID 10
#define POWERUP_CONTROLLER_TO_DEVICE_ACK_ID 11
#define POWERUP_CONTROLLER_TO_DEVICE_VIDEO_ACK_ID 13
#define POWERUP_CONTROLLER_TO_DEVICE_AUDIO_ACK_ID 14
#define POWERUP_DEVICE_TO_CONTROLLER_NAVDATA_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 1)
#define POWERUP_DEVICE_TO_CONTROLLER_EVENT_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 2)
#define POWERUP_DEVICE_TO_CONTROLLER_VIDEO_DATA_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 3)
#define POWERUP_DEVICE_TO_CONTROLLER_AUDIO_DATA_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 4)

// RollingSpider buffer IDs
#define ROLLINGSPIDER_CONTROLLER_TO_DEVICE_NONACK_ID 10
#define ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID 11
#define ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 1)
#define ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 2)

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_DiscoveryConnect (ARDISCOVERY_Device_t *device);

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData);

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData);

/*************************
 * Implementation
 *************************/

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_CreateSpecificParameters (ARDISCOVERY_Device_t *device, const char *name, const char *address, int port)
{
    // Initialize wifi specific parameters
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParam = NULL;

    // Check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_NET))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        specificWifiParam = malloc(sizeof(ARDISCOVERY_DEVICE_WIFI_t));
        if (specificWifiParam != NULL)
        {
            device->specificParameters = specificWifiParam;
            specificWifiParam->address = NULL;
            specificWifiParam->dicoveryPort = port;
            specificWifiParam->sendJsonCallback = NULL;
            specificWifiParam->receiveJsonCallback = NULL;
            specificWifiParam->jsonCallbacksCustomData = NULL;

            // Parameters sended by discovery Json :
            specificWifiParam->deviceToControllerPort = DEVICE_TO_CONTROLLER_PORT;

            // Parameters received by discovery Json :
            specificWifiParam->controllerToDevicePort = -1;
            specificWifiParam->requested_qos_level = 1; /* request QoS by default */
            specificWifiParam->qos_level = 0;
            specificWifiParam->connectionStatus = ARDISCOVERY_OK;
        }
        else
        {
            error = ARDISCOVERY_ERROR_ALLOC;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        specificWifiParam->address = strdup(address);
        if (specificWifiParam->address == NULL)
        {
            error = ARDISCOVERY_ERROR_ALLOC;
        }
    }

    // if an error occurred and it is not a bad parameters error
    if (error != ARDISCOVERY_OK)
    {
        // try to delete SpecificParameters
        ARDISCOVERY_DEVICE_Wifi_DeleteSpecificParameters (device);
    }
    // No else: skipped no error

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_SetDeviceToControllerPort (ARDISCOVERY_Device_t *device, int d2c_port)
{
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParam = NULL;

    // check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_NET))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        if (device->specificParameters != NULL)
        {
            specificWifiParam = (ARDISCOVERY_DEVICE_WIFI_t *)device->specificParameters;
            specificWifiParam->deviceToControllerPort = d2c_port;
        } else {
            error = ARDISCOVERY_ERROR_BAD_PARAMETER;
        }
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_DeleteSpecificParameters (ARDISCOVERY_Device_t *device)
{
    // -- Delete SpecificParameters allocated by the wifi initialization --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParam = NULL;

    // check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_NET))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        // free specific parameter
        if (device->specificParameters != NULL)
        {
            specificWifiParam = (ARDISCOVERY_DEVICE_WIFI_t *)device->specificParameters;
            if (specificWifiParam->address != NULL)
            {
                free (specificWifiParam->address);
                specificWifiParam->address = NULL;
            }

            free (device->specificParameters);
            device->specificParameters = NULL;
            specificWifiParam = NULL;
        }
    }

    return error;
}

void *ARDISCOVERY_DEVICE_Wifi_GetCopyOfSpecificParameters (ARDISCOVERY_Device_t *device, eARDISCOVERY_ERROR *error)
{
    // -- Copy the specificParameters --

    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParamToCopy = NULL;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParam = NULL;

    // check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_NET))
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (localError == ARDISCOVERY_OK)
    {
        // cast device->specificWifiParam
        specificWifiParamToCopy = (ARDISCOVERY_DEVICE_WIFI_t *)device->specificParameters;

        if (specificWifiParamToCopy != NULL)
        {
            // Copy wifi specific parameters
            specificWifiParam = malloc(sizeof(ARDISCOVERY_DEVICE_WIFI_t));
            if (specificWifiParam != NULL)
            {
                specificWifiParam->address = NULL;
                specificWifiParam->dicoveryPort = specificWifiParamToCopy->dicoveryPort;

                specificWifiParam->sendJsonCallback = specificWifiParamToCopy->sendJsonCallback;
                specificWifiParam->receiveJsonCallback = specificWifiParamToCopy->receiveJsonCallback;
                specificWifiParam->jsonCallbacksCustomData = specificWifiParamToCopy->jsonCallbacksCustomData;

                // Parameters sended by discovery Json :
                specificWifiParam->deviceToControllerPort = specificWifiParamToCopy->deviceToControllerPort;

                // Parameters received by discovery Json :
                specificWifiParam->controllerToDevicePort = specificWifiParamToCopy->controllerToDevicePort;
                specificWifiParam->requested_qos_level = specificWifiParamToCopy->requested_qos_level;
                specificWifiParam->qos_level = specificWifiParamToCopy->qos_level;
                specificWifiParam->connectionStatus = specificWifiParamToCopy->connectionStatus;
            }
            else
            {
                localError = ARDISCOVERY_ERROR_ALLOC;
            }

            // Copy address
            if ((localError == ARDISCOVERY_OK) && (specificWifiParamToCopy->address != NULL))
            {
                specificWifiParam->address = strdup (specificWifiParamToCopy->address);
                if (specificWifiParam->address == NULL)
                {
                    localError = ARDISCOVERY_ERROR_ALLOC;
                }
            }
            // No else: skipped by error or no address To Copy.
        }
        // NO Else ; No Specific Wifi Parameters To Copy.
    }
    // No else: skipped by error

    // delete the SpecificParameters if an error occurred
    if (localError != ARDISCOVERY_OK)
    {
        ARDISCOVERY_DEVICE_Wifi_DeleteSpecificParameters (device);
    }
    // No else: skipped no error

    // return the error
    if (error != NULL)
    {
        *error = localError;
    }
    // No else: error is not returned

    return specificWifiParam;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_AddConnectionCallbacks (ARDISCOVERY_Device_t *device, ARDISCOVERY_Device_ConnectionJsonCallback_t sendJsonCallback, ARDISCOVERY_Device_ConnectionJsonCallback_t receiveJsonCallback, void *customData)
{
    // -- Wifi Add Connection Callbacks --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParam = NULL;

    // check parameters
    if ((device == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_NET) ||
        (device->specificParameters == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        // cast device->specificWifiParam
        specificWifiParam = (ARDISCOVERY_DEVICE_WIFI_t *)device->specificParameters;

        specificWifiParam->sendJsonCallback = sendJsonCallback;
        specificWifiParam->receiveJsonCallback = receiveJsonCallback;
        specificWifiParam->jsonCallbacksCustomData = customData;
    }

    return error;
}

ARNETWORKAL_Manager_t *ARDISCOVERY_DEVICE_Wifi_NewARNetworkAL (ARDISCOVERY_Device_t *device, eARDISCOVERY_ERROR *error, eARNETWORKAL_ERROR *errorAL)
{
    // -- Create a new networlAL adapted to the device --

    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    eARNETWORKAL_ERROR localErrorAL = ARNETWORKAL_OK;
    ARNETWORKAL_Manager_t *networkAL = NULL;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParam = NULL;

    // check parameters
    if ((device == NULL) ||
        (device->specificParameters == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_NET))
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (localError == ARDISCOVERY_OK)
    {
        // Cast of device->specificParameters
        specificWifiParam = (ARDISCOVERY_DEVICE_WIFI_t *) device->specificParameters;

        // discovery connection
        localError = ARDISCOVERY_DEVICE_Wifi_DiscoveryConnect (device);
    }

    if (localError == ARDISCOVERY_OK)
    {
        // Create the ARNetworkALManager
        networkAL = ARNETWORKAL_Manager_New (&localErrorAL);
    }

    if ((localError == ARDISCOVERY_OK) && (localErrorAL == ARNETWORKAL_OK))
    {
        // Initialize the ARNetworkALManager
        localErrorAL = ARNETWORKAL_Manager_InitWifiNetwork (networkAL, specificWifiParam->address, specificWifiParam->controllerToDevicePort, specificWifiParam->deviceToControllerPort, 1);

    }

    if ((localError == ARDISCOVERY_OK) && (localErrorAL == ARNETWORKAL_OK))
    {
        // Set the send socket QoS (if needed)
        if (specificWifiParam->qos_level == 1)
        {
            localErrorAL = ARNETWORKAL_Manager_SetSendClassSelector(networkAL, ARSAL_SOCKET_CLASS_SELECTOR_CS6);
        }
    }

    if ((localError == ARDISCOVERY_OK) && (localErrorAL == ARNETWORKAL_OK))
    {
        // Set the recv socket QoS (if needed)
        if (specificWifiParam->qos_level == 1)
        {
            localErrorAL = ARNETWORKAL_Manager_SetRecvClassSelector(networkAL, ARSAL_SOCKET_CLASS_SELECTOR_CS6);
        }
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
        ARDISCOVERY_DEVICE_Wifi_DeleteARNetworkAL (device, &networkAL);
    }

    return networkAL;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_DeleteARNetworkAL (ARDISCOVERY_Device_t *device, ARNETWORKAL_Manager_t **networkAL)
{
    // --  Delete a networlAL create by ARDISCOVERY_DEVICE_Wifi_NewARNetworkAL --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_NET))
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

                ARNETWORKAL_Manager_CloseWifiNetwork((*networkAL));
                ARNETWORKAL_Manager_Delete(networkAL);
            }
        }
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_GetIpAddress (ARDISCOVERY_Device_t *device, char *ipAddress, int length)
{
    // -- Get the IP address of the device  --

    // local declarations
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParam = NULL;
    int ipAddressSize = 0;

    // check parameters
    if((device == NULL) ||
       (device->networkType != ARDISCOVERY_NETWORK_TYPE_NET) ||
       (device->specificParameters == NULL) ||
       (ipAddress == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }

    if (error == ARDISCOVERY_OK)
    {
        // Cast of device->specificParameters
        specificWifiParam = (ARDISCOVERY_DEVICE_WIFI_t *) device->specificParameters;

        ipAddressSize = strlen(specificWifiParam->address) + 1;
        // check length
        if(length < ipAddressSize)
        {
            error = ARDISCOVERY_ERROR_OUTPUT_LENGTH;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        // copy the ip address
        snprintf(ipAddress, length, "%s", specificWifiParam->address);
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_SetQoSLevel (ARDISCOVERY_Device_t *device, int level)
{
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParam = NULL;

    if ((device == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_NET) ||
        (level < 0) ||
        (level > 1))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }

    if (error == ARDISCOVERY_OK)
    {
        specificWifiParam = (ARDISCOVERY_DEVICE_WIFI_t *) device->specificParameters;
        specificWifiParam->requested_qos_level = level;
    }

    return error;
}

static eARCOMMANDS_GENERATOR_ERROR BebopStartStreamingGenerator(uint8_t *buffer, int32_t buffLen, int32_t *cmdLen)
{
    return ARCOMMANDS_Generator_GenerateARDrone3MediaStreamingVideoEnable(buffer, buffLen, cmdLen, 1);
}

static eARCOMMANDS_GENERATOR_ERROR BebopStopStreamingGenerator(uint8_t *buffer, int32_t buffLen, int32_t *cmdLen)
{
    return ARCOMMANDS_Generator_GenerateARDrone3MediaStreamingVideoEnable(buffer, buffLen, cmdLen, 0);
}


eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitBebopNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a Bebop, a Bebop 2 or linked product (as the SkyController). --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) ||
        (networkConfiguration == NULL) ||
        ((device->productID != ARDISCOVERY_PRODUCT_ARDRONE) &&
         (device->productID != ARDISCOVERY_PRODUCT_SKYCONTROLLER) &&
         (device->productID != ARDISCOVERY_PRODUCT_BEBOP_2) &&
         (device->productID != ARDISCOVERY_PRODUCT_EVINRUDE) &&
         (device->productID != ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_4) &&
         (device->productID != ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_5) &&
         (device->productID != ARDISCOVERY_PRODUCT_CHIMERA))
        )
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    memset(networkConfiguration, 0x0, sizeof(*networkConfiguration));

    static ARNETWORK_IOBufferParam_t c2dParams[] = {
        /* Non-acknowledged commands. */
        {
            .ID = BEBOP_CONTROLLER_TO_DEVICE_NONACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 6,
            .dataCopyMaxSize = 128,
            .isOverwriting = 1,
        },
        /* Acknowledged commands. */
        {
            .ID = BEBOP_CONTROLLER_TO_DEVICE_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 150,
            .numberOfRetry = 5,
            .numberOfCell = 100,
            .dataCopyMaxSize = 4096,
            .isOverwriting = 0,
        },
        /* Emergency commands. */
        {
            .ID = BEBOP_CONTROLLER_TO_DEVICE_EMERGENCY_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 150,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 1,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
    };
    size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static ARNETWORK_IOBufferParam_t d2cParams[] = {
        {
            .ID = BEBOP_DEVICE_TO_CONTROLLER_NAVDATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 20,
            .dataCopyMaxSize = 128,
            .isOverwriting = 1,
        },
        {
            .ID = BEBOP_DEVICE_TO_CONTROLLER_EVENT_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 150,
            .numberOfRetry = 5,
            .numberOfCell = 100,
            .dataCopyMaxSize = 4096,
            .isOverwriting = 0,
        },
    };
    size_t numD2cParams = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static int commandBufferIds[] = {
        BEBOP_DEVICE_TO_CONTROLLER_NAVDATA_ID,
        BEBOP_DEVICE_TO_CONTROLLER_EVENT_ID,
    };

    size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);

    if (error == ARDISCOVERY_OK)
    {
        networkConfiguration->controllerLoopIntervalMs = 25;

        networkConfiguration->controllerToDeviceNotAckId = BEBOP_CONTROLLER_TO_DEVICE_NONACK_ID;
        networkConfiguration->controllerToDeviceAckId = BEBOP_CONTROLLER_TO_DEVICE_ACK_ID;
        networkConfiguration->controllerToDeviceHightPriority = BEBOP_CONTROLLER_TO_DEVICE_EMERGENCY_ID;
        networkConfiguration->controllerToDeviceARStreamAck = -1;
        networkConfiguration->controllerToDeviceARStreamAudioAck = -1;
        networkConfiguration->controllerToDeviceARStreamAudioData = -1;
        networkConfiguration->deviceToControllerNotAckId = BEBOP_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerAckId = BEBOP_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerARStreamData = -1;
        networkConfiguration->deviceToControllerARStreamAudioData = -1;
        networkConfiguration->deviceToControllerARStreamAudioAck = -1;

        networkConfiguration->hasVideo = 1;
        networkConfiguration->streamType = ARDISCOVERY_STREAM_STARTSTOP_ARCOMMANDS;
        networkConfiguration->startCommand = &BebopStartStreamingGenerator;
        networkConfiguration->stopCommand = &BebopStopStreamingGenerator;

        networkConfiguration->controllerToDeviceParams = c2dParams;
        networkConfiguration->numberOfControllerToDeviceParam = numC2dParams;

        networkConfiguration->deviceToControllerParams = d2cParams;
        networkConfiguration->numberOfDeviceToControllerParam = numD2cParams;

        networkConfiguration->pingDelayMs = 0;

        networkConfiguration->numberOfDeviceToControllerCommandsBufferIds = numOfCommandBufferIds;
        networkConfiguration->deviceToControllerCommandsBufferIds = commandBufferIds;
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitSkyControllerNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a SkyController. --
    // This should be the same as the Bebop to be able to route the packets
    return ARDISCOVERY_DEVICE_Wifi_InitBebopNetworkConfiguration(device, networkConfiguration);
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitBebop2NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a Bebop 2. --
    // This should be the same as the Bebop to be able to be used by the SkyController
    return ARDISCOVERY_DEVICE_Wifi_InitBebopNetworkConfiguration(device, networkConfiguration);
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitUnknownproduct_4NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a Unknownproduct_4. --
    // This should be the same as the Bebop to be able to be used by the SkyController
    return ARDISCOVERY_DEVICE_Wifi_InitBebopNetworkConfiguration(device, networkConfiguration);
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitUnknownproduct_5NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to an Unknownproduct_5. --
    // This should be the same as the Bebop to be able to be used by the SkyController
    return ARDISCOVERY_DEVICE_Wifi_InitBebopNetworkConfiguration(device, networkConfiguration);
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitDelos3NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a Mambo FPV. --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) ||
        (networkConfiguration == NULL) ||
        (device->productID != ARDISCOVERY_PRODUCT_MINIDRONE_DELOS3))
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
            .dataCopyMaxSize = 128,
            .isOverwriting = 1,
        },
        /* Acknowledged commands. */
        {
            .ID = ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = 150,
            .numberOfRetry = 5,
            .numberOfCell = 100,
            .dataCopyMaxSize = 128,
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
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        {
            .ID = ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = 500,
            .numberOfRetry = 3,
            .numberOfCell = 20,
            .dataCopyMaxSize = 128,
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
        networkConfiguration->controllerToDeviceHightPriority = -1;
        networkConfiguration->controllerToDeviceARStreamAck = -1;
        networkConfiguration->controllerToDeviceARStreamAudioAck = -1;
        networkConfiguration->controllerToDeviceARStreamAudioData = -1;
        networkConfiguration->deviceToControllerNotAckId = ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerAckId = ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerARStreamData = -1;
        networkConfiguration->deviceToControllerARStreamAudioData = -1;
        networkConfiguration->deviceToControllerARStreamAudioAck = -1;

        networkConfiguration->hasVideo = 1;
        networkConfiguration->streamType = ARDISCOVERY_STREAM_STARTSTOP_RTSP;
        networkConfiguration->rtspAddress = strdup("rtsp://192.168.99.1/media/stream2");

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



eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitChimeraNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a Chimera. --
    // This should be the same as the Bebop to be able to be used by the SkyController
    return ARDISCOVERY_DEVICE_Wifi_InitBebopNetworkConfiguration(device, networkConfiguration);
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitSkyController2NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a SkyControllerNG. --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) ||
        (networkConfiguration == NULL) ||
        ((device->productID != ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG) &&
         (device->productID != ARDISCOVERY_PRODUCT_SKYCONTROLLER_2) &&
         (device->productID != ARDISCOVERY_PRODUCT_SKYCONTROLLER_2P))
        )
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    memset(networkConfiguration, 0x0, sizeof(*networkConfiguration));

    static ARNETWORK_IOBufferParam_t c2dParams[] = {
        /* Non-acknowledged commands. */
        {
            .ID = BEBOP_CONTROLLER_TO_DEVICE_NONACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 6,
            .dataCopyMaxSize = 128,
            .isOverwriting = 1,
        },
        /* Acknowledged commands. */
        {
            .ID = BEBOP_CONTROLLER_TO_DEVICE_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 150,
            .numberOfRetry = 5,
            .numberOfCell = 100,
            .dataCopyMaxSize = 4096,
            .isOverwriting = 0,
        },
        /* Emergency commands. */
        {
            .ID = BEBOP_CONTROLLER_TO_DEVICE_EMERGENCY_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 150,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 1,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
    };
    size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static ARNETWORK_IOBufferParam_t d2cParams[] = {
        {
            .ID = BEBOP_DEVICE_TO_CONTROLLER_NAVDATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 20,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        {
            .ID = BEBOP_DEVICE_TO_CONTROLLER_EVENT_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 150,
            .numberOfRetry = 5,
            .numberOfCell = 100,
            .dataCopyMaxSize = 4096,
            .isOverwriting = 0,
        },
    };
    size_t numD2cParams = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static int commandBufferIds[] = {
        BEBOP_DEVICE_TO_CONTROLLER_NAVDATA_ID,
        BEBOP_DEVICE_TO_CONTROLLER_EVENT_ID,
    };

    size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);

    if (error == ARDISCOVERY_OK)
    {
        networkConfiguration->controllerLoopIntervalMs = 25;

        networkConfiguration->controllerToDeviceNotAckId = BEBOP_CONTROLLER_TO_DEVICE_NONACK_ID;
        networkConfiguration->controllerToDeviceAckId = BEBOP_CONTROLLER_TO_DEVICE_ACK_ID;
        networkConfiguration->controllerToDeviceHightPriority = BEBOP_CONTROLLER_TO_DEVICE_EMERGENCY_ID;
        networkConfiguration->controllerToDeviceARStreamAck = -1;
        networkConfiguration->controllerToDeviceARStreamAudioAck = -1;
        networkConfiguration->controllerToDeviceARStreamAudioData = -1;
        networkConfiguration->deviceToControllerNotAckId = BEBOP_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerAckId = BEBOP_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerARStreamData = -1;
        networkConfiguration->deviceToControllerARStreamAudioData = -1;
        networkConfiguration->deviceToControllerARStreamAudioAck = -1;

        networkConfiguration->hasVideo = 1;
        networkConfiguration->streamType = ARDISCOVERY_STREAM_STARTSTOP_ARCOMMANDS;
        networkConfiguration->startCommand = &BebopStartStreamingGenerator;
        networkConfiguration->stopCommand = &BebopStopStreamingGenerator;

        networkConfiguration->controllerToDeviceParams = c2dParams;
        networkConfiguration->numberOfControllerToDeviceParam = numC2dParams;

        networkConfiguration->deviceToControllerParams = d2cParams;
        networkConfiguration->numberOfDeviceToControllerParam = numD2cParams;

        networkConfiguration->pingDelayMs = 0;

        networkConfiguration->numberOfDeviceToControllerCommandsBufferIds = numOfCommandBufferIds;
        networkConfiguration->deviceToControllerCommandsBufferIds = commandBufferIds;
    }

    return error;
}

static eARCOMMANDS_GENERATOR_ERROR JSStartStreamingGenerator(uint8_t *buffer, int32_t buffLen, int32_t *cmdLen)
{
    return ARCOMMANDS_Generator_GenerateJumpingSumoMediaStreamingVideoEnable(buffer, buffLen, cmdLen, 1);
}

static eARCOMMANDS_GENERATOR_ERROR JSStopStreamingGenerator(uint8_t *buffer, int32_t buffLen, int32_t *cmdLen)
{
    return ARCOMMANDS_Generator_GenerateJumpingSumoMediaStreamingVideoEnable(buffer, buffLen, cmdLen, 0);
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitJumpingSumoNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a Jumping Sumo. --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) ||
        (device->productID != ARDISCOVERY_PRODUCT_JS) ||
        (networkConfiguration == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    memset(networkConfiguration, 0x0, sizeof(*networkConfiguration));

    static ARNETWORK_IOBufferParam_t c2dParams[] = {
        /* Non-acknowledged commands. */
        {
            .ID = JUMPINGSUMO_CONTROLLER_TO_DEVICE_NONACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 5,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 10,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        /* Acknowledged commands. */
        {
            .ID = JUMPINGSUMO_CONTROLLER_TO_DEVICE_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = 500,
            .numberOfRetry = 3,
            .numberOfCell = 20,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        /* Video ACK (Initialized later) */
        {
            .ID = JUMPINGSUMO_CONTROLLER_TO_DEVICE_VIDEO_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
    };
    size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static ARNETWORK_IOBufferParam_t d2cParams[] = {
        {
            .ID = JUMPINGSUMO_DEVICE_TO_CONTROLLER_NAVDATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 10,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        {
            .ID = JUMPINGSUMO_DEVICE_TO_CONTROLLER_EVENT_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = 500,
            .numberOfRetry = 3,
            .numberOfCell = 20,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        /* Video data (Initialized later) */
        {
            .ID = JUMPINGSUMO_DEVICE_TO_CONTROLLER_VIDEO_DATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
    };
    size_t numD2cParams = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static int commandBufferIds[] = {
        JUMPINGSUMO_DEVICE_TO_CONTROLLER_NAVDATA_ID,
        JUMPINGSUMO_DEVICE_TO_CONTROLLER_EVENT_ID,
    };

    size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);

    if (error == ARDISCOVERY_OK)
    {
        networkConfiguration->controllerLoopIntervalMs = 50;

        networkConfiguration->controllerToDeviceNotAckId = JUMPINGSUMO_CONTROLLER_TO_DEVICE_NONACK_ID;
        networkConfiguration->controllerToDeviceAckId = JUMPINGSUMO_CONTROLLER_TO_DEVICE_ACK_ID;
        networkConfiguration->controllerToDeviceHightPriority = -1;
        networkConfiguration->controllerToDeviceARStreamAck = JUMPINGSUMO_CONTROLLER_TO_DEVICE_VIDEO_ACK_ID;
        networkConfiguration->controllerToDeviceARStreamAudioAck = -1;
        networkConfiguration->controllerToDeviceARStreamAudioData = -1;
        networkConfiguration->deviceToControllerNotAckId = JUMPINGSUMO_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerAckId = JUMPINGSUMO_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        //int deviceToControllerHightPriority = -1;
        networkConfiguration->deviceToControllerARStreamData = JUMPINGSUMO_DEVICE_TO_CONTROLLER_VIDEO_DATA_ID;
        networkConfiguration->deviceToControllerARStreamAudioData = -1;
        networkConfiguration->deviceToControllerARStreamAudioAck = -1;

        networkConfiguration->hasVideo = 1;
        networkConfiguration->streamType = ARDISCOVERY_STREAM_STARTSTOP_ARCOMMANDS;
        networkConfiguration->startCommand = &JSStartStreamingGenerator;
        networkConfiguration->stopCommand = &JSStopStreamingGenerator;

        networkConfiguration->controllerToDeviceParams = c2dParams;
        networkConfiguration->numberOfControllerToDeviceParam = numC2dParams;

        networkConfiguration->deviceToControllerParams = d2cParams;
        networkConfiguration->numberOfDeviceToControllerParam = numD2cParams;

        networkConfiguration->pingDelayMs = 0;

        networkConfiguration->numberOfDeviceToControllerCommandsBufferIds = numOfCommandBufferIds;
        networkConfiguration->deviceToControllerCommandsBufferIds = commandBufferIds;
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitJumpingSumoEvoNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a Jumping Sumo. --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) ||
        ((device->productID != ARDISCOVERY_PRODUCT_JS_EVO_LIGHT) &&
         (device->productID != ARDISCOVERY_PRODUCT_JS_EVO_RACE)) ||
        (networkConfiguration == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    memset(networkConfiguration, 0x0, sizeof(*networkConfiguration));

    static ARNETWORK_IOBufferParam_t c2dParams[] = {
        /* Non-acknowledged commands. */
        {
            .ID = JUMPINGSUMO_CONTROLLER_TO_DEVICE_NONACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 5,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 10,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        /* Acknowledged commands. */
        {
            .ID = JUMPINGSUMO_CONTROLLER_TO_DEVICE_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
                .sendingWaitTimeMs = 20,
            .ackTimeoutMs = 500,
            .numberOfRetry = 3,
            .numberOfCell = 20,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        /* Video ACK (Initialized later) */
        {
            .ID = JUMPINGSUMO_CONTROLLER_TO_DEVICE_VIDEO_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
        /* Audio ACK (Initialized later) */
        {
            .ID = JUMPINGSUMO_CONTROLLER_TO_DEVICE_AUDIO_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
        /* Audio Data (Initialized later) */
        {
            .ID = JUMPINGSUMO_CONTROLLER_TO_DEVICE_AUDIO_DATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
    };
    size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static ARNETWORK_IOBufferParam_t d2cParams[] = {
        {
            .ID = JUMPINGSUMO_DEVICE_TO_CONTROLLER_NAVDATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 10,
           .dataCopyMaxSize = 128,
           .isOverwriting = 0,
        },
        {
            .ID = JUMPINGSUMO_DEVICE_TO_CONTROLLER_EVENT_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = 500,
            .numberOfRetry = 3,
            .numberOfCell = 20,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        /* Video data (Initialized later) */
        {
            .ID = JUMPINGSUMO_DEVICE_TO_CONTROLLER_VIDEO_DATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
        /* Audio data (Initialized later) */
        {
            .ID = JUMPINGSUMO_DEVICE_TO_CONTROLLER_AUDIO_DATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
        /* Audio ACK (Initialized later) */
        {
            .ID = JUMPINGSUMO_DEVICE_TO_CONTROLLER_AUDIO_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
    };
    size_t numD2cParams = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static int commandBufferIds[] = {
        JUMPINGSUMO_DEVICE_TO_CONTROLLER_NAVDATA_ID,
        JUMPINGSUMO_DEVICE_TO_CONTROLLER_EVENT_ID,
    };

    size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);

    if (error == ARDISCOVERY_OK)
    {
        networkConfiguration->controllerLoopIntervalMs = 50;

        networkConfiguration->controllerToDeviceNotAckId = JUMPINGSUMO_CONTROLLER_TO_DEVICE_NONACK_ID;
        networkConfiguration->controllerToDeviceAckId = JUMPINGSUMO_CONTROLLER_TO_DEVICE_ACK_ID;
        networkConfiguration->controllerToDeviceHightPriority = -1;
        networkConfiguration->controllerToDeviceARStreamAck = JUMPINGSUMO_CONTROLLER_TO_DEVICE_VIDEO_ACK_ID;
        networkConfiguration->controllerToDeviceARStreamAudioAck = JUMPINGSUMO_CONTROLLER_TO_DEVICE_AUDIO_ACK_ID;
        networkConfiguration->controllerToDeviceARStreamAudioData = JUMPINGSUMO_CONTROLLER_TO_DEVICE_AUDIO_DATA_ID;
        networkConfiguration->deviceToControllerNotAckId = JUMPINGSUMO_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerAckId = JUMPINGSUMO_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        //int deviceToControllerHightPriority = -1;
        networkConfiguration->deviceToControllerARStreamData = JUMPINGSUMO_DEVICE_TO_CONTROLLER_VIDEO_DATA_ID;
        networkConfiguration->deviceToControllerARStreamAudioData = JUMPINGSUMO_DEVICE_TO_CONTROLLER_AUDIO_DATA_ID;
        networkConfiguration->deviceToControllerARStreamAudioAck = JUMPINGSUMO_DEVICE_TO_CONTROLLER_AUDIO_ACK_ID;

        networkConfiguration->hasVideo = 1;
        networkConfiguration->streamType = ARDISCOVERY_STREAM_STARTSTOP_ARCOMMANDS;
        networkConfiguration->startCommand = &JSStartStreamingGenerator;
        networkConfiguration->stopCommand = &JSStopStreamingGenerator;

        networkConfiguration->controllerToDeviceParams = c2dParams;
        networkConfiguration->numberOfControllerToDeviceParam = numC2dParams;

        networkConfiguration->deviceToControllerParams = d2cParams;
        networkConfiguration->numberOfDeviceToControllerParam = numD2cParams;

        networkConfiguration->pingDelayMs = 0;

        networkConfiguration->numberOfDeviceToControllerCommandsBufferIds = numOfCommandBufferIds;
        networkConfiguration->deviceToControllerCommandsBufferIds = commandBufferIds;
    }

    return error;
}

static eARCOMMANDS_GENERATOR_ERROR PowerUpStartStreamingGenerator(uint8_t *buffer, int32_t buffLen, int32_t *cmdLen)
{
    return ARCOMMANDS_Generator_GeneratePowerupMediaStreamingVideoEnable(buffer, buffLen, cmdLen, 1);
}

static eARCOMMANDS_GENERATOR_ERROR PowerUpStopStreamingGenerator(uint8_t *buffer, int32_t buffLen, int32_t *cmdLen)
{
    return ARCOMMANDS_Generator_GeneratePowerupMediaStreamingVideoEnable(buffer, buffLen, cmdLen, 0);
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitPowerUpNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a PowerUp. --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) ||
        (device->productID != ARDISCOVERY_PRODUCT_POWER_UP) ||
        (networkConfiguration == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    memset(networkConfiguration, 0x0, sizeof(*networkConfiguration));

    static ARNETWORK_IOBufferParam_t c2dParams[] = {
        /* Non-acknowledged commands. */
        {
            .ID = POWERUP_CONTROLLER_TO_DEVICE_NONACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 5,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 10,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        /* Acknowledged commands. */
        {
            .ID = POWERUP_CONTROLLER_TO_DEVICE_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = 500,
            .numberOfRetry = 3,
            .numberOfCell = 20,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        /* Video ACK (Initialized later) */
        {
            .ID = POWERUP_CONTROLLER_TO_DEVICE_VIDEO_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
        /* Audio ACK (Initialized later) */
        {
            .ID = POWERUP_CONTROLLER_TO_DEVICE_AUDIO_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
    };
    size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static ARNETWORK_IOBufferParam_t d2cParams[] = {
        {
            .ID = POWERUP_DEVICE_TO_CONTROLLER_NAVDATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 10,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        {
            .ID = POWERUP_DEVICE_TO_CONTROLLER_EVENT_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = 500,
            .numberOfRetry = 3,
            .numberOfCell = 20,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        /* Video data (Initialized later) */
        {
            .ID = POWERUP_DEVICE_TO_CONTROLLER_VIDEO_DATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
        /* Audio data (Initialized later) */
        {
            .ID = POWERUP_DEVICE_TO_CONTROLLER_AUDIO_DATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
            .sendingWaitTimeMs = 0,
            .ackTimeoutMs = 0,
            .numberOfRetry = 0,
            .numberOfCell = 0,
            .dataCopyMaxSize = 0,
            .isOverwriting = 0,
        },
    };
    size_t numD2cParams = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static int commandBufferIds[] = {
        POWERUP_DEVICE_TO_CONTROLLER_NAVDATA_ID,
        POWERUP_DEVICE_TO_CONTROLLER_EVENT_ID,
    };

    size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);

    if (error == ARDISCOVERY_OK)
    {
        networkConfiguration->controllerLoopIntervalMs = 50;

        networkConfiguration->controllerToDeviceNotAckId = POWERUP_CONTROLLER_TO_DEVICE_NONACK_ID;
        networkConfiguration->controllerToDeviceAckId = POWERUP_CONTROLLER_TO_DEVICE_ACK_ID;
        networkConfiguration->controllerToDeviceHightPriority = -1;
        networkConfiguration->controllerToDeviceARStreamAck = POWERUP_CONTROLLER_TO_DEVICE_VIDEO_ACK_ID;
        networkConfiguration->controllerToDeviceARStreamAudioAck = POWERUP_CONTROLLER_TO_DEVICE_AUDIO_ACK_ID;
        networkConfiguration->controllerToDeviceARStreamAudioData = -1;
        networkConfiguration->deviceToControllerNotAckId = POWERUP_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerAckId = POWERUP_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        //int deviceToControllerHightPriority = -1;
        networkConfiguration->deviceToControllerARStreamData = POWERUP_DEVICE_TO_CONTROLLER_VIDEO_DATA_ID;
        networkConfiguration->deviceToControllerARStreamAudioData = POWERUP_DEVICE_TO_CONTROLLER_AUDIO_DATA_ID;
        networkConfiguration->deviceToControllerARStreamAudioAck = -1;

        networkConfiguration->hasVideo = 1;
        networkConfiguration->streamType = ARDISCOVERY_STREAM_STARTSTOP_ARCOMMANDS;
        networkConfiguration->startCommand = &PowerUpStartStreamingGenerator;
        networkConfiguration->stopCommand = &PowerUpStopStreamingGenerator;

        networkConfiguration->controllerToDeviceParams = c2dParams;
        networkConfiguration->numberOfControllerToDeviceParam = numC2dParams;

        networkConfiguration->deviceToControllerParams = d2cParams;
        networkConfiguration->numberOfDeviceToControllerParam = numD2cParams;

        networkConfiguration->pingDelayMs = 0;

        networkConfiguration->numberOfDeviceToControllerCommandsBufferIds = numOfCommandBufferIds;
        networkConfiguration->deviceToControllerCommandsBufferIds = commandBufferIds;
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitEvinrudeNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
    // -- Initilize network Configuration adapted to a Bebop 2. --
    // This should be the same as the SkyController to be able to route the packets
    return ARDISCOVERY_DEVICE_Wifi_InitBebopNetworkConfiguration(device, networkConfiguration);
}


 /*************************
 * local Implementation
 *************************/

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_DiscoveryConnect (ARDISCOVERY_Device_t *device)
{

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParam = NULL;
    ARDISCOVERY_Connection_ConnectionData_t *discoveryData = NULL;

    // check parameters
    if ((device == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_NET) ||
        (device->specificParameters == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        // Cast of device->specificParameters
        specificWifiParam = (ARDISCOVERY_DEVICE_WIFI_t *)device->specificParameters;

        // New Discovery Connection
        discoveryData = ARDISCOVERY_Connection_New (ARDISCOVERY_DEVICE_Wifi_SendJsonCallback, ARDISCOVERY_DEVICE_Wifi_ReceiveJsonCallback, device, &error);
    }

    if (error == ARDISCOVERY_OK)
    {
        error = ARDISCOVERY_Connection_ControllerConnection (discoveryData, specificWifiParam->dicoveryPort, specificWifiParam->address);
    }

    // Cleanup
    ARDISCOVERY_Connection_Delete(&discoveryData);

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData)
{
    // -- Connection callback to send the Json --

    // local declarations
    ARDISCOVERY_Device_t *device = (ARDISCOVERY_Device_t *)customData;
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParam = NULL;

    int jsonSize = 0;
    json_object *jsonObj = NULL;
    json_object *valueJsonObj = NULL;

    if ((dataTx == NULL) ||
        (dataTxSize == NULL) ||
        (device == NULL) ||
        (device->specificParameters == NULL))
    {
        error = ARDISCOVERY_ERROR; //TODO see if set bad parameter
    }

    if (error == ARDISCOVERY_OK)
    {
        // Cast of device->specificParameters
        specificWifiParam = (ARDISCOVERY_DEVICE_WIFI_t *)device->specificParameters;

        jsonObj = json_object_new_object ();

        // add ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY
        valueJsonObj = json_object_new_int (specificWifiParam->requested_qos_level);
        json_object_object_add (jsonObj, ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY, valueJsonObj);

        // add ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY
        valueJsonObj = json_object_new_int (specificWifiParam->deviceToControllerPort);
        json_object_object_add (jsonObj, ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY, valueJsonObj);

        // sending Json callback
        if (specificWifiParam->sendJsonCallback != NULL)
        {
            error = specificWifiParam->sendJsonCallback (jsonObj, specificWifiParam->jsonCallbacksCustomData);
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        // copy json in dataTx
        jsonSize = strlen(json_object_to_json_string (jsonObj));
        if (jsonSize <= ARDISCOVERY_CONNECTION_TX_BUFFER_SIZE)
        {
            memcpy (dataTx, json_object_to_json_string (jsonObj), jsonSize);
            *dataTxSize = jsonSize;
        }
        else
        {
            error = ARDISCOVERY_ERROR_JSON_BUFFER_SIZE;
        }
    }

    if (jsonObj != NULL)
    {
        /* free json object */
        json_object_put (jsonObj);
        jsonObj = NULL;
    }


    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData)
{
    // -- Connection callback to receive the Json --

    // local declarations
    ARDISCOVERY_Device_t *device = (ARDISCOVERY_Device_t *)customData;
    ARDISCOVERY_DEVICE_WIFI_t *specificWifiParam = NULL;
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    json_object *jsonObj = NULL;
    json_object *valueJsonObj = NULL;
    json_bool json_res;


    if ((dataRx == NULL) ||
        (dataRxSize == 0) ||
        (device == NULL) ||
        (device->specificParameters == NULL))
    {
        error = ARDISCOVERY_ERROR;
    }

    if (error == ARDISCOVERY_OK)
    {
        // Cast of device->specificParameters
        specificWifiParam = (ARDISCOVERY_DEVICE_WIFI_t *)device->specificParameters;

        // parssing of the json
        jsonObj = json_tokener_parse ((char *)dataRx);
        if (is_error (jsonObj))
        {
            error = ARDISCOVERY_ERROR_JSON_PARSSING;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        // get ARDISCOVERY_CONNECTION_JSON_C2DPORT_KEY
        json_res = json_object_object_get_ex (jsonObj, ARDISCOVERY_CONNECTION_JSON_C2DPORT_KEY, &valueJsonObj);
        if (json_res && valueJsonObj != NULL)
        {
            specificWifiParam->controllerToDevicePort = json_object_get_int(valueJsonObj);
        }

        // get ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY
        json_object_object_get_ex (jsonObj, ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY, &valueJsonObj);
        if (json_res && valueJsonObj != NULL)
        {
            specificWifiParam->qos_level = json_object_get_int(valueJsonObj);
        }

        // get ARDISCOVERY_CONNECTION_JSON_STATUS_KEY

        json_res = json_object_object_get_ex (jsonObj, ARDISCOVERY_CONNECTION_JSON_STATUS_KEY, &valueJsonObj);
        if (json_res && valueJsonObj != NULL)
        {
            specificWifiParam->connectionStatus = json_object_get_int(valueJsonObj);
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        // receiving Json callback
        if (specificWifiParam->receiveJsonCallback != NULL)
        {
            specificWifiParam->receiveJsonCallback (jsonObj, specificWifiParam->jsonCallbacksCustomData);
        }
    }

    if (jsonObj != NULL)
    {
        /* free json object */
        json_object_put (jsonObj);
        jsonObj = NULL;
    }

    return error;
}
