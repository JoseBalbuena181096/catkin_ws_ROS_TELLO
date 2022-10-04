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
 * @file ARDISCOVERY_DEVICE_Usb.c
 * @brief Discovery usb Device contains the informations of a device discovered
 * @date 02/03/2015
 * @author maxime.maitre@parrot.com
 */

#include <stdlib.h>
#include <json-c/json.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetworkAL/ARNETWORKAL_Manager.h>
#include <libARNetworkAL/ARNETWORKAL_Error.h>
#include <libARDiscovery/ARDISCOVERY_MuxDiscovery.h>
#include <libARDiscovery/ARDISCOVERY_Device.h>

#ifdef BUILD_LIBMUX
#include <libmux.h>
#else
struct mux_ctx;
#endif

#include "ARDISCOVERY_DEVICE_Usb.h"

#define ARDISCOVERY_DEVICE_USB_TAG "ARDISCOVERY_DEVICE_USB"

/*************************
 * Private header
 *************************/

// Mpp
#define MPP_DEVICE_TO_CONTROLLER_PORT 43210

#define MPP_CONTROLLER_TO_DEVICE_NONACK_ID 10
#define MPP_CONTROLLER_TO_DEVICE_ACK_ID 11
#define MPP_CONTROLLER_TO_DEVICE_EMERGENCY_ID 12
#define MPP_CONTROLLER_TO_DEVICE_VIDEO_ACK_ID 13
#define MPP_DEVICE_TO_CONTROLLER_NAVDATA_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 1)
#define MPP_DEVICE_TO_CONTROLLER_EVENT_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 2)
#define MPP_DEVICE_TO_CONTROLLER_VIDEO_DATA_ID ((ARNETWORKAL_MANAGER_WIFI_ID_MAX /2) - 3)

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_DiscoveryConnect (ARDISCOVERY_Device_t *device);

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData);

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData);

/*************************
 * Implementation
 *************************/

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_CreateSpecificParameters (ARDISCOVERY_Device_t *device, struct mux_ctx *mux)
{
#ifdef BUILD_LIBMUX
    // Initialize usb specific parameters
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_USB_t *specificUsbParam = NULL;

    // Check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_USBMUX))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        specificUsbParam = malloc(sizeof(ARDISCOVERY_DEVICE_USB_t));
        if (specificUsbParam != NULL)
        {
            device->specificParameters = specificUsbParam;
            mux_ref(mux);
            specificUsbParam->mux = mux;
            ARSAL_Sem_Init(&specificUsbParam->sem, 0, 0);
            specificUsbParam->sendJsonCallback = NULL;
            specificUsbParam->receiveJsonCallback = NULL;
            specificUsbParam->jsonCallbacksCustomData = NULL;

            specificUsbParam->connectionStatus = ARDISCOVERY_OK;
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
        ARDISCOVERY_DEVICE_Usb_DeleteSpecificParameters (device);
    }
    // No else: skipped no error

    return error;
#else
    return ARDISCOVERY_ERROR;
#endif
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_DeleteSpecificParameters (ARDISCOVERY_Device_t *device)
{
#ifdef BUILD_LIBMUX
    // -- Delete SpecificParameters allocated by the usb initialization --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_USB_t *specificUsbParam = NULL;

    // check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_USBMUX))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        // free specific parameter
        if (device->specificParameters != NULL)
        {
            specificUsbParam = (ARDISCOVERY_DEVICE_USB_t *)device->specificParameters;
            ARSAL_Sem_Destroy(&specificUsbParam->sem);
            mux_unref(specificUsbParam->mux);
            free (device->specificParameters);
            device->specificParameters = NULL;
            specificUsbParam = NULL;
        }
    }

    return error;
#else
    return ARDISCOVERY_ERROR;
#endif
}

void *ARDISCOVERY_DEVICE_Usb_GetCopyOfSpecificParameters (ARDISCOVERY_Device_t *device, eARDISCOVERY_ERROR *error)
{
#ifdef BUILD_LIBMUX
    // -- Copy the specificParameters --

    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_USB_t *specificUsbParamToCopy = NULL;
    ARDISCOVERY_DEVICE_USB_t *specificUsbParam = NULL;

    // check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_USBMUX))
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (localError == ARDISCOVERY_OK)
    {
        // cast device->specificUsbParam
        specificUsbParamToCopy = (ARDISCOVERY_DEVICE_USB_t *)device->specificParameters;

        if (specificUsbParamToCopy != NULL)
        {
            // Copy usb specific parameters
            specificUsbParam = malloc(sizeof(ARDISCOVERY_DEVICE_USB_t));
            if (specificUsbParam != NULL)
            {
                int val;
                specificUsbParam->mux = specificUsbParamToCopy->mux;
                mux_ref(specificUsbParam->mux);

                ARSAL_Sem_Getvalue(&specificUsbParamToCopy->sem, &val);
                ARSAL_Sem_Init(&specificUsbParam->sem, 0, val);

                specificUsbParam->sendJsonCallback = specificUsbParamToCopy->sendJsonCallback;
                specificUsbParam->receiveJsonCallback = specificUsbParamToCopy->receiveJsonCallback;
                specificUsbParam->jsonCallbacksCustomData = specificUsbParamToCopy->jsonCallbacksCustomData;

                specificUsbParam->connectionStatus = specificUsbParamToCopy->connectionStatus;
            }
            else
            {
                localError = ARDISCOVERY_ERROR_ALLOC;
            }

            // No else: skipped by error or no address To Copy.
        }
        // NO Else ; No Specific Usb Parameters To Copy.
    }
    // No else: skipped by error

    // delete the SpecificParameters if an error occurred
    if (localError != ARDISCOVERY_OK)
    {
        ARDISCOVERY_DEVICE_Usb_DeleteSpecificParameters (device);
    }
    // No else: skipped no error

    // return the error
    if (error != NULL)
    {
        *error = localError;
    }
    // No else: error is not returned

    return specificUsbParam;
#else
    if (error)
	    *error = ARDISCOVERY_ERROR;
    return NULL;
#endif
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_AddConnectionCallbacks (ARDISCOVERY_Device_t *device, ARDISCOVERY_Device_ConnectionJsonCallback_t sendJsonCallback, ARDISCOVERY_Device_ConnectionJsonCallback_t receiveJsonCallback, void *customData)
{
#ifdef BUILD_LIBMUX
    // -- Usb Add Connection Callbacks --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_USB_t *specificUsbParam = NULL;

    // check parameters
    if ((device == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_USBMUX) ||
        (device->specificParameters == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        // cast device->specificUsbParam
        specificUsbParam = (ARDISCOVERY_DEVICE_USB_t *)device->specificParameters;

        specificUsbParam->sendJsonCallback = sendJsonCallback;
        specificUsbParam->receiveJsonCallback = receiveJsonCallback;
        specificUsbParam->jsonCallbacksCustomData = customData;
    }

    return error;
#else
    return ARDISCOVERY_ERROR;
#endif
}

ARNETWORKAL_Manager_t *ARDISCOVERY_DEVICE_Usb_NewARNetworkAL (ARDISCOVERY_Device_t *device, eARDISCOVERY_ERROR *error, eARNETWORKAL_ERROR *errorAL)
{
#ifdef BUILD_LIBMUX
    // -- Create a new networlAL adapted to the device --

    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    eARNETWORKAL_ERROR localErrorAL = ARNETWORKAL_OK;
    ARNETWORKAL_Manager_t *networkAL = NULL;
    ARDISCOVERY_DEVICE_USB_t *specificUsbParam = NULL;

    // check parameters
    if ((device == NULL) ||
        (device->specificParameters == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_USBMUX))
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (localError == ARDISCOVERY_OK)
    {
        // Cast of device->specificParameters
        specificUsbParam = (ARDISCOVERY_DEVICE_USB_t *) device->specificParameters;

        // discovery connection
        localError = ARDISCOVERY_DEVICE_Usb_DiscoveryConnect (device);
    }

    if (localError == ARDISCOVERY_OK)
    {
        // Create the ARNetworkALManager
        networkAL = ARNETWORKAL_Manager_New (&localErrorAL);
    }

    if ((localError == ARDISCOVERY_OK) && (localErrorAL == ARNETWORKAL_OK))
    {
        // Initialize the ARNetworkALManager
        localErrorAL = ARNETWORKAL_Manager_InitMuxNetwork (networkAL, specificUsbParam->mux);

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
        ARDISCOVERY_DEVICE_Usb_DeleteARNetworkAL (device, &networkAL);
    }

    return networkAL;
#else
    if (error)
	    *error = ARDISCOVERY_ERROR;
    return NULL;
#endif
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_DeleteARNetworkAL (ARDISCOVERY_Device_t *device, ARNETWORKAL_Manager_t **networkAL)
{
#ifdef BUILD_LIBMUX
    // --  Delete a networlAL create by ARDISCOVERY_DEVICE_Usb_NewARNetworkAL --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_USBMUX))
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

                ARNETWORKAL_Manager_CloseMuxNetwork((*networkAL));
                ARNETWORKAL_Manager_Delete(networkAL);
            }
        }
    }

    return error;
#else
    return ARDISCOVERY_ERROR;
#endif
}

static eARCOMMANDS_GENERATOR_ERROR SkyController2StartStreamingGenerator(uint8_t *buffer, int32_t buffLen, int32_t *cmdLen)
{
    return ARCOMMANDS_Generator_GenerateARDrone3MediaStreamingVideoEnable(buffer, buffLen, cmdLen, 1);
}

static eARCOMMANDS_GENERATOR_ERROR SkyController2StopStreamingGenerator(uint8_t *buffer, int32_t buffLen, int32_t *cmdLen)
{
    return ARCOMMANDS_Generator_GenerateARDrone3MediaStreamingVideoEnable(buffer, buffLen, cmdLen, 0);
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_InitSkyController2NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration)
{
#ifdef BUILD_LIBMUX
    // -- Initilize network Configuration adapted to a SkyController2. --

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    // check parameters
    if ((device == NULL) ||
        (networkConfiguration == NULL) ||
        ((device->productID != ARDISCOVERY_PRODUCT_SKYCONTROLLER_2) &&
         (device->productID != ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG) &&
         (device->productID != ARDISCOVERY_PRODUCT_SKYCONTROLLER_2P)))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    memset(networkConfiguration, 0x0, sizeof(*networkConfiguration));

    static ARNETWORK_IOBufferParam_t c2dParams[] = {
        /* Non-acknowledged commands. */
        {
            .ID = MPP_CONTROLLER_TO_DEVICE_NONACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 2,
            .dataCopyMaxSize = 128,
            .isOverwriting = 1,
        },
        /* Acknowledged commands. */
        {
            .ID = MPP_CONTROLLER_TO_DEVICE_ACK_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = 500,
            .numberOfRetry = 3,
            .numberOfCell = 20,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        /* Emergency commands. */
        {
            .ID = MPP_CONTROLLER_TO_DEVICE_EMERGENCY_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
            .sendingWaitTimeMs = 10,
            .ackTimeoutMs = 100,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 1,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
    };
    size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);

    static ARNETWORK_IOBufferParam_t d2cParams[] = {
        {
            .ID = MPP_DEVICE_TO_CONTROLLER_NAVDATA_ID,
            .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
            .sendingWaitTimeMs = 20,
            .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
            .numberOfCell = 20,
            .dataCopyMaxSize = 128,
            .isOverwriting = 0,
        },
        {
            .ID = MPP_DEVICE_TO_CONTROLLER_EVENT_ID,
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
        MPP_DEVICE_TO_CONTROLLER_NAVDATA_ID,
        MPP_DEVICE_TO_CONTROLLER_EVENT_ID,
    };

    size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);

    if (error == ARDISCOVERY_OK)
    {
        networkConfiguration->controllerLoopIntervalMs = 25;

        networkConfiguration->controllerToDeviceNotAckId = MPP_CONTROLLER_TO_DEVICE_NONACK_ID;
        networkConfiguration->controllerToDeviceAckId = MPP_CONTROLLER_TO_DEVICE_ACK_ID;
        networkConfiguration->controllerToDeviceHightPriority = MPP_CONTROLLER_TO_DEVICE_EMERGENCY_ID;
        networkConfiguration->controllerToDeviceARStreamAck = MPP_CONTROLLER_TO_DEVICE_VIDEO_ACK_ID;
        networkConfiguration->controllerToDeviceARStreamAudioAck = -1;
        networkConfiguration->controllerToDeviceARStreamAudioData = -1;
        networkConfiguration->deviceToControllerNotAckId = MPP_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerAckId = MPP_DEVICE_TO_CONTROLLER_NAVDATA_ID;
        networkConfiguration->deviceToControllerARStreamData = MPP_DEVICE_TO_CONTROLLER_VIDEO_DATA_ID;
        networkConfiguration->deviceToControllerARStreamAudioData = -1;
        networkConfiguration->deviceToControllerARStreamAudioAck = -1;

        networkConfiguration->hasVideo = 1;
        networkConfiguration->streamType = ARDISCOVERY_STREAM_STARTSTOP_ARCOMMANDS;
        networkConfiguration->startCommand = &SkyController2StartStreamingGenerator;
        networkConfiguration->stopCommand = &SkyController2StopStreamingGenerator;

        networkConfiguration->controllerToDeviceParams = c2dParams;
        networkConfiguration->numberOfControllerToDeviceParam = numC2dParams;

        networkConfiguration->deviceToControllerParams = d2cParams;
        networkConfiguration->numberOfDeviceToControllerParam = numD2cParams;

        networkConfiguration->pingDelayMs = 0;

        networkConfiguration->numberOfDeviceToControllerCommandsBufferIds = numOfCommandBufferIds;
        networkConfiguration->deviceToControllerCommandsBufferIds = commandBufferIds;
    }

    return error;
#else
    return ARDISCOVERY_ERROR;
#endif
}

static void device_conn_resp_cb(uint32_t status, const char* json, void *userdata)
{
    // -- Connection callback to receive the Json --

    // local declarations
    ARDISCOVERY_Device_t *device = userdata;
    ARDISCOVERY_DEVICE_USB_t *specificUsbParam = NULL;

    json_object *jsonObj = NULL;

    if ((device == NULL) ||
        (device->specificParameters == NULL))
    {
        /* Oops ? Should never happen, but in this case don't go further ! */
        return;
    }

    specificUsbParam = device->specificParameters;
    if (json && specificUsbParam->receiveJsonCallback) {
        jsonObj = json_tokener_parse ((char *)json);
        if (!is_error (jsonObj)) {
            specificUsbParam->receiveJsonCallback(jsonObj, specificUsbParam->jsonCallbacksCustomData);
        }
        json_object_put(jsonObj);
    }

    specificUsbParam->connectionStatus = status;
    ARSAL_Sem_Post(&specificUsbParam->sem);
}


/*************************
 * local Implementation
 *************************/

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_DiscoveryConnect (ARDISCOVERY_Device_t *device)
{
#ifdef BUILD_LIBMUX
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_DEVICE_USB_t *specificUsbParam = NULL;
    struct MuxConnectionCtx *conn_ctx = NULL;

    // check parameters
    if ((device == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_USBMUX) ||
        (device->specificParameters == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing

    if (error == ARDISCOVERY_OK)
    {
        // Cast of device->specificParameters
        specificUsbParam = (ARDISCOVERY_DEVICE_USB_t *)device->specificParameters;

        // New Discovery Connection
        conn_ctx = ARDiscovery_MuxConnection_new(specificUsbParam->mux, &device_conn_resp_cb, device);
    }

    if (error == ARDISCOVERY_OK)
    {
        json_object *jsonObj = json_object_new_object();
        json_object *json_name, *json_type;
        char *name = NULL;
        char *type = NULL;
        char *serial = "";
        char *json = NULL;

        // sending Json callback
        if (specificUsbParam->sendJsonCallback != NULL)
        {
            error = specificUsbParam->sendJsonCallback (jsonObj, specificUsbParam->jsonCallbacksCustomData);
            if (error != ARDISCOVERY_OK)
                goto conn_end;
            json_object_object_get_ex(jsonObj, ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY, &json_name);
            json_object_get(json_name);
            json_object_object_get_ex(jsonObj, ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY, &json_type);
            json_object_get(json_type);
            json_object_object_del(jsonObj, ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY);
            json_object_object_del(jsonObj, ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY);
            json = strdup(json_object_to_json_string(jsonObj));
        } else {
            json_name = NULL;
            json_type = NULL;
            json = strdup("");
        }
        if (json_name) {
            name = strdup(json_object_get_string(json_name));
            json_object_put(json_name);
        } else {
            name = strdup("generic_device");
        }
        if (json_type) {
            type = strdup(json_object_get_string(json_type));
            json_object_put(json_type);
        } else {
            type = strdup("arsdk_client");
        }

        error = ARDiscovery_MuxConnection_sendConnReq(conn_ctx, name, type, serial, json);
        if (error != ARDISCOVERY_OK)
            goto conn_end;

        /* wait for conn end */
        ARSAL_Sem_Wait(&specificUsbParam->sem);

    conn_end:
        json_object_put(jsonObj);
        free(json);
        free(name);
        free(type);
    }

    // Cleanup
    ARDiscovery_MuxConnection_dispose(conn_ctx);

    return error;
#else
    return ARDISCOVERY_ERROR;
#endif
}

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_GetMux(ARDISCOVERY_Device_t *device, struct mux_ctx **mux)
{
#ifdef BUILD_LIBMUX
    ARDISCOVERY_DEVICE_USB_t *specificUsbParam = NULL;

    // check parameters
    if ((device == NULL) || (mux == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_USBMUX) ||
        (device->specificParameters == NULL))
    {
        return ARDISCOVERY_ERROR_BAD_PARAMETER;
    }

    specificUsbParam = (ARDISCOVERY_DEVICE_USB_t *)device->specificParameters;

    (*mux) = specificUsbParam->mux;

    return ARDISCOVERY_OK;
#else
    return ARDISCOVERY_ERROR;
#endif
}
