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
 * @file ARDISCOVERY_DEVICE_Usb.h
 * @brief Discovery USB Device contains the informations of a device discovered
 * @date 02/03/2015
 * @author maxime.maitre@parrot.com
 */

#ifndef _ARDISCOVERY_DEVICE_USB_H_
#define _ARDISCOVERY_DEVICE_USB_H_

#include <json-c/json.h>
#include <libARSAL/ARSAL_Sem.h>
#include <libARNetworkAL/ARNETWORKAL_Manager.h>
#include <libARNetworkAL/ARNETWORKAL_Error.h>
#include <libARNetwork/ARNETWORK_IOBufferParam.h>
#include <libARDiscovery/ARDISCOVERY_Error.h>
#include <libARDiscovery/ARDISCOVERY_Discovery.h>
#include <libARDiscovery/ARDISCOVERY_NetworkConfiguration.h>
#include <libARDiscovery/ARDISCOVERY_Device.h>

struct mux_ctx;

/**
 * @brief specific parameters for Usb Device
 */
typedef struct
{
    // mux
    struct mux_ctx *mux;

    // Parameters sent by discovery Json :
    ARDISCOVERY_Device_ConnectionJsonCallback_t sendJsonCallback; //TODO must be not usb specific
    ARDISCOVERY_Device_ConnectionJsonCallback_t receiveJsonCallback; //TODO must be not usb specific
    void *jsonCallbacksCustomData; //TODO must be not usb specific

    // Parameters received by discovery Json :
    ARSAL_Sem_t sem;
    eARDISCOVERY_ERROR connectionStatus;

}ARDISCOVERY_DEVICE_USB_t;

/**
 * @brief Create usb SpecificParameters
 * @warning This function allocate memory.
 * @param device The Discovery Device to Initialize.
 * @return executing error.
 * @see ARDISCOVERY_DEVICE_Usb_DeleteSpecificParameters.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_CreateSpecificParameters (ARDISCOVERY_Device_t *device, struct mux_ctx *mux);

/**
 * @brief Delete usb SpecificParameters
 * @warning This function free memory.
 * @param device The Discovery Device to Initialize.
 * @return executing error.
 * @see ARDISCOVERY_DEVICE_Usb_CreateSpecificParameters.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_DeleteSpecificParameters (ARDISCOVERY_Device_t *device);

/**
 * @brief Copy usb specificParameters
 * @param deviceToCopy The Discovery Device to copy.
 * @param[out] error Executing error.
 * @return new specificParameters.
 */
void *ARDISCOVERY_DEVICE_Usb_GetCopyOfSpecificParameters (ARDISCOVERY_Device_t *deviceToCopy, eARDISCOVERY_ERROR *error);

/**
 * @brief Add connection callbacks to the device
 * @param device The device.
 * @param sendJsonCallback The callback which will be called before sending the json to the device.
 * @param receiveJsonCallback The callback which will be called after receiving the json from the device.
 * @param customData additional parameter passed to previous callbacks.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_AddConnectionCallbacks (ARDISCOVERY_Device_t *device,
 ARDISCOVERY_Device_ConnectionJsonCallback_t sendJsonCallback, ARDISCOVERY_Device_ConnectionJsonCallback_t receiveJsonCallback, void *customData);

/**
 * @brief Create a new networlAL adapted to the device.
 * @param device The Discovery Device.
 * @param[out] error Executing error.
 * @param[out] errorAL Executing networkAL error.
 * @return new networkAL.
 * @see ARDISCOVERY_DEVICE_Usb_DeleteARNetworkAL
 */
ARNETWORKAL_Manager_t *ARDISCOVERY_DEVICE_Usb_NewARNetworkAL (ARDISCOVERY_Device_t *device, eARDISCOVERY_ERROR *error, eARNETWORKAL_ERROR *errorAL);

/**
 * @brief Delete a networlAL create by ARDISCOVERY_DEVICE_Usb_NewARNetworkAL
 * @param device The Discovery Device.
 * @param networkAL The networkAL to delete.
 * @return executing error.
 * @see ARDISCOVERY_DEVICE_Usb_NewARNetworkAL
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_DeleteARNetworkAL (ARDISCOVERY_Device_t *device, ARNETWORKAL_Manager_t **networkAL);

/**
 * @brief Initilize network Configuration adapted to a SkyController2.
 * @param device The Discovery Device. Must be a SkyController2 Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_InitSkyController2NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);


/**
 * @brief Get the mux associated with an usb device.
 * @param device The Discovery Device.
 * @param[out] mux the mux pointer.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Usb_GetMux(ARDISCOVERY_Device_t *device, struct mux_ctx **mux);

#endif // _ARDISCOVERY_DEVICE_USB_H_
