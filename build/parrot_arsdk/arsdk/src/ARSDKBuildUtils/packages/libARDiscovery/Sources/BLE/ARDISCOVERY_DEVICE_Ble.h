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
 * @file ARDISCOVERY_DEVICE_Ble.h
 * @brief Discovery BLE Device contains the informations of a device discovered
 * @date 02/03/2015
 * @author maxime.maitre@parrot.com
 */

#ifndef _ARDISCOVERY_DEVICE_BLE_H_
#define _ARDISCOVERY_DEVICE_BLE_H_

#include <json-c/json.h>
#include <libARNetworkAL/ARNETWORKAL_Manager.h>
#include <libARNetworkAL/ARNETWORKAL_Error.h>
#include <libARNetwork/ARNETWORK_IOBufferParam.h>
#include <libARDiscovery/ARDISCOVERY_Error.h>
#include <libARDiscovery/ARDISCOVERY_Discovery.h>
#include <libARDiscovery/ARDISCOVERY_NetworkConfiguration.h>
#include <libARDiscovery/ARDISCOVERY_Device.h>

// RollingSpider buffer IDs

#define ROLLINGSPIDER_CONTROLLER_TO_DEVICE_NONACK_ID 10
#define ROLLINGSPIDER_CONTROLLER_TO_DEVICE_ACK_ID 11
#define ROLLINGSPIDER_CONTROLLER_TO_DEVICE_EMERGENCY_ID 12
#define ROLLINGSPIDER_DEVICE_TO_CONTROLLER_NAVDATA_ID ((ARNETWORKAL_MANAGER_BLE_ID_MAX /2) - 1)
#define ROLLINGSPIDER_DEVICE_TO_CONTROLLER_EVENT_ID ((ARNETWORKAL_MANAGER_BLE_ID_MAX /2) - 2)

/**
 * @brief specific parameters for Wifi Device
 */
typedef struct
{
    ARNETWORKAL_BLEDeviceManager_t deviceManager;
    ARNETWORKAL_BLEDevice_t device;
}ARDISCOVERY_DEVICE_BLE_t;

/**
 * @brief Create BLE SpecificParameters
 * @warning This function allocate memory.
 * @param device The Discovery Device to Initialize.
 * @return executing error.
 * @see ARDISCOVERY_DEVICE_Ble_DeleteSpecificParameters.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Ble_CreateSpecificParameters (ARDISCOVERY_Device_t *device, ARNETWORKAL_BLEDeviceManager_t bleDeviceManager, ARNETWORKAL_BLEDevice_t bleDevice);
/**
 * @brief Delete BLE SpecificParameters
 * @warning This function free memory.
 * @param device The Discovery Device to Initialize.
 * @return executing error.
 * @see ARDISCOVERY_DEVICE_Ble_CreateSpecificParameters.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Ble_DeleteSpecificParameters (ARDISCOVERY_Device_t *device);

/**
 * @brief Copy BLE specificParameters
 * @param deviceToCopy The Discovery Device to copy.
 * @param[out] error Executing error.
 * @return new specificParameters.
 */
void *ARDISCOVERY_DEVICE_Ble_GetCopyOfSpecificParameters (ARDISCOVERY_Device_t *deviceToCopy, eARDISCOVERY_ERROR *error);

/**
 * @brief Create a new networlAL adapted to the device.
 * @param device The Discovery Device.
 * @param[out] error Executing error.
 * @param[out] errorAL Executing networkAL error.
 * @return new networkAL.
 * @see ARDISCOVERY_DEVICE_Ble_DeleteARNetworkAL
 */
ARNETWORKAL_Manager_t *ARDISCOVERY_DEVICE_Ble_NewARNetworkAL (ARDISCOVERY_Device_t *device, eARDISCOVERY_ERROR *error, eARNETWORKAL_ERROR *errorAL);

/**
 * @brief Delete a networlAL create by ARDISCOVERY_DEVICE_Ble_NewARNetworkAL
 * @param device The Discovery Device.
 * @param networkAL The networkAL to delete.
 * @return executing error.
 * @see ARDISCOVERY_DEVICE_Ble_NewARNetworkAL
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Ble_DeleteARNetworkAL (ARDISCOVERY_Device_t *device, ARNETWORKAL_Manager_t **networkAL);

/**
 * @brief Initilize network Configuration adapted to a Rolling Spider.
 * @param device The Discovery Device. Must be a Jumping Sumo Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Ble_InitRollingSpiderNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Gets the BLEDeviceManager associated with the given device.
 * @param device The Discovery Device.
 * @param[out] manager Pointer where the BLEDeviceManager will be stored
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_Device_Ble_GetManager(ARDISCOVERY_Device_t *device, ARNETWORKAL_BLEDeviceManager_t **manager);

/**
 * @brief Gets the BLEDevice associated with the given device.
 * @param device The Discovery Device.
 * @param[out] bleDevice Pointer where the BLEDevice will be stored
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_Device_Ble_GetDevice(ARDISCOVERY_Device_t *device, ARNETWORKAL_BLEDevice_t **bleDevice);

#endif // _ARDISCOVERY_DEVICE_BLE_H_
