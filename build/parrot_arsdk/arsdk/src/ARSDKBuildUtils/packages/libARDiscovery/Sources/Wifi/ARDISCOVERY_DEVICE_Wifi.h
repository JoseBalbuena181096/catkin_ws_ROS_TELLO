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
 * @file ARDISCOVERY_DEVICE_Wifi.h
 * @brief Discovery WIFI Device contains the informations of a device discovered
 * @date 02/03/2015
 * @author maxime.maitre@parrot.com
 */

#ifndef _ARDISCOVERY_DEVICE_WIFI_H_
#define _ARDISCOVERY_DEVICE_WIFI_H_

#include <json-c/json.h>
#include <libARNetworkAL/ARNETWORKAL_Manager.h>
#include <libARNetworkAL/ARNETWORKAL_Error.h>
#include <libARNetwork/ARNETWORK_IOBufferParam.h>
#include <libARDiscovery/ARDISCOVERY_Error.h>
#include <libARDiscovery/ARDISCOVERY_Discovery.h>
#include <libARDiscovery/ARDISCOVERY_NetworkConfiguration.h>
#include <libARDiscovery/ARDISCOVERY_Device.h>

/**
 * @brief specific parameters for Wifi Device
 */
typedef struct
{
    char *address;
    int dicoveryPort;
    // Parameters sended by discovery Json :
    int deviceToControllerPort;
    ARDISCOVERY_Device_ConnectionJsonCallback_t sendJsonCallback; //TODO must be not wifi specific
    ARDISCOVERY_Device_ConnectionJsonCallback_t receiveJsonCallback; //TODO must be not wifi specific
    void *jsonCallbacksCustomData; //TODO must be not wifi specific
    
    // Parameters received by discovery Json :
    int controllerToDevicePort;
    eARDISCOVERY_ERROR connectionStatus;
    int requested_qos_level;
    int qos_level;
    
}ARDISCOVERY_DEVICE_WIFI_t;

/**
 * @brief Create wifi SpecificParameters
 * @warning This function allocate memory.
 * @param device The Discovery Device to Initialize.
 * @return executing error.
 * @see ARDISCOVERY_DEVICE_Wifi_DeleteSpecificParameters.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_CreateSpecificParameters (ARDISCOVERY_Device_t *device, const char *name, const char *address, int port);


eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_SetDeviceToControllerPort (ARDISCOVERY_Device_t *device, int d2c_port);

/**
 * @brief Delete wifi SpecificParameters
 * @warning This function free memory.
 * @param device The Discovery Device to Initialize.
 * @return executing error.
 * @see ARDISCOVERY_DEVICE_Wifi_CreateSpecificParameters.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_DeleteSpecificParameters (ARDISCOVERY_Device_t *device);

/**
 * @brief Copy wifi specificParameters 
 * @param deviceToCopy The Discovery Device to copy.
 * @param[out] error Executing error.
 * @return new specificParameters.
 */
void *ARDISCOVERY_DEVICE_Wifi_GetCopyOfSpecificParameters (ARDISCOVERY_Device_t *deviceToCopy, eARDISCOVERY_ERROR *error);

eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_AddConnectionCallbacks (ARDISCOVERY_Device_t *device, ARDISCOVERY_Device_ConnectionJsonCallback_t sendJsonCallback, ARDISCOVERY_Device_ConnectionJsonCallback_t receiveJsonCallback, void *customData);

/**
 * @brief Create a new networlAL adapted to the device.
 * @param device The Discovery Device.
 * @param[out] error Executing error.
 * @param[out] errorAL Executing networkAL error.
 * @return new networkAL.
 * @see ARDISCOVERY_DEVICE_Wifi_DeleteARNetworkAL
 */
ARNETWORKAL_Manager_t *ARDISCOVERY_DEVICE_Wifi_NewARNetworkAL (ARDISCOVERY_Device_t *device, eARDISCOVERY_ERROR *error, eARNETWORKAL_ERROR *errorAL);

/**
 * @brief Delete a networlAL create by ARDISCOVERY_DEVICE_Wifi_NewARNetworkAL
 * @param device The Discovery Device.
 * @param networkAL The networkAL to delete.
 * @return executing error.
 * @see ARDISCOVERY_DEVICE_Wifi_NewARNetworkAL
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_DeleteARNetworkAL (ARDISCOVERY_Device_t *device, ARNETWORKAL_Manager_t **networkAL);

/**
 * @brief Get wifi IP address
 * @param device The Discovery Device.
 * @param[out] ipAddress ip address of the device
 * @param[in] length maximum length of the ipAddress output.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_GetIpAddress (ARDISCOVERY_Device_t *device, char *ipAddress, int length);

/**
 * @brief Set requested QoS level.
 * 0: No QoS, 1: QoS enabled (default)
 * @param device The Discovery Device.
 * @param [in] level The requested QoS level
 * @note Must be called before "NewARNetworkAL" to have any effect.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_SetQoSLevel (ARDISCOVERY_Device_t *device, int level);

/**
 * @brief Initilize network Configuration adapted to a BebopDrone.
 * @param device The Discovery Device. Must be a Bebop Drone Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitBebopNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Initilize network Configuration adapted to a SkyController.
 * @param device The Discovery Device. Must be a SkyController Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitSkyControllerNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Initilize network Configuration adapted to a Bebop 2.
 * @param device The Discovery Device. Must be a Bebop 2 Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitBebop2NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Initilize network Configuration adapted to a Unknownproduct_4.
 * @param device The Discovery Device. Must be a Unknownproduct_4 Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitUnknownproduct_4NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Initilize network Configuration adapted to an Unknownproduct_5.
 * @param device The Discovery Device. Must be a Unknownproduct_5 Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitUnknownproduct_5NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Initilize network Configuration adapted to a Chimera.
 * @param device The Discovery Device. Must be a Chimera Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitChimeraNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Initilize network Configuration adapted to a Mambo FPV.
 * @param device The Discovery Device. Must be a Mambo FPV Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitDelos3NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Initilize network Configuration adapted to a SkyController2.
 * @param device The Discovery Device. Must be a SkyController2/2P/NG Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitSkyController2NetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Initilize network Configuration adapted to a Jumping Sumo.
 * @param device The Discovery Device. Must be a Jumping Sumo Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitJumpingSumoNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Initilize network Configuration adapted to a Jumping Sumo Evo.
 * @param device The Discovery Device. Must be a Jumping Sumo Evo Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitJumpingSumoEvoNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Initilize network Configuration adapted to a Power Up.
 * @param device The Discovery Device. Must be a Power Up Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitPowerUpNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

/**
 * @brief Initilize network Configuration adapted to an Evinrude.
 * @param device The Discovery Device. Must be an Evinrude Device
 * @param[out] networkConfiguration The networkConfiguration to Initilize.
 * @return executing error.
 */
eARDISCOVERY_ERROR ARDISCOVERY_DEVICE_Wifi_InitEvinrudeNetworkConfiguration (ARDISCOVERY_Device_t *device, ARDISCOVERY_NetworkConfiguration_t *networkConfiguration);

#endif // _ARDISCOVERY_DEVICE_WIFI_H_
