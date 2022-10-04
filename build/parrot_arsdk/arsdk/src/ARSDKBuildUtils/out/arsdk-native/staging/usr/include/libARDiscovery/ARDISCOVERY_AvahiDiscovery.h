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
#ifndef _ARDISCOVERY_AVAHIDISCOVERY_H_
#define _ARDISCOVERY_AVAHIDISCOVERY_H_

#include <libARDiscovery/ARDISCOVERY_Error.h>

/**
 * @brief Default service parameters
 */
#define ARDISCOVERY_AVAHIDISCOVERY_DEFAULT_NETWORK "local"
#define ARDISCOVERY_AVAHIDISCOVERY_SERVICEJSONDATA_SIZE 128

#define ARDISCOVERY_AVAHIDISCOVERY_SERVICE_NB_MAX ARDISCOVERY_PRODUCT_MAX

/**
 * @brief Structure to allow data sharing across discovery process
 */
typedef struct ARDISCOVERY_AvahiDiscovery_PublisherData_t ARDISCOVERY_AvahiDiscovery_PublisherData_t;

/**
 * @brief Structure to allow data sharing across service browsing process
 */
typedef struct ARDISCOVERY_AvahiDiscovery_BrowserData_t ARDISCOVERY_AvahiDiscovery_BrowserData_t;

/**
 * @brief callback to notify whether a service of the subscribed type has appeared or disappeared
 * @param[in] state Service appeared (1) or disappeared (0)
 * @param[in] serviceName Service name
 * @param[in] serviceType Service type
 * @param[in] ipAddr IP address of the host for said service
 * @return error during callback execution
 */
typedef eARDISCOVERY_ERROR (*ARDISCOVERY_AvahiDiscovery_Browser_Callback_t) (void* custom, uint8_t state, const char* serviceName, const char* serviceType, const char* ipAddr, uint16_t port);

/**
 * @brief Initialize Publication related Avahi data
 * @param[in] serviceName Discovery service name
 * @param[in] serviceType Discovery service type
 * @param[in] errorPtr Error during execution
 * @return Pointer to allocated service data
 */
ARDISCOVERY_AvahiDiscovery_PublisherData_t* ARDISCOVERY_AvahiDiscovery_Publisher_New(char* serviceName, char* serviceType, uint32_t publishedPort, char *serviceJsonData, eARDISCOVERY_ERROR* errorPtr);

/**
 * @brief Start Avahi process of service advertisement
 * @param[in] serviceData Service data
 */
void ARDISCOVERY_AvahiDiscovery_Publish(ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData);

/**
 * @brief Reset the entry group then re-create the service
 * @param[in] c Avahi client
 * @param[in] serviceData service data
 * @return error during execution
 */
eARDISCOVERY_ERROR ARDISCOVERY_AvahiDiscovery_ResetService(ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData);

/*
 * @brief Change service name. No change will be seen until the service is reset.
 * @param[in] serviceData Service data
 * @param[in] serviceName New service name. NULL to use machine hostname.
 */
eARDISCOVERY_ERROR ARDISCOVERY_AvahiDiscovery_SetServiceName(ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData, const char *serviceName);

/**
 * @brief Stop Avahi process of service advertisement
 * @param[in] serviceData Service data
 */
void ARDISCOVERY_AvahiDiscovery_StopPublishing(ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData);

/*
 * @brief Free data structures
 * @param[in] serviceDataPtrAddr Pointer to Service data
 */
void ARDISCOVERY_AvahiDiscovery_Publisher_Delete(ARDISCOVERY_AvahiDiscovery_PublisherData_t** serviceDataPtrAddr);

/**
 * @brief Initialize Browsing related Avahi data
 * @param[in] callback Callback called whenever a service appears or disappears
 * @param[in] customData Data forwarded to callback
 * @param[in] serviceTypes Table of pointers to service type strings
 * @param[in] serviceTypesNb Number of service type strings in table
 * @param[in] errorPtr Error during execution
 * @return Pointer to allocated browser data
 */
ARDISCOVERY_AvahiDiscovery_BrowserData_t* ARDISCOVERY_AvahiDiscovery_Browser_New(ARDISCOVERY_AvahiDiscovery_Browser_Callback_t callback, void* customData, char** serviceTypes, uint8_t serviceTypesNb, eARDISCOVERY_ERROR* errorPtr);

/**
 * @brief Start Avahi process of service browsing
 * @param[in] browserData Browser data
 */
void ARDISCOVERY_AvahiDiscovery_Browse(ARDISCOVERY_AvahiDiscovery_BrowserData_t* browserData);

/**
 * @brief Stop Avahi process of service browsing
 * @param[in] browserData Browser data
 */
void ARDISCOVERY_AvahiDiscovery_StopBrowsing(ARDISCOVERY_AvahiDiscovery_BrowserData_t* browserData);

/*
 * @brief Free data structures
 * @param[in] browserDataPtrAddr Pointer to browser data
 */
void ARDISCOVERY_AvahiDiscovery_Browser_Delete(ARDISCOVERY_AvahiDiscovery_BrowserData_t** browserDataPtrAddr);

#endif /* _ARDISCOVERY_AVAHIDISCOVERY_H_ */
