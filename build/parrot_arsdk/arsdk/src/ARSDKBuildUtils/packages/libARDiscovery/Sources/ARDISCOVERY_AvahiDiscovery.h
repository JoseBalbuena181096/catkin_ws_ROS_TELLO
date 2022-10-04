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
#ifndef _ARDISCOVERY_AVAHIDISCOVERY_PRIVATE_H_
#define _ARDISCOVERY_AVAHIDISCOVERY_PRIVATE_H_

#include <avahi-client/client.h>
#include <avahi-client/publish.h>
#include <avahi-client/lookup.h>

#include <avahi-common/alternative.h>
#include <avahi-common/simple-watch.h>
#include <avahi-common/malloc.h>
#include <avahi-common/error.h>

#include <libARDiscovery/ARDISCOVERY_AvahiDiscovery.h>
#include <libARDiscovery/ARDISCOVERY_Discovery.h>

/**
 * @brief Service strings lengths
 */
#define ARDISCOVERY_AVAHIDISCOVERY_SERVICENAME_SIZE 64
#define ARDISCOVERY_AVAHIDISCOVERY_SERVICETYPE_SIZE 64

/**
 * @brief structure to allow service data sharing across publishing process
 */
struct ARDISCOVERY_AvahiDiscovery_PublisherData_t
{
    char* serviceName;              // Specific to each device ("ARDrone_21452365")
    char* serviceType;              // Specific to each platform ("_ardrone3._ucp")
    uint32_t devicePort;            // Port advertised by device
    AvahiClient *client;            // Avahi client
    AvahiEntryGroup *entryGroup;    // Avahi entry group
    AvahiSimplePoll *simplePoll;    // Avahi simple poll
    char* serviceJsonData;          // Custom data
};

/**
 * @brief structure to allow service data sharing across browsing process
 */
struct ARDISCOVERY_AvahiDiscovery_BrowserData_t
{
    char** serviceTypes;         // Service types to browse for
    uint8_t serviceTypesNb;         // Number of service types to browse for
    AvahiServiceBrowser* serviceBrowsers[ARDISCOVERY_AVAHIDISCOVERY_SERVICE_NB_MAX]; // Avahi service browsers
    AvahiClient *clients[ARDISCOVERY_AVAHIDISCOVERY_SERVICE_NB_MAX];            // Avahi client
    AvahiSimplePoll *simplePoll;    // Avahi simple poll
    ARDISCOVERY_AvahiDiscovery_Browser_Callback_t callback; // Service browsing callback
    void* customData;               // Custom data to forward to callback
};

#endif /* _ARDISCOVERY_AVAHIDISCOVERY_PRIVATE_H_ */
