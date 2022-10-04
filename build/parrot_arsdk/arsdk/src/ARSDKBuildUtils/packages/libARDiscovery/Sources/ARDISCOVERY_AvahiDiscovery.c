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
#include <stdio.h>
#include <stdlib.h>

#include <libARSAL/ARSAL_Print.h>

#include <libARDiscovery/ARDISCOVERY_AvahiDiscovery.h>
#include "ARDISCOVERY_AvahiDiscovery.h"

#define __ARDISCOVERY_AVAHIDISCOVERY_TAG__ "ARDISCOVERY_AvahiDiscovery"

#define ERR(...)    ARSAL_PRINT(ARSAL_PRINT_ERROR, __ARDISCOVERY_AVAHIDISCOVERY_TAG__, __VA_ARGS__)
#define SAY(...)    ARSAL_PRINT(ARSAL_PRINT_WARNING, __ARDISCOVERY_AVAHIDISCOVERY_TAG__, __VA_ARGS__)

/*
 * Private header
 */

/**
 * @brief Build final name
 * @return Pointer to name
 */
static char* ARDISCOVERY_AvahiDiscovery_BuildName(void);

/**
 * @brief Create service to be published
 * @param[in] c Avahi client
 * @param[in] serviceData service data
 * @return error during execution
 */
static eARDISCOVERY_ERROR ARDISCOVERY_AvahiDiscovery_CreateService(AvahiClient *c, ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData);

/**
 * @brief Entry group callback
 * @param[in] g Entry group
 * @param[in] state service data
 * @param[in] userData service data
 */
static void ARDISCOVERY_AvahiDiscovery_EntryGroupCb(AvahiEntryGroup* g, AvahiEntryGroupState state, void* userdata);

/**
 * @brief Client callback (publishing side)
 * @param[in] c Avahi client
 * @param[in] state Client state
 * @param[in] userData service data
 */
static void ARDISCOVERY_AvahiDiscovery_Publisher_ClientCb(AvahiClient* c, AvahiClientState state, void* userdata);

/**
 * @brief Client callback (browsing side)
 * @param[in] c Avahi client
 * @param[in] state Client state
 * @param[in] userData service data
 */
static void ARDISCOVERY_AvahiDiscovery_Browser_ClientCb(AvahiClient* c, AvahiClientState state, void* userdata);

/**
 * @brief Browser callback
 */
static void ARDISCOVERY_AvahiDiscovery_Browser_BrowseCb(AvahiServiceBrowser *b, AvahiIfIndex interface, AvahiProtocol protocol, AvahiBrowserEvent event,
        const char *name, const char *type, const char *domain, AVAHI_GCC_UNUSED AvahiLookupResultFlags flags, void* userdata);

/**
 * @brief Resolver callback
 */
static void ARDISCOVERY_AvahiDiscovery_Browser_ResolveCb(AvahiServiceResolver *r, AVAHI_GCC_UNUSED AvahiIfIndex interface, AVAHI_GCC_UNUSED AvahiProtocol protocol,
        AvahiResolverEvent event, const char *name, const char *type, const char *domain, const char *host_name, const AvahiAddress *address, uint16_t port, AvahiStringList *txt,
        AvahiLookupResultFlags flags, void* userdata);

/*
 * Publisher
 */

ARDISCOVERY_AvahiDiscovery_PublisherData_t* ARDISCOVERY_AvahiDiscovery_Publisher_New(char* serviceName, char* serviceType, uint32_t publishedPort, char *serviceJsonData, eARDISCOVERY_ERROR* errorPtr)
{
    /*
     * Create and initialize discovery data
     */
    ARDISCOVERY_AvahiDiscovery_PublisherData_t *serviceData = NULL;
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    if (serviceName == NULL || serviceType == NULL)
    {
        ERR("Null parameter");
        error = ARDISCOVERY_ERROR;
    }

    if (error == ARDISCOVERY_OK)
    {
        serviceData = malloc(sizeof(ARDISCOVERY_AvahiDiscovery_PublisherData_t));
        if (serviceData != NULL)
        {
            /* Init Avahi data */
            serviceData->client = NULL;
            serviceData->entryGroup = NULL;
            serviceData->simplePoll = NULL;
            serviceData->devicePort = publishedPort;

            /* Set Service Type */
            if (error == ARDISCOVERY_OK)
            {
                serviceData->serviceType = malloc(sizeof(uint8_t) * ARDISCOVERY_AVAHIDISCOVERY_SERVICETYPE_SIZE);
                if (serviceData->serviceType != NULL)
                {
                    strcpy(serviceData->serviceType, serviceType);
                }
                else
                {
                    error = ARDISCOVERY_ERROR_ALLOC;
                }
            }

            /* Set Service Name */
            if (error == ARDISCOVERY_OK)
            {
                serviceData->serviceName = malloc(sizeof(uint8_t) * ARDISCOVERY_AVAHIDISCOVERY_SERVICENAME_SIZE);
                if (serviceData->serviceName != NULL)
                {
                    strcpy(serviceData->serviceName, serviceName);
                }
                else
                {
                    error = ARDISCOVERY_ERROR_ALLOC;
                }
            }

            /* Set Service Custom Data */
            if (error == ARDISCOVERY_OK)
            {
                if (serviceJsonData != NULL)
                {
                    serviceData->serviceJsonData = malloc(sizeof(uint8_t) * ARDISCOVERY_AVAHIDISCOVERY_SERVICEJSONDATA_SIZE);
                    if (serviceData->serviceJsonData != NULL)
                    {
                        strcpy(serviceData->serviceJsonData, serviceJsonData);
                    }
                    else
                    {
                        error = ARDISCOVERY_ERROR_ALLOC;
                    }
                }
            }
        }
    }

    /* Delete connection data if an error occurred */
    if (error != ARDISCOVERY_OK)
    {
        ERR("error: %s", ARDISCOVERY_Error_ToString (error));
        ARDISCOVERY_AvahiDiscovery_Publisher_Delete(&serviceData);
    }

    if (errorPtr != NULL)
    {
        *errorPtr = error;
    }

    return serviceData;
}

static char* ARDISCOVERY_AvahiDiscovery_BuildName(void)
{
    /*
     * Get hostname and build the final name
     */
    char hostname[HOST_NAME_MAX + 1]; /* POSIX hostname max length + the null terminating byte. */
    int error = gethostname(hostname, sizeof(hostname));
    if (error == 0)
    {
        hostname[sizeof(hostname) - 1] = '\0';
        return strdup((const char *)hostname);
    }
    return NULL;
}

static void ARDISCOVERY_AvahiDiscovery_EntryGroupCb(AvahiEntryGroup* g, AvahiEntryGroupState state, void* userdata)
{
    /*
     * Avahi entry group callback
     */
    ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData = (ARDISCOVERY_AvahiDiscovery_PublisherData_t*) userdata;

    if (g == NULL || serviceData == NULL)
    {
        ERR("Null parameter");
        return;
    }

    /* Called whenever the entry group state changes */
    switch (state)
    {
    case AVAHI_ENTRY_GROUP_ESTABLISHED:
    {
        /* The entry group has been established successfully */
        SAY("Service '%s' successfully established.", serviceData->serviceName);
        break;
    }
    case AVAHI_ENTRY_GROUP_COLLISION:
    {
        /* A service name collision happened. Let's pick a new name */
        char* n = avahi_alternative_service_name((const char *)serviceData->serviceName);
        avahi_free(serviceData->serviceName);
        serviceData->serviceName = n;

        ERR("Service name collision, renaming service to '%s'", serviceData->serviceName);

        /* And recreate the services */
        ARDISCOVERY_AvahiDiscovery_CreateService(serviceData->client, serviceData);
        break;
    }

    case AVAHI_ENTRY_GROUP_FAILURE:
    {
        ERR( "Entry group failure: %s", avahi_strerror(avahi_client_errno(avahi_entry_group_get_client(g))));

        /* Some kind of failure happened while we were registering our services */
        avahi_simple_poll_quit(serviceData->simplePoll);
        break;
    }
    case AVAHI_ENTRY_GROUP_UNCOMMITED:
    case AVAHI_ENTRY_GROUP_REGISTERING:
    default:
        break;
    }
}

static eARDISCOVERY_ERROR ARDISCOVERY_AvahiDiscovery_CreateService(AvahiClient* c, ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData)
{
    /*
     * Create Avahi service
     */
    int ret;
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    if (c == NULL || serviceData == NULL)
    {
        ERR("Null parameter");
        error = ARDISCOVERY_ERROR;
    }

    if (error == ARDISCOVERY_OK)
    {
        /* If this is the first time we're called, let's create a new entry group */
        if (!serviceData->entryGroup)
        {
            serviceData->entryGroup = avahi_entry_group_new(c, ARDISCOVERY_AvahiDiscovery_EntryGroupCb, (void*) serviceData);
            if (!serviceData->entryGroup)
            {
                error = ARDISCOVERY_ERROR_ENTRY_GROUP;
            }
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Add the service for AR.Drone */
        ret = avahi_entry_group_add_service(serviceData->entryGroup, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC, 0, (const char *)serviceData->serviceName,
                                            (const char *)serviceData->serviceType, ARDISCOVERY_AVAHIDISCOVERY_DEFAULT_NETWORK, NULL, serviceData->devicePort, NULL, NULL);
        if (ret < 0)
        {
            error = ARDISCOVERY_ERROR_ADD_SERVICE;
        }
    }

    /* Tell the server to register the service */
    if (error == ARDISCOVERY_OK)
    {
        ret = avahi_entry_group_commit(serviceData->entryGroup);
        if (ret < 0)
        {
            error = ARDISCOVERY_ERROR_GROUP_COMMIT;
        }
    }

    if (error != ARDISCOVERY_OK)
    {
        ERR("error: %s", ARDISCOVERY_Error_ToString (error));
        avahi_simple_poll_quit(serviceData->simplePoll);
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_AvahiDiscovery_ResetService(ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData)
{
    /*
     * Reset Avahi service
     */
    int ret;
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    if (serviceData == NULL)
    {
        ERR("Null parameter");
        error = ARDISCOVERY_ERROR;
    }

    if (serviceData->entryGroup == NULL)
    {
        ERR("Entry group null, reset required too soon");
        return error;
    }

    if (error == ARDISCOVERY_OK)
    {
        ret = avahi_entry_group_free(serviceData->entryGroup);
        if (ret < 0)
        {
            ERR("Entry group reset failed");
            error = ARDISCOVERY_ERROR;
        }
        serviceData->entryGroup = NULL;
    }

    if (error == ARDISCOVERY_OK)
    {
        error = ARDISCOVERY_AvahiDiscovery_CreateService(serviceData->client, serviceData);
    }

    return error;
}

static void ARDISCOVERY_AvahiDiscovery_Publisher_ClientCb(AvahiClient* c, AvahiClientState state, void* userdata)
{
    /*
     * Avahi client callback
     * Called whenever the client or server state changes
     */
    ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData = (ARDISCOVERY_AvahiDiscovery_PublisherData_t*) userdata;

    if (c == NULL || serviceData == NULL)
    {
        ERR("Null parameter");
        return;
    }

    switch (state)
    {
    case AVAHI_CLIENT_S_RUNNING:
    {
        /* The server has startup successfully and registered its host
         * name on the network, so it's time to create our services */
        if (!serviceData->entryGroup)
        {
            ARDISCOVERY_AvahiDiscovery_CreateService(c, serviceData);
        }
        break;
    }
    case AVAHI_CLIENT_FAILURE:
    {
        ERR("Client failure: %s", avahi_strerror(avahi_client_errno(c)));
        avahi_simple_poll_quit(serviceData->simplePoll);
        break;
    }
    case AVAHI_CLIENT_S_COLLISION:
    {
        /* Let's drop our registered services. When the server is back
         * in AVAHI_SERVER_RUNNING state we will register them
         * again with the new host name. */
    }
    case AVAHI_CLIENT_S_REGISTERING:
    {
        /* The server records are now being established. This
         * might be caused by a host name change. We need to wait
         * for our own records to register until the host name is
         * properly esatblished. */
        if (serviceData->entryGroup)
        {
            avahi_entry_group_reset(serviceData->entryGroup);
        }
        break;
    }
    case AVAHI_CLIENT_CONNECTING:
    default:
        break;
    }
}

eARDISCOVERY_ERROR ARDISCOVERY_AvahiDiscovery_SetServiceName(ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData, const char *serviceName)
{
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    char *newName;

    if (serviceData == NULL)
    {
        ERR("Null parameter");
        error = ARDISCOVERY_ERROR;
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Use device hostname if no service name was specified. */
        if (serviceName == NULL)
        {
            newName = ARDISCOVERY_AvahiDiscovery_BuildName();
            if (newName == NULL)
            {
                error = ARDISCOVERY_ERROR_BUILD_NAME;
            }
        }
        else
        {
            const size_t maxsize = ARDISCOVERY_AVAHIDISCOVERY_SERVICENAME_SIZE;
            newName = malloc(maxsize);
            if (newName != NULL)
            {
                strncpy(newName, serviceName, maxsize - 1);
                newName[maxsize - 1] = '\0';
            }
            else
            {
                error = ARDISCOVERY_ERROR_ALLOC;
            }
        }
        if (error == ARDISCOVERY_OK)
        {
            if (serviceData->serviceName != NULL) {
                free(serviceData->serviceName);
            }
            serviceData->serviceName = newName;
        }
    }

    return error;
}

void ARDISCOVERY_AvahiDiscovery_Publish(ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData)
{
    int avahiError;
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    if (serviceData == NULL)
    {
        ERR("Null parameter");
        error = ARDISCOVERY_ERROR;
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Allocate main loop object */
        serviceData->simplePoll = avahi_simple_poll_new();
        if (serviceData->simplePoll == NULL)
        {
            error = ARDISCOVERY_ERROR_SIMPLE_POLL;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Build service name */
        serviceData->serviceName = ARDISCOVERY_AvahiDiscovery_BuildName();
        if (serviceData->serviceName == NULL)
        {
            error = ARDISCOVERY_ERROR_BUILD_NAME;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Allocate a new client */
        serviceData->client = avahi_client_new(avahi_simple_poll_get(serviceData->simplePoll), 0, ARDISCOVERY_AvahiDiscovery_Publisher_ClientCb, serviceData, &avahiError);
        if (serviceData->client == NULL)
        {
            ERR("Failed to create client: %s\n", avahi_strerror(avahiError));
            error = ARDISCOVERY_ERROR_CLIENT;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Run the main loop */
        avahi_simple_poll_loop(serviceData->simplePoll);
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Main loop exited, cleanup */
        if (serviceData)
        {
            if (serviceData->client)
            {
                avahi_client_free(serviceData->client);
                serviceData->client = NULL;
            }
            if (serviceData->simplePoll)
            {
                avahi_simple_poll_free(serviceData->simplePoll);
                serviceData->simplePoll = NULL;
            }
            avahi_free(serviceData->serviceName);
            serviceData->serviceName = NULL;
        }
    }

    if (error != ARDISCOVERY_OK)
    {
        ERR("error: %s", ARDISCOVERY_Error_ToString (error));
    }
}

void ARDISCOVERY_AvahiDiscovery_StopPublishing(ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData)
{
    /*
     * Stop publishing service
     */
    if (serviceData == NULL)
    {
        return;
    }

    if (serviceData->simplePoll != NULL)
    {
        avahi_simple_poll_quit(serviceData->simplePoll);
    }
}

void ARDISCOVERY_AvahiDiscovery_Publisher_Delete(ARDISCOVERY_AvahiDiscovery_PublisherData_t** serviceDataPtrAddr)
{
    /*
     * Free discovery data
     */
    ARDISCOVERY_AvahiDiscovery_PublisherData_t *serviceDataPtr = NULL;

    if (serviceDataPtrAddr != NULL)
    {
        serviceDataPtr = *serviceDataPtrAddr;

        if (serviceDataPtr != NULL)
        {
            ARDISCOVERY_AvahiDiscovery_StopPublishing(serviceDataPtr);

            if (serviceDataPtr->serviceName)
            {
                free(serviceDataPtr->serviceName);
                serviceDataPtr->serviceName = NULL;
            }
            if (serviceDataPtr->serviceType)
            {
                free(serviceDataPtr->serviceType);
                serviceDataPtr->serviceType = NULL;
            }
            if (serviceDataPtr->serviceJsonData)
            {
                free(serviceDataPtr->serviceJsonData);
                serviceDataPtr->serviceJsonData = NULL;
            }

            free(serviceDataPtr);
            serviceDataPtr = NULL;
        }

        *serviceDataPtrAddr = NULL;
    }
}

/*
 * Browser
 */

ARDISCOVERY_AvahiDiscovery_BrowserData_t* ARDISCOVERY_AvahiDiscovery_Browser_New(ARDISCOVERY_AvahiDiscovery_Browser_Callback_t callback, void* customData, char** serviceTypes, uint8_t serviceTypesNb, eARDISCOVERY_ERROR* errorPtr)
{
    /*
     * Initialize Browsing related data
     */

    ARDISCOVERY_AvahiDiscovery_BrowserData_t *browserData = NULL;
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    int i;

    if (callback == NULL)
    {
        ERR("Null parameter");
        error = ARDISCOVERY_ERROR;
    }

    if (error == ARDISCOVERY_OK)
    {
        browserData = malloc(sizeof(ARDISCOVERY_AvahiDiscovery_BrowserData_t));
        if (browserData != NULL)
        {
            /* Init Avahi data */
            browserData->simplePoll = NULL;
            browserData->callback = callback;
            browserData->customData = customData;

            /* Allocate pointer to services */
            browserData->serviceTypes = malloc(sizeof(char*) * serviceTypesNb);
            for (i = 0; i < serviceTypesNb; i++)
            {
                browserData->serviceTypes[i] = strdup(serviceTypes[i]);
            }
            browserData->serviceTypesNb = serviceTypesNb;
        }
    }

    /* Delete browser data if an error occurred */
    if (error != ARDISCOVERY_OK)
    {
        ERR("error: %s", ARDISCOVERY_Error_ToString (error));
        ARDISCOVERY_AvahiDiscovery_Browser_Delete(&browserData);
    }

    if (errorPtr != NULL)
    {
        *errorPtr = error;
    }

    return browserData;
}

void ARDISCOVERY_AvahiDiscovery_Browse(ARDISCOVERY_AvahiDiscovery_BrowserData_t* browserData)
{
    /*
     * Start browsing for services
     */

    int avahiError;
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    int i;

    if (browserData == NULL)
    {
        ERR("Null parameter");
        error = ARDISCOVERY_ERROR;
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Allocate main loop object */
        browserData->simplePoll = avahi_simple_poll_new();
        if (browserData->simplePoll == NULL)
        {
            error = ARDISCOVERY_ERROR_SIMPLE_POLL;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Create each service browser and associated client */
        for (i = 0 ; i < browserData->serviceTypesNb ; i++)
        {
            browserData->clients[i] = avahi_client_new(avahi_simple_poll_get(browserData->simplePoll), 0, ARDISCOVERY_AvahiDiscovery_Browser_ClientCb, browserData, &avahiError);
            if (browserData->clients[i] == NULL)
            {
                ERR("Failed to create client #%d: %s\n", i+1, avahi_strerror(avahiError));
                error = ARDISCOVERY_ERROR_CLIENT;
            }

            browserData->serviceBrowsers[i] = avahi_service_browser_new(browserData->clients[i], AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC,
                    *(browserData->serviceTypes+i), NULL, 0, ARDISCOVERY_AvahiDiscovery_Browser_BrowseCb, browserData);
            if (!browserData->serviceBrowsers[i])
            {
                ERR("Failed to create service browser #%d (%s): %s\n", i+1, *(browserData->serviceTypes+i), avahi_strerror(avahiError));
                error = ARDISCOVERY_ERROR_BROWSER_NEW;
            }
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Run the main loop */
        avahi_simple_poll_loop(browserData->simplePoll);
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Main loop exited, cleanup */
        if (browserData)
        {
            for (i = 0 ; i < browserData->serviceTypesNb ; i++)
            {
                if (browserData->serviceBrowsers[i])
                {
                    avahi_service_browser_free(browserData->serviceBrowsers[i]);
                    browserData->serviceBrowsers[i] = NULL;
                }
            }
            for (i = 0 ; i < browserData->serviceTypesNb ; i++)
            {
                if (browserData->clients[i])
                {
                    avahi_client_free(browserData->clients[i]);
                    browserData->clients[i] = NULL;
                }
            }
            if (browserData->simplePoll)
            {
                avahi_simple_poll_free(browserData->simplePoll);
            }
        }
    }

    if (error != ARDISCOVERY_OK)
    {
        ERR("error: %s", ARDISCOVERY_Error_ToString (error));
    }
}

static void ARDISCOVERY_AvahiDiscovery_Browser_ClientCb(AvahiClient* c, AvahiClientState state, void* userdata)
{
    /*
     * Avahi client callback
     * Called whenever the client or server state changes
     */

    ARDISCOVERY_AvahiDiscovery_BrowserData_t* browserData = (ARDISCOVERY_AvahiDiscovery_BrowserData_t*) userdata;

    if (c == NULL || browserData == NULL)
    {
        ERR("Null parameter");
        return;
    }

    switch (state)
    {
        case AVAHI_CLIENT_FAILURE:
        {
            ERR("Client failure: %s", avahi_strerror(avahi_client_errno(c)));
            avahi_simple_poll_quit(browserData->simplePoll);
            break;
        }
        default:
            break;
    }
}

static AvahiClient* ARDISCOVERY_AvahiDiscovery_Browser_FindClient(AvahiServiceBrowser *b, ARDISCOVERY_AvahiDiscovery_BrowserData_t* bdt)
{
    /*
     * Find Avahi client from its corresponding index and browser
     */

    int i;

    for (i = 0 ; i < bdt->serviceTypesNb ; i++)
    {
        if (bdt->serviceBrowsers[i] == b)
        {
            return bdt->clients[i];
        }
    }

    return NULL;
}

static void ARDISCOVERY_AvahiDiscovery_Browser_BrowseCb(AvahiServiceBrowser *b, AvahiIfIndex interface, AvahiProtocol protocol, AvahiBrowserEvent event, const char *name, const char *type, const char *domain, AVAHI_GCC_UNUSED AvahiLookupResultFlags flags, void* userdata)
{
    /*
     * Called whenever a new services becomes available on the LAN or is removed from the LAN
     */

    ARDISCOVERY_AvahiDiscovery_BrowserData_t* browserData = (ARDISCOVERY_AvahiDiscovery_BrowserData_t*) userdata;
    AvahiServiceResolver *r = NULL;

    AvahiClient *c = ARDISCOVERY_AvahiDiscovery_Browser_FindClient(b, browserData);

    if (b == NULL || browserData == NULL || c == NULL)
    {
        ERR("Null parameter");
        return;
    }

    switch (event)
    {
        case AVAHI_BROWSER_FAILURE:
        {
            ERR("Browser failure: %s", avahi_strerror(avahi_client_errno(avahi_service_browser_get_client(b))));
            avahi_simple_poll_quit(browserData->simplePoll);
            break;
        }
        case AVAHI_BROWSER_NEW:
        {
            /* Resolve service and call callback in resolver */
            r = avahi_service_resolver_new(c, interface, protocol, name, type, domain, AVAHI_PROTO_UNSPEC, 0,
                    ARDISCOVERY_AvahiDiscovery_Browser_ResolveCb, browserData);
            if (r == NULL)
            {
                ERR("Failed to resolve service '%s': %s\n", name, avahi_strerror(avahi_client_errno(c)));
            }
            break;
        }
        case AVAHI_BROWSER_REMOVE:
        {
            /* Call upper layer callback for service removal */
            browserData->callback(browserData->customData, 0, name, type, NULL, 0);
            break;
        }
        case AVAHI_BROWSER_ALL_FOR_NOW:
        case AVAHI_BROWSER_CACHE_EXHAUSTED:
        default:
            break;
    }
}

static void ARDISCOVERY_AvahiDiscovery_Browser_ResolveCb(AvahiServiceResolver *r, AVAHI_GCC_UNUSED AvahiIfIndex interface, AVAHI_GCC_UNUSED AvahiProtocol protocol, AvahiResolverEvent event, const char *name, const char *type, const char *domain, const char *host_name, const AvahiAddress *address, uint16_t port, AvahiStringList *txt, AvahiLookupResultFlags flags, void* userdata)
{
    /*
     * Called whenever a service has been resolved successfully or timed out
     */

    ARDISCOVERY_AvahiDiscovery_BrowserData_t* browserData = (ARDISCOVERY_AvahiDiscovery_BrowserData_t*) userdata;

    if (r == NULL)
    {
        ERR("Null parameter");
        return;
    }

    switch (event)
    {
        case AVAHI_RESOLVER_FAILURE:
        {
            ERR("Failed to resolve service '%s' of type '%s' in domain '%s': %s\n", name, type, domain, avahi_strerror(avahi_client_errno(avahi_service_resolver_get_client(r))));
            break;
        }
        case AVAHI_RESOLVER_FOUND:
        {
            char a[AVAHI_ADDRESS_STR_MAX];

            /* Concert Avahi object to readable ip address */
            avahi_address_snprint(a, sizeof(a), address);
            /* Call to say we found a service */
            browserData->callback(browserData->customData, 1, name, type, a, port);
        }
    }

    avahi_service_resolver_free(r);
}

void ARDISCOVERY_AvahiDiscovery_StopBrowsing(ARDISCOVERY_AvahiDiscovery_BrowserData_t* browserData)
{
    /*
     * Stop browsing services
     */

    if (browserData == NULL)
    {
        return;
    }

    avahi_simple_poll_quit(browserData->simplePoll);
}

void ARDISCOVERY_AvahiDiscovery_Browser_Delete(ARDISCOVERY_AvahiDiscovery_BrowserData_t** browserDataPtrAddr)
{
    /*
     * Free browser data
     */

    ARDISCOVERY_AvahiDiscovery_BrowserData_t *browserDataPtr = NULL;
    int i;

    if (browserDataPtrAddr != NULL)
    {
        browserDataPtr = *browserDataPtrAddr;

        if (browserDataPtr != NULL)
        {
            ARDISCOVERY_AvahiDiscovery_StopBrowsing(browserDataPtr);

            for (i = 0; i < browserDataPtr->serviceTypesNb; i++)
            {
                free(browserDataPtr->serviceTypes[i]);
            }
            free(browserDataPtr->serviceTypes);

            free(browserDataPtr);
            browserDataPtr = NULL;
        }

        *browserDataPtrAddr = NULL;
    }
}
