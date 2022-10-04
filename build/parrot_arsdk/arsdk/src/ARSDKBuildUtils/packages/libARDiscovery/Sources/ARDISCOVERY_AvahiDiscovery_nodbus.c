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
#include "ARDISCOVERY_AvahiDiscovery_nodbus.h"

#define __ARDISCOVERY_AVAHIDISCOVERY_TAG__ "ARDISCOVERY_AvahiDiscovery"

#define ERR(...)    ARSAL_PRINT(ARSAL_PRINT_ERROR, __ARDISCOVERY_AVAHIDISCOVERY_TAG__, __VA_ARGS__)
#define SAY(...)    ARSAL_PRINT(ARSAL_PRINT_WARNING, __ARDISCOVERY_AVAHIDISCOVERY_TAG__, __VA_ARGS__)

static const char * ARDISCOVERY_CONFIG_FILE = "/etc/avahi/services/ardiscovery.service";

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
 * @param[in] serviceData service data
 * @return error during execution
 */
static eARDISCOVERY_ERROR ARDISCOVERY_AvahiDiscovery_CreateService(ARDISCOVERY_AvahiDiscovery_PublisherData_t *serviceData);

/*
 * Publisher
 */

ARDISCOVERY_AvahiDiscovery_PublisherData_t* ARDISCOVERY_AvahiDiscovery_Publisher_New(char *serviceName, char *serviceType, uint32_t publishedPort, char *serviceJsonData, eARDISCOVERY_ERROR *errorPtr)
{
    /*
     * Create and initialize discovery data
     */
    ARDISCOVERY_AvahiDiscovery_PublisherData_t *serviceData = NULL;
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    if (serviceType == NULL)
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
            if (error == ARDISCOVERY_OK && serviceName != NULL)
            {
                const size_t maxsize = ARDISCOVERY_AVAHIDISCOVERY_SERVICENAME_SIZE;
                serviceData->serviceName = malloc(maxsize);
                if (serviceData->serviceName != NULL)
                {
                    strncpy(serviceData->serviceName, serviceName, maxsize - 1);
                    serviceData->serviceName[maxsize - 1] = '\0';
                }
                else
                {
                    error = ARDISCOVERY_ERROR_ALLOC;
                }
            }
            else if (error == ARDISCOVERY_OK)
            {
                serviceData->serviceName = NULL;
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



static eARDISCOVERY_ERROR ARDISCOVERY_AvahiDiscovery_CreateService(ARDISCOVERY_AvahiDiscovery_PublisherData_t *serviceData)
{
    FILE *configfile;

    /*
     * Create Avahi service
     */

    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    if (serviceData == NULL)
    {
        ERR("Null parameter");
        return ARDISCOVERY_ERROR;
    }

    configfile = fopen(ARDISCOVERY_CONFIG_FILE,"w");

    if (NULL!=configfile)
    {
        fprintf(configfile ,
                "<?xml version=\"1.0\" standalone='no'?><!--*-nxml-*-->\n"
                "<!DOCTYPE service-group SYSTEM \"avahi-service.dtd\">\n"
                "<service-group>\n"
                "<name replace-wildcards=\"yes\">%s</name>\n"
                "<service>\n"
                "<type>%s</type>\n"
                "<port>%d</port>\n",
                (const char *)serviceData->serviceName,
                (const char *)serviceData->serviceType,
                serviceData->devicePort);

        if (serviceData->serviceJsonData != NULL)
        {
            fprintf(configfile, "<txt-record>%s</txt-record>\n", (const char *)serviceData->serviceJsonData);
        }

        fprintf(configfile,
                "</service>\n"
                "</service-group>\n");

        fclose(configfile);
        configfile=NULL;
    }
    else
    {
        error = ARDISCOVERY_ERROR_CREATE_CONFIG;
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_AvahiDiscovery_ResetService(ARDISCOVERY_AvahiDiscovery_PublisherData_t* serviceData)
{
    /*
     * Reset Avahi service
     */
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    if (serviceData == NULL)
    {
        ERR("Null parameter");
        error = ARDISCOVERY_ERROR;
    }

    if (0!=unlink(ARDISCOVERY_CONFIG_FILE))
    {
        error = ARDISCOVERY_ERROR_DELETE_CONFIG;
    }

    if (error == ARDISCOVERY_OK)
    {
        error = ARDISCOVERY_AvahiDiscovery_CreateService(serviceData);
    }

    return error;
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
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    if (serviceData == NULL)
    {
        ERR("Null parameter");
        error = ARDISCOVERY_ERROR;
    }

    if (error == ARDISCOVERY_OK)
    {
        /* Use device hostname if no service name was specified. */
        if (serviceData->serviceName == NULL)
        {
            serviceData->serviceName = ARDISCOVERY_AvahiDiscovery_BuildName();
        }
        if (serviceData->serviceName == NULL)
        {
            error = ARDISCOVERY_ERROR_BUILD_NAME;
        }
        else
        {
            error = ARDISCOVERY_AvahiDiscovery_CreateService(serviceData);
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
    unlink(ARDISCOVERY_CONFIG_FILE);
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
