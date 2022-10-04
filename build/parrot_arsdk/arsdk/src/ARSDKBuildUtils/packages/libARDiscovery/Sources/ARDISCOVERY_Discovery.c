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
#include <libARDiscovery/ARDISCOVERY_Discovery.h>
#include <string.h>
#include <libARSAL/ARSAL_Print.h>

#define ARDISCOVERY_DISCOVERY_TAG "ARDISCOVERY_Discovery"

static const uint16_t ARDISCOVERY_Discovery_ProductTable[ARDISCOVERY_PRODUCT_MAX] =
{
    // BLE Service
    [ARDISCOVERY_PRODUCT_MINIDRONE]     = 0x0900,
    [ARDISCOVERY_PRODUCT_MINIDRONE_EVO_LIGHT] = 0x0907,
    [ARDISCOVERY_PRODUCT_MINIDRONE_EVO_BRICK] = 0x0909,
    [ARDISCOVERY_PRODUCT_MINIDRONE_EVO_HYDROFOIL] = 0x090a,
    [ARDISCOVERY_PRODUCT_MINIDRONE_DELOS3] = 0x090b,
    [ARDISCOVERY_PRODUCT_MINIDRONE_WINGX] = 0x0910,

    // NSNet Service
    [ARDISCOVERY_PRODUCT_ARDRONE]       = 0x0901,
    [ARDISCOVERY_PRODUCT_JS]            = 0x0902,
    [ARDISCOVERY_PRODUCT_SKYCONTROLLER] = 0x0903,
    [ARDISCOVERY_PRODUCT_JS_EVO_LIGHT]  = 0x0905,
    [ARDISCOVERY_PRODUCT_JS_EVO_RACE]   = 0x0906,
    [ARDISCOVERY_PRODUCT_BEBOP_2]       = 0x090c,
    [ARDISCOVERY_PRODUCT_POWER_UP]      = 0x090d,
    [ARDISCOVERY_PRODUCT_EVINRUDE]      = 0x090e,
    [ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_4]         = 0x0911,
    [ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG] = 0x0913,
    [ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_5]       = 0x0914,
    [ARDISCOVERY_PRODUCT_CHIMERA]       = 0x0916,

    // USB Service
    [ARDISCOVERY_PRODUCT_SKYCONTROLLER_2] = 0x090f,
    [ARDISCOVERY_PRODUCT_SKYCONTROLLER_2P] = 0x0915,

    // Unsupported Service
    [ARDISCOVERY_PRODUCT_TINOS] = 0x0912,
    [ARDISCOVERY_PRODUCT_SEQUOIA] = 0x0917,
};

static const char* ARDISCOVERY_Discovery_ProductNameTable[ARDISCOVERY_PRODUCT_MAX] =
{
    // BLE Service
    [ARDISCOVERY_PRODUCT_MINIDRONE]     = "Rolling Spider",
    [ARDISCOVERY_PRODUCT_MINIDRONE_EVO_LIGHT] = "Airborne Night",
    [ARDISCOVERY_PRODUCT_MINIDRONE_EVO_BRICK] = "Airborne Cargo",
    [ARDISCOVERY_PRODUCT_MINIDRONE_EVO_HYDROFOIL] = "Hydrofoil",
    [ARDISCOVERY_PRODUCT_MINIDRONE_DELOS3] = "Mambo",
    [ARDISCOVERY_PRODUCT_MINIDRONE_WINGX] = "Swing",

    // NSNet Service
    [ARDISCOVERY_PRODUCT_ARDRONE]       = "Bebop Drone",
    [ARDISCOVERY_PRODUCT_JS]            = "Jumping Sumo",
    [ARDISCOVERY_PRODUCT_SKYCONTROLLER] = "SkyController",
    [ARDISCOVERY_PRODUCT_JS_EVO_LIGHT]  = "Jumping Night",
    [ARDISCOVERY_PRODUCT_JS_EVO_RACE]   = "Jumping Race",
    [ARDISCOVERY_PRODUCT_BEBOP_2]       = "Bebop 2",
    [ARDISCOVERY_PRODUCT_POWER_UP]      = "Power Up",
    [ARDISCOVERY_PRODUCT_EVINRUDE]      = "Disco",
    [ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_4]         = "Unknownproduct_4",
    [ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG] = "SkyController",
    [ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_5]       = "Unknownproduct_5",
    [ARDISCOVERY_PRODUCT_CHIMERA]       = "Bluegrass",

    // USB service
    [ARDISCOVERY_PRODUCT_SKYCONTROLLER_2] = "SkyController 2",
    [ARDISCOVERY_PRODUCT_SKYCONTROLLER_2P] = "SkyController 2P",

    // Unsupported Service
    [ARDISCOVERY_PRODUCT_TINOS] = "Flypad",
    [ARDISCOVERY_PRODUCT_SEQUOIA] = "Sequoia",
};

uint16_t ARDISCOVERY_getProductID(eARDISCOVERY_PRODUCT product)
{
    if (product < ARDISCOVERY_PRODUCT_MAX)
    {
        return ARDISCOVERY_Discovery_ProductTable[product];
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDISCOVERY_DISCOVERY_TAG,
                    "Unknown product : %d", product);
        return 0;
    }
}

const char* ARDISCOVERY_getProductName(eARDISCOVERY_PRODUCT product)
{
    char *name = NULL;
    if(product < ARDISCOVERY_PRODUCT_MAX)
    {
        name = (char *)ARDISCOVERY_Discovery_ProductNameTable[product];
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDISCOVERY_DISCOVERY_TAG,
                    "Unknown product : %d", product);
        name = "UNKNOWN";
    }

    return (const char *)name;
}

void ARDISCOVERY_getProductPathName(eARDISCOVERY_PRODUCT product, char *buffer, int length)
{
    if ((buffer == NULL) || (length <= 0))
    {
        return;
    }

    if (product < ARDISCOVERY_PRODUCT_MAX)
    {
        const char *name = ARDISCOVERY_getProductName(product);
        int nameLen = strlen(name);
        char *index;

        if (length > nameLen)
        {
            strncpy(buffer, name, nameLen + 1);
            index = buffer;
            while (*index != '\0')
            {
                if (*index == '.' ||
                    *index == ' ')
                {
                    *index = '_';
                }
                index++;
            }
        }
        else
        {
            *buffer = '\0';
        }
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARDISCOVERY_DISCOVERY_TAG,
                    "Unknown product : %d", product);
    }
}

eARDISCOVERY_PRODUCT ARDISCOVERY_getProductFromName(const char *name)
{
    uint8_t product = ARDISCOVERY_PRODUCT_MAX;
    int i = 0;

    if (name == NULL)
        return ARDISCOVERY_PRODUCT_MAX;
    for (i = 0; (product == ARDISCOVERY_PRODUCT_MAX) && (i < ARDISCOVERY_PRODUCT_MAX); i++)
    {
        if(strcmp(name, ARDISCOVERY_Discovery_ProductNameTable[i]) == 0)
            product = i;
    }

    return product;
}

eARDISCOVERY_PRODUCT ARDISCOVERY_getProductFromPathName(const char *name)
{
    uint8_t product = ARDISCOVERY_PRODUCT_MAX;
    size_t namelen;
    int i = 0;

    char buffer[256];

    if (name == NULL)
        return ARDISCOVERY_PRODUCT_MAX;

    namelen = strlen(name);
    for (i = 0; (product == ARDISCOVERY_PRODUCT_MAX) && (i < ARDISCOVERY_PRODUCT_MAX); i++)
    {
        ARDISCOVERY_getProductPathName(i, buffer, 256);
        if(namelen < strlen(buffer))
            continue;

        if(strncmp(name, buffer, strlen(buffer)) == 0)
            product = i;
    }

    return product;
}

eARDISCOVERY_PRODUCT ARDISCOVERY_getProductFromProductID(uint16_t productID)
{
    uint8_t product = ARDISCOVERY_PRODUCT_MAX;
    int i = 0;

    for (i = 0; (product == ARDISCOVERY_PRODUCT_MAX) && (i < ARDISCOVERY_PRODUCT_MAX); i++)
    {
        if (ARDISCOVERY_Discovery_ProductTable[i] == productID)
        {
            product = i;
        }
    }

    return product;
}

eARDISCOVERY_PRODUCT_FAMILY ARDISCOVERY_getProductFamily(eARDISCOVERY_PRODUCT product)
{
    eARDISCOVERY_PRODUCT_FAMILY family = ARDISCOVERY_PRODUCT_FAMILY_MAX;

    switch (product)
    {
    case ARDISCOVERY_PRODUCT_ARDRONE:
    case ARDISCOVERY_PRODUCT_BEBOP_2:
    case ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_4:
    case ARDISCOVERY_PRODUCT_UNKNOWNPRODUCT_5:
    case ARDISCOVERY_PRODUCT_CHIMERA:
        family = ARDISCOVERY_PRODUCT_FAMILY_ARDRONE;
        break;
    case ARDISCOVERY_PRODUCT_JS:
    case ARDISCOVERY_PRODUCT_JS_EVO_LIGHT:
    case ARDISCOVERY_PRODUCT_JS_EVO_RACE:
        family = ARDISCOVERY_PRODUCT_FAMILY_JS;
        break;
    case ARDISCOVERY_PRODUCT_SKYCONTROLLER:
    case ARDISCOVERY_PRODUCT_SKYCONTROLLER_2:
    case ARDISCOVERY_PRODUCT_SKYCONTROLLER_2P:
    case ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG:
        family = ARDISCOVERY_PRODUCT_FAMILY_SKYCONTROLLER;
        break;
    case ARDISCOVERY_PRODUCT_MINIDRONE:
    case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_LIGHT:
    case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_BRICK:
    case ARDISCOVERY_PRODUCT_MINIDRONE_EVO_HYDROFOIL:
    case ARDISCOVERY_PRODUCT_MINIDRONE_DELOS3:
    case ARDISCOVERY_PRODUCT_MINIDRONE_WINGX:
        family = ARDISCOVERY_PRODUCT_FAMILY_MINIDRONE;
        break;
    case ARDISCOVERY_PRODUCT_POWER_UP:
        family = ARDISCOVERY_PRODUCT_FAMILY_POWER_UP;
        break;
    case ARDISCOVERY_PRODUCT_EVINRUDE:
        family = ARDISCOVERY_PRODUCT_FAMILY_FIXED_WING;
        break;
    case ARDISCOVERY_PRODUCT_TINOS:
        family = ARDISCOVERY_PRODUCT_FAMILY_GAMEPAD;
        break;
    case ARDISCOVERY_PRODUCT_SEQUOIA:
        family = ARDISCOVERY_PRODUCT_FAMILY_CAMERA;
        break;
    default:
        break;
    }

    return family;
}
