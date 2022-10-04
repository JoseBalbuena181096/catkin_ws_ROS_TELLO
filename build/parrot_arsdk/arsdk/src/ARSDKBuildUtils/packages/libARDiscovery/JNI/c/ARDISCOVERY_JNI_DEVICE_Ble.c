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
 * @file ARDISCOVERY_JNI_DEVICE_Ble.c
 * @brief Discovery BLE Device contains the informations of a device discovered
 * @date 02/03/2015
 * @author maxime.maitre@parrot.com
 */

#include <jni.h>
#include <stdlib.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetworkAL/ARNETWORKAL_Manager.h>
#include <libARNetworkAL/ARNETWORKAL_Error.h>
#include <libARDiscovery/ARDISCOVERY_Discovery.h>
#include <libARDiscovery/ARDISCOVERY_Device.h>

#include "ARDISCOVERY_JNI_DEVICE_Ble.h"

#define ARDISCOVERY_JNI_DEVICE_BLE_TAG "ARDISCOVERY_DEVICE_BLE"

/*************************
 * Private header
 *************************/

/*************************
 * Implementation
 *************************/

eARDISCOVERY_ERROR ARDISCOVERY_JNI_DEVICE_Ble_CreateSpecificParameters (JNIEnv *env, ARDISCOVERY_Device_t *device, jobject jBLEPart)
{
    // Initialize BLE specific parameters 
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_JNI_DEVICE_BLE_t *specificBLEParam = NULL;
        
    // Check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing
    
    if (error == ARDISCOVERY_OK)
    {
        specificBLEParam = malloc(sizeof(ARDISCOVERY_JNI_DEVICE_BLE_t));
        if (specificBLEParam != NULL)
        {
            device->specificParameters = specificBLEParam;
            specificBLEParam->jBLEPart = (*env)->NewGlobalRef (env, jBLEPart);
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
        ARDISCOVERY_JNI_DEVICE_Ble_DeleteSpecificParameters (device);
    }
    // No else: skipped no error
    
    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_JNI_DEVICE_Ble_DeleteSpecificParameters (ARDISCOVERY_Device_t *device)
{
    // -- Delete SpecificParameters allocated by the BLE initialization --
        
    // local declarations
    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    JNIEnv* env = NULL;
    jint getEnvResult = JNI_OK;
    jint attachResult = 1;
    
    ARDISCOVERY_JNI_DEVICE_BLE_t *specificBLEParam = NULL;
    
    // check parameters
    if ((device == NULL) || 
        (device->specificParameters == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE))
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing
    
    if (localError == ARDISCOVERY_OK)
    {
        // get the environment
        getEnvResult = (*ARDISCOVERY_JNI_VM)->GetEnv(ARDISCOVERY_JNI_VM, (void **) &env, JNI_VERSION_1_6);

        // if no environment then attach the thread to the virtual machine
        if (getEnvResult == JNI_EDETACHED)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDISCOVERY_JNI_DEVICE_BLE_TAG, "attach the thread to the virtual machine ...");
            attachResult = (*ARDISCOVERY_JNI_VM)->AttachCurrentThread(ARDISCOVERY_JNI_VM, &env, NULL);
        }

        if (env == NULL)
        {
            localError = ARDISCOVERY_ERROR_JNI_ENV;
        }
    }
    
    if (localError == ARDISCOVERY_OK)
    {
        // free specific parameter

        // cast device->specificBLEParam
        specificBLEParam = (ARDISCOVERY_JNI_DEVICE_BLE_t *)device->specificParameters;

        (*env)->DeleteGlobalRef (env, specificBLEParam->jBLEPart);
        specificBLEParam->jBLEPart = NULL;
        
        free (device->specificParameters);
        device->specificParameters = NULL;
    }
    
    // if the thread has been attached then detach the thread from the virtual machine
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARDISCOVERY_JNI_VM)->DetachCurrentThread(ARDISCOVERY_JNI_VM);
    }
    
    return localError;
}

void *ARDISCOVERY_JNI_DEVICE_Ble_GetCopyOfSpecificParameters (ARDISCOVERY_Device_t *device, eARDISCOVERY_ERROR *error)
{
    // -- Copy BLE specificParameters --
    
    // local declarations
    JNIEnv* env = NULL;
    jint getEnvResult = JNI_OK;
    jint attachResult = 1;
    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    ARDISCOVERY_JNI_DEVICE_BLE_t *specificBLEParamToCopy = NULL;
    ARDISCOVERY_JNI_DEVICE_BLE_t *specificBLEParam = NULL;
    
    // check parameters
    if ((device == NULL) || (device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE))
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing
    
    if (localError == ARDISCOVERY_OK)
    {
        // get the environment
        getEnvResult = (*ARDISCOVERY_JNI_VM)->GetEnv(ARDISCOVERY_JNI_VM, (void **) &env, JNI_VERSION_1_6);

        // if no environment then attach the thread to the virtual machine
        if (getEnvResult == JNI_EDETACHED)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDISCOVERY_JNI_DEVICE_BLE_TAG, "attach the thread to the virtual machine ...");
            attachResult = (*ARDISCOVERY_JNI_VM)->AttachCurrentThread(ARDISCOVERY_JNI_VM, &env, NULL);
        }

        if (env == NULL)
        {
            localError = ARDISCOVERY_ERROR_JNI_ENV;
        }
    }
    
    if (localError == ARDISCOVERY_OK)
    {
        // cast device->specificBLEParam
        specificBLEParamToCopy = (ARDISCOVERY_JNI_DEVICE_BLE_t *)device->specificParameters;
        
        if (specificBLEParamToCopy != NULL)
        {
            // Copy BLE specific parameters 
            specificBLEParam = malloc(sizeof(ARDISCOVERY_JNI_DEVICE_BLE_t));
            if (specificBLEParam != NULL)
            {
                specificBLEParam->jBLEPart = (*env)->NewGlobalRef (env, specificBLEParamToCopy->jBLEPart);
            }
            else
            {
                localError = ARDISCOVERY_ERROR_ALLOC;
            }
        }
        // NO Else ; No Specific Wifi Parameters To Copy.
    }
    // No else: skipped by error
    
    // delete the SpecificParameters if an error occurred
    if (localError != ARDISCOVERY_OK)
    {
        ARDISCOVERY_JNI_DEVICE_Ble_DeleteSpecificParameters (device);
    }
    // No else: skipped no error
    
    // if the thread has been attached then detach the thread from the virtual machine
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARDISCOVERY_JNI_VM)->DetachCurrentThread(ARDISCOVERY_JNI_VM);
    }
    
    // return the error
    if (error != NULL)
    {
        *error = localError;
    }
    // No else: error is not returned
        
    return specificBLEParam;
}

ARNETWORKAL_Manager_t *ARDISCOVERY_JNI_DEVICE_Ble_NewARNetworkAL (ARDISCOVERY_Device_t *device, eARDISCOVERY_ERROR *error, eARNETWORKAL_ERROR *errorAL)
{
    //-- Create a new networlAL adapted to the device. --
        
    // local declarations
    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    JNIEnv* env = NULL;
    jint getEnvResult = JNI_OK;
    jint attachResult = 1;
    
    ARNETWORKAL_Manager_t *networkAL = NULL;
    
    ARDISCOVERY_JNI_DEVICE_BLE_t *specificBLEParam = NULL;
    
    // check parameters
    if ((device == NULL) ||
        (device->specificParameters == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE))
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing
    
    if (localError == ARDISCOVERY_OK)
    {
        // get the environment
        getEnvResult = (*ARDISCOVERY_JNI_VM)->GetEnv(ARDISCOVERY_JNI_VM, (void **) &env, JNI_VERSION_1_6);

        // if no environment then attach the thread to the virtual machine
        if (getEnvResult == JNI_EDETACHED)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDISCOVERY_JNI_DEVICE_BLE_TAG, "attach the thread to the virtual machine ...");
            attachResult = (*ARDISCOVERY_JNI_VM)->AttachCurrentThread(ARDISCOVERY_JNI_VM, &env, NULL);
        }

        if (env == NULL)
        {
            localError = ARDISCOVERY_ERROR_JNI_ENV;
        }
    }
    
    if (localError == ARDISCOVERY_OK)
    {
        // cast device->specificBLEParam
        specificBLEParam = (ARDISCOVERY_JNI_DEVICE_BLE_t *)device->specificParameters;
        
        networkAL = (ARNETWORKAL_Manager_t*) (intptr_t) (*env)->CallLongMethod(env, specificBLEParam->jBLEPart, ARDISCOVERY_JNIDEVICE_BLE_METHOD_NEWNETWORKAL);
    }
    
    // if the thread has been attached then detach the thread from the virtual machine
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARDISCOVERY_JNI_VM)->DetachCurrentThread(ARDISCOVERY_JNI_VM);
    }
    
    // return localError
    if (error != NULL)
    {
        *error = localError;
    }
    
    // delete networkAL if an error occured
    if ((localError != ARDISCOVERY_OK) && (networkAL != NULL))
    {
        ARDISCOVERY_JNI_DEVICE_Ble_DeleteARNetworkAL (device, &networkAL);
    }

    return networkAL;
}

eARDISCOVERY_ERROR ARDISCOVERY_JNI_DEVICE_Ble_DeleteARNetworkAL (ARDISCOVERY_Device_t *device, ARNETWORKAL_Manager_t **networkAL)
{
    // -- Delete a networlAL create by ARDISCOVERY_JNI_DEVICE_Ble_NewARNetworkAL --
    
    // local declarations
    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    JNIEnv* env = NULL;
    jint getEnvResult = JNI_OK;
    jint attachResult = 1;
    
    ARDISCOVERY_JNI_DEVICE_BLE_t *specificBLEParam = NULL;
    
    // check parameters
    if ((device == NULL) ||
        (device->specificParameters == NULL) ||
        (device->networkType != ARDISCOVERY_NETWORK_TYPE_BLE))
    {
        localError = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }
    // No Else: the checking parameters sets error to ARNETWORK_ERROR_BAD_PARAMETER and stop the processing
    
    if (localError == ARDISCOVERY_OK)
    {
        // get the environment
        getEnvResult = (*ARDISCOVERY_JNI_VM)->GetEnv(ARDISCOVERY_JNI_VM, (void **) &env, JNI_VERSION_1_6);

        // if no environment then attach the thread to the virtual machine
        if (getEnvResult == JNI_EDETACHED)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDISCOVERY_JNI_DEVICE_BLE_TAG, "attach the thread to the virtual machine ...");
            attachResult = (*ARDISCOVERY_JNI_VM)->AttachCurrentThread(ARDISCOVERY_JNI_VM, &env, NULL);
        }

        if (env == NULL)
        {
            localError = ARDISCOVERY_ERROR_JNI_ENV;
        }
    }
    
    if (localError == ARDISCOVERY_OK)
    {
        // cast device->specificBLEParam
        specificBLEParam = (ARDISCOVERY_JNI_DEVICE_BLE_t *)device->specificParameters;

        localError = (*env)->CallIntMethod(env, specificBLEParam->jBLEPart, ARDISCOVERY_JNIDEVICE_BLE_METHOD_DELETENETWORKAL);
    }
    
    // if the thread has been attached then detach the thread from the virtual machine
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARDISCOVERY_JNI_VM)->DetachCurrentThread(ARDISCOVERY_JNI_VM);
    }
    
    return localError;
}


 /*************************
 * local Implementation
 *************************/
