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
 * @file ARDISCOVERY_JNI_Connection.c
 * @brief libARDiscovery JNI connection c file.
 **/

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <jni.h>
#include <stdlib.h>

#include <libARSAL/ARSAL_Print.h>

#include <libARDiscovery/ARDISCOVERY_Connection.h>

/*****************************************
 *
 *             define :
 *
 *****************************************/

#define ARDISCOVERY_JNICONNECTION_TAG "JNIDiscoveryConnection"

static jmethodID ARDISCOVERY_JNICONNECTION_METHOD_CONNECTION_SEND_JSON_CALLBACK;
static jmethodID ARDISCOVERY_JNICONNECTION_METHOD_CONNECTION_RECEIVE_JSON_CALLBACK;

static jmethodID ARDISCOVERY_JNICONNECTIONCALLBACKRETURN_METHOD_DATA_GET_ERROR;
static jmethodID ARDISCOVERY_JNICONNECTIONCALLBACKRETURN_METHOD_DATA_GET_DATA_TX;

/**
 * @brief JNI BLE DiscoveryConnection
 */
typedef struct
{
    ARDISCOVERY_Connection_ConnectionData_t *nativeConnectionData;
    jobject javaConnectionData;
} ARDISCOVERY_JNICONNECTIONDATA_t;

/*****************************************
 *
 *             private header:
 *
 *****************************************/

/**
 * @brief Create a new ARDISCOVERY_JNICONNECTIONDATA_t
 * @warning This function allocate memory
 * @post ARDISCOVERY_JNIConnectionData_Delete() must be called to delete the NIConnectionData and free the memory allocated.
 * @param env java environement
 * @param[in] error eARDISCOVERY_ERROR
 * @return the new ARDISCOVERY_JNICONNECTIONDATA_t
 * @see ARNETWORKAL_ARDISCOVERY_JNIConnectionData_Delete()
 */
ARDISCOVERY_JNICONNECTIONDATA_t *ARDISCOVERY_JNIConnectionData_New (JNIEnv *env, eARDISCOVERY_ERROR *error);

/**
 * @brief Delete the ARDISCOVERY_JNICONNECTIONDATA_t
 * @warning This function free memory
 * @param env java environement
 * @param jniBLENetwork JNIConnectionData to delete
 * @return error during callback execution
 * @see ARDISCOVERY_JNIConnectionData_New()
 */
eARDISCOVERY_ERROR ARNETWORKAL_ARDISCOVERY_JNIConnectionData_Delete (JNIEnv *env, ARDISCOVERY_JNICONNECTIONDATA_t **jniConnectionData);

/**
 * @brief callback use to send json information of the connection
 * @param[out] dataTx Transmission buffer ; must be filled with the json information of the connection
 * @param[out] dataTxSize Transmission data size
 * @param[in] customData custom data
 * @return error during callback execution
 */
eARDISCOVERY_ERROR ARDISCOVERY_JNIConnection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData);

/**
 * @brief callback use to receive json information of the connection
 * @param[in] dataRx Reception buffer; containing json information of the connection
 * @param[in] dataRxSize Reception data size
 * @param[in] ip ip address of the sender
 * @param[in] customData custom data
 * @return error during callback execution
 */
eARDISCOVERY_ERROR ARDISCOVERY_JNIConnection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData);

/*****************************************
 *
 *             implementation :
 *
 *****************************************/

JavaVM* ARDISCOVERY_JNI_VM = NULL; /** reference to the java virtual machine */

/**
 * @brief save the reference to the java virtual machine
 * @note this function is automatically call on the JNI startup
 * @param[in] VM reference to the java virtual machine
 * @param[in] reserved data reserved
 * @return JNI version
 */
JNIEXPORT jint JNICALL
JNI_OnLoad(JavaVM *VM, void *reserved)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDISCOVERY_JNICONNECTION_TAG, "Library has been loaded");

    /* Saving the reference to the java virtual machine */
    ARDISCOVERY_JNI_VM = VM;

    /* Return the JNI version */
    return JNI_VERSION_1_6;
}


JNIEXPORT void JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeStaticInit (JNIEnv *env, jclass class)
{
    /* local declarations */
    jclass jARDiscoveryConnectionCls = NULL;
    jclass jARDiscoveryConnectionCallbackReturnCls = NULL;

    /* get ARDiscoveryConnection */
    jARDiscoveryConnectionCls = (*env)->FindClass(env, "com/parrot/arsdk/ardiscovery/ARDiscoveryConnection");

    ARDISCOVERY_JNICONNECTION_METHOD_CONNECTION_SEND_JSON_CALLBACK = (*env)->GetMethodID (env, jARDiscoveryConnectionCls, "sendJsonCallback", "()Lcom/parrot/arsdk/ardiscovery/ARDiscoveryConnection$ARDiscoveryConnectionCallbackReturn;");
    ARDISCOVERY_JNICONNECTION_METHOD_CONNECTION_RECEIVE_JSON_CALLBACK = (*env)->GetMethodID (env, jARDiscoveryConnectionCls, "receiveJsonCallback", "([BLjava/lang/String;)I");

    /* get ARDiscoveryConnectionCallbackReturn */
    jARDiscoveryConnectionCallbackReturnCls = (*env)->FindClass(env, "com/parrot/arsdk/ardiscovery/ARDiscoveryConnection$ARDiscoveryConnectionCallbackReturn");

    ARDISCOVERY_JNICONNECTIONCALLBACKRETURN_METHOD_DATA_GET_ERROR = (*env)->GetMethodID(env, jARDiscoveryConnectionCallbackReturnCls, "getError", "()I");
    ARDISCOVERY_JNICONNECTIONCALLBACKRETURN_METHOD_DATA_GET_DATA_TX = (*env)->GetMethodID(env, jARDiscoveryConnectionCallbackReturnCls, "getDataTx", "()Ljava/lang/String;");

    /* cleanup */
    (*env)->DeleteLocalRef (env, jARDiscoveryConnectionCls);
    (*env)->DeleteLocalRef (env, jARDiscoveryConnectionCallbackReturnCls);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_STATUS_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_STATUS_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonStatusKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_STATUS_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_C2DPORT_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_C2DPORT_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonC2DPortKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_C2DPORT_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonD2CPortKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonControllerTypeKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonControllerNameKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_DEVICE_ID_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_DEVICE_ID_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonDeviceNameIdKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_DEVICE_ID_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM_FRAGMENT_SIZE_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM_FRAGMENT_SIZE_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStreamFragmentSizeKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM_FRAGMENT_SIZE_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM_FRAGMENT_MAXIMUM_NUMBER_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM_FRAGMENT_MAXIMUM_NUMBER_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStreamFragmentMaximumNumberKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM_FRAGMENT_MAXIMUM_NUMBER_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM_MAX_ACK_INTERVAL_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM_MAX_ACK_INTERVAL_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStreamMaxAckIntervalKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM_MAX_ACK_INTERVAL_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_C2D_UPDATE_PORT_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_C2D_UPDATE_PORT_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonC2DUpdatePortKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_C2D_UPDATE_PORT_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_C2D_USER_PORT_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_C2D_USER_PORT_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonC2DUserPortKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_C2D_USER_PORT_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_SKYCONTROLLER_VERSION
 * @return value of ARDISCOVERY_CONNECTION_JSON_SKYCONTROLLER_VERSION
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonSkyControllerVersionKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_SKYCONTROLLER_VERSION);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_FEATURES_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_FEATURES_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonFeaturesKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_FEATURES_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonQosModeKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_STREAM_PORT_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_STREAM_PORT_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStream2ClientStreamPortKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_STREAM_PORT_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_CONTROL_PORT_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_CONTROL_PORT_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStream2ClientControlPortKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_CONTROL_PORT_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_STREAM_PORT_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_STREAM_PORT_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStream2ServerStreamPortKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_STREAM_PORT_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_CONTROL_PORT_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_CONTROL_PORT_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStream2ServerControlPortKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_CONTROL_PORT_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_PACKET_SIZE_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_PACKET_SIZE_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStream2MaxPacketSizeKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_PACKET_SIZE_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_LATENCY_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_LATENCY_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStream2MaxLatencyKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_LATENCY_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_NETWORK_LATENCY_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_NETWORK_LATENCY_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStream2MaxNetworkLatencyKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_NETWORK_LATENCY_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_BITRATE_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_BITRATE_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStream2MaxBitrateKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_BITRATE_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SUPPORTED_METADATA_VERSION_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SUPPORTED_METADATA_VERSION_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStream2SupportedMetadataVersionKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SUPPORTED_METADATA_VERSION_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_PARAMETER_SETS_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_PARAMETER_SETS_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonARStream2ParameterSetsKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_PARAMETER_SETS_KEY);
}

/**
 * @brief get ARDISCOVERY_CONNECTION_JSON_AUDIO_CODEC_VERSION_KEY
 * @return value of ARDISCOVERY_CONNECTION_JSON_AUDIO_CODEC_VERSION_KEY
 */
JNIEXPORT jstring JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineJsonAudioCodecVersionKey (JNIEnv *env, jclass class)
{
    return  (*env)->NewStringUTF(env, ARDISCOVERY_CONNECTION_JSON_AUDIO_CODEC_VERSION_KEY);
}



/**
 * @brief get ARDISCOVERY_CONNECTION_TX_BUFFER_SIZE
 * @return value of ARDISCOVERY_CONNECTION_TX_BUFFER_SIZE
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeGetDefineTxBufferSize (JNIEnv *env, jclass class)
{
    return ARDISCOVERY_CONNECTION_TX_BUFFER_SIZE;
}

/**
 * @brief Create and initialize connection data
 * @param env reference to the java environment
 * @param thizz reference to the object calling this function
 * @param[in] callback Connection data management callback
 * @param[in] customData custom data
 * @return new jni connection data object
 */
JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeNew (JNIEnv *env, jobject thizz)
{
    /* local declarations */
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_JNICONNECTIONDATA_t *jniConnectionData;

    /* allocate the jniConnectionData */
    jniConnectionData = ARDISCOVERY_JNIConnectionData_New (env, &error);

    return (long) jniConnectionData;
}

/**
 * @brief Free data structures
 * @param env reference to the java environment
 * @param thizz reference to the object calling this function
 * @param jConnectionData jniConnectionData to delete
 * @param[in] connectionData Connection data
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeDelete (JNIEnv *env, jobject thizz, jlong jConnectionData)
{
    /* local declarations */
    ARDISCOVERY_JNICONNECTIONDATA_t *jniConnectionData = (ARDISCOVERY_JNICONNECTIONDATA_t *) (intptr_t) jConnectionData;

    return ARNETWORKAL_ARDISCOVERY_JNIConnectionData_Delete (env, &jniConnectionData);
}

/**
 * @brief Open a connection as a Controller
 * @warning this function keep a Global reference on the java ConnectionData call nativeControllerConnectionAbort() to free it
 * @post nativeControllerConnectionAbort() must be called to delete the Global reference on the java ConnectionData.
 * @param env reference to the java environment
 * @param thizz reference to the object calling this function
 * @param[in] jConnectionData JNI Connection data
 * @param[in] port port use to the discovery
 * @param[in] ipAddr device IP address
 * @return error during execution
 * @see nativeControllerConnectionAbort()
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeControllerConnection (JNIEnv *env, jobject thizz, jlong jConnectionData, jint port, jstring javaIP)
{
    /* local declarations */
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_JNICONNECTIONDATA_t *jniConnectionData = (ARDISCOVERY_JNICONNECTIONDATA_t *) (intptr_t) jConnectionData;
    const char *nativeIP = (*env)->GetStringUTFChars(env, javaIP, 0);

    /* check parameters */
    if (jniConnectionData == NULL)
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }

    if (error == ARDISCOVERY_OK)
    {
        /* create a global reference of the java object calling the fuction, delete by the close */
        jniConnectionData->javaConnectionData = (*env)->NewGlobalRef (env, thizz);

        /* call the native fonction */
        error = ARDISCOVERY_Connection_ControllerConnection (jniConnectionData->nativeConnectionData, port, nativeIP);
    }

    /* clean up */
    (*env)->ReleaseStringUTFChars(env, javaIP, nativeIP);

    return error;
}

/**
 * @brief Close connection
 * @param env reference to the java environment
 * @param thizz reference to the object calling this function
 * @param[in] jConnectionData Connection data
 * @see nativeControllerConnection()
 */
JNIEXPORT void JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeControllerConnectionAbort (JNIEnv *env, jobject thizz, jlong jConnectionData)
{
    /* local declarations */
    ARDISCOVERY_JNICONNECTIONDATA_t *jniConnectionData = (ARDISCOVERY_JNICONNECTIONDATA_t *) (intptr_t) jConnectionData;

    ARDISCOVERY_Connection_ControllerConnectionAbort (jniConnectionData->nativeConnectionData);

    /* free javaConnectionData */
    /* delete global references */
    (*env)->DeleteGlobalRef (env, jniConnectionData->javaConnectionData);
    jniConnectionData->javaConnectionData = NULL;
}

/**
 * @brief Open a connection as a Device
 * @param env reference to the java environment
 * @param thizz reference to the object calling this function
 * @param[in] jConnectionData JNI Connection data
 * @param[in] port port use to the discovery
 * @return error during execution
 * @see nativeDeviceStopListening()
 */
JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeDeviceListeningLoop (JNIEnv *env, jobject thizz, jlong jConnectionData, jint port)
{
    /* local declarations */
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    ARDISCOVERY_JNICONNECTIONDATA_t *jniConnectionData = (ARDISCOVERY_JNICONNECTIONDATA_t *) (intptr_t) jConnectionData;

    /* check parameters */
    if (jniConnectionData == NULL)
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }

    if (error == ARDISCOVERY_OK)
    {
        /* create a global reference of the java object calling the fuction, delete by the close */
        jniConnectionData->javaConnectionData = (*env)->NewGlobalRef (env, thizz);

        /* call the native fonction */
        error = ARDISCOVERY_Connection_DeviceListeningLoop (jniConnectionData->nativeConnectionData, port);
    }

    (*env)->DeleteGlobalRef (env, jniConnectionData->javaConnectionData);
    jniConnectionData->javaConnectionData = NULL;

    return error;
}

/**
 * @brief Stop listening as a device
 * @param env reference to the java environment
 * @param thizz reference to the object calling this function
 * @param[in] jConnectionData Connection data
 * @see nativeDeviceListeningLoop()
 */
JNIEXPORT void JNICALL
Java_com_parrot_arsdk_ardiscovery_ARDiscoveryConnection_nativeDeviceStopListening (JNIEnv *env, jobject thizz, jlong jConnectionData)
{
    /* local declarations */
    ARDISCOVERY_JNICONNECTIONDATA_t *jniConnectionData = (ARDISCOVERY_JNICONNECTIONDATA_t *) (intptr_t) jConnectionData;

    ARDISCOVERY_Connection_Device_StopListening (jniConnectionData->nativeConnectionData);
}


/*****************************************
 *
 *             private implementation:
 *
 *****************************************/

ARDISCOVERY_JNICONNECTIONDATA_t *ARDISCOVERY_JNIConnectionData_New (JNIEnv *env, eARDISCOVERY_ERROR *error)
{
    /* - Create a new ARDISCOVERY_JNICONNECTIONDATA - */

    eARDISCOVERY_ERROR localError = ARDISCOVERY_OK;
    ARDISCOVERY_JNICONNECTIONDATA_t *jniConnectionData = NULL;

    /* allocate the jniConnectionData */
    jniConnectionData = malloc (sizeof (ARDISCOVERY_JNICONNECTIONDATA_t));
    if (jniConnectionData != NULL)
    {
        /* initilaze the jniConnectionData */
        jniConnectionData->nativeConnectionData = NULL;
        jniConnectionData->javaConnectionData = NULL;
    }
    else
    {
        localError = ARDISCOVERY_ERROR_ALLOC;
    }

    if (localError == ARDISCOVERY_OK)
    {
        jniConnectionData->nativeConnectionData = ARDISCOVERY_Connection_New (&ARDISCOVERY_JNIConnection_SendJsonCallback, &ARDISCOVERY_JNIConnection_ReceiveJsonCallback, jniConnectionData, error);
    }

    /* delete the jniBLENetwork if an error occurred */
    if (localError != ARDISCOVERY_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARDISCOVERY_JNICONNECTION_TAG, "error: %s", ARDISCOVERY_Error_ToString (localError));
        ARNETWORKAL_ARDISCOVERY_JNIConnectionData_Delete (env, &jniConnectionData);
    }

    /* error return */
    if (error != NULL)
    {
        *error = localError;
    }

    return jniConnectionData;
}

eARDISCOVERY_ERROR ARNETWORKAL_ARDISCOVERY_JNIConnectionData_Delete (JNIEnv *env, ARDISCOVERY_JNICONNECTIONDATA_t **jniConnectionData)
{
    /* - ARDISCOVERY_JNICONNECTIONDATA - */

    /* local declarations */
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;

    /* check parameters */
    if (jniConnectionData == NULL)
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }

    if (error == ARDISCOVERY_OK)
    {
        if (*jniConnectionData != NULL)
        {
            /* delete javaConnectionData */
            (*env)->DeleteGlobalRef (env, (*jniConnectionData)->javaConnectionData);
            (*jniConnectionData)->javaConnectionData = NULL;

            /* delete the nativeConnectionData */
            error = ARDISCOVERY_Connection_Delete (&((*jniConnectionData)->nativeConnectionData));

            /* free jniConnectionData */
            free (*jniConnectionData);
            *jniConnectionData = NULL;
        }
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_JNIConnection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData)
{
    /* - callback use to send json information of the connection -*/

    /* local declarations */
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    JNIEnv* env = NULL;
    jint getEnvResult = JNI_OK;
    jint attachResult = 1;

    ARDISCOVERY_JNICONNECTIONDATA_t *jniConnectionData = (ARDISCOVERY_JNICONNECTIONDATA_t *) customData;
    jobject jcallbackReturn = NULL;
    jstring jDataTx = NULL;
    char *nativeDataTx = NULL;
    int nativeDataTxLength = 0;

    /* check the virtual machine */
    if (ARDISCOVERY_JNI_VM == NULL)
    {
        error = ARDISCOVERY_ERROR_JNI_VM;
    }

    /* check parameters */
    if ((jniConnectionData == NULL) || (jniConnectionData->javaConnectionData == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }

    if (error == ARDISCOVERY_OK)
    {
        /* get the environment */
        getEnvResult = (*ARDISCOVERY_JNI_VM)->GetEnv(ARDISCOVERY_JNI_VM, (void **) &env, JNI_VERSION_1_6);

        /* if no environment then attach the thread to the virtual machine */
        if (getEnvResult == JNI_EDETACHED)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDISCOVERY_JNICONNECTION_TAG, "attach the thread to the virtual machine ...");
            attachResult = (*ARDISCOVERY_JNI_VM)->AttachCurrentThread(ARDISCOVERY_JNI_VM, &env, NULL);
        }

        if (env == NULL)
        {
            error = ARDISCOVERY_ERROR_JNI_ENV;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        /* java send json callback */
        jcallbackReturn = (*env)->CallObjectMethod(env, jniConnectionData->javaConnectionData, ARDISCOVERY_JNICONNECTION_METHOD_CONNECTION_SEND_JSON_CALLBACK);

        /* get callback return */
        if (jcallbackReturn != NULL)
        {
            /* get the java callback jDataTx */
            jDataTx = (*env)->CallObjectMethod (env, jcallbackReturn, ARDISCOVERY_JNICONNECTIONCALLBACKRETURN_METHOD_DATA_GET_DATA_TX);
            if (jDataTx != NULL)
            {
                /* copy jDataTx in dataTx */
                nativeDataTx = (char *) (*env)->GetStringUTFChars(env, jDataTx, NULL);
                nativeDataTxLength = strlen(nativeDataTx);

                memcpy (dataTx, nativeDataTx, nativeDataTxLength);
                *dataTxSize = nativeDataTxLength;

                (*env)->ReleaseStringUTFChars(env, jDataTx, nativeDataTx);
                nativeDataTx = NULL;
                nativeDataTxLength = 0;
            }

            /* get the java callback error */
            error = (*env)->CallIntMethod (env, jcallbackReturn, ARDISCOVERY_JNICONNECTIONCALLBACKRETURN_METHOD_DATA_GET_ERROR);
        }
    }

    /* cleanup */
    /* delete local references */
    (*env)->DeleteLocalRef (env, jcallbackReturn);
    jcallbackReturn = NULL;
    (*env)->DeleteLocalRef (env, jDataTx);
    jDataTx = NULL;

    /* if the thread has been attached then detach the thread from the virtual machine */
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARDISCOVERY_JNI_VM)->DetachCurrentThread(ARDISCOVERY_JNI_VM);
    }

    if (error != ARDISCOVERY_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARDISCOVERY_JNICONNECTION_TAG, "error occured: %s", ARDISCOVERY_Error_ToString (error));
    }

    return error;
}

eARDISCOVERY_ERROR ARDISCOVERY_JNIConnection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData)
{
    /* - callback use to receive json information of the connection - */

    /* local declarations */
    eARDISCOVERY_ERROR error = ARDISCOVERY_OK;
    JNIEnv* env = NULL;
    jint getEnvResult = JNI_OK;
    jint attachResult = 1;

    ARDISCOVERY_JNICONNECTIONDATA_t *jniConnectionData = (ARDISCOVERY_JNICONNECTIONDATA_t *) customData;
    jbyteArray jDataRx = NULL;
    jstring jIP = NULL;

    /* check the virtual machine */
    if (ARDISCOVERY_JNI_VM == NULL)
    {
        error = ARDISCOVERY_ERROR_JNI_VM;
    }

    /* check parameters */
    if ((jniConnectionData == NULL) || (jniConnectionData->javaConnectionData == NULL))
    {
        error = ARDISCOVERY_ERROR_BAD_PARAMETER;
    }

    if (error == ARDISCOVERY_OK)
    {
        /* get the environment */
        getEnvResult = (*ARDISCOVERY_JNI_VM)->GetEnv(ARDISCOVERY_JNI_VM, (void **) &env, JNI_VERSION_1_6);

        /* if no environment then attach the thread to the virtual machine */
        if (getEnvResult == JNI_EDETACHED)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARDISCOVERY_JNICONNECTION_TAG, "attach the thread to the virtual machine ...");
            attachResult = (*ARDISCOVERY_JNI_VM)->AttachCurrentThread(ARDISCOVERY_JNI_VM, &env, NULL);
        }

        if (env == NULL)
        {
            error = ARDISCOVERY_ERROR_JNI_ENV;
        }
    }

    if (error == ARDISCOVERY_OK)
    {
        /* convert native IP to java IP */
        if (ip != NULL)
        {
            jIP = (*env)->NewStringUTF(env,ip);
        }

        if (dataRx != NULL && dataRxSize != 0)
        {
            /* create the data array jDataRx to send to the java callback */
            jDataRx = (*env)->NewByteArray(env, dataRxSize);
            (*env)->SetByteArrayRegion(env, jDataRx, 0, dataRxSize, (jbyte *)dataRx);
        }

        /* java receive json callback */
        error = (*env)->CallIntMethod(env, jniConnectionData->javaConnectionData, ARDISCOVERY_JNICONNECTION_METHOD_CONNECTION_RECEIVE_JSON_CALLBACK, jDataRx, jIP);

	/* cleanup */
	/* delete local references */
	(*env)->DeleteLocalRef (env, jDataRx);
	jDataRx = NULL;
	(*env)->DeleteLocalRef (env, jIP);
	jIP = NULL;
    }

    /* if the thread has been attached then detach the thread from the virtual machine */
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARDISCOVERY_JNI_VM)->DetachCurrentThread(ARDISCOVERY_JNI_VM);
    }

    if (error != ARDISCOVERY_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, ARDISCOVERY_JNICONNECTION_TAG, "error occured: %s", ARDISCOVERY_Error_ToString (error));
    }

    return error;
}
