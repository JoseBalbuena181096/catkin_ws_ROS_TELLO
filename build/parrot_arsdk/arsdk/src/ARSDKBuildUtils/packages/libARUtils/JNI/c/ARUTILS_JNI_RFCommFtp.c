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
 * @file  ARNETWORKAL_JNIRFCommNetwork.c
 * @brief private headers of RFComm network manager allow to send over rfcomm network.
 * @date
 * @author
 */

#ifdef NDEBUG
/* Android ndk-build NDK_DEBUG=0*/
#else
/* Android ndk-build NDK_DEBUG=1*/
#ifndef DEBUG
#define DEBUG
#endif
#endif

#include <jni.h>
#include <stdlib.h>

#include <libARSAL/ARSAL_Print.h>
#include <libARUtils/ARUTILS_Manager.h>
#include "ARUTILS_JNI_RFCommFtp.h"
//#include "../../Sources/BLE/ARNETWORKAL_BLENetwork.h"

/*****************************************
 *
 *             define :
 *
 *****************************************/

#define ARUTILS_JNI_RFCOMMFTP_TAG "JNIRFCommFtp"

extern JavaVM *ARUTILS_JNI_Manager_VM; /** reference to the java virtual machine */

static jmethodID ARUTILS_JNI_RFCOMMFTP_METHOD_CONNECTION_CANCEL;
static jmethodID ARUTILS_JNI_RFCOMMFTP_METHOD_IS_CANCELED;
static jmethodID ARUTILS_JNI_RFCOMMFTP_METHOD_CONNECTION_RESET;
static jmethodID ARUTILS_JNI_RFCOMMFTP_METHOD_PUT;



/*****************************************
 *
 *             implementation :
 *
 *****************************************/


JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arutils_ARUtilsRFCommFtp_nativeJNIInit(JNIEnv *env, jobject obj)
{
    /* -- initialize the JNI part -- */
    /* load the references of java methods */

    jclass jRFCommFtpCls = (*env)->FindClass(env, "com/parrot/arsdk/arutils/ARUtilsRFCommFtp");
    
    ARUTILS_JNI_RFCOMMFTP_METHOD_CONNECTION_CANCEL = (*env)->GetMethodID(env, jRFCommFtpCls, "cancelFileAL", "(Ljava/util/concurrent/Semaphore;)Z");
    ARUTILS_JNI_RFCOMMFTP_METHOD_IS_CANCELED = (*env)->GetMethodID(env, jRFCommFtpCls, "isConnectionCanceledAL", "(Ljava/util/concurrent/Semaphore;)Z");
    ARUTILS_JNI_RFCOMMFTP_METHOD_CONNECTION_RESET = (*env)->GetMethodID(env, jRFCommFtpCls, "resetConnectionAL", "(Ljava/util/concurrent/Semaphore;)Z");
    ARUTILS_JNI_RFCOMMFTP_METHOD_PUT = (*env)->GetMethodID(env, jRFCommFtpCls, "putFileAL", "(Ljava/lang/String;Ljava/lang/String;JZLjava/util/concurrent/Semaphore;)Z");

    /* cleanup */
    (*env)->DeleteLocalRef (env, jRFCommFtpCls);
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arutils_ARUtilsRFCommFtp_nativeProgressCallback(JNIEnv *env, jobject obj, jlong jCallback, jfloat percent)
{
    ARUTILS_RFCommFtp_Command_t * callback = (ARUTILS_RFCommFtp_Command_t*)(intptr_t) jCallback;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, " %p %f ", callback, percent);
    if (callback != NULL && callback->rfcommFtpProgressCallback != NULL)
    {
        callback->rfcommFtpProgressCallback(callback->rfcommProgressArg, percent);
    }
}

/**
 * @brief Create a new Ftp Connection
 * @warning This function allocates memory
 * @param cancelSem The pointer of the Ftp get/put cancel semaphore or null
 * @param device The RFComm Ftp device
 * @param[out] error The pointer of the error code: if success ARUTILS_OK, otherwise an error number of eARUTILS_ERROR
 * @retval On success, returns an ARUTILS_FtpAL_Connection_t. Otherwise, it returns null.
 * @see ARUTILS_FtpAL_DeleteConnection ()
 */
ARUTILS_RFCommFtp_Connection_t * ARUTILS_RFCommFtp_Connection_New(jobject rfcommFtp, jobject cancelSem, eARUTILS_ERROR *error)
{

    jobject rfcommFtpObject = NULL;
    jobject cancelSemObject = NULL;

    ARUTILS_RFCommFtp_Connection_t *newConnection = NULL;

    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;

    /* get the environment */
    if (ARUTILS_JNI_Manager_VM != NULL)
    {
        getEnvResult = (*ARUTILS_JNI_Manager_VM)->GetEnv(ARUTILS_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);
    }
    /* if no environment then attach the thread to the virtual machine */
    if (getEnvResult == JNI_EDETACHED)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, "attach the thread to the virtual machine ...");
        (*ARUTILS_JNI_Manager_VM)->AttachCurrentThread(ARUTILS_JNI_Manager_VM, &env, NULL);
    }
    /* check the environment  */
    if (env == NULL)
    {
        *error = ARUTILS_ERROR;
    }

    if (*error == ARUTILS_OK)
    {
        newConnection = calloc(1, sizeof(ARUTILS_RFCommFtp_Connection_t));
        if (newConnection == NULL)
        {
            *error = ARUTILS_ERROR_ALLOC;

        }
        else
        {
            rfcommFtpObject = (*env)->NewGlobalRef(env, rfcommFtp);
            cancelSemObject = (*env)->NewGlobalRef(env, cancelSem);
        }
        if (rfcommFtpObject == NULL || cancelSemObject == NULL)
        {
            *error = ARUTILS_ERROR_ALLOC;
        }
        else
        {
            newConnection->rfcommFtpObject = rfcommFtpObject;
            newConnection->cancelSemObject = cancelSemObject;
        }
    }

    /* if the thread has been attached then detach the thread from the virtual machine */
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARUTILS_JNI_Manager_VM)->DetachCurrentThread(ARUTILS_JNI_Manager_VM);
    }


    return newConnection;
}

/**
 * @brief Delete an Ftp Connection
 * @warning This function frees memory
 * @param connection The address of the pointer on the Ftp Connection
 * @see ARUTILS_FtpAL_NewConnection ()
 */
void ARUTILS_RFCommFtp_Connection_Delete(ARUTILS_RFCommFtp_Connection_t **connectionAddr)
{

    eARUTILS_ERROR error = ARUTILS_OK;

    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;

    /* Check parameters */
    /**/


    /* get the environment */
    if (ARUTILS_JNI_Manager_VM != NULL)
    {
        getEnvResult = (*ARUTILS_JNI_Manager_VM)->GetEnv(ARUTILS_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);
    }
    /* if no environment then attach the thread to the virtual machine */
    if (getEnvResult == JNI_EDETACHED)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, "attach the thread to the virtual machine ...");
        (*ARUTILS_JNI_Manager_VM)->AttachCurrentThread(ARUTILS_JNI_Manager_VM, &env, NULL);
    }
    /* check the environment  */
    if (env == NULL)
    {
        error = ARUTILS_ERROR;
    }

    if ((error == ARUTILS_OK) && (connectionAddr != NULL))
    {
        ARUTILS_RFCommFtp_Connection_t *connection = *connectionAddr;
        if (connection != NULL)
        {
            // rfcommFTP java unregisterCharacteristics
            (*env)->DeleteGlobalRef (env, connection->rfcommFtpObject);
            (*env)->DeleteGlobalRef (env, connection->cancelSemObject);

            connection->rfcommFtpObject = NULL;
            connection->cancelSemObject = NULL;

            free(connection);
        }
        *connectionAddr = NULL;
    }
}

/**
 * @brief Cancel an Ftp Connection command in progress (put)
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Ftp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_RFCommFtp_Connection_Cancel(ARUTILS_RFCommFtp_Connection_t *connection)
{
    /* -- cancel the RFCommFtp connection -- */

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, " RFCOmmFtp_Connection_Cancel ");

    eARUTILS_ERROR result = ARUTILS_OK;
    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    jboolean ret = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, "%s", "");

    if (connection == NULL)
    {
        result = ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (result == ARUTILS_OK)
    {
        /* get the environment */
        if (ARUTILS_JNI_Manager_VM != NULL)
        {
            getEnvResult = (*ARUTILS_JNI_Manager_VM)->GetEnv(ARUTILS_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);
        }
        /* if no environment then attach the thread to the virtual machine */
        if (getEnvResult == JNI_EDETACHED)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, "attach the thread to the virtual machine ...");
            (*ARUTILS_JNI_Manager_VM)->AttachCurrentThread(ARUTILS_JNI_Manager_VM, &env, NULL);
        }
        /* check the environment  */
        if (env == NULL)
        {
            result = ARUTILS_ERROR;
        }
    }

    if (result == ARUTILS_OK)
    {
        jobject rfcommFtpObject = connection->rfcommFtpObject;
        jobject cancelSemObject = connection->cancelSemObject;

        ret = (*env)->CallBooleanMethod(env, rfcommFtpObject, ARUTILS_JNI_RFCOMMFTP_METHOD_CONNECTION_CANCEL, cancelSemObject);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_RFCOMMFTP_TAG, "EXCEPTION Details:");
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
            ret = JNI_FALSE;
        }

        if (ret == 0)
        {
            result = ARUTILS_ERROR_SYSTEM;
        }
    }

    return result;
}

/**
 * @brief Check if the connection has received a cancel to it's semaphore
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see cURL
 */
eARUTILS_ERROR ARUTILS_RFCommFtp_Connection_IsCanceled(ARUTILS_RFCommFtp_Connection_t *connection)
{
    eARUTILS_ERROR result = ARUTILS_OK;
    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    jboolean ret = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, "%s", "");

    if (connection == NULL)
    {
        result = ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (result == ARUTILS_OK)
    {
        /* get the environment */
        if (ARUTILS_JNI_Manager_VM != NULL)
        {
            getEnvResult = (*ARUTILS_JNI_Manager_VM)->GetEnv(ARUTILS_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);
        }
        /* if no environment then attach the thread to the virtual machine */
        if (getEnvResult == JNI_EDETACHED)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, "attach the thread to the virtual machine ...");
            (*ARUTILS_JNI_Manager_VM)->AttachCurrentThread(ARUTILS_JNI_Manager_VM, &env, NULL);
        }
        /* check the environment  */
        if (env == NULL)
        {
            result = ARUTILS_ERROR;
        }
    }

    if (result == ARUTILS_OK)
    {
        jobject rfcommFtpObject = connection->rfcommFtpObject;
        jobject cancelSemObject = connection->cancelSemObject;

        ret = (*env)->CallBooleanMethod(env, rfcommFtpObject, ARUTILS_JNI_RFCOMMFTP_METHOD_IS_CANCELED, cancelSemObject);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_RFCOMMFTP_TAG, "EXCEPTION Details:");
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
            ret = JNI_FALSE;
        }

        if (ret != 0)
        {
            result = ARUTILS_ERROR_FTP_CANCELED;
        }
    }

    return result;
}

/**
 * @brief Reset an Ftp Connection command in progress (put)
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Ftp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_RFCommFtp_Connection_Reset(ARUTILS_RFCommFtp_Connection_t *connection)
{
    /* -- cancel the RFCommFtp connection -- */

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, " RFCommFtp_Connection_Reset ");

    eARUTILS_ERROR result = ARUTILS_OK;
    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    jboolean ret = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, "%s", "");

    if (connection == NULL)
    {
        result = ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (result == ARUTILS_OK)
    {
        /* get the environment */
        if (ARUTILS_JNI_Manager_VM != NULL)
        {
            getEnvResult = (*ARUTILS_JNI_Manager_VM)->GetEnv(ARUTILS_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);
        }
        /* if no environment then attach the thread to the virtual machine */
        if (getEnvResult == JNI_EDETACHED)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, "attach the thread to the virtual machine ...");
            (*ARUTILS_JNI_Manager_VM)->AttachCurrentThread(ARUTILS_JNI_Manager_VM, &env, NULL);
        }
        /* check the environment  */
        if (env == NULL)
        {
            result = ARUTILS_ERROR;
        }
    }

    if (result == ARUTILS_OK)
    {
        jobject rfcommFtpObject = connection->rfcommFtpObject;
        jobject cancelSemObject = connection->cancelSemObject;

        ret = (*env)->CallBooleanMethod(env, rfcommFtpObject, ARUTILS_JNI_RFCOMMFTP_METHOD_CONNECTION_RESET, cancelSemObject);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_RFCOMMFTP_TAG, "EXCEPTION Details:");
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
            ret = JNI_FALSE;
        }

        if (ret == 0)
        {
            result = ARUTILS_ERROR_SYSTEM;
        }
    }

    return result;
}

/**
 * @brief Put an remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param srcFile The string of the local file name path to be put
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @param resume The resume capability requested
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_RFCommFtp_NewConnection (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_RFCommFtp_Put(ARUTILS_RFCommFtp_Connection_t *connection, const char *remotePath, const char *srcFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume)
{
    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    eARUTILS_ERROR error = ARUTILS_OK;
    ARUTILS_RFCommFtp_Command_t *callback = NULL;
    jboolean ret = 0;

    /* Check parameters */
    if ((connection == NULL) || (connection->rfcommFtpObject == NULL) || (remotePath == NULL) || (srcFile == NULL))
    {
        error = ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (error == ARUTILS_OK)
    {
        callback = calloc(1, sizeof(ARUTILS_RFCommFtp_Command_t));
        if (callback == NULL)
        {
            error = ARUTILS_ERROR;
        }
    }

    if (error == ARUTILS_OK)
    {
        callback->rfcommFtpProgressCallback = progressCallback;
        callback->rfcommProgressArg = progressArg;
    }

    if (error == ARUTILS_OK)
    {
        /* get the environment */
        if (ARUTILS_JNI_Manager_VM != NULL)
        {
            getEnvResult = (*ARUTILS_JNI_Manager_VM)->GetEnv(ARUTILS_JNI_Manager_VM, (void **) &env, JNI_VERSION_1_6);
        }
        /* if no environment then attach the thread to the virtual machine */
        if (getEnvResult == JNI_EDETACHED)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, "attach the thread to the virtual machine ...");
            (*ARUTILS_JNI_Manager_VM)->AttachCurrentThread(ARUTILS_JNI_Manager_VM, &env, NULL);
        }
        /* check the environment  */
        if (env == NULL)
        {
            error = ARUTILS_ERROR;
        }
    }

    if (error == ARUTILS_OK)
    {
        jobject rfcommFtpObject = connection->rfcommFtpObject;
        jobject cancelSemObject = connection->cancelSemObject;
        jstring jRemotePath = (*env)->NewStringUTF(env, remotePath);
        jstring jSrcFile = (*env)->NewStringUTF(env, srcFile);
        
        ret = (*env)->CallBooleanMethod(env, rfcommFtpObject, ARUTILS_JNI_RFCOMMFTP_METHOD_PUT, jRemotePath, jSrcFile, (jlong)(intptr_t)callback, resume, cancelSemObject);
        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_RFCOMMFTP_TAG, "EXCEPTION Details:");
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
            ret = JNI_FALSE;
        }
        if (ret == 0)
        {
            error = ARUTILS_ERROR_RFCOMM_FAILED;
        }

        /* Cleanup */
        if (jRemotePath != NULL)
        {
            (*env)->DeleteLocalRef(env, jRemotePath);
        }

        if (jSrcFile != NULL)
        {
            (*env)->DeleteLocalRef(env, jSrcFile);
        }
    }

    /* if the thread has been attached then detach the thread from the virtual machine */
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARUTILS_JNI_Manager_VM)->DetachCurrentThread(ARUTILS_JNI_Manager_VM);
    }

    if (callback != NULL)
    {
        free(callback);
        callback = NULL;
    }

    return error;
}

eARUTILS_ERROR ARUTILS_RFCommFtpAL_Connection_Disconnect(ARUTILS_Manager_t *manager)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, "%s", "");
    return ARUTILS_OK;
}

eARUTILS_ERROR ARUTILS_RFCommFtpAL_Connection_Reconnect(ARUTILS_Manager_t *manager)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_RFCOMMFTP_TAG, "%s", "");
    return ARUTILS_OK;
}

eARUTILS_ERROR ARUTILS_RFCommFtpAL_Connection_Cancel(ARUTILS_Manager_t *manager)
{
    return ARUTILS_RFCommFtp_Connection_Cancel((ARUTILS_RFCommFtp_Connection_t *)manager->connectionObject);
}

eARUTILS_ERROR ARUTILS_RFCommFtpAL_Connection_IsCanceled(ARUTILS_Manager_t *manager)
{
    return ARUTILS_RFCommFtp_Connection_IsCanceled((ARUTILS_RFCommFtp_Connection_t *)manager->connectionObject);
}

eARUTILS_ERROR ARUTILS_RFCommFtpAL_Connection_Reset(ARUTILS_Manager_t *manager)
{
    return ARUTILS_RFCommFtp_Connection_Reset((ARUTILS_RFCommFtp_Connection_t *)manager->connectionObject);
}

eARUTILS_ERROR ARUTILS_RFCommFtpAL_Put(ARUTILS_Manager_t *manager, const char *namePath, const char *srcFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume)
{
    return ARUTILS_RFCommFtp_Put((ARUTILS_RFCommFtp_Connection_t *)manager->connectionObject, namePath, srcFile, progressCallback, progressArg, resume);
}
