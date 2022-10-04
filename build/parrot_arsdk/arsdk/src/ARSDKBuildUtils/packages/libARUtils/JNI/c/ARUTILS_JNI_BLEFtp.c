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
 * @file  ARNETWORKAL_JNIBLENetwork.c
 * @brief private headers of BLE network manager allow to send over ble network.
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
#include "ARUTILS_JNI_BLEFtp.h"
//#include "../../Sources/BLE/ARNETWORKAL_BLENetwork.h"

/*****************************************
 *
 *             define :
 *
 *****************************************/

#define ARUTILS_JNI_BLEFTP_TAG "JNIBLEFtp"

extern JavaVM *ARUTILS_JNI_Manager_VM; /** reference to the java virtual machine */

static jmethodID ARUTILS_JNI_BLEFTP_METHOD_CONNECTION_CANCEL;
static jmethodID ARUTILS_JNI_BLEFTP_METHOD_IS_CANCELED;
static jmethodID ARUTILS_JNI_BLEFTP_METHOD_CONNECTION_RESET;
static jmethodID ARUTILS_JNI_BLEFTP_METHOD_FTP_LIST;
static jmethodID ARUTILS_JNI_BLEFTP_METHOD_FTP_SIZE;
static jmethodID ARUTILS_JNI_BLEFTP_METHOD_GET_WITH_BUFFER;
static jmethodID ARUTILS_JNI_BLEFTP_METHOD_GET;
static jmethodID ARUTILS_JNI_BLEFTP_METHOD_PUT;
static jmethodID ARUTILS_JNI_BLEFTP_METHOD_DELETE;
static jmethodID ARUTILS_JNI_BLEFTP_METHOD_RENAME;



/*****************************************
 *
 *             implementation :
 *
 *****************************************/


JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arutils_ARUtilsBLEFtp_nativeJNIInit(JNIEnv *env, jobject obj)
{
    /* -- initialize the JNI part -- */
    /* load the references of java methods */

    jclass jBLEFtpCls = (*env)->FindClass(env, "com/parrot/arsdk/arutils/ARUtilsBLEFtp");

    ARUTILS_JNI_BLEFTP_METHOD_CONNECTION_CANCEL = (*env)->GetMethodID(env, jBLEFtpCls, "cancelFileAL", "(Ljava/util/concurrent/Semaphore;)Z");
    ARUTILS_JNI_BLEFTP_METHOD_IS_CANCELED = (*env)->GetMethodID(env, jBLEFtpCls, "isConnectionCanceledAL", "(Ljava/util/concurrent/Semaphore;)Z");
    ARUTILS_JNI_BLEFTP_METHOD_CONNECTION_RESET = (*env)->GetMethodID(env, jBLEFtpCls, "resetConnectionAL", "(Ljava/util/concurrent/Semaphore;)Z");
    ARUTILS_JNI_BLEFTP_METHOD_FTP_LIST = (*env)->GetMethodID(env, jBLEFtpCls, "listFilesAL", "(Ljava/lang/String;[Ljava/lang/String;)Z");
    ARUTILS_JNI_BLEFTP_METHOD_FTP_SIZE = (*env)->GetMethodID(env, jBLEFtpCls, "sizeFileAL", "(Ljava/lang/String;[D)Z");
    ARUTILS_JNI_BLEFTP_METHOD_GET_WITH_BUFFER = (*env)->GetMethodID(env, jBLEFtpCls, "getFileWithBufferAL", "(Ljava/lang/String;[[BJZLjava/util/concurrent/Semaphore;)Z");
    ARUTILS_JNI_BLEFTP_METHOD_GET = (*env)->GetMethodID(env, jBLEFtpCls, "getFileAL", "(Ljava/lang/String;Ljava/lang/String;JZLjava/util/concurrent/Semaphore;)Z");
    ARUTILS_JNI_BLEFTP_METHOD_PUT = (*env)->GetMethodID(env, jBLEFtpCls, "putFileAL", "(Ljava/lang/String;Ljava/lang/String;JZLjava/util/concurrent/Semaphore;)Z");
    ARUTILS_JNI_BLEFTP_METHOD_DELETE = (*env)->GetMethodID(env, jBLEFtpCls, "deleteFileAL", "(Ljava/lang/String;)Z");
    ARUTILS_JNI_BLEFTP_METHOD_RENAME = (*env)->GetMethodID(env, jBLEFtpCls, "renameFileAL", "(Ljava/lang/String;Ljava/lang/String;)Z");

    /* cleanup */
    (*env)->DeleteLocalRef (env, jBLEFtpCls);
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_arutils_ARUtilsBLEFtp_nativeProgressCallback(JNIEnv *env, jobject obj, jlong jCallback, jfloat percent)
{
    ARUTILS_BLEFtp_Command_t * callback = (ARUTILS_BLEFtp_Command_t*)(intptr_t) jCallback;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, " %p %f ", callback, percent);
    if (callback != NULL && callback->bleFtpProgressCallback != NULL)
    {
        callback->bleFtpProgressCallback(callback->bleProgressArg, percent);
    }
}

/**
 * @brief Create a new Ftp Connection
 * @warning This function allocates memory
 * @param cancelSem The pointer of the Ftp get/put cancel semaphore or null
 * @param device The BLE Ftp device
 * @param[out] error The pointer of the error code: if success ARUTILS_OK, otherwise an error number of eARUTILS_ERROR
 * @retval On success, returns an ARUTILS_FtpAL_Connection_t. Otherwise, it returns null.
 * @see ARUTILS_FtpAL_DeleteConnection ()
 */
ARUTILS_BLEFtp_Connection_t * ARUTILS_BLEFtp_Connection_New(jobject bleFtp, jobject cancelSem, eARUTILS_ERROR *error)
{

    jobject bleFtpObject = NULL;
    jobject cancelSemObject = NULL;

    ARUTILS_BLEFtp_Connection_t *newConnection = NULL;

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
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
        (*ARUTILS_JNI_Manager_VM)->AttachCurrentThread(ARUTILS_JNI_Manager_VM, &env, NULL);
    }
    /* check the environment  */
    if (env == NULL)
    {
        *error = ARUTILS_ERROR;
    }

    if (*error == ARUTILS_OK)
    {
        newConnection = calloc(1, sizeof(ARUTILS_BLEFtp_Connection_t));
        if (newConnection == NULL)
        {
            *error = ARUTILS_ERROR_ALLOC;

        }
        else
        {
            bleFtpObject = (*env)->NewGlobalRef(env, bleFtp);
            cancelSemObject = (*env)->NewGlobalRef(env, cancelSem);
        }
        if (bleFtpObject == NULL || cancelSemObject == NULL)
        {
            *error = ARUTILS_ERROR_ALLOC;
        }
        else
        {
            newConnection->bleFtpObject = bleFtpObject;
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
void ARUTILS_BLEFtp_Connection_Delete(ARUTILS_BLEFtp_Connection_t **connectionAddr)
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
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
        (*ARUTILS_JNI_Manager_VM)->AttachCurrentThread(ARUTILS_JNI_Manager_VM, &env, NULL);
    }
    /* check the environment  */
    if (env == NULL)
    {
        error = ARUTILS_ERROR;
    }

    if ((error == ARUTILS_OK) && (connectionAddr != NULL))
    {
        ARUTILS_BLEFtp_Connection_t *connection = *connectionAddr;
        if (connection != NULL)
        {
            // bleFTP java unregisterCharacteristics
            (*env)->DeleteGlobalRef (env, connection->bleFtpObject);
            (*env)->DeleteGlobalRef (env, connection->cancelSemObject);

            connection->bleFtpObject = NULL;
            connection->cancelSemObject = NULL;

            free(connection);
        }
        *connectionAddr = NULL;
    }
}

/**
 * @brief Cancel an Ftp Connection command in progress (get, put, list etc)
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Ftp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_BLEFtp_Connection_Cancel(ARUTILS_BLEFtp_Connection_t *connection)
{
    /* -- cancel the BLEFtp connection -- */

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, " BLEFtp_Connection_Cancel ");

    eARUTILS_ERROR result = ARUTILS_OK;
    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    jboolean ret = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "%s", "");

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
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
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
        jobject bleFtpObject = connection->bleFtpObject;
        jobject cancelSemObject = connection->cancelSemObject;

        ret = (*env)->CallBooleanMethod(env, bleFtpObject, ARUTILS_JNI_BLEFTP_METHOD_CONNECTION_CANCEL, cancelSemObject);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_BLEFTP_TAG, "EXCEPTION Details:");
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
eARUTILS_ERROR ARUTILS_BLEFtp_Connection_IsCanceled(ARUTILS_BLEFtp_Connection_t *connection)
{
    eARUTILS_ERROR result = ARUTILS_OK;
    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    jboolean ret = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "%s", "");

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
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
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
        jobject bleFtpObject = connection->bleFtpObject;
        jobject cancelSemObject = connection->cancelSemObject;

        ret = (*env)->CallBooleanMethod(env, bleFtpObject, ARUTILS_JNI_BLEFTP_METHOD_IS_CANCELED, cancelSemObject);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_BLEFTP_TAG, "EXCEPTION Details:");
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
 * @brief Reset an Ftp Connection command in progress (get, put, list etc)
 * @param connection The address of the pointer on the Ftp Connection
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Ftp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_BLEFtp_Connection_Reset(ARUTILS_BLEFtp_Connection_t *connection)
{
    /* -- cancel the BLEFtp connection -- */

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, " BLEFtp_Connection_Reset ");

    eARUTILS_ERROR result = ARUTILS_OK;
    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    jboolean ret = 0;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "%s", "");

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
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
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
        jobject bleFtpObject = connection->bleFtpObject;
        jobject cancelSemObject = connection->cancelSemObject;

        ret = (*env)->CallBooleanMethod(env, bleFtpObject, ARUTILS_JNI_BLEFTP_METHOD_CONNECTION_RESET, cancelSemObject);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_BLEFTP_TAG, "EXCEPTION Details:");
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
 * @brief Execute Ftp List command to retrieve directory content
 * @warning This function allocates memory
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the directory path on the remote Ftp server
 * @param resultList The pointer of the string of the directory content null terminated
 * @param resultListLen The pointer of the lenght of the resultList string including null terminated
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_BLEFtp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_BLEFtp_List(ARUTILS_BLEFtp_Connection_t *connection, const char *remotePath, char **resultList, uint32_t *resultListLen)
{

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, " BLEFtp_list ");

    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    eARUTILS_ERROR error = ARUTILS_OK;
    jboolean ret = 0;

    /* Check parameters */
    if ((connection == NULL) || (remotePath == NULL) || (resultList == NULL) || (connection->bleFtpObject == NULL) )
    {
        error = ARUTILS_ERROR_BAD_PARAMETER;
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
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
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
        jobject bleFtpObject = connection->bleFtpObject;
        jstring jRemotePath = (*env)->NewStringUTF(env, remotePath);
        jclass objectClass = (*env)->FindClass(env, "java/lang/String");
        jobjectArray objectList = (*env)->NewObjectArray(env, 1, objectClass, NULL);

        ret = (*env)->CallBooleanMethod(env, bleFtpObject, ARUTILS_JNI_BLEFTP_METHOD_FTP_LIST, jRemotePath, objectList);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_BLEFTP_TAG, "EXCEPTION Details:");
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
            ret = JNI_FALSE;
        }

        if (ret == 0)
        {
            error = ARUTILS_ERROR_BLE_FAILED;
        }
        else
        {
            jstring jList = (jstring) (*env)->GetObjectArrayElement(env, objectList, 0);

            const char *dataList;

            if (jList != NULL)
            {
                dataList = (*env)->GetStringUTFChars(env, jList, 0);
            }
            else
            {
                dataList = "";
            }

            int dataLen = 0;
            if (dataList != NULL)
            {
                dataLen = strlen(dataList);
            }

            char *stringList = malloc(dataLen + 1);
            if (stringList == NULL)
            {
                error = ARUTILS_ERROR_ALLOC;
            }

            if (error == ARUTILS_OK)
            {
                if (dataLen != 0)
                {
                    memcpy(stringList, dataList, dataLen);
                }
                stringList[dataLen] = '\0';

                *resultList = stringList;
                *resultListLen = dataLen + 1;
            }

            if (dataList != NULL && jList != NULL)
            {
                (*env)->ReleaseStringUTFChars(env, jList, dataList);
            }
            if (jList != NULL)
            {
                (*env)->DeleteLocalRef(env, jList);
            }
        }

        /* Cleanup */
        if (jRemotePath != NULL)
        {
            (*env)->DeleteLocalRef(env, jRemotePath);
        }

        if (objectList != NULL)
        {
            (*env)->DeleteLocalRef(env, objectList);
        }

        if (objectClass != NULL)
        {
            (*env)->DeleteLocalRef(env, objectClass);
        }

    }

    /* if the thread has been attached then detach the thread from the virtual machine */
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARUTILS_JNI_Manager_VM)->DetachCurrentThread(ARUTILS_JNI_Manager_VM);
    }

    return error;
}

/**
 * @brief Execute Ftp Size command to retrieve file size
 * @warning This function allocates memory
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the directory path on the remote Ftp server
 * @param fileSize The retuned file size
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_BLEFtp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_BLEFtp_Size(ARUTILS_BLEFtp_Connection_t *connection, const char *remotePath, double *fileSize)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, " BLEFtp_size ");

    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    eARUTILS_ERROR error = ARUTILS_OK;
    jboolean ret = 0;

    /* Check parameters */
    if ((connection == NULL) || (remotePath == NULL) || (fileSize == NULL) || (connection->bleFtpObject == NULL) )
    {
        error = ARUTILS_ERROR_BAD_PARAMETER;
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
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
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
        jobject bleFtpObject = connection->bleFtpObject;
        jstring jRemotePath = (*env)->NewStringUTF(env, remotePath);
        jdoubleArray objectSizeArray = (*env)->NewDoubleArray(env, 1);
        
        ret = (*env)->CallBooleanMethod(env, bleFtpObject, ARUTILS_JNI_BLEFTP_METHOD_FTP_SIZE, jRemotePath, objectSizeArray);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_BLEFTP_TAG, "EXCEPTION Details:");
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
            ret = JNI_FALSE;
        }

        if (ret == 0)
        {
            error = ARUTILS_ERROR_BLE_FAILED;
        }
        else
        {
            jdouble *jDoubleArray = (jdouble*) (*env)->GetDoubleArrayElements(env, objectSizeArray, JNI_FALSE);
            
            if (jDoubleArray != NULL)
            {
                *fileSize = (double)*jDoubleArray;
            }
            else
            {
                error = ARUTILS_ERROR_BLE_FAILED;
            }
            
            if (jDoubleArray != NULL)
            {
                (*env)->ReleaseDoubleArrayElements(env, objectSizeArray, jDoubleArray, JNI_ABORT);
            }
        }

        /* Cleanup */
        if (jRemotePath != NULL)
        {
            (*env)->DeleteLocalRef(env, jRemotePath);
        }

        if (objectSizeArray != NULL)
        {
            (*env)->DeleteLocalRef(env, objectSizeArray);
        }
    }

    /* if the thread has been attached then detach the thread from the virtual machine */
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARUTILS_JNI_Manager_VM)->DetachCurrentThread(ARUTILS_JNI_Manager_VM);
    }

    return error;
}

/**
 * @brief Delete an remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_Ftp_NewConnection ()
 */
eARUTILS_ERROR ARUTILS_BLEFtp_Delete(ARUTILS_BLEFtp_Connection_t *connection, const char *remotePath)
{

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, " BLEFtp_Delete ");

    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    eARUTILS_ERROR error = ARUTILS_OK;
    jboolean ret = 0;

    /* Check parameters */
    if ((connection == NULL) || (connection->bleFtpObject == NULL) || (remotePath == NULL))
    {
        error = ARUTILS_ERROR_BAD_PARAMETER;
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
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
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
        jobject bleFtpObject = connection->bleFtpObject;
        jstring jRemotePath = (*env)->NewStringUTF(env, remotePath);

        ret = (*env)->CallBooleanMethod(env, bleFtpObject, ARUTILS_JNI_BLEFTP_METHOD_DELETE, jRemotePath);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_BLEFTP_TAG, "EXCEPTION Details:");
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
            ret = JNI_FALSE;
        }

        if (ret == 0)
        {
            error = ARUTILS_ERROR_BLE_FAILED;
        }

        /* Cleanup */
        if (jRemotePath != NULL)
        {
            (*env)->DeleteLocalRef(env, jRemotePath);
        }
    }
    /* if the thread has been attached then detach the thread from the virtual machine */
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARUTILS_JNI_Manager_VM)->DetachCurrentThread(ARUTILS_JNI_Manager_VM);
    }
    return error;
}

/**
 * @brief Get an remote Ftp server file
 * @warning This function allocates memory
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param data The pointer of the data buffer of the file data
 * @param dataLen The pointer of the length of the data buffer
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_BLEFtp_NewConnection (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_BLEFtp_Get_WithBuffer(ARUTILS_BLEFtp_Connection_t *connection, const char *remotePath, uint8_t **data, uint32_t *dataLen,  ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, " BLEFtp_Get_WithBuffer ");

    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    eARUTILS_ERROR error = ARUTILS_OK;
    ARUTILS_BLEFtp_Command_t *callback = NULL;
    jboolean ret = 0;

    /* Check parameters */
    if ((connection == NULL) || (connection->bleFtpObject == NULL) || (remotePath == NULL))
    {
        error = ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (error == ARUTILS_OK)
    {
        callback = calloc(1, sizeof(ARUTILS_BLEFtp_Command_t));
        if (callback == NULL)
        {
            error = ARUTILS_ERROR;
        }
    }

    if (error == ARUTILS_OK)
    {
        callback->bleFtpProgressCallback = progressCallback;
        callback->bleProgressArg = progressArg;
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
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
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
        jobject bleFtpObject = connection->bleFtpObject;
        jobject cancelSemObject = connection->cancelSemObject;
        jstring jRemotePath = (*env)->NewStringUTF(env, remotePath);
        jclass objectClass = (*env)->FindClass(env, "[B");
        jboolean wantProgress = callback->bleFtpProgressCallback != NULL;

        jobjectArray dataArray = (*env)->NewObjectArray(env, 1, objectClass, NULL);

        ret = (*env)->CallBooleanMethod(env, bleFtpObject, ARUTILS_JNI_BLEFTP_METHOD_GET_WITH_BUFFER, jRemotePath, dataArray, (jlong)(intptr_t)callback, wantProgress, cancelSemObject);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_BLEFTP_TAG, "EXCEPTION Details:");
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
            ret = JNI_FALSE;
        }

        if (ret == 0)
        {
            error = ARUTILS_ERROR_BLE_FAILED;
        }
        else
        {
            jbyteArray javaByteArray = (jbyteArray) (*env)->GetObjectArrayElement(env, dataArray, 0);
            if (javaByteArray != NULL)
            {
                int size = (*env)->GetArrayLength(env, javaByteArray);
                jbyte* javaData = (*env)->GetByteArrayElements(env, javaByteArray, NULL);
                if (size > 0 && javaData != NULL)
                {
                    *data = malloc(size * sizeof(uint8_t));
                    memcpy(*data, javaData, size);
                }
                else
                {
                    error = ARUTILS_ERROR_SYSTEM;
                }
                *dataLen = size;

                if (javaData != NULL)
                {
                    (*env)->ReleaseByteArrayElements(env, javaByteArray, javaData, JNI_ABORT);
                }
                (*env)->DeleteLocalRef(env, javaByteArray);
            }
            else
            {
                //we come here if the file content is empty
                error = ARUTILS_ERROR_SYSTEM;
            }

        }
        if (dataArray != NULL)
        {
            (*env)->DeleteLocalRef(env, dataArray);
        }

        /* Cleanup */
        if (jRemotePath != NULL)
        {
            (*env)->DeleteLocalRef(env, jRemotePath);
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

/**
 * @brief Get an remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param dstFile The string of the local file name path to be get
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @param resume The resume capability requested
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_BLEFtp_NewConnection (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_BLEFtp_Get(ARUTILS_BLEFtp_Connection_t *connection, const char *remotePath, const char *dstFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, " BLEFtp_Get ");

    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    eARUTILS_ERROR error = ARUTILS_OK;
    ARUTILS_BLEFtp_Command_t *callback = NULL;
    jboolean ret = 0;

    /* Check parameters */
    if ((connection == NULL) || (connection->bleFtpObject == NULL) || (remotePath == NULL) || (dstFile == NULL))
    {
        error = ARUTILS_ERROR_BAD_PARAMETER;
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
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
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
        callback = calloc(1, sizeof(ARUTILS_BLEFtp_Command_t));
        if (callback == NULL)
        {
            error = ARUTILS_ERROR;
        }
    }

    if (error == ARUTILS_OK)
    {
        callback->bleFtpProgressCallback = progressCallback;
        callback->bleProgressArg = progressArg;
    }

    if (error == ARUTILS_OK)
    {
        jobject bleFtpObject = connection->bleFtpObject;
        jobject cancelSemObject = connection->cancelSemObject;
        jstring jRemotePath = (*env)->NewStringUTF(env, remotePath);
        jstring jDstFile = (*env)->NewStringUTF(env, dstFile);
        jboolean wantProgress = callback->bleFtpProgressCallback != NULL;

        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, " %p %p", callback, callback->bleFtpProgressCallback);
        ret = (*env)->CallBooleanMethod(env, bleFtpObject, ARUTILS_JNI_BLEFTP_METHOD_GET, jRemotePath, jDstFile, (jlong)(intptr_t)callback, wantProgress, cancelSemObject);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_BLEFTP_TAG, "EXCEPTION Details:");
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
            ret = JNI_FALSE;
        }

        if (ret == 0)
        {
            error = ARUTILS_ERROR_BLE_FAILED;
        }

        /* Cleanup */
        if (jRemotePath != NULL)
        {
            (*env)->DeleteLocalRef(env, jRemotePath);
        }

        if (jDstFile != NULL)
        {
            (*env)->DeleteLocalRef(env, jDstFile);
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

/**
 * @brief Put an remote Ftp server file
 * @param connection The address of the pointer on the Ftp Connection
 * @param namePath The string of the file name path on the remote Ftp server
 * @param srcFile The string of the local file name path to be put
 * @param progressCallback The progress callback function
 * @param progressArg The progress callback function arg
 * @param resume The resume capability requested
 * @retval On success, returns ARUTILS_OK. Otherwise, it returns an error number of eARUTILS_ERROR.
 * @see ARUTILS_BLEFtp_NewConnection (), ARUTILS_Ftp_ProgressCallback_t, eARUTILS_FTP_RESUME
 */
eARUTILS_ERROR ARUTILS_BLEFtp_Put(ARUTILS_BLEFtp_Connection_t *connection, const char *remotePath, const char *srcFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, " BLEFtp_Put ");

    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    eARUTILS_ERROR error = ARUTILS_OK;
    ARUTILS_BLEFtp_Command_t *callback = NULL;
    jboolean ret = 0;

    /* Check parameters */
    if ((connection == NULL) || (connection->bleFtpObject == NULL) || (remotePath == NULL) || (srcFile == NULL))
    {
        error = ARUTILS_ERROR_BAD_PARAMETER;
    }

    if (error == ARUTILS_OK)
    {
        callback = calloc(1, sizeof(ARUTILS_BLEFtp_Command_t));
        if (callback == NULL)
        {
            error = ARUTILS_ERROR;
        }
    }

    if (error == ARUTILS_OK)
    {
        callback->bleFtpProgressCallback = progressCallback;
        callback->bleProgressArg = progressArg;
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
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
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
        jobject bleFtpObject = connection->bleFtpObject;
        jobject cancelSemObject = connection->cancelSemObject;
        jstring jRemotePath = (*env)->NewStringUTF(env, remotePath);
        jstring jSrcFile = (*env)->NewStringUTF(env, srcFile);

        ret = (*env)->CallBooleanMethod(env, bleFtpObject, ARUTILS_JNI_BLEFTP_METHOD_PUT, jRemotePath, jSrcFile, (jlong)(intptr_t)callback, resume, cancelSemObject);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_BLEFTP_TAG, "EXCEPTION Details:");
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
            ret = JNI_FALSE;
        }

        if (ret == 0)
        {
            error = ARUTILS_ERROR_BLE_FAILED;
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

eARUTILS_ERROR ARUTILS_BLEFtp_Rename(ARUTILS_BLEFtp_Connection_t *connection, const char *oldNamePath, const char *newNamePath)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, " connection = %p, oldNamePath = %s, newNamePath = %s ", connection, oldNamePath, newNamePath);
    /* local declarations */
    JNIEnv *env = NULL;
    jint getEnvResult = JNI_OK;
    eARUTILS_ERROR error = ARUTILS_OK;
    jboolean ret = 0;

    /* Check parameters */
    if ((connection == NULL) || (connection->bleFtpObject == NULL) || (oldNamePath == NULL) || (newNamePath == NULL))
    {
        error = ARUTILS_ERROR_BAD_PARAMETER;
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
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "attach the thread to the virtual machine ...");
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
        jobject bleFtpObject = connection->bleFtpObject;
        jstring jOldNamePath = (*env)->NewStringUTF(env, oldNamePath);
        jstring jNewNamePath = (*env)->NewStringUTF(env, newNamePath);

        ret = (*env)->CallBooleanMethod(env, bleFtpObject, ARUTILS_JNI_BLEFTP_METHOD_RENAME, jOldNamePath, jNewNamePath);

        if ((*env)->ExceptionOccurred(env))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARUTILS_JNI_BLEFTP_TAG, "EXCEPTION Details:");
            (*env)->ExceptionDescribe(env);
            (*env)->ExceptionClear(env);
            ret = JNI_FALSE;
        }

        if (ret == 0)
        {
            error = ARUTILS_ERROR_BLE_FAILED;
        }

        /* Cleanup */
        if (jOldNamePath != NULL)
        {
            (*env)->DeleteLocalRef(env, jOldNamePath);
        }

        if (jNewNamePath != NULL)
        {
            (*env)->DeleteLocalRef(env, jNewNamePath);
        }
    }

    /* if the thread has been attached then detach the thread from the virtual machine */
    if ((getEnvResult == JNI_EDETACHED) && (env != NULL))
    {
        (*ARUTILS_JNI_Manager_VM)->DetachCurrentThread(ARUTILS_JNI_Manager_VM);
    }

    return error;
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_Connection_Disconnect(ARUTILS_Manager_t *manager)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "%s", "");
    return ARUTILS_OK;
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_Connection_Reconnect(ARUTILS_Manager_t *manager)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARUTILS_JNI_BLEFTP_TAG, "%s", "");
    return ARUTILS_OK;
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_Connection_Cancel(ARUTILS_Manager_t *manager)
{
    return ARUTILS_BLEFtp_Connection_Cancel((ARUTILS_BLEFtp_Connection_t *)manager->connectionObject);
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_Connection_IsCanceled(ARUTILS_Manager_t *manager)
{
    return ARUTILS_BLEFtp_Connection_IsCanceled((ARUTILS_BLEFtp_Connection_t *)manager->connectionObject);
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_Connection_Reset(ARUTILS_Manager_t *manager)
{
    return ARUTILS_BLEFtp_Connection_Reset((ARUTILS_BLEFtp_Connection_t *)manager->connectionObject);
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_List(ARUTILS_Manager_t *manager, const char *namePath, char **resultList, uint32_t *resultListLen)
{
    return ARUTILS_BLEFtp_List((ARUTILS_BLEFtp_Connection_t *)manager->connectionObject, namePath, resultList, resultListLen);
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_Size(ARUTILS_Manager_t *manager, const char *namePath, double *fileSize)
{
    return ARUTILS_BLEFtp_Size((ARUTILS_BLEFtp_Connection_t *)manager->connectionObject, namePath, fileSize);
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_Get_WithBuffer(ARUTILS_Manager_t *manager, const char *namePath, uint8_t **data, uint32_t *dataLen,  ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg)
{
    return ARUTILS_BLEFtp_Get_WithBuffer((ARUTILS_BLEFtp_Connection_t *)manager->connectionObject, namePath, data, dataLen, progressCallback, progressArg);
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_Get(ARUTILS_Manager_t *manager, const char *namePath, const char *dstFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume)
{
    return ARUTILS_BLEFtp_Get((ARUTILS_BLEFtp_Connection_t *)manager->connectionObject, namePath, dstFile, progressCallback, progressArg, resume);
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_Put(ARUTILS_Manager_t *manager, const char *namePath, const char *srcFile, ARUTILS_Ftp_ProgressCallback_t progressCallback, void* progressArg, eARUTILS_FTP_RESUME resume)
{
    return ARUTILS_BLEFtp_Put((ARUTILS_BLEFtp_Connection_t *)manager->connectionObject, namePath, srcFile, progressCallback, progressArg, resume);
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_Delete(ARUTILS_Manager_t *manager, const char *namePath)
{
    return ARUTILS_BLEFtp_Delete((ARUTILS_BLEFtp_Connection_t *)manager->connectionObject, namePath);
}

eARUTILS_ERROR ARUTILS_BLEFtpAL_Rename(ARUTILS_Manager_t *manager, const char *oldNamePath, const char *newNamePath)
{
    return ARUTILS_BLEFtp_Rename((ARUTILS_BLEFtp_Connection_t *)manager->connectionObject, oldNamePath, newNamePath);
}
