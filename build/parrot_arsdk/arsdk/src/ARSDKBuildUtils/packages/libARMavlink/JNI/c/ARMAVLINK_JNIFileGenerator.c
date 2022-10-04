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
 * @file ARMAVLINK_JNIFileGenerator.c
 * @brief 
 * @date 
 * @author 
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <jni.h>
#include <stdlib.h>

#include <libARSAL/ARSAL_Print.h>
#include <libARMavlink/ARMAVLINK_Error.h>
#include <libARMavlink/ARMAVLINK_FileGenerator.h>

#define JNI_FAILED -1

jclass Mavlink_Exception_Class_Ref = NULL;
jmethodID Mavlink_Exception_Init_MethodeId = NULL;
int FileGenerator_Instance_Count = 0;
/*****************************************
 *
 *             private header:
 *
 *****************************************/

#define ARMAVLINK_JNIMAVLINK_TAG "ARMAVLINK_JNIFileGenerator" /** tag used by the print of the file */

int Load_ARMavlink_Java_Exception(JNIEnv *env);
void DeLoad_ARMavlink_Java_Exception(JNIEnv *env);
void Throw_Mavlink_Exception(JNIEnv *env, eARMAVLINK_ERROR nativeError);

JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkFileGenerator_nativeNew (JNIEnv *env, jobject obj)
{

    ARMAVLINK_FileGenerator_t *nativeFileGenerator = NULL;
    eARMAVLINK_ERROR result = ARMAVLINK_OK;

    nativeFileGenerator = ARMAVLINK_FileGenerator_New(&result);
    int error = Load_ARMavlink_Java_Exception(env);

    if (error != JNI_OK)
    {
        result = ARMAVLINK_ERROR;
    }
    if (result == ARMAVLINK_OK)
    {
        FileGenerator_Instance_Count++;
    }else{
        ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "error: %d occurred", result);
        Throw_Mavlink_Exception(env, result);
    }

    return (long)nativeFileGenerator;
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkFileGenerator_nativeDelete (JNIEnv *env, jobject obj, jlong fileGeneratorPtr)
{
	ARMAVLINK_FileGenerator_t *nativeFileGenerator = (ARMAVLINK_FileGenerator_t*) (intptr_t) fileGeneratorPtr;

    ARMAVLINK_FileGenerator_Delete (&nativeFileGenerator);

    if (FileGenerator_Instance_Count > 0)
    {
        FileGenerator_Instance_Count--;
        if (FileGenerator_Instance_Count == 0)
        {
            DeLoad_ARMavlink_Java_Exception(env);
        }
    }
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkFileGenerator_nativeAddMissionItem (JNIEnv *env, jobject obj, jlong fileGeneratorPtr, jlong missionItemPtr)
{
	ARMAVLINK_FileGenerator_t *nativeFileGenerator = (ARMAVLINK_FileGenerator_t*) (intptr_t) fileGeneratorPtr;
	mavlink_mission_item_t *nativeMissionItem = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
	eARMAVLINK_ERROR result = ARMAVLINK_FileGenerator_AddMissionItem(nativeFileGenerator, nativeMissionItem);
	return result;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkFileGenerator_nativeReplaceMissionItem (JNIEnv *env, jobject obj, jlong fileGeneratorPtr, jlong missionItemPtr, jint index)
{
	ARMAVLINK_FileGenerator_t *nativeFileGenerator = (ARMAVLINK_FileGenerator_t*) (intptr_t) fileGeneratorPtr;
	mavlink_mission_item_t *nativeMissionItem = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
	eARMAVLINK_ERROR result = ARMAVLINK_FileGenerator_ReplaceMissionItem(nativeFileGenerator, nativeMissionItem, index);
	return result;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkFileGenerator_nativeInsertMissionItem (JNIEnv *env, jobject obj, jlong fileGeneratorPtr, jlong missionItemPtr, jint index)
{
	ARMAVLINK_FileGenerator_t *nativeFileGenerator = (ARMAVLINK_FileGenerator_t*) (intptr_t) fileGeneratorPtr;
	mavlink_mission_item_t *nativeMissionItem = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
	eARMAVLINK_ERROR result = ARMAVLINK_FileGenerator_InsertMissionItem(nativeFileGenerator, nativeMissionItem, index);
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - InsertMissionItem ");
	return result;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkFileGenerator_nativeDeleteMissionItem (JNIEnv *env, jobject obj, jlong fileGeneratorPtr, jint index)
{
	ARMAVLINK_FileGenerator_t *nativeFileGenerator = (ARMAVLINK_FileGenerator_t*) (intptr_t) fileGeneratorPtr;
	eARMAVLINK_ERROR result = ARMAVLINK_FileGenerator_DeleteMissionItem(nativeFileGenerator, index);
	return result;
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkFileGenerator_nativeCreateMavlinkFile (JNIEnv *env, jobject obj, jlong fileGeneratorPtr, jstring path)
{
	ARMAVLINK_FileGenerator_t *nativeFileGenerator = (ARMAVLINK_FileGenerator_t*) (intptr_t) fileGeneratorPtr;
	const char *filePath = (*env)->GetStringUTFChars(env, path, 0);
	ARMAVLINK_FileGenerator_CreateMavlinkFile(nativeFileGenerator, filePath);
}

JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkFileGenerator_nativeGetCurrentMissionItemList (JNIEnv *env, jobject obj, jlong fileGeneratorPtr)
{
	ARMAVLINK_FileGenerator_t *nativeFileGenerator = (ARMAVLINK_FileGenerator_t*) (intptr_t) fileGeneratorPtr;
	eARMAVLINK_ERROR result = ARMAVLINK_OK;
    mission_item_list_t * itemList = ARMAVLINK_FileGenerator_GetCurrentMissionItemList (nativeFileGenerator, &result);

    if (result != ARMAVLINK_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "error: %d occurred", result);
        Throw_Mavlink_Exception(env, result);
    }
    ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "get mission item list: %lu ", (long) itemList);
    return (long) itemList;
}

/*****************************************
 *
 *             Private implementation:
 *
 *****************************************/
int Load_ARMavlink_Java_Exception(JNIEnv *env)
{
    jclass locClassUPDATER_Exception = NULL;
    int error = JNI_OK;

    if (Mavlink_Exception_Class_Ref == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "%s", "");

        if (env == NULL)
        {
           error = JNI_FAILED;
        }

        if (error == JNI_OK)
        {
            locClassUPDATER_Exception  = (*env)->FindClass(env, "com/parrot/arsdk/armavlink/ARMavlinkException");

            if (locClassUPDATER_Exception == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "ARMavlinkException class not found");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            Mavlink_Exception_Class_Ref = (*env)->NewGlobalRef(env,locClassUPDATER_Exception);
            if (Mavlink_Exception_Class_Ref == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "ARMavlinkException global ref failed");
                error = JNI_FAILED;
            }
        }

        if (error == JNI_OK)
        {
            Mavlink_Exception_Init_MethodeId = (*env)->GetMethodID(env, Mavlink_Exception_Class_Ref, "<init>", "(I)V");
            if (Mavlink_Exception_Init_MethodeId == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "init method not found");
                error = JNI_FAILED;
            }
        }
    }
    return error;
}

void DeLoad_ARMavlink_Java_Exception(JNIEnv *env)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "DeLoad_ARMavlink_Java_Exception");

    if (env != NULL)
    {
        if (Mavlink_Exception_Class_Ref != NULL)
        {
            (*env)->DeleteGlobalRef(env, Mavlink_Exception_Class_Ref);
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "DeLoad_ARMavlink DeleteGlobalRef");
            Mavlink_Exception_Class_Ref = NULL;
        }

        Mavlink_Exception_Init_MethodeId = NULL;
    }
}

jobject Create_New_Mavlink_Exception(JNIEnv *env, eARMAVLINK_ERROR nativeError)
{
    jobject jException = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "%d", nativeError);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        error = Load_ARMavlink_Java_Exception(env);
    }

    if (error == JNI_OK)
    {
        jException = (*env)->NewObject(env, Mavlink_Exception_Class_Ref, Mavlink_Exception_Init_MethodeId, nativeError);
    }

    return jException;
}

void Throw_Mavlink_Exception(JNIEnv *env, eARMAVLINK_ERROR nativeError)
{
    jthrowable jThrowable = NULL;
    int error = JNI_OK;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "%d", error);

    if (env == NULL)
    {
       error = JNI_FAILED;
    }

    if (error == JNI_OK)
    {
        jThrowable = Create_New_Mavlink_Exception(env, nativeError);

        if (jThrowable == NULL)
        {
           error = JNI_FAILED;
        }
    }

    if (error == JNI_OK)
    {
        (*env)->Throw(env, jThrowable);
    }
}
