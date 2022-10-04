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
#include <libARMavlink/ARMAVLINK_FileParser.h>

/*****************************************
 *
 *             private header:
 *
 *****************************************/

int Load_ARMavlink_Java_Exception(JNIEnv *env);
void DeLoad_ARMavlink_Java_Exception(JNIEnv *env);
void Throw_Mavlink_Exception(JNIEnv *env, eARMAVLINK_ERROR nativeError);

#define ARMAVLINK_JNIMAVLINK_TAG "ARMAVLINK_JNIFileParser" /** tag used by the print of the file */

JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkFileParser_nativeNew (JNIEnv *env, jobject obj)
{

    ARMAVLINK_FileParser_t *nativeFileParser = NULL;
    eARMAVLINK_ERROR result = ARMAVLINK_OK;

    nativeFileParser = ARMAVLINK_FileParser_New(&result);
    int error = Load_ARMavlink_Java_Exception(env);

    if (error != JNI_OK)
    {
        result = ARMAVLINK_ERROR;
    }
    if (result != ARMAVLINK_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "error: %d occurred", result);
        Throw_Mavlink_Exception(env, result);
    }

    return (long)nativeFileParser;
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkFileParser_nativeDelete (JNIEnv *env, jobject obj, jlong fileParserPtr)
{
	ARMAVLINK_FileParser_t *nativeFileParser = (ARMAVLINK_FileParser_t*) (intptr_t) fileParserPtr;

    ARMAVLINK_FileParser_Delete (&nativeFileParser);

    DeLoad_ARMavlink_Java_Exception(env);
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkFileParser_nativeParse (JNIEnv *env, jobject obj, jlong fileParserPtr, jstring path, jlong missionListPtr)
{
	ARMAVLINK_FileParser_t *nativeFileParser = (ARMAVLINK_FileParser_t*) (intptr_t) fileParserPtr;
	mission_item_list_t *nativeMissionList = (mission_item_list_t*) (intptr_t) missionListPtr;
    const char *filePath = (*env)->GetStringUTFChars(env, path, 0);

	eARMAVLINK_ERROR result = ARMAVLINK_FileParser_Parse(nativeFileParser, filePath, nativeMissionList);

    if (result != ARMAVLINK_OK)
    {
        ARSAL_PRINT (ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "error: %d occurred", result);
        Throw_Mavlink_Exception(env, result);
    }
}
