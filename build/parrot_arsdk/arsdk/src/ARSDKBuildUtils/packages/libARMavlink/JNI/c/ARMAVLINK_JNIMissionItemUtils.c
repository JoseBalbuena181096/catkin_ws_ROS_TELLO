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
 * @file ARMAVLINK_JNIMissionItemUtils.c
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

#include <libARMavlink/ARMAVLINK_MissionItemUtils.h>


/*****************************************
 *
 *             private header:
 *
 *****************************************/

#define ARMAVLINK_JNIMAVLINK_TAG "ARMAVLINK_JNIMissionItemUtils" /** tag used by the print of the file */

JNIEXPORT jlong JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeNew (JNIEnv *env, jobject obj)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - Create MissionItem ptr");
    mavlink_mission_item_t* item = malloc (sizeof (mavlink_mission_item_t));
    return (long) item;
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeDelete (JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - delete MissionItem ptr");
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    free(item);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkMissionItemWithAllParams (JNIEnv *env, jclass class, jlong missionItemPtr, jfloat param1, jfloat param2, jfloat param3, jfloat param4, jfloat latitude, jfloat longitude, jfloat altitude, jint command, jint seq,  jint frame, jint current, jint autocontinue)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    return ARMAVLINK_MissionItemUtils_CreateMavlinkMissionItemWithAllParams(item, param1, param2, param3, param4, latitude, longitude, altitude, command, seq, frame, current, autocontinue);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkNavWaypointMissionItem (JNIEnv *env, jclass class, jlong missionItemPtr, jfloat latitude, jfloat longitude, jfloat altitude, jfloat yaw)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - CreateMavlinkNavWaypointMissionItem");
    return ARMAVLINK_MissionItemUtils_CreateMavlinkNavWaypointMissionItem(item, latitude, longitude, altitude, yaw);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkNavWaypointMissionItemWithRadius (JNIEnv *env, jclass class, jlong missionItemPtr, jfloat latitude, jfloat longitude, jfloat altitude, jfloat radius)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - CreateMavlinkNavWaypointMissionItem (with radius)");
    return ARMAVLINK_MissionItemUtils_CreateMavlinkNavWaypointMissionItemWithRadius(item, latitude, longitude, altitude, radius);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkLandMissionItem (JNIEnv *env, jclass class, jlong missionItemPtr, jfloat latitude, jfloat longitude, jfloat altitude, jfloat yaw)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    return ARMAVLINK_MissionItemUtils_CreateMavlinkLandMissionItem(item, latitude, longitude, altitude, yaw);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkTakeoffMissionItem (JNIEnv *env, jclass class, jlong missionItemPtr, jfloat latitude, jfloat longitude, jfloat altitude, jfloat yaw, jfloat pitch)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    return ARMAVLINK_MissionItemUtils_CreateMavlinkTakeoffMissionItem(item, latitude, longitude, altitude, yaw, pitch);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkChangeSpeedMissionItem (JNIEnv *env, jclass class, jlong missionItemPtr, jint groundSpeed, jfloat speed, jfloat throttle)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    return ARMAVLINK_MissionItemUtils_CreateMavlinkChangeSpeedMissionItem(item, groundSpeed, speed, throttle);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkVideoStartCapture (JNIEnv *env, jclass class, jlong missionItemPtr, jint cameraId, jfloat framesPerSeconds, jfloat resolution)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - CreateMavlinkVideoStartCapture");
    return ARMAVLINK_MissionItemUtils_CreateMavlinkVideoStartCapture(item, cameraId, framesPerSeconds, resolution);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkVideoStopCapture (JNIEnv *env, jclass class, jlong missionItemPtr)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - CreateMavlinkVideoStopCapture");
    return ARMAVLINK_MissionItemUtils_CreateMavlinkVideoStopCapture(item);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkImageStartCapture (JNIEnv *env, jclass class, jlong missionItemPtr, jfloat period,jfloat imagesCount,jfloat resolution)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    return ARMAVLINK_MissionItemUtils_CreateMavlinkImageStartCapture(item,  period, imagesCount, resolution);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkImageStopCapture (JNIEnv *env, jclass class, jlong missionItemPtr)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    return ARMAVLINK_MissionItemUtils_CreateMavlinkImageStopCapture(item);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkCreatePanorama (JNIEnv *env, jclass class, jlong missionItemPtr, jfloat horizontalAngle,jfloat verticalAngle,jfloat horizontalRotationSpeed, jfloat verticalRotationSpeed)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - CreateMavlinkCreatePanorama");
    return ARMAVLINK_MissionItemUtils_CreateMavlinkCreatePanorama(item, horizontalAngle, verticalAngle, horizontalRotationSpeed, verticalRotationSpeed);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkDelay (JNIEnv *env, jclass class, jlong missionItemPtr, jfloat delayDuration)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - CreateMavlinkCreatePanorama");
    return ARMAVLINK_MissionItemUtils_CreateMavlinkDelay(item, delayDuration);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkSetROI (JNIEnv *env, jclass class, jlong missionItemPtr, jint roiMode, jint missionIndex, jint roiIndex, jfloat latitude, jfloat longitude, jfloat altitude)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - CreateMavlinkSetROI");
    return ARMAVLINK_MissionItemUtils_CreateMavlinkSetROI(item, roiMode, missionIndex, roiIndex, latitude, longitude, altitude);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkSetViewMode (JNIEnv *env, jclass class, jlong missionItemPtr, jint viewModeType, jint roiIndex)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - CreateMavlinkSetViewMode");
    return ARMAVLINK_MissionItemUtils_CreateMavlinkSetViewMode(item, viewModeType, roiIndex);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkSetPictureMode (JNIEnv *env, jclass class, jlong missionItemPtr, jint captureMode, jfloat interval)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - CreateMavlinkSetPictureMode");
    return ARMAVLINK_MissionItemUtils_CreateMavlinkSetPictureMode(item, captureMode, interval);
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeCreateMavlinkSetPhotoSensors (JNIEnv *env, jclass class, jlong missionItemPtr, jint sensorsBitfield)
{
    mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARMAVLINK_JNIMAVLINK_TAG, "native - CreateMavlinkSetPhotoSensors");
    return ARMAVLINK_MissionItemUtils_CreateMavlinkSetPhotoSensors(item, sensorsBitfield);
}

/*****************************************
 *
 *             Setters
 *
 *****************************************/
JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetParam1(JNIEnv *env, jobject obj, jlong missionItemPtr, jfloat param1)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        item->param1 = param1;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetParam2(JNIEnv *env, jobject obj, jlong missionItemPtr, jfloat param2)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        item->param2 = param2;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetParam3(JNIEnv *env, jobject obj, jlong missionItemPtr, jfloat param3)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        item->param3 = param3;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetParam4(JNIEnv *env, jobject obj, jlong missionItemPtr, jfloat param4)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        item->param4 = param4;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetX(JNIEnv *env, jobject obj, jlong missionItemPtr, jfloat x)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        item->x = x;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetY(JNIEnv *env, jobject obj, jlong missionItemPtr, jfloat y)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        item->y = y;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetZ(JNIEnv *env, jobject obj, jlong missionItemPtr, jfloat z)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        item->z = z;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetSeq(JNIEnv *env, jobject obj, jlong missionItemPtr, jint seq)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        uint16_t sequence = (uint16_t) seq;
        item->seq = sequence;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetCommand(JNIEnv *env, jobject obj, jlong missionItemPtr, jint command)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        uint16_t com = (uint16_t) command;
        item->command = com;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetTargetSystem(JNIEnv *env, jobject obj, jlong missionItemPtr, jint systemId)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        uint8_t target_system = (uint8_t) systemId;
        item->target_system = target_system;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetTargetComponent(JNIEnv *env, jobject obj, jlong missionItemPtr, jint componentId)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        uint8_t target_component = (uint8_t) componentId;
        item->target_component = target_component;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetFrame(JNIEnv *env, jobject obj, jlong missionItemPtr, jint frame)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        uint8_t mFrame = (uint8_t) frame;
        item->frame = mFrame;
    }
}


JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetCurrent(JNIEnv *env, jobject obj, jlong missionItemPtr, jint curr)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        uint8_t current = (uint8_t) curr;
        item->current = current;
    }
}

JNIEXPORT void JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeSetAutocontinue(JNIEnv *env, jobject obj, jlong missionItemPtr, jint autocon)
{
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        uint8_t autocontinue = (uint8_t) autocon;
        item->autocontinue = autocontinue;
    }
}

/*****************************************
 *
 *             Getters
 *
 *****************************************/

JNIEXPORT jfloat JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetParam1(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jfloat retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->param1;
    }
    return retVal;
}

JNIEXPORT jfloat JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetParam2(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jfloat retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->param2;
    }
    return retVal;
}

JNIEXPORT jfloat JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetParam3(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jfloat retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->param3;
    }
    return retVal;
}

JNIEXPORT jfloat JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetParam4(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jfloat retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->param4;
    }
    return retVal;
}

JNIEXPORT jfloat JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetX(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jfloat retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->x;
    }
    return retVal;
}

JNIEXPORT jfloat JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetY(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jfloat retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->y;
    }
    return retVal;
}

JNIEXPORT jfloat JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetZ(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jfloat retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->z;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetSeq(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jint retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->seq;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetCommand(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jint retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->command;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetTargetSystem(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jint retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->target_system;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetTargetComponent(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jint retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->target_component;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetFrame(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jint retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->frame;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetCurrent(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jint retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->current;
    }
    return retVal;
}

JNIEXPORT jint JNICALL
Java_com_parrot_arsdk_armavlink_ARMavlinkMissionItem_nativeGetAutocontinue(JNIEnv *env, jobject obj, jlong missionItemPtr)
{
    jint retVal = -1;
    if (missionItemPtr != 0)
    {
        mavlink_mission_item_t *item = (mavlink_mission_item_t*) (intptr_t) missionItemPtr;
        retVal = item->autocontinue;
    }
    return retVal;
}
