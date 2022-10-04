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

package com.parrot.arsdk.armavlink;

import com.parrot.arsdk.arsal.ARSALPrint;

public class ARMavlinkMissionItem
{
    private static String TAG = ARMavlinkMissionItem.class.getSimpleName();

    private static native int nativeCreateMavlinkMissionItemWithAllParams(long nativeItem, float param1, float param2, float param3, float param4, float latitude, float longitude, float altitude, int command, int seq,  int frame, int current, int autocontinue);
    private static native int nativeCreateMavlinkNavWaypointMissionItem(long nativeItem, float latitude, float longitude, float altitude, float yaw);
    private static native int nativeCreateMavlinkNavWaypointMissionItemWithRadius(long nativeItem, float latitude, float longitude, float altitude, float radius);
    private static native int nativeCreateMavlinkLandMissionItem(long nativeItem, float latitude, float longitude, float altitude, float yaw);
    private static native int nativeCreateMavlinkChangeSpeedMissionItem(long nativeItem, int groundSpeed, float speed, float throttle);
    private static native int nativeCreateMavlinkTakeoffMissionItem(long nativeItem, float latitude, float longitude, float altitude, float yaw, float pitch);

    private static native int nativeCreateMavlinkVideoStartCapture(long nativeItem, int cameraId, float framesPerSeconds, float resolution);
    private static native int nativeCreateMavlinkVideoStopCapture(long nativeItem);
    private static native int nativeCreateMavlinkImageStartCapture(long nativeItem, float period,float imagesCount,float resolution);
    private static native int nativeCreateMavlinkImageStopCapture(long nativeItem);
    private static native int nativeCreateMavlinkCreatePanorama(long nativeItem, float horizontalAngle, float verticalAngle, float horizontalRotationSpeed, float verticalRotationSpeed);
    private static native int nativeCreateMavlinkDelay(long nativeItem, float delayDuration);
    private static native int nativeCreateMavlinkSetROI(long nativeItem, int roiMode, int missionIndex, int roiIndex, float latitude, float longitude, float altitude);
    private static native int nativeCreateMavlinkSetViewMode(long nativeItem, int viewModeType, int roiIndex);
    private static native int nativeCreateMavlinkSetPictureMode(long nativeItem, int captureMode, float interval);
    private static native int nativeCreateMavlinkSetPhotoSensors(long nativeItem, int sensorsBitfield);

    private native long nativeNew();
    private native long nativeDelete(long nativeItem);

    private native void nativeSetParam1(long nativeItem,float param1);
    private native void nativeSetParam2(long nativeItem,float param2);
    private native void nativeSetParam3(long nativeItem,float param3);
    private native void nativeSetParam4(long nativeItem,float param4);
    private native void nativeSetX(long nativeItem,float x);
    private native void nativeSetY(long nativeItem,float y);
    private native void nativeSetZ(long nativeItem,float z);
    private native void nativeSetSeq(long nativeItem,int seq);
    private native void nativeSetCommand(long nativeItem,int command);
    private native void nativeSetTargetSystem(long nativeItem,int target_system);
    private native void nativeSetTargetComponent(long nativeItem,int target_component);
    private native void nativeSetFrame(long nativeItem,int frame);
    private native void nativeSetCurrent(long nativeItem,int current);
    private native void nativeSetAutocontinue(long nativeItem,int autocontinue);

    private native float nativeGetParam1(long nativeItem);
    private native float nativeGetParam2(long nativeItem);
    private native float nativeGetParam3(long nativeItem);
    private native float nativeGetParam4(long nativeItem);
    private native float nativeGetX(long nativeItem);
    private native float nativeGetY(long nativeItem);
    private native float nativeGetZ(long nativeItem);
    private native int nativeGetSeq(long nativeItem);
    private native int nativeGetCommand(long nativeItem);
    private native int nativeGetTargetSystem(long nativeItem);
    private native int nativeGetTargetComponent(long nativeItem);
    private native int nativeGetFrame(long nativeItem);
    private native int nativeGetCurrent(long nativeItem);
    private native int nativeGetAutocontinue(long nativeItem);

    private float param1; //< PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
    private float param2; //< PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
    private float param3; //< PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
    private float param4; //< PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
    private float x; //< PARAM5 / local: x position, global: latitude
    private float y; //< PARAM6 / y position: global: longitude
    private float z; //< PARAM7 / z position: global: altitude
    private int seq; //< Sequence
    private int command; //< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
    private int target_system; //< System ID
    private int target_component; //< Component ID
    private int frame; //< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
    private boolean current; //< false:0, true:1
    private int autocontinue; //< autocontinue to next wp

    private long nativeMissionItem = 0;
    private boolean isCreateByMe;

    /**
     * ARMavlinkMissionItem constructor
     * Can't be Instantiated by user
     */
    private ARMavlinkMissionItem ()
    {
        nativeMissionItem = nativeNew();
        isCreateByMe = true;
    }

    /**
     * ARMavlinkMissionItem constructor
     * @param itemPtr ARMavlinkMissionItem native Pointre
     */
    public ARMavlinkMissionItem (long itemPtr)
    {
        nativeMissionItem = itemPtr;
        isCreateByMe = false;
    }

    public long getNativePointer()
    {
        return nativeMissionItem;
    }

    /**
     * Dispose NetworkIOBufferParam
     */
    public void dispose() 
    {
        if(nativeMissionItem != 0 && isCreateByMe) 
        {
            nativeDelete(nativeMissionItem);
            nativeMissionItem = 0;
        }
    }

    /**
     * Destructor
     */
    public void finalize () throws Throwable 
    {
        try 
        {
            dispose ();
        } 
        finally 
        {
            super.finalize ();
        }
    }

    /**
     * create a ARMavlinkMissionItem with all the given params
     * @param param1 Radius in which the MISSION is accepted as reached, in meters
     * @param param2 Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
     * @param param3 For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
     * @param param4 For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
     * @param latitude the latitude of the mission item
     * @param longitude the longitude of the mission item
     * @param altitude the altitude of the mission item
     * @param command the command of the mission item
     * @param seq the seq of the mission item
     * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
     * @param current false:0, true:1
     * @param autocontinue autocontinue to next wp
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkMissionItemWithAllParams(float param1, float param2, float param3, float param4, float latitude, float longitude, float altitude, int command, int seq,  int frame, int current, int autocontinue)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkMissionItemWithAllParams(missionItem.getNativePointer(), param1, param2, param3, param4, latitude, longitude, altitude, command, seq, frame, current, autocontinue);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * create a ARMavlinkMissionItem of Command MAV_CMD_NAV_WAYPOINT with the given/default params
     * @param latitude (saved in x) the latitude of the mission item
     * @param longitude (saved in y) the longitude of the mission item
     * @param altitude (saved in z) the altitude of the mission item
     * @param yaw (saved in param4) the yaw of the mission item
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkNavWaypointMissionItem(float latitude, float longitude, float altitude, float yaw)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkNavWaypointMissionItem(missionItem.getNativePointer(), latitude, longitude, altitude, yaw);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * create a ARMavlinkMissionItem of Command MAV_CMD_NAV_WAYPOINT with the given/default params
     * @param latitude (saved in x) the latitude of the mission item
     * @param longitude (saved in y) the longitude of the mission item
     * @param altitude (saved in z) the altitude of the mission item
     * @param radius the radius to pass by WP (in meters). 0 to pass through the WP.
 *   *               Positive value for clockwise orbit, negative value for counter-clockwise orbit.
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkNavWaypointMissionItemWithRadius(float latitude, float longitude, float altitude, float radius)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkNavWaypointMissionItemWithRadius(missionItem.getNativePointer(), latitude, longitude, altitude, radius);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * create a ARMavlinkMissionItem of Command MAV_CMD_NAV_LAND with the given/default params  
     * @param latitude (saved in x) the latitude of the mission item
     * @param longitude (saved in y) the longitude of the mission item
     * @param altitude (saved in z) the altitude of the mission item
     * @param yaw (saved in param4) the yaw of the mission item
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkLandMissionItem(float latitude, float longitude, float altitude, float yaw)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkLandMissionItem(missionItem.getNativePointer(), latitude, longitude, altitude, yaw);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * create a ARMavlinkMissionItem of Command MAV_CMD_DO_CHANGE_SPEED with the given/default params
     * @param groundSpeed (saved in param1) 1 if ground speed, 0 if airspeed
     * @param speed (saved in param2) the speed of the mission item
     * @param throttle (saved in param3) throttle in percent, -1 indicates no changes
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkChangeSpeedMissionItem(int groundSpeed, float speed, float throttle)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkChangeSpeedMissionItem(missionItem.getNativePointer(), groundSpeed, speed, throttle);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * create a ARMavlinkMissionItem of Command MAV_CMD_NAV_TAKEOFF with the given/default params 
     * @param latitude (saved in x) the latitude of the mission item
     * @param longitude (saved in y) the longitude of the mission item
     * @param altitude (saved in z) the altitude of the mission item
     * @param yaw (saved in param4) the yaw of the mission item
     * @param pitch (saved in param1) desired pitch
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkTakeoffMissionItem(float latitude, float longitude, float altitude, float yaw, float pitch)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkTakeoffMissionItem(missionItem.getNativePointer(), latitude, longitude, altitude, yaw, pitch);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * create a ARMavlinkMissionItem of Command MAV_CMD_VIDEO_START_CAPTURE with the given/default params 
     * @param cameraId (saved in param1)
     * @param framesPerSeconds (saved in param2)
     * @param resolution (saved in param3)
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkVideoStartCaptureMissionItem(int cameraId, float framesPerSeconds, float resolution)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkVideoStartCapture(missionItem.getNativePointer(), cameraId, framesPerSeconds, resolution);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * create a ARMavlinkMissionItem of Command MAV_CMD_VIDEO_STOP_CAPTURE 
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkVideoStopCaptureMissionItem()
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkVideoStopCapture(missionItem.getNativePointer());

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }
    /**
     * create a ARMavlinkMissionItem of Command MAV_CMD_IMAGE_START_CAPTURE with the given/default params
     * @param period (saved in param1)
     * @param imagesCount (saved in param2)
     * @param resolution (saved in param3)
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkImageStartCaptureMissionItem(float period, float imagesCount, float resolution)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkImageStartCapture(missionItem.getNativePointer(), period, imagesCount, resolution);
        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);
        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * create a ARMavlinkMissionItem of Command MAV_CMD_CONDITION_DELAY with the given/default params
     * @param durationDelay (saved in param1)
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkDelay(float durationDelay)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkDelay(missionItem.getNativePointer(), durationDelay);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * create a ARMavlinkMissionItem of Command MAV_CMD_IMAGE_STOP_CAPTURE 
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkImageStopCaptureMissionItem()
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkImageStopCapture(missionItem.getNativePointer());

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * create a ARMavlinkMissionItem of Command MAV_CMD_PANORAMA_CREATE with the given/default params 
     *
     * This item will start a move by the drone or its camera on the yaw/pan axis and on the tilt when it will
     * be read by the drone
     *
     * Note that, the first vertical angle will be applied relatively to the physical center of the camera.
     *
     * @param horizontalAngle the horizontal relative angle (deg) (saved in param1)
     * @param verticalAngle the vertical relative angle (deg) (negative angle moves the camera down)
     *   (saved in param2)
     * @param horizontalRotationSpeed the desired horizontal rotation speed (m/s) (saved in param3)
     * @param verticalRotationSpeed the desired vertical rotation speed (m/s) (saved in param4)
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkCreatePanoramaMissionItem(float horizontalAngle, float verticalAngle, float horizontalRotationSpeed, float verticalRotationSpeed)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkCreatePanorama(missionItem.getNativePointer(), horizontalAngle, verticalAngle, horizontalRotationSpeed, verticalRotationSpeed);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * Fill a set view mode mission item with the given params and the default params 
     * This item will set the way the drone should behave regarding its orientation between two waypoints
     * @param mode  The mode of the ROI (see MAV_ROI) (currently must be MAV_ROI_LOCATION)
     * @param missionIndex   The index of the mission or the target id (used if mode is MAV_ROI_TARGET or MAV_ROI_WPINDEX)
     * @param roiIndex   The index of roi. This is used to make a reference to this ROI in others mission items
     * @param latitude   the latitude of the ROI (used if mode is MAV_ROI_LOCATION)
     * @param longitude   the longitude of the ROI (used if mode is MAV_ROI_LOCATION)
     * @param altitude   the altitude of the ROI (used if mode is MAV_ROI_LOCATION)
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkSetROI(MAV_ROI mode, int missionIndex, int roiIndex, float latitude, float longitude, float altitude)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkSetROI(missionItem.getNativePointer(), mode.ordinal(), missionIndex, roiIndex, latitude, longitude, altitude);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * Fill a set view mode mission item with the given params and the default params
     * This item will set the way the drone should behave regarding its orientation between two waypoints
     * @param viewModeType  The type of the view mode (see MAV_VIEW_MODE_TYPE)
     * @param roiIndex   The index of the ROI to follow if type is VIEW_MODE_TYPE_ROI. Ignored otherwise.
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkSetViewMode(MAV_VIEW_MODE_TYPE viewModeType, int roiIndex)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkSetViewMode(missionItem.getNativePointer(), viewModeType.ordinal(), roiIndex);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * Fill a set picture mode mission item with the given params and the default params
     * This item will set the still capture mode. Only use if the target is equiped by a Sequoia. This item will be
     * ignored otherwise.
     * @param mode  The mode chosen (see MAV_STILL_CAPTURE_MODE_TYPE)
     * @param interval If mode is STILL_CAPTURE_MODE_TYPE_TIMELAPSE, interval is in milliseconds.
     *                 If mode is STILL_CAPTURE_MODE_TYPE_GPS_POSITION, interval is in centimeters.
     *                 If mode is STILL_CAPTURE_MODE_TYPE_AUTOMATIC_OVERLAP, interval is in overlapping percentage.
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkSetPictureMode(MAV_STILL_CAPTURE_MODE_TYPE mode, float interval)
    {
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkSetPictureMode(missionItem.getNativePointer(), mode.ordinal(), interval);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * Fill a set photo sensors mission item with the given params and the default params
     * This item will set the photo sensors that should be used to take a picture.
     * Only use if the target is equiped by a Sequoia. This item will be ignored otherwise.
     * @param sensors  list of sensors that should be used (see MAV_PHOTO_SENSORS_FLAG)
     * @return {@link ARMavlinkMissionItem} mission item
     */
    public static ARMavlinkMissionItem CreateMavlinkSetPhotoSensors(MAV_PHOTO_SENSORS_FLAG... sensors)
    {
        int bitfield = 0;
        for (MAV_PHOTO_SENSORS_FLAG sensor : sensors) {
            bitfield |= sensor.getValue();
        }
        ARMavlinkMissionItem missionItem = new ARMavlinkMissionItem();
        int value = nativeCreateMavlinkSetPhotoSensors(missionItem.getNativePointer(), bitfield);

        ARMAVLINK_ERROR_ENUM error = ARMAVLINK_ERROR_ENUM.getFromValue(value);

        if(error == ARMAVLINK_ERROR_ENUM.ARMAVLINK_OK)
        {
            missionItem.updateFromNative();
        }
        else
        {
            ARSALPrint.e (TAG, "Create Mavlink Mission Item Error : "+ error.toString());
            missionItem.dispose();
            missionItem = null;
        }
        return missionItem;
    }

    /**
     * Update the java information from native part
     */
    public void updateFromNative()
    {
        if(nativeMissionItem != 0)
        {
            param1 = nativeGetParam1(nativeMissionItem);
            param2 = nativeGetParam2(nativeMissionItem);
            param3 = nativeGetParam3(nativeMissionItem);
            param4 = nativeGetParam4(nativeMissionItem);
            x = nativeGetX(nativeMissionItem);
            y = nativeGetY(nativeMissionItem);
            z = nativeGetZ(nativeMissionItem);
            seq = nativeGetSeq(nativeMissionItem);
            command = nativeGetCommand(nativeMissionItem);
            target_system = nativeGetTargetSystem(nativeMissionItem);
            target_component = nativeGetTargetComponent(nativeMissionItem);
            frame = nativeGetFrame(nativeMissionItem);
            if(nativeGetCurrent(nativeMissionItem) == 1) current = true;
            else current = false;
            autocontinue = nativeGetAutocontinue(nativeMissionItem);
        }
        
    }

    public void updateCommandCode()
    {
        if(nativeMissionItem != 0){
            this.command = nativeGetCommand(nativeMissionItem);
        } 
    }

    public float getParam1() 
    {
        return param1;
    }

    public float getParam1FromNative() 
    {
        float retVal = -1;
        if(nativeMissionItem != 0){
            retVal = nativeGetParam1(nativeMissionItem);
        }
        return retVal;
    }

    public void setParam1(float param1) {
        if(nativeMissionItem !=0)
        {
            this.param1 = param1;
            nativeSetParam1(nativeMissionItem, param1); 
        } 
    }

    public float getParam2() {
        return param2;
    }

    public float getParam2FromNative() 
    {
        float retVal = -1;
        if(nativeMissionItem != 0){
            retVal = nativeGetParam2(nativeMissionItem);
        }
        return retVal;
    }

    public void setParam2(float param2) {
        if(nativeMissionItem !=0)
        {
            this.param2 = param2;
            nativeSetParam2(nativeMissionItem, param2); 
        } 
    }

    public float getParam3() {
        return param3;
    }

    public float getParam3FromNative() 
    {
        float retVal = -1;
        if(nativeMissionItem != 0){
            retVal = nativeGetParam3(nativeMissionItem);
        }
        return retVal;
    }

    public void setParam3(float param3) {
        if(nativeMissionItem !=0)
        {
            this.param3 = param3;
            nativeSetParam3(nativeMissionItem, param3); 
        } 
    }

    public float getParam4() {
        return param4;
    }

    public float getParam4FromNative() 
    {
        float retVal = -1;
        if(nativeMissionItem != 0){
            retVal = nativeGetParam4(nativeMissionItem);
        }
        return retVal;
    }

    public void setParam4(float param4) {
        if(nativeMissionItem !=0)
        {
            this.param4 = param4;
            nativeSetParam4(nativeMissionItem, param4); 
        } 
    }

    public float getX() {
        return x;
    }

    public float getXFromNative() 
    {
        float retVal = -1;
        if(nativeMissionItem != 0){
            retVal = nativeGetX(nativeMissionItem);
        }
        return retVal;
    }

    public void setX(float x) {
        if(nativeMissionItem !=0)
        {
            this.x = x;
            nativeSetX(nativeMissionItem, x); 
        } 
    }

    public float getY() {
        return y;
    }

    public float getYFromNative() 
    {
        float retVal = -1;
        if(nativeMissionItem != 0){
            retVal = nativeGetY(nativeMissionItem);
        }
        return retVal;
    }

    public void setY(float y) {
        if(nativeMissionItem !=0)
        {
            this.y = y;
            nativeSetY(nativeMissionItem, y); 
        } 
    }

    public float getZ() {
        return z;
    }

    public float getZFromNative() 
    {
        float retVal = -1;
        if(nativeMissionItem != 0){
            retVal = nativeGetZ(nativeMissionItem);
        }
        return retVal;
    }

    public void setZ(float z) {
        if(nativeMissionItem !=0)
        {
            this.z = z;
            nativeSetZ(nativeMissionItem, z); 
        }
    }

    public int getSeq() {
        return seq;
    }

    public void setSeq(int seq) {
        if(nativeMissionItem !=0)
        {
            this.seq = seq;
            nativeSetSeq(nativeMissionItem, seq); 
        }
    }

    public int getCommand() {
        return command;
    }

    public void setCommand(int command) {
        if(nativeMissionItem !=0)
        {
            this.command = command;
            nativeSetCommand(nativeMissionItem, command); 
        }
    }

    public int getTargetSystem() {
        return target_system;
    }

    public void setTargetSystem(int target_system) {
        if(nativeMissionItem !=0)
        {
            this.target_system = target_system;
            nativeSetTargetSystem(nativeMissionItem, target_system); 
        }
    }

    public int getTargetComponent() {
        return target_component;
    }

    public void setTargetComponent(int target_component) {
        if(nativeMissionItem !=0)
        {
            this.target_component = target_component;
            nativeSetTargetComponent(nativeMissionItem, target_component); 
        }
    }

    public int getFrame() {
        return frame;
    }

    public void setFrame(int frame) {
        if(nativeMissionItem !=0)
        {
            this.frame = frame;
            nativeSetFrame(nativeMissionItem, frame); 
        }
    }

    public boolean isCurrent() {
        return current;
    }

    public void setCurrent(boolean current) {
        if(nativeMissionItem !=0)
        {
            this.current = current;
            if(current) nativeSetCurrent(nativeMissionItem, 1);
            else nativeSetCurrent(nativeMissionItem, 0);
        }
    }

    public int getAutocontinue() {
        return autocontinue;
    }

    public void setAutocontinue(int autocontinue) {
        if(nativeMissionItem !=0)
        {
            this.autocontinue = autocontinue;
            nativeSetAutocontinue(nativeMissionItem, autocontinue); 
        }
    }

    @Override
    public String toString() {
        return "FPTimelineFragment [param1=" + param1 + ", param2=" + param2
                + ", param3=" + param3 + ", param4=" + param4 + ", x=" + x
                + ", y=" + y + ", z=" + z + ", seq=" + seq + ", command="
                + command + ", target_system=" + target_system
                + ", target_component=" + target_component + ", frame=" + frame
                + ", current=" + current + ", autocontinue=" + autocontinue
                + "]";
    }

}
