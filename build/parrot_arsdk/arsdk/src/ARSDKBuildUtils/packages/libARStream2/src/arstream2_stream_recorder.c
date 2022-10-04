/**
 * @file arstream2_stream_recorder.c
 * @brief Parrot Streaming Library - Stream Recorder
 * @date 06/01/2016
 * @author aurelien.barre@parrot.com
 */

#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <math.h>

#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Mutex.h>
#if BUILD_LIBARMEDIA
#include <libARMedia/ARMedia.h>
#endif

#include "arstream2_stream_recorder.h"


#define ARSTREAM2_STREAM_RECORDER_TAG "ARSTREAM2_StreamRecorder"

#define ARSTREAM2_STREAM_RECORDER_FIFO_COND_TIMEOUT_MS (500)
#define ARSTREAM2_STREAM_RECORDER_FILE_SYNC_MAX_INTERVAL (30)


//TODO: metadata definitions should be removed when the definitions will be available in a public ARSDK library

#define ARSTREAM2_STREAM_RECORDER_GPS_ALTITUDE_MASK   (0xFFFFFF00)                          /**< GPS altitude mask */
#define ARSTREAM2_STREAM_RECORDER_GPS_ALTITUDE_SHIFT  (8)                                   /**< GPS altitude shift */
#define ARSTREAM2_STREAM_RECORDER_ALTITUDE_MASK       (0xFFFFFF00)                          /**< Altitude mask */
#define ARSTREAM2_STREAM_RECORDER_ALTITUDE_SHIFT      (8)                                   /**< Altitude shift */
#define ARSTREAM2_STREAM_RECORDER_GPS_SV_COUNT_MASK   (0x000000FF)                          /**< GPS SV count mask */
#define ARSTREAM2_STREAM_RECORDER_GPS_SV_COUNT_SHIFT  (0)                                   /**< GPS SV count shift */
#define ARSTREAM2_STREAM_RECORDER_FLYING_STATE_MASK   (0x7F)                                /**< Flying state mask */
#define ARSTREAM2_STREAM_RECORDER_FLYING_STATE_SHIFT  (0)                                   /**< Flying state shift */
#define ARSTREAM2_STREAM_RECORDER_BINNING_MASK        (0x80)                                /**< Binning mask */
#define ARSTREAM2_STREAM_RECORDER_BINNING_SHIFT       (7)                                   /**< Binning shift */
#define ARSTREAM2_STREAM_RECORDER_PILOTING_MODE_MASK  (0x7F)                                /**< Piloting mode mask */
#define ARSTREAM2_STREAM_RECORDER_PILOTING_MODE_SHIFT (0)                                   /**< Piloting mode shift */
#define ARSTREAM2_STREAM_RECORDER_ANIMATION_MASK      (0x80)                                /**< Animation mask */
#define ARSTREAM2_STREAM_RECORDER_ANIMATION_SHIFT     (7)                                   /**< Animation shift */

/**
 * @brief Flying states.
 */
typedef enum
{
    ARSTREAM2_STREAM_RECORDER_FLYING_STATE_LANDED = 0,       /**< Landed state */
    ARSTREAM2_STREAM_RECORDER_FLYING_STATE_TAKINGOFF,        /**< Taking off state */
    ARSTREAM2_STREAM_RECORDER_FLYING_STATE_HOVERING,         /**< Hovering state */
    ARSTREAM2_STREAM_RECORDER_FLYING_STATE_FLYING,           /**< Flying state */
    ARSTREAM2_STREAM_RECORDER_FLYING_STATE_LANDING,          /**< Landing state */
    ARSTREAM2_STREAM_RECORDER_FLYING_STATE_EMERGENCY,        /**< Emergency state */

} ARSTREAM2_STREAM_RECORDER_FlyingState_t;


/**
 * @brief Flying states.
 */
typedef enum
{
    ARSTREAM2_STREAM_RECORDER_PILOTING_MODE_MANUAL = 0,      /**< Manual piloting by the user */
    ARSTREAM2_STREAM_RECORDER_PILOTING_MODE_RETURN_HOME,     /**< Automatic return home in progress */
    ARSTREAM2_STREAM_RECORDER_PILOTING_MODE_FLIGHT_PLAN,     /**< Automatic flight plan in progress */
    ARSTREAM2_STREAM_RECORDER_PILOTING_MODE_FOLLOW_ME,       /**< Automatic "follow-me" in progress */

} ARSTREAM2_STREAM_RECORDER_PilotingMode_t;


/**
 * @brief Video metadata types.
 */
typedef enum
{
    ARSTREAM2_STREAM_RECORDER_VIDEO_METADATA_TYPE_UNKNOWN = 0,                              /**< Unknown video metadata type */
    ARSTREAM2_STREAM_RECORDER_VIDEO_METADATA_TYPE_STREAMING_V1,                             /**< "Parrot Video Streaming Metadata" v1 */
    ARSTREAM2_STREAM_RECORDER_VIDEO_METADATA_TYPE_RECORDING_V1,                             /**< "Parrot Video Recording Metadata" v1 */
    ARSTREAM2_STREAM_RECORDER_VIDEO_METADATA_TYPE_V2,                                       /**< "Parrot Video Metadata" v2 */

} ARSTREAM2_STREAM_RECORDER_VideoMetadataTypes_t;


/**
 * "Parrot Video Streaming Metadata" v1 specific identifier.
 */
#define ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_STREAMING_METADATA_V1_ID 0x5031


/**
 * "Parrot Video Recording Metadata" v1 MIME format.
 */
#define ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_RECORDING_METADATA_V1_MIME_FORMAT "application/octet-stream;type=com.parrot.videometadata1"


/**
 * "Parrot Video Recording Metadata" v1 content encoding.
 */
#define ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_RECORDING_METADATA_V1_CONTENT_ENCODING ""


/**
 * "Parrot Video Metadata" v2 identifier.
 */
#define ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_METADATA_V2_ID 0x5032


/**
 * "Parrot Video Metadata" v2 timestamp extension identifier.
 */
#define ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_METADATA_V2_TIMESTAMP_EXTENSION_ID 0x4531


/**
 * "Parrot Video Metadata" v2 MIME format.
 */
#define ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_METADATA_V2_MIME_FORMAT "application/octet-stream;type=com.parrot.videometadata2"


/**
 * "Parrot Video Metadata" v2 content encoding.
 */
#define ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_METADATA_V2_CONTENT_ENCODING ""


/**
 * @brief "Parrot Video Streaming Metadata" v1 basic definition.
 */
typedef struct
{
    uint16_t specific;             /**< Identifier = 0x5031 */
    uint16_t length;               /**< Size in 32 bits words = 5 */
    int16_t  droneYaw;             /**< Drone yaw/psi (rad), Q4.12 */
    int16_t  dronePitch;           /**< Drone pitch/theta (rad), Q4.12 */
    int16_t  droneRoll;            /**< Drone roll/phi (rad), Q4.12 */
    int16_t  cameraPan;            /**< Camera pan (rad), Q4.12 */
    int16_t  cameraTilt;           /**< Camera tilt (rad), Q4.12 */
    int16_t  frameW;               /**< Frame view quaternion W, Q4.12 */
    int16_t  frameX;               /**< Frame view quaternion X, Q4.12 */
    int16_t  frameY;               /**< Frame view quaternion Y, Q4.12 */
    int16_t  frameZ;               /**< Frame view quaternion Z, Q4.12 */
    uint16_t exposureTime;         /**< Frame exposure time (ms), Q8.8 */
    uint16_t gain;                 /**< Frame ISO gain */
    int8_t   wifiRssi;             /**< Wifi RSSI (dBm) */
    uint8_t  batteryPercentage;    /**< Battery charge percentage */

} ARSTREAM2_STREAM_RECORDER_ParrotVideoStreamingMetadataV1Basic_t;


/**
 * @brief "Parrot Video Streaming Metadata" v1 extended definition.
 */
typedef struct
{
    uint16_t specific;             /**< Identifier = 0x5031 */
    uint16_t length;               /**< Size in 32 bits words = 12 */
    int16_t  droneYaw;             /**< Drone yaw/psi (rad), Q4.12 */
    int16_t  dronePitch;           /**< Drone pitch/theta (rad), Q4.12 */
    int16_t  droneRoll;            /**< Drone roll/phi (rad), Q4.12 */
    int16_t  cameraPan;            /**< Camera pan (rad), Q4.12 */
    int16_t  cameraTilt;           /**< Camera tilt (rad), Q4.12 */
    int16_t  frameW;               /**< Frame view quaternion W, Q4.12 */
    int16_t  frameX;               /**< Frame view quaternion X, Q4.12 */
    int16_t  frameY;               /**< Frame view quaternion Y, Q4.12 */
    int16_t  frameZ;               /**< Frame view quaternion Z, Q4.12 */
    uint16_t exposureTime;         /**< Frame exposure time (ms), Q8.8 */
    uint16_t gain;                 /**< Frame ISO gain */
    int8_t   wifiRssi;             /**< Wifi RSSI (dBm) */
    uint8_t  batteryPercentage;    /**< Battery charge percentage */
    int32_t  gpsLatitude;          /**< GPS latitude (deg), Q12.20 */
    int32_t  gpsLongitude;         /**< GPS longitude (deg), Q12.20 */
    int32_t  gpsAltitudeAndSv;     /**< Bits 31..8 = GPS altitude (m) Q16.8, bits 7..0 = GPS SV count */
    int32_t  altitude;             /**< Altitude relative to take-off (m), Q16.16 */
    uint32_t distanceFromHome;     /**< Distance from home (m), Q16.16 */
    int16_t  xSpeed;               /**< X speed (m/s), Q8.8 */
    int16_t  ySpeed;               /**< Y speed (m/s), Q8.8 */
    int16_t  zSpeed;               /**< Z speed (m/s), Q8.8 */
    uint8_t  state;                /**< Bit 7 = binning, bits 6..0 = flyingState */
    uint8_t  mode;                 /**< Bit 7 = animation, bits 6..0 = pilotingMode */

} ARSTREAM2_STREAM_RECORDER_ParrotVideoStreamingMetadataV1Extended_t;


/**
 * @brief "Parrot Video Recording Metadata" v1 definition.
 */
typedef struct
{
    uint32_t frameTimestampH;      /**< Frame timestamp (µs, monotonic), high 32 bits */
    uint32_t frameTimestampL;      /**< Frame timestamp (µs, monotonic), low 32 bits */
    int16_t  droneYaw;             /**< Drone yaw/psi (rad), Q4.12 */
    int16_t  dronePitch;           /**< Drone pitch/theta (rad), Q4.12 */
    int16_t  droneRoll;            /**< Drone roll/phi (rad), Q4.12 */
    int16_t  cameraPan;            /**< Camera pan (rad), Q4.12 */
    int16_t  cameraTilt;           /**< Camera tilt (rad), Q4.12 */
    int16_t  frameW;               /**< Frame view quaternion W, Q4.12 */
    int16_t  frameX;               /**< Frame view quaternion X, Q4.12 */
    int16_t  frameY;               /**< Frame view quaternion Y, Q4.12 */
    int16_t  frameZ;               /**< Frame view quaternion Z, Q4.12 */
    uint16_t exposureTime;         /**< Frame exposure time (ms), Q8.8 */
    uint16_t gain;                 /**< Frame ISO gain */
    int8_t   wifiRssi;             /**< Wifi RSSI (dBm) */
    uint8_t  batteryPercentage;    /**< Battery charge percentage */
    int32_t  gpsLatitude;          /**< GPS latitude (deg), Q12.20 */
    int32_t  gpsLongitude;         /**< GPS longitude (deg), Q12.20 */
    int32_t  gpsAltitudeAndSv;     /**< Bits 31..8 = GPS altitude (m) Q16.8, bits 7..0 = GPS SV count */
    int32_t  altitude;             /**< Altitude relative to take-off (m), Q16.16 */
    uint32_t distanceFromHome;     /**< Distance from home (m), Q16.16 */
    int16_t  xSpeed;               /**< X speed (m/s), Q8.8 */
    int16_t  ySpeed;               /**< Y speed (m/s), Q8.8 */
    int16_t  zSpeed;               /**< Z speed (m/s), Q8.8 */
    uint8_t  state;                /**< Bit 7 = binning, bits 6..0 = flyingState */
    uint8_t  mode;                 /**< Bit 7 = animation, bits 6..0 = pilotingMode */

} ARSTREAM2_STREAM_RECORDER_ParrotVideoRecordingMetadataV1_t;


/**
 * @brief "Parrot Video Metadata" v2 base structure definition.
 */
typedef struct
{
    uint16_t id;                   /**< Identifier = 0x5032 */
    uint16_t length;               /**< Structure size in 32 bits words excluding the id and length
                                        fields and including extensions */
    int32_t  groundDistance;       /**< Best ground distance estimation (m), Q16.16 */
    int32_t  latitude;             /**< Absolute latitude (deg), Q10.22 */
    int32_t  longitude;            /**< Absolute longitude (deg), Q10.22 */
    int32_t  altitudeAndSv;        /**< Bits 31..8 = altitude (m) Q16.8, bits 7..0 = GPS SV count */
    int16_t  northSpeed;           /**< North speed (m/s), Q8.8 */
    int16_t  eastSpeed;            /**< East speed (m/s), Q8.8 */
    int16_t  downSpeed;            /**< Down speed (m/s), Q8.8 */
    int16_t  airSpeed;             /**< Speed relative to air (m/s), negative means no data, Q8.8 */
    int16_t  droneW;               /**< Drone quaternion W, Q2.14 */
    int16_t  droneX;               /**< Drone quaternion X, Q2.14 */
    int16_t  droneY;               /**< Drone quaternion Y, Q2.14 */
    int16_t  droneZ;               /**< Drone quaternion Z, Q2.14 */
    int16_t  frameW;               /**< Frame view quaternion W, Q2.14 */
    int16_t  frameX;               /**< Frame view quaternion X, Q2.14 */
    int16_t  frameY;               /**< Frame view quaternion Y, Q2.14 */
    int16_t  frameZ;               /**< Frame view quaternion Z, Q2.14 */
    int16_t  cameraPan;            /**< Camera pan (rad), Q4.12 */
    int16_t  cameraTilt;           /**< Camera tilt (rad), Q4.12 */
    uint16_t exposureTime;         /**< Frame exposure time (ms), Q8.8 */
    uint16_t gain;                 /**< Frame ISO gain */
    uint8_t  state;                /**< Bit 7 = binning, bits 6..0 = flyingState */
    uint8_t  mode;                 /**< Bit 7 = animation, bits 6..0 = pilotingMode */
    int8_t   wifiRssi;             /**< Wifi RSSI (dBm) */
    uint8_t  batteryPercentage;    /**< Battery charge percentage */

} ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2Base_t;


/**
 * @brief "Parrot Video Metadata" v2 timestamp extension definition.
 */
typedef struct
{
    uint16_t extId;                /**< Extension structure id = 0x4531 */
    uint16_t extLength;            /**< Extension structure size in 32 bits words excluding the
                                        extId and extSize fields */
    uint32_t frameTimestampH;      /**< Frame timestamp (µs, monotonic), high 32 bits */
    uint32_t frameTimestampL;      /**< Frame timestamp (µs, monotonic), low 32 bits */

} ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2TimestampExtension_t;


typedef enum
{
    ARSTREAM2_STREAM_RECORDER_FILE_TYPE_H264_BYTE_STREAM = 0,   /**< H.264 byte stream file format */
    ARSTREAM2_STREAM_RECORDER_FILE_TYPE_MP4,                    /**< ISO base media file format (MP4) */
    ARSTREAM2_STREAM_RECORDER_FILE_TYPE_MAX,

} eARSTREAM2_STREAM_RECORDER_FILE_TYPE;


typedef struct ARSTREAM2_StreamRecorder_s
{
    int threadShouldStop;
    int threadStarted;
    eARSTREAM2_STREAM_RECORDER_FILE_TYPE fileType;
    uint32_t videoWidth;
    uint32_t videoHeight;
    FILE *outputFile;
#if BUILD_LIBARMEDIA
    ARMEDIA_VideoEncapsuler_t* videoEncap;
    ARMEDIA_Frame_Header_t videoEncapFrameHeader;
#endif
    ARSTREAM2_H264_AuFifo_t *auFifo;
    ARSTREAM2_H264_AuFifoQueue_t *auFifoQueue;
    ARSAL_Mutex_t *mutex;
    ARSAL_Cond_t *cond;
    uint32_t auCount;
    uint32_t lastSyncIndex;
    void *recordingMetadata;
    unsigned int recordingMetadataSize;
    ARSTREAM2_STREAM_RECORDER_VideoMetadataTypes_t recordingMetadataType;
    void *savedMetadata;
    unsigned int savedMetadataSize;

} ARSTREAM2_StreamRecorder_t;


static ARSTREAM2_STREAM_RECORDER_VideoMetadataTypes_t ARSTREAM2_StreamRecorder_StreamingToRecordingMetadataType(void *streamingMetadata, unsigned int streamingMetadataSize, int *size)
{
    if ((!streamingMetadata) || (streamingMetadataSize < 4))
    {
        return -1;
    }

    uint16_t specific = ntohs(*((uint16_t*)streamingMetadata));

    if (specific == ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_STREAMING_METADATA_V1_ID)
    {
        if (size) *size = sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoRecordingMetadataV1_t);
        return ARSTREAM2_STREAM_RECORDER_VIDEO_METADATA_TYPE_RECORDING_V1;
    }
    else if (specific == ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_METADATA_V2_ID)
    {
        if (size) *size = sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2Base_t) + sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2TimestampExtension_t);
        return ARSTREAM2_STREAM_RECORDER_VIDEO_METADATA_TYPE_V2;
    }
    else
    {
        return ARSTREAM2_STREAM_RECORDER_VIDEO_METADATA_TYPE_UNKNOWN;
    }
}

static int ARSTREAM2_StreamRecorder_StreamingToRecordingMetadata(uint64_t timestamp,
                                                                 const void *streamingMetadata, unsigned int streamingMetadataSize,
                                                                 void *savedMetadata, unsigned int savedMetadataSize,
                                                                 void *recordingMetadata, unsigned int recordingMetadataSize,
                                                                 ARSTREAM2_STREAM_RECORDER_VideoMetadataTypes_t type)
{
    int ret = 0;

    if ((!streamingMetadata) || (streamingMetadataSize < 4)
            || (!recordingMetadata) || (!recordingMetadataSize)
            || (!savedMetadata) || (!savedMetadataSize))
    {
        return -1;
    }

    uint16_t specific = ntohs(*((uint16_t*)streamingMetadata));
    uint16_t length = ntohs(*((uint16_t*)streamingMetadata + 1));

    if ((specific == ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_STREAMING_METADATA_V1_ID) && (type == ARSTREAM2_STREAM_RECORDER_VIDEO_METADATA_TYPE_RECORDING_V1))
    {
        if (recordingMetadataSize != sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoRecordingMetadataV1_t))
        {
            return -1;
        }
        if (savedMetadataSize != sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoRecordingMetadataV1_t))
        {
            return -1;
        }
        ARSTREAM2_STREAM_RECORDER_ParrotVideoRecordingMetadataV1_t *recMeta = (ARSTREAM2_STREAM_RECORDER_ParrotVideoRecordingMetadataV1_t*)recordingMetadata;
        ARSTREAM2_STREAM_RECORDER_ParrotVideoRecordingMetadataV1_t *savedMeta = (ARSTREAM2_STREAM_RECORDER_ParrotVideoRecordingMetadataV1_t*)savedMetadata;
        if ((length == (sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoStreamingMetadataV1Extended_t) - 4) / 4)
                && (streamingMetadataSize >= sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoStreamingMetadataV1Extended_t)))
        {
            ARSTREAM2_STREAM_RECORDER_ParrotVideoStreamingMetadataV1Extended_t *streamMeta = (ARSTREAM2_STREAM_RECORDER_ParrotVideoStreamingMetadataV1Extended_t*)streamingMetadata;
            recMeta->frameTimestampH = htonl((uint32_t)(timestamp >> 32));
            recMeta->frameTimestampL = htonl((uint32_t)(timestamp & 0xFFFFFFFF));
            recMeta->droneYaw = streamMeta->droneYaw;
            recMeta->dronePitch = streamMeta->dronePitch;
            recMeta->droneRoll = streamMeta->droneRoll;
            recMeta->cameraPan = streamMeta->cameraPan;
            recMeta->cameraTilt = streamMeta->cameraTilt;
            recMeta->frameW = streamMeta->frameW;
            recMeta->frameX = streamMeta->frameX;
            recMeta->frameY = streamMeta->frameY;
            recMeta->frameZ = streamMeta->frameZ;
            recMeta->exposureTime = streamMeta->exposureTime;
            recMeta->gain = streamMeta->gain;
            recMeta->wifiRssi = streamMeta->wifiRssi;
            recMeta->batteryPercentage = streamMeta->batteryPercentage;
            recMeta->gpsLatitude = savedMeta->gpsLatitude = streamMeta->gpsLatitude;
            recMeta->gpsLongitude = savedMeta->gpsLongitude = streamMeta->gpsLongitude;
            recMeta->gpsAltitudeAndSv = savedMeta->gpsAltitudeAndSv = streamMeta->gpsAltitudeAndSv;
            recMeta->altitude = savedMeta->altitude = streamMeta->altitude;
            recMeta->distanceFromHome = savedMeta->distanceFromHome = streamMeta->distanceFromHome;
            recMeta->xSpeed = savedMeta->xSpeed = streamMeta->xSpeed;
            recMeta->ySpeed = savedMeta->ySpeed = streamMeta->ySpeed;
            recMeta->zSpeed = savedMeta->zSpeed = streamMeta->zSpeed;
            recMeta->state = savedMeta->state = streamMeta->state;
            recMeta->mode = savedMeta->mode = streamMeta->mode;
        }
        else if ((length == (sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoStreamingMetadataV1Basic_t) - 4) / 4)
                && (streamingMetadataSize >= sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoStreamingMetadataV1Basic_t)))
        {
            ARSTREAM2_STREAM_RECORDER_ParrotVideoStreamingMetadataV1Basic_t *streamMeta = (ARSTREAM2_STREAM_RECORDER_ParrotVideoStreamingMetadataV1Basic_t*)streamingMetadata;
            recMeta->frameTimestampH = htonl((uint32_t)(timestamp >> 32));
            recMeta->frameTimestampL = htonl((uint32_t)(timestamp & 0xFFFFFFFF));
            recMeta->droneYaw = streamMeta->droneYaw;
            recMeta->dronePitch = streamMeta->dronePitch;
            recMeta->droneRoll = streamMeta->droneRoll;
            recMeta->cameraPan = streamMeta->cameraPan;
            recMeta->cameraTilt = streamMeta->cameraTilt;
            recMeta->frameW = streamMeta->frameW;
            recMeta->frameX = streamMeta->frameX;
            recMeta->frameY = streamMeta->frameY;
            recMeta->frameZ = streamMeta->frameZ;
            recMeta->exposureTime = streamMeta->exposureTime;
            recMeta->gain = streamMeta->gain;
            recMeta->wifiRssi = streamMeta->wifiRssi;
            recMeta->batteryPercentage = streamMeta->batteryPercentage;
            recMeta->gpsLatitude = savedMeta->gpsLatitude;
            recMeta->gpsLongitude = savedMeta->gpsLongitude;
            recMeta->gpsAltitudeAndSv = savedMeta->gpsAltitudeAndSv;
            recMeta->altitude = savedMeta->altitude;
            recMeta->distanceFromHome = savedMeta->distanceFromHome;
            recMeta->xSpeed = savedMeta->xSpeed;
            recMeta->ySpeed = savedMeta->ySpeed;
            recMeta->zSpeed = savedMeta->zSpeed;
            recMeta->state = savedMeta->state;
            recMeta->mode = savedMeta->mode;
        }
        else
        {
            return -1;
        }
    }
    else if ((specific == ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_METADATA_V2_ID) && (type == ARSTREAM2_STREAM_RECORDER_VIDEO_METADATA_TYPE_V2))
    {
        if (recordingMetadataSize != sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2Base_t) + sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2TimestampExtension_t))
        {
            return -1;
        }
        ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2Base_t *recMeta = (ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2Base_t*)recordingMetadata;
        ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2TimestampExtension_t *recMetaTSExt = (ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2TimestampExtension_t*)((uint8_t*)recordingMetadata + sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2Base_t));
        if ((length >= (sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2Base_t) - 4) / 4)
                && (streamingMetadataSize >= sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2Base_t)))
        {
            memcpy(recMeta, streamingMetadata, sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2Base_t));
            recMeta->length = htons((sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2Base_t) + sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2TimestampExtension_t) - 4) / 4);
            recMetaTSExt->extId = htons(ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_METADATA_V2_TIMESTAMP_EXTENSION_ID);
            recMetaTSExt->extLength = htons((sizeof(ARSTREAM2_STREAM_RECORDER_ParrotVideoMetadataV2TimestampExtension_t) - 4) / 4);
            recMetaTSExt->frameTimestampH = htonl((uint32_t)(timestamp >> 32));
            recMetaTSExt->frameTimestampL = htonl((uint32_t)(timestamp & 0xFFFFFFFF));
        }
    }
    else
    {
        return -1;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamRecorder_Init(ARSTREAM2_StreamRecorder_Handle *streamRecorderHandle,
                                               ARSTREAM2_StreamRecorder_Config_t *config)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    ARSTREAM2_StreamRecorder_t *streamRecorder = NULL;

    if (!streamRecorderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid pointer for handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!config)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid pointer for config");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((!config->mediaFileName) || (strlen(config->mediaFileName) < 4))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid media file name");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    int mediaFileNameLen = strlen(config->mediaFileName);
    if ((strcasecmp(config->mediaFileName + mediaFileNameLen - 4, ".mp4") != 0)
            && (strcasecmp(config->mediaFileName + mediaFileNameLen - 4, ".264") != 0)
            && (strcasecmp(config->mediaFileName + mediaFileNameLen - 5, ".h264") != 0))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid media file name extension");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((!config->sps) || (!config->spsSize))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid SPS");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((!config->pps) || (!config->ppsSize))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid PPS");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!config->auFifo)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid AU FIFO");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!config->auFifoQueue)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid AU FIFO queue");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!config->mutex)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid mutex");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!config->cond)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid cond");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    streamRecorder = (ARSTREAM2_StreamRecorder_t*)malloc(sizeof(*streamRecorder));
    if (!streamRecorder)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Allocation failed (size %zu)", sizeof(*streamRecorder));
        ret = ARSTREAM2_ERROR_ALLOC;
    }

    if (ret == ARSTREAM2_OK)
    {
        memset(streamRecorder, 0, sizeof(*streamRecorder));
        streamRecorder->auFifo = config->auFifo;
        streamRecorder->auFifoQueue = config->auFifoQueue;
        streamRecorder->mutex = config->mutex;
        streamRecorder->cond = config->cond;
        streamRecorder->videoWidth = config->videoWidth;
        streamRecorder->videoHeight = config->videoHeight;
        if (strcasecmp(config->mediaFileName + mediaFileNameLen - 4, ".mp4") == 0)
        {
            streamRecorder->fileType = ARSTREAM2_STREAM_RECORDER_FILE_TYPE_MP4;
        }
        else
        {
            streamRecorder->fileType = ARSTREAM2_STREAM_RECORDER_FILE_TYPE_H264_BYTE_STREAM;
        }
    }

#if BUILD_LIBARMEDIA
    if ((ret == ARSTREAM2_OK) && (streamRecorder->fileType == ARSTREAM2_STREAM_RECORDER_FILE_TYPE_MP4))
    {
        eARMEDIA_ERROR err = ARMEDIA_OK;
        streamRecorder->videoEncap = ARMEDIA_VideoEncapsuler_New(config->mediaFileName,
                                                                 round(config->videoFramerate),
                                                                 "", "", config->ardiscoveryProductType, &err);
        if (streamRecorder->videoEncap == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "ARMEDIA_VideoEncapsuler_New() failed: %d (%s)", err, ARMEDIA_Error_ToString(err));
            ret = ARSTREAM2_ERROR_UNSUPPORTED;
        }
    }

    if ((ret == ARSTREAM2_OK) && (streamRecorder->fileType == ARSTREAM2_STREAM_RECORDER_FILE_TYPE_MP4))
    {
        eARMEDIA_ERROR err;
        err = ARMEDIA_VideoEncapsuler_SetAvcParameterSets(streamRecorder->videoEncap, config->sps, config->spsSize, config->pps, config->ppsSize);
        if (err != ARMEDIA_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "ARMEDIA_VideoEncapsuler_SetAvcParameterSets() failed: %d (%s)", err, ARMEDIA_Error_ToString(err));
            ret = ARSTREAM2_ERROR_UNSUPPORTED;
        }
    }
#else
    if ((ret == ARSTREAM2_OK) && (streamRecorder->fileType == ARSTREAM2_STREAM_RECORDER_FILE_TYPE_MP4))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Unsupported file format: MP4");
        ret = ARSTREAM2_ERROR_UNSUPPORTED;
    }
#endif

    if ((ret == ARSTREAM2_OK) && (streamRecorder->fileType == ARSTREAM2_STREAM_RECORDER_FILE_TYPE_H264_BYTE_STREAM))
    {
        streamRecorder->outputFile = fopen(config->mediaFileName, "wb");
        if (streamRecorder->outputFile == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Failed to open file '%s'", config->mediaFileName);
            ret = ARSTREAM2_ERROR_ALLOC;
        }

        if (streamRecorder->outputFile)
        {
            fwrite(config->sps, config->spsSize, 1, streamRecorder->outputFile);
            fwrite(config->pps, config->ppsSize, 1, streamRecorder->outputFile);
            fflush(streamRecorder->outputFile);
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        *streamRecorderHandle = streamRecorder;
    }
    else
    {
        if (streamRecorder)
        {
            if (streamRecorder->outputFile) fclose(streamRecorder->outputFile);
            free(streamRecorder);
        }
        *streamRecorderHandle = NULL;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamRecorder_Free(ARSTREAM2_StreamRecorder_Handle *streamRecorderHandle)
{
    ARSTREAM2_StreamRecorder_t* streamRecorder;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    int canDelete = 0;

    if ((!streamRecorderHandle) || (!*streamRecorderHandle))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid pointer for handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    streamRecorder = (ARSTREAM2_StreamRecorder_t*)*streamRecorderHandle;

    ARSAL_Mutex_Lock(streamRecorder->mutex);
    if (streamRecorder->threadStarted == 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_STREAM_RECORDER_TAG, "Thread is stopped");
        canDelete = 1;
    }
    ARSAL_Mutex_Unlock(streamRecorder->mutex);

    if (canDelete == 1)
    {
        if (streamRecorder->outputFile) fclose(streamRecorder->outputFile);
        free(streamRecorder->recordingMetadata);
        free(streamRecorder->savedMetadata);

        free(streamRecorder);
        *streamRecorderHandle = NULL;
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Call ARSTREAM2_StreamRecorder_Stop before calling this function");
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamRecorder_Stop(ARSTREAM2_StreamRecorder_Handle streamRecorderHandle)
{
    ARSTREAM2_StreamRecorder_t* streamRecorder = (ARSTREAM2_StreamRecorder_t*)streamRecorderHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!streamRecorderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_STREAM_RECORDER_TAG, "Stopping stream recorder...");
    ARSAL_Mutex_Lock(streamRecorder->mutex);
    streamRecorder->threadShouldStop = 1;
    ARSAL_Mutex_Unlock(streamRecorder->mutex);
    /* signal the thread to avoid a deadlock */
    ARSAL_Cond_Signal(streamRecorder->cond);

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamRecorder_SetUntimedMetadata(ARSTREAM2_StreamRecorder_Handle streamRecorderHandle,
                                                             const ARSTREAM2_StreamRecorder_UntimedMetadata_t *metadata)
{
    ARSTREAM2_StreamRecorder_t* streamRecorder = streamRecorderHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!streamRecorderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!metadata)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid metadata");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

#if BUILD_LIBARMEDIA
    ARMEDIA_Untimed_Metadata_t meta;
    memset(&meta, 0, sizeof(ARMEDIA_Untimed_Metadata_t));
    if (metadata->maker)
    {
        snprintf(meta.maker,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_MAKER_SIZE,
                 "%s", metadata->maker);
    }
    if (metadata->model)
    {
        snprintf(meta.model,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_MODEL_SIZE,
                 "%s", metadata->model);
    }
    if (metadata->modelId)
    {
        snprintf(meta.modelId,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_MODEL_ID_SIZE,
                 "%s", metadata->modelId);
    }
    if (metadata->serialNumber)
    {
        snprintf(meta.serialNumber,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_SERIAL_NUM_SIZE,
                 "%s", metadata->serialNumber);
    }
    if (metadata->softwareVersion)
    {
        snprintf(meta.softwareVersion,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_SOFT_VER_SIZE,
                 "%s", metadata->softwareVersion);
    }
    if (metadata->buildId)
    {
        snprintf(meta.buildId,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_BUILD_ID_SIZE,
                 "%s", metadata->buildId);
    }
    if (metadata->artist)
    {
        snprintf(meta.artist,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_ARTIST_SIZE,
                 "%s", metadata->artist);
    }
    if (metadata->title)
    {
        snprintf(meta.title,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_TITLE_SIZE,
                 "%s", metadata->title);
    }
    if (metadata->comment)
    {
        snprintf(meta.comment,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_COMMENT_SIZE,
                 "%s", metadata->comment);
    }
    if (metadata->copyright)
    {
        snprintf(meta.copyright,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_COPYRIGHT_SIZE,
                 "%s", metadata->copyright);
    }
    if (metadata->mediaDate)
    {
        snprintf(meta.mediaDate,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_MEDIA_DATE_SIZE,
                 "%s", metadata->mediaDate);
    }
    if (metadata->runDate)
    {
        snprintf(meta.runDate,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_RUN_DATE_SIZE,
                 "%s", metadata->runDate);
    }
    if (metadata->runUuid)
    {
        snprintf(meta.runUuid,
                 ARMEDIA_ENCAPSULER_UNTIMED_METADATA_RUN_UUID_SIZE,
                 "%s", metadata->runUuid);
    }
    meta.takeoffLatitude = metadata->takeoffLatitude;
    meta.takeoffLongitude = metadata->takeoffLongitude;
    meta.takeoffAltitude = metadata->takeoffAltitude;
    meta.pictureHFov = metadata->pictureHFov;
    meta.pictureVFov = metadata->pictureVFov;
    eARMEDIA_ERROR err = ARMEDIA_VideoEncapsuler_SetUntimedMetadata(streamRecorder->videoEncap, &meta);
    if (err != ARMEDIA_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "ARMEDIA_VideoEncapsuler_SetUntimedMetadata() failed: %d (%s)", err, ARMEDIA_Error_ToString(err));
        ret = ARSTREAM2_ERROR_INVALID_STATE;
    }
#endif

    return ret;
}


void* ARSTREAM2_StreamRecorder_RunThread(void *param)
{
    ARSTREAM2_StreamRecorder_t* streamRecorder = (ARSTREAM2_StreamRecorder_t*)param;
    int shouldStop;

    if (!param)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Invalid handle");
        return 0;
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_STREAM_RECORDER_TAG, "Stream recorder thread running");
    ARSAL_Mutex_Lock(streamRecorder->mutex);
    streamRecorder->threadStarted = 1;
    shouldStop = streamRecorder->threadShouldStop;
    ARSAL_Mutex_Unlock(streamRecorder->mutex);

    while (!shouldStop)
    {
        ARSTREAM2_H264_AuFifoItem_t *auItem;

        /* dequeue an access unit */
        auItem = ARSTREAM2_H264_AuFifoDequeueItem(streamRecorder->auFifoQueue);

        while (auItem != NULL)
        {
            ARSTREAM2_H264_AccessUnit_t *au = &auItem->au;
            ARSTREAM2_H264_NaluFifoItem_t *naluItem;

            /* Record the frame */
            switch (streamRecorder->fileType)
            {
            case ARSTREAM2_STREAM_RECORDER_FILE_TYPE_H264_BYTE_STREAM:
            {
                if (streamRecorder->outputFile)
                {
                    for (naluItem = au->naluHead; naluItem; naluItem = naluItem->next)
                    {
                        fwrite(naluItem->nalu.nalu, naluItem->nalu.naluSize, 1, streamRecorder->outputFile);
                    }

                    if ((au->syncType != ARSTREAM2_H264_AU_SYNC_TYPE_NONE)
                            || (streamRecorder->auCount >= streamRecorder->lastSyncIndex + ARSTREAM2_STREAM_RECORDER_FILE_SYNC_MAX_INTERVAL))
                    {
                        fflush(streamRecorder->outputFile);
                        fsync(fileno(streamRecorder->outputFile));
                        streamRecorder->lastSyncIndex = streamRecorder->auCount;
                    }
                }
                break;
            }
#if BUILD_LIBARMEDIA
            case ARSTREAM2_STREAM_RECORDER_FILE_TYPE_MP4:
            {
                int gotMetadata = 0;
                memset(&streamRecorder->videoEncapFrameHeader, 0, sizeof(ARMEDIA_Frame_Header_t));
                streamRecorder->videoEncapFrameHeader.codec = CODEC_MPEG4_AVC;
                streamRecorder->videoEncapFrameHeader.frame_size = au->auSize;
                streamRecorder->videoEncapFrameHeader.frame_number = streamRecorder->auCount;
                streamRecorder->videoEncapFrameHeader.width = streamRecorder->videoWidth;
                streamRecorder->videoEncapFrameHeader.height = streamRecorder->videoHeight;
                streamRecorder->videoEncapFrameHeader.timestamp = au->ntpTimestampRaw;
                streamRecorder->videoEncapFrameHeader.frame_type = (au->syncType == ARSTREAM2_H264_AU_SYNC_TYPE_NONE) ? ARMEDIA_ENCAPSULER_FRAME_TYPE_P_FRAME : ARMEDIA_ENCAPSULER_FRAME_TYPE_I_FRAME;
                streamRecorder->videoEncapFrameHeader.frame = NULL;
                streamRecorder->videoEncapFrameHeader.avc_insert_ps = 0;
                unsigned int naluCount = 0;
                for (naluItem = au->naluHead; naluItem; naluItem = naluItem->next)
                {
                    streamRecorder->videoEncapFrameHeader.avc_nalu_size[naluCount] = naluItem->nalu.naluSize;
                    streamRecorder->videoEncapFrameHeader.avc_nalu_data[naluCount] = naluItem->nalu.nalu;
                    naluCount++;
                }
                streamRecorder->videoEncapFrameHeader.avc_nalu_count = naluCount;

                if ((au->buffer->metadataBuffer) && (au->metadataSize) && (streamRecorder->recordingMetadataSize == 0))
                {
                    /* Setup the metadata */
                    int size = 0;
                    streamRecorder->recordingMetadataType = ARSTREAM2_StreamRecorder_StreamingToRecordingMetadataType(au->buffer->metadataBuffer, au->metadataSize, &size);
                    if (size > 0)
                    {
                        int ret = 0;
                        streamRecorder->recordingMetadataSize = (unsigned)size;
                        streamRecorder->recordingMetadata = malloc(streamRecorder->recordingMetadataSize);
                        if (!streamRecorder->recordingMetadata)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Metadata buffer allocation failed (size: %d)", streamRecorder->recordingMetadataSize);
                            ret = -1;
                        }
                        if (ret == 0)
                        {
                            streamRecorder->savedMetadataSize = streamRecorder->recordingMetadataSize;
                            streamRecorder->savedMetadata = malloc(streamRecorder->savedMetadataSize);
                            if (!streamRecorder->savedMetadata)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Metadata buffer allocation failed (size: %d)", streamRecorder->savedMetadataSize);
                                ret = -1;
                            }
                        }
                        if (ret == 0)
                        {
                            eARMEDIA_ERROR err = ARMEDIA_OK;
                            if (streamRecorder->recordingMetadataType == ARSTREAM2_STREAM_RECORDER_VIDEO_METADATA_TYPE_RECORDING_V1)
                            {
                                err = ARMEDIA_VideoEncapsuler_SetMetadataInfo(streamRecorder->videoEncap,
                                                                              ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_RECORDING_METADATA_V1_CONTENT_ENCODING,
                                                                              ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_RECORDING_METADATA_V1_MIME_FORMAT,
                                                                              size);
                            }
                            else if (streamRecorder->recordingMetadataType == ARSTREAM2_STREAM_RECORDER_VIDEO_METADATA_TYPE_V2)
                            {
                                err = ARMEDIA_VideoEncapsuler_SetMetadataInfo(streamRecorder->videoEncap,
                                                                              ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_METADATA_V2_CONTENT_ENCODING,
                                                                              ARSTREAM2_STREAM_RECORDER_PARROT_VIDEO_METADATA_V2_MIME_FORMAT,
                                                                              size);
                            }
                            if (err != ARMEDIA_OK)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "ARMEDIA_VideoEncapsuler_SetMetadataInfo() failed: %d (%s)", err, ARMEDIA_Error_ToString(err));
                                ret = -1;
                            }
                        }
                        if (ret != 0)
                        {
                            free(streamRecorder->recordingMetadata);
                            streamRecorder->recordingMetadata = NULL;
                            streamRecorder->recordingMetadataSize = 0;
                            free(streamRecorder->savedMetadata);
                            streamRecorder->savedMetadata = NULL;
                            streamRecorder->savedMetadataSize = 0;
                        }
                    }
                }
                if ((au->buffer->metadataBuffer) && (au->metadataSize) && (streamRecorder->recordingMetadataSize > 0))
                {
                    /* Convert the metadata */
                    int ret = ARSTREAM2_StreamRecorder_StreamingToRecordingMetadata(au->ntpTimestampRaw,
                                                                                    au->buffer->metadataBuffer, au->metadataSize,
                                                                                    streamRecorder->savedMetadata, streamRecorder->savedMetadataSize,
                                                                                    streamRecorder->recordingMetadata, streamRecorder->recordingMetadataSize,
                                                                                    streamRecorder->recordingMetadataType);
                    if (ret != 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "ARSTREAM2_StreamRecorder_StreamingToRecordingMetadata() failed: %d", ret);
                    }
                    else
                    {
                        gotMetadata = 1;
                    }
                }

                eARMEDIA_ERROR err = ARMEDIA_VideoEncapsuler_AddFrame(streamRecorder->videoEncap, &streamRecorder->videoEncapFrameHeader, ((gotMetadata) ? streamRecorder->recordingMetadata : NULL));
                if (err != ARMEDIA_OK)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "ARMEDIA_VideoEncapsuler_AddFrame() failed: %d (%s)", err, ARMEDIA_Error_ToString(err));
                }
                break;
            }
#endif
            default:
                break;
            }

            streamRecorder->auCount++;

            /* free the access unit */
            int ret = ARSTREAM2_H264_AuFifoUnrefBuffer(streamRecorder->auFifo, auItem->au.buffer);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Failed to unref buffer (%d)", ret);
            }
            ret = ARSTREAM2_H264_AuFifoPushFreeItem(streamRecorder->auFifo, auItem);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "Failed to push free item in the AU FIFO (%d)", ret);
            }

            /* dequeue the next access unit */
            auItem = ARSTREAM2_H264_AuFifoDequeueItem(streamRecorder->auFifoQueue);
        }

        ARSAL_Mutex_Lock(streamRecorder->mutex);
        shouldStop = streamRecorder->threadShouldStop;
        ARSAL_Mutex_Unlock(streamRecorder->mutex);

        if (!shouldStop)
        {
            /* Wake up when a new AU is in the FIFO or when we need to exit */
            ARSAL_Mutex_Lock(streamRecorder->mutex);
            ARSAL_Cond_Timedwait(streamRecorder->cond, streamRecorder->mutex, ARSTREAM2_STREAM_RECORDER_FIFO_COND_TIMEOUT_MS);
            ARSAL_Mutex_Unlock(streamRecorder->mutex);
        }
    }

#if BUILD_LIBARMEDIA
    if (streamRecorder->fileType == ARSTREAM2_STREAM_RECORDER_FILE_TYPE_MP4)
    {
        eARMEDIA_ERROR err = ARMEDIA_VideoEncapsuler_Finish(&streamRecorder->videoEncap);
        if (err != ARMEDIA_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECORDER_TAG, "ARMEDIA_VideoEncapsuler_AddFrame() failed: %d (%s)", err, ARMEDIA_Error_ToString(err));
        }
    }
#endif

    ARSAL_Mutex_Lock(streamRecorder->mutex);
    streamRecorder->threadStarted = 0;
    ARSAL_Mutex_Unlock(streamRecorder->mutex);
    ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_STREAM_RECORDER_TAG, "Stream recorder thread has ended");

    return (void*)0;
}
