/**
 * @file arstream2_stream_recorder.h
 * @brief Parrot Streaming Library - Stream Recorder
 * @date 06/01/2016
 * @author aurelien.barre@parrot.com
 */

#ifndef _ARSTREAM2_STREAM_RECORDER_H_
#define _ARSTREAM2_STREAM_RECORDER_H_

#ifdef __cplusplus
extern "C" {
#endif /* #ifdef __cplusplus */

#include <inttypes.h>
#include <libARStream2/arstream2_error.h>
#include "arstream2_h264.h"


/**
 * @brief ARSTREAM2 StreamRecorder instance handle.
 */
typedef struct ARSTREAM2_StreamRecorder_s *ARSTREAM2_StreamRecorder_Handle;


/**
 * @brief ARSTREAM2 StreamRecorder configuration for initialization.
 */
typedef struct
{
    const char *mediaFileName;              /**< Destination media file name */
    float videoFramerate;                   /**< Video framerate (frame/s) */
    uint32_t videoWidth;                    /**< Video width (pixels) */
    uint32_t videoHeight;                   /**< Video height (pixels) */
    const uint8_t *sps;                     /**< H.264 video SPS buffer pointer */
    uint32_t spsSize;                       /**< H.264 video SPS buffer size in bytes */
    const uint8_t *pps;                     /**< H.264 video PPS buffer pointer */
    uint32_t ppsSize;                       /**< H.264 video PPS buffer size in bytes */
    int ardiscoveryProductType;             /**< ARDiscovery product type */
    ARSTREAM2_H264_AuFifo_t *auFifo;
    ARSTREAM2_H264_AuFifoQueue_t *auFifoQueue;
    ARSAL_Mutex_t *mutex;
    ARSAL_Cond_t *cond;

} ARSTREAM2_StreamRecorder_Config_t;


typedef struct
{
    char *maker;                            /**< product maker */
    char *model;                            /**< product model */
    char *modelId;                          /**< product model ID (ARSDK 16-bit model ID in hex ASCII) */
    char *serialNumber;                     /**< product serial number */
    char *softwareVersion;                  /**< software version */
    char *buildId;                          /**< software build ID */
    char *artist;
    char *title;
    char *comment;
    char *copyright;
    char *mediaDate;                        /**< media date and time */
    char *runDate;                          /**< run date and time */
    char *runUuid;                          /**< run UUID */
    double takeoffLatitude;                 /**< takeoff latitude (500 means unknown) */
    double takeoffLongitude;                /**< takeoff longitude (500 means unknown) */
    float takeoffAltitude;                  /**< takeoff altitude */
    float pictureHFov;                      /**< camera horizontal field of view (0 means unknown) */
    float pictureVFov;                      /**< camera vertical field of view (0 means unknown) */

} ARSTREAM2_StreamRecorder_UntimedMetadata_t;


/**
 * @brief Initialize a StreamRecorder instance.
 *
 * The library allocates the required resources. The user must call ARSTREAM2_StreamRecorder_Free() to free the resources.
 *
 * @param streamRecorderHandle Pointer to the handle used in future calls to the library.
 * @param config The instance configuration.
 *
 * @return ARSTREAM2_OK if no error occurred.
 * @return a eARSTREAM2_ERROR if an error occurred.
 */
eARSTREAM2_ERROR ARSTREAM2_StreamRecorder_Init(ARSTREAM2_StreamRecorder_Handle *streamRecorderHandle,
                                               ARSTREAM2_StreamRecorder_Config_t *config);


/**
 * @brief Free a StreamRecorder instance.
 *
 * The library frees the allocated resources. On success the streamRecorderHandle is set to NULL.
 *
 * @param streamRecorderHandle Pointer to the instance handle.
 *
 * @return ARSTREAM2_OK if no error occurred.
 * @return a eARSTREAM2_ERROR if an error occurred.
 */
eARSTREAM2_ERROR ARSTREAM2_StreamRecorder_Free(ARSTREAM2_StreamRecorder_Handle *streamRecorderHandle);


/**
 * @brief Stop a StreamRecorder instance.
 *
 * The function ends the thread before it can be joined.
 *
 * @param streamRecorderHandle Instance handle.
 *
 * @return ARSTREAM2_OK if no error occurred.
 * @return a eARSTREAM2_ERROR if an error occurred.
 */
eARSTREAM2_ERROR ARSTREAM2_StreamRecorder_Stop(ARSTREAM2_StreamRecorder_Handle streamRecorderHandle);


/**
 * @brief Set the untimed metadata
 *
 * @param streamRecorderHandle Instance handle.
 * @param[in] metadata Untimed metadata structure
 *
 * @return ARSTREAM2_OK if no error happened
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the streamRecorderHandle or metadata pointer are invalid
 */
eARSTREAM2_ERROR ARSTREAM2_StreamRecorder_SetUntimedMetadata(ARSTREAM2_StreamRecorder_Handle streamRecorderHandle,
                                                             const ARSTREAM2_StreamRecorder_UntimedMetadata_t *metadata);


/**
 * @brief Run a StreamRecorder thread.
 *
 * The instance must be correctly allocated using ARSTREAM2_StreamRecorder_Init().
 * @warning This function never returns until ARSTREAM2_StreamRecorder_Stop() is called. The tread can then be joined.
 *
 * @param streamRecorderHandle Instance handle casted as (void*).
 *
 * @return NULL in all cases.
 */
void* ARSTREAM2_StreamRecorder_RunThread(void *streamRecorderHandle);


#ifdef __cplusplus
}
#endif /* #ifdef __cplusplus */

#endif /* #ifndef _ARSTREAM2_STREAM_RECORDER_H_ */
