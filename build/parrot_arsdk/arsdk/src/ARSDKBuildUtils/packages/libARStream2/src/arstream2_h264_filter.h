/**
 * @file arstream2_h264_filter.c
 * @brief Parrot Reception Library - H.264 Filter
 * @date 08/04/2015
 * @author aurelien.barre@parrot.com
 */


#ifndef _ARSTREAM2_H264_FILTER_H_
#define _ARSTREAM2_H264_FILTER_H_


#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <arpa/inet.h>

#include <libARSAL/ARSAL_Print.h>

#include <libARStream2/arstream2_h264_parser.h>
#include <libARStream2/arstream2_h264_writer.h>
#include <libARStream2/arstream2_h264_sei.h>
#include <libARStream2/arstream2_stream_stats.h>

#include "arstream2_h264.h"


#define ARSTREAM2_H264_FILTER_MAX_INFERRED_IDR_INTERVAL 60


/*
 * Types
 */

/**
 * @brief ARSTREAM2 H264Filter instance handle.
 */
typedef struct ARSTREAM2_H264Filter_s *ARSTREAM2_H264Filter_Handle;


typedef int (*ARSTREAM2_H264Filter_SpsPpsSyncCallback_t)(uint8_t *spsBuffer, int spsSize, uint8_t *ppsBuffer, int ppsSize, void *userPtr);


/**
 * @brief ARSTREAM2 H264Filter configuration for initialization.
 */
typedef struct
{
    ARSTREAM2_H264_ReceiverAuCallback_t auCallback;
    void *auCallbackUserPtr;
    ARSTREAM2_H264Filter_SpsPpsSyncCallback_t spsPpsCallback;
    void *spsPpsCallbackUserPtr;
    int outputIncompleteAu;                                         /**< if true, output incomplete access units */
    int generateSkippedPSlices;                                     /**< if true, generate skipped P slices to replace missing slices */

} ARSTREAM2_H264Filter_Config_t;


typedef struct ARSTREAM2_H264Filter_s
{
    int outputIncompleteAu;
    int generateSkippedPSlices;

    int currentAuOutputIndex;
    int currentAuSize;
    int currentAuIncomplete;
    int currentAuFrameNum;
    int previousAuFrameNum;
    int currentAuSlicesReceived;
    int currentAuSlicesAllI;
    int currentAuStreamingInfoAvailable;
    uint16_t currentAuStreamingSliceMbCount[ARSTREAM2_H264_SEI_PARROT_STREAMING_MAX_SLICE_COUNT];
    int currentAuStreamingSliceCount;
    int currentAuStreamingInfoV1Available;
    ARSTREAM2_H264Sei_ParrotStreamingV1_t currentAuStreamingInfoV1;
    int currentAuStreamingInfoV2Available;
    ARSTREAM2_H264Sei_ParrotStreamingV2_t currentAuStreamingInfoV2;
    int currentAuIsRecoveryPoint;
    int currentAuPreviousSliceIndex;
    int currentAuPreviousSliceFirstMb;
    int currentAuCurrentSliceFirstMb;
    uint8_t previousSliceType;

    uint8_t *currentAuRefMacroblockStatus;
    uint8_t *currentAuMacroblockStatus;
    int currentAuIsIdr;
    int currentAuIsRef;
    int currentAuInferredSliceMbCount;
    int currentAuInferredPreviousSliceFirstMb;

    /* H.264-level stats */
    ARSTREAM2_H264_VideoStats_t stats;

    ARSTREAM2_H264Parser_Handle parser;
    ARSTREAM2_H264Writer_Handle writer;
    ARSTREAM2_H264_SliceContext_t savedSliceContext;
    int savedSliceContextAvailable;

    int sync;
    int spsSync;
    int spsSize;
    uint8_t* pSps;
    int ppsSync;
    int ppsSize;
    uint8_t* pPps;
    ARSTREAM2_H264Filter_SpsPpsSyncCallback_t spsPpsCallback;
    void *spsPpsCallbackUserPtr;
    int resyncPending;
    int mbWidth;
    int mbHeight;
    int mbCount;
    float framerate;
    int maxFrameNum;
    int inferredIdrInterval;

} ARSTREAM2_H264Filter_t;


/*
 * Functions
 */

/**
 * @brief Initialize an H264Filter instance.
 *
 * The library allocates the required resources. The user must call ARSTREAM2_H264Filter_Free() to free the resources.
 *
 * @param filterHandle Pointer to the handle used in future calls to the library.
 * @param config The instance configuration.
 *
 * @return ARSTREAM2_OK if no error occurred.
 * @return an eARSTREAM2_ERROR error code if an error occurred.
 */
eARSTREAM2_ERROR ARSTREAM2_H264Filter_Init(ARSTREAM2_H264Filter_Handle *filterHandle, ARSTREAM2_H264Filter_Config_t *config);


/**
 * @brief Free an H264Filter instance.
 *
 * The library frees the allocated resources. On success the filterHandle is set to NULL.
 *
 * @param filterHandle Pointer to the instance handle.
 *
 * @return ARSTREAM2_OK if no error occurred.
 * @return an eARSTREAM2_ERROR error code if an error occurred.
 */
eARSTREAM2_ERROR ARSTREAM2_H264Filter_Free(ARSTREAM2_H264Filter_Handle *filterHandle);


/**
 * @brief Get the SPS and PPS buffers.
 *
 * The buffers are filled by the function and must be provided by the user. The size of the buffers are given
 * by a first call to the function with both buffer pointers null.
 * When the buffer pointers are not null the size pointers must point to the values of the user-allocated buffer sizes.
 *
 * @param filterHandle Instance handle.
 * @param spsBuffer SPS buffer pointer.
 * @param spsSize pointer to the SPS size.
 * @param ppsBuffer PPS buffer pointer.
 * @param ppsSize pointer to the PPS size.
 *
 * @return ARSTREAM2_OK if no error occurred.
 * @return ARSTREAM2_ERROR_WAITING_FOR_SYNC if SPS/PPS are not available (no sync).
 * @return an eARSTREAM2_ERROR error code if another error occurred.
 */
eARSTREAM2_ERROR ARSTREAM2_H264Filter_GetSpsPps(ARSTREAM2_H264Filter_Handle filterHandle, uint8_t *spsBuffer, int *spsSize, uint8_t *ppsBuffer, int *ppsSize);


int ARSTREAM2_H264Filter_GetVideoParams(ARSTREAM2_H264Filter_Handle filterHandle, int *mbWidth, int *mbHeight, int *width, int *height, float *framerate);


int ARSTREAM2_H264Filter_ProcessAu(ARSTREAM2_H264Filter_t *filter, ARSTREAM2_H264_AccessUnit_t *au);


void ARSTREAM2_H264Filter_ResetAu(ARSTREAM2_H264Filter_t *filter);


/*
 * Error concealment functions
 */

int ARSTREAM2_H264FilterError_GenerateGrayIdrFrame(ARSTREAM2_H264Filter_t *filter, ARSTREAM2_H264_AccessUnit_t *nextAu,
                                                   ARSTREAM2_H264_AuFifoItem_t *auItem);


int ARSTREAM2_H264FilterError_HandleMissingSlices(ARSTREAM2_H264Filter_t *filter, ARSTREAM2_H264_AccessUnit_t *au,
                                                  ARSTREAM2_H264_NaluFifoItem_t *nextNaluItem);


int ARSTREAM2_H264FilterError_HandleMissingEndOfFrame(ARSTREAM2_H264Filter_t *filter, ARSTREAM2_H264_AccessUnit_t *au,
                                                      ARSTREAM2_H264_NaluFifoItem_t *prevNaluItem);


#endif /* #ifndef _ARSTREAM2_H264_FILTER_H_ */
