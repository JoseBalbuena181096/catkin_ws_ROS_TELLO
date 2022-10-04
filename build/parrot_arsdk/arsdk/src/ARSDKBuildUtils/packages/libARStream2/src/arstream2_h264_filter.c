/**
 * @file arstream2_h264_filter.c
 * @brief Parrot Reception Library - H.264 Filter
 * @date 08/04/2015
 * @author aurelien.barre@parrot.com
 */

#include "arstream2_h264_filter.h"


#define ARSTREAM2_H264_FILTER_TAG "ARSTREAM2_H264Filter"


static int ARSTREAM2_H264Filter_Sync(ARSTREAM2_H264Filter_t *filter)
{
    int ret = 0;
    eARSTREAM2_ERROR err = ARSTREAM2_OK;

    /* Configure the writer */
    if (ret == 0)
    {
        ARSTREAM2_H264_SpsContext_t *spsContext = NULL;
        ARSTREAM2_H264_PpsContext_t *ppsContext = NULL;
        err = ARSTREAM2_H264Parser_GetSpsPpsContext(filter->parser, (void**)&spsContext, (void**)&ppsContext);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Parser_GetSpsPpsContext() failed (%d)", err);
            ret = -1;
        }
        else
        {
            filter->mbWidth = spsContext->pic_width_in_mbs_minus1 + 1;
            filter->mbHeight = (spsContext->pic_height_in_map_units_minus1 + 1) * ((spsContext->frame_mbs_only_flag) ? 1 : 2);
            filter->mbCount = filter->mbWidth * filter->mbHeight;
            filter->framerate = (spsContext->num_units_in_tick != 0) ? (float)spsContext->time_scale / (float)(spsContext->num_units_in_tick * 2) : 30.;
            filter->maxFrameNum = 1 << (spsContext->log2_max_frame_num_minus4 + 4);
            err = ARSTREAM2_H264Writer_SetSpsPpsContext(filter->writer, (void*)spsContext, (void*)ppsContext);
            if (err != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Parser_GetSpsPpsContext() failed (%d)", err);
                ret = -1;
            }
        }
    }

    if (ret == 0)
    {
        filter->currentAuMacroblockStatus = realloc(filter->currentAuMacroblockStatus, filter->mbCount * sizeof(uint8_t));
        filter->currentAuRefMacroblockStatus = realloc(filter->currentAuRefMacroblockStatus, filter->mbCount * sizeof(uint8_t));
        if (filter->currentAuMacroblockStatus)
        {
            memset(filter->currentAuMacroblockStatus, ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_UNKNOWN, filter->mbCount);
        }
        if (filter->currentAuRefMacroblockStatus)
        {
            memset(filter->currentAuRefMacroblockStatus, ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_UNKNOWN, filter->mbCount);
        }
        filter->previousAuFrameNum = -1;
    }

    if (ret == 0)
    {
        filter->sync = 1;

        ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_FILTER_TAG, "SPS/PPS sync OK");

        /* SPS/PPS callback */
        if (filter->spsPpsCallback)
        {
            int cbRet = filter->spsPpsCallback(filter->pSps, filter->spsSize, filter->pPps, filter->ppsSize, filter->spsPpsCallbackUserPtr);
            if (cbRet != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "spsPpsCallback failed: %s", ARSTREAM2_Error_ToString(cbRet));
            }
        }
    }

    return ret;
}


static void ARSTREAM2_H264Filter_HandleGapsInFrameNum(ARSTREAM2_H264Filter_t *filter)
{
    if ((filter->currentAuIsRef) && (filter->previousAuFrameNum != -1) && (!filter->currentAuIsIdr) && (filter->currentAuFrameNum != (filter->previousAuFrameNum + 1) % filter->maxFrameNum))
    {
        /* count missed frames (missing non-ref frames are not counted as missing) */
        int currentAuFrameNum = filter->currentAuFrameNum;
        if (currentAuFrameNum < filter->previousAuFrameNum)
        {
            currentAuFrameNum += filter->maxFrameNum;
        }
        int missed = currentAuFrameNum - filter->previousAuFrameNum - 1;

        /* ignore very large gaps in frame nums since it is probably a combination of missed frame + frame_num reset on IDR frame */
        if (missed >= filter->inferredIdrInterval) missed = 0;

        /* mark the ref as missing even if is an ignored very large gap */
        if (filter->currentAuRefMacroblockStatus)
        {
            memset(filter->currentAuRefMacroblockStatus, ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_MISSING, filter->mbCount);
        }

        filter->stats.totalFrameCount += missed;
        filter->stats.missedFrameCount += missed;

        /* update video stats macroblock status counters */
        int i, j, n;
        for (n = 0; n < missed; n++)
        {
            for (j = 0; j < filter->mbHeight; j++)
            {
                for (i = 0; i < filter->mbWidth; i++)
                {
                    int zone = j * filter->stats.mbStatusZoneCount / filter->mbHeight;
                    filter->stats.macroblockStatus[ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_MISSING][zone]++;
                }
            }
        }
    }
}


static int ARSTREAM2_H264Filter_ParseNalu(ARSTREAM2_H264Filter_t *filter, ARSTREAM2_H264_AccessUnit_t *au, ARSTREAM2_H264_NalUnit_t *nalu)
{
    int ret = 0;
    eARSTREAM2_ERROR err = ARSTREAM2_OK, _err = ARSTREAM2_OK;

    if (nalu->naluSize <= 4)
    {
        return -1;
    }

    err = ARSTREAM2_H264Parser_SetupNalu_buffer(filter->parser, nalu->nalu + 4, nalu->naluSize - 4);
    if (err != ARSTREAM2_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Parser_SetupNalu_buffer() failed (%d)", err);
    }

    if (err == ARSTREAM2_OK)
    {
        err = ARSTREAM2_H264Parser_ParseNalu(filter->parser, NULL);
        if (err < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Parser_ParseNalu() failed (%d)", err);
        }
    }

    if (err == ARSTREAM2_OK)
    {
        nalu->naluType = ARSTREAM2_H264Parser_GetLastNaluType(filter->parser);
        switch (nalu->naluType)
        {
            case ARSTREAM2_H264_NALU_TYPE_SLICE_IDR:
                au->syncType = ARSTREAM2_H264_AU_SYNC_TYPE_IDR;
                if (filter->previousAuFrameNum != -1)
                {
                    filter->inferredIdrInterval = (filter->previousAuFrameNum + 1 < ARSTREAM2_H264_FILTER_MAX_INFERRED_IDR_INTERVAL)
                            ? filter->previousAuFrameNum + 1 : ARSTREAM2_H264_FILTER_MAX_INFERRED_IDR_INTERVAL;
                }
                // fall through
            case ARSTREAM2_H264_NALU_TYPE_SLICE:
                /* Slice */
                filter->currentAuCurrentSliceFirstMb = -1;
                if (filter->sync)
                {
                    ARSTREAM2_H264Parser_SliceInfo_t sliceInfo;
                    memset(&sliceInfo, 0, sizeof(sliceInfo));
                    _err = ARSTREAM2_H264Parser_GetSliceInfo(filter->parser, &sliceInfo);
                    if (_err != ARSTREAM2_OK)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Parser_GetSliceInfo() failed (%d)", _err);
                    }
                    else
                    {
                        filter->currentAuIsIdr = (au->syncType == ARSTREAM2_H264_AU_SYNC_TYPE_IDR) ? 1 : 0;
                        filter->currentAuIsRef = (sliceInfo.nal_ref_idc != 0) ? 1 : 0;
                        if (sliceInfo.sliceTypeMod5 == 2)
                        {
                            nalu->sliceType = ARSTREAM2_H264_SLICE_TYPE_I;
                        }
                        else if (sliceInfo.sliceTypeMod5 == 0)
                        {
                            nalu->sliceType = ARSTREAM2_H264_SLICE_TYPE_P;
                            filter->currentAuSlicesAllI = 0;
                        }
                        filter->currentAuCurrentSliceFirstMb = sliceInfo.first_mb_in_slice;
                        if ((filter->sync) && (filter->currentAuFrameNum == -1))
                        {
                            filter->currentAuFrameNum = sliceInfo.frame_num;
                            ARSTREAM2_H264Filter_HandleGapsInFrameNum(filter);
                            _err = ARSTREAM2_H264Parser_GetSliceContext(filter->parser, (void**)&filter->savedSliceContext);
                            if (_err != ARSTREAM2_OK)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Parser_GetSliceContext() failed (%d)", err);
                            }
                            else
                            {
                                filter->savedSliceContextAvailable = 1;
                            }
                        }
                    }
                }
                break;
            case ARSTREAM2_H264_NALU_TYPE_SEI:
                /* SEI */
                if (filter->sync)
                {
                    int i, count, recPt;
                    void *pUserDataSei = NULL;
                    unsigned int userDataSeiSize = 0;
                    ARSTREAM2_H264Parser_RecoveryPointSei_t recoveryPoint;
                    recPt = ARSTREAM2_H264Parser_GetRecoveryPointSei(filter->parser, &recoveryPoint);
                    if (recPt == 1)
                    {
                        filter->currentAuIsRecoveryPoint = 1;
                    }
                    else if (recPt < 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Parser_GetRecoveryPointSei() failed (%d)", _err);
                    }
                    count = ARSTREAM2_H264Parser_GetUserDataSeiCount(filter->parser);
                    for (i = 0; i < count; i++)
                    {
                        _err = ARSTREAM2_H264Parser_GetUserDataSei(filter->parser, (unsigned int)i, &pUserDataSei, &userDataSeiSize);
                        if (_err != ARSTREAM2_OK)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Parser_GetUserDataSei() failed (%d)", _err);
                        }
                        else
                        {
                            if (ARSTREAM2_H264Sei_IsUserDataParrotStreamingV2(pUserDataSei, userDataSeiSize) == 1)
                            {
                                _err = ARSTREAM2_H264Sei_DeserializeUserDataParrotStreamingV2(pUserDataSei, userDataSeiSize, &filter->currentAuStreamingInfoV2);
                                if (_err != ARSTREAM2_OK)
                                {
                                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Sei_DeserializeUserDataParrotStreamingV2() failed (%d)", _err);
                                }
                                else
                                {
                                    filter->currentAuStreamingInfoAvailable = 1;
                                    filter->currentAuStreamingInfoV2Available = 1;
                                    filter->currentAuInferredSliceMbCount = filter->currentAuStreamingInfoV2.sliceMbCount;
                                    filter->currentAuStreamingSliceCount = filter->currentAuStreamingInfoV2.frameSliceCount;
                                    int k, cnt;
                                    for (k = 0, cnt = 0; k < filter->currentAuStreamingSliceCount; k++, cnt += filter->currentAuStreamingInfoV2.sliceMbCount)
                                    {
                                        filter->currentAuStreamingSliceMbCount[k] = (cnt + filter->currentAuStreamingInfoV2.sliceMbCount <= filter->mbCount) ? filter->currentAuStreamingInfoV2.sliceMbCount : filter->mbCount - count;
                                    }
                                }
                            }
                            else if (ARSTREAM2_H264Sei_IsUserDataParrotStreamingV1(pUserDataSei, userDataSeiSize) == 1)
                            {
                                _err = ARSTREAM2_H264Sei_DeserializeUserDataParrotStreamingV1(pUserDataSei, userDataSeiSize, &filter->currentAuStreamingInfoV1, filter->currentAuStreamingSliceMbCount);
                                if (_err != ARSTREAM2_OK)
                                {
                                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Sei_DeserializeUserDataParrotStreamingV1() failed (%d)", _err);
                                }
                                else
                                {
                                    filter->currentAuStreamingInfoAvailable = 1;
                                    filter->currentAuStreamingInfoV1Available = 1;
                                    filter->currentAuInferredSliceMbCount = filter->currentAuStreamingSliceMbCount[0];
                                    filter->currentAuStreamingSliceCount = filter->currentAuStreamingInfoV1.sliceCount;
                                }
                            }
                            else
                            {
                                if (userDataSeiSize > au->buffer->userDataBufferSize)
                                {
                                    uint8_t *newPtr = realloc(au->buffer->userDataBuffer, userDataSeiSize);
                                    if (newPtr == NULL)
                                    {
                                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "Reallocation failed for user data SEI (size %d)", userDataSeiSize);
                                    }
                                    else
                                    {
                                        au->buffer->userDataBuffer = newPtr;
                                        au->buffer->userDataBufferSize = userDataSeiSize;
                                    }
                                }
                                if (userDataSeiSize <= au->buffer->userDataBufferSize)
                                {
                                    memcpy(au->buffer->userDataBuffer, pUserDataSei, userDataSeiSize);
                                    au->userDataSize = userDataSeiSize;
                                }
                            }
                        }
                    }
                }
                break;
            case ARSTREAM2_H264_NALU_TYPE_SPS:
                /* SPS */
                if (!filter->spsSync)
                {
                    filter->pSps = malloc(nalu->naluSize);
                    if (!filter->pSps)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "Allocation failed for SPS (size %d)", nalu->naluSize);
                    }
                    else
                    {
                        memcpy(filter->pSps, nalu->nalu, nalu->naluSize);
                        filter->spsSize = nalu->naluSize;
                        filter->spsSync = 1;
                    }
                }
                break;
            case ARSTREAM2_H264_NALU_TYPE_PPS:
                /* PPS */
                if (!filter->ppsSync)
                {
                    filter->pPps = malloc(nalu->naluSize);
                    if (!filter->pPps)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "Allocation failed for PPS (size %d)", nalu->naluSize);
                    }
                    else
                    {
                        memcpy(filter->pPps, nalu->nalu, nalu->naluSize);
                        filter->ppsSize = nalu->naluSize;
                        filter->ppsSync = 1;
                    }
                }
                break;
            default:
                break;
        }
    }

    if ((filter->spsSync) && (filter->ppsSync) && (!filter->sync))
    {
        ret = ARSTREAM2_H264Filter_Sync(filter);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Filter_Sync() failed (%d)", ret);
        }
    }

    return (ret >= 0) ? 0 : ret;
}


void ARSTREAM2_H264Filter_ResetAu(ARSTREAM2_H264Filter_t *filter)
{
    filter->currentAuIncomplete = 0;
    filter->currentAuSlicesAllI = 1;
    filter->currentAuSlicesReceived = 0;
    filter->currentAuStreamingInfoV1Available = 0;
    if (!filter->currentAuStreamingInfoV2Available)
    {
        filter->currentAuStreamingInfoAvailable = 0;
    }
    filter->currentAuIsRecoveryPoint = 0;
    filter->currentAuPreviousSliceIndex = -1;
    filter->currentAuPreviousSliceFirstMb = 0;
    filter->currentAuInferredPreviousSliceFirstMb = 0;
    filter->currentAuCurrentSliceFirstMb = -1;
    filter->previousSliceType = ARSTREAM2_H264_SLICE_TYPE_NON_VCL;
    if ((filter->sync) && (filter->currentAuMacroblockStatus))
    {
        memset(filter->currentAuMacroblockStatus, ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_UNKNOWN, filter->mbCount);
    }
    if (filter->currentAuIsRef) filter->previousAuFrameNum = filter->currentAuFrameNum;
    filter->currentAuFrameNum = -1;
    filter->currentAuIsIdr = 0;
    filter->currentAuIsRef = 0;
    filter->savedSliceContextAvailable = 0;
}


static void ARSTREAM2_H264Filter_FillSliceMbStatus(ARSTREAM2_H264Filter_t *filter, uint8_t sliceType, int sliceFirstMb, int sliceMbCount)
{
    int i, idx;
    uint8_t status;
    if (sliceFirstMb + sliceMbCount > filter->mbCount) sliceMbCount = filter->mbCount - sliceFirstMb;
    for (i = 0, idx = sliceFirstMb; i < sliceMbCount; i++, idx++)
    {
        if (sliceType == ARSTREAM2_H264_SLICE_TYPE_I)
        {
            status = ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_VALID_ISLICE;
        }
        else
        {
            if ((!filter->currentAuRefMacroblockStatus)
                    || ((filter->currentAuRefMacroblockStatus[idx] != ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_VALID_ISLICE) && (filter->currentAuRefMacroblockStatus[idx] != ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_VALID_PSLICE)))
            {
                status = ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_ERROR_PROPAGATION;
            }
            else
            {
                status = ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_VALID_PSLICE;
            }
        }
        filter->currentAuMacroblockStatus[idx] = status;
    }
}


static int ARSTREAM2_H264Filter_ProcessNalu(ARSTREAM2_H264Filter_t *filter, ARSTREAM2_H264_NalUnit_t *nalu)
{
    int ret = 0;

    if ((nalu->naluType == ARSTREAM2_H264_NALU_TYPE_SLICE_IDR) || (nalu->naluType == ARSTREAM2_H264_NALU_TYPE_SLICE))
    {
        int sliceMbCount = 0, sliceFirstMb = 0;
        int previousSliceMbCount = 0, previousSliceFirstMb = 0;
        filter->currentAuSlicesReceived = 1;
        if (filter->currentAuStreamingInfoAvailable)
        {
            if (filter->currentAuStreamingSliceCount <= ARSTREAM2_H264_SEI_PARROT_STREAMING_MAX_SLICE_COUNT)
            {
                // Update slice index and firstMb
                if (filter->currentAuPreviousSliceIndex < 0)
                {
                    filter->currentAuPreviousSliceFirstMb = 0;
                    filter->currentAuPreviousSliceIndex = 0;
                }
                while ((filter->currentAuPreviousSliceIndex < filter->currentAuStreamingSliceCount) && (filter->currentAuPreviousSliceFirstMb < filter->currentAuCurrentSliceFirstMb))
                {
                    filter->currentAuPreviousSliceFirstMb += filter->currentAuStreamingSliceMbCount[filter->currentAuPreviousSliceIndex];
                    filter->currentAuPreviousSliceIndex++;
                }
                sliceFirstMb = filter->currentAuPreviousSliceFirstMb = filter->currentAuCurrentSliceFirstMb;
                sliceMbCount = filter->currentAuStreamingSliceMbCount[filter->currentAuPreviousSliceIndex];
                //ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_H264_FILTER_TAG, "previousSliceIndex: %d - previousSliceFirstMb: %d", filter->currentAuPreviousSliceIndex, filter->currentAuCurrentSliceFirstMb); //TODO: debug
            }
        }
        else if (filter->currentAuCurrentSliceFirstMb >= 0)
        {
            previousSliceFirstMb = filter->currentAuInferredPreviousSliceFirstMb;
            previousSliceMbCount = filter->currentAuCurrentSliceFirstMb - previousSliceFirstMb;
            sliceFirstMb = filter->currentAuInferredPreviousSliceFirstMb = filter->currentAuCurrentSliceFirstMb;
            sliceMbCount = (filter->currentAuInferredSliceMbCount > 0) ? filter->currentAuInferredSliceMbCount : 0;
        }
        if ((filter->sync) && (filter->currentAuMacroblockStatus) && (sliceFirstMb >= 0) && (sliceMbCount > 0))
        {
            ARSTREAM2_H264Filter_FillSliceMbStatus(filter, nalu->sliceType, sliceFirstMb, sliceMbCount);
        }
        else if ((filter->sync) && (filter->currentAuMacroblockStatus) && (!filter->currentAuStreamingInfoAvailable)
                && (nalu->isLastInAu) && (sliceFirstMb >= 0))
        {
            // Fix the current slice MB status in case it is the last slice of the frame and no streaming info is available
            ARSTREAM2_H264Filter_FillSliceMbStatus(filter, nalu->sliceType, sliceFirstMb, filter->mbCount - sliceFirstMb);
        }
        if ((filter->sync) && (filter->currentAuMacroblockStatus) && (!filter->currentAuStreamingInfoAvailable)
                && (nalu->missingPacketsBefore == 0) && (previousSliceFirstMb >= 0) && (previousSliceMbCount > 0))
        {
            // Fix the previous slice MB status in case no streaming info is available
            ARSTREAM2_H264Filter_FillSliceMbStatus(filter, filter->previousSliceType, previousSliceFirstMb, previousSliceMbCount);
        }
    }

    return ret;
}


int ARSTREAM2_H264Filter_ProcessAu(ARSTREAM2_H264Filter_t *filter, ARSTREAM2_H264_AccessUnit_t *au)
{
    ARSTREAM2_H264_NaluFifoItem_t *naluItem, *prevNaluItem = NULL;
    int cancelAuOutput = 0, discarded = 0, hasErrors = 0;
    int ret = 0, err;

    if ((!filter) || (!au))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "Invalid pointer");
        return -1;
    }

    if (filter->resyncPending)
    {
        filter->sync = 0;
        filter->resyncPending = 0;
    }

    ARSTREAM2_H264Filter_ResetAu(filter);

    /* process the NAL units */
    for (naluItem = au->naluHead; naluItem; naluItem = naluItem->next)
    {
        err = ARSTREAM2_H264Filter_ParseNalu(filter, au, &naluItem->nalu);
        if (err < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Filter_ParseNalu() failed (%d)", err);
        }

        if (err == 0)
        {
            if (naluItem->nalu.missingPacketsBefore)
            {
                /* error concealment: missing slices before the current slice */
                err = ARSTREAM2_H264FilterError_HandleMissingSlices(filter, au, naluItem);
                if (err < 0)
                {
                    if (err != -2)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264FilterError_HandleMissingSlices() failed (%d)", err);
                    }
                    filter->currentAuIncomplete = 1;
                }
            }
            else if (filter->sync)
            {
                if ((filter->currentAuPreviousSliceIndex < 0) && (filter->currentAuInferredPreviousSliceFirstMb < 0))
                {
                    if (filter->currentAuCurrentSliceFirstMb > 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "FIXME! missingPacketsBefore==0 but currentSliceFirstMb=%d and expectedSliceFirstMb=0, this should not happen!",
                                    filter->currentAuCurrentSliceFirstMb);
                    }
                }
                else if ((filter->currentAuPreviousSliceIndex >= 0) && (filter->currentAuStreamingInfoAvailable))
                {
                    if (filter->currentAuCurrentSliceFirstMb != filter->currentAuPreviousSliceFirstMb + filter->currentAuStreamingSliceMbCount[filter->currentAuPreviousSliceIndex])
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "FIXME! missingPacketsBefore==0 but currentSliceFirstMb=%d and expectedSliceFirstMb=%d, this should not happen!",
                                    filter->currentAuCurrentSliceFirstMb, filter->currentAuPreviousSliceFirstMb + filter->currentAuStreamingSliceMbCount[filter->currentAuPreviousSliceIndex]);
                    }
                }
            }

            err = ARSTREAM2_H264Filter_ProcessNalu(filter, &naluItem->nalu);
            if (err != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Filter_ProcessNalu() failed (%d)", err);
            }

            filter->previousSliceType = naluItem->nalu.sliceType;
        }

        prevNaluItem = naluItem;
    }

    if ((prevNaluItem) && (prevNaluItem->nalu.isLastInAu == 0))
    {
        /* error concealment: missing slices at the end of frame */
        err = ARSTREAM2_H264FilterError_HandleMissingEndOfFrame(filter, au, prevNaluItem);
        if (err < 0)
        {
            if (err != -2)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264FilterError_HandleMissingEndOfFrame() failed (%d)", err);
            }
            filter->currentAuIncomplete = 1;
        }
    }

    if (filter->currentAuMacroblockStatus)
    {
        int k;
        for (k = 0; k < filter->mbCount; k++)
        {
            if ((filter->currentAuMacroblockStatus[k] != ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_VALID_ISLICE)
                    && (filter->currentAuMacroblockStatus[k] != ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_VALID_PSLICE))
            {
                hasErrors = 1;
            }
        }
    }

    if (au->syncType != ARSTREAM2_H264_AU_SYNC_TYPE_IDR)
    {
        if (filter->currentAuSlicesAllI)
        {
            au->syncType = ARSTREAM2_H264_AU_SYNC_TYPE_IFRAME;
        }
        else if ((filter->currentAuStreamingInfoV1Available) && (filter->currentAuStreamingInfoV1.indexInGop == 0))
        {
            au->syncType = ARSTREAM2_H264_AU_SYNC_TYPE_PIR_START;
        }
        else if (filter->currentAuIsRecoveryPoint)
        {
            au->syncType = ARSTREAM2_H264_AU_SYNC_TYPE_PIR_START;
        }
    }
    au->isComplete = (filter->currentAuIncomplete) ? 0 : 1;
    au->hasErrors = hasErrors;
    au->isRef = (filter->currentAuIsRef) ? 1 : 0;

    if ((!filter->outputIncompleteAu) && (filter->currentAuIncomplete))
    {
        /* filter out incomplete access units */
        cancelAuOutput = 1;
        discarded = 1;
        ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_FILTER_TAG, "AU output cancelled (!outputIncompleteAu)"); //TODO: debug
    }

    if (!cancelAuOutput)
    {
        if (!filter->sync)
        {
            /* cancel if not synchronized */
            cancelAuOutput = 1;
            ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_H264_FILTER_TAG, "AU output cancelled (waitForSync)"); //TODO: debug
        }
    }

    if (!cancelAuOutput)
    {
        if (filter->currentAuMacroblockStatus)
        {
            err = ARSTREAM2_H264_AuMbStatusCheckSizeRealloc(au, filter->mbCount);
            if (err == 0)
            {
                memcpy(au->buffer->mbStatusBuffer, filter->currentAuMacroblockStatus, filter->mbCount);
                au->mbStatusAvailable = 1;
            }
            else
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "MB status buffer is too small");
            }
        }

        ret = 1;
        filter->currentAuOutputIndex++;
    }

    struct timespec t1;
    ARSAL_Time_GetTime(&t1);
    uint64_t curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

    /* update the stats */
    if (filter->sync)
    {
        if ((filter->currentAuMacroblockStatus) && ((discarded) || (ret != 1)) && (filter->currentAuIsRef))
        {
            /* missed frame (missing non-ref frames are not counted as missing) */
            memset(filter->currentAuMacroblockStatus, ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_MISSING, filter->mbCount);
        }
        if (filter->currentAuMacroblockStatus)
        {
            /* update macroblock status and error second counters */
            int i, j, k;
            for (j = 0, k = 0; j < filter->mbHeight; j++)
            {
                for (i = 0; i < filter->mbWidth; i++, k++)
                {
                    int zone = j * filter->stats.mbStatusZoneCount / filter->mbHeight;
                    filter->stats.macroblockStatus[filter->currentAuMacroblockStatus[k]][zone]++;

                    if ((ret == 1) && (filter->currentAuMacroblockStatus[k] != ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_VALID_ISLICE)
                            && (filter->currentAuMacroblockStatus[k] != ARSTREAM2_STREAM_STATS_MACROBLOCK_STATUS_VALID_PSLICE))
                    {
                        //TODO: we should not use curTime but an AU timestamp
                        if (curTime > filter->stats.erroredSecondStartTime + 1000000)
                        {
                            filter->stats.erroredSecondStartTime = curTime;
                            filter->stats.erroredSecondCount++;
                        }
                        if (curTime > filter->stats.erroredSecondStartTimeByZone[zone] + 1000000)
                        {
                            filter->stats.erroredSecondStartTimeByZone[zone] = curTime;
                            filter->stats.erroredSecondCountByZone[zone]++;
                        }
                    }
                }
            }
        }
        if (filter->currentAuSlicesReceived)
        {
            /* Count all frames for which at least 1 slice has been received
             * (missing ref frames for which no slice has been received will
             * be counted by the gap in frame_num on the next frame).
             *
             * Totally missing non-ref frames are never counted because there
             * is no way of knowing of their existence (their frame_num is the
             * same as the next ref frame).
             *
             * Non-ref frames that are discarded are not counted as discarded/missed
             * frames. Therefore, totalFrameCount = outputFrameCount
             *                                    + missedFrameCount
             *                                    + (missedNonRefFrameCount)
             *
             * discardedFrameCount is included in missedFrameCount.
             */
            filter->stats.totalFrameCount++;
            if (ret == 1)
            {
                /* count all output frames (including non-ref frames) */
                filter->stats.outputFrameCount++;
                if (hasErrors)
                {
                    filter->stats.erroredOutputFrameCount++;
                }
            }
            if ((discarded) && (filter->currentAuIsRef))
            {
                /* count discarded frames (discarded non-ref frames are not counted) */
                filter->stats.discardedFrameCount++;
            }
            if (((discarded) || (ret != 1)) && (filter->currentAuIsRef))
            {
                /* count missed frames (missing non-ref frames are not counted) */
                filter->stats.missedFrameCount++;
            }
        }
        if (filter->currentAuIsRef)
        {
            /* reference frame => exchange macroblock status buffers */
            uint8_t *tmp = filter->currentAuMacroblockStatus;
            filter->currentAuMacroblockStatus = filter->currentAuRefMacroblockStatus;
            filter->currentAuRefMacroblockStatus = tmp;
        }

        if ((au->buffer->videoStatsBuffer) && (au->buffer->videoStatsBufferSize >= sizeof(ARSTREAM2_H264_VideoStats_t)))
        {
            memcpy(au->buffer->videoStatsBuffer, &filter->stats, sizeof(ARSTREAM2_H264_VideoStats_t));
            au->videoStatsAvailable = 1;
        }
        else
        {
            au->videoStatsAvailable = 0;
        }
    }
    else
    {
        if ((au->buffer->videoStatsBuffer) && (au->buffer->videoStatsBufferSize >= sizeof(ARSTREAM2_H264_VideoStats_t)))
        {
            memset(au->buffer->videoStatsBuffer, 0, sizeof(ARSTREAM2_H264_VideoStats_t));
        }
        au->videoStatsAvailable = 0;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_H264Filter_GetSpsPps(ARSTREAM2_H264Filter_Handle filterHandle, uint8_t *spsBuffer, int *spsSize, uint8_t *ppsBuffer, int *ppsSize)
{
    ARSTREAM2_H264Filter_t* filter = (ARSTREAM2_H264Filter_t*)filterHandle;
    int ret = ARSTREAM2_OK;

    if (!filterHandle)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if ((!spsSize) || (!ppsSize))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (!filter->sync)
    {
        ret = ARSTREAM2_ERROR_WAITING_FOR_SYNC;
    }

    if (ret == ARSTREAM2_OK)
    {
        if ((!spsBuffer) || (*spsSize < filter->spsSize))
        {
            *spsSize = filter->spsSize;
        }
        else
        {
            memcpy(spsBuffer, filter->pSps, filter->spsSize);
            *spsSize = filter->spsSize;
        }

        if ((!ppsBuffer) || (*ppsSize < filter->ppsSize))
        {
            *ppsSize = filter->ppsSize;
        }
        else
        {
            memcpy(ppsBuffer, filter->pPps, filter->ppsSize);
            *ppsSize = filter->ppsSize;
        }
    }

    return ret;
}


int ARSTREAM2_H264Filter_GetVideoParams(ARSTREAM2_H264Filter_Handle filterHandle, int *mbWidth, int *mbHeight, int *width, int *height, float *framerate)
{
    ARSTREAM2_H264Filter_t* filter = (ARSTREAM2_H264Filter_t*)filterHandle;
    int ret = 0;

    if (!filterHandle)
    {
        return -1;
    }

    if (!filter->sync)
    {
        return -1;
    }

    if (mbWidth) *mbWidth = filter->mbWidth; //TODO
    if (mbHeight) *mbHeight = filter->mbHeight; //TODO
    if (width) *width = filter->mbWidth * 16; //TODO
    if (height) *height = filter->mbHeight * 16; //TODO
    if (framerate) *framerate = filter->framerate;

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_H264Filter_Init(ARSTREAM2_H264Filter_Handle *filterHandle, ARSTREAM2_H264Filter_Config_t *config)
{
    ARSTREAM2_H264Filter_t* filter;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!filterHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "Invalid pointer for handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!config)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "Invalid pointer for config");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    filter = (ARSTREAM2_H264Filter_t*)malloc(sizeof(*filter));
    if (!filter)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "Allocation failed (size %zu)", sizeof(*filter));
        ret = ARSTREAM2_ERROR_ALLOC;
    }

    if (ret == ARSTREAM2_OK)
    {
        memset(filter, 0, sizeof(*filter));

        filter->outputIncompleteAu = (config->outputIncompleteAu > 0) ? 1 : 0;
        filter->generateSkippedPSlices = (config->generateSkippedPSlices > 0) ? 1 : 0;
        filter->spsPpsCallback = config->spsPpsCallback;
        filter->spsPpsCallbackUserPtr = config->spsPpsCallbackUserPtr;
        filter->stats.mbStatusZoneCount = ARSTREAM2_H264_MB_STATUS_ZONE_COUNT;
        filter->stats.mbStatusClassCount = ARSTREAM2_H264_MB_STATUS_CLASS_COUNT;
        filter->inferredIdrInterval = ARSTREAM2_H264_FILTER_MAX_INFERRED_IDR_INTERVAL;
    }

    if (ret == ARSTREAM2_OK)
    {
        ARSTREAM2_H264Parser_Config_t parserConfig;
        memset(&parserConfig, 0, sizeof(parserConfig));
        parserConfig.extractUserDataSei = 1;
        parserConfig.printLogs = 0;

        ret = ARSTREAM2_H264Parser_Init(&(filter->parser), &parserConfig);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Parser_Init() failed (%d)", ret);
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        ARSTREAM2_H264Writer_Config_t writerConfig;
        memset(&writerConfig, 0, sizeof(writerConfig));
        writerConfig.naluPrefix = 1;

        ret = ARSTREAM2_H264Writer_Init(&(filter->writer), &writerConfig);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_H264_FILTER_TAG, "ARSTREAM2_H264Writer_Init() failed (%d)", ret);
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        *filterHandle = filter;
    }
    else
    {
        if (filter)
        {
            if (filter->parser) ARSTREAM2_H264Parser_Free(filter->parser);
            if (filter->writer) ARSTREAM2_H264Writer_Free(filter->writer);
            free(filter);
        }
        *filterHandle = NULL;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_H264Filter_Free(ARSTREAM2_H264Filter_Handle *filterHandle)
{
    ARSTREAM2_H264Filter_t* filter;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if ((!filterHandle) || (!*filterHandle))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    filter = (ARSTREAM2_H264Filter_t*)*filterHandle;

    ARSTREAM2_H264Parser_Free(filter->parser);
    ARSTREAM2_H264Writer_Free(filter->writer);

    free(filter->currentAuMacroblockStatus);
    free(filter->currentAuRefMacroblockStatus);
    free(filter->pSps);
    free(filter->pPps);

    free(filter);
    *filterHandle = NULL;

    return ret;
}
