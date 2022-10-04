/**
 * @file arstream2_stream_receiver.c
 * @brief Parrot Streaming Library - Stream Receiver
 * @date 08/04/2015
 * @author aurelien.barre@parrot.com
 */

#include <stdio.h>
#include <stdlib.h>

#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Mutex.h>
#include <libARSAL/ARSAL_Thread.h>

#include <libARStream2/arstream2_stream_receiver.h>
#include "arstream2_stream_recorder.h"
#include "arstream2_rtp_receiver.h"
#include "arstream2_rtp_resender.h"
#include "arstream2_h264_filter.h"
#include "arstream2_h264.h"
#include "arstream2_stream_stats_internal.h"


#define ARSTREAM2_STREAM_RECEIVER_TAG "ARSTREAM2_StreamReceiver"

#define ARSTREAM2_STREAM_RECEIVER_DEFAULT_MIN_PACKET_FIFO_BUFFER_COUNT (500)
#define ARSTREAM2_STREAM_RECEIVER_DEFAULT_PACKET_FIFO_BUFFER_TO_ITEM_FACTOR (4)
#define ARSTREAM2_STREAM_RECEIVER_DEFAULT_MIN_PACKET_FIFO_ITEM_COUNT (ARSTREAM2_STREAM_RECEIVER_DEFAULT_MIN_PACKET_FIFO_BUFFER_COUNT * ARSTREAM2_STREAM_RECEIVER_DEFAULT_PACKET_FIFO_BUFFER_TO_ITEM_FACTOR)

#define ARSTREAM2_STREAM_RECEIVER_DEFAULT_AU_FIFO_ITEM_COUNT (200)
#define ARSTREAM2_STREAM_RECEIVER_DEFAULT_AU_FIFO_ITEM_NALU_COUNT (128)
#define ARSTREAM2_STREAM_RECEIVER_DEFAULT_AU_FIFO_BUFFER_COUNT (60)

#define ARSTREAM2_STREAM_RECEIVER_AU_BUFFER_SIZE (128 * 1024)
#define ARSTREAM2_STREAM_RECEIVER_AU_METADATA_BUFFER_SIZE (1024)
#define ARSTREAM2_STREAM_RECEIVER_AU_USER_DATA_BUFFER_SIZE (1024)

#define ARSTREAM2_STREAM_RECEIVER_VIDEO_AUTOREC_OUTPUT_PATH "videorec"
#define ARSTREAM2_STREAM_RECEIVER_VIDEO_AUTOREC_OUTPUT_FILENAME "videorec"
#define ARSTREAM2_STREAM_RECEIVER_VIDEO_AUTOREC_OUTPUT_FILEEXT "mp4"

#define ARSTREAM2_STREAM_RECEIVER_VIDEO_STATS_RTCP_SEND_INTERVAL (1000000)
#define ARSTREAM2_STREAM_RECEIVER_LOSS_REPORT_RTCP_SEND_INTERVAL (500000)
#define ARSTREAM2_STREAM_RECEIVER_DJB_REPORT_RTCP_SEND_INTERVAL (1000000)
#define ARSTREAM2_STREAM_RECEIVER_UNTIMED_METADATA_DEFAULT_SEND_INTERVAL (5000000)


typedef struct ARSTREAM2_StreamReceiver_s
{
    ARSTREAM2_RTP_PacketFifo_t packetFifo;
    ARSTREAM2_RTP_PacketFifoQueue_t packetFifoQueue;
    ARSTREAM2_H264_AuFifo_t auFifo;
    ARSTREAM2_H264Filter_Handle filter;
    ARSTREAM2_RtpReceiver_t *receiver;
    ARSTREAM2_RtpResender_t *resender;
    ARSTREAM2_RTP_PacketFifoQueue_t **resendQueue;
    uint32_t *resendTimeout;
    unsigned int resendCount;
    ARSAL_Mutex_t resendMutex;

    int maxPacketSize;
    int sync;
    uint8_t *pSps;
    int spsSize;
    uint8_t *pPps;
    int ppsSize;
    uint64_t lastAuNtpTimestamp;
    uint64_t lastAuNtpTimestampRaw;
    uint64_t lastAuOutputTimestamp;
    uint64_t timestampDeltaIntegral;
    uint64_t timestampDeltaIntegralSq;
    uint64_t timingErrorIntegral;
    uint64_t timingErrorIntegralSq;
    uint64_t estimatedLatencyIntegral;
    uint64_t estimatedLatencyIntegralSq;

    /* Network thread status */
    ARSAL_Mutex_t threadMutex;
    int threadStarted;
    int threadShouldStop;
    int signalPipe[2];

    struct
    {
        ARSTREAM2_H264_AuFifoQueue_t auFifoQueue;
        int generateGrayIFrame;
        int grayIFramePending;
        int filterOutSpsPps;
        int filterOutSei;
        int replaceStartCodesWithNaluSize;
        ARSAL_Mutex_t threadMutex;
        ARSAL_Cond_t threadCond;
        int threadRunning;
        int threadShouldStop;
        int running;
        ARSAL_Mutex_t callbackMutex;
        ARSAL_Cond_t callbackCond;
        int callbackInProgress;
        ARSTREAM2_StreamReceiver_SpsPpsCallback_t spsPpsCallback;
        void *spsPpsCallbackUserPtr;
        ARSTREAM2_StreamReceiver_GetAuBufferCallback_t getAuBufferCallback;
        void *getAuBufferCallbackUserPtr;
        ARSTREAM2_StreamReceiver_AuReadyCallback_t auReadyCallback;
        void *auReadyCallbackUserPtr;
        int mbWidth;
        int mbHeight;
        ARSTREAM2_StreamStats_VideoStats_t videoStats;

    } appOutput;

    struct
    {
        ARSTREAM2_H264_AuFifoQueue_t auFifoQueue;
        char *fileName;
        int ardiscoveryProductType;
        time_t startTime;
        int startPending;
        int running;
        int generateGrayIFrame;
        int grayIFramePending;
        ARSAL_Thread_t thread;
        ARSAL_Mutex_t threadMutex;
        ARSAL_Cond_t threadCond;
        ARSTREAM2_StreamRecorder_Handle recorder;

    } recorder;

    /* Debug files */
    char *friendlyName;
    char *dateAndTime;
    char *debugPath;
    ARSTREAM2_StreamStats_VideoStatsContext_t videoStatsCtx;
    ARSTREAM2_StreamStats_RtpStatsContext_t rtpStatsCtx;
    ARSTREAM2_StreamStats_RtpLossContext_t rtpLossCtx;
    int8_t lastKnownRssi;

} ARSTREAM2_StreamReceiver_t;


static int ARSTREAM2_StreamReceiver_GenerateGrayIdrFrame(ARSTREAM2_StreamReceiver_t *streamReceiver, ARSTREAM2_H264_AccessUnit_t *nextAu);
static int ARSTREAM2_StreamReceiver_RtpReceiverAuCallback(ARSTREAM2_H264_AuFifoItem_t *auItem, void *userPtr);
static void ARSTREAM2_StreamReceiver_RtpReceiverStatsCallback(const ARSTREAM2_RTP_RtpStats_t *rtpStats, void *userPtr);
static int ARSTREAM2_StreamReceiver_H264FilterSpsPpsCallback(uint8_t *spsBuffer, int spsSize, uint8_t *ppsBuffer, int ppsSize, void *userPtr);
static int ARSTREAM2_StreamReceiver_StreamRecorderInit(ARSTREAM2_StreamReceiver_t *streamReceiver);
static int ARSTREAM2_StreamReceiver_StreamRecorderStop(ARSTREAM2_StreamReceiver_t *streamReceiver);
static int ARSTREAM2_StreamReceiver_StreamRecorderFree(ARSTREAM2_StreamReceiver_t *streamReceiver);
static void ARSTREAM2_StreamReceiver_AutoStartRecorder(ARSTREAM2_StreamReceiver_t *streamReceiver);


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_Init(ARSTREAM2_StreamReceiver_Handle *streamReceiverHandle,
                                               const ARSTREAM2_StreamReceiver_Config_t *config,
                                               const ARSTREAM2_StreamReceiver_NetConfig_t *net_config,
                                               const ARSTREAM2_StreamReceiver_MuxConfig_t *mux_config)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    ARSTREAM2_StreamReceiver_t *streamReceiver = NULL;
    int auFifoCreated = 0, packetFifoWasCreated = 0;
    int appOutputThreadMutexInit = 0, appOutputThreadCondInit = 0;
    int appOutputCallbackMutexInit = 0, appOutputCallbackCondInit = 0;
    int recorderThreadMutexInit = 0, recorderThreadCondInit = 0;
    int threadMutexInit = 0, resendMutexInit = 0;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid pointer for handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!config)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid pointer for config");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!net_config && !mux_config)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "No net nor mux config");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (net_config && mux_config)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Both net and mux config provided");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    int usemux = mux_config != NULL;

    streamReceiver = (ARSTREAM2_StreamReceiver_t*)malloc(sizeof(*streamReceiver));
    if (!streamReceiver)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Allocation failed (size %zu)", sizeof(*streamReceiver));
        ret = ARSTREAM2_ERROR_ALLOC;
    }

    if (ret == ARSTREAM2_OK)
    {
        memset(streamReceiver, 0, sizeof(*streamReceiver));
        streamReceiver->signalPipe[0] = -1;
        streamReceiver->signalPipe[1] = -1;
        streamReceiver->maxPacketSize = (config->maxPacketSize > 0) ? config->maxPacketSize - ARSTREAM2_RTP_TOTAL_HEADERS_SIZE : ARSTREAM2_RTP_MAX_PAYLOAD_SIZE;
        streamReceiver->appOutput.generateGrayIFrame = (config->generateFirstGrayIFrame > 0) ? 1 : 0;
        streamReceiver->appOutput.filterOutSpsPps = (config->filterOutSpsPps > 0) ? 1 : 0;
        streamReceiver->appOutput.filterOutSei = (config->filterOutSei > 0) ? 1 : 0;
        streamReceiver->appOutput.replaceStartCodesWithNaluSize = (config->replaceStartCodesWithNaluSize > 0) ? 1 : 0;
        if ((config->debugPath) && (strlen(config->debugPath)))
        {
            streamReceiver->debugPath = strdup(config->debugPath);
        }
        if ((config->friendlyName) && (strlen(config->friendlyName)))
        {
            streamReceiver->friendlyName = strndup(config->friendlyName, 40);
        }
        else if ((config->canonicalName) && (strlen(config->canonicalName)))
        {
            streamReceiver->friendlyName = strndup(config->canonicalName, 40);
        }
        streamReceiver->appOutput.videoStats.mbStatusClassCount = ARSTREAM2_H264_MB_STATUS_CLASS_COUNT;
        streamReceiver->appOutput.videoStats.mbStatusZoneCount = ARSTREAM2_H264_MB_STATUS_ZONE_COUNT;
        streamReceiver->appOutput.videoStats.erroredSecondCountByZone = malloc(ARSTREAM2_H264_MB_STATUS_ZONE_COUNT * sizeof(uint32_t));
        if (!streamReceiver->appOutput.videoStats.erroredSecondCountByZone)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Allocation failed");
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        streamReceiver->appOutput.videoStats.macroblockStatus = malloc(ARSTREAM2_H264_MB_STATUS_ZONE_COUNT * ARSTREAM2_H264_MB_STATUS_CLASS_COUNT * sizeof(uint32_t));
        if (!streamReceiver->appOutput.videoStats.macroblockStatus)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Allocation failed");
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        streamReceiver->recorder.ardiscoveryProductType = config->ardiscoveryProductType;
        streamReceiver->recorder.generateGrayIFrame = (config->generateFirstGrayIFrame > 0) ? 1 : 0;
        char szDate[200];
        time_t rawtime;
        struct tm timeinfo;
        time(&rawtime);
        localtime_r(&rawtime, &timeinfo);
        /* Date format : <YYYY-MM-DDTHHMMSS+HHMM */
        strftime(szDate, 200, "%FT%H%M%S%z", &timeinfo);
        streamReceiver->dateAndTime = strndup(szDate, 200);
        ARSTREAM2_StreamReceiver_AutoStartRecorder(streamReceiver);
        ARSTREAM2_StreamStats_VideoStatsFileOpen(&streamReceiver->videoStatsCtx, streamReceiver->debugPath, streamReceiver->friendlyName,
                                                 streamReceiver->dateAndTime, ARSTREAM2_H264_MB_STATUS_ZONE_COUNT, ARSTREAM2_H264_MB_STATUS_CLASS_COUNT);
        ARSTREAM2_StreamStats_RtpStatsFileOpen(&streamReceiver->rtpStatsCtx, streamReceiver->debugPath,
                                               streamReceiver->friendlyName, streamReceiver->dateAndTime);
        ARSTREAM2_StreamStats_RtpLossFileOpen(&streamReceiver->rtpLossCtx, streamReceiver->debugPath,
                                              streamReceiver->friendlyName, streamReceiver->dateAndTime);
    }

    if (ret == ARSTREAM2_OK)
    {
        if (pipe(streamReceiver->signalPipe) != 0)
        {
            ret = ARSTREAM2_ERROR_RESOURCE_UNAVAILABLE;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init(&(streamReceiver->threadMutex));
        if (mutexInitRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Mutex creation failed (%d)", mutexInitRet);
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            threadMutexInit = 1;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init(&(streamReceiver->resendMutex));
        if (mutexInitRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Mutex creation failed (%d)", mutexInitRet);
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            resendMutexInit = 1;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init(&(streamReceiver->appOutput.threadMutex));
        if (mutexInitRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Mutex creation failed (%d)", mutexInitRet);
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            appOutputThreadMutexInit = 1;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        int condInitRet = ARSAL_Cond_Init(&(streamReceiver->appOutput.threadCond));
        if (condInitRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Cond creation failed (%d)", condInitRet);
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            appOutputThreadCondInit = 1;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init(&(streamReceiver->appOutput.callbackMutex));
        if (mutexInitRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Mutex creation failed (%d)", mutexInitRet);
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            appOutputCallbackMutexInit = 1;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        int condInitRet = ARSAL_Cond_Init(&(streamReceiver->appOutput.callbackCond));
        if (condInitRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Cond creation failed (%d)", condInitRet);
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            appOutputCallbackCondInit = 1;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init(&(streamReceiver->recorder.threadMutex));
        if (mutexInitRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Mutex creation failed (%d)", mutexInitRet);
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            recorderThreadMutexInit = 1;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        int condInitRet = ARSAL_Cond_Init(&(streamReceiver->recorder.threadCond));
        if (condInitRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Cond creation failed (%d)", condInitRet);
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            recorderThreadCondInit = 1;
        }
    }

    /* Setup the packet FIFO */
    if (ret == ARSTREAM2_OK)
    {
        int packetFifoRet = ARSTREAM2_RTP_PacketFifoInit(&streamReceiver->packetFifo, ARSTREAM2_STREAM_RECEIVER_DEFAULT_MIN_PACKET_FIFO_ITEM_COUNT,
                                                         ARSTREAM2_STREAM_RECEIVER_DEFAULT_MIN_PACKET_FIFO_BUFFER_COUNT,
                                                         streamReceiver->maxPacketSize);
        if (packetFifoRet != 0)
        {
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            packetFifoRet = ARSTREAM2_RTP_PacketFifoAddQueue(&streamReceiver->packetFifo, &streamReceiver->packetFifoQueue);
            if (packetFifoRet != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RTP_PacketFifoAddQueue() failed (%d)", packetFifoRet);
                ret = ARSTREAM2_ERROR_ALLOC;
            }
            packetFifoWasCreated = 1;
        }
    }

    /* Setup the access unit FIFO */
    if (ret == ARSTREAM2_OK)
    {
        int auFifoRet = ARSTREAM2_H264_AuFifoInit(&streamReceiver->auFifo,
                                                  ARSTREAM2_STREAM_RECEIVER_DEFAULT_AU_FIFO_ITEM_COUNT,
                                                  ARSTREAM2_STREAM_RECEIVER_DEFAULT_AU_FIFO_ITEM_NALU_COUNT,
                                                  ARSTREAM2_STREAM_RECEIVER_DEFAULT_AU_FIFO_BUFFER_COUNT,
                                                  ARSTREAM2_STREAM_RECEIVER_AU_BUFFER_SIZE,
                                                  ARSTREAM2_STREAM_RECEIVER_AU_METADATA_BUFFER_SIZE,
                                                  ARSTREAM2_STREAM_RECEIVER_AU_USER_DATA_BUFFER_SIZE,
                                                  sizeof(ARSTREAM2_H264_VideoStats_t));
        if (auFifoRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264_AuFifoInit() failed (%d)", auFifoRet);
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            auFifoCreated = 1;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        ARSTREAM2_RtpReceiver_Config_t receiverConfig;
        ARSTREAM2_RtpReceiver_NetConfig_t receiver_net_config;
        ARSTREAM2_RtpReceiver_MuxConfig_t receiver_mux_config;
        memset(&receiverConfig, 0, sizeof(receiverConfig));
        memset(&receiver_net_config, 0, sizeof(receiver_net_config));
        memset(&receiver_mux_config, 0, sizeof(receiver_mux_config));


        receiverConfig.canonicalName = config->canonicalName;
        receiverConfig.friendlyName = config->friendlyName;
        receiverConfig.applicationName = config->applicationName;
        receiverConfig.packetFifo = &(streamReceiver->packetFifo);
        receiverConfig.packetFifoQueue = &(streamReceiver->packetFifoQueue);
        receiverConfig.auFifo = &(streamReceiver->auFifo);
        receiverConfig.auCallback = ARSTREAM2_StreamReceiver_RtpReceiverAuCallback;
        receiverConfig.auCallbackUserPtr = streamReceiver;
        receiverConfig.rtpStatsCallback = ARSTREAM2_StreamReceiver_RtpReceiverStatsCallback;
        receiverConfig.rtpStatsCallbackUserPtr = streamReceiver;
        receiverConfig.maxPacketSize = config->maxPacketSize;
        receiverConfig.insertStartCodes = 1;
        receiverConfig.generateReceiverReports = config->generateReceiverReports;
        receiverConfig.videoStatsSendTimeInterval = ARSTREAM2_STREAM_RECEIVER_VIDEO_STATS_RTCP_SEND_INTERVAL;
        receiverConfig.lossReportSendTimeInterval = ARSTREAM2_STREAM_RECEIVER_LOSS_REPORT_RTCP_SEND_INTERVAL;
        receiverConfig.djbReportSendTimeInterval = ARSTREAM2_STREAM_RECEIVER_DJB_REPORT_RTCP_SEND_INTERVAL;

        if (usemux) {
            receiver_mux_config.mux = mux_config->mux;
            streamReceiver->receiver = ARSTREAM2_RtpReceiver_New(&receiverConfig, NULL, &receiver_mux_config, &ret);
        } else {
            receiver_net_config.serverAddr = net_config->serverAddr;
            receiver_net_config.mcastAddr = net_config->mcastAddr;
            receiver_net_config.mcastIfaceAddr = net_config->mcastIfaceAddr;
            receiver_net_config.serverStreamPort = net_config->serverStreamPort;
            receiver_net_config.serverControlPort = net_config->serverControlPort;
            receiver_net_config.clientStreamPort = net_config->clientStreamPort;
            receiver_net_config.clientControlPort = net_config->clientControlPort;
            receiver_net_config.classSelector = net_config->classSelector;
            streamReceiver->receiver = ARSTREAM2_RtpReceiver_New(&receiverConfig, &receiver_net_config, NULL, &ret);
        }

        if (ret != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Error while creating receiver : %s", ARSTREAM2_Error_ToString(ret));
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        ARSTREAM2_H264Filter_Config_t filterConfig;
        memset(&filterConfig, 0, sizeof(filterConfig));
        filterConfig.spsPpsCallback = ARSTREAM2_StreamReceiver_H264FilterSpsPpsCallback;
        filterConfig.spsPpsCallbackUserPtr = streamReceiver;
        filterConfig.outputIncompleteAu = config->outputIncompleteAu;
        filterConfig.generateSkippedPSlices = config->generateSkippedPSlices;

        ret = ARSTREAM2_H264Filter_Init(&streamReceiver->filter, &filterConfig);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Error while creating H264Filter: %s", ARSTREAM2_Error_ToString(ret));
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        *streamReceiverHandle = streamReceiver;
    }
    else
    {
        if (streamReceiver)
        {
            int err;
            if (streamReceiver->receiver) ARSTREAM2_RtpReceiver_Delete(&(streamReceiver->receiver));
            if (streamReceiver->filter) ARSTREAM2_H264Filter_Free(&(streamReceiver->filter));
            if (packetFifoWasCreated) ARSTREAM2_RTP_PacketFifoFree(&(streamReceiver->packetFifo));
            if (auFifoCreated) ARSTREAM2_H264_AuFifoFree(&(streamReceiver->auFifo));
            if (streamReceiver->signalPipe[0] != -1)
            {
                while (((err = close(streamReceiver->signalPipe[0])) == -1) && (errno == EINTR));
                streamReceiver->signalPipe[0] = -1;
            }
            if (streamReceiver->signalPipe[1] != -1)
            {
                while (((err = close(streamReceiver->signalPipe[1])) == -1) && (errno == EINTR));
                streamReceiver->signalPipe[1] = -1;
            }
            if (threadMutexInit) ARSAL_Mutex_Destroy(&(streamReceiver->threadMutex));
            if (resendMutexInit) ARSAL_Mutex_Destroy(&(streamReceiver->resendMutex));
            if (appOutputThreadMutexInit) ARSAL_Mutex_Destroy(&(streamReceiver->appOutput.threadMutex));
            if (appOutputThreadCondInit) ARSAL_Cond_Destroy(&(streamReceiver->appOutput.threadCond));
            if (appOutputCallbackMutexInit) ARSAL_Mutex_Destroy(&(streamReceiver->appOutput.callbackMutex));
            if (appOutputCallbackCondInit) ARSAL_Cond_Destroy(&(streamReceiver->appOutput.callbackCond));
            if (recorderThreadMutexInit) ARSAL_Mutex_Destroy(&(streamReceiver->recorder.threadMutex));
            if (recorderThreadCondInit) ARSAL_Cond_Destroy(&(streamReceiver->recorder.threadCond));
            ARSTREAM2_StreamStats_VideoStatsFileClose(&streamReceiver->videoStatsCtx);
            ARSTREAM2_StreamStats_RtpStatsFileClose(&streamReceiver->rtpStatsCtx);
            ARSTREAM2_StreamStats_RtpLossFileClose(&streamReceiver->rtpLossCtx);
            free(streamReceiver->debugPath);
            free(streamReceiver->friendlyName);
            free(streamReceiver->dateAndTime);
            free(streamReceiver->appOutput.videoStats.erroredSecondCountByZone);
            free(streamReceiver->appOutput.videoStats.macroblockStatus);
            free(streamReceiver);
        }
        *streamReceiverHandle = NULL;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_Free(ARSTREAM2_StreamReceiver_Handle *streamReceiverHandle)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    int err;

    if ((!streamReceiverHandle) || (!*streamReceiverHandle))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid pointer for handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    streamReceiver = (ARSTREAM2_StreamReceiver_t*)*streamReceiverHandle;

    if (streamReceiver->appOutput.threadRunning == 1)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Call ARSTREAM2_StreamReceiver_StopAppOutput() before calling this function");
        return ARSTREAM2_ERROR_BUSY;
    }

    ARSAL_Mutex_Lock(&(streamReceiver->threadMutex));
    if (streamReceiver->threadStarted == 1)
    {
        ARSAL_Mutex_Unlock(&(streamReceiver->threadMutex));
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Call ARSTREAM2_StreamReceiver_Stop() before calling this function");
        return ARSTREAM2_ERROR_BUSY;
    }
    ARSAL_Mutex_Unlock(&(streamReceiver->threadMutex));


    int recErr = ARSTREAM2_StreamReceiver_StreamRecorderFree(streamReceiver);
    if (recErr != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamReceiver_StreamRecorderFree() failed (%d)", recErr);
    }

    ARSTREAM2_RtpResender_t *resender, *next;
    for (resender = streamReceiver->resender; resender; resender = next)
    {
        ret = ARSTREAM2_RtpSender_Delete(&resender->sender);
        if (ret != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Unable to delete sender: %s", ARSTREAM2_Error_ToString(ret));
        }
        next = resender->next;
        free(resender);
    }
    free(streamReceiver->resendQueue);
    free(streamReceiver->resendTimeout);

    ret = ARSTREAM2_RtpReceiver_Delete(&streamReceiver->receiver);
    if (ret != ARSTREAM2_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Unable to delete receiver: %s", ARSTREAM2_Error_ToString(ret));
    }

    ret = ARSTREAM2_H264Filter_Free(&streamReceiver->filter);
    if (ret != ARSTREAM2_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Unable to delete H264Filter: %s", ARSTREAM2_Error_ToString(ret));
    }

    ARSTREAM2_RTP_PacketFifoFree(&(streamReceiver->packetFifo));
    ARSTREAM2_H264_AuFifoFree(&(streamReceiver->auFifo));
    ARSAL_Mutex_Destroy(&(streamReceiver->threadMutex));
    ARSAL_Mutex_Destroy(&(streamReceiver->resendMutex));
    ARSAL_Mutex_Destroy(&(streamReceiver->appOutput.threadMutex));
    ARSAL_Cond_Destroy(&(streamReceiver->appOutput.threadCond));
    ARSAL_Mutex_Destroy(&(streamReceiver->appOutput.callbackMutex));
    ARSAL_Cond_Destroy(&(streamReceiver->appOutput.callbackCond));
    ARSAL_Mutex_Destroy(&(streamReceiver->recorder.threadMutex));
    ARSAL_Cond_Destroy(&(streamReceiver->recorder.threadCond));
    if (streamReceiver->signalPipe[0] != -1)
    {
        while (((err = close(streamReceiver->signalPipe[0])) == -1) && (errno == EINTR));
        streamReceiver->signalPipe[0] = -1;
    }
    if (streamReceiver->signalPipe[1] != -1)
    {
        while (((err = close(streamReceiver->signalPipe[1])) == -1) && (errno == EINTR));
        streamReceiver->signalPipe[1] = -1;
    }
    free(streamReceiver->recorder.fileName);
    free(streamReceiver->pSps);
    free(streamReceiver->pPps);
    ARSTREAM2_StreamStats_VideoStatsFileClose(&streamReceiver->videoStatsCtx);
    ARSTREAM2_StreamStats_RtpStatsFileClose(&streamReceiver->rtpStatsCtx);
    ARSTREAM2_StreamStats_RtpLossFileClose(&streamReceiver->rtpLossCtx);
    free(streamReceiver->debugPath);
    free(streamReceiver->friendlyName);
    free(streamReceiver->dateAndTime);
    free(streamReceiver->appOutput.videoStats.erroredSecondCountByZone);
    free(streamReceiver->appOutput.videoStats.macroblockStatus);

    free(streamReceiver);
    *streamReceiverHandle = NULL;

    return ret;
}


static int ARSTREAM2_StreamReceiver_AppOutputAuEnqueue(ARSTREAM2_StreamReceiver_t *streamReceiver, ARSTREAM2_H264_AuFifoItem_t *auItem)
{
    int err = 0, ret = 0, needUnref = 0, needFree = 0;
    ARSTREAM2_H264_AuFifoItem_t *appOutputAuItem = NULL;

    /* add ref to AU buffer */
    ret = ARSTREAM2_H264_AuFifoBufferAddRef(&streamReceiver->auFifo, auItem->au.buffer);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264_AuFifoBufferAddRef() failed (%d)", ret);
    }
    if (ret == 0)
    {
        /* duplicate the AU and associated NALUs */
        appOutputAuItem = ARSTREAM2_H264_AuFifoDuplicateItem(&streamReceiver->auFifo, auItem);
        if (appOutputAuItem)
        {
            appOutputAuItem->au.buffer = auItem->au.buffer;
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to pop free item from the AU FIFO");
            ret = -1;
            needUnref = 1;
        }
    }

    if ((ret == 0) && (appOutputAuItem))
    {
        /* enqueue the AU */
        ret = ARSTREAM2_H264_AuFifoEnqueueItem(&streamReceiver->appOutput.auFifoQueue, appOutputAuItem);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264_AuFifoEnqueueItem() failed (%d)", ret);
            err = -1;
            needUnref = 1;
            needFree = 1;
        }
        else
        {
            ARSAL_Cond_Signal(&(streamReceiver->appOutput.threadCond));
        }
    }
    else
    {
        err = -1;
    }

    /* error handling */
    if (needFree)
    {
        ret = ARSTREAM2_H264_AuFifoPushFreeItem(&streamReceiver->auFifo, appOutputAuItem);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to push free item in the AU FIFO (%d)", ret);
        }
        needFree = 0;
    }
    if (needUnref)
    {
        ret = ARSTREAM2_H264_AuFifoUnrefBuffer(&streamReceiver->auFifo, auItem->au.buffer);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to unref buffer (%d)", ret);
        }
        needUnref = 0;
    }

    return err;
}


static int ARSTREAM2_StreamReceiver_RecorderAuEnqueue(ARSTREAM2_StreamReceiver_t *streamReceiver, ARSTREAM2_H264_AuFifoItem_t *auItem)
{
    int err = 0, ret = 0, needUnref = 0, needFree = 0;
    ARSTREAM2_H264_AuFifoItem_t *recordtAuItem = NULL;

    ret = ARSTREAM2_H264_AuFifoBufferAddRef(&streamReceiver->auFifo, auItem->au.buffer);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264_AuFifoBufferAddRef() failed (%d)", ret);
    }
    if (ret == 0)
    {
        /* duplicate the AU and associated NALUs */
        recordtAuItem = ARSTREAM2_H264_AuFifoDuplicateItem(&streamReceiver->auFifo, auItem);
        if (recordtAuItem)
        {
            recordtAuItem->au.buffer = auItem->au.buffer;
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to pop free item from the AU FIFO");
            ret = -1;
            needUnref = 1;
        }
    }

    if ((ret == 0) && (recordtAuItem))
    {
        /* enqueue the AU */
        ret = ARSTREAM2_H264_AuFifoEnqueueItem(&streamReceiver->recorder.auFifoQueue, recordtAuItem);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264_AuFifoEnqueueItem() failed (%d)", ret);
            err = -1;
            needUnref = 1;
            needFree = 1;
        }
        else
        {
            ARSAL_Cond_Signal(&(streamReceiver->recorder.threadCond));
        }
    }
    else
    {
        err = -1;
    }

    /* error handling */
    if (needFree)
    {
        ret = ARSTREAM2_H264_AuFifoPushFreeItem(&streamReceiver->auFifo, recordtAuItem);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to push free item in the AU FIFO (%d)", ret);
        }
        needFree = 0;
    }
    if (needUnref)
    {
        ret = ARSTREAM2_H264_AuFifoUnrefBuffer(&streamReceiver->auFifo, auItem->au.buffer);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to unref buffer (%d)", ret);
        }
        needUnref = 0;
    }

    return err;
}


static int ARSTREAM2_StreamReceiver_GenerateGrayIdrFrame(ARSTREAM2_StreamReceiver_t *streamReceiver, ARSTREAM2_H264_AccessUnit_t *nextAu)
{
    int err = 0, ret;

    ARSTREAM2_H264_AuFifoBuffer_t *auBuffer = ARSTREAM2_H264_AuFifoGetBuffer(&streamReceiver->auFifo);
    ARSTREAM2_H264_AuFifoItem_t *auItem = ARSTREAM2_H264_AuFifoPopFreeItem(&streamReceiver->auFifo);

    if ((!auBuffer) || (!auItem))
    {
        if (auBuffer)
        {
            ret = ARSTREAM2_H264_AuFifoUnrefBuffer(&streamReceiver->auFifo, auBuffer);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to unref buffer (%d)", ret);
            }
            auBuffer = NULL;
        }
        if (auItem)
        {
            ret = ARSTREAM2_H264_AuFifoPushFreeItem(&streamReceiver->auFifo, auItem);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to push free item in the AU FIFO (%d)", ret);
            }
            auItem = NULL;
        }
        ret = ARSTREAM2_H264_AuFifoFlush(&streamReceiver->auFifo);
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "AU FIFO is full, cannot generate gray I-frame => flush to recover (%d AU flushed)", ret);
    }
    else
    {
        ARSTREAM2_H264_AuReset(&auItem->au);
        auItem->au.buffer = auBuffer;

        ret = ARSTREAM2_H264FilterError_GenerateGrayIdrFrame(streamReceiver->filter, nextAu, auItem);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264FilterError_GenerateGrayIdrFrame() failed (%d)", ret);

            if (auBuffer)
            {
                ret = ARSTREAM2_H264_AuFifoUnrefBuffer(&streamReceiver->auFifo, auBuffer);
                if (ret != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to unref buffer (%d)", ret);
                }
                auBuffer = NULL;
            }
            if (auItem)
            {
                ret = ARSTREAM2_H264_AuFifoPushFreeItem(&streamReceiver->auFifo, auItem);
                if (ret != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to push free item in the AU FIFO (%d)", ret);
                }
                auItem = NULL;
            }
        }
    }

    if (auItem)
    {
        /* application output */
        ARSAL_Mutex_Lock(&(streamReceiver->appOutput.threadMutex));
        int appOutputRunning = streamReceiver->appOutput.running;
        ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.threadMutex));
        if ((appOutputRunning) && (streamReceiver->appOutput.grayIFramePending))
        {
            ret = ARSTREAM2_StreamReceiver_AppOutputAuEnqueue(streamReceiver, auItem);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamReceiver_AppOutputAuEnqueue() failed (%d)", ret);
            }
            else
            {
                streamReceiver->appOutput.grayIFramePending = 0;
            }
        }

        /* stream recording */
        ARSAL_Mutex_Lock(&(streamReceiver->recorder.threadMutex));
        int recorderRunning = streamReceiver->recorder.running;
        ARSAL_Mutex_Unlock(&(streamReceiver->recorder.threadMutex));
        if ((recorderRunning) && (streamReceiver->recorder.grayIFramePending))
        {
            ret = ARSTREAM2_StreamReceiver_RecorderAuEnqueue(streamReceiver, auItem);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamReceiver_RecorderAuEnqueue() failed (%d)", ret);
            }
            else
            {
                streamReceiver->recorder.grayIFramePending = 0;
            }
        }

        /* free the access unit */
        ret = ARSTREAM2_H264_AuFifoUnrefBuffer(&streamReceiver->auFifo, auItem->au.buffer);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to unref buffer (%d)", ret);
        }
        ret = ARSTREAM2_H264_AuFifoPushFreeItem(&streamReceiver->auFifo, auItem);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to push free item in the AU FIFO (%d)", ret);
        }
    }

    return err;
}


static int ARSTREAM2_StreamReceiver_RtpReceiverAuCallback(ARSTREAM2_H264_AuFifoItem_t *auItem, void *userPtr)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)userPtr;
    int err = 0, ret;

    if ((!auItem) || (!userPtr))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid pointer");
        return -1;
    }

    ret = ARSTREAM2_H264Filter_ProcessAu(streamReceiver->filter, &auItem->au);
    if (ret == 1)
    {
        /* gray IDR frame generation */
        if ((streamReceiver->appOutput.grayIFramePending) || (streamReceiver->recorder.grayIFramePending))
        {
            if (auItem->au.syncType == ARSTREAM2_H264_AU_SYNC_TYPE_IDR)
            {
                streamReceiver->appOutput.grayIFramePending = 0;
                streamReceiver->recorder.grayIFramePending = 0;
            }
            else if (auItem->au.syncType != ARSTREAM2_H264_AU_SYNC_TYPE_NONE)
            {
                ret = ARSTREAM2_StreamReceiver_GenerateGrayIdrFrame(streamReceiver, &auItem->au);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamReceiver_GenerateGrayIdrFrame() failed (%d)", ret);
                }
            }
        }

        if (auItem->au.videoStatsAvailable)
        {
            ARSTREAM2_H264_VideoStats_t *vs = (ARSTREAM2_H264_VideoStats_t*)auItem->au.buffer->videoStatsBuffer;
            uint32_t ntpTimestampDelta = ((auItem->au.ntpTimestamp) && (streamReceiver->lastAuNtpTimestamp))
                    ? (uint32_t)(auItem->au.ntpTimestamp - streamReceiver->lastAuNtpTimestamp) : 0;
            uint32_t ntpTimestampRawDelta = ((auItem->au.ntpTimestampRaw) && (streamReceiver->lastAuNtpTimestampRaw))
                    ? (uint32_t)(auItem->au.ntpTimestampRaw - streamReceiver->lastAuNtpTimestampRaw) : 0;
            vs->timestampDelta = ((auItem->au.ntpTimestamp) && (streamReceiver->lastAuNtpTimestamp))
                    ? ntpTimestampDelta : ((auItem->au.ntpTimestampRaw) && (streamReceiver->lastAuNtpTimestampRaw))
                        ? ntpTimestampRawDelta : 0;
            streamReceiver->timestampDeltaIntegral += vs->timestampDelta;
            vs->timestampDeltaIntegral = streamReceiver->timestampDeltaIntegral;
            streamReceiver->timestampDeltaIntegralSq += (uint64_t)vs->timestampDelta * (uint64_t)vs->timestampDelta;
            vs->timestampDeltaIntegralSq = streamReceiver->timestampDeltaIntegralSq;
        }
        streamReceiver->lastAuNtpTimestamp = auItem->au.ntpTimestamp;
        streamReceiver->lastAuNtpTimestampRaw = auItem->au.ntpTimestampRaw;

        /* application output */
        ARSAL_Mutex_Lock(&(streamReceiver->appOutput.threadMutex));
        int appOutputRunning = streamReceiver->appOutput.running;
        ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.threadMutex));
        if ((appOutputRunning) && ((!streamReceiver->appOutput.grayIFramePending) || (auItem->au.syncType == ARSTREAM2_H264_AU_SYNC_TYPE_IDR)))
        {
            ret = ARSTREAM2_StreamReceiver_AppOutputAuEnqueue(streamReceiver, auItem);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamReceiver_AppOutputAuEnqueue() failed (%d)", ret);
            }
        }
        else if (auItem->au.videoStatsAvailable)
        {
            struct timespec t1;
            ARSAL_Time_GetTime(&t1);
            uint64_t curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
            ARSTREAM2_H264_VideoStats_t *vs = (ARSTREAM2_H264_VideoStats_t*)auItem->au.buffer->videoStatsBuffer;
            uint32_t outputTimestampDelta = (streamReceiver->lastAuOutputTimestamp)
                    ? (uint32_t)(curTime - streamReceiver->lastAuOutputTimestamp) : 0;
            uint32_t estimatedLatency = ((auItem->au.ntpTimestampLocal) && (curTime > auItem->au.ntpTimestampLocal))
                    ? (uint32_t)(curTime - auItem->au.ntpTimestampLocal) : 0;
            int32_t timingError = ((vs->timestampDelta) && (streamReceiver->lastAuOutputTimestamp))
                    ? ((int32_t)vs->timestampDelta - (int32_t)outputTimestampDelta) : 0;
            vs->timingError = timingError;
            streamReceiver->timingErrorIntegral += (timingError < 0) ? (uint32_t)(-timingError) : (uint32_t)timingError;
            vs->timingErrorIntegral = streamReceiver->timingErrorIntegral;
            streamReceiver->timingErrorIntegralSq += (int64_t)timingError * (int64_t)timingError;
            vs->timingErrorIntegralSq = streamReceiver->timingErrorIntegralSq;
            vs->estimatedLatency = estimatedLatency;
            streamReceiver->estimatedLatencyIntegral += estimatedLatency;
            vs->estimatedLatencyIntegral = streamReceiver->estimatedLatencyIntegral;
            streamReceiver->estimatedLatencyIntegralSq += (uint64_t)estimatedLatency * (uint64_t)estimatedLatency;
            vs->estimatedLatencyIntegralSq = streamReceiver->estimatedLatencyIntegralSq;
            streamReceiver->lastAuOutputTimestamp = curTime;
            vs->timestamp = auItem->au.ntpTimestampRaw;

            /* get the RSSI from the streaming metadata */
            //TODO: remove this hack once we have a better way of getting the RSSI
            if ((auItem->au.metadataSize >= 27) && (ntohs(*((uint16_t*)auItem->au.buffer->metadataBuffer)) == 0x5031))
            {
                vs->rssi = (int8_t)auItem->au.buffer->metadataBuffer[26];
            }
            if ((auItem->au.metadataSize >= 55) && (ntohs(*((uint16_t*)auItem->au.buffer->metadataBuffer)) == 0x5032))
            {
                vs->rssi = (int8_t)auItem->au.buffer->metadataBuffer[54];
            }
            streamReceiver->lastKnownRssi = vs->rssi;

            eARSTREAM2_ERROR recvErr = ARSTREAM2_RtpReceiver_UpdateVideoStats(streamReceiver->receiver, vs);
            if (recvErr != ARSTREAM2_OK)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpReceiver_UpdateVideoStats() failed (%d)", recvErr);
            }
            ARSTREAM2_StreamStats_VideoStatsFileWrite(&streamReceiver->videoStatsCtx, vs);
        }

        /* stream recording */
        ARSAL_Mutex_Lock(&(streamReceiver->recorder.threadMutex));
        int recorderRunning = streamReceiver->recorder.running;
        ARSAL_Mutex_Unlock(&(streamReceiver->recorder.threadMutex));
        if ((recorderRunning) && ((!streamReceiver->recorder.grayIFramePending) || (auItem->au.syncType == ARSTREAM2_H264_AU_SYNC_TYPE_IDR)))
        {
            ret = ARSTREAM2_StreamReceiver_RecorderAuEnqueue(streamReceiver, auItem);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamReceiver_RecorderAuEnqueue() failed (%d)", ret);
            }
        }
    }
    else
    {
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264Filter_ProcessAu() failed (%d)", ret);
        }
    }

    /* free the access unit */
    ret = ARSTREAM2_H264_AuFifoUnrefBuffer(&streamReceiver->auFifo, auItem->au.buffer);
    if (ret != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to unref buffer (%d)", ret);
    }
    ret = ARSTREAM2_H264_AuFifoPushFreeItem(&streamReceiver->auFifo, auItem);
    if (ret != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to push free item in the AU FIFO (%d)", ret);
    }

    return err;
}


static void ARSTREAM2_StreamReceiver_RtpReceiverStatsCallback(const ARSTREAM2_RTP_RtpStats_t *rtpStats, void *userPtr)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)userPtr;

    if (!userPtr)
    {
        return;
    }

    if (rtpStats)
    {
        ARSTREAM2_RTP_RtpStats_t s;
        struct timespec t1;
        uint64_t curTime;

        ARSAL_Time_GetTime(&t1);
        curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

        memcpy(&s, rtpStats, sizeof(ARSTREAM2_RTP_RtpStats_t));
        s.rssi = streamReceiver->lastKnownRssi;
        ARSTREAM2_StreamStats_RtpStatsFileWrite(&streamReceiver->rtpStatsCtx, &s, curTime);
        ARSTREAM2_StreamStats_RtpLossFileWrite(&streamReceiver->rtpLossCtx, &s);
    }
}


static int ARSTREAM2_StreamReceiver_H264FilterSpsPpsCallback(uint8_t *spsBuffer, int spsSize, uint8_t *ppsBuffer, int ppsSize, void *userPtr)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)userPtr;
    int ret = 0;
    eARSTREAM2_ERROR cbRet;

    if (!userPtr)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid pointer");
        return -1;
    }

    /* sync */
    if ((spsSize > 0) && (ppsSize > 0))
    {
        streamReceiver->pSps = realloc(streamReceiver->pSps, spsSize);
        if (!streamReceiver->pSps)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Allocation failed");
            return -1;
        }
        streamReceiver->pPps = realloc(streamReceiver->pPps, ppsSize);
        if (!streamReceiver->pPps)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Allocation failed");
            return -1;
        }
        memcpy(streamReceiver->pSps, spsBuffer, spsSize);
        streamReceiver->spsSize = spsSize;
        memcpy(streamReceiver->pPps, ppsBuffer, ppsSize);
        streamReceiver->ppsSize = ppsSize;
        streamReceiver->sync = 1;
    }

    /* stream recording */
    if (streamReceiver->recorder.startPending)
    {
        int recRet;
        recRet = ARSTREAM2_StreamReceiver_StreamRecorderInit(streamReceiver);
        if (recRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264Filter_StreamRecorderInit() failed (%d)", recRet);
        }
        streamReceiver->recorder.startPending = 0;
    }

    /* call the app output SPS/PPS callback if app output is started */
    ARSAL_Mutex_Lock(&(streamReceiver->appOutput.callbackMutex));
    streamReceiver->appOutput.callbackInProgress = 1;
    if (streamReceiver->appOutput.spsPpsCallback)
    {
        ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.callbackMutex));

        cbRet = streamReceiver->appOutput.spsPpsCallback(streamReceiver->pSps, streamReceiver->spsSize,
                                                         streamReceiver->pPps, streamReceiver->ppsSize,
                                                         streamReceiver->appOutput.spsPpsCallbackUserPtr);
        if (cbRet != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_STREAM_RECEIVER_TAG, "Application SPS/PPS callback failed");
        }

        ARSAL_Mutex_Lock(&(streamReceiver->appOutput.callbackMutex));
    }
    streamReceiver->appOutput.callbackInProgress = 0;
    ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.callbackMutex));
    ARSAL_Cond_Signal(&(streamReceiver->appOutput.callbackCond));

    return ret;
}


static int ARSTREAM2_StreamReceiver_StreamRecorderInit(ARSTREAM2_StreamReceiver_t *streamReceiver)
{
    int ret = -1;

    if (!streamReceiver->sync)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "No sync");
        return -1;
    }

    if ((!streamReceiver->recorder.recorder) && (streamReceiver->recorder.fileName))
    {
        eARSTREAM2_ERROR recErr;
        int width = 0, height = 0;
        float framerate = 0.0;
        ARSTREAM2_StreamRecorder_Config_t recConfig;
        memset(&recConfig, 0, sizeof(ARSTREAM2_StreamRecorder_Config_t));
        recConfig.mediaFileName = streamReceiver->recorder.fileName;
        int err = ARSTREAM2_H264Filter_GetVideoParams(streamReceiver->filter, NULL, NULL, &width, &height, &framerate);
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264Filter_GetVideoParams() failed (%d)",err);
        }
        recConfig.videoFramerate = framerate;
        recConfig.videoWidth = (uint32_t)width;
        recConfig.videoHeight = (uint32_t)height;
        recConfig.sps = streamReceiver->pSps;
        recConfig.spsSize = streamReceiver->spsSize;
        recConfig.pps = streamReceiver->pPps;
        recConfig.ppsSize = streamReceiver->ppsSize;
        recConfig.ardiscoveryProductType = streamReceiver->recorder.ardiscoveryProductType;
        recConfig.auFifo = &streamReceiver->auFifo;
        recConfig.auFifoQueue = &streamReceiver->recorder.auFifoQueue;
        recConfig.mutex = &streamReceiver->recorder.threadMutex;
        recConfig.cond = &streamReceiver->recorder.threadCond;
        recErr = ARSTREAM2_StreamRecorder_Init(&streamReceiver->recorder.recorder, &recConfig);
        if (recErr != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamRecorder_Init() failed (%d): %s",
                        recErr, ARSTREAM2_Error_ToString(recErr));
        }
        else
        {
            time(&streamReceiver->recorder.startTime);
            int thErr = ARSAL_Thread_Create(&streamReceiver->recorder.thread, ARSTREAM2_StreamRecorder_RunThread, (void*)streamReceiver->recorder.recorder);
            if (thErr != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Recorder thread creation failed (%d)", thErr);
            }
            else
            {
                ret = 0;
                int auFifoRet = ARSTREAM2_H264_AuFifoAddQueue(&streamReceiver->auFifo, &streamReceiver->recorder.auFifoQueue);
                if (auFifoRet != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264_AuFifoAddQueue() failed (%d)", auFifoRet);
                }
                else
                {
                    ARSAL_Mutex_Lock(&(streamReceiver->recorder.threadMutex));
                    streamReceiver->recorder.grayIFramePending = streamReceiver->recorder.generateGrayIFrame;
                    streamReceiver->recorder.running = 1;
                    ARSAL_Mutex_Unlock(&(streamReceiver->recorder.threadMutex));
                }
            }
        }
    }

    return ret;
}


static int ARSTREAM2_StreamReceiver_StreamRecorderStop(ARSTREAM2_StreamReceiver_t *streamReceiver)
{
    int ret = 0;

    if (streamReceiver->recorder.recorder)
    {
        eARSTREAM2_ERROR err;

        /* Set the untimed metadata */
        ARSTREAM2_Stream_UntimedMetadata_t metadata;
        memset(&metadata, 0, sizeof(ARSTREAM2_Stream_UntimedMetadata_t));
        err = ARSTREAM2_StreamReceiver_GetPeerUntimedMetadata((ARSTREAM2_StreamReceiver_Handle)streamReceiver, &metadata);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamReceiver_GetPeerUntimedMetadata() failed: %d (%s)",
                        err, ARSTREAM2_Error_ToString(err));
        }
        else
        {
            /* Date format : <YYYY-MM-DDTHHMMSS+HHMM */
            #define DATE_SIZE 23
            #define DATE_FORMAT "%FT%H%M%S%z"
            char mediaDate[DATE_SIZE];
            struct tm timeInfo;
            localtime_r(&streamReceiver->recorder.startTime, &timeInfo);
            strftime(mediaDate, DATE_SIZE, DATE_FORMAT, &timeInfo);

            ARSTREAM2_StreamRecorder_UntimedMetadata_t meta;
            memset(&meta, 0, sizeof(ARSTREAM2_StreamRecorder_UntimedMetadata_t));
            meta.maker = metadata.maker;
            meta.model = metadata.model;
            meta.modelId = metadata.modelId;
            meta.serialNumber = metadata.serialNumber;
            meta.softwareVersion = metadata.softwareVersion;
            meta.buildId = metadata.buildId;
            meta.artist = metadata.friendlyName;
            meta.title = metadata.title;
            meta.comment = metadata.comment;
            meta.copyright = metadata.copyright;
            meta.mediaDate = mediaDate;
            meta.runDate = metadata.runDate;
            meta.runUuid = metadata.runUuid;
            meta.takeoffLatitude = metadata.takeoffLatitude;
            meta.takeoffLongitude = metadata.takeoffLongitude;
            meta.takeoffAltitude = metadata.takeoffAltitude;
            meta.pictureHFov = metadata.pictureHFov;
            meta.pictureVFov = metadata.pictureVFov;
            err = ARSTREAM2_StreamRecorder_SetUntimedMetadata(streamReceiver->recorder.recorder, &meta);
            if (err != ARSTREAM2_OK)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamRecorder_SetUntimedMetadata() failed: %d (%s)",
                            err, ARSTREAM2_Error_ToString(err));
            }
        }

        /* Stop the recorder */
        err = ARSTREAM2_StreamRecorder_Stop(streamReceiver->recorder.recorder);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamRecorder_Stop() failed: %d (%s)",
                        err, ARSTREAM2_Error_ToString(err));
            ret = -1;
        }
        else
        {
            ARSAL_Mutex_Lock(&(streamReceiver->recorder.threadMutex));
            streamReceiver->recorder.running = 0;
            ARSAL_Mutex_Unlock(&(streamReceiver->recorder.threadMutex));
        }
    }

    return ret;
}


static int ARSTREAM2_StreamReceiver_StreamRecorderFree(ARSTREAM2_StreamReceiver_t *streamReceiver)
{
    int ret = 0;

    if (streamReceiver->recorder.recorder)
    {
        int thErr;
        eARSTREAM2_ERROR err;
        ARSAL_Mutex_Lock(&(streamReceiver->recorder.threadMutex));
        int recRunning = streamReceiver->recorder.running;
        ARSAL_Mutex_Unlock(&(streamReceiver->recorder.threadMutex));
        if (recRunning)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Stream recorder is not stopped, cannot free");
            return -1;
        }
        if (streamReceiver->recorder.thread)
        {
            thErr = ARSAL_Thread_Join(streamReceiver->recorder.thread, NULL);
            if (thErr != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSAL_Thread_Join() failed (%d)", thErr);
                ret = -1;
            }
            thErr = ARSAL_Thread_Destroy(&streamReceiver->recorder.thread);
            if (thErr != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSAL_Thread_Destroy() failed (%d)", thErr);
                ret = -1;
            }
            streamReceiver->recorder.thread = NULL;
        }
        err = ARSTREAM2_StreamRecorder_Free(&streamReceiver->recorder.recorder);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamRecorder_Free() failed (%d): %s",
                        err, ARSTREAM2_Error_ToString(err));
            ret = -1;
        }

        int auFifoRet = ARSTREAM2_H264_AuFifoRemoveQueue(&streamReceiver->auFifo, &streamReceiver->recorder.auFifoQueue);
        if (auFifoRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264_AuFifoRemoveQueue() failed (%d)", auFifoRet);
            ret = -1;
        }
    }

    return ret;
}


void* ARSTREAM2_StreamReceiver_RunAppOutputThread(void *streamReceiverHandle)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    int shouldStop, running, ret;
    struct timespec t1;
    uint64_t curTime;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return NULL;
    }

    streamReceiver->appOutput.threadRunning = 1;
    ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_STREAM_RECEIVER_TAG, "App output thread running");

    ARSAL_Mutex_Lock(&(streamReceiver->appOutput.threadMutex));
    shouldStop = streamReceiver->appOutput.threadShouldStop;
    running = streamReceiver->appOutput.running;
    ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.threadMutex));

    while (shouldStop == 0)
    {
        ARSTREAM2_H264_AuFifoItem_t *auItem = NULL;

        ARSAL_Time_GetTime(&t1);
        curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

        if (running)
        {
            /* dequeue an access unit */
            auItem = ARSTREAM2_H264_AuFifoDequeueItem(&streamReceiver->appOutput.auFifoQueue);
        }

        while (auItem != NULL)
        {
            ARSTREAM2_H264_AccessUnit_t *au = &auItem->au;
            ARSTREAM2_H264_NaluFifoItem_t *naluItem;
            unsigned int auSize = 0;

            if ((streamReceiver->appOutput.mbWidth == 0) || (streamReceiver->appOutput.mbHeight == 0))
            {
                int mbWidth = 0, mbHeight = 0;
                int err = ARSTREAM2_H264Filter_GetVideoParams(streamReceiver->filter, &mbWidth, &mbHeight, NULL, NULL, NULL);
                if (err != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264Filter_GetVideoParams() failed (%d)",err);
                }
                streamReceiver->appOutput.mbWidth = mbWidth;
                streamReceiver->appOutput.mbHeight = mbHeight;
            }

            /* pre-check the access unit size to avoid calling getAuBufferCallback+auReadyCallback for null sized frames */
            for (naluItem = au->naluHead; naluItem; naluItem = naluItem->next)
            {
                /* filter out unwanted NAL units */
                if ((streamReceiver->appOutput.filterOutSpsPps) && ((naluItem->nalu.naluType == ARSTREAM2_H264_NALU_TYPE_SPS) || (naluItem->nalu.naluType == ARSTREAM2_H264_NALU_TYPE_PPS)))
                {
                    continue;
                }
                if ((streamReceiver->appOutput.filterOutSei) && (naluItem->nalu.naluType == ARSTREAM2_H264_NALU_TYPE_SEI))
                {
                    continue;
                }

                auSize += naluItem->nalu.naluSize;
            }

            if ((running) && (auSize > 0))
            {
                eARSTREAM2_ERROR cbRet = ARSTREAM2_OK;
                uint8_t *auBuffer = NULL;
                int auBufferSize = 0;
                void *auBufferUserPtr = NULL;
                ARSTREAM2_StreamReceiver_AuReadyCallbackTimestamps_t auTimestamps;
                ARSTREAM2_StreamReceiver_AuReadyCallbackMetadata_t auMetadata;

                ARSAL_Mutex_Lock(&(streamReceiver->appOutput.callbackMutex));
                streamReceiver->appOutput.callbackInProgress = 1;
                if (streamReceiver->appOutput.getAuBufferCallback)
                {
                    /* call the getAuBufferCallback */
                    ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.callbackMutex));

                    cbRet = streamReceiver->appOutput.getAuBufferCallback(&auBuffer, &auBufferSize, &auBufferUserPtr, streamReceiver->appOutput.getAuBufferCallbackUserPtr);

                    ARSAL_Mutex_Lock(&(streamReceiver->appOutput.callbackMutex));
                }

                if ((cbRet != ARSTREAM2_OK) || (!auBuffer) || (auBufferSize <= 0))
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "getAuBufferCallback failed: %s", ARSTREAM2_Error_ToString(cbRet));
                    streamReceiver->appOutput.callbackInProgress = 0;
                    ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.callbackMutex));
                    ARSAL_Cond_Signal(&(streamReceiver->appOutput.callbackCond));
                }
                else
                {
                    auSize = 0;

                    for (naluItem = au->naluHead; naluItem; naluItem = naluItem->next)
                    {
                        /* filter out unwanted NAL units */
                        if ((streamReceiver->appOutput.filterOutSpsPps) && ((naluItem->nalu.naluType == ARSTREAM2_H264_NALU_TYPE_SPS) || (naluItem->nalu.naluType == ARSTREAM2_H264_NALU_TYPE_PPS)))
                        {
                            continue;
                        }

                        if ((streamReceiver->appOutput.filterOutSei) && (naluItem->nalu.naluType == ARSTREAM2_H264_NALU_TYPE_SEI))
                        {
                            continue;
                        }

                        /* copy to output buffer */
                        if (auSize + naluItem->nalu.naluSize <= (unsigned)auBufferSize)
                        {
                            memcpy(auBuffer + auSize, naluItem->nalu.nalu, naluItem->nalu.naluSize);

                            if ((naluItem->nalu.naluSize >= 4) && (streamReceiver->appOutput.replaceStartCodesWithNaluSize))
                            {
                                /* replace the NAL unit 4 bytes start code with the NALU size */
                                *(auBuffer + auSize + 0) = ((naluItem->nalu.naluSize - 4) >> 24) & 0xFF;
                                *(auBuffer + auSize + 1) = ((naluItem->nalu.naluSize - 4) >> 16) & 0xFF;
                                *(auBuffer + auSize + 2) = ((naluItem->nalu.naluSize - 4) >>  8) & 0xFF;
                                *(auBuffer + auSize + 3) = ((naluItem->nalu.naluSize - 4) >>  0) & 0xFF;
                            }

                            auSize += naluItem->nalu.naluSize;
                        }
                        else
                        {
                            break;
                        }
                    }

                    /* map the access unit sync type */
                    eARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE auSyncType;
                    switch (au->syncType)
                    {
                        default:
                        case ARSTREAM2_H264_AU_SYNC_TYPE_NONE:
                            auSyncType = ARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE_NONE;
                            break;
                        case ARSTREAM2_H264_AU_SYNC_TYPE_IDR:
                            auSyncType = ARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE_IDR;
                            break;
                        case ARSTREAM2_H264_AU_SYNC_TYPE_IFRAME:
                            auSyncType = ARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE_IFRAME;
                            break;
                        case ARSTREAM2_H264_AU_SYNC_TYPE_PIR_START:
                            auSyncType = ARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE_PIR_START;
                            break;
                    }

                    ARSAL_Time_GetTime(&t1);
                    curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
                    if (au->videoStatsAvailable)
                    {
                        ARSTREAM2_H264_VideoStats_t *vs = (ARSTREAM2_H264_VideoStats_t*)au->buffer->videoStatsBuffer;
                        uint32_t outputTimestampDelta = (streamReceiver->lastAuOutputTimestamp)
                                ? (uint32_t)(curTime - streamReceiver->lastAuOutputTimestamp) : 0;
                        uint32_t estimatedLatency = ((au->ntpTimestampLocal) && (curTime > au->ntpTimestampLocal))
                                ? (uint32_t)(curTime - au->ntpTimestampLocal) : 0;
                        int32_t timingError = ((vs->timestampDelta) && (streamReceiver->lastAuOutputTimestamp))
                                ? ((int32_t)vs->timestampDelta - (int32_t)outputTimestampDelta) : 0;
                        vs->timingError = timingError;
                        streamReceiver->timingErrorIntegral += (timingError < 0) ? (uint32_t)(-timingError) : (uint32_t)timingError;
                        vs->timingErrorIntegral = streamReceiver->timingErrorIntegral;
                        streamReceiver->timingErrorIntegralSq += (int64_t)timingError * (int64_t)timingError;
                        vs->timingErrorIntegralSq = streamReceiver->timingErrorIntegralSq;
                        vs->estimatedLatency = estimatedLatency;
                        streamReceiver->estimatedLatencyIntegral += estimatedLatency;
                        vs->estimatedLatencyIntegral = streamReceiver->estimatedLatencyIntegral;
                        streamReceiver->estimatedLatencyIntegralSq += (uint64_t)estimatedLatency * (uint64_t)estimatedLatency;
                        vs->estimatedLatencyIntegralSq = streamReceiver->estimatedLatencyIntegralSq;
                        vs->timestamp = au->ntpTimestampRaw;

                        /* get the RSSI from the streaming metadata */
                        //TODO: remove this hack once we have a better way of getting the RSSI
                        if ((au->metadataSize >= 27) && (ntohs(*((uint16_t*)au->buffer->metadataBuffer)) == 0x5031))
                        {
                            vs->rssi = (int8_t)au->buffer->metadataBuffer[26];
                        }
                        if ((au->metadataSize >= 55) && (ntohs(*((uint16_t*)au->buffer->metadataBuffer)) == 0x5032))
                        {
                            vs->rssi = (int8_t)au->buffer->metadataBuffer[54];
                        }
                        streamReceiver->lastKnownRssi = vs->rssi;

                        eARSTREAM2_ERROR recvErr = ARSTREAM2_RtpReceiver_UpdateVideoStats(streamReceiver->receiver, vs);
                        if (recvErr != ARSTREAM2_OK)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpReceiver_UpdateVideoStats() failed (%d)", recvErr);
                        }
                        ARSTREAM2_StreamStats_VideoStatsFileWrite(&streamReceiver->videoStatsCtx, vs);
                    }

                    /* timestamps and metadata */
                    memset(&auTimestamps, 0, sizeof(auTimestamps));
                    memset(&auMetadata, 0, sizeof(auMetadata));
                    auTimestamps.auNtpTimestamp = au->ntpTimestamp;
                    auTimestamps.auNtpTimestampRaw = au->ntpTimestampRaw;
                    auTimestamps.auNtpTimestampLocal = au->ntpTimestampLocal;
                    auMetadata.isComplete = au->isComplete;
                    auMetadata.hasErrors = au->hasErrors;
                    auMetadata.isRef = au->isRef;
                    auMetadata.auMetadata = (au->metadataSize > 0) ? au->buffer->metadataBuffer : NULL;
                    auMetadata.auMetadataSize = au->metadataSize;
                    auMetadata.auUserData = (au->userDataSize > 0) ? au->buffer->userDataBuffer : NULL;
                    auMetadata.auUserDataSize = au->userDataSize;
                    auMetadata.mbWidth = streamReceiver->appOutput.mbWidth;
                    auMetadata.mbHeight = streamReceiver->appOutput.mbHeight;
                    auMetadata.mbStatus = (au->mbStatusAvailable) ? au->buffer->mbStatusBuffer : NULL;
                    if (au->videoStatsAvailable)
                    {
                        /* Map the video stats */
                        ARSTREAM2_H264_VideoStats_t *vs = (ARSTREAM2_H264_VideoStats_t*)au->buffer->videoStatsBuffer;
                        ARSTREAM2_StreamStats_VideoStats_t *vsOut = &streamReceiver->appOutput.videoStats;
                        uint32_t i, j;
                        vsOut->timestamp = vs->timestamp;
                        vsOut->rssi = vs->rssi;
                        vsOut->totalFrameCount = vs->totalFrameCount;
                        vsOut->outputFrameCount = vs->outputFrameCount;
                        vsOut->erroredOutputFrameCount = vs->erroredOutputFrameCount;
                        vsOut->missedFrameCount = vs->missedFrameCount;
                        vsOut->discardedFrameCount = vs->discardedFrameCount;
                        vsOut->timestampDeltaIntegral = vs->timestampDeltaIntegral;
                        vsOut->timestampDeltaIntegralSq = vs->timestampDeltaIntegralSq;
                        vsOut->timingErrorIntegral = vs->timingErrorIntegral;
                        vsOut->timingErrorIntegralSq = vs->timingErrorIntegralSq;
                        vsOut->estimatedLatencyIntegral = vs->estimatedLatencyIntegral;
                        vsOut->estimatedLatencyIntegralSq = vs->estimatedLatencyIntegralSq;
                        vsOut->erroredSecondCount = vs->erroredSecondCount;
                        vsOut->mbStatusZoneCount = vs->mbStatusZoneCount;
                        vsOut->mbStatusClassCount = vs->mbStatusClassCount;
                        if (vs->mbStatusZoneCount == ARSTREAM2_H264_MB_STATUS_ZONE_COUNT)
                        {
                            if (vsOut->erroredSecondCountByZone)
                            {
                                for (i = 0; i < vs->mbStatusZoneCount; i++)
                                {
                                    vsOut->erroredSecondCountByZone[i] = vs->erroredSecondCountByZone[i];
                                }
                            }
                            if (vs->mbStatusClassCount == ARSTREAM2_H264_MB_STATUS_CLASS_COUNT)
                            {
                                if (vsOut->macroblockStatus)
                                {
                                    for (j = 0; j < vs->mbStatusClassCount; j++)
                                    {
                                        for (i = 0; i < vs->mbStatusZoneCount; i++)
                                        {
                                            vsOut->macroblockStatus[j * vs->mbStatusZoneCount + i] = vs->macroblockStatus[j][i];
                                        }
                                    }
                                }
                            }
                        }
                        auMetadata.videoStats = vsOut;
                    }
                    else
                    {
                        auMetadata.videoStats = NULL;
                    }
                    auMetadata.debugString = NULL; //TODO

                    if (streamReceiver->appOutput.auReadyCallback)
                    {
                        /* call the auReadyCallback */
                        ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.callbackMutex));

                        cbRet = streamReceiver->appOutput.auReadyCallback(auBuffer, auSize, &auTimestamps, auSyncType, &auMetadata,
                                                                          auBufferUserPtr, streamReceiver->appOutput.auReadyCallbackUserPtr);

                        ARSAL_Mutex_Lock(&(streamReceiver->appOutput.callbackMutex));
                    }
                    streamReceiver->appOutput.callbackInProgress = 0;
                    ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.callbackMutex));
                    ARSAL_Cond_Signal(&(streamReceiver->appOutput.callbackCond));

                    if (cbRet != ARSTREAM2_OK)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_STREAM_RECEIVER_TAG, "auReadyCallback failed: %s", ARSTREAM2_Error_ToString(cbRet));
                        if (cbRet == ARSTREAM2_ERROR_RESYNC_REQUIRED)
                        {
                            /* schedule gray IDR frame */
                            streamReceiver->appOutput.grayIFramePending = streamReceiver->appOutput.generateGrayIFrame;
                        }
                    }
                    streamReceiver->lastAuOutputTimestamp = curTime;
                }
            }

            /* free the access unit */
            ret = ARSTREAM2_H264_AuFifoUnrefBuffer(&streamReceiver->auFifo, auItem->au.buffer);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to unref buffer (%d)", ret);
            }
            ret = ARSTREAM2_H264_AuFifoPushFreeItem(&streamReceiver->auFifo, auItem);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to push free item in the AU FIFO (%d)", ret);
            }

            /* dequeue the next access unit */
            auItem = ARSTREAM2_H264_AuFifoDequeueItem(&streamReceiver->appOutput.auFifoQueue);
        }

        ARSAL_Mutex_Lock(&(streamReceiver->appOutput.threadMutex));
        shouldStop = streamReceiver->appOutput.threadShouldStop;
        running = streamReceiver->appOutput.running;
        if (!shouldStop)
        {
            ARSAL_Cond_Wait(&(streamReceiver->appOutput.threadCond), &(streamReceiver->appOutput.threadMutex));
        }
        ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.threadMutex));
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_STREAM_RECEIVER_TAG, "App output thread has ended");
    streamReceiver->appOutput.threadRunning = 0;

    return (void*)0;
}


void* ARSTREAM2_StreamReceiver_RunNetworkThread(void *streamReceiverHandle)
{
    ARSTREAM2_StreamReceiver_t *streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    ARSTREAM2_RtpResender_t *resender;
    int shouldStop, selectRet = 0;
    fd_set readSet, writeSet, exceptSet;
    fd_set *pReadSet, *pWriteSet, *pExceptSet;
    int maxFd = 0, _maxFd = 0;
    struct timeval tv;
    uint32_t nextTimeout = 0, _timeout = 0;
    eARSTREAM2_ERROR err;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return NULL;
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_STREAM_RECEIVER_TAG, "Receiver thread running");
    ARSAL_Mutex_Lock(&(streamReceiver->threadMutex));
    streamReceiver->threadStarted = 1;
    shouldStop = streamReceiver->threadShouldStop;
    ARSAL_Mutex_Unlock(&(streamReceiver->threadMutex));

    FD_ZERO(&readSet);
    FD_ZERO(&writeSet);
    FD_ZERO(&exceptSet);
    pReadSet = &readSet;
    pWriteSet = &writeSet;
    pExceptSet = &exceptSet;

    err = ARSTREAM2_RtpReceiver_GetSelectParams(streamReceiver->receiver, &pReadSet, &pWriteSet, &pExceptSet, &maxFd, &nextTimeout);
    if (err != ARSTREAM2_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpReceiver_GetSelectParams() failed (%d)", err);
        return (void *)0;
    }

    ARSAL_Mutex_Lock(&(streamReceiver->resendMutex));
    for (resender = streamReceiver->resender; resender; resender = resender->next)
    {
        err = ARSTREAM2_RtpSender_GetSelectParams(resender->sender, &pReadSet, &pWriteSet, &pExceptSet, &_maxFd, &_timeout);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpReceiver_GetSelectParams() failed (%d)", err);
            return (void *)0;
        }
        if (_timeout < nextTimeout) nextTimeout = _timeout;
        if (_maxFd > maxFd) maxFd = _maxFd;
    }
    ARSAL_Mutex_Unlock(&(streamReceiver->resendMutex));

    if (pReadSet)
        FD_SET(streamReceiver->signalPipe[0], pReadSet);
    if (pExceptSet)
        FD_SET(streamReceiver->signalPipe[0], pExceptSet);
    if (streamReceiver->signalPipe[0] > maxFd) maxFd = streamReceiver->signalPipe[0];
    maxFd++;
    tv.tv_sec = 0;
    tv.tv_usec = nextTimeout;

    while (shouldStop == 0)
    {
        if ((pReadSet) && (pWriteSet) && (pExceptSet))
        {
            while (((selectRet = select(maxFd, pReadSet, pWriteSet, pExceptSet, &tv)) == -1) && (errno == EINTR));

            if (selectRet < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Select error (%d): %s", errno, strerror(errno));
            }
        }

        ARSAL_Mutex_Lock(&(streamReceiver->resendMutex));

        err = ARSTREAM2_RtpReceiver_ProcessRtcp(streamReceiver->receiver, selectRet, pReadSet, pWriteSet, pExceptSet, &shouldStop);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpReceiver_ProcessRtcp() failed (%d)", err);
        }
        err = ARSTREAM2_RtpReceiver_ProcessRtp(streamReceiver->receiver, selectRet, pReadSet, pWriteSet, pExceptSet, &shouldStop,
                                               streamReceiver->resendQueue, streamReceiver->resendTimeout, streamReceiver->resendCount);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpReceiver_ProcessRtp() failed (%d)", err);
        }

        for (resender = streamReceiver->resender; resender; resender = resender->next)
        {
            err = ARSTREAM2_RtpSender_ProcessRtcp(resender->sender, selectRet, pReadSet, pWriteSet, pExceptSet);
            if (err != ARSTREAM2_OK)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpSender_ProcessRtcp() failed (%d)", err);
            }
            err = ARSTREAM2_RtpSender_ProcessRtp(resender->sender, selectRet, pReadSet, pWriteSet, pExceptSet);
            if (err != ARSTREAM2_OK)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpSender_ProcessRtp() failed (%d)", err);
            }
        }

        ARSAL_Mutex_Unlock(&(streamReceiver->resendMutex));

        if ((pReadSet) && ((selectRet >= 0) && (FD_ISSET(streamReceiver->signalPipe[0], pReadSet))))
        {
            /* Dump bytes (so it won't be ready next time) */
            char dump[10];
            int readRet;
            while (((readRet = read(streamReceiver->signalPipe[0], &dump, 10)) == -1) && (errno == EINTR));
            if (readRet < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Failed to read from pipe (%d): %s", errno, strerror(errno));
            }
        }

        if (!shouldStop)
        {
            ARSAL_Mutex_Lock(&(streamReceiver->threadMutex));
            shouldStop = streamReceiver->threadShouldStop;
            ARSAL_Mutex_Unlock(&(streamReceiver->threadMutex));
        }

        if (!shouldStop)
        {
            /* Prepare the next select */
            FD_ZERO(&readSet);
            FD_ZERO(&writeSet);
            FD_ZERO(&exceptSet);
            pReadSet = &readSet;
            pWriteSet = &writeSet;
            pExceptSet = &exceptSet;

            err = ARSTREAM2_RtpReceiver_GetSelectParams(streamReceiver->receiver, &pReadSet, &pWriteSet, &pExceptSet, &maxFd, &nextTimeout);
            if (err != ARSTREAM2_OK)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpReceiver_GetSelectParams() failed (%d)", err);
                break;
            }

            ARSAL_Mutex_Lock(&(streamReceiver->resendMutex));
            for (resender = streamReceiver->resender; resender; resender = resender->next)
            {
                err = ARSTREAM2_RtpSender_GetSelectParams(resender->sender, &pReadSet, &pWriteSet, &pExceptSet, &_maxFd, &_timeout);
                if (err != ARSTREAM2_OK)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpReceiver_GetSelectParams() failed (%d)", err);
                    break;
                }
                if (_timeout < nextTimeout) nextTimeout = _timeout;
                if (_maxFd > maxFd) maxFd = _maxFd;
            }
            ARSAL_Mutex_Unlock(&(streamReceiver->resendMutex));

            if (pReadSet)
                FD_SET(streamReceiver->signalPipe[0], pReadSet);
            if (pExceptSet)
                FD_SET(streamReceiver->signalPipe[0], pExceptSet);
            if (streamReceiver->signalPipe[0] > maxFd) maxFd = streamReceiver->signalPipe[0];
            maxFd++;
            tv.tv_sec = 0;
            tv.tv_usec = nextTimeout;
        }
    }

    ARSAL_Mutex_Lock(&(streamReceiver->threadMutex));
    streamReceiver->threadStarted = 0;
    ARSAL_Mutex_Unlock(&(streamReceiver->threadMutex));

    err = ARSTREAM2_RtpReceiver_ProcessEnd(streamReceiver->receiver, 0);
    if (err != ARSTREAM2_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpReceiver_ProcessEnd() failed (%d)", err);
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_STREAM_RECEIVER_TAG, "Receiver thread has ended");

    return (void*)0;
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_StartAppOutput(ARSTREAM2_StreamReceiver_Handle streamReceiverHandle,
                                                         ARSTREAM2_StreamReceiver_SpsPpsCallback_t spsPpsCallback, void* spsPpsCallbackUserPtr,
                                                         ARSTREAM2_StreamReceiver_GetAuBufferCallback_t getAuBufferCallback, void* getAuBufferCallbackUserPtr,
                                                         ARSTREAM2_StreamReceiver_AuReadyCallback_t auReadyCallback, void* auReadyCallbackUserPtr)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!getAuBufferCallback)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid getAuBufferCallback function pointer");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!auReadyCallback)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid auReadyCallback function pointer");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    ARSAL_Mutex_Lock(&(streamReceiver->appOutput.threadMutex));
    int running = streamReceiver->appOutput.running;
    ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.threadMutex));
    if (running)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Application output is already running");
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    int auFifoRet = ARSTREAM2_H264_AuFifoAddQueue(&streamReceiver->auFifo, &streamReceiver->appOutput.auFifoQueue);
    if (auFifoRet != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264_AuFifoAddQueue() failed (%d)", auFifoRet);
        ret = ARSTREAM2_ERROR_ALLOC;
    }

    ARSAL_Mutex_Lock(&(streamReceiver->appOutput.callbackMutex));
    while (streamReceiver->appOutput.callbackInProgress)
    {
        ARSAL_Cond_Wait(&(streamReceiver->appOutput.callbackCond), &(streamReceiver->appOutput.callbackMutex));
    }
    streamReceiver->appOutput.spsPpsCallback = spsPpsCallback;
    streamReceiver->appOutput.spsPpsCallbackUserPtr = spsPpsCallbackUserPtr;
    streamReceiver->appOutput.getAuBufferCallback = getAuBufferCallback;
    streamReceiver->appOutput.getAuBufferCallbackUserPtr = getAuBufferCallbackUserPtr;
    streamReceiver->appOutput.auReadyCallback = auReadyCallback;
    streamReceiver->appOutput.auReadyCallbackUserPtr = auReadyCallbackUserPtr;
    ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.callbackMutex));

    if (streamReceiver->sync)
    {
        /* call the app output SPS/PPS callback if already synchronized */
        ARSAL_Mutex_Lock(&(streamReceiver->appOutput.callbackMutex));
        streamReceiver->appOutput.callbackInProgress = 1;
        if (streamReceiver->appOutput.spsPpsCallback)
        {
            ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.callbackMutex));

            eARSTREAM2_ERROR cbRet;
            cbRet = streamReceiver->appOutput.spsPpsCallback(streamReceiver->pSps, streamReceiver->spsSize,
                                                             streamReceiver->pPps, streamReceiver->ppsSize,
                                                             streamReceiver->appOutput.spsPpsCallbackUserPtr);
            if (cbRet != ARSTREAM2_OK)
            {
                ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_STREAM_RECEIVER_TAG, "Application SPS/PPS callback failed");
            }

            ARSAL_Mutex_Lock(&(streamReceiver->appOutput.callbackMutex));
        }
        streamReceiver->appOutput.callbackInProgress = 0;
        ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.callbackMutex));
        ARSAL_Cond_Signal(&(streamReceiver->appOutput.callbackCond));
    }

    ARSAL_Mutex_Lock(&(streamReceiver->appOutput.threadMutex));
    streamReceiver->appOutput.grayIFramePending = streamReceiver->appOutput.generateGrayIFrame;
    streamReceiver->appOutput.running = 1;
    ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.threadMutex));

    ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_STREAM_RECEIVER_TAG, "App output is running");

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_StopAppOutput(ARSTREAM2_StreamReceiver_Handle streamReceiverHandle)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    ARSAL_Mutex_Lock(&(streamReceiver->appOutput.threadMutex));
    streamReceiver->appOutput.running = 0;
    ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.threadMutex));

    ARSAL_Mutex_Lock(&(streamReceiver->appOutput.callbackMutex));
    while (streamReceiver->appOutput.callbackInProgress)
    {
        ARSAL_Cond_Wait(&(streamReceiver->appOutput.callbackCond), &(streamReceiver->appOutput.callbackMutex));
    }
    streamReceiver->appOutput.spsPpsCallback = NULL;
    streamReceiver->appOutput.spsPpsCallbackUserPtr = NULL;
    streamReceiver->appOutput.getAuBufferCallback = NULL;
    streamReceiver->appOutput.getAuBufferCallbackUserPtr = NULL;
    streamReceiver->appOutput.auReadyCallback = NULL;
    streamReceiver->appOutput.auReadyCallbackUserPtr = NULL;
    ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.callbackMutex));

    int auFifoRet = ARSTREAM2_H264_AuFifoRemoveQueue(&streamReceiver->auFifo, &streamReceiver->appOutput.auFifoQueue);
    if (auFifoRet != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_H264_AuFifoRemoveQueue() failed (%d)", auFifoRet);
        ret = ARSTREAM2_ERROR_ALLOC;
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_STREAM_RECEIVER_TAG, "App output is stopped");

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_Stop(ARSTREAM2_StreamReceiver_Handle streamReceiverHandle)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_STREAM_RECEIVER_TAG, "Stopping receiver...");

    int recErr = ARSTREAM2_StreamReceiver_StreamRecorderStop(streamReceiver);
    if (recErr != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamReceiver_StreamRecorderStop() failed (%d)", recErr);
    }

    ARSAL_Mutex_Lock(&(streamReceiver->threadMutex));
    streamReceiver->threadShouldStop = 1;
    ARSAL_Mutex_Unlock(&(streamReceiver->threadMutex));

    if (streamReceiver->signalPipe[1] != -1)
    {
        char * buff = "x";
        write(streamReceiver->signalPipe[1], buff, 1);
    }

    ARSTREAM2_RtpReceiver_Stop(streamReceiver->receiver);

    ARSAL_Mutex_Lock(&(streamReceiver->appOutput.threadMutex));
    streamReceiver->appOutput.threadShouldStop = 1;
    ARSAL_Mutex_Unlock(&(streamReceiver->appOutput.threadMutex));
    /* signal the thread to avoid a deadlock */
    ARSAL_Cond_Signal(&(streamReceiver->appOutput.threadCond));

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_GetSpsPps(ARSTREAM2_StreamReceiver_Handle streamReceiverHandle, uint8_t *spsBuffer, int *spsSize, uint8_t *ppsBuffer, int *ppsSize)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if ((!spsSize) || (!ppsSize))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid size pointers");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (!streamReceiver->sync)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "No sync");
        ret = ARSTREAM2_ERROR_WAITING_FOR_SYNC;
    }

    if (ret == ARSTREAM2_OK)
    {
        if ((!spsBuffer) || (*spsSize < streamReceiver->spsSize))
        {
            *spsSize = streamReceiver->spsSize;
        }
        else
        {
            memcpy(spsBuffer, streamReceiver->pSps, streamReceiver->spsSize);
            *spsSize = streamReceiver->spsSize;
        }

        if ((!ppsBuffer) || (*ppsSize < streamReceiver->ppsSize))
        {
            *ppsSize = streamReceiver->ppsSize;
        }
        else
        {
            memcpy(ppsBuffer, streamReceiver->pPps, streamReceiver->ppsSize);
            *ppsSize = streamReceiver->ppsSize;
        }
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_GetUntimedMetadata(ARSTREAM2_StreamReceiver_Handle streamReceiverHandle,
                                                             ARSTREAM2_Stream_UntimedMetadata_t *metadata, uint32_t *sendInterval)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK, _ret;
    uint32_t _sendInterval = 0, minSendInterval = (uint32_t)(-1);
    char *ptr;
    int i;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!metadata)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid metadata");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_CNAME_ITEM, NULL, &metadata->serialNumber, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->serialNumber = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_NAME_ITEM, NULL, &metadata->friendlyName, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->friendlyName = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_TOOL_ITEM, NULL, &metadata->softwareVersion, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->softwareVersion = NULL;
    }

    ptr = NULL;
    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_LOC_ITEM, NULL, &ptr, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
        if (ptr)
        {
            if (sscanf(ptr, "%lf,%lf,%f", &metadata->takeoffLatitude, &metadata->takeoffLongitude, &metadata->takeoffAltitude) != 3)
            {
                metadata->takeoffLatitude = 500.;
                metadata->takeoffLongitude = 500.;
                metadata->takeoffAltitude = 0.;
            }
        }
    }
    else
    {
        metadata->takeoffLatitude = 500.;
        metadata->takeoffLongitude = 500.;
        metadata->takeoffAltitude = 0.;
    }

    ptr = NULL;
    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_hfov", &ptr, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
        if (ptr)
        {
            if (sscanf(ptr, "%f", &metadata->pictureHFov) != 1)
            {
                metadata->pictureHFov = 0.;
            }
        }
    }
    else
    {
        metadata->pictureHFov = 0.;
    }

    ptr = NULL;
    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_vfov", &ptr, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
        if (ptr)
        {
            if (sscanf(ptr, "%f", &metadata->pictureVFov) != 1)
            {
                metadata->pictureVFov = 0.;
            }
        }
    }
    else
    {
        metadata->pictureVFov = 0.;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_date", &metadata->runDate, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->runDate = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_id", &metadata->runUuid, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->runUuid = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "maker", &metadata->maker, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->maker = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model", &metadata->model, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->model = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model_id", &metadata->modelId, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->modelId = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "build_id", &metadata->buildId, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->buildId = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "title", &metadata->title, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->title = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "comment", &metadata->comment, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->comment = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "copyright", &metadata->copyright, &_sendInterval);
    if (_ret == ARSTREAM2_OK)
    {
        if (_sendInterval < minSendInterval)
        {
            minSendInterval = _sendInterval;
        }
    }
    else
    {
        metadata->copyright = NULL;
    }

    for (i = 0; i < ARSTREAM2_STREAM_UNTIMEDMETADATA_CUSTOM_MAX_COUNT; i++)
    {
        if ((metadata->custom[i].key) && (strlen(metadata->custom[i].key)))
        {
            _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, metadata->custom[i].key, &metadata->custom[i].value, &_sendInterval);
            if (_ret == ARSTREAM2_OK)
            {
                if (_sendInterval < minSendInterval)
                {
                    minSendInterval = _sendInterval;
                }
            }
            else
            {
                metadata->custom[i].value = NULL;
            }
        }
    }

    if (sendInterval)
    {
        *sendInterval = minSendInterval;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_SetUntimedMetadata(ARSTREAM2_StreamReceiver_Handle streamReceiverHandle,
                                                             const ARSTREAM2_Stream_UntimedMetadata_t *metadata, uint32_t sendInterval)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK, _ret;
    char *ptr;
    int i;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!metadata)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid metadata");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (sendInterval == 0)
    {
        sendInterval = ARSTREAM2_STREAM_RECEIVER_UNTIMED_METADATA_DEFAULT_SEND_INTERVAL;
    }

    if ((metadata->serialNumber) && (strlen(metadata->serialNumber)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_CNAME_ITEM, NULL, &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->serialNumber, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_CNAME_ITEM, NULL, metadata->serialNumber, sendInterval);
        }
    }

    if ((metadata->friendlyName) && (strlen(metadata->friendlyName)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_NAME_ITEM, NULL, &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->friendlyName, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_NAME_ITEM, NULL, metadata->friendlyName, sendInterval);
        }
    }

    if ((metadata->softwareVersion) && (strlen(metadata->softwareVersion)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_TOOL_ITEM, NULL, &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->softwareVersion, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_TOOL_ITEM, NULL, metadata->softwareVersion, sendInterval);
        }
    }

    if ((metadata->takeoffLatitude != 500.) && (metadata->takeoffLongitude != 500.))
    {
        double takeoffLatitude = 500.;
        double takeoffLongitude = 500.;
        float takeoffAltitude = 0.;
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_LOC_ITEM, NULL, &ptr, NULL);
        if (_ret == ARSTREAM2_OK)
        {
            if (ptr)
            {
                if (sscanf(ptr, "%lf,%lf,%f", &takeoffLatitude, &takeoffLongitude, &takeoffAltitude) != 3)
                {
                    takeoffLatitude = 500.;
                    takeoffLongitude = 500.;
                    takeoffAltitude = 0.;
                }
            }
        }
        if ((takeoffLatitude != metadata->takeoffLatitude) || (takeoffLongitude != metadata->takeoffLongitude) || (takeoffAltitude != metadata->takeoffAltitude))
        {
            char str[100];
            snprintf(str, sizeof(str), "%.8f,%.8f,%.2f", metadata->takeoffLatitude, metadata->takeoffLongitude, metadata->takeoffAltitude);
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_LOC_ITEM, NULL, str, sendInterval);
        }
    }

    if (metadata->pictureHFov != 0.)
    {
        float pictureHFov = 0.;
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_hfov", &ptr, NULL);
        if (_ret == ARSTREAM2_OK)
        {
            if (ptr)
            {
                if (sscanf(ptr, "%f", &pictureHFov) != 1)
                {
                    pictureHFov = 0.;
                }
            }
        }
        if (pictureHFov != metadata->pictureHFov)
        {
            char str[100];
            snprintf(str, sizeof(str), "%.2f", metadata->pictureHFov);
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_hfov", str, sendInterval);
        }
    }

    if (metadata->pictureVFov != 0.)
    {
        float pictureVFov = 0.;
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_vfov", &ptr, NULL);
        if (_ret == ARSTREAM2_OK)
        {
            if (ptr)
            {
                if (sscanf(ptr, "%f", &pictureVFov) != 1)
                {
                    pictureVFov = 0.;
                }
            }
        }
        if (pictureVFov != metadata->pictureVFov)
        {
            char str[100];
            snprintf(str, sizeof(str), "%.2f", metadata->pictureVFov);
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_vfov", str, sendInterval);
        }
    }

    if ((metadata->runDate) && (strlen(metadata->runDate)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_date", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->runDate, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_date", metadata->runDate, sendInterval);
        }
    }

    if ((metadata->runUuid) && (strlen(metadata->runUuid)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_id", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->runUuid, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_id", metadata->runUuid, sendInterval);
        }
    }

    if ((metadata->maker) && (strlen(metadata->maker)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "maker", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->maker, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "maker", metadata->maker, sendInterval);
        }
    }

    if ((metadata->model) && (strlen(metadata->model)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->model, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model", metadata->model, sendInterval);
        }
    }

    if ((metadata->modelId) && (strlen(metadata->modelId)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model_id", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->modelId, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model_id", metadata->modelId, sendInterval);
        }
    }

    if ((metadata->buildId) && (strlen(metadata->buildId)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "build_id", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->buildId, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "build_id", metadata->buildId, sendInterval);
        }
    }

    if ((metadata->title) && (strlen(metadata->title)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "title", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->title, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "title", metadata->title, sendInterval);
        }
    }

    if ((metadata->comment) && (strlen(metadata->comment)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "comment", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->comment, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "comment", metadata->comment, sendInterval);
        }
    }

    if ((metadata->copyright) && (strlen(metadata->copyright)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "copyright", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->copyright, 256)))
        {
            ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "copyright", metadata->copyright, sendInterval);
        }
    }

    for (i = 0; i < ARSTREAM2_STREAM_UNTIMEDMETADATA_CUSTOM_MAX_COUNT; i++)
    {
        if ((metadata->custom[i].key) && (strlen(metadata->custom[i].key)) && (metadata->custom[i].value) && (strlen(metadata->custom[i].value)))
        {
            ptr = NULL;
            _ret = ARSTREAM2_RtpReceiver_GetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, metadata->custom[i].key, &ptr, NULL);
            if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->custom[i].value, 256)))
            {
                ARSTREAM2_RtpReceiver_SetSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, metadata->custom[i].key, metadata->custom[i].value, sendInterval);
            }
        }
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_GetPeerUntimedMetadata(ARSTREAM2_StreamReceiver_Handle streamReceiverHandle,
                                                                 ARSTREAM2_Stream_UntimedMetadata_t *metadata)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK, _ret;
    char *ptr;
    int i;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!metadata)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid metadata");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_CNAME_ITEM, NULL, &metadata->serialNumber);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->serialNumber = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_NAME_ITEM, NULL, &metadata->friendlyName);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->friendlyName = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_TOOL_ITEM, NULL, &metadata->softwareVersion);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->softwareVersion = NULL;
    }

    ptr = NULL;
    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_LOC_ITEM, NULL, &ptr);
    if (_ret == ARSTREAM2_OK)
    {
        if (ptr)
        {
            if (sscanf(ptr, "%lf,%lf,%f", &metadata->takeoffLatitude, &metadata->takeoffLongitude, &metadata->takeoffAltitude) != 3)
            {
                metadata->takeoffLatitude = 500.;
                metadata->takeoffLongitude = 500.;
                metadata->takeoffAltitude = 0.;
            }
        }
    }
    else
    {
        metadata->takeoffLatitude = 500.;
        metadata->takeoffLongitude = 500.;
        metadata->takeoffAltitude = 0.;
    }

    ptr = NULL;
    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_hfov", &ptr);
    if (_ret == ARSTREAM2_OK)
    {
        if (ptr)
        {
            if (sscanf(ptr, "%f", &metadata->pictureHFov) != 1)
            {
                metadata->pictureHFov = 0.;
            }
        }
    }
    else
    {
        metadata->pictureHFov = 0.;
    }

    ptr = NULL;
    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_vfov", &ptr);
    if (_ret == ARSTREAM2_OK)
    {
        if (ptr)
        {
            if (sscanf(ptr, "%f", &metadata->pictureVFov) != 1)
            {
                metadata->pictureVFov = 0.;
            }
        }
    }
    else
    {
        metadata->pictureVFov = 0.;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_date", &metadata->runDate);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->runDate = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_id", &metadata->runUuid);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->runUuid = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "maker", &metadata->maker);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->maker = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model", &metadata->model);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->model = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model_id", &metadata->modelId);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->modelId = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "build_id", &metadata->buildId);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->buildId = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "title", &metadata->title);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->title = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "comment", &metadata->comment);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->comment = NULL;
    }

    _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "copyright", &metadata->copyright);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->copyright = NULL;
    }

    for (i = 0; i < ARSTREAM2_STREAM_UNTIMEDMETADATA_CUSTOM_MAX_COUNT; i++)
    {
        if ((metadata->custom[i].key) && (strlen(metadata->custom[i].key)))
        {
            _ret = ARSTREAM2_RtpReceiver_GetPeerSdesItem(streamReceiver->receiver, ARSTREAM2_RTCP_SDES_PRIV_ITEM, metadata->custom[i].key, &metadata->custom[i].value);
            if (_ret != ARSTREAM2_OK)
            {
                metadata->custom[i].value = NULL;
            }
        }
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_StartResender(ARSTREAM2_StreamReceiver_Handle streamReceiverHandle,
                                                        ARSTREAM2_StreamReceiver_ResenderHandle *streamResenderHandle,
                                                        const ARSTREAM2_StreamReceiver_ResenderConfig_t *config)
{
    ARSTREAM2_StreamReceiver_t* streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    ARSTREAM2_RtpResender_t* resender = NULL;
    int packetFifoQueueCreated = 0;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!streamResenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid pointer for resender");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!config)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid pointer for config");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    resender = (ARSTREAM2_RtpResender_t*)malloc(sizeof(*resender));
    if (!resender)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Allocation failed (size %zu)", sizeof(*resender));
        ret = ARSTREAM2_ERROR_ALLOC;
    }

    if (ret == ARSTREAM2_OK)
    {
        memset(resender, 0, sizeof(*resender));
        resender->streamSocketSendBufferSize = (config->streamSocketBufferSize > 0) ? config->streamSocketBufferSize : ARSTREAM2_RTP_RESENDER_DEFAULT_STREAM_SOCKET_SEND_BUFFER_SIZE;
        resender->maxNetworkLatencyUs = (config->maxNetworkLatencyMs > 0) ? config->maxNetworkLatencyMs * 1000 : 0;
    }

    ARSAL_Mutex_Lock(&(streamReceiver->resendMutex));

    if (ret == ARSTREAM2_OK)
    {
        int packetFifoRet = ARSTREAM2_RTP_PacketFifoAddQueue(&streamReceiver->packetFifo, &resender->packetFifoQueue);
        if (packetFifoRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RTP_PacketFifoAddQueue() failed (%d)", packetFifoRet);
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            packetFifoQueueCreated = 1;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        ARSTREAM2_RtpSender_Config_t senderConfig;
        memset(&senderConfig, 0, sizeof(senderConfig));
        senderConfig.canonicalName = config->canonicalName;
        senderConfig.friendlyName = config->friendlyName;
        senderConfig.applicationName = config->applicationName;
        senderConfig.clientAddr = config->clientAddr;
        senderConfig.mcastAddr = config->mcastAddr;
        senderConfig.mcastIfaceAddr = config->mcastIfaceAddr;
        senderConfig.serverStreamPort = config->serverStreamPort;
        senderConfig.serverControlPort = config->serverControlPort;
        senderConfig.clientStreamPort = config->clientStreamPort;
        senderConfig.clientControlPort = config->clientControlPort;
        senderConfig.classSelector = config->classSelector;
        senderConfig.streamSocketSendBufferSize = resender->streamSocketSendBufferSize;
        senderConfig.maxPacketSize = streamReceiver->maxPacketSize;
        senderConfig.naluFifo = NULL;
        senderConfig.packetFifo = &streamReceiver->packetFifo;
        senderConfig.packetFifoQueue = &resender->packetFifoQueue;
        senderConfig.debugPath = streamReceiver->debugPath;
        senderConfig.dateAndTime = streamReceiver->dateAndTime;

        resender->sender = ARSTREAM2_RtpSender_New(&senderConfig, &ret);
        if (ret != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Error while creating sender : %s", ARSTREAM2_Error_ToString(ret));
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        resender->prev = NULL;
        resender->next = streamReceiver->resender;
        if (resender->next)
        {
            resender->next->prev = resender;
        }
        streamReceiver->resender = resender;
    }

    if (ret == ARSTREAM2_OK)
    {
        ARSTREAM2_RtpResender_t* r;
        for (r = streamReceiver->resender, streamReceiver->resendCount = 0; r; r = resender->next)
        {
            streamReceiver->resendCount++;
        }
        if (streamReceiver->resendCount > 0)
        {
            streamReceiver->resendQueue = realloc(streamReceiver->resendQueue, streamReceiver->resendCount * sizeof(ARSTREAM2_RTP_PacketFifoQueue_t*));
            if (!streamReceiver->resendQueue)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Allocation failed (size %zu)", streamReceiver->resendCount * sizeof(ARSTREAM2_RTP_PacketFifoQueue_t*));
                ret = ARSTREAM2_ERROR_ALLOC;
            }
            else
            {
                int k;
                for (r = streamReceiver->resender, k = 0; r; r = resender->next, k++)
                    streamReceiver->resendQueue[k] = &r->packetFifoQueue;
            }
            streamReceiver->resendTimeout = realloc(streamReceiver->resendTimeout, streamReceiver->resendCount * sizeof(uint32_t));
            if (!streamReceiver->resendTimeout)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Allocation failed (size %zu)", streamReceiver->resendCount * sizeof(uint32_t));
                ret = ARSTREAM2_ERROR_ALLOC;
            }
            else
            {
                int k;
                for (r = streamReceiver->resender, k = 0; r; r = resender->next, k++)
                    streamReceiver->resendTimeout[k] = r->maxNetworkLatencyUs;
            }
        }
        else
        {
            if (streamReceiver->resendQueue)
            {
                free(streamReceiver->resendQueue);
                streamReceiver->resendQueue = NULL;
            }
            if (streamReceiver->resendTimeout)
            {
                free(streamReceiver->resendTimeout);
                streamReceiver->resendTimeout = NULL;
            }
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        *streamResenderHandle = resender;
    }
    else
    {
        if (resender)
        {
            if (resender->sender) ARSTREAM2_RtpSender_Delete(&(resender->sender));
            if (packetFifoQueueCreated) ARSTREAM2_RTP_PacketFifoRemoveQueue(&streamReceiver->packetFifo, &resender->packetFifoQueue);
            free(resender);
        }
        *streamResenderHandle = NULL;
    }

    ARSAL_Mutex_Unlock(&(streamReceiver->resendMutex));

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_StopResender(ARSTREAM2_StreamReceiver_Handle streamReceiverHandle,
                                                       ARSTREAM2_StreamReceiver_ResenderHandle *streamResenderHandle)
{
    ARSTREAM2_StreamReceiver_t *streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    ARSTREAM2_RtpResender_t* resender;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid receiver handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((!streamResenderHandle) || (!*streamResenderHandle))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid pointer for handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    resender = (ARSTREAM2_RtpResender_t*)*streamResenderHandle;

    ARSAL_Mutex_Lock(&(streamReceiver->resendMutex));

    ret = ARSTREAM2_RtpSender_ProcessEnd(resender->sender, 1);
    if (ret != ARSTREAM2_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_RtpSender_ProcessEnd() failed (%d)", ret);
    }
    ARSTREAM2_RTP_PacketFifoRemoveQueue(&streamReceiver->packetFifo, &resender->packetFifoQueue);

    ret = ARSTREAM2_RtpSender_Delete(&resender->sender);
    if (ret != ARSTREAM2_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Unable to delete sender: %s", ARSTREAM2_Error_ToString(ret));
    }

    if (resender->prev)
    {
        resender->prev->next = resender->next;
    }
    if (resender->next)
    {
        resender->next->prev = resender->prev;
    }
    if (resender == streamReceiver->resender)
    {
        streamReceiver->resender = resender->next;
    }

    ARSTREAM2_RtpResender_t* r;
    for (r = streamReceiver->resender, streamReceiver->resendCount = 0; r; r = resender->next)
    {
        streamReceiver->resendCount++;
    }
    if (streamReceiver->resendCount > 0)
    {
        streamReceiver->resendQueue = realloc(streamReceiver->resendQueue, streamReceiver->resendCount * sizeof(ARSTREAM2_RTP_PacketFifoQueue_t*));
        if (!streamReceiver->resendQueue)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Allocation failed (size %zu)", streamReceiver->resendCount * sizeof(ARSTREAM2_RTP_PacketFifoQueue_t*));
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            int k;
            for (r = streamReceiver->resender, k = 0; r; r = resender->next, k++)
                streamReceiver->resendQueue[k] = &r->packetFifoQueue;
        }
        streamReceiver->resendTimeout = realloc(streamReceiver->resendTimeout, streamReceiver->resendCount * sizeof(uint32_t));
        if (!streamReceiver->resendTimeout)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Allocation failed (size %zu)", streamReceiver->resendCount * sizeof(uint32_t));
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            int k;
            for (r = streamReceiver->resender, k = 0; r; r = resender->next, k++)
                streamReceiver->resendTimeout[k] = r->maxNetworkLatencyUs;
        }
    }
    else
    {
        if (streamReceiver->resendQueue)
        {
            free(streamReceiver->resendQueue);
            streamReceiver->resendQueue = NULL;
        }
        if (streamReceiver->resendTimeout)
        {
            free(streamReceiver->resendTimeout);
            streamReceiver->resendTimeout = NULL;
        }
    }

    ARSAL_Mutex_Unlock(&(streamReceiver->resendMutex));

    free(resender);
    *streamResenderHandle = NULL;

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_StartRecorder(ARSTREAM2_StreamReceiver_Handle streamReceiverHandle, const char *recordFileName)
{
    ARSTREAM2_StreamReceiver_t *streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((!recordFileName) || (!strlen(recordFileName)))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid record file name");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (streamReceiver->recorder.recorder)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Recorder is already started");
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    streamReceiver->recorder.fileName = strdup(recordFileName);
    if (!streamReceiver->recorder.fileName)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "String allocation failed");
        ret = ARSTREAM2_ERROR_ALLOC;
    }
    else
    {
        if (streamReceiver->sync)
        {
            streamReceiver->recorder.startPending = 0;
            int recRet;
            recRet = ARSTREAM2_StreamReceiver_StreamRecorderInit(streamReceiver);
            if (recRet != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamReceiver_StreamRecorderInit() failed (%d)", recRet);
            }
        }
        else
        {
            streamReceiver->recorder.startPending = 1;
        }
    }

    return ret;
}


static void ARSTREAM2_StreamReceiver_AutoStartRecorder(ARSTREAM2_StreamReceiver_t *streamReceiver)
{
    char szOutputFileName[500];
    szOutputFileName[0] = '\0';

    if ((streamReceiver->debugPath) && (strlen(streamReceiver->debugPath)))
    {
        snprintf(szOutputFileName, 500, "%s/%s", streamReceiver->debugPath,
                 ARSTREAM2_STREAM_RECEIVER_VIDEO_AUTOREC_OUTPUT_PATH);
        if ((access(szOutputFileName, F_OK) == 0) && (access(szOutputFileName, W_OK) == 0))
        {
            // directory exists and we have write permission
            snprintf(szOutputFileName, 500, "%s/%s/%s_%s.%s", streamReceiver->debugPath,
                     ARSTREAM2_STREAM_RECEIVER_VIDEO_AUTOREC_OUTPUT_PATH,
                     ARSTREAM2_STREAM_RECEIVER_VIDEO_AUTOREC_OUTPUT_FILENAME,
                     streamReceiver->dateAndTime,
                     ARSTREAM2_STREAM_RECEIVER_VIDEO_AUTOREC_OUTPUT_FILEEXT);
        }
        else
        {
            szOutputFileName[0] = '\0';
        }
    }

    if (strlen(szOutputFileName))
    {
        if (streamReceiver->recorder.recorder)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Auto record failed: recorder is already started");
            return;
        }

        streamReceiver->recorder.fileName = strdup(szOutputFileName);
        if (!streamReceiver->recorder.fileName)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Auto record failed: string allocation failed");
            return;
        }
        else
        {
            if (streamReceiver->sync)
            {
                streamReceiver->recorder.startPending = 0;
                int recRet;
                recRet = ARSTREAM2_StreamReceiver_StreamRecorderInit(streamReceiver);
                if (recRet != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Auto record failed: ARSTREAM2_StreamReceiver_StreamRecorderInit() failed (%d)", recRet);
                }
            }
            else
            {
                streamReceiver->recorder.startPending = 1;
            }
            ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_STREAM_RECEIVER_TAG, "Auto record started (file '%s')", streamReceiver->recorder.fileName);
        }
    }
}


eARSTREAM2_ERROR ARSTREAM2_StreamReceiver_StopRecorder(ARSTREAM2_StreamReceiver_Handle streamReceiverHandle)
{
    ARSTREAM2_StreamReceiver_t *streamReceiver = (ARSTREAM2_StreamReceiver_t*)streamReceiverHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!streamReceiverHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!streamReceiver->recorder.recorder)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "Recorder not started");
        return ARSTREAM2_ERROR_INVALID_STATE;
    }

    int recRet = ARSTREAM2_StreamReceiver_StreamRecorderStop(streamReceiver);
    if (recRet != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamReceiver_StreamRecorderStop() failed (%d)", recRet);
        ret = ARSTREAM2_ERROR_INVALID_STATE;
    }

    recRet = ARSTREAM2_StreamReceiver_StreamRecorderFree(streamReceiver);
    if (recRet != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_RECEIVER_TAG, "ARSTREAM2_StreamReceiver_StreamRecorderFree() failed (%d)", recRet);
        ret = ARSTREAM2_ERROR_INVALID_STATE;
    }

    free(streamReceiver->recorder.fileName);
    streamReceiver->recorder.fileName = NULL;

    return ret;
}
