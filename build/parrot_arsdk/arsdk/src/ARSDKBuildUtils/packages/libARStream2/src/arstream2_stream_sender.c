/**
 * @file arstream2_stream_sender.c
 * @brief Parrot Streaming Library - Stream Sender
 * @date 08/03/2016
 * @author aurelien.barre@parrot.com
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <libARSAL/ARSAL_Print.h>

#include <libARStream2/arstream2_stream_sender.h>
#include "arstream2_rtp_sender.h"
#include "arstream2_stream_stats_internal.h"


/**
 * Tag for ARSAL_PRINT
 */
#define ARSTREAM2_STREAM_SENDER_TAG "ARSTREAM2_StreamSender"

#define ARSTREAM2_STREAM_SENDER_UNTIMED_METADATA_DEFAULT_SEND_INTERVAL (5000000)

/**
 * Default NAL unit FIFO size
 */
#define ARSTREAM2_STREAM_SENDER_DEFAULT_NALU_FIFO_SIZE         (1024)

/**
 * Default minimum packet FIFO size
 */
#define ARSTREAM2_STREAM_SENDER_DEFAULT_MIN_PACKET_FIFO_BUFFER_COUNT (100)
#define ARSTREAM2_STREAM_SENDER_DEFAULT_PACKET_FIFO_BUFFER_TO_ITEM_FACTOR (1)
#define ARSTREAM2_STREAM_SENDER_DEFAULT_MIN_PACKET_FIFO_ITEM_COUNT (ARSTREAM2_STREAM_SENDER_DEFAULT_MIN_PACKET_FIFO_BUFFER_COUNT * ARSTREAM2_STREAM_SENDER_DEFAULT_PACKET_FIFO_BUFFER_TO_ITEM_FACTOR)

/**
 * Default stream socket send buffer size (100ms at 10 Mbit/s)
 */
#define ARSTREAM2_STREAM_SENDER_DEFAULT_STREAM_SOCKET_SEND_BUFFER_SIZE (10000000 * 100 / 1000 / 8)


typedef struct ARSTREAM2_StreamSender_s
{
    ARSTREAM2_RtpSender_t *sender;
    ARSTREAM2_StreamSender_RtpStatsCallback_t rtpStatsCallback;
    void *rtpStatsCallbackUserPtr;
    ARSTREAM2_StreamSender_VideoStatsCallback_t videoStatsCallback;
    void *videoStatsCallbackUserPtr;
    ARSTREAM2_StreamStats_VideoStats_t videoStatsForCb;
    int streamSocketSendBufferSize;
    int maxBitrate;
    uint32_t maxPacketSize;
    uint32_t targetPacketSize;
    uint32_t maxLatencyUs;
    uint32_t maxNetworkLatencyUs[ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS];

    /* NALU and packet FIFO */
    int naluFifoSize;
    ARSTREAM2_H264_NaluFifo_t naluFifo;
    ARSTREAM2_RTP_PacketFifo_t packetFifo;
    ARSTREAM2_RTP_PacketFifoQueue_t packetFifoQueue;

    /* Thread status */
    ARSAL_Mutex_t threadMutex;
    int threadStarted;
    int threadShouldStop;
    int signalPipe[2];

    /* Debug files */
    char *friendlyName;
    char *dateAndTime;
    char *debugPath;
    ARSTREAM2_StreamStats_RtpStatsContext_t rtpStatsCtx;
    ARSTREAM2_StreamStats_RtpLossContext_t rtpLossCtx;
    int8_t lastKnownRssi;
    ARSTREAM2_StreamStats_VideoStatsContext_t videoStatsCtx;
    int videoStatsInitPending;

} ARSTREAM2_StreamSender_t;


static void ARSTREAM2_StreamSender_RtpStatsCallback(const ARSTREAM2_RTP_RtpStats_t *rtpStats, void *userPtr);
static void ARSTREAM2_StreamSender_VideoStatsCallback(const ARSTREAM2_H264_VideoStats_t *videoStats, void *userPtr);


eARSTREAM2_ERROR ARSTREAM2_StreamSender_Init(ARSTREAM2_StreamSender_Handle *streamSenderHandle,
                                             const ARSTREAM2_StreamSender_Config_t *config)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    int threadMutexWasInit = 0, packetFifoWasCreated = 0, naluFifoWasCreated = 0;
    ARSTREAM2_StreamSender_t *streamSender = NULL;

    if (!streamSenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid pointer for handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!config)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid pointer for config");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    streamSender = (ARSTREAM2_StreamSender_t*)malloc(sizeof(*streamSender));
    if (!streamSender)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Allocation failed (size %zu)", sizeof(*streamSender));
        ret = ARSTREAM2_ERROR_ALLOC;
    }

    if (ret == ARSTREAM2_OK)
    {
        memset(streamSender, 0, sizeof(*streamSender));
        streamSender->signalPipe[0] = -1;
        streamSender->signalPipe[1] = -1;
        streamSender->rtpStatsCallback = config->rtpStatsCallback;
        streamSender->rtpStatsCallbackUserPtr = config->rtpStatsCallbackUserPtr;
        streamSender->videoStatsCallback = config->videoStatsCallback;
        streamSender->videoStatsCallbackUserPtr = config->videoStatsCallbackUserPtr;
        streamSender->naluFifoSize = (config->naluFifoSize > 0) ? config->naluFifoSize : ARSTREAM2_STREAM_SENDER_DEFAULT_NALU_FIFO_SIZE;
        streamSender->maxPacketSize = (config->maxPacketSize > (int)ARSTREAM2_RTP_TOTAL_HEADERS_SIZE) ? (uint32_t)config->maxPacketSize - ARSTREAM2_RTP_TOTAL_HEADERS_SIZE : ARSTREAM2_RTP_MAX_PAYLOAD_SIZE;
        streamSender->targetPacketSize = (config->targetPacketSize > (int)ARSTREAM2_RTP_TOTAL_HEADERS_SIZE)
                ? (uint32_t)config->targetPacketSize - ARSTREAM2_RTP_TOTAL_HEADERS_SIZE
                : ((config->targetPacketSize) ? streamSender->maxPacketSize : 0);
        streamSender->maxBitrate = (config->maxBitrate > 0) ? config->maxBitrate : 0;

        if (config->streamSocketBufferSize > 0)
        {
            streamSender->streamSocketSendBufferSize = config->streamSocketBufferSize;
        }
        else
        {
            int totalBufSize = 0;
            if (config->maxNetworkLatencyMs[0] > 0)
            {
                totalBufSize = streamSender->maxBitrate * config->maxNetworkLatencyMs[0] / 1000 / 8;
            }
            else if (config->maxLatencyMs > 0)
            {
                totalBufSize = streamSender->maxBitrate * config->maxLatencyMs / 1000 / 8;
            }
            int minStreamSocketSendBufferSize = (streamSender->maxBitrate > 0) ? streamSender->maxBitrate * 50 / 1000 / 8 : ARSTREAM2_STREAM_SENDER_DEFAULT_STREAM_SOCKET_SEND_BUFFER_SIZE;
            streamSender->streamSocketSendBufferSize = (totalBufSize / 4 > minStreamSocketSendBufferSize) ? totalBufSize / 4 : minStreamSocketSendBufferSize;
        }

        streamSender->maxLatencyUs = (config->maxLatencyMs > 0) ? config->maxLatencyMs * 1000 - ((streamSender->maxBitrate > 0) ? (int)((uint64_t)streamSender->streamSocketSendBufferSize * 8 * 1000000 / streamSender->maxBitrate) : 0) : 0;
        int i;
        for (i = 0; i < ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS; i++)
        {
            streamSender->maxNetworkLatencyUs[i] = (config->maxNetworkLatencyMs[i] > 0) ? config->maxNetworkLatencyMs[i] * 1000 - ((streamSender->maxBitrate > 0) ? (int)((uint64_t)streamSender->streamSocketSendBufferSize * 8 * 1000000 / streamSender->maxBitrate) : 0) : 0;
        }

        if ((config->debugPath) && (strlen(config->debugPath)))
        {
            streamSender->debugPath = strdup(config->debugPath);
        }
        if ((config->friendlyName) && (strlen(config->friendlyName)))
        {
            streamSender->friendlyName = strndup(config->friendlyName, 40);
        }
        else if ((config->canonicalName) && (strlen(config->canonicalName)))
        {
            streamSender->friendlyName = strndup(config->canonicalName, 40);
        }
        char szDate[200];
        time_t rawtime;
        struct tm timeinfo;
        time(&rawtime);
        localtime_r(&rawtime, &timeinfo);
        /* Date format : <YYYY-MM-DDTHHMMSS+HHMM */
        strftime(szDate, 200, "%FT%H%M%S%z", &timeinfo);
        streamSender->dateAndTime = strndup(szDate, 200);
        streamSender->videoStatsInitPending = 1;
        ARSTREAM2_StreamStats_RtpStatsFileOpen(&streamSender->rtpStatsCtx, streamSender->debugPath,
                                               streamSender->friendlyName, streamSender->dateAndTime);
        ARSTREAM2_StreamStats_RtpLossFileOpen(&streamSender->rtpLossCtx, streamSender->debugPath,
                                              streamSender->friendlyName, streamSender->dateAndTime);
    }

    /* Setup the NAL unit FIFO */
    if (ret == ARSTREAM2_OK)
    {
        int naluFifoRet = ARSTREAM2_H264_NaluFifoInit(&streamSender->naluFifo, streamSender->naluFifoSize);
        if (naluFifoRet != 0)
        {
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            naluFifoWasCreated = 1;
        }
    }

    /* Setup the packet FIFO */
    if (ret == ARSTREAM2_OK)
    {
        int packetFifoBufferCount = ((streamSender->maxBitrate > 0) && (streamSender->maxNetworkLatencyUs[0] > 0) && (streamSender->targetPacketSize > 0))
                ? (int)((uint64_t)streamSender->maxBitrate * streamSender->maxNetworkLatencyUs[0] * 5 / streamSender->targetPacketSize / 8 / 1000000)
                : streamSender->naluFifoSize;
        if (packetFifoBufferCount < ARSTREAM2_STREAM_SENDER_DEFAULT_MIN_PACKET_FIFO_BUFFER_COUNT)
        {
            packetFifoBufferCount = ARSTREAM2_STREAM_SENDER_DEFAULT_MIN_PACKET_FIFO_BUFFER_COUNT;
        }
        int packetFifoItemCount = packetFifoBufferCount * ARSTREAM2_STREAM_SENDER_DEFAULT_PACKET_FIFO_BUFFER_TO_ITEM_FACTOR;
        if (packetFifoItemCount < ARSTREAM2_STREAM_SENDER_DEFAULT_MIN_PACKET_FIFO_ITEM_COUNT)
        {
            packetFifoItemCount = ARSTREAM2_STREAM_SENDER_DEFAULT_MIN_PACKET_FIFO_ITEM_COUNT;
        }
        int packetFifoRet = ARSTREAM2_RTP_PacketFifoInit(&streamSender->packetFifo, packetFifoItemCount, packetFifoBufferCount, streamSender->maxPacketSize);
        if (packetFifoRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "ARSTREAM2_RTP_PacketFifoAddQueue() failed (%d)", packetFifoRet);
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            packetFifoRet = ARSTREAM2_RTP_PacketFifoAddQueue(&streamSender->packetFifo, &streamSender->packetFifoQueue);
            if (packetFifoRet != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "ARSTREAM2_RTP_PacketFifoAddQueue() failed (%d)", packetFifoRet);
                ret = ARSTREAM2_ERROR_ALLOC;
            }
            packetFifoWasCreated = 1;
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
        senderConfig.auCallback = config->auCallback;
        senderConfig.auCallbackUserPtr = config->auCallbackUserPtr;
        senderConfig.naluCallback = config->naluCallback;
        senderConfig.naluCallbackUserPtr = config->naluCallbackUserPtr;
        senderConfig.rtpStatsCallback = ARSTREAM2_StreamSender_RtpStatsCallback;
        senderConfig.rtpStatsCallbackUserPtr = streamSender;
        senderConfig.videoStatsCallback = ARSTREAM2_StreamSender_VideoStatsCallback;
        senderConfig.videoStatsCallbackUserPtr = streamSender;
        senderConfig.disconnectionCallback = config->disconnectionCallback;
        senderConfig.disconnectionCallbackUserPtr = config->disconnectionCallbackUserPtr;
        senderConfig.naluFifo = &streamSender->naluFifo;
        senderConfig.packetFifo = &streamSender->packetFifo;
        senderConfig.packetFifoQueue = &streamSender->packetFifoQueue;
        senderConfig.maxPacketSize = streamSender->maxPacketSize;
        senderConfig.targetPacketSize = streamSender->targetPacketSize;
        senderConfig.streamSocketSendBufferSize = streamSender->streamSocketSendBufferSize;
        senderConfig.maxBitrate = streamSender->maxBitrate;
        senderConfig.useRtpHeaderExtensions = config->useRtpHeaderExtensions;
        senderConfig.debugPath = streamSender->debugPath;
        senderConfig.dateAndTime = streamSender->dateAndTime;

        streamSender->sender = ARSTREAM2_RtpSender_New(&senderConfig, &ret);
        if (ret != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Error while creating sender : %s", ARSTREAM2_Error_ToString(ret));
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        if (pipe(streamSender->signalPipe) != 0)
        {
            ret = ARSTREAM2_ERROR_RESOURCE_UNAVAILABLE;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init(&(streamSender->threadMutex));
        if (mutexInitRet != 0)
        {
            ret = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            threadMutexWasInit = 1;
        }
    }

    if (ret == ARSTREAM2_OK)
    {
        *streamSenderHandle = streamSender;
    }
    else
    {
        if (streamSender)
        {
            int err;
            if (streamSender->signalPipe[0] != -1)
            {
                while (((err = close(streamSender->signalPipe[0])) == -1) && (errno == EINTR));
                streamSender->signalPipe[0] = -1;
            }
            if (streamSender->signalPipe[1] != -1)
            {
                while (((err = close(streamSender->signalPipe[1])) == -1) && (errno == EINTR));
                streamSender->signalPipe[1] = -1;
            }
            if (threadMutexWasInit == 1) ARSAL_Mutex_Destroy(&(streamSender->threadMutex));
            if (streamSender->sender) ARSTREAM2_RtpSender_Delete(&(streamSender->sender));
            if (naluFifoWasCreated == 1) ARSTREAM2_H264_NaluFifoFree(&(streamSender->naluFifo));
            if (packetFifoWasCreated == 1) ARSTREAM2_RTP_PacketFifoFree(&(streamSender->packetFifo));
            ARSTREAM2_StreamStats_RtpStatsFileClose(&streamSender->rtpStatsCtx);
            ARSTREAM2_StreamStats_RtpLossFileClose(&streamSender->rtpLossCtx);
            free(streamSender->debugPath);
            free(streamSender->friendlyName);
            free(streamSender->dateAndTime);
            free(streamSender);
        }
        *streamSenderHandle = NULL;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamSender_Stop(ARSTREAM2_StreamSender_Handle streamSenderHandle)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)streamSenderHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if (!streamSenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    ARSAL_Mutex_Lock(&(streamSender->threadMutex));
    streamSender->threadShouldStop = 1;
    ARSAL_Mutex_Unlock(&(streamSender->threadMutex));

    /* signal the thread to avoid a deadlock */
    if (streamSender->signalPipe[1] != -1)
    {
        char * buff = "x";
        ssize_t err;
        while (((err = write(streamSender->signalPipe[1], buff, 1)) == -1) && (errno == EINTR));
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamSender_Free(ARSTREAM2_StreamSender_Handle *streamSenderHandle)
{
    ARSTREAM2_StreamSender_t* streamSender;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if ((!streamSenderHandle) || (!*streamSenderHandle))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid pointer for handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    streamSender = (ARSTREAM2_StreamSender_t*)*streamSenderHandle;

    int canDelete = 0;
    ARSAL_Mutex_Lock(&(streamSender->threadMutex));
    if (streamSender->threadStarted == 0)
    {
        canDelete = 1;
    }
    ARSAL_Mutex_Unlock(&(streamSender->threadMutex));

    if (canDelete == 1)
    {
        ret = ARSTREAM2_RtpSender_Delete(&streamSender->sender);
        if (ret != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Unable to delete sender: %s", ARSTREAM2_Error_ToString(ret));
        }

        if (streamSender->signalPipe[0] != -1)
        {
            close(streamSender->signalPipe[0]);
            streamSender->signalPipe[0] = -1;
        }
        if (streamSender->signalPipe[1] != -1)
        {
            close(streamSender->signalPipe[1]);
            streamSender->signalPipe[1] = -1;
        }
        ARSAL_Mutex_Destroy(&(streamSender->threadMutex));
        ARSTREAM2_H264_NaluFifoFree(&(streamSender->naluFifo));
        ARSTREAM2_RTP_PacketFifoFree(&(streamSender->packetFifo));
        ARSTREAM2_StreamStats_VideoStatsFileClose(&streamSender->videoStatsCtx);
        ARSTREAM2_StreamStats_RtpStatsFileClose(&streamSender->rtpStatsCtx);
        ARSTREAM2_StreamStats_RtpLossFileClose(&streamSender->rtpLossCtx);
        free(streamSender->debugPath);
        free(streamSender->friendlyName);
        free(streamSender->dateAndTime);
        free(streamSender->videoStatsForCb.erroredSecondCountByZone);
        free(streamSender->videoStatsForCb.macroblockStatus);

        free(streamSender);
        *streamSenderHandle = NULL;
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Call ARSTREAM2_StreamSender_Stop() before calling this function");
        ret = ARSTREAM2_ERROR_BUSY;
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamSender_SendNewNalu(ARSTREAM2_StreamSender_Handle streamSenderHandle,
                                                    const ARSTREAM2_StreamSender_H264NaluDesc_t *nalu,
                                                    uint64_t inputTime)
{
    return ARSTREAM2_StreamSender_SendNNewNalu(streamSenderHandle, nalu, 1, inputTime);
}


eARSTREAM2_ERROR ARSTREAM2_StreamSender_SendNNewNalu(ARSTREAM2_StreamSender_Handle streamSenderHandle,
                                                     const ARSTREAM2_StreamSender_H264NaluDesc_t *nalu,
                                                     int naluCount, uint64_t inputTime)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)streamSenderHandle;
    eARSTREAM2_ERROR retVal = ARSTREAM2_OK;
    int k, res;

    if (!streamSenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!nalu)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid pointer");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (naluCount <= 0)
    {
        retVal = ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (retVal == ARSTREAM2_OK)
    {
        for (k = 0; k < naluCount; k++)
        {
            if ((nalu[k].naluBuffer == NULL) ||
                (nalu[k].naluSize == 0) ||
                (nalu[k].auTimestamp == 0))
            {
                retVal = ARSTREAM2_ERROR_BAD_PARAMETERS;
            }
        }
    }

    if (retVal == ARSTREAM2_OK)
    {
        ARSAL_Mutex_Lock(&(streamSender->threadMutex));
        if (!streamSender->threadStarted)
        {
            retVal = ARSTREAM2_ERROR_BAD_PARAMETERS;
        }
        ARSAL_Mutex_Unlock(&(streamSender->threadMutex));
    }

    if (retVal == ARSTREAM2_OK)
    {
        for (k = 0; k < naluCount; k++)
        {
            ARSTREAM2_H264_NaluFifoItem_t *item = ARSTREAM2_H264_NaluFifoPopFreeItem(&streamSender->naluFifo);
            if (item)
            {
                ARSTREAM2_H264_NaluReset(&item->nalu);
                item->nalu.inputTimestamp = inputTime;
                item->nalu.ntpTimestamp = nalu[k].auTimestamp;
                item->nalu.isLastInAu = nalu[k].isLastNaluInAu;
                item->nalu.seqNumForcedDiscontinuity = nalu[k].seqNumForcedDiscontinuity;
                item->nalu.importance = (nalu[k].importance < ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS) ? nalu[k].importance : 0;
                item->nalu.priority = (nalu[k].priority < ARSTREAM2_STREAM_SENDER_MAX_PRIORITY_LEVELS) ? nalu[k].priority : 0;
                uint64_t timeoutTimestamp1 = (streamSender->maxLatencyUs > 0) ? nalu[k].auTimestamp + streamSender->maxLatencyUs : 0;
                uint64_t timeoutTimestamp2 = ((streamSender->maxNetworkLatencyUs[item->nalu.importance] > 0) && (inputTime > 0)) ? inputTime + streamSender->maxNetworkLatencyUs[item->nalu.importance] : 0;
                item->nalu.timeoutTimestamp = timeoutTimestamp1;
                if ((timeoutTimestamp1 == 0) || ((timeoutTimestamp2 > 0) && (timeoutTimestamp2 < timeoutTimestamp1)))
                {
                    item->nalu.timeoutTimestamp = timeoutTimestamp2;
                }
                item->nalu.metadata = nalu[k].auMetadata;
                item->nalu.metadataSize = nalu[k].auMetadataSize;
                item->nalu.nalu = nalu[k].naluBuffer;
                item->nalu.naluSize = nalu[k].naluSize;
                item->nalu.auUserPtr = nalu[k].auUserPtr;
                item->nalu.naluUserPtr = nalu[k].naluUserPtr;

                res = ARSTREAM2_H264_NaluFifoEnqueueItem(&streamSender->naluFifo, item);
                if (res != 0)
                {
                    res = ARSTREAM2_H264_NaluFifoPushFreeItem(&streamSender->naluFifo, item);
                    retVal = ARSTREAM2_ERROR_INVALID_STATE;
                    break;
                }
            }
            else
            {
                retVal = ARSTREAM2_ERROR_QUEUE_FULL;
                break;
            }
        }

        if (streamSender->signalPipe[1] != -1)
        {
            char * buff = "x";
            ssize_t err;
            while (((err = write(streamSender->signalPipe[1], buff, 1)) == -1) && (errno == EINTR));
        }
    }

    return retVal;
}


eARSTREAM2_ERROR ARSTREAM2_StreamSender_FlushNaluQueue(ARSTREAM2_StreamSender_Handle streamSenderHandle)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)streamSenderHandle;

    if (!streamSenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    return ARSTREAM2_RtpSender_FlushNaluQueue(streamSender->sender);
}


void* ARSTREAM2_StreamSender_RunThread(void *streamSenderHandle)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)streamSenderHandle;
    int shouldStop, selectRet = 0;
    fd_set readSet, writeSet, exceptSet;
    fd_set *pReadSet, *pWriteSet, *pExceptSet;
    int maxFd = 0;
    struct timeval tv;
    uint32_t nextTimeout = 0;
    eARSTREAM2_ERROR err;

    if (!streamSenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid handle");
        return (void*)NULL;
    }

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_STREAM_SENDER_TAG, "Sender thread running");
    ARSAL_Mutex_Lock(&(streamSender->threadMutex));
    streamSender->threadStarted = 1;
    shouldStop = streamSender->threadShouldStop;
    ARSAL_Mutex_Unlock(&(streamSender->threadMutex));

    FD_ZERO(&readSet);
    FD_ZERO(&writeSet);
    FD_ZERO(&exceptSet);
    pReadSet = &readSet;
    pWriteSet = &writeSet;
    pExceptSet = &exceptSet;

    err = ARSTREAM2_RtpSender_GetSelectParams(streamSender->sender, &pReadSet, &pWriteSet, &pExceptSet, &maxFd, &nextTimeout);
    if (err != ARSTREAM2_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "ARSTREAM2_RtpSender_GetSelectParams() failed (%d)", err);
        return (void *)0;
    }

    if (pReadSet)
        FD_SET(streamSender->signalPipe[0], pReadSet);
    if (pExceptSet)
        FD_SET(streamSender->signalPipe[0], pExceptSet);
    if (streamSender->signalPipe[0] > maxFd) maxFd = streamSender->signalPipe[0];
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
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Select error (%d): %s", errno, strerror(errno));
            }
        }

        err = ARSTREAM2_RtpSender_ProcessRtcp(streamSender->sender, selectRet, pReadSet, pWriteSet, pExceptSet);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "ARSTREAM2_RtpSender_ProcessRtcp() failed (%d)", err);
        }
        err = ARSTREAM2_RtpSender_ProcessRtp(streamSender->sender, selectRet, pReadSet, pWriteSet, pExceptSet);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "ARSTREAM2_RtpSender_ProcessRtp() failed (%d)", err);
        }

        if ((pReadSet) && (selectRet >= 0) && (FD_ISSET(streamSender->signalPipe[0], pReadSet)))
        {
            /* Dump bytes (so it won't be ready next time) */
            char dump[10];
            int readRet = read(streamSender->signalPipe[0], &dump, 10);
            if (readRet < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Failed to read from pipe (%d): %s", errno, strerror(errno));
            }
        }

        ARSAL_Mutex_Lock(&(streamSender->threadMutex));
        shouldStop = streamSender->threadShouldStop;
        ARSAL_Mutex_Unlock(&(streamSender->threadMutex));

        if (!shouldStop)
        {
            /* Prepare the next select */
            FD_ZERO(&readSet);
            FD_ZERO(&writeSet);
            FD_ZERO(&exceptSet);
            pReadSet = &readSet;
            pWriteSet = &writeSet;
            pExceptSet = &exceptSet;

            err = ARSTREAM2_RtpSender_GetSelectParams(streamSender->sender, &pReadSet, &pWriteSet, &pExceptSet, &maxFd, &nextTimeout);
            if (err != ARSTREAM2_OK)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "ARSTREAM2_RtpSender_GetSelectParams() failed (%d)", err);
                break;
            }

            if (pReadSet)
                FD_SET(streamSender->signalPipe[0], pReadSet);
            if (pExceptSet)
                FD_SET(streamSender->signalPipe[0], pExceptSet);
            if (streamSender->signalPipe[0] > maxFd) maxFd = streamSender->signalPipe[0];
            maxFd++;
            tv.tv_sec = 0;
            tv.tv_usec = nextTimeout;
        }
    }

    ARSAL_Mutex_Lock(&(streamSender->threadMutex));
    streamSender->threadStarted = 0;
    ARSAL_Mutex_Unlock(&(streamSender->threadMutex));

    err = ARSTREAM2_RtpSender_ProcessEnd(streamSender->sender, 0);
    if (err != ARSTREAM2_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "ARSTREAM2_RtpSender_GetSelectParams() failed (%d)", err);
    }

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_STREAM_SENDER_TAG, "Sender thread ended");

    return (void*)0;
}


eARSTREAM2_ERROR ARSTREAM2_StreamSender_GetDynamicConfig(ARSTREAM2_StreamSender_Handle streamSenderHandle,
                                                         ARSTREAM2_StreamSender_DynamicConfig_t *config)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)streamSenderHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    int i;

    if (!streamSenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!config)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid config");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    config->targetPacketSize = (streamSender->targetPacketSize > 0) ? streamSender->targetPacketSize + ARSTREAM2_RTP_TOTAL_HEADERS_SIZE : 0;
    config->streamSocketBufferSize = streamSender->streamSocketSendBufferSize;
    config->maxBitrate = streamSender->maxBitrate;
    if (streamSender->maxLatencyUs > 0)
    {
        config->maxLatencyMs = (streamSender->maxLatencyUs + ((streamSender->maxBitrate > 0) ? (int)((uint64_t)streamSender->streamSocketSendBufferSize * 8 * 1000000 / streamSender->maxBitrate) : 0)) / 1000;
    }
    else
    {
        config->maxLatencyMs = 0;
    }
    for (i = 0; i < ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS; i++)
    {
        if (streamSender->maxNetworkLatencyUs[i] > 0)
        {
            config->maxNetworkLatencyMs[i] = (streamSender->maxNetworkLatencyUs[i] + ((streamSender->maxBitrate > 0) ? (int)((uint64_t)streamSender->streamSocketSendBufferSize * 8 * 1000000 / streamSender->maxBitrate) : 0)) / 1000;
        }
        else
        {
            config->maxNetworkLatencyMs[i] = 0;
        }
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamSender_SetDynamicConfig(ARSTREAM2_StreamSender_Handle streamSenderHandle,
                                                         const ARSTREAM2_StreamSender_DynamicConfig_t *config)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)streamSenderHandle;
    int i;

    if (!streamSenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!config)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid config");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    streamSender->targetPacketSize = (config->targetPacketSize > (int)ARSTREAM2_RTP_TOTAL_HEADERS_SIZE)
            ? (uint32_t)config->targetPacketSize - ARSTREAM2_RTP_TOTAL_HEADERS_SIZE
            : ((config->targetPacketSize) ? streamSender->maxPacketSize : 0);
    streamSender->maxBitrate = (config->maxBitrate > 0) ? config->maxBitrate : 0;
    if (config->streamSocketBufferSize > 0)
    {
        streamSender->streamSocketSendBufferSize = config->streamSocketBufferSize;
    }
    else
    {
        int totalBufSize = 0;
        if (config->maxNetworkLatencyMs[0] > 0)
        {
            totalBufSize = streamSender->maxBitrate * config->maxNetworkLatencyMs[0] / 1000 / 8;
        }
        else if (config->maxLatencyMs > 0)
        {
            totalBufSize = streamSender->maxBitrate * config->maxLatencyMs / 1000 / 8;
        }
        int minStreamSocketSendBufferSize = (streamSender->maxBitrate > 0) ? streamSender->maxBitrate * 50 / 1000 / 8 : ARSTREAM2_STREAM_SENDER_DEFAULT_STREAM_SOCKET_SEND_BUFFER_SIZE;
        streamSender->streamSocketSendBufferSize = (totalBufSize / 4 > minStreamSocketSendBufferSize) ? totalBufSize / 4 : minStreamSocketSendBufferSize;
    }
    streamSender->maxLatencyUs = (config->maxLatencyMs > 0) ? config->maxLatencyMs * 1000 - ((streamSender->maxBitrate > 0) ? (int)((uint64_t)streamSender->streamSocketSendBufferSize * 8 * 1000000 / streamSender->maxBitrate) : 0) : 0;
    for (i = 0; i < ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS; i++)
    {
        streamSender->maxNetworkLatencyUs[i] = (config->maxNetworkLatencyMs[i] > 0) ? config->maxNetworkLatencyMs[i] * 1000 - ((streamSender->maxBitrate > 0) ? (int)((uint64_t)streamSender->streamSocketSendBufferSize * 8 * 1000000 / streamSender->maxBitrate) : 0) : 0;
    }

    ARSTREAM2_RtpSender_DynamicConfig_t senderConfig;
    memset(&senderConfig, 0, sizeof(senderConfig));
    senderConfig.targetPacketSize = streamSender->targetPacketSize;
    senderConfig.streamSocketSendBufferSize = streamSender->streamSocketSendBufferSize;
    senderConfig.maxBitrate = streamSender->maxBitrate;

    return ARSTREAM2_RtpSender_SetDynamicConfig(streamSender->sender, &senderConfig);
}


eARSTREAM2_ERROR ARSTREAM2_StreamSender_GetUntimedMetadata(ARSTREAM2_StreamSender_Handle streamSenderHandle,
                                                           ARSTREAM2_Stream_UntimedMetadata_t *metadata, uint32_t *sendInterval)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)streamSenderHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK, _ret;
    uint32_t _sendInterval = 0, minSendInterval = (uint32_t)(-1);
    char *ptr;
    int i;

    if (!streamSenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!metadata)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid metadata");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_CNAME_ITEM, NULL, &metadata->serialNumber, &_sendInterval);
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

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_NAME_ITEM, NULL, &metadata->friendlyName, &_sendInterval);
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

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_TOOL_ITEM, NULL, &metadata->softwareVersion, &_sendInterval);
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
    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_LOC_ITEM, NULL, &ptr, &_sendInterval);
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
    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_hfov", &ptr, &_sendInterval);
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
    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_vfov", &ptr, &_sendInterval);
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

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_date", &metadata->runDate, &_sendInterval);
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

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_id", &metadata->runUuid, &_sendInterval);
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

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "maker", &metadata->maker, &_sendInterval);
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

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model", &metadata->model, &_sendInterval);
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

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model_id", &metadata->modelId, &_sendInterval);
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

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "build_id", &metadata->buildId, &_sendInterval);
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

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "title", &metadata->title, &_sendInterval);
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

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "comment", &metadata->comment, &_sendInterval);
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

    _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "copyright", &metadata->copyright, &_sendInterval);
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
            _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, metadata->custom[i].key, &metadata->custom[i].value, &_sendInterval);
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


eARSTREAM2_ERROR ARSTREAM2_StreamSender_SetUntimedMetadata(ARSTREAM2_StreamSender_Handle streamSenderHandle,
                                                           const ARSTREAM2_Stream_UntimedMetadata_t *metadata, uint32_t sendInterval)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)streamSenderHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK, _ret;
    char *ptr;
    int i;

    if (!streamSenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!metadata)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid metadata");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (sendInterval == 0)
    {
        sendInterval = ARSTREAM2_STREAM_SENDER_UNTIMED_METADATA_DEFAULT_SEND_INTERVAL;
    }

    if ((metadata->serialNumber) && (strlen(metadata->serialNumber)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_CNAME_ITEM, NULL, &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->serialNumber, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_CNAME_ITEM, NULL, metadata->serialNumber, sendInterval);
        }
    }

    if ((metadata->friendlyName) && (strlen(metadata->friendlyName)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_NAME_ITEM, NULL, &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->friendlyName, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_NAME_ITEM, NULL, metadata->friendlyName, sendInterval);
        }
    }

    if ((metadata->softwareVersion) && (strlen(metadata->softwareVersion)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_TOOL_ITEM, NULL, &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->softwareVersion, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_TOOL_ITEM, NULL, metadata->softwareVersion, sendInterval);
        }
    }

    if ((metadata->takeoffLatitude != 500.) && (metadata->takeoffLongitude != 500.))
    {
        double takeoffLatitude = 500.;
        double takeoffLongitude = 500.;
        float takeoffAltitude = 0.;
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_LOC_ITEM, NULL, &ptr, NULL);
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
            snprintf(str, sizeof(str), "%.8f,%.8f,%.8f", metadata->takeoffLatitude, metadata->takeoffLongitude, metadata->takeoffAltitude);
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_LOC_ITEM, NULL, str, sendInterval);
        }
    }

    if (metadata->pictureHFov != 0.)
    {
        float pictureHFov = 0.;
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_hfov", &ptr, NULL);
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
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_hfov", str, sendInterval);
        }
    }

    if (metadata->pictureVFov != 0.)
    {
        float pictureVFov = 0.;
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_vfov", &ptr, NULL);
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
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_vfov", str, sendInterval);
        }
    }

    if ((metadata->runDate) && (strlen(metadata->runDate)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_date", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->runDate, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_date", metadata->runDate, sendInterval);
        }
    }

    if ((metadata->runUuid) && (strlen(metadata->runUuid)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_id", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->runUuid, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_id", metadata->runUuid, sendInterval);
        }
    }

    if ((metadata->maker) && (strlen(metadata->maker)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "maker", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->maker, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "maker", metadata->maker, sendInterval);
        }
    }

    if ((metadata->model) && (strlen(metadata->model)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->model, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model", metadata->model, sendInterval);
        }
    }

    if ((metadata->modelId) && (strlen(metadata->modelId)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model_id", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->modelId, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model_id", metadata->modelId, sendInterval);
        }
    }

    if ((metadata->buildId) && (strlen(metadata->buildId)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "build_id", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->buildId, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "build_id", metadata->buildId, sendInterval);
        }
    }

    if ((metadata->title) && (strlen(metadata->title)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "title", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->title, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "title", metadata->title, sendInterval);
        }
    }

    if ((metadata->comment) && (strlen(metadata->comment)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "comment", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->comment, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "comment", metadata->comment, sendInterval);
        }
    }

    if ((metadata->copyright) && (strlen(metadata->copyright)))
    {
        ptr = NULL;
        _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "copyright", &ptr, NULL);
        if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->copyright, 256)))
        {
            ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "copyright", metadata->copyright, sendInterval);
        }
    }

    for (i = 0; i < ARSTREAM2_STREAM_UNTIMEDMETADATA_CUSTOM_MAX_COUNT; i++)
    {
        if ((metadata->custom[i].key) && (strlen(metadata->custom[i].key)) && (metadata->custom[i].value) && (strlen(metadata->custom[i].value)))
        {
            ptr = NULL;
            _ret = ARSTREAM2_RtpSender_GetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, metadata->custom[i].key, &ptr, NULL);
            if ((_ret != ARSTREAM2_OK) || (strncmp(ptr, metadata->custom[i].value, 256)))
            {
                ARSTREAM2_RtpSender_SetSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, metadata->custom[i].key, metadata->custom[i].value, sendInterval);
            }
        }
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamSender_GetPeerUntimedMetadata(ARSTREAM2_StreamSender_Handle streamSenderHandle,
                                                               ARSTREAM2_Stream_UntimedMetadata_t *metadata)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)streamSenderHandle;
    eARSTREAM2_ERROR ret = ARSTREAM2_OK, _ret;
    char *ptr;
    int i;

    if (!streamSenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!metadata)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid metadata");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_CNAME_ITEM, NULL, &metadata->serialNumber);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->serialNumber = NULL;
    }

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_NAME_ITEM, NULL, &metadata->friendlyName);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->friendlyName = NULL;
    }

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_TOOL_ITEM, NULL, &metadata->softwareVersion);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->softwareVersion = NULL;
    }

    ptr = NULL;
    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_LOC_ITEM, NULL, &ptr);
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
    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_hfov", &ptr);
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
    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "picture_vfov", &ptr);
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

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_date", &metadata->runDate);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->runDate = NULL;
    }

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "run_id", &metadata->runUuid);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->runUuid = NULL;
    }

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "maker", &metadata->maker);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->maker = NULL;
    }

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model", &metadata->model);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->model = NULL;
    }

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "model_id", &metadata->modelId);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->modelId = NULL;
    }

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "build_id", &metadata->buildId);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->buildId = NULL;
    }

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "title", &metadata->title);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->title = NULL;
    }

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "comment", &metadata->comment);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->comment = NULL;
    }

    _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, "copyright", &metadata->copyright);
    if (_ret != ARSTREAM2_OK)
    {
        metadata->copyright = NULL;
    }

    for (i = 0; i < ARSTREAM2_STREAM_UNTIMEDMETADATA_CUSTOM_MAX_COUNT; i++)
    {
        if ((metadata->custom[i].key) && (strlen(metadata->custom[i].key)))
        {
            _ret = ARSTREAM2_RtpSender_GetPeerSdesItem(streamSender->sender, ARSTREAM2_RTCP_SDES_PRIV_ITEM, metadata->custom[i].key, &metadata->custom[i].value);
            if (_ret != ARSTREAM2_OK)
            {
                metadata->custom[i].value = NULL;
            }
        }
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_StreamSender_GetMonitoring(ARSTREAM2_StreamSender_Handle streamSenderHandle,
                                                      uint64_t startTime, uint32_t timeIntervalUs,
                                                      ARSTREAM2_StreamSender_MonitoringData_t *monitoringData)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)streamSenderHandle;

    if (!streamSenderHandle)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid handle");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if (!monitoringData)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_STREAM_SENDER_TAG, "Invalid pointer");
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    return ARSTREAM2_RtpSender_GetMonitoring(streamSender->sender, startTime, timeIntervalUs, monitoringData);
}


static void ARSTREAM2_StreamSender_RtpStatsCallback(const ARSTREAM2_RTP_RtpStats_t *rtpStats, void *userPtr)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)userPtr;

    if (!streamSender)
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
        s.rssi = streamSender->lastKnownRssi;
        ARSTREAM2_StreamStats_RtpStatsFileWrite(&streamSender->rtpStatsCtx, &s, curTime);
        ARSTREAM2_StreamStats_RtpLossFileWrite(&streamSender->rtpLossCtx, &s);

        if (streamSender->rtpStatsCallback)
        {
            ARSTREAM2_StreamStats_RtpStats_t rtpsOut;
            memset(&rtpsOut, 0, sizeof(ARSTREAM2_StreamStats_RtpStats_t));

            /* Map the RTP stats */
            rtpsOut.rssi = streamSender->lastKnownRssi;
            rtpsOut.senderStats.timestamp = rtpStats->senderStats.timestamp;
            rtpsOut.senderStats.sentPacketCount = rtpStats->senderStats.sentPacketCount;
            rtpsOut.senderStats.droppedPacketCount = rtpStats->senderStats.droppedPacketCount;
            rtpsOut.senderStats.sentByteIntegral = rtpStats->senderStats.sentByteIntegral;
            rtpsOut.senderStats.sentByteIntegralSq = rtpStats->senderStats.sentByteIntegralSq;
            rtpsOut.senderStats.droppedByteIntegral = rtpStats->senderStats.droppedByteIntegral;
            rtpsOut.senderStats.droppedByteIntegralSq = rtpStats->senderStats.droppedByteIntegralSq;
            rtpsOut.senderStats.inputToSentTimeIntegral = rtpStats->senderStats.inputToSentTimeIntegral;
            rtpsOut.senderStats.inputToSentTimeIntegralSq = rtpStats->senderStats.inputToSentTimeIntegralSq;
            rtpsOut.senderStats.inputToDroppedTimeIntegral = rtpStats->senderStats.inputToDroppedTimeIntegral;
            rtpsOut.senderStats.inputToDroppedTimeIntegralSq = rtpStats->senderStats.inputToDroppedTimeIntegralSq;
            rtpsOut.senderReport.timestamp = rtpStats->senderReport.timestamp;
            rtpsOut.senderReport.lastInterval = rtpStats->senderReport.lastInterval;
            rtpsOut.senderReport.intervalPacketCount = rtpStats->senderReport.intervalPacketCount;
            rtpsOut.senderReport.intervalByteCount = rtpStats->senderReport.intervalByteCount;
            rtpsOut.receiverReport.timestamp = rtpStats->receiverReport.timestamp;
            rtpsOut.receiverReport.roundTripDelay = rtpStats->receiverReport.roundTripDelay;
            rtpsOut.receiverReport.interarrivalJitter = rtpStats->receiverReport.interarrivalJitter;
            rtpsOut.receiverReport.receiverLostCount = rtpStats->receiverReport.receiverLostCount;
            rtpsOut.receiverReport.receiverFractionLost = rtpStats->receiverReport.receiverFractionLost;
            rtpsOut.receiverReport.receiverExtHighestSeqNum = rtpStats->receiverReport.receiverExtHighestSeqNum;
            rtpsOut.lossReport.timestamp = rtpStats->lossReport.timestamp;
            rtpsOut.lossReport.startSeqNum = rtpStats->lossReport.startSeqNum;
            rtpsOut.lossReport.endSeqNum = rtpStats->lossReport.endSeqNum;
            rtpsOut.lossReport.receivedFlag = rtpStats->lossReport.receivedFlag;
            rtpsOut.djbMetricsReport.timestamp = rtpStats->djbMetricsReport.timestamp;
            rtpsOut.djbMetricsReport.djbNominal = rtpStats->djbMetricsReport.djbNominal;
            rtpsOut.djbMetricsReport.djbMax = rtpStats->djbMetricsReport.djbMax;
            rtpsOut.djbMetricsReport.djbHighWatermark = rtpStats->djbMetricsReport.djbHighWatermark;
            rtpsOut.djbMetricsReport.djbLowWatermark = rtpStats->djbMetricsReport.djbLowWatermark;
            rtpsOut.clockDelta.peerClockDelta = rtpStats->clockDelta.peerClockDelta;
            rtpsOut.clockDelta.roundTripDelay = rtpStats->clockDelta.roundTripDelay;
            rtpsOut.clockDelta.peer2meDelay = rtpStats->clockDelta.peer2meDelay;
            rtpsOut.clockDelta.me2peerDelay = rtpStats->clockDelta.me2peerDelay;

            /* Call the receiver report callback function */
            streamSender->rtpStatsCallback(&rtpsOut, streamSender->rtpStatsCallbackUserPtr);
        }
    }
}


static void ARSTREAM2_StreamSender_VideoStatsCallback(const ARSTREAM2_H264_VideoStats_t *videoStats, void *userPtr)
{
    ARSTREAM2_StreamSender_t *streamSender = (ARSTREAM2_StreamSender_t*)userPtr;

    if (!streamSender)
    {
        return;
    }

    if (videoStats)
    {
        if (streamSender->videoStatsInitPending)
        {
            ARSTREAM2_StreamStats_VideoStatsFileOpen(&streamSender->videoStatsCtx, streamSender->debugPath, streamSender->friendlyName,
                                                     streamSender->dateAndTime, videoStats->mbStatusZoneCount, videoStats->mbStatusClassCount);
            streamSender->videoStatsInitPending = 0;
        }
        streamSender->lastKnownRssi = videoStats->rssi;

        ARSTREAM2_StreamStats_VideoStatsFileWrite(&streamSender->videoStatsCtx, videoStats);

        if (streamSender->videoStatsCallback)
        {
            /* Map the video stats */
            ARSTREAM2_StreamStats_VideoStats_t *vsOut = &streamSender->videoStatsForCb;
            uint32_t i, j;

            vsOut->timestamp = videoStats->timestamp;
            vsOut->rssi = videoStats->rssi;
            vsOut->totalFrameCount = videoStats->totalFrameCount;
            vsOut->outputFrameCount = videoStats->outputFrameCount;
            vsOut->erroredOutputFrameCount = videoStats->erroredOutputFrameCount;
            vsOut->missedFrameCount = videoStats->missedFrameCount;
            vsOut->discardedFrameCount = videoStats->discardedFrameCount;
            vsOut->timestampDeltaIntegral = videoStats->timestampDeltaIntegral;
            vsOut->timestampDeltaIntegralSq = videoStats->timestampDeltaIntegralSq;
            vsOut->timingErrorIntegral = videoStats->timingErrorIntegral;
            vsOut->timingErrorIntegralSq = videoStats->timingErrorIntegralSq;
            vsOut->estimatedLatencyIntegral = videoStats->estimatedLatencyIntegral;
            vsOut->estimatedLatencyIntegralSq = videoStats->estimatedLatencyIntegralSq;
            vsOut->erroredSecondCount = videoStats->erroredSecondCount;
            if (videoStats->mbStatusZoneCount)
            {
                if ((!vsOut->erroredSecondCountByZone) || (videoStats->mbStatusZoneCount > vsOut->mbStatusZoneCount))
                {
                    vsOut->erroredSecondCountByZone = realloc(vsOut->erroredSecondCountByZone, videoStats->mbStatusZoneCount * sizeof(uint32_t));
                    if (!vsOut->erroredSecondCountByZone)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_STREAM_SENDER_TAG, "Allocation failed");
                    }
                    vsOut->mbStatusZoneCount = videoStats->mbStatusZoneCount;
                }
                if (vsOut->erroredSecondCountByZone)
                {
                    for (i = 0; i < vsOut->mbStatusZoneCount; i++)
                    {
                        vsOut->erroredSecondCountByZone[i] = videoStats->erroredSecondCountByZone[i];
                    }
                }

                if (videoStats->mbStatusClassCount)
                {
                    if ((!vsOut->macroblockStatus) || (videoStats->mbStatusClassCount > vsOut->mbStatusClassCount))
                    {
                        vsOut->macroblockStatus = realloc(vsOut->macroblockStatus, videoStats->mbStatusClassCount * vsOut->mbStatusZoneCount * sizeof(uint32_t));
                        if (!vsOut->macroblockStatus)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_STREAM_SENDER_TAG, "Allocation failed");
                        }
                        vsOut->mbStatusClassCount = videoStats->mbStatusClassCount;
                    }
                    if (vsOut->macroblockStatus)
                    {
                        for (j = 0; j < vsOut->mbStatusClassCount; j++)
                        {
                            for (i = 0; i < vsOut->mbStatusZoneCount; i++)
                            {
                                vsOut->macroblockStatus[j * vsOut->mbStatusZoneCount + i] = videoStats->macroblockStatus[j][i];
                            }
                        }
                    }
                }
            }

            /* Call the video stats reception callback function */
            streamSender->videoStatsCallback(vsOut, streamSender->videoStatsCallbackUserPtr);
        }
    }
}
