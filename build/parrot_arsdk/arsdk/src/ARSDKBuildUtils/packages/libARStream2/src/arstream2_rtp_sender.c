/**
 * @file arstream2_rtp_sender.c
 * @brief Parrot Streaming Library - RTP Sender
 * @date 04/17/2015
 * @author aurelien.barre@parrot.com
 */

#include <config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#define __USE_GNU
#include <sys/socket.h>
#undef __USE_GNU
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <fcntl.h>
#include <math.h>

#include "arstream2_rtp_sender.h"
#include "arstream2_rtp.h"
#include "arstream2_rtp_h264.h"
#include "arstream2_rtcp.h"

#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Mutex.h>


//#define ARSTREAM2_RTP_SENDER_RANDOM_DROP
#define ARSTREAM2_RTP_SENDER_RANDOM_DROP_RATIO 0.01
//#define ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION
#define ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION_WAIT_MIN 0
#define ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION_WAIT_MAX 60000
#define ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION_MSG_MIN 0.8
#define ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION_MSG_MAX 1.0

/**
 * Tag for ARSAL_PRINT
 */
#define ARSTREAM2_RTP_SENDER_TAG "ARSTREAM2_RtpSender"


/**
 * Timeout value (microseconds)
 */
#define ARSTREAM2_RTP_SENDER_TIMEOUT_US (100 * 1000)


/**
 * Maximum number of elements for the monitoring
 */
#define ARSTREAM2_RTP_SENDER_MONITORING_MAX_POINTS (2048)


/**
 * Timeout drops minimum log interval in seconds
 */
#define ARSTREAM2_RTP_SENDER_TIMEOUT_DROP_LOG_INTERVAL (10)


/**
 * RTCP drops minimum log interval in seconds
 */
#define ARSTREAM2_RTP_SENDER_RTCP_DROP_LOG_INTERVAL (10)


/**
 * Default minimum stream socket send buffer size: 50ms @ 5Mbit/s
 */
#define ARSTREAM2_RTP_SENDER_DEFAULT_MIN_STREAM_SOCKET_SEND_BUFFER_SIZE (31250)


/**
 * Maximum interval between calls to the RTPStats callback function in microseconds
 * (this is useful when the callback is needed but the receiver doesn't send RTCP packets)
 */
#define ARSTREAM2_RTP_SENDER_RTPSTATS_CALLBACK_MAX_INTERVAL (120000)


/**
 * Sets *PTR to VAL if PTR is not null
 */
#define SET_WITH_CHECK(PTR,VAL)                 \
    do                                          \
    {                                           \
        if (PTR != NULL)                        \
        {                                       \
            *PTR = VAL;                         \
        }                                       \
    } while (0)


#define ARSTREAM2_RTP_SENDER_MONITORING_OUTPUT_PATH "rtpmonitor"
#define ARSTREAM2_RTP_SENDER_MONITORING_OUTPUT_FILENAME "sender_rtp_monitor"
#define ARSTREAM2_RTP_SENDER_MONITORING_OUTPUT_FILEEXT "dat"


typedef struct ARSTREAM2_RtpSender_MonitoringPoint_s {
    uint64_t inputTimestamp;
    uint64_t outputTimestamp;
    uint64_t ntpTimestamp;
    uint32_t rtpTimestamp;
    uint16_t seqNum;
    uint16_t markerBit;
    uint32_t importance;
    uint32_t priority;
    uint32_t bytesSent;
    uint32_t bytesDropped;
} ARSTREAM2_RtpSender_MonitoringPoint_t;


struct ARSTREAM2_RtpSender_t {
    /* Configuration on New */
    char *canonicalName;
    char *friendlyName;
    char *applicationName;
    char *clientAddr;
    char *mcastIfaceAddr;
    int serverStreamPort;
    int serverControlPort;
    int clientStreamPort;
    int clientControlPort;
    int classSelector;
    ARSTREAM2_RtpSender_RtpStatsCallback_t rtpStatsCallback;
    void *rtpStatsCallbackUserPtr;
    uint64_t lastRtpStatsCallbackTime;
    ARSTREAM2_RtpSender_VideoStatsCallback_t videoStatsCallback;
    void *videoStatsCallbackUserPtr;
    ARSTREAM2_StreamSender_DisconnectionCallback_t disconnectionCallback;
    void *disconnectionCallbackUserPtr;
    int maxBitrate;
    uint8_t *rtcpMsgBuffer;

    ARSTREAM2_RTP_SenderContext_t rtpSenderContext;
    ARSTREAM2_RTCP_SenderContext_t rtcpSenderContext;

    /* Sockets */
    int isMulticast;
    int streamSocketSendBufferSize;
    struct sockaddr_in streamSendSin;
    struct sockaddr_in controlSendSin;
    int streamSocket;
    int controlSocket;
    int packetsPending;
    int previouslySending;
    uint32_t nextSrDelay;

    /* NALU and packet FIFO */
    ARSTREAM2_H264_NaluFifo_t *naluFifo;
    ARSTREAM2_RTP_PacketFifo_t *packetFifo;
    ARSTREAM2_RTP_PacketFifoQueue_t *packetFifoQueue;
    struct mmsghdr *msgVec;
    unsigned int msgVecCount;

    /* Monitoring & debug */
    ARSTREAM2_H264_VideoStats_t videoStats;
    char *dateAndTime;
    char *debugPath;
    ARSAL_Mutex_t monitoringMutex;
    int monitoringCount;
    int monitoringIndex;
    ARSTREAM2_RtpSender_MonitoringPoint_t monitoringPoint[ARSTREAM2_RTP_SENDER_MONITORING_MAX_POINTS];
    FILE* fMonitorOut;

    unsigned int timeoutDropCount[ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS];
    unsigned int timeoutDropStatsTotalPackets;
    uint64_t timeoutDropLogStartTime;
    unsigned int rtcpDropCount;
    unsigned int rtcpDropStatsTotalPackets;
    uint64_t rtcpDropLogStartTime;
};


static int ARSTREAM2_RtpSender_SetSocketSendBufferSize(ARSTREAM2_RtpSender_t *sender, int socket, int size)
{
    int ret = 0, err;
    socklen_t size2 = sizeof(size2);

    err = setsockopt(socket, SOL_SOCKET, SO_SNDBUF, (void*)&size, sizeof(size));
    if (err != 0)
    {
        ret = -1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to set socket send buffer size to 2*%d bytes: error=%d (%s)", size, errno, strerror(errno));
    }

    size = -1;
    err = getsockopt(socket, SOL_SOCKET, SO_SNDBUF, (void*)&size, &size2);
    if (err != 0)
    {
        ret = -1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to get socket send buffer size: error=%d (%s)", errno, strerror(errno));
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_RTP_SENDER_TAG, "Socket send buffer size is 2*%d bytes", size / 2);
    }

    return ret;
}


static int ARSTREAM2_RtpSender_StreamSocketSetup(ARSTREAM2_RtpSender_t *sender)
{
    int ret = 0;
    int err;
    struct sockaddr_in sourceSin;

    /* create socket */
    sender->streamSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (sender->streamSocket < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to create stream socket");
        ret = -1;
    }

    /* initialize socket */
#if HAVE_DECL_SO_NOSIGPIPE
    if (ret == 0)
    {
        /* remove SIGPIPE */
        int set = 1;
        err = setsockopt(sender->streamSocket, SOL_SOCKET, SO_NOSIGPIPE, (void*)&set, sizeof(int));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Error on setsockopt: error=%d (%s)", errno, strerror(errno));
        }
    }
#endif

    if (ret == 0)
    {
        int tos = sender->classSelector;
        err = setsockopt(sender->streamSocket, IPPROTO_IP, IP_TOS, (void*)&tos, sizeof(int));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Error on setsockopt: error=%d (%s)", errno, strerror(errno));
        }
    }

    if (ret == 0)
    {
        /* set to non-blocking */
        int flags = fcntl(sender->streamSocket, F_GETFL, 0);
        err = fcntl(sender->streamSocket, F_SETFL, flags | O_NONBLOCK);
        if (err < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to set to non-blocking: error=%d (%s)", errno, strerror(errno));
        }

        /* source address */
        memset(&sourceSin, 0, sizeof(sourceSin));
        sourceSin.sin_family = AF_INET;
        sourceSin.sin_port = htons(sender->serverStreamPort);
        sourceSin.sin_addr.s_addr = htonl(INADDR_ANY);

        /* send address */
        memset(&sender->streamSendSin, 0, sizeof(struct sockaddr_in));
        sender->streamSendSin.sin_family = AF_INET;
        sender->streamSendSin.sin_port = htons(sender->clientStreamPort);
        err = inet_pton(AF_INET, sender->clientAddr, &(sender->streamSendSin.sin_addr));
        if (err <= 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to convert address '%s'", sender->clientAddr);
            ret = -1;
        }
    }

    if ((ret == 0) && (sender->isMulticast))
    {
        int addrFirst = atoi(sender->clientAddr);
        if ((addrFirst >= 224) && (addrFirst <= 239))
        {
            /* multicast */
            if ((sender->mcastIfaceAddr) && (strlen(sender->mcastIfaceAddr) > 0))
            {
                err = inet_pton(AF_INET, sender->mcastIfaceAddr, &(sourceSin.sin_addr));
                if (err <= 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to convert address '%s'", sender->mcastIfaceAddr);
                    ret = -1;
                }
            }
            else
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Trying to send multicast to address '%s' without an interface address", sender->clientAddr);
                ret = -1;
            }
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Invalid multicast address '%s'", sender->clientAddr);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* bind the socket */
        err = bind(sender->streamSocket, (struct sockaddr*)&sourceSin, sizeof(sourceSin));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Error on stream socket bind: error=%d (%s)", errno, strerror(errno));
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* set the socket buffer size */
        if (sender->streamSocketSendBufferSize)
        {
            err = ARSTREAM2_RtpSender_SetSocketSendBufferSize(sender, sender->streamSocket, sender->streamSocketSendBufferSize);
            if (err != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to set the send socket buffer size");
                ret = -1;
            }
        }
    }

    if (ret != 0)
    {
        if (sender->streamSocket >= 0)
        {
            while (((err = close(sender->streamSocket)) == -1) && (errno == EINTR));
        }
        sender->streamSocket = -1;
    }

    return ret;
}


static int ARSTREAM2_RtpSender_ControlSocketSetup(ARSTREAM2_RtpSender_t *sender)
{
    int ret = 0;
    struct sockaddr_in recvSin;
    int err;

    if (ret == 0)
    {
        /* create socket */
        sender->controlSocket = socket(AF_INET, SOCK_DGRAM, 0);
        if (sender->controlSocket < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to create control socket");
            ret = -1;
        }
    }

#if HAVE_DECL_SO_NOSIGPIPE
    if (ret == 0)
    {
        /* remove SIGPIPE */
        int set = 1;
        err = setsockopt(sender->controlSocket, SOL_SOCKET, SO_NOSIGPIPE, (void*)&set, sizeof(int));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Error on setsockopt: error=%d (%s)", errno, strerror(errno));
        }
    }
#endif

    if (ret == 0)
    {
        int tos = sender->classSelector;
        err = setsockopt(sender->controlSocket, IPPROTO_IP, IP_TOS, (void*)&tos, sizeof(int));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Error on setsockopt: error=%d (%s)", errno, strerror(errno));
        }
    }

    if (ret == 0)
    {
        /* set to non-blocking */
        int flags = fcntl(sender->controlSocket, F_GETFL, 0);
        err = fcntl(sender->controlSocket, F_SETFL, flags | O_NONBLOCK);
        if (err < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to set to non-blocking: error=%d (%s)", errno, strerror(errno));
        }

        /* receive address */
        memset(&recvSin, 0, sizeof(struct sockaddr_in));
        recvSin.sin_family = AF_INET;
        recvSin.sin_port = htons(sender->serverControlPort);
        recvSin.sin_addr.s_addr = htonl(INADDR_ANY);
    }

    if (ret == 0)
    {
        /* allow multiple sockets to use the same port */
        unsigned int yes = 1;
        err = setsockopt(sender->controlSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to set socket option SO_REUSEADDR: error=%d (%s)", errno, strerror(errno));
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* send address */
        memset(&sender->controlSendSin, 0, sizeof(struct sockaddr_in));
        sender->controlSendSin.sin_family = AF_INET;
        sender->controlSendSin.sin_port = htons(sender->clientControlPort);
        err = inet_pton(AF_INET, sender->clientAddr, &(sender->controlSendSin.sin_addr));
        if (err <= 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to convert address '%s'", sender->clientAddr);
            ret = -1;
        }
    }

    if ((ret == 0) && (sender->isMulticast))
    {
        int addrFirst = atoi(sender->clientAddr);
        if ((addrFirst >= 224) && (addrFirst <= 239))
        {
            /* multicast */
            if ((sender->mcastIfaceAddr) && (strlen(sender->mcastIfaceAddr) > 0))
            {
                err = inet_pton(AF_INET, sender->mcastIfaceAddr, &(recvSin.sin_addr));
                if (err <= 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to convert address '%s'", sender->mcastIfaceAddr);
                    ret = -1;
                }
            }
            else
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Trying to send multicast to address '%s' without an interface address", sender->clientAddr);
                ret = -1;
            }
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Invalid multicast address '%s'", sender->clientAddr);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* bind the socket */
        err = bind(sender->controlSocket, (struct sockaddr*)&recvSin, sizeof(recvSin));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Error on control socket bind port=%d: error=%d (%s)", sender->serverControlPort, errno, strerror(errno));
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* set the socket buffer size */
        err = ARSTREAM2_RtpSender_SetSocketSendBufferSize(sender, sender->controlSocket, 4096);
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to set the send socket buffer size");
            ret = -1;
        }
    }

    if (ret != 0)
    {
        if (sender->controlSocket >= 0)
        {
            while (((err = close(sender->controlSocket)) == -1) && (errno == EINTR));
        }
        sender->controlSocket = -1;
    }

    return ret;
}


#ifndef HAS_MMSG
static int sendmmsg(int sockfd, struct mmsghdr *msgvec, unsigned int vlen, unsigned int flags)
{
    unsigned int i, count;
    ssize_t ret;

    if (!msgvec)
    {
        return -1;
    }

    for (i = 0, count = 0; i < vlen; i++)
    {
        while (((ret = sendmsg(sockfd, &msgvec[i].msg_hdr, flags)) == -1) && (errno == EINTR));
        if (ret < 0)
        {
            if (count == 0)
            {
                return ret;
            }
            else
            {
                break;
            }
        }
        else
        {
            count++;
            msgvec[i].msg_len = (unsigned int)ret;
        }
    }

    return count;
}
#endif


static void ARSTREAM2_RtpSender_UpdateMonitoring(uint64_t inputTimestamp, uint64_t outputTimestamp, uint64_t ntpTimestamp,
                                                 uint32_t rtpTimestamp, uint16_t seqNum, uint16_t markerBit,
                                                 uint32_t importance, uint32_t priority,
                                                 uint32_t bytesSent, uint32_t bytesDropped, void *userPtr)
{
    ARSTREAM2_RtpSender_t *sender = (ARSTREAM2_RtpSender_t*)userPtr;

    if (!sender)
    {
        return;
    }

    ARSAL_Mutex_Lock(&(sender->monitoringMutex));

    if (sender->monitoringCount < ARSTREAM2_RTP_SENDER_MONITORING_MAX_POINTS)
    {
        sender->monitoringCount++;
    }
    sender->monitoringIndex = (sender->monitoringIndex + 1) % ARSTREAM2_RTP_SENDER_MONITORING_MAX_POINTS;
    sender->monitoringPoint[sender->monitoringIndex].inputTimestamp = inputTimestamp;
    sender->monitoringPoint[sender->monitoringIndex].outputTimestamp = outputTimestamp;
    sender->monitoringPoint[sender->monitoringIndex].ntpTimestamp = ntpTimestamp;
    sender->monitoringPoint[sender->monitoringIndex].rtpTimestamp = rtpTimestamp;
    sender->monitoringPoint[sender->monitoringIndex].seqNum = seqNum;
    sender->monitoringPoint[sender->monitoringIndex].markerBit = markerBit;
    sender->monitoringPoint[sender->monitoringIndex].importance = importance;
    sender->monitoringPoint[sender->monitoringIndex].priority = priority;
    sender->monitoringPoint[sender->monitoringIndex].bytesSent = bytesSent;
    sender->monitoringPoint[sender->monitoringIndex].bytesDropped = bytesDropped;

    ARSAL_Mutex_Unlock(&(sender->monitoringMutex));

    if (sender->fMonitorOut)
    {
        fprintf(sender->fMonitorOut, "%llu ", (long long unsigned int)ntpTimestamp);
        fprintf(sender->fMonitorOut, "%llu ", (long long unsigned int)inputTimestamp);
        fprintf(sender->fMonitorOut, "%llu ", (long long unsigned int)outputTimestamp);
        fprintf(sender->fMonitorOut, "%lu %u %u %u %u %lu %lu\n", (long unsigned int)rtpTimestamp, seqNum, markerBit,
                importance, priority, (long unsigned int)bytesSent, (long unsigned int)bytesDropped);
    }
}


ARSTREAM2_RtpSender_t* ARSTREAM2_RtpSender_New(const ARSTREAM2_RtpSender_Config_t *config, eARSTREAM2_ERROR *error)
{
    ARSTREAM2_RtpSender_t *retSender = NULL;
    int monitoringMutexWasInit = 0;
    eARSTREAM2_ERROR internalError = ARSTREAM2_OK;

    /* ARGS Check */
    if (config == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "No config provided");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retSender;
    }
    if ((config->canonicalName == NULL) || (!strlen(config->canonicalName)))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Config: no canonical name provided");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retSender;
    }
    if (((config->clientAddr == NULL) || (!strlen(config->clientAddr)))
            && (((config->mcastAddr == NULL) || (!strlen(config->mcastAddr))) || ((config->mcastIfaceAddr == NULL) || (!strlen(config->mcastIfaceAddr)))))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Config: no destination address provided");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retSender;
    }
    if ((config->clientStreamPort <= 0) || (config->clientControlPort <= 0))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Config: no client ports provided");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retSender;
    }
    if ((!config->packetFifo) || (!config->packetFifoQueue))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Config: no packet FIFO provided");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retSender;
    }

    /* Alloc new sender */
    retSender = malloc(sizeof(ARSTREAM2_RtpSender_t));
    if (retSender == NULL)
    {
        internalError = ARSTREAM2_ERROR_ALLOC;
    }

    /* Initialize the sender and copy parameters */
    if (internalError == ARSTREAM2_OK)
    {
        memset(retSender, 0, sizeof(ARSTREAM2_RtpSender_t));
        retSender->isMulticast = 0;
        retSender->streamSocket = -1;
        retSender->controlSocket = -1;
        if (config->canonicalName)
        {
            retSender->canonicalName = strndup(config->canonicalName, 40);
        }
        if (config->friendlyName)
        {
            retSender->friendlyName = strndup(config->friendlyName, 40);
        }
        if (config->applicationName)
        {
            retSender->applicationName = strndup(config->applicationName, 40);
        }
        if (config->mcastAddr)
        {
            retSender->isMulticast = 1;
            retSender->clientAddr = strndup(config->mcastAddr, 16);
        }
        if (config->mcastIfaceAddr)
        {
            retSender->mcastIfaceAddr = strndup(config->mcastIfaceAddr, 16);
        }
        if ((!retSender->clientAddr) && (config->clientAddr))
        {
            retSender->clientAddr = strndup(config->clientAddr, 16);
        }
        retSender->serverStreamPort = (config->serverStreamPort > 0) ? config->serverStreamPort : ARSTREAM2_RTP_SENDER_DEFAULT_SERVER_STREAM_PORT;
        retSender->serverControlPort = (config->serverControlPort > 0) ? config->serverControlPort : ARSTREAM2_RTP_SENDER_DEFAULT_SERVER_CONTROL_PORT;
        retSender->clientStreamPort = config->clientStreamPort;
        retSender->clientControlPort = config->clientControlPort;
        retSender->classSelector = config->classSelector;
        retSender->rtpSenderContext.auCallback = config->auCallback;
        retSender->rtpSenderContext.auCallbackUserPtr = config->auCallbackUserPtr;
        retSender->rtpSenderContext.naluCallback = config->naluCallback;
        retSender->rtpSenderContext.naluCallbackUserPtr = config->naluCallbackUserPtr;
        retSender->rtpSenderContext.monitoringCallback = ARSTREAM2_RtpSender_UpdateMonitoring;
        retSender->rtpSenderContext.monitoringCallbackUserPtr = retSender;
        retSender->rtpStatsCallback = config->rtpStatsCallback;
        retSender->rtpStatsCallbackUserPtr = config->rtpStatsCallbackUserPtr;
        retSender->videoStatsCallback = config->videoStatsCallback;
        retSender->videoStatsCallbackUserPtr = config->videoStatsCallbackUserPtr;
        retSender->disconnectionCallback = config->disconnectionCallback;
        retSender->disconnectionCallbackUserPtr = config->disconnectionCallbackUserPtr;
        retSender->naluFifo = config->naluFifo;
        retSender->packetFifo = config->packetFifo;
        retSender->packetFifoQueue = config->packetFifoQueue;
        retSender->msgVecCount = retSender->packetFifo->bufferPoolSize;
        retSender->rtpSenderContext.maxPacketSize = config->maxPacketSize;
        retSender->rtpSenderContext.targetPacketSize = config->targetPacketSize;
        retSender->maxBitrate = config->maxBitrate;
        retSender->streamSocketSendBufferSize = config->streamSocketSendBufferSize;
        retSender->rtpSenderContext.useRtpHeaderExtensions = (config->useRtpHeaderExtensions > 0) ? 1 : 0;
        retSender->rtpSenderContext.senderSsrc = ARSTREAM2_RTP_SENDER_SSRC;
        retSender->rtpSenderContext.rtpClockRate = 90000;
        retSender->rtpSenderContext.rtpTimestampOffset = 0;
        retSender->rtcpSenderContext.senderSsrc = ARSTREAM2_RTP_SENDER_SSRC;
        retSender->rtcpSenderContext.sdesItemCount = 0;
        if ((retSender->canonicalName) && (strlen(retSender->canonicalName)))
        {
            retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].type = ARSTREAM2_RTCP_SDES_CNAME_ITEM;
            snprintf(retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].value, 256, "%s", retSender->canonicalName);
            retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].sendTimeInterval = 0;
            retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].lastSendTime = 0;
            retSender->rtcpSenderContext.sdesItemCount++;
        }
        if ((retSender->friendlyName) && (strlen(retSender->friendlyName)))
        {
            retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].type = ARSTREAM2_RTCP_SDES_NAME_ITEM;
            snprintf(retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].value, 256, "%s", retSender->friendlyName);
            retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].sendTimeInterval = 5000000;
            retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].lastSendTime = 0;
            retSender->rtcpSenderContext.sdesItemCount++;
        }
        if ((retSender->applicationName) && (strlen(retSender->applicationName)))
        {
            retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].type = ARSTREAM2_RTCP_SDES_TOOL_ITEM;
            snprintf(retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].value, 256, "%s", retSender->applicationName);
            retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].sendTimeInterval = 5000000;
            retSender->rtcpSenderContext.sdesItem[retSender->rtcpSenderContext.sdesItemCount].lastSendTime = 0;
            retSender->rtcpSenderContext.sdesItemCount++;
        }
        retSender->rtcpSenderContext.rtcpByteRate = (retSender->maxBitrate > 0) ? retSender->maxBitrate * ARSTREAM2_RTCP_SENDER_BANDWIDTH_SHARE / 8 : ARSTREAM2_RTCP_SENDER_DEFAULT_BITRATE / 8;
        retSender->rtcpSenderContext.rtpClockRate = 90000;
        retSender->rtcpSenderContext.rtpTimestampOffset = 0;
        retSender->packetsPending = 0;
        retSender->previouslySending = 0;
        retSender->nextSrDelay = ARSTREAM2_RTCP_SENDER_MIN_PACKET_TIME_INTERVAL;

        if (retSender->rtpSenderContext.maxPacketSize < sizeof(ARSTREAM2_RTCP_SenderReport_t))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Config: max packet size is too small to hold a sender report");
            internalError = ARSTREAM2_ERROR_BAD_PARAMETERS;
        }

        if ((config->debugPath) && (strlen(config->debugPath)))
        {
            retSender->debugPath = strdup(config->debugPath);
        }
        if ((config->dateAndTime) && (strlen(config->dateAndTime)))
        {
            retSender->dateAndTime = strdup(config->dateAndTime);
        }

        struct timespec t1;
        ARSAL_Time_GetTime(&t1);
        srand(t1.tv_nsec);
    }

    /* Setup internal mutexes/sems */
    if (internalError == ARSTREAM2_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init(&(retSender->monitoringMutex));
        if (mutexInitRet != 0)
        {
            internalError = ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            monitoringMutexWasInit = 1;
        }
    }

    /* MsgVec array */
    if (internalError == ARSTREAM2_OK)
    {
        if (retSender->msgVecCount > 0)
        {
            retSender->msgVec = malloc(retSender->msgVecCount * sizeof(struct mmsghdr));
            if (!retSender->msgVec)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "MsgVec allocation failed (size %ld)", (long)retSender->msgVecCount * sizeof(struct mmsghdr));
                internalError = ARSTREAM2_ERROR_ALLOC;
            }
            else
            {
                memset(retSender->msgVec, 0, retSender->msgVecCount * sizeof(struct mmsghdr));
            }
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Invalid msgVecCount: %d", retSender->msgVecCount);
            internalError = ARSTREAM2_ERROR_BAD_PARAMETERS;
        }
    }

    /* Stream socket setup */
    if (internalError == ARSTREAM2_OK)
    {
        int socketRet = ARSTREAM2_RtpSender_StreamSocketSetup(retSender);
        if (socketRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to setup the stream socket (error %d)", socketRet);
            internalError = ARSTREAM2_ERROR_RESOURCE_UNAVAILABLE;
        }
    }

    /* Control socket setup */
    if (internalError == ARSTREAM2_OK)
    {
        int socketRet = ARSTREAM2_RtpSender_ControlSocketSetup(retSender);
        if (socketRet != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to setup the control socket (error %d)", socketRet);
            internalError = ARSTREAM2_ERROR_RESOURCE_UNAVAILABLE;
        }
    }

    /* RTCP message buffer */
    if (internalError == ARSTREAM2_OK)
    {
        retSender->rtcpMsgBuffer = malloc(retSender->rtpSenderContext.maxPacketSize);
        if (retSender->rtcpMsgBuffer == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Memory allocation failed (%d)", retSender->rtpSenderContext.maxPacketSize);
            internalError = ARSTREAM2_ERROR_ALLOC;
        }
    }

    if (internalError == ARSTREAM2_OK)
    {
        char szOutputFileName[500];
        szOutputFileName[0] = '\0';

        if ((retSender->debugPath) && (strlen(retSender->debugPath)) && (retSender->dateAndTime) && (strlen(retSender->dateAndTime)))
        {
            snprintf(szOutputFileName, 500, "%s/%s", retSender->debugPath,
                     ARSTREAM2_RTP_SENDER_MONITORING_OUTPUT_PATH);
            if ((access(szOutputFileName, F_OK) == 0) && (access(szOutputFileName, W_OK) == 0))
            {
                // directory exists and we have write permission
                snprintf(szOutputFileName, 500, "%s/%s/%s_%s.%s", retSender->debugPath,
                         ARSTREAM2_RTP_SENDER_MONITORING_OUTPUT_PATH,
                         ARSTREAM2_RTP_SENDER_MONITORING_OUTPUT_FILENAME,
                         retSender->dateAndTime,
                         ARSTREAM2_RTP_SENDER_MONITORING_OUTPUT_FILEEXT);
            }
            else
            {
                szOutputFileName[0] = '\0';
            }
        }

        if (strlen(szOutputFileName))
        {
            retSender->fMonitorOut = fopen(szOutputFileName, "w");
            if (!retSender->fMonitorOut)
            {
                ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTP_SENDER_TAG, "Unable to open RTP monitor output file '%s'", szOutputFileName);
            }
            else
            {
                ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_RTP_SENDER_TAG, "Opened RTP monitor output file '%s'", szOutputFileName);
            }
        }

        if (retSender->fMonitorOut)
        {
            fprintf(retSender->fMonitorOut, "ntpTimestamp inputTimestamp outputTimestamp rtpTimestamp rtpSeqNum rtpMarkerBit importance priority bytesSent bytesDropped\n");
        }
    }

    if ((internalError != ARSTREAM2_OK) &&
        (retSender != NULL))
    {
        int err;
        if (retSender->streamSocket != -1)
        {
            while (((err = close(retSender->streamSocket)) == -1) && (errno == EINTR));
            retSender->streamSocket = -1;
        }
        if (retSender->controlSocket != -1)
        {
            while (((err = close(retSender->controlSocket)) == -1) && (errno == EINTR));
            retSender->controlSocket = -1;
        }
        if (monitoringMutexWasInit == 1) ARSAL_Mutex_Destroy(&(retSender->monitoringMutex));
        free(retSender->msgVec);
        free(retSender->rtcpMsgBuffer);
        free(retSender->canonicalName);
        free(retSender->friendlyName);
        free(retSender->applicationName);
        free(retSender->clientAddr);
        free(retSender->mcastIfaceAddr);
        free(retSender->debugPath);
        free(retSender->dateAndTime);
        if (retSender->fMonitorOut)
        {
            fclose(retSender->fMonitorOut);
        }
        free(retSender);
        retSender = NULL;
    }

    SET_WITH_CHECK(error, internalError);
    return retSender;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_Delete(ARSTREAM2_RtpSender_t **sender)
{
    eARSTREAM2_ERROR retVal = ARSTREAM2_ERROR_BAD_PARAMETERS;
    if ((sender != NULL) &&
        (*sender != NULL))
    {
        int err;
        ARSAL_Mutex_Destroy(&((*sender)->monitoringMutex));
        if ((*sender)->streamSocket != -1)
        {
            while (((err = close((*sender)->streamSocket)) == -1) && (errno == EINTR));
            (*sender)->streamSocket = -1;
        }
        if ((*sender)->controlSocket != -1)
        {
            while (((err = close((*sender)->controlSocket)) == -1) && (errno == EINTR));
            (*sender)->controlSocket = -1;
        }
        free((*sender)->msgVec);
        free((*sender)->rtcpMsgBuffer);
        free((*sender)->friendlyName);
        free((*sender)->applicationName);
        free((*sender)->clientAddr);
        free((*sender)->mcastIfaceAddr);
        free((*sender)->debugPath);
        free((*sender)->dateAndTime);
        if ((*sender)->fMonitorOut)
        {
            fclose((*sender)->fMonitorOut);
        }
        free((*sender)->rtcpSenderContext.lossReportCtx.receivedFlag);
        free(*sender);
        *sender = NULL;
        retVal = ARSTREAM2_OK;
    }
    return retVal;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_FlushNaluQueue(ARSTREAM2_RtpSender_t *sender)
{
    eARSTREAM2_ERROR retVal = ARSTREAM2_OK;
    struct timespec t1;
    uint64_t curTime;

    // Args check
    if (sender == NULL)
    {
        retVal = ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (retVal == ARSTREAM2_OK)
    {
        ARSAL_Time_GetTime(&t1);
        curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

        int ret = ARSTREAM2_RTPH264_Sender_FifoFlush(&sender->rtpSenderContext, sender->naluFifo, curTime);
        if (ret != 0)
        {
            retVal = ARSTREAM2_ERROR_BAD_PARAMETERS;
        }
    }

    return retVal;
}


static void ARSTREAM2_RtpSender_RtpStatsCallback(ARSTREAM2_RtpSender_t *sender, uint64_t curTime, int gotLossReport)
{
    ARSTREAM2_RTP_RtpStats_t rtpStats;

    memset(&rtpStats, 0, sizeof(ARSTREAM2_RTP_RtpStats_t));
    rtpStats.senderStats.timestamp = curTime;
    rtpStats.senderStats.sentPacketCount = sender->rtpSenderContext.sentPacketCount;
    rtpStats.senderStats.droppedPacketCount = sender->rtpSenderContext.droppedPacketCount;
    rtpStats.senderStats.sentByteIntegral = sender->rtpSenderContext.sentByteIntegral;
    rtpStats.senderStats.sentByteIntegralSq = sender->rtpSenderContext.sentByteIntegralSq;
    rtpStats.senderStats.droppedByteIntegral = sender->rtpSenderContext.droppedByteIntegral;
    rtpStats.senderStats.droppedByteIntegralSq = sender->rtpSenderContext.droppedByteIntegralSq;
    rtpStats.senderStats.inputToSentTimeIntegral = sender->rtpSenderContext.inputToSentTimeIntegral;
    rtpStats.senderStats.inputToSentTimeIntegralSq = sender->rtpSenderContext.inputToSentTimeIntegralSq;
    rtpStats.senderStats.inputToDroppedTimeIntegral = sender->rtpSenderContext.inputToDroppedTimeIntegral;
    rtpStats.senderStats.inputToDroppedTimeIntegralSq = sender->rtpSenderContext.inputToDroppedTimeIntegralSq;
    if (sender->rtcpSenderContext.lastSrTimestamp != 0)
    {
        rtpStats.senderReport.timestamp = sender->rtcpSenderContext.lastSrTimestamp;
        rtpStats.senderReport.lastInterval = sender->rtcpSenderContext.lastSrInterval;
        rtpStats.senderReport.intervalPacketCount = sender->rtcpSenderContext.srIntervalPacketCount;
        rtpStats.senderReport.intervalByteCount = sender->rtcpSenderContext.srIntervalByteCount;
    }
    if (sender->rtcpSenderContext.lastRrReceptionTimestamp != 0)
    {
        rtpStats.receiverReport.timestamp = sender->rtcpSenderContext.lastRrReceptionTimestamp;
        rtpStats.receiverReport.roundTripDelay = sender->rtcpSenderContext.roundTripDelay;
        rtpStats.receiverReport.interarrivalJitter = sender->rtcpSenderContext.interarrivalJitter;
        rtpStats.receiverReport.receiverLostCount = sender->rtcpSenderContext.receiverLostCount;
        rtpStats.receiverReport.receiverFractionLost = sender->rtcpSenderContext.receiverFractionLost;
        rtpStats.receiverReport.receiverExtHighestSeqNum = sender->rtcpSenderContext.receiverExtHighestSeqNum;
    }
    if ((gotLossReport) && (sender->rtcpSenderContext.lossReportCtx.lastReceptionTimestamp != 0))
    {
        rtpStats.lossReport.timestamp = sender->rtcpSenderContext.lossReportCtx.lastReceptionTimestamp;
        rtpStats.lossReport.startSeqNum = (uint16_t)(sender->rtcpSenderContext.lossReportCtx.startSeqNum & 0xFFFF);
        rtpStats.lossReport.endSeqNum = (uint16_t)(sender->rtcpSenderContext.lossReportCtx.endSeqNum & 0xFFFF);
        rtpStats.lossReport.receivedFlag = sender->rtcpSenderContext.lossReportCtx.receivedFlag;
    }
    if ((sender->rtcpSenderContext.djbReportCtx.djbMetricsAvailable) && (sender->rtcpSenderContext.djbReportCtx.lastReceptionTimestamp != 0))
    {
        rtpStats.djbMetricsReport.timestamp = sender->rtcpSenderContext.djbReportCtx.lastReceptionTimestamp;
        rtpStats.djbMetricsReport.djbNominal = sender->rtcpSenderContext.djbReportCtx.djbNominal;
        rtpStats.djbMetricsReport.djbMax = sender->rtcpSenderContext.djbReportCtx.djbMax;
        rtpStats.djbMetricsReport.djbHighWatermark = sender->rtcpSenderContext.djbReportCtx.djbHighWatermark;
        rtpStats.djbMetricsReport.djbLowWatermark = sender->rtcpSenderContext.djbReportCtx.djbLowWatermark;
    }
    rtpStats.clockDelta.peerClockDelta = sender->rtcpSenderContext.clockDeltaCtx.clockDeltaAvg;
    rtpStats.clockDelta.roundTripDelay = (uint32_t)sender->rtcpSenderContext.clockDeltaCtx.rtDelayAvg;
    rtpStats.clockDelta.peer2meDelay = (uint32_t)sender->rtcpSenderContext.clockDeltaCtx.p2mDelayAvg;
    rtpStats.clockDelta.me2peerDelay = (uint32_t)sender->rtcpSenderContext.clockDeltaCtx.m2pDelayAvg;

    /* Call the RTP stats callback function */
    sender->rtpStatsCallback(&rtpStats, sender->rtpStatsCallbackUserPtr);

    sender->lastRtpStatsCallbackTime = curTime;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_GetSelectParams(ARSTREAM2_RtpSender_t *sender, fd_set **readSet, fd_set **writeSet, fd_set **exceptSet, int *maxFd, uint32_t *nextTimeout)
{
    eARSTREAM2_ERROR retVal = ARSTREAM2_OK;
    int _maxFd;

    // Args check
    if (sender == NULL)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    _maxFd = -1;
    if (sender->streamSocket > _maxFd) _maxFd = sender->streamSocket;
    if (sender->controlSocket > _maxFd) _maxFd = sender->controlSocket;

    if (readSet)
    {
        FD_SET(sender->controlSocket, *readSet);
    }
    if (writeSet)
    {
        if (sender->packetsPending)
            FD_SET(sender->streamSocket, *writeSet);
    }
    if (exceptSet)
    {
        FD_SET(sender->streamSocket, *exceptSet);
        FD_SET(sender->controlSocket, *exceptSet);
    }

    if (maxFd) *maxFd = _maxFd;
    if (nextTimeout) *nextTimeout = (sender->nextSrDelay < ARSTREAM2_RTP_SENDER_TIMEOUT_US) ? sender->nextSrDelay : ARSTREAM2_RTP_SENDER_TIMEOUT_US;

    return retVal;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_ProcessRtp(ARSTREAM2_RtpSender_t *sender, int selectRet, fd_set *readSet, fd_set *writeSet, fd_set *exceptSet)
{
    eARSTREAM2_ERROR retVal = ARSTREAM2_OK;
    unsigned int dropCount[ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS];
    struct timespec t1;
    uint64_t curTime;
    int ret;

    // Args check
    if (sender == NULL)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if ((exceptSet) && (FD_ISSET(sender->streamSocket, exceptSet)))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Exception on stream socket");
    }

    ARSAL_Time_GetTime(&t1);
    curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

    /* RTP packet FIFO cleanup (packets on timeout) */
    ret = ARSTREAM2_RTP_Sender_PacketFifoCleanFromTimeout(&sender->rtpSenderContext, sender->packetFifo, sender->packetFifoQueue,
                                                          curTime, dropCount, ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS);
    if (ret < 0)
    {
        if (ret != -2)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to clean FIFO from timeout (%d)", ret);
        }
    }
    else if (ret > 0)
    {
        /* Log drops once in a while */
        if (sender->timeoutDropLogStartTime)
        {
            if (curTime >= sender->timeoutDropLogStartTime + (uint64_t)ARSTREAM2_RTP_SENDER_TIMEOUT_DROP_LOG_INTERVAL * 1000000)
            {
                char strDrops[16];
                char *str = strDrops;
                int i, l, len;
                unsigned int totalCount;
                for (i = 0, len = 0, totalCount = 0; i < ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS; i++)
                {
                    totalCount += sender->timeoutDropCount[i];
                    l = snprintf(str, 16 - len, "%s%d", (i > 0) ? " " : "", sender->timeoutDropCount[i]);
                    len += l;
                    str += l;
                }
                ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTP_SENDER_TAG, "Dropped %d packets out of %d (%.1f%%) from FIFO on timeout (%s) in last %.1f seconds",
                            totalCount, sender->timeoutDropStatsTotalPackets, (float)totalCount * 100. / (float)sender->timeoutDropStatsTotalPackets,
                            strDrops, (float)(curTime - sender->timeoutDropLogStartTime) / 1000000.);
                for (i = 0; i < ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS; i++)
                {
                    sender->timeoutDropCount[i] = 0;
                }
                sender->timeoutDropLogStartTime = 0;
                sender->timeoutDropStatsTotalPackets = 0;
            }
            else
            {
                int i;
                for (i = 0; i < ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS; i++)
                {
                    sender->timeoutDropCount[i] += dropCount[i];
                }
            }
        }
        else
        {
            int i, totalCount;
            for (i = 0, totalCount = 0; i < ARSTREAM2_STREAM_SENDER_MAX_IMPORTANCE_LEVELS; i++)
            {
                totalCount += dropCount[i];
                sender->timeoutDropCount[i] += dropCount[i];
            }
            if (totalCount > 0)
            {
                sender->timeoutDropLogStartTime = curTime;
            }
        }
    }

    /* RTP packets creation */
    if (sender->naluFifo != NULL)
    {
        int newPacketsCount = 0;
#ifdef ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION
        int dropOnTimeout = 0;
#else
        int dropOnTimeout = 1;
#endif
        ret = ARSTREAM2_RTPH264_Sender_NaluFifoToPacketFifo(&sender->rtpSenderContext, sender->naluFifo,
                                                            sender->packetFifo, sender->packetFifoQueue,
                                                            curTime, dropOnTimeout, &newPacketsCount);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "ARSTREAM2_RTPH264_Sender_NaluFifoToPacketFifo() failed (%d)", ret);
        }
        sender->timeoutDropStatsTotalPackets = ((int)sender->timeoutDropStatsTotalPackets+ newPacketsCount > 0) ? ((int)sender->timeoutDropStatsTotalPackets+ newPacketsCount) : 0;
    }

#ifdef ARSTREAM2_RTP_SENDER_RANDOM_DROP
    ret = ARSTREAM2_RTP_Sender_PacketFifoRandomDrop(&sender->rtpSenderContext, sender->packetFifo,
                                                    sender->packetFifoQueue, ARSTREAM2_RTP_SENDER_RANDOM_DROP_RATIO, curTime);
    if (ret < 0)
    {
        if (ret != -2)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "ARSTREAM2_RTP_Sender_PacketFifoRandomDrop() failed (%d)", ret);
        }
    }
#endif

#ifdef ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION
    uint32_t waitTime = ((uint64_t)rand() * (ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION_WAIT_MAX - ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION_WAIT_MIN) / RAND_MAX + ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION_WAIT_MIN);
    usleep(waitTime);
#endif

    /* RTP packets sending */
    if ((!sender->packetsPending) || ((sender->packetsPending) && ((!writeSet) || ((selectRet >= 0) && (FD_ISSET(sender->streamSocket, writeSet))))))
    {
        ret = ARSTREAM2_RTP_Sender_PacketFifoFillMsgVec(sender->packetFifoQueue, sender->msgVec, sender->msgVecCount, (void*)&sender->streamSendSin, sizeof(sender->streamSendSin));
        if (ret < 0)
        {
            if (ret != -2)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to fill msgVec (%d)", ret);
            }
        }
        else if (ret > 0)
        {
            int msgVecCount = ret;
            int msgVecSentCount = 0;

#ifdef ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION
            msgVecCount = round((float)msgVecCount * ((float)rand() * (ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION_MSG_MAX - ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION_MSG_MIN) / RAND_MAX + ARSTREAM2_RTP_SENDER_RANDOM_CONGESTION_MSG_MIN));
#endif

            sender->packetsPending = 1;
            while (((ret = sendmmsg(sender->streamSocket, sender->msgVec, msgVecCount, 0)) == -1) && (errno == EINTR));
            if (ret < 0)
            {
                if (errno == EAGAIN)
                {
                    //ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTP_SENDER_TAG, "Stream socket buffer full (no packets dropped, will retry later) - sendmmsg error (%d): %s", errno, strerror(errno)); //TODO: debug
                    int i;
                    for (i = 0, msgVecSentCount = 0; i < msgVecCount; i++)
                    {
                        if (sender->msgVec[i].msg_len > 0) msgVecSentCount++;
                    }
                    sender->packetsPending = (msgVecSentCount < msgVecCount) ? 1 : 0;
                    //if (sender->packetsPending) ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTP_SENDER_TAG, "Sent %d packets out of %d (socket buffer is full)", msgVecSentCount, msgVecCount); //TODO: debug
                }
                else
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Stream socket - sendmmsg error (%d): %s", errno, strerror(errno));
                    if ((sender->disconnectionCallback) && (sender->previouslySending) && (errno == ECONNREFUSED))
                    {
                        /* Call the disconnection callback */
                        sender->disconnectionCallback(sender->disconnectionCallbackUserPtr);
                    }
                }
            }
            else
            {
                sender->previouslySending = 1;
                msgVecSentCount = ret;
                sender->packetsPending = (msgVecSentCount < msgVecCount) ? 1 : 0;
                //if (sender->packetsPending) ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTP_SENDER_TAG, "Sent %d packets out of %d", msgVecSentCount, msgVecCount); //TODO: debug
            }

            ret = ARSTREAM2_RTP_Sender_PacketFifoCleanFromMsgVec(&sender->rtpSenderContext, sender->packetFifo,
                                                                 sender->packetFifoQueue, sender->msgVec,
                                                                 msgVecSentCount, curTime);
            if (ret < 0)
            {
                if (ret != -2)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to clean FIFO from msgVec (%d)", ret);
                }
            }
        }
    }

    if ((sender->rtpStatsCallback != NULL) && (curTime >= sender->lastRtpStatsCallbackTime + ARSTREAM2_RTP_SENDER_RTPSTATS_CALLBACK_MAX_INTERVAL))
    {
        ARSTREAM2_RtpSender_RtpStatsCallback(sender, curTime, 0);
    }

    return retVal;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_ProcessRtcp(ARSTREAM2_RtpSender_t *sender, int selectRet, fd_set *readSet, fd_set *writeSet, fd_set *exceptSet)
{
    eARSTREAM2_ERROR retVal = ARSTREAM2_OK;
    struct timespec t1;
    uint64_t curTime;
    int ret;

    // Args check
    if (sender == NULL)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if ((exceptSet) && (FD_ISSET(sender->controlSocket, exceptSet)))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Exception on control socket");
    }

    ARSAL_Time_GetTime(&t1);
    curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

    /* RTCP receiver reports */
    if ((!readSet) || ((selectRet >= 0) && (FD_ISSET(sender->controlSocket, readSet))))
    {
        /* The control socket is ready for reading */
        //TODO: recvmmsg?
        ssize_t bytes;
        while (((bytes = recv(sender->controlSocket, sender->rtcpMsgBuffer, sender->rtpSenderContext.maxPacketSize, 0)) == -1) && (errno == EINTR));
        if ((bytes < 0) && (errno != EAGAIN))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Control socket - read error (%d): %s", errno, strerror(errno));
        }
        while (bytes > 0)
        {
            int gotReceptionReport = 0;
            int gotVideoStats = 0;
            int gotLossReport = 0;
            int gotDjbReport = 0;

            ret = ARSTREAM2_RTCP_Sender_ProcessCompoundPacket(sender->rtcpMsgBuffer, (unsigned int)bytes,
                                                              curTime, &sender->rtcpSenderContext,
                                                              &gotReceptionReport, &gotVideoStats,
                                                              &gotLossReport, &gotDjbReport);
            if ((ret != 0) && (bytes != 24)) /* workaround to avoid logging when it's an old clockSync packet with old FF or SC versions */
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to process compound RTCP packet (%d)", ret);
            }

            if ((gotVideoStats) && (sender->videoStatsCallback != NULL))
            {
                /* Call the receiver report callback function */
                sender->videoStatsCallback(&sender->rtcpSenderContext.videoStatsCtx.videoStats, sender->videoStatsCallbackUserPtr);
            }

            if (((gotReceptionReport) || (gotLossReport) || (gotDjbReport)) && (sender->rtpStatsCallback != NULL))
            {
                ARSTREAM2_RtpSender_RtpStatsCallback(sender, curTime, gotLossReport);
            }

            while (((bytes = recv(sender->controlSocket, sender->rtcpMsgBuffer, sender->rtpSenderContext.maxPacketSize, 0)) == -1) && (errno == EINTR));
            if ((bytes < 0) && (errno != EAGAIN))
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Control socket - read error (%d): %s", errno, strerror(errno));
            }
        }
    }

    /* RTCP sender reports */
    uint32_t srDelay = (uint32_t)(curTime - sender->rtcpSenderContext.lastRtcpTimestamp);
    if (srDelay >= sender->nextSrDelay)
    {
        unsigned int size = 0;

        ret = ARSTREAM2_RTCP_Sender_GenerateCompoundPacket(sender->rtcpMsgBuffer, sender->rtpSenderContext.maxPacketSize, curTime, 1, 1, 1,
                                                           sender->rtpSenderContext.packetCount, sender->rtpSenderContext.byteCount,
                                                           &sender->rtcpSenderContext, &size);

        if ((ret == 0) && (size > 0))
        {
            sender->rtcpDropStatsTotalPackets++;
            ssize_t bytes;
            while (((bytes = sendto(sender->controlSocket, sender->rtcpMsgBuffer, size, 0, (struct sockaddr*)&sender->controlSendSin, sizeof(sender->controlSendSin))) == -1) && (errno == EINTR));
            if (bytes < 0)
            {
                if (errno == EAGAIN)
                {
                    /* Log drops once in a while */
                    sender->rtcpDropCount++;
                    if (sender->rtcpDropLogStartTime)
                    {
                        if (curTime >= sender->rtcpDropLogStartTime + (uint64_t)ARSTREAM2_RTP_SENDER_RTCP_DROP_LOG_INTERVAL * 1000000)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTP_SENDER_TAG, "Dropped %d RTCP packets out of %d (%.1f%%) on socket buffer full in last %.1f seconds",
                                        sender->rtcpDropCount, sender->rtcpDropStatsTotalPackets, (float)sender->rtcpDropCount * 100. / (float)sender->rtcpDropStatsTotalPackets,
                                        (float)(curTime - sender->rtcpDropLogStartTime) / 1000000.);
                            sender->rtcpDropCount = 0;
                            sender->rtcpDropStatsTotalPackets = 0;
                            sender->rtcpDropLogStartTime = 0;
                        }
                    }
                    else
                    {
                        sender->rtcpDropLogStartTime = curTime;
                    }
                }
                else
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Control socket - send error (%d): %s", errno, strerror(errno));
                }
            }
        }

        sender->rtcpSenderContext.lastRtcpTimestamp = curTime;
        srDelay = 0;
        sender->nextSrDelay = (size + ARSTREAM2_RTP_UDP_HEADER_SIZE + ARSTREAM2_RTP_IP_HEADER_SIZE) * 1000000 / sender->rtcpSenderContext.rtcpByteRate;
        if (sender->nextSrDelay < ARSTREAM2_RTCP_SENDER_MIN_PACKET_TIME_INTERVAL) sender->nextSrDelay = ARSTREAM2_RTCP_SENDER_MIN_PACKET_TIME_INTERVAL;
    }

    return retVal;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_ProcessEnd(ARSTREAM2_RtpSender_t *sender, int queueOnly)
{
    eARSTREAM2_ERROR retVal = ARSTREAM2_OK;
    struct timespec t1;
    uint64_t curTime;

    // Args check
    if (sender == NULL)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    /* flush the NALU FIFO and packet FIFO */
    ARSAL_Time_GetTime(&t1);
    curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
    if (sender->naluFifo != NULL) ARSTREAM2_RTPH264_Sender_FifoFlush(&sender->rtpSenderContext, sender->naluFifo, curTime);
    if (queueOnly)
        ARSTREAM2_RTP_Sender_PacketFifoFlushQueue(&sender->rtpSenderContext, sender->packetFifo, sender->packetFifoQueue, curTime);
    else
        ARSTREAM2_RTP_Sender_PacketFifoFlush(&sender->rtpSenderContext, sender->packetFifo, curTime);

    return retVal;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_GetDynamicConfig(ARSTREAM2_RtpSender_t *sender, ARSTREAM2_RtpSender_DynamicConfig_t *config)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if ((sender == NULL) || (config == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    config->targetPacketSize = sender->rtpSenderContext.targetPacketSize;
    config->streamSocketSendBufferSize = sender->streamSocketSendBufferSize;
    config->maxBitrate = sender->maxBitrate;

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_SetDynamicConfig(ARSTREAM2_RtpSender_t *sender, const ARSTREAM2_RtpSender_DynamicConfig_t *config)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if ((sender == NULL) || (config == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    sender->rtpSenderContext.targetPacketSize = config->targetPacketSize;
    sender->maxBitrate = config->maxBitrate;
    sender->rtcpSenderContext.rtcpByteRate = (sender->maxBitrate > 0) ? sender->maxBitrate * ARSTREAM2_RTCP_SENDER_BANDWIDTH_SHARE / 8 : ARSTREAM2_RTCP_SENDER_DEFAULT_BITRATE / 8;
    sender->streamSocketSendBufferSize = config->streamSocketSendBufferSize;

    if ((sender->streamSocket != -1) && (sender->streamSocketSendBufferSize))
    {
        int err = ARSTREAM2_RtpSender_SetSocketSendBufferSize(sender, sender->streamSocket, sender->streamSocketSendBufferSize);
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_SENDER_TAG, "Failed to set the send socket buffer size");
        }
    }

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_GetSdesItem(ARSTREAM2_RtpSender_t *sender, uint8_t type, const char *prefix, char **value, uint32_t *sendInterval)
{
    int k, found;

    if ((sender == NULL) || (value == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((type == ARSTREAM2_RTCP_SDES_PRIV_ITEM) && (prefix == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    for (k = 0, found = 0; k < sender->rtcpSenderContext.sdesItemCount; k++)
    {
        if (type == sender->rtcpSenderContext.sdesItem[k].type)
        {
            if (type == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
            {
                if (!strncmp(prefix, sender->rtcpSenderContext.sdesItem[k].prefix, 256))
                {
                    *value = sender->rtcpSenderContext.sdesItem[k].value;
                    if (sendInterval) *sendInterval = sender->rtcpSenderContext.sdesItem[k].sendTimeInterval;
                    found = 1;
                    break;
                }
            }
            else
            {
                *value = sender->rtcpSenderContext.sdesItem[k].value;
                if (sendInterval) *sendInterval = sender->rtcpSenderContext.sdesItem[k].sendTimeInterval;
                found = 1;
                break;
            }
        }
    }

    return (found) ? ARSTREAM2_OK : ARSTREAM2_ERROR_NOT_FOUND;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_SetSdesItem(ARSTREAM2_RtpSender_t *sender, uint8_t type, const char *prefix, const char *value, uint32_t sendInterval)
{
    int k, found;

    if ((sender == NULL) || (value == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((type == ARSTREAM2_RTCP_SDES_PRIV_ITEM) && (prefix == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    for (k = 0, found = 0; k < sender->rtcpSenderContext.sdesItemCount; k++)
    {
        if (type == sender->rtcpSenderContext.sdesItem[k].type)
        {
            if (type == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
            {
                if (!strncmp(prefix, sender->rtcpSenderContext.sdesItem[k].prefix, 256))
                {
                    snprintf(sender->rtcpSenderContext.sdesItem[k].value, 256, "%s", value);
                    sender->rtcpSenderContext.sdesItem[k].sendTimeInterval = sendInterval;
                    sender->rtcpSenderContext.sdesItem[k].lastSendTime = 0;
                    found = 1;
                    break;
                }
            }
            else
            {
                snprintf(sender->rtcpSenderContext.sdesItem[k].value, 256, "%s", value);
                sender->rtcpSenderContext.sdesItem[k].sendTimeInterval = sendInterval;
                sender->rtcpSenderContext.sdesItem[k].lastSendTime = 0;
                found = 1;
                break;
            }
        }
    }

    if (!found)
    {
        if (sender->rtcpSenderContext.sdesItemCount >= ARSTREAM2_RTCP_SDES_ITEM_MAX_COUNT)
        {
            return ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            k = sender->rtcpSenderContext.sdesItemCount;
            sender->rtcpSenderContext.sdesItem[k].type = type;
            snprintf(sender->rtcpSenderContext.sdesItem[k].value, 256, "%s", value);
            if (type == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
            {
                snprintf(sender->rtcpSenderContext.sdesItem[k].prefix, 256, "%s", prefix);
            }
            sender->rtcpSenderContext.sdesItem[k].sendTimeInterval = sendInterval;
            sender->rtcpSenderContext.sdesItem[k].lastSendTime = 0;
            sender->rtcpSenderContext.sdesItemCount++;
        }
    }

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_GetPeerSdesItem(ARSTREAM2_RtpSender_t *sender, uint8_t type, const char *prefix, char **value)
{
    int k, found;

    if ((sender == NULL) || (value == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((type == ARSTREAM2_RTCP_SDES_PRIV_ITEM) && (prefix == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    for (k = 0, found = 0; k < sender->rtcpSenderContext.peerSdesItemCount; k++)
    {
        if (type == sender->rtcpSenderContext.peerSdesItem[k].type)
        {
            if (type == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
            {
                if (!strncmp(prefix, sender->rtcpSenderContext.peerSdesItem[k].prefix, 256))
                {
                    *value = sender->rtcpSenderContext.peerSdesItem[k].value;
                    found = 1;
                    break;
                }
            }
            else
            {
                *value = sender->rtcpSenderContext.peerSdesItem[k].value;
                found = 1;
                break;
            }
        }
    }

    return (found) ? ARSTREAM2_OK : ARSTREAM2_ERROR_NOT_FOUND;
}


eARSTREAM2_ERROR ARSTREAM2_RtpSender_GetMonitoring(ARSTREAM2_RtpSender_t *sender, uint64_t startTime, uint32_t timeIntervalUs,
                                                   ARSTREAM2_StreamSender_MonitoringData_t *monitoringData)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    uint64_t endTime, curTime, previousTime, acqToNetworkSum = 0, networkSum = 0, acqToNetworkVarSum = 0, networkVarSum = 0, packetSizeVarSum = 0;
    uint32_t bytesSent, bytesDropped, bytesSentSum = 0, bytesDroppedSum = 0, meanPacketSize = 0, acqToNetworkTime = 0, networkTime = 0;
    uint32_t acqToNetworkJitter = 0, networkJitter = 0, meanAcqToNetworkTime = 0, meanNetworkTime = 0, packetSizeStdDev = 0;
    uint32_t minAcqToNetworkTime = (uint32_t)(-1), minNetworkTime = (uint32_t)(-1), minPacketSize = (uint32_t)(-1);
    uint32_t maxAcqToNetworkTime = 0, maxNetworkTime = 0, maxPacketSize = 0;
    int points = 0, usefulPoints = 0, packetsSent = 0, packetsDropped = 0, idx, i, firstUsefulIdx = -1;

    if ((sender == NULL) || (timeIntervalUs == 0) || (monitoringData == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (startTime == 0)
    {
        struct timespec t1;
        ARSAL_Time_GetTime(&t1);
        startTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
    }
    endTime = startTime;

    ARSAL_Mutex_Lock(&(sender->monitoringMutex));

    if (sender->monitoringCount > 0)
    {
        idx = sender->monitoringIndex;
        previousTime = startTime;

        while (points < sender->monitoringCount)
        {
            curTime = sender->monitoringPoint[idx].outputTimestamp;
            if (curTime > startTime)
            {
                points++;
                idx = (idx - 1 >= 0) ? idx - 1 : ARSTREAM2_RTP_SENDER_MONITORING_MAX_POINTS - 1;
                continue;
            }
            if (startTime - curTime > timeIntervalUs)
            {
                break;
            }
            if (firstUsefulIdx == -1)
            {
                firstUsefulIdx = idx;
            }
            bytesSent = sender->monitoringPoint[idx].bytesSent;
            bytesSentSum += bytesSent;
            if (bytesSent)
            {
                packetsSent++;
                acqToNetworkTime = curTime - sender->monitoringPoint[idx].ntpTimestamp;
                acqToNetworkSum += acqToNetworkTime;
                networkTime = curTime - sender->monitoringPoint[idx].inputTimestamp;
                networkSum += networkTime;
                if (acqToNetworkTime < minAcqToNetworkTime) minAcqToNetworkTime = acqToNetworkTime;
                if (acqToNetworkTime > maxAcqToNetworkTime) maxAcqToNetworkTime = acqToNetworkTime;
                if (networkTime < minNetworkTime) minNetworkTime = networkTime;
                if (networkTime > maxNetworkTime) maxNetworkTime = networkTime;
                if (bytesSent < minPacketSize) minPacketSize = bytesSent;
                if (bytesSent > maxPacketSize) maxPacketSize = bytesSent;
            }
            bytesDropped = sender->monitoringPoint[idx].bytesDropped;
            bytesDroppedSum += bytesDropped;
            if (bytesDropped)
            {
                packetsDropped++;
            }
            previousTime = curTime;
            usefulPoints++;
            points++;
            idx = (idx - 1 >= 0) ? idx - 1 : ARSTREAM2_RTP_SENDER_MONITORING_MAX_POINTS - 1;
        }

        endTime = previousTime;
        meanPacketSize = (packetsSent > 0) ? (bytesSentSum / packetsSent) : 0;
        meanAcqToNetworkTime = (packetsSent > 0) ? (uint32_t)(acqToNetworkSum / packetsSent) : 0;
        meanNetworkTime = (packetsSent > 0) ? (uint32_t)(networkSum / packetsSent) : 0;

        for (i = 0, idx = firstUsefulIdx; i < usefulPoints; i++)
        {
            idx = (idx - 1 >= 0) ? idx - 1 : ARSTREAM2_RTP_SENDER_MONITORING_MAX_POINTS - 1;
            curTime = sender->monitoringPoint[idx].outputTimestamp;
            bytesSent = sender->monitoringPoint[idx].bytesSent;
            if (bytesSent)
            {
                acqToNetworkTime = curTime - sender->monitoringPoint[idx].ntpTimestamp;
                networkTime = curTime - sender->monitoringPoint[idx].inputTimestamp;
                packetSizeVarSum += ((bytesSent - meanPacketSize) * (bytesSent - meanPacketSize));
                acqToNetworkVarSum += ((acqToNetworkTime - meanAcqToNetworkTime) * (acqToNetworkTime - meanAcqToNetworkTime));
                networkVarSum += ((networkTime - meanNetworkTime) * (networkTime - meanNetworkTime));
            }
        }
        acqToNetworkJitter = (packetsSent > 0) ? (uint32_t)(sqrt((double)acqToNetworkVarSum / packetsSent)) : 0;
        networkJitter = (packetsSent > 0) ? (uint32_t)(sqrt((double)networkVarSum / packetsSent)) : 0;
        packetSizeStdDev = (packetsSent > 0) ? (uint32_t)(sqrt((double)packetSizeVarSum / packetsSent)) : 0;
    }

    ARSAL_Mutex_Unlock(&(sender->monitoringMutex));

    monitoringData->startTimestamp = endTime;
    monitoringData->timeInterval = (startTime - endTime);
    monitoringData->acqToNetworkTimeMin = minAcqToNetworkTime;
    monitoringData->acqToNetworkTimeMax = maxAcqToNetworkTime;
    monitoringData->acqToNetworkTimeMean = meanAcqToNetworkTime;
    monitoringData->acqToNetworkTimeJitter = acqToNetworkJitter;
    monitoringData->networkTimeMin = minNetworkTime;
    monitoringData->networkTimeMax = maxNetworkTime;
    monitoringData->networkTimeMean = meanNetworkTime;
    monitoringData->networkTimeJitter = networkJitter;
    monitoringData->bytesSent = bytesSentSum;
    monitoringData->packetSizeMin = minPacketSize;
    monitoringData->packetSizeMax = maxPacketSize;
    monitoringData->packetSizeMean = meanPacketSize;
    monitoringData->packetSizeStdDev = packetSizeStdDev;
    monitoringData->packetsSent = packetsSent;
    monitoringData->bytesDropped = bytesDroppedSum;
    monitoringData->packetsDropped = packetsDropped;

    return ret;
}
