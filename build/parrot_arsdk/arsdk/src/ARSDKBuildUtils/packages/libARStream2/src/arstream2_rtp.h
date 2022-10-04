/**
 * @file arstream2_rtp.h
 * @brief Parrot Streaming Library - RTP implementation
 * @date 04/17/2015
 * @author aurelien.barre@parrot.com
 */

#ifndef _ARSTREAM2_RTP_H_
#define _ARSTREAM2_RTP_H_

#include <inttypes.h>
#define __USE_GNU
#include <sys/socket.h>
#undef __USE_GNU

#include "arstream2_rtcp.h"


/*
 * Macros
 */

#define ARSTREAM2_RTP_IP_HEADER_SIZE 20
#define ARSTREAM2_RTP_UDP_HEADER_SIZE 8

#define ARSTREAM2_RTP_SENDER_SSRC 0x41525353
#define ARSTREAM2_RTP_RECEIVER_SSRC 0x41525352

#define ARSTREAM2_RTP_CLOCKSKEW_WINDOW_SIZE 400
#define ARSTREAM2_RTP_CLOCKSKEW_WINDOW_TIMEOUT 5000000
#define ARSTREAM2_RTP_CLOCKSKEW_AVG_ALPHA 64


/*
 * Types
 */

#ifndef HAS_MMSG
struct mmsghdr {
    struct msghdr msg_hdr;  /* Message header */
    unsigned int  msg_len;  /* Number of received bytes for header */
};
#else
struct mmsghdr;
#endif


/**
 * @brief RTP Header (see RFC3550)
 */
typedef struct {
    uint16_t flags;
    uint16_t seqNum;
    uint32_t timestamp;
    uint32_t ssrc;
} __attribute__ ((packed)) ARSTREAM2_RTP_Header_t;

#define ARSTREAM2_RTP_TOTAL_HEADERS_SIZE (sizeof(ARSTREAM2_RTP_Header_t) + ARSTREAM2_RTP_UDP_HEADER_SIZE + ARSTREAM2_RTP_IP_HEADER_SIZE)
#define ARSTREAM2_RTP_MAX_PAYLOAD_SIZE (0xFFFF - ARSTREAM2_RTP_TOTAL_HEADERS_SIZE)


/**
 * @brief Source description item
 */
typedef struct ARSTREAM2_RTP_RtpStats_s
{
    int8_t rssi;
    struct {
        uint64_t timestamp;
        uint32_t sentPacketCount;
        uint32_t droppedPacketCount;
        uint64_t sentByteIntegral;
        uint64_t sentByteIntegralSq;
        uint64_t droppedByteIntegral;
        uint64_t droppedByteIntegralSq;
        uint64_t inputToSentTimeIntegral;
        uint64_t inputToSentTimeIntegralSq;
        uint64_t inputToDroppedTimeIntegral;
        uint64_t inputToDroppedTimeIntegralSq;
    } senderStats;
    struct {
        uint64_t timestamp;
        uint32_t lastInterval;
        uint32_t intervalPacketCount;
        uint32_t intervalByteCount;
    } senderReport;
    struct {
        uint64_t timestamp;
        uint32_t roundTripDelay;
        uint32_t interarrivalJitter;
        uint32_t receiverLostCount;
        uint32_t receiverFractionLost;
        uint32_t receiverExtHighestSeqNum;
    } receiverReport;
    struct {
        uint64_t timestamp;
        uint16_t startSeqNum;
        uint16_t endSeqNum;
        uint32_t *receivedFlag;
    } lossReport;
    struct {
        uint64_t timestamp;
        uint32_t djbNominal;
        uint32_t djbMax;
        uint32_t djbHighWatermark;
        uint32_t djbLowWatermark;
    } djbMetricsReport;
    struct {
        int64_t peerClockDelta;
        uint32_t roundTripDelay;
        uint32_t peer2meDelay;
        uint32_t me2peerDelay;
    } clockDelta;

} ARSTREAM2_RTP_RtpStats_t;


/**
 * @brief Access unit FIFO buffer pool item
 */
typedef struct ARSTREAM2_RTP_PacketFifoBuffer_s
{
    uint8_t *buffer;
    unsigned int bufferSize;
    uint8_t *header;
    unsigned int headerSize;
    struct iovec msgIov[3];

    unsigned int refCount;
    struct ARSTREAM2_RTP_PacketFifoBuffer_s* prev;
    struct ARSTREAM2_RTP_PacketFifoBuffer_s* next;

} ARSTREAM2_RTP_PacketFifoBuffer_t;


/**
 * @brief RTP packet data
 */
typedef struct ARSTREAM2_RTP_Packet_s
{
    ARSTREAM2_RTP_PacketFifoBuffer_t *buffer;
    uint64_t inputTimestamp;
    uint64_t timeoutTimestamp;
    uint64_t ntpTimestamp;
    uint64_t ntpTimestampUnskewed;
    uint64_t ntpTimestampRaw;
    uint64_t ntpTimestampRawUnskewed;
    uint64_t ntpTimestampLocal;
    uint64_t extRtpTimestamp;
    uint32_t rtpTimestamp;
    uint16_t seqNum;
    uint32_t extSeqNum;
    uint32_t markerBit;
    ARSTREAM2_RTP_Header_t *header;
    uint8_t *headerExtension;
    unsigned int headerExtensionSize;
    uint8_t *payload;
    unsigned int payloadSize;
    uint32_t importance;
    uint32_t priority;
    size_t msgIovLength;

} ARSTREAM2_RTP_Packet_t;


/**
 * @brief RTP packet FIFO item
 */
typedef struct ARSTREAM2_RTP_PacketFifoItem_s
{
    ARSTREAM2_RTP_Packet_t packet;

    struct ARSTREAM2_RTP_PacketFifoItem_s* prev;
    struct ARSTREAM2_RTP_PacketFifoItem_s* next;

} ARSTREAM2_RTP_PacketFifoItem_t;


/**
 * @brief Access unit FIFO queue
 */
typedef struct ARSTREAM2_RTP_PacketFifoQueue_s
{
    int count;
    ARSTREAM2_RTP_PacketFifoItem_t *head;
    ARSTREAM2_RTP_PacketFifoItem_t *tail;

    struct ARSTREAM2_RTP_PacketFifoQueue_s* prev;
    struct ARSTREAM2_RTP_PacketFifoQueue_s* next;

} ARSTREAM2_RTP_PacketFifoQueue_t;


/**
 * @brief RTP packet FIFO
 */
typedef struct ARSTREAM2_RTP_PacketFifo_s
{
    int queueCount;
    ARSTREAM2_RTP_PacketFifoQueue_t *queue;
    int itemPoolSize;
    ARSTREAM2_RTP_PacketFifoItem_t *itemPool;
    ARSTREAM2_RTP_PacketFifoItem_t *itemFree;
    int bufferPoolSize;
    ARSTREAM2_RTP_PacketFifoBuffer_t *bufferPool;
    ARSTREAM2_RTP_PacketFifoBuffer_t *bufferFree;

} ARSTREAM2_RTP_PacketFifo_t;


typedef void (*ARSTREAM2_RTP_SenderMonitoringCallback_t)(uint64_t inputTimestamp, uint64_t outputTimestamp,
                                                         uint64_t ntpTimestamp, uint32_t rtpTimestamp,
                                                         uint16_t seqNum, uint16_t markerBit,
                                                         uint32_t importance, uint32_t priority,
                                                         uint32_t bytesSent, uint32_t bytesDropped, void *userPtr);


/**
 * @brief RTP sender context
 */
typedef struct ARSTREAM2_RTP_SenderContext_s
{
    uint32_t senderSsrc;
    uint32_t rtpClockRate;
    uint32_t rtpTimestampOffset;
    uint32_t maxPacketSize;
    uint32_t targetPacketSize;
    uint16_t seqNum;
    uint32_t packetCount;
    uint64_t byteCount;
    int useRtpHeaderExtensions;

    uint64_t previousTimestamp;
    void *previousAuUserPtr;
    int stapPending;
    ARSTREAM2_RTP_PacketFifoItem_t *stapItem;
    int stapFirstNalu;
    uint64_t stapNtpTimestamp;
    uint64_t stapInputTimestamp;
    uint64_t stapTimeoutTimestamp;
    int stapSeqNumForcedDiscontinuity;
    uint8_t stapMaxNri;
    uint32_t stapImportance;
    uint32_t stapPriority;
    unsigned int stapOffsetInBuffer;
    uint8_t *stapPayload;
    uint8_t *stapHeaderExtension;
    unsigned int stapPayloadSize;
    unsigned int stapHeaderExtensionSize;

    uint32_t sentPacketCount;
    uint32_t droppedPacketCount;
    uint64_t sentByteIntegral;
    uint64_t sentByteIntegralSq;
    uint64_t droppedByteIntegral;
    uint64_t droppedByteIntegralSq;
    uint64_t inputToSentTimeIntegral;
    uint64_t inputToSentTimeIntegralSq;
    uint64_t inputToDroppedTimeIntegral;
    uint64_t inputToDroppedTimeIntegralSq;

    void *auCallback;
    void *auCallbackUserPtr;
    uint64_t lastAuCallbackTimestamp;
    void *naluCallback;
    void *naluCallbackUserPtr;
    ARSTREAM2_RTP_SenderMonitoringCallback_t monitoringCallback;
    void *monitoringCallbackUserPtr;

} ARSTREAM2_RTP_SenderContext_t;


/**
 * @brief RTP sender context
 */
typedef struct ARSTREAM2_RTP_ReceiverContext_s
{
    uint32_t rtpClockRate;
    uint32_t maxPacketSize;
    uint32_t nominalDelay;
    uint64_t extHighestRtpTimestamp;
    uint32_t extHighestSeqNum;
    int32_t previousExtSeqNum;
    uint64_t previousExtRtpTimestamp;
    uint64_t previousRecvRtpTimestamp;
    uint64_t firstExtRtpTimestamp;
    uint64_t firstRecvRtpTimestamp;
    int64_t clockSkewWindow[ARSTREAM2_RTP_CLOCKSKEW_WINDOW_SIZE];
    int clockSkewWindowSize;
    uint64_t clockSkewWindowStartTimestamp;
    int clockSkewInit;
    int64_t clockSkewOffset;
    int64_t clockSkewMin;
    int64_t clockSkewMinAvg;
    int64_t clockSkew;

} ARSTREAM2_RTP_ReceiverContext_t;


/*
 * Functions
 */

void ARSTREAM2_RTP_PacketReset(ARSTREAM2_RTP_Packet_t *packet);

void ARSTREAM2_RTP_PacketCopy(ARSTREAM2_RTP_Packet_t *dst, const ARSTREAM2_RTP_Packet_t *src);

int ARSTREAM2_RTP_PacketFifoInit(ARSTREAM2_RTP_PacketFifo_t *fifo, int itemMaxCount, int bufferMaxCount, int packetBufferSize);

int ARSTREAM2_RTP_PacketFifoFree(ARSTREAM2_RTP_PacketFifo_t *fifo);

int ARSTREAM2_RTP_PacketFifoAddQueue(ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoQueue_t *queue);

int ARSTREAM2_RTP_PacketFifoRemoveQueue(ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoQueue_t *queue);

ARSTREAM2_RTP_PacketFifoBuffer_t* ARSTREAM2_RTP_PacketFifoGetBuffer(ARSTREAM2_RTP_PacketFifo_t *fifo);

int ARSTREAM2_RTP_PacketFifoBufferAddRef(ARSTREAM2_RTP_PacketFifoBuffer_t *buffer);

int ARSTREAM2_RTP_PacketFifoUnrefBuffer(ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoBuffer_t *buffer);

ARSTREAM2_RTP_PacketFifoItem_t* ARSTREAM2_RTP_PacketFifoPopFreeItem(ARSTREAM2_RTP_PacketFifo_t *fifo);

int ARSTREAM2_RTP_PacketFifoPushFreeItem(ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoItem_t *item);

int ARSTREAM2_RTP_PacketFifoEnqueueItem(ARSTREAM2_RTP_PacketFifoQueue_t *queue, ARSTREAM2_RTP_PacketFifoItem_t *item);

int ARSTREAM2_RTP_PacketFifoEnqueueItemOrderedByPriority(ARSTREAM2_RTP_PacketFifoQueue_t *queue, ARSTREAM2_RTP_PacketFifoItem_t *item);

int ARSTREAM2_RTP_PacketFifoEnqueueItemOrderedBySeqNum(ARSTREAM2_RTP_PacketFifoQueue_t *queue, ARSTREAM2_RTP_PacketFifoItem_t *item);

ARSTREAM2_RTP_PacketFifoItem_t* ARSTREAM2_RTP_PacketFifoDequeueItem(ARSTREAM2_RTP_PacketFifoQueue_t *queue);

ARSTREAM2_RTP_PacketFifoItem_t* ARSTREAM2_RTP_PacketFifoPeekItem(ARSTREAM2_RTP_PacketFifoQueue_t *queue);

ARSTREAM2_RTP_PacketFifoItem_t* ARSTREAM2_RTP_PacketFifoDuplicateItem(ARSTREAM2_RTP_PacketFifo_t *fifo,
                                                                      ARSTREAM2_RTP_PacketFifoItem_t *item);

int ARSTREAM2_RTP_Sender_PacketFifoFillMsgVec(ARSTREAM2_RTP_PacketFifoQueue_t *queue, struct mmsghdr *msgVec, unsigned int msgVecCount, void *msgName, socklen_t msgNamelen);

int ARSTREAM2_RTP_Sender_PacketFifoCleanFromMsgVec(ARSTREAM2_RTP_SenderContext_t *context,
                                                   ARSTREAM2_RTP_PacketFifo_t *fifo,
                                                   ARSTREAM2_RTP_PacketFifoQueue_t *queue,
                                                   struct mmsghdr *msgVec, unsigned int msgVecCount, uint64_t curTime);

int ARSTREAM2_RTP_Sender_PacketFifoCleanFromTimeout(ARSTREAM2_RTP_SenderContext_t *context,
                                                    ARSTREAM2_RTP_PacketFifo_t *fifo,
                                                    ARSTREAM2_RTP_PacketFifoQueue_t *queue, uint64_t curTime,
                                                    unsigned int *dropCount, unsigned int importanceLevelCount);

int ARSTREAM2_RTP_Sender_PacketFifoRandomDrop(ARSTREAM2_RTP_SenderContext_t *context,
                                              ARSTREAM2_RTP_PacketFifo_t *fifo,
                                              ARSTREAM2_RTP_PacketFifoQueue_t *queue, float ratio, uint64_t curTime);

int ARSTREAM2_RTP_Sender_PacketFifoFlushQueue(ARSTREAM2_RTP_SenderContext_t *context,
                                              ARSTREAM2_RTP_PacketFifo_t *fifo,
                                              ARSTREAM2_RTP_PacketFifoQueue_t *queue, uint64_t curTime);

int ARSTREAM2_RTP_Sender_PacketFifoFlush(ARSTREAM2_RTP_SenderContext_t *context,
                                         ARSTREAM2_RTP_PacketFifo_t *fifo, uint64_t curTime);

int ARSTREAM2_RTP_Sender_GeneratePacket(ARSTREAM2_RTP_SenderContext_t *context, ARSTREAM2_RTP_Packet_t *packet,
                                        uint8_t *payload, unsigned int payloadSize,
                                        uint8_t *headerExtension, unsigned int headerExtensionSize,
                                        uint64_t ntpTimestamp, uint64_t inputTimestamp,
                                        uint64_t timeoutTimestamp, uint16_t seqNum, uint32_t markerBit,
                                        uint32_t importance, uint32_t priority);

int ARSTREAM2_RTP_Sender_FinishPacket(ARSTREAM2_RTP_SenderContext_t *context, ARSTREAM2_RTP_Packet_t *packet, uint64_t curTime, int dropped);

/* WARNING: the call sequence ARSTREAM2_RTP_Receiver_PacketFifoFillMsgVec -> recvmmsg -> ARSTREAM2_RTP_Receiver_PacketFifoAddFromMsgVec
   must not be broken (no change made to the free items list) */
int ARSTREAM2_RTP_Receiver_PacketFifoFillMsgVec(ARSTREAM2_RTP_PacketFifo_t *fifo, struct mmsghdr *msgVec, unsigned int msgVecCount);

/* WARNING: the call sequence ARSTREAM2_RTP_Receiver_PacketFifoFillMsgVec -> recvmmsg -> ARSTREAM2_RTP_Receiver_PacketFifoAddFromMsgVec
   must not be broken (no change made to the free items list) */
int ARSTREAM2_RTP_Receiver_PacketFifoAddFromMsgVec(ARSTREAM2_RTP_ReceiverContext_t *context,
                                                   ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoQueue_t *queue,
                                                   ARSTREAM2_RTP_PacketFifoQueue_t **resendQueue, uint32_t *resendTimeout, unsigned int resendCount,
                                                   struct mmsghdr *msgVec, unsigned int msgVecCount, uint64_t curTime,
                                                   ARSTREAM2_RTCP_ReceiverContext_t *rtcpContext);

int ARSTREAM2_RTP_Receiver_PacketFifoFlushQueue(ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoQueue_t *queue);

int ARSTREAM2_RTP_Receiver_PacketFifoFlush(ARSTREAM2_RTP_PacketFifo_t *fifo);

#endif /* _ARSTREAM2_RTP_H_ */
