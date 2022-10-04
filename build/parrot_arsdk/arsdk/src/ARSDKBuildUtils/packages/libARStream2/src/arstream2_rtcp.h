/**
 * @file arstream2_rtcp.h
 * @brief Parrot Streaming Library - RTCP implementation
 * @date 04/06/2016
 * @author aurelien.barre@parrot.com
 */

#ifndef _ARSTREAM2_RTCP_H_
#define _ARSTREAM2_RTCP_H_

#include <inttypes.h>
#include "arstream2_h264.h"


/*
 * Macros
 */

#define ARSTREAM2_RTCP_SENDER_REPORT_PACKET_TYPE 200
#define ARSTREAM2_RTCP_RECEIVER_REPORT_PACKET_TYPE 201
#define ARSTREAM2_RTCP_SDES_PACKET_TYPE 202
#define ARSTREAM2_RTCP_BYE_PACKET_TYPE 203
#define ARSTREAM2_RTCP_APP_PACKET_TYPE 204
#define ARSTREAM2_RTCP_EXTENDED_REPORT_PACKET_TYPE 207

#define ARSTREAM2_RTCP_SDES_CNAME_ITEM 1
#define ARSTREAM2_RTCP_SDES_NAME_ITEM 2
#define ARSTREAM2_RTCP_SDES_EMAIL_ITEM 3
#define ARSTREAM2_RTCP_SDES_PHONE_ITEM 4
#define ARSTREAM2_RTCP_SDES_LOC_ITEM 5
#define ARSTREAM2_RTCP_SDES_TOOL_ITEM 6
#define ARSTREAM2_RTCP_SDES_NOTE_ITEM 7
#define ARSTREAM2_RTCP_SDES_PRIV_ITEM 8

#define ARSTREAM2_RTCP_APP_PACKET_NAME 0x41525354
#define ARSTREAM2_RTCP_APP_PACKET_CLOCKDELTA_SUBTYPE 1
#define ARSTREAM2_RTCP_APP_PACKET_VIDEOSTATS_SUBTYPE 2

#define ARSTREAM2_RTCP_SENDER_DEFAULT_BITRATE 25000
#define ARSTREAM2_RTCP_RECEIVER_DEFAULT_BITRATE 25000
#define ARSTREAM2_RTCP_SENDER_BANDWIDTH_SHARE 0.025
#define ARSTREAM2_RTCP_RECEIVER_BANDWIDTH_SHARE 0.025
#define ARSTREAM2_RTCP_SENDER_MIN_PACKET_TIME_INTERVAL 100000
#define ARSTREAM2_RTCP_RECEIVER_MIN_PACKET_TIME_INTERVAL 100000

#define ARSTREAM2_RTCP_LOSS_REPORT_INITIAL_WORD_COUNT 32

#define ARSTREAM2_RTCP_LOSS_RLE_REPORT_BLOCK_TYPE 1
#define ARSTREAM2_RTCP_DJB_METRICS_REPORT_BLOCK_TYPE 23

#define ARSTREAM2_RTCP_VIDEOSTATS_VERSION (1)

#define ARSTREAM2_RTCP_CLOCKDELTA_MIN_TS_DELTA 1000
#define ARSTREAM2_RTCP_CLOCKDELTA_TIMEOUT 1000000
#define ARSTREAM2_RTCP_CLOCKDELTA_WINDOW_SIZE 10
#define ARSTREAM2_RTCP_CLOCKDELTA_WINDOW_TIMEOUT 1000000
#define ARSTREAM2_RTCP_CLOCKDELTA_MAX_RTDELAY 500000
#define ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA 16
#define ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA_LONG 64
#define ARSTREAM2_RTCP_CLOCKDELTA_ONE_WAY_DELAY_DIFF_THRES 0.5

#define ARSTREAM2_RTCP_SDES_ITEM_MAX_COUNT 10


/*
 * Types
 */

/**
 * @brief RTCP Sender Report (SR) Packet (see RFC3550)
 */
typedef struct {
    uint8_t flags;
    uint8_t packetType;
    uint16_t length;
    uint32_t ssrc;
    uint32_t ntpTimestampH;
    uint32_t ntpTimestampL;
    uint32_t rtpTimestamp;
    uint32_t senderPacketCount;
    uint32_t senderByteCount;
} __attribute__ ((packed)) ARSTREAM2_RTCP_SenderReport_t;

/**
 * @brief RTCP Reception Report Block (see RFC3550)
 */
typedef struct {
    uint32_t ssrc;
    uint32_t lost;
    uint32_t extHighestSeqNum;
    uint32_t interarrivalJitter;
    uint32_t lsr;
    uint32_t dlsr;
} __attribute__ ((packed)) ARSTREAM2_RTCP_ReceptionReportBlock_t;

/**
 * @brief RTCP Receiver Report (RR) Packet (see RFC3550)
 */
typedef struct {
    uint8_t flags;
    uint8_t packetType;
    uint16_t length;
    uint32_t ssrc;
} __attribute__ ((packed)) ARSTREAM2_RTCP_ReceiverReport_t;

/**
 * @brief RTCP Source Description (SDES) Packet (see RFC3550)
 */
typedef struct {
    uint8_t flags;
    uint8_t packetType;
    uint16_t length;
} __attribute__ ((packed)) ARSTREAM2_RTCP_Sdes_t;

/**
 * @brief RTCP Goodbye (BYE) Packet (see RFC3550)
 */
typedef struct {
    uint8_t flags;
    uint8_t packetType;
    uint16_t length;
    uint32_t ssrc;
} __attribute__ ((packed)) ARSTREAM2_RTCP_Goodbye_t;

/**
 * @brief RTCP Extended Report (XR) Packet (see RFC3611)
 */
typedef struct {
    uint8_t flags;
    uint8_t packetType;
    uint16_t length;
    uint32_t ssrc;
} __attribute__ ((packed)) ARSTREAM2_RTCP_ExtendedReport_t;

/**
 * @brief RTCP Extended - Loss RLE Report Block (see RFC3611)
 */
typedef struct {
    uint8_t blockType;
    uint8_t thinning;
    uint16_t length;
    uint32_t ssrc;
    uint16_t beginSeq;
    uint16_t endSeq;
} __attribute__ ((packed)) ARSTREAM2_RTCP_LossRleReportBlock_t;

/**
 * @brief RTCP Extended - De-jitter Buffer Metrics Report Block (see RFC7005)
 */
typedef struct {
    uint8_t blockType;
    uint8_t flags;
    uint16_t length;
    uint32_t ssrc;
    uint16_t djbNominal;
    uint16_t djbMax;
    uint16_t djbHighWatermark;
    uint16_t djbLowWatermark;
} __attribute__ ((packed)) ARSTREAM2_RTCP_DjbMetricsReportBlock_t;

/**
 * @brief RTCP Application-Defined (APP) Packet (see RFC3550)
 */
typedef struct {
    uint8_t flags;
    uint8_t packetType;
    uint16_t length;
    uint32_t ssrc;
    uint32_t name;
} __attribute__ ((packed)) ARSTREAM2_RTCP_Application_t;

/**
 * @brief Application defined clock delta data
 */
typedef struct {
    uint32_t originateTimestampH;
    uint32_t originateTimestampL;
    uint32_t receiveTimestampH;
    uint32_t receiveTimestampL;
    uint32_t transmitTimestampH;
    uint32_t transmitTimestampL;
} __attribute__ ((packed)) ARSTREAM2_RTCP_ClockDelta_t;

/**
 * @brief Application defined video stats data
 */
typedef struct {
    uint8_t version;
    int8_t rssi;
    uint8_t reserved1;
    uint8_t reserved2;
    uint32_t timestampH;
    uint32_t timestampL;
    uint32_t totalFrameCount;
    uint32_t outputFrameCount;
    uint32_t erroredOutputFrameCount;
    uint32_t missedFrameCount;
    uint32_t discardedFrameCount;
    uint32_t timestampDeltaIntegralH;
    uint32_t timestampDeltaIntegralL;
    uint32_t timestampDeltaIntegralSqH;
    uint32_t timestampDeltaIntegralSqL;
    uint32_t timingErrorIntegralH;
    uint32_t timingErrorIntegralL;
    uint32_t timingErrorIntegralSqH;
    uint32_t timingErrorIntegralSqL;
    uint32_t estimatedLatencyIntegralH;
    uint32_t estimatedLatencyIntegralL;
    uint32_t estimatedLatencyIntegralSqH;
    uint32_t estimatedLatencyIntegralSqL;
    uint32_t erroredSecondCount;
    uint32_t mbStatusClassCount;
    uint32_t mbStatusZoneCount;
    //uint32_t erroredSecondCountByZone[mbStatusZoneCount];
    //uint32_t macroblockStatus[mbStatusClassCount][mbStatusZoneCount];
} __attribute__ ((packed)) ARSTREAM2_RTCP_VideoStats_t;

/**
 * @brief Source description item
 */
typedef struct ARSTREAM2_RTCP_SdesItem_s {
    uint8_t type;
    char prefix[256];
    char value[256];
    uint32_t sendTimeInterval;
    uint64_t lastSendTime;
} ARSTREAM2_RTCP_SdesItem_t;

/**
 * @brief RTP loss report context
 */
typedef struct ARSTREAM2_RTCP_LossReportContext_s
{
    int count;
    uint32_t startSeqNum;
    uint32_t endSeqNum;
    uint32_t *receivedFlag;
    int wordCount;
    uint64_t lastSendTime;
    uint32_t sendTimeInterval;
    uint64_t lastReceptionTimestamp;

} ARSTREAM2_RTCP_LossReportContext_t;

/**
 * @brief RTP de-jitter buffer metrics report context
 */
typedef struct ARSTREAM2_RTCP_DjbReportContext_s
{
    int djbMetricsAvailable;
    uint32_t djbNominal;
    uint32_t djbMax;
    uint32_t djbHighWatermark;
    uint32_t djbLowWatermark;
    uint64_t lastSendTime;
    uint32_t sendTimeInterval;
    uint64_t lastReceptionTimestamp;

} ARSTREAM2_RTCP_DjbReportContext_t;

/**
 * @brief Application clock delta context
 */
typedef struct ARSTREAM2_RTCP_ClockDeltaContext_s {
    uint64_t expectedOriginateTimestamp;
    uint64_t nextPeerOriginateTimestamp;
    uint64_t nextReceiveTimestamp;
    int64_t clockDeltaWindow[ARSTREAM2_RTCP_CLOCKDELTA_WINDOW_SIZE];
    int64_t rtDelayWindow[ARSTREAM2_RTCP_CLOCKDELTA_WINDOW_SIZE];
    int windowSize;
    uint64_t windowStartTimestamp;
    int64_t clockDelta;
    int64_t clockDeltaAvg;
    int64_t rtDelayAvg;
    int64_t rtDelayMinAvg;
    int64_t p2mDelayAvg;
    int64_t m2pDelayAvg;
} ARSTREAM2_RTCP_ClockDeltaContext_t;

/**
 * @brief Application video stats context
 */
typedef struct ARSTREAM2_RTCP_VideoStatsContext_s {
    ARSTREAM2_H264_VideoStats_t videoStats;
    uint64_t lastSendTime;
    uint32_t sendTimeInterval;
    int updatedSinceLastTime;
} ARSTREAM2_RTCP_VideoStatsContext_t;

/**
 * @brief RTCP sender context
 */
typedef struct ARSTREAM2_RTCP_SenderContext_s {
    uint32_t senderSsrc;
    uint32_t receiverSsrc;
    uint32_t rtcpByteRate;
    ARSTREAM2_RTCP_SdesItem_t sdesItem[ARSTREAM2_RTCP_SDES_ITEM_MAX_COUNT];
    int sdesItemCount;
    ARSTREAM2_RTCP_SdesItem_t peerSdesItem[ARSTREAM2_RTCP_SDES_ITEM_MAX_COUNT];
    int peerSdesItemCount;

    uint32_t rtpClockRate;
    uint32_t rtpTimestampOffset;

    uint64_t lastRrReceptionTimestamp;
    uint32_t roundTripDelay;
    uint32_t interarrivalJitter;
    uint32_t receiverFractionLost;
    uint32_t receiverLostCount;
    uint32_t receiverExtHighestSeqNum;

    uint64_t lastSrTimestamp;
    uint32_t lastSrInterval; // in microseconds
    uint32_t prevSrPacketCount;
    uint32_t prevSrByteCount;
    uint32_t srIntervalPacketCount; // over the last SR interval
    uint32_t srIntervalByteCount; // over the last SR interval
    uint64_t lastRtcpTimestamp;

    ARSTREAM2_RTCP_ClockDeltaContext_t clockDeltaCtx;
    ARSTREAM2_RTCP_VideoStatsContext_t videoStatsCtx;
    ARSTREAM2_RTCP_LossReportContext_t lossReportCtx;
    ARSTREAM2_RTCP_DjbReportContext_t djbReportCtx;
} ARSTREAM2_RTCP_SenderContext_t;

/**
 * @brief RTCP receiver context
 */
typedef struct ARSTREAM2_RTCP_ReceiverContext_s {
    uint32_t receiverSsrc;
    uint32_t senderSsrc;
    uint32_t rtcpByteRate;
    ARSTREAM2_RTCP_SdesItem_t sdesItem[ARSTREAM2_RTCP_SDES_ITEM_MAX_COUNT];
    int sdesItemCount;
    ARSTREAM2_RTCP_SdesItem_t peerSdesItem[ARSTREAM2_RTCP_SDES_ITEM_MAX_COUNT];
    int peerSdesItemCount;

    uint64_t extHighestRtpTimestamp;
    uint64_t prevSrRtpTimestamp;
    uint64_t prevSrNtpTimestamp;
    uint32_t prevSrPacketCount;
    uint32_t prevSrByteCount;
    uint32_t rtpClockRate;
    int64_t tsAnum;
    int64_t tsAden;
    uint32_t lastSrInterval; // in microseconds
    uint32_t srIntervalPacketCount; // over the last SR interval
    uint32_t srIntervalByteCount; // over the last SR interval

    uint32_t firstSeqNum;
    uint32_t extHighestSeqNum;
    uint32_t packetsReceived;
    uint32_t packetsLost;
    uint32_t interarrivalJitter;
    uint32_t lastRrExtHighestSeqNum;
    uint32_t lastRrPacketsReceived;
    uint32_t lastRrPacketsLost;
    uint32_t lastRrFractionLost;
    uint32_t lastRrInterarrivalJitter;
    uint64_t lastSrReceptionTimestamp;
    uint64_t lastRrTimestamp;
    uint64_t lastRtcpTimestamp;

    ARSTREAM2_RTCP_ClockDeltaContext_t clockDeltaCtx;
    ARSTREAM2_RTCP_VideoStatsContext_t videoStatsCtx;
    ARSTREAM2_RTCP_LossReportContext_t lossReportCtx;
    ARSTREAM2_RTCP_DjbReportContext_t djbReportCtx;
} ARSTREAM2_RTCP_ReceiverContext_t;


/*
 * Functions
 */

int ARSTREAM2_RTCP_GetPacketType(const uint8_t *buffer, unsigned int bufferSize, int *receptionReportCount, unsigned int *size);

int ARSTREAM2_RTCP_GetApplicationPacketSubtype(const uint8_t *buffer, unsigned int bufferSize);

int ARSTREAM2_RTCP_Sender_ProcessReceiverReport(const uint8_t *buffer, unsigned int bufferSize,
                                                uint64_t receptionTimestamp,
                                                ARSTREAM2_RTCP_SenderContext_t *context,
                                                int *gotReceptionReport);

int ARSTREAM2_RTCP_Sender_GenerateSenderReport(ARSTREAM2_RTCP_SenderReport_t *senderReport,
                                               uint64_t sendTimestamp, uint32_t packetCount, uint64_t byteCount,
                                               ARSTREAM2_RTCP_SenderContext_t *context);

int ARSTREAM2_RTCP_Receiver_ProcessSenderReport(const uint8_t *buffer, unsigned int bufferSize,
                                                uint64_t receptionTimestamp,
                                                ARSTREAM2_RTCP_ReceiverContext_t *context);

int ARSTREAM2_RTCP_Receiver_GenerateReceiverReport(ARSTREAM2_RTCP_ReceiverReport_t *receiverReport,
                                                   ARSTREAM2_RTCP_ReceptionReportBlock_t *receptionReport,
                                                   uint64_t sendTimestamp,
                                                   ARSTREAM2_RTCP_ReceiverContext_t *context,
                                                   unsigned int *size);

int ARSTREAM2_RTCP_GenerateSourceDescription(ARSTREAM2_RTCP_Sdes_t *sdes, unsigned int maxSize, uint32_t ssrc, uint64_t sendTimestamp,
                                             ARSTREAM2_RTCP_SdesItem_t *sdesItem, int sdesItemCount, unsigned int *size);

int ARSTREAM2_RTCP_ProcessSourceDescription(const uint8_t *buffer, unsigned int bufferSize, ARSTREAM2_RTCP_SdesItem_t *sdesItem,
                                            int sdesItemMaxCount, int *sdesItemCount);

int ARSTREAM2_RTCP_LossReportReset(ARSTREAM2_RTCP_LossReportContext_t *context);

int ARSTREAM2_RTCP_LossReportSet(ARSTREAM2_RTCP_LossReportContext_t *context, uint32_t extSeqNum);

int ARSTREAM2_RTCP_GenerateExtendedReport(ARSTREAM2_RTCP_ExtendedReport_t *xr,
                                          unsigned int maxSize, uint64_t sendTimestamp, uint32_t receiverSsrc, uint32_t senderSsrc,
                                          ARSTREAM2_RTCP_LossReportContext_t *lossReportCtx,
                                          ARSTREAM2_RTCP_DjbReportContext_t *djbReportCtx,
                                          unsigned int *size);

int ARSTREAM2_RTCP_ProcessExtendedReport(const uint8_t *buffer, unsigned int bufferSize,
                                         uint64_t receptionTimestamp, uint32_t receiverSsrc, uint32_t senderSsrc,
                                         ARSTREAM2_RTCP_LossReportContext_t *lossReportCtx,
                                         ARSTREAM2_RTCP_DjbReportContext_t *djbReportCtx,
                                         int *gotLossReport, int *gotDjbReport);

int ARSTREAM2_RTCP_GenerateApplicationClockDelta(ARSTREAM2_RTCP_Application_t *app, ARSTREAM2_RTCP_ClockDelta_t *clockDelta,
                                                 uint64_t sendTimestamp, uint32_t ssrc,
                                                 ARSTREAM2_RTCP_ClockDeltaContext_t *context);

int ARSTREAM2_RTCP_ProcessApplicationClockDelta(const uint8_t *buffer, unsigned int bufferSize,
                                                uint64_t receptionTimestamp, uint32_t peerSsrc,
                                                ARSTREAM2_RTCP_ClockDeltaContext_t *context);

int ARSTREAM2_RTCP_GenerateApplicationVideoStats(ARSTREAM2_RTCP_Application_t *app, ARSTREAM2_RTCP_VideoStats_t *videoStats,
                                                 unsigned int maxSize, uint64_t sendTimestamp,
                                                 uint32_t ssrc, ARSTREAM2_RTCP_VideoStatsContext_t *context, unsigned int *size);

int ARSTREAM2_RTCP_ProcessApplicationVideoStats(const uint8_t *buffer, unsigned int bufferSize,
                                                uint64_t receptionTimestamp, uint32_t peerSsrc,
                                                ARSTREAM2_RTCP_VideoStatsContext_t *context, int *gotVideoStats);

int ARSTREAM2_RTCP_Sender_GenerateCompoundPacket(uint8_t *packet, unsigned int maxPacketSize,
                                                 uint64_t sendTimestamp, int generateSenderReport,
                                                 int generateSourceDescription, int generateApplicationClockDelta,
                                                 uint32_t packetCount, uint64_t byteCount,
                                                 ARSTREAM2_RTCP_SenderContext_t *context,
                                                 unsigned int *size);

int ARSTREAM2_RTCP_Receiver_GenerateCompoundPacket(uint8_t *packet, unsigned int maxPacketSize,
                                                   uint64_t sendTimestamp, int generateReceiverReport,
                                                   int generateSourceDescription, int generateApplicationClockDelta,
                                                   int generateApplicationVideoStats, int generateLossReport, int generateDjbReport,
                                                   ARSTREAM2_RTCP_ReceiverContext_t *context, unsigned int *size);

int ARSTREAM2_RTCP_Sender_ProcessCompoundPacket(const uint8_t *packet, unsigned int packetSize,
                                                uint64_t receptionTimestamp,
                                                ARSTREAM2_RTCP_SenderContext_t *context,
                                                int *gotReceptionReport, int *gotVideoStats,
                                                int *gotLossReport, int *gotDjbReport);

int ARSTREAM2_RTCP_Receiver_ProcessCompoundPacket(const uint8_t *packet, unsigned int packetSize,
                                                  uint64_t receptionTimestamp,
                                                  ARSTREAM2_RTCP_ReceiverContext_t *context);

static inline uint64_t ARSTREAM2_RTCP_Receiver_GetNtpTimestampFromRtpTimestamp(ARSTREAM2_RTCP_ReceiverContext_t *context, uint64_t extRtpTimestamp);


/*
 * Inline functions
 */

static inline uint64_t ARSTREAM2_RTCP_Receiver_GetNtpTimestampFromRtpTimestamp(ARSTREAM2_RTCP_ReceiverContext_t *context, uint64_t extRtpTimestamp)
{
    return ((context->tsAnum != 0) && (context->tsAden != 0)) ? (uint64_t)((((int64_t)extRtpTimestamp - (int64_t)context->prevSrRtpTimestamp) * context->tsAden + context->tsAnum / 2) / context->tsAnum + context->prevSrNtpTimestamp) : 0;
}

#endif /* _ARSTREAM2_RTCP_H_ */
