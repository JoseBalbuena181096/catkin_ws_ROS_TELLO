/**
 * @file arstream2_rtp_h264.h
 * @brief Parrot Streaming Library - RTP H.264 payloading implementation
 * @date 04/25/2016
 * @author aurelien.barre@parrot.com
 */

#ifndef _ARSTREAM2_RTPH264_H_
#define _ARSTREAM2_RTPH264_H_

#include "arstream2_rtp.h"
#include "arstream2_h264.h"


/*
 * Macros
 */

#define ARSTREAM2_RTPH264_NALU_TYPE_STAPA 24
#define ARSTREAM2_RTPH264_NALU_TYPE_FUA 28


/*
 * Types
 */

/**
 * @brief RTP H.264 payload receiver context
 */
typedef struct ARSTREAM2_RTPH264_ReceiverContext_s
{
    int64_t previousDepayloadExtSeqNum;
    uint64_t previousDepayloadExtRtpTimestamp;
    uint32_t missingBeforePending;
    int fuPending;
    uint32_t fuPacketCount;
    ARSTREAM2_H264_NaluFifoItem_t *fuNaluItem;

    uint32_t startCode;
    int startCodeLength;
    ARSTREAM2_H264_AuFifoItem_t *auItem;

    ARSTREAM2_H264_ReceiverAuCallback_t auCallback;
    void *auCallbackUserPtr;

} ARSTREAM2_RTPH264_ReceiverContext_t;


/*
 * Functions
 */

int ARSTREAM2_RTPH264_Sender_NaluFifoToPacketFifo(ARSTREAM2_RTP_SenderContext_t *context,
                                                  ARSTREAM2_H264_NaluFifo_t *naluFifo,
                                                  ARSTREAM2_RTP_PacketFifo_t *packetFifo,
                                                  ARSTREAM2_RTP_PacketFifoQueue_t *packetFifoQueue,
                                                  uint64_t curTime, int dropOnTimeout, int *newPacketsCount);

int ARSTREAM2_RTPH264_Sender_FifoFlush(ARSTREAM2_RTP_SenderContext_t *context,
                                       ARSTREAM2_H264_NaluFifo_t *naluFifo,
                                       uint64_t curTime);

int ARSTREAM2_RTPH264_Sender_NaluDrop(ARSTREAM2_RTP_SenderContext_t *context,
                                      ARSTREAM2_H264_NalUnit_t *nalu,
                                      uint64_t curTime);

int ARSTREAM2_RTPH264_Receiver_PacketFifoToAuFifo(ARSTREAM2_RTPH264_ReceiverContext_t *context,
                                                  ARSTREAM2_RTP_PacketFifo_t *packetFifo,
                                                  ARSTREAM2_RTP_PacketFifoQueue_t *packetFifoQueue,
                                                  ARSTREAM2_H264_AuFifo_t *auFifo,
                                                  uint64_t curTime, ARSTREAM2_RTCP_ReceiverContext_t *rtcpContext);

#endif /* _ARSTREAM2_RTPH264_H_ */
