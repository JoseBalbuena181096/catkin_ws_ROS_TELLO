/**
 * @file arstream2_rtcp.c
 * @brief Parrot Streaming Library - RTCP implementation
 * @date 04/06/2016
 * @author aurelien.barre@parrot.com
 */

#include "arstream2_rtcp.h"

#include <stdlib.h>
#include <netinet/in.h>
#include <libARSAL/ARSAL_Print.h>


/**
 * Tag for ARSAL_PRINT
 */
#define ARSTREAM2_RTCP_TAG "ARSTREAM2_Rtcp"


int ARSTREAM2_RTCP_GetPacketType(const uint8_t *buffer, unsigned int bufferSize, int *receptionReportCount, unsigned int *size)
{
    if (!buffer)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }

    if (bufferSize < 8)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size (%d)", bufferSize);
        return -1;
    }

    uint8_t version = (*buffer >> 6) & 0x3;
    if (version != 2)
    {
        if (bufferSize != 24) /* workaround to avoid logging when it's an old clockSync packet with old FF or SC versions */
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid RTCP protocol version (%d)", version);
        }
        return -1;
    }

    uint8_t type = *(buffer + 1);
    if ((type == ARSTREAM2_RTCP_SENDER_REPORT_PACKET_TYPE) || (type == ARSTREAM2_RTCP_RECEIVER_REPORT_PACKET_TYPE))
    {
        uint8_t rc = *buffer & 0x1F;
        if (receptionReportCount)
        {
            *receptionReportCount = (int)rc;
        }
    }

    uint16_t length = ntohs(*((uint16_t*)(buffer + 2)));
    if ((unsigned int)length * 4 + 4 > bufferSize)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid length (%d -> %d bytes) for %d bytes buffer size", length, (unsigned int)length * 4 + 4, bufferSize);
        return -1;
    }

    if (size)
    {
        *size = (unsigned int)length * 4 + 4;
    }

    return type;
}


int ARSTREAM2_RTCP_Sender_ProcessReceiverReport(const uint8_t *buffer, unsigned int bufferSize,
                                                uint64_t receptionTimestamp,
                                                ARSTREAM2_RTCP_SenderContext_t *context,
                                                int *gotReceptionReport)
{
    const ARSTREAM2_RTCP_ReceiverReport_t *receiverReport = (const ARSTREAM2_RTCP_ReceiverReport_t*)buffer;
    const ARSTREAM2_RTCP_ReceptionReportBlock_t *receptionReport = (const ARSTREAM2_RTCP_ReceptionReportBlock_t*)(buffer + sizeof(ARSTREAM2_RTCP_ReceiverReport_t));
    uint32_t ssrc, ssrc_1;
    uint32_t lost, extHighestSeqNum, interarrivalJitter;
    uint32_t lsr, dlsr;
    uint64_t lsr_us, dlsr_us;

    if ((!buffer) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }
    if (bufferSize < sizeof(ARSTREAM2_RTCP_ReceiverReport_t))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size");
        return -1;
    }

    uint8_t version = (receiverReport->flags >> 6) & 0x3;
    if (version != 2)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid receiver report protocol version (%d)", version);
        return -1;
    }

    if (receiverReport->packetType != ARSTREAM2_RTCP_RECEIVER_REPORT_PACKET_TYPE)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid receiver report packet type (%d)", receiverReport->packetType);
        return -1;
    }

    unsigned int rrCount = receiverReport->flags & 0x1F;
    if (rrCount != 1)
    {
        if (rrCount > 1)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Unsupported reception report count (%d)", rrCount);
            return -1;
        }
        else
        {
            /* No reception report available in the receiver report */
            if (gotReceptionReport) *gotReceptionReport = 0;
            return 0;
        }
    }

    if (bufferSize < sizeof(ARSTREAM2_RTCP_ReceiverReport_t) + rrCount * sizeof(ARSTREAM2_RTCP_ReceptionReportBlock_t))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size");
        return -1;
    }

    uint16_t length = ntohs(receiverReport->length);
    if ((unsigned int)length * 4 + 4 > bufferSize)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid length (%d -> %d bytes) for %d bytes buffer size", length, (unsigned int)length * 4 + 4, bufferSize);
        return -1;
    }
    if (length < 7)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid receiver report length");
        return -1;
    }

    ssrc = ntohl(receiverReport->ssrc);
    ssrc_1 = ntohl(receptionReport->ssrc);
    lost = ntohl(receptionReport->lost);
    extHighestSeqNum = ntohl(receptionReport->extHighestSeqNum);
    interarrivalJitter = ntohl(receptionReport->interarrivalJitter);
    lsr = ntohl(receptionReport->lsr);
    dlsr = ntohl(receptionReport->dlsr);

    if (context->receiverSsrc == 0)
    {
        context->receiverSsrc = ssrc;
    }

    if (ssrc != context->receiverSsrc)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "Unexpected receiver SSRC");
        return -1;
    }

    if (ssrc_1 != context->senderSsrc)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "Unexpected sender SSRC");
        return -1;
    }

    context->receiverFractionLost = ((lost >> 24) & 0xFF);
    context->receiverLostCount = (lost & 0x00FFFFFF);
    context->receiverExtHighestSeqNum = extHighestSeqNum;
    context->interarrivalJitter = (uint32_t)(((uint64_t)interarrivalJitter * 1000000 + context->rtpClockRate / 2) / context->rtpClockRate);
    if ((lsr > 0) || (dlsr > 0))
    {
        lsr_us = ((uint64_t)lsr * 1000000) >> 16;
        //TODO: handle the LSR timestamp loopback after 18h
        dlsr_us = ((uint64_t)dlsr * 1000000) >> 16;
        context->roundTripDelay = (uint32_t)(receptionTimestamp - lsr_us - dlsr_us);
    }
    else
    {
        lsr_us = dlsr_us = 0;
        context->roundTripDelay = 0;
    }
    context->lastRrReceptionTimestamp = receptionTimestamp;

    if (gotReceptionReport) *gotReceptionReport = 1;
    return 0;
}


int ARSTREAM2_RTCP_Sender_GenerateSenderReport(ARSTREAM2_RTCP_SenderReport_t *senderReport,
                                               uint64_t sendTimestamp, uint32_t packetCount, uint64_t byteCount,
                                               ARSTREAM2_RTCP_SenderContext_t *context)
{
    if ((!senderReport) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }

    uint64_t ntpTimestampL = ((sendTimestamp % 1000000) << 32) / 1000000ULL;
    uint32_t rtpTimestamp = context->rtpTimestampOffset + (uint32_t)((((sendTimestamp * context->rtpClockRate) + 500000) / 1000000) & 0xFFFFFFFF); /* microseconds to rtpClockRate */

    senderReport->flags = (2 << 6);
    senderReport->packetType = ARSTREAM2_RTCP_SENDER_REPORT_PACKET_TYPE;
    senderReport->length = htons(6);
    senderReport->ssrc = htonl(context->senderSsrc);
    senderReport->ntpTimestampH = htonl((uint32_t)(sendTimestamp / 1000000));
    senderReport->ntpTimestampL = htonl((uint32_t)(ntpTimestampL & 0xFFFFFFFF));
    senderReport->rtpTimestamp = htonl(rtpTimestamp);
    senderReport->senderPacketCount = htonl(packetCount);
    senderReport->senderByteCount = htonl((uint32_t)(byteCount & 0xFFFFFFFF));

    // Packet and byte rates
    if (context->lastSrTimestamp)
    {
        context->lastSrInterval = (uint32_t)(sendTimestamp - context->lastSrTimestamp);
        if (context->lastSrInterval > 0)
        {
            context->srIntervalPacketCount = packetCount - context->prevSrPacketCount;
            context->srIntervalByteCount = (uint32_t)(byteCount - context->prevSrByteCount);
        }
        else
        {
            context->srIntervalPacketCount = context->srIntervalByteCount = 0;
        }

        // Update values
        context->prevSrPacketCount = packetCount;
        context->prevSrByteCount = byteCount;
    }

    context->lastSrTimestamp = sendTimestamp;

    return 0;
}


int ARSTREAM2_RTCP_Receiver_ProcessSenderReport(const uint8_t *buffer, unsigned int bufferSize,
                                                uint64_t receptionTimestamp,
                                                ARSTREAM2_RTCP_ReceiverContext_t *context)
{
    const ARSTREAM2_RTCP_SenderReport_t *senderReport = (const ARSTREAM2_RTCP_SenderReport_t*)buffer;
    uint32_t ssrc, rtpTimestamp, senderPacketCount, senderByteCount;
    uint64_t extRtpTimestamp, ntpTimestamp;

    if ((!buffer) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }
    if (bufferSize < sizeof(ARSTREAM2_RTCP_SenderReport_t))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size");
        return -1;
    }

    uint8_t version = (senderReport->flags >> 6) & 0x3;
    if (version != 2)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid sender report protocol version (%d)", version);
        return -1;
    }

    if (senderReport->packetType != ARSTREAM2_RTCP_SENDER_REPORT_PACKET_TYPE)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid sender report packet type (%d)", senderReport->packetType);
        return -1;
    }

    uint16_t length = ntohs(senderReport->length);
    if ((unsigned int)length * 4 + 4 > bufferSize)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid length (%d -> %d bytes) for %d bytes buffer size", length, (unsigned int)length * 4 + 4, bufferSize);
        return -1;
    }
    if (length < 6)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid sender report length");
        return -1;
    }

    ssrc = ntohl(senderReport->ssrc);
    senderPacketCount = ntohl(senderReport->senderPacketCount);
    senderByteCount = ntohl(senderReport->senderByteCount);

    ntpTimestamp = ((uint64_t)(ntohl(senderReport->ntpTimestampH)) * 1000000) + (((uint64_t)(ntohl(senderReport->ntpTimestampL)) * 1000000) >> 32);
    rtpTimestamp = ntohl(senderReport->rtpTimestamp);

    if (ssrc != context->senderSsrc)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "Unexpected sender SSRC");
        return -1;
    }

    if (!context->prevSrNtpTimestamp)
    {
        context->prevSrNtpTimestamp = ntpTimestamp;
    }
    else if (ntpTimestamp <= context->prevSrNtpTimestamp)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "Out of order or duplicate sender report (%"PRIu64" vs. %"PRIu64")", ntpTimestamp, context->prevSrNtpTimestamp);
        return -1;
    }
    if (!context->prevSrRtpTimestamp)
    {
        extRtpTimestamp = rtpTimestamp;
        context->extHighestRtpTimestamp = extRtpTimestamp;
        context->prevSrRtpTimestamp = extRtpTimestamp;
    }
    else
    {
        extRtpTimestamp = (context->extHighestRtpTimestamp & 0xFFFFFFFF00000000ULL) | ((uint64_t)rtpTimestamp & 0xFFFFFFFFULL);
        if ((int64_t)extRtpTimestamp - (int64_t)context->prevSrRtpTimestamp < -2147483648LL)
        {
            extRtpTimestamp += 0x100000000ULL;
        }
        else if ((int64_t)extRtpTimestamp - (int64_t)context->prevSrRtpTimestamp > 2147483648LL)
        {
            extRtpTimestamp -= 0x100000000ULL;
        }
        if (extRtpTimestamp > context->extHighestRtpTimestamp)
        {
            context->extHighestRtpTimestamp = extRtpTimestamp;
        }
    }

    /* NTP to RTP linear regression:
     * for samples m and n, RTPn = a * NTPn + b and RTPm = a * NTPm + b
     * <=> RTPn - RTPm = a * (NTPn - NTPm)
     * <=> NTPn = (RTPn - RTPm) / a + NTPm
     * m sample is the last RTP/NTP pair received from a RTCP sender report
     * n sample in the current RTP packet for which we want to derive the NTP timestamp
     */
    context->tsAnum = (int64_t)(extRtpTimestamp - context->prevSrRtpTimestamp);
    context->tsAden = (int64_t)(ntpTimestamp - context->prevSrNtpTimestamp);

    // Packet and byte rates
    context->lastSrInterval = (uint32_t)(ntpTimestamp - context->prevSrNtpTimestamp);
    if (context->lastSrInterval > 0)
    {
        context->srIntervalPacketCount = senderPacketCount - context->prevSrPacketCount;
        context->srIntervalByteCount = senderByteCount - context->prevSrByteCount; //TODO: handle 32bits loopback
    }
    else
    {
        context->srIntervalPacketCount = context->srIntervalByteCount = 0;
    }

    // Update values
    context->prevSrRtpTimestamp = extRtpTimestamp;
    context->prevSrNtpTimestamp = ntpTimestamp;
    context->prevSrPacketCount = senderPacketCount;
    context->prevSrByteCount = senderByteCount;

    context->lastSrReceptionTimestamp = receptionTimestamp;

    return 0;
}


int ARSTREAM2_RTCP_Receiver_GenerateReceiverReport(ARSTREAM2_RTCP_ReceiverReport_t *receiverReport,
                                                   ARSTREAM2_RTCP_ReceptionReportBlock_t *receptionReport,
                                                   uint64_t sendTimestamp,
                                                   ARSTREAM2_RTCP_ReceiverContext_t *context,
                                                   unsigned int *size)
{
    if ((!receiverReport) || (!receptionReport) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }

    if (context->lastSrReceptionTimestamp == 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "No sender report received");
        return -1;
    }

    int rrCount = ((context->packetsReceived > 0) && (context->packetsReceived > context->lastRrPacketsReceived)) ? 1 : 0;
    unsigned int _size = sizeof(ARSTREAM2_RTCP_ReceiverReport_t) + rrCount * sizeof(ARSTREAM2_RTCP_ReceptionReportBlock_t);

    receiverReport->flags = (2 << 6) | (rrCount & 0x1F);
    receiverReport->packetType = ARSTREAM2_RTCP_RECEIVER_REPORT_PACKET_TYPE;
    receiverReport->length = htons(1 + 6 * rrCount);
    receiverReport->ssrc = htonl(context->receiverSsrc);

    if (rrCount == 1)
    {
        //uint32_t cumulativeLost = context->extHighestSeqNum - context->firstSeqNum + 1 - context->packetsReceived;
        // NB: cumulativeLost is unreliable because (context->extHighestSeqNum - context->firstSeqNum) is before jitter buffer and context->packetsReceived is after
        uint32_t fractionLost = 0;
        if ((context->lastRrExtHighestSeqNum != 0) && (context->extHighestSeqNum > context->lastRrExtHighestSeqNum))
        {
            fractionLost = (context->packetsLost - context->lastRrPacketsLost) * 256 / (context->extHighestSeqNum - context->lastRrExtHighestSeqNum);
            if (fractionLost > 256) fractionLost = 0;
        }
        receptionReport->ssrc = htonl(context->senderSsrc);
        receptionReport->lost = htonl(((fractionLost & 0xFF) << 24) | (context->packetsLost & 0x00FFFFFF));
        receptionReport->extHighestSeqNum = htonl(context->extHighestSeqNum);
        receptionReport->interarrivalJitter = htonl(context->interarrivalJitter);
        receptionReport->lsr = htonl((uint32_t)(((context->prevSrNtpTimestamp << 16) / 1000000) & 0xFFFFFFFF));
        receptionReport->dlsr = htonl(((sendTimestamp - context->lastSrReceptionTimestamp) << 16) / 1000000);

        context->lastRrExtHighestSeqNum = context->extHighestSeqNum;
        context->lastRrPacketsReceived = context->packetsReceived;
        context->lastRrInterarrivalJitter = (uint32_t)(((uint64_t)context->interarrivalJitter * 1000000 + context->rtpClockRate / 2) / context->rtpClockRate);
        context->lastRrPacketsLost = context->packetsLost;
        context->lastRrFractionLost = fractionLost;
        context->lastRrTimestamp = sendTimestamp;
    }
    else if (rrCount > 1)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "Unsupported receiver report count");
        return -1;
    }

    if (size)
        *size = _size;

    return 0;
}


int ARSTREAM2_RTCP_GenerateSourceDescription(ARSTREAM2_RTCP_Sdes_t *sdes, unsigned int maxSize, uint32_t ssrc, uint64_t sendTimestamp,
                                             ARSTREAM2_RTCP_SdesItem_t *sdesItem, int sdesItemCount, unsigned int *size)
{
    int i;

    if ((!sdes) || (!sdesItem))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }
    if (sdesItemCount <= 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid SDES item count");
        return -1;
    }

    unsigned int _size = sizeof(ARSTREAM2_RTCP_Sdes_t) + 4;
    if (_size + 1 > maxSize)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for SDES");
        return -1;
    }

    uint32_t *ssrc_1 = (uint32_t*)((uint8_t*)sdes + sizeof(ARSTREAM2_RTCP_Sdes_t));
    uint8_t *item = (uint8_t*)sdes + sizeof(ARSTREAM2_RTCP_Sdes_t) + 4;

    int sourceCount = 1;
    sdes->flags = (2 << 6) | (sourceCount & 0x1F);
    sdes->packetType = ARSTREAM2_RTCP_SDES_PACKET_TYPE;
    *ssrc_1 = htonl(ssrc);

    /* SDES items list */
    for (i = 0; i < sdesItemCount; i++)
    {
        if (strlen(sdesItem[i].value))
        {
            if (sdesItem[i].type == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
            {
                /* private extension item */
                if (strlen(sdesItem[i].prefix))
                {
                    if (_size + 2 + 1 + strlen(sdesItem[i].prefix) + strlen(sdesItem[i].value) + 1 > maxSize)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for SDES");
                        break;
                    }
                    if ((!sdesItem[i].lastSendTime) || (sendTimestamp >= sdesItem[i].lastSendTime + sdesItem[i].sendTimeInterval))
                    {
                        *item = sdesItem[i].type;
                        *(item + 1) = 1 + strlen(sdesItem[i].prefix) + strlen(sdesItem[i].value);
                        *(item + 2) = strlen(sdesItem[i].prefix);
                        memcpy(item + 2 + 1, sdesItem[i].prefix, strlen(sdesItem[i].prefix));
                        memcpy(item + 2 + 1 + strlen(sdesItem[i].prefix), sdesItem[i].value, strlen(sdesItem[i].value));
                        item += 2 + 1 + strlen(sdesItem[i].prefix) + strlen(sdesItem[i].value);
                        _size += 2 + 1 + strlen(sdesItem[i].prefix) + strlen(sdesItem[i].value);
                        sdesItem[i].lastSendTime = sendTimestamp;
                    }
                }
            }
            else
            {
                if (_size + 2 + strlen(sdesItem[i].value) + 1 > maxSize)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for SDES");
                    break;
                }
                if ((sdesItem[i].type == ARSTREAM2_RTCP_SDES_CNAME_ITEM)
                        || (!sdesItem[i].lastSendTime) || (sendTimestamp >= sdesItem[i].lastSendTime + sdesItem[i].sendTimeInterval))
                {
                    *item = sdesItem[i].type;
                    *(item + 1) = strlen(sdesItem[i].value);
                    memcpy(item + 2, sdesItem[i].value, strlen(sdesItem[i].value));
                    item += 2 + strlen(sdesItem[i].value);
                    _size += 2 + strlen(sdesItem[i].value);
                    sdesItem[i].lastSendTime = sendTimestamp;
                }
            }
        }
    }

    /* terminating zero + padding */
    do
    {
        if (_size + 1 > maxSize)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for SDES");
            return -1;
        }
        *item = 0;
        item++;
        _size++;
    }
    while (_size & 3);

    sdes->length = htons((_size - 4) / 4);

    if (size)
        *size = _size;

    return 0;
}


int ARSTREAM2_RTCP_ProcessSourceDescription(const uint8_t *buffer, unsigned int bufferSize, ARSTREAM2_RTCP_SdesItem_t *sdesItem,
                                            int sdesItemMaxCount, int *sdesItemCount)
{
    const ARSTREAM2_RTCP_Sdes_t *sdes = (const ARSTREAM2_RTCP_Sdes_t*)buffer;

    if ((!buffer) || (!sdesItem) || (!sdesItemCount))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }
    if (bufferSize < sizeof(ARSTREAM2_RTCP_Sdes_t))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size");
        return -1;
    }
    if (sdesItemMaxCount <= 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid SDES item max count");
        return -1;
    }
    if (*sdesItemCount < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid SDES item count");
        return -1;
    }

    uint8_t version = (sdes->flags >> 6) & 0x3;
    if (version != 2)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid source description protocol version (%d)", version);
        return -1;
    }

    if (sdes->packetType != ARSTREAM2_RTCP_SDES_PACKET_TYPE)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid source description packet type (%d)", sdes->packetType);
        return -1;
    }

    uint8_t sc = sdes->flags & 0x1F;
    if (bufferSize < sizeof(ARSTREAM2_RTCP_Sdes_t) + sc * 8) // at least SSRC + CNAME + NULL
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size");
        return -1;
    }

    uint16_t length = ntohs(sdes->length);
    if ((unsigned int)length * 4 + 4 > bufferSize)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid length (%d -> %d bytes) for %d bytes buffer size", length, (unsigned int)length * 4 + 4, bufferSize);
        return -1;
    }
    if (length < sc * 2)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid source description packet length");
        return -1;
    }

    if (sc < 1)
    {
        /* source count is null, nothing more to do */
        return 0;
    }

    const uint8_t *ptr = buffer + 4;
    unsigned int remLength = (unsigned int)length * 4, i;
    for (i = 0; (i < sc) && (remLength >= 4); i++)
    {
        /* read the SSRC */
        //uint32_t ssrc = ntohl(*((const uint32_t*)ptr)); // Unused
        ptr += 4;
        remLength -= 4;

        /* read the SDES items */
        while ((*ptr != 0) && (remLength >= 3))
        {
            uint8_t id = *ptr;
            uint8_t len = *(ptr + 1);
            ptr += 2;
            remLength -= 2;
            char str[256];
            str[0] = '\0';
            char prefix[256];
            prefix[0] = '\0';

            if (remLength < len)
            {
                break;
            }

            if ((id == ARSTREAM2_RTCP_SDES_PRIV_ITEM) && (len > 2))
            {
                /* private extension item */
                uint8_t prefixLen = *ptr;
                uint8_t strLen = len - prefixLen - 1;
                if (remLength < (unsigned int)3 + prefixLen)
                {
                    break;
                }
                memcpy(prefix, ptr + 1, prefixLen);
                prefix[prefixLen] = '\0';
                if (remLength < (unsigned int)3 + prefixLen + strLen)
                {
                    break;
                }
                memcpy(str, ptr + 1 + prefixLen, strLen);
                str[strLen] = '\0';
            }
            else
            {
                uint8_t strLen = len;
                if (remLength < (unsigned int)2 + strLen)
                {
                    break;
                }
                memcpy(str, ptr, strLen);
                str[strLen] = '\0';
            }

            /* add the item if it is valid */
            if ((id <= 8) && (strlen(str)) && ((id != ARSTREAM2_RTCP_SDES_PRIV_ITEM) || (strlen(prefix))))
            {
                /* uncomment to print values for debug */
                /*static const char *ARSTREAM2_RTCP_SdesItemName[] =
                {
                    "NULL", "CNAME", "NAME", "EMAIL", "PHONE", "LOC", "TOOL", "NOTE", "PRIV",
                };
                if (id == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
                {
                    ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_RTCP_TAG, "SDES SSRC=0x%08X %s prefix='%s' value='%s'", ssrc, ARSTREAM2_RTCP_SdesItemName[id], prefix, str);
                }
                else
                {
                    ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_RTCP_TAG, "SDES SSRC=0x%08X %s value='%s'", ssrc, ARSTREAM2_RTCP_SdesItemName[id], str);
                }*/

                int k, found;
                /* existing item */
                for (k = 0, found = 0; k < *sdesItemCount; k++)
                {
                    if (id == sdesItem[k].type)
                    {
                        if (id == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
                        {
                            if (!strncmp(prefix, sdesItem[k].prefix, 256))
                            {
                                snprintf(sdesItem[k].value, 256, "%s", str);
                                found = 1;
                                break;
                            }
                        }
                        else
                        {
                            snprintf(sdesItem[k].value, 256, "%s", str);
                            found = 1;
                            break;
                        }
                    }
                }
                /* new item */
                if ((!found) && (*sdesItemCount < sdesItemMaxCount))
                {
                    k = *sdesItemCount;
                    sdesItem[k].type = id;
                    snprintf(sdesItem[k].value, 256, "%s", str);
                    if (id == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
                    {
                        snprintf(sdesItem[k].prefix, 256, "%s", prefix);
                    }
                    *sdesItemCount = k + 1;
                }
            }

            ptr += len;
            remLength -= len;
        }

        /* align to multiple of 4 bytes */
        if ((*ptr == 0) && (remLength))
        {
            unsigned int align = ((remLength + 3) & ~3) - remLength;
            if (remLength < align)
            {
                break;
            }
            remLength -= align;
            ptr += align;
        }
    }

    return 0;
}


int ARSTREAM2_RTCP_LossReportReset(ARSTREAM2_RTCP_LossReportContext_t *context)
{
    if (!context)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }

    context->count = 0;
    if ((context->receivedFlag) && (context->wordCount))
    {
        memset(context->receivedFlag, 0, context->wordCount * sizeof(uint32_t));
    }

    return 0;
}


int ARSTREAM2_RTCP_LossReportSet(ARSTREAM2_RTCP_LossReportContext_t *context, uint32_t extSeqNum)
{
    if (!context)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }

    /* NB: out of order packets are not supported for startSeqNum, LossReportSet() must be called after reordering */
    if (context->count == 0)
    {
        context->startSeqNum = context->endSeqNum = extSeqNum;
    }

    int packetCount = (int)context->endSeqNum - (int)context->startSeqNum + 1;
    if (packetCount <= 0)
        packetCount += (1 << 16);
    if (packetCount >= 65534)
    {
        /* Loss RLE blocks cannot account for more than 65534 packets (RFC3611) */
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Loss report packet count is too large (%d), resetting", packetCount);
        ARSTREAM2_RTCP_LossReportReset(context);
        return -1;
    }

    /* Realloc the receivedFlag buffer if necessary */
    if ((!context->receivedFlag) || ((int)extSeqNum - (int)context->startSeqNum >= context->wordCount * 32))
    {
        if (context->wordCount == 0)
        {
            context->wordCount = ARSTREAM2_RTCP_LOSS_REPORT_INITIAL_WORD_COUNT;
        }
        else
        {
            context->wordCount *= 2;
            if (context->wordCount > 65536 / 32)
            {
                /* Loss RLE blocks cannot account for more than 65534 packets (RFC3611)
                 * so there is no need to realloc with more than 65536 / 32 words */
                context->wordCount = 65536 / 32;
            }
        }
        uint32_t *receivedFlag = realloc(context->receivedFlag, context->wordCount * sizeof(uint32_t));
        if (!receivedFlag)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Allocation failed (size %zu)", context->wordCount * sizeof(uint32_t));
            free(context->receivedFlag);
            context->receivedFlag = NULL;
            context->wordCount = 0;
            ARSTREAM2_RTCP_LossReportReset(context);
            return -1;
        }
        else
        {
            context->receivedFlag = receivedFlag;
        }
    }

    if (extSeqNum > context->endSeqNum)
    {
        context->endSeqNum = extSeqNum;
    }

    int wordIdx = (extSeqNum - context->startSeqNum) >> 5;
    int bitIdx = 31 - ((extSeqNum - context->startSeqNum) & 0x1F);
    context->receivedFlag[wordIdx] |= (1 << bitIdx);
    context->count++;

    return 0;
}


int ARSTREAM2_RTCP_GenerateExtendedReport(ARSTREAM2_RTCP_ExtendedReport_t *xr,
                                          unsigned int maxSize, uint64_t sendTimestamp, uint32_t receiverSsrc, uint32_t senderSsrc,
                                          ARSTREAM2_RTCP_LossReportContext_t *lossReportCtx,
                                          ARSTREAM2_RTCP_DjbReportContext_t *djbReportCtx,
                                          unsigned int *size)
{
    unsigned int _size = 0, chunkCount = 0;

    if (!xr)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }

    if ((!lossReportCtx) && (!djbReportCtx))
    {
        /* Nothing to do */
        if (size)
            *size = _size;

        return 0;
    }

    _size += sizeof(ARSTREAM2_RTCP_ExtendedReport_t);
    if (_size > maxSize)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for XR");
        return -1;
    }

    xr->flags = (2 << 6);
    xr->packetType = ARSTREAM2_RTCP_EXTENDED_REPORT_PACKET_TYPE;
    xr->ssrc = htonl(receiverSsrc);

    if (djbReportCtx)
    {
        ARSTREAM2_RTCP_DjbMetricsReportBlock_t *djbReport = (ARSTREAM2_RTCP_DjbMetricsReportBlock_t*)((uint8_t*)xr + _size);

        _size += sizeof(ARSTREAM2_RTCP_DjbMetricsReportBlock_t);
        if (_size > maxSize)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for XR");
            return -1;
        }

        djbReport->blockType = ARSTREAM2_RTCP_DJB_METRICS_REPORT_BLOCK_TYPE;
        djbReport->flags = 0x60; // I = 01 (sampled value), C = 1 (adaptive de-jitter buffer)
        djbReport->length = htons(sizeof(ARSTREAM2_RTCP_DjbMetricsReportBlock_t) / 4 - 1);
        djbReport->ssrc = htonl(senderSsrc);
        djbReport->djbNominal = (djbReportCtx->djbMetricsAvailable)
                ? ((djbReportCtx->djbNominal <= 0xFFFD) ? htons((uint16_t)djbReportCtx->djbNominal) : htons(0xFFFE))
                : htons(0xFFFF);
        djbReport->djbMax = (djbReportCtx->djbMetricsAvailable)
                ? ((djbReportCtx->djbMax <= 0xFFFD) ? htons((uint16_t)djbReportCtx->djbMax) : htons(0xFFFE))
                : htons(0xFFFF);
        djbReport->djbHighWatermark = (djbReportCtx->djbMetricsAvailable)
                ? ((djbReportCtx->djbHighWatermark <= 0xFFFD) ? htons((uint16_t)djbReportCtx->djbHighWatermark) : htons(0xFFFE))
                : htons(0xFFFF);
        djbReport->djbLowWatermark = (djbReportCtx->djbMetricsAvailable)
                ? ((djbReportCtx->djbLowWatermark <= 0xFFFD) ? htons((uint16_t)djbReportCtx->djbLowWatermark) : htons(0xFFFE))
                : htons(0xFFFF);
    }

    if (lossReportCtx)
    {
        ARSTREAM2_RTCP_LossRleReportBlock_t *lossRle = (ARSTREAM2_RTCP_LossRleReportBlock_t*)((uint8_t*)xr + _size);

        _size += sizeof(ARSTREAM2_RTCP_LossRleReportBlock_t);
        if (_size > maxSize)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for XR");
            return -1;
        }

        lossRle->blockType = ARSTREAM2_RTCP_LOSS_RLE_REPORT_BLOCK_TYPE;
        lossRle->thinning = 0;
        lossRle->ssrc = htonl(senderSsrc);
        lossRle->beginSeq = htons(lossReportCtx->startSeqNum & 0xFFFF);
        lossRle->endSeq = htons((lossReportCtx->endSeqNum & 0xFFFF) + 1);

        int i, j, k, packetCount = lossReportCtx->endSeqNum - lossReportCtx->startSeqNum + 1, wordCount = (packetCount >> 5) + ((packetCount & 0x1F) ? 1 : 0), chunkBitIdx = 14;
        uint16_t chunk = 0;
        int bitVal, runLength = 0, runBit = -1;
        for (i = 0, k = 0; i < wordCount; i++)
        {
            if (k == packetCount)
            {
                break;
            }
            for (j = 0; j < 32; j++, k++)
            {
                if (k == packetCount)
                {
                    break;
                }

                /* Dynamically choose between bit vector chunks and run length chunks in a single loop */
                bitVal = ((lossReportCtx->receivedFlag[i] >> (31 - j)) & 1);
                if (runBit == -1)
                {
                    /* Start new run */
                    runBit = bitVal;
                    runLength = 1;
                }
                else if (bitVal == runBit)
                {
                    /* Continue run */
                    runLength++;
                }
                else
                {
                    /* End run */
                    if (runLength >= 16)
                    {
                        /* Choose run length chunk */
                        if (_size + chunkCount * 2 + 2 > maxSize)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for XR");
                            return -1;
                        }
                        *((uint16_t*)lossRle + 6 + chunkCount) = htons((runBit << 14) | (runLength & 0x3FFF));
                        chunkCount++;

                        /* Reset the bit vector chunk */
                        chunkBitIdx = 14;
                        chunk = 0;
                    }

                    /* Start new run */
                    runBit = bitVal;
                    runLength = 1;
                }
                if (chunkBitIdx >= 0)
                {
                    chunk |= (bitVal << chunkBitIdx);
                }
                chunkBitIdx--;
                if ((chunkBitIdx < 0) && ((runBit == -1) || (runLength < 16)))
                {
                    /* Choose bit vector chunk */
                    if (_size + chunkCount * 2 + 2 > maxSize)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for XR");
                        return -1;
                    }
                    *((uint16_t*)lossRle + 6 + chunkCount) = htons(0x8000 | chunk);
                    chunkCount++;

                    /* Reset the bit vector chunk and the run */
                    chunkBitIdx = 14;
                    chunk = 0;
                    runBit = -1;
                }
            }
        }
        if ((runBit != -1) && (runLength >= 16))
        {
            /* Write the last run length chunk */
            if (_size + chunkCount * 2 + 2 > maxSize)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for XR");
                return -1;
            }
            *((uint16_t*)lossRle + 6 + chunkCount) = htons((runBit << 14) | (runLength & 0x3FFF));
            chunkCount++;
        }
        else if (chunkBitIdx < 14)
        {
            /* Write the last bit vector chunk */
            if (_size + chunkCount * 2 + 2 > maxSize)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for XR");
                return -1;
            }
            *((uint16_t*)lossRle + 6 + chunkCount) = htons(0x8000 | chunk);
            chunkCount++;
        }

        if (chunkCount & 1)
        {
            /* Odd count: write a terminating null chunk */
            if (_size + chunkCount * 2 + 2 > maxSize)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for XR");
                return -1;
            }
            *((uint16_t*)lossRle + 6 + chunkCount) = 0;
            chunkCount++;
        }

        lossRle->length = htons((sizeof(ARSTREAM2_RTCP_LossRleReportBlock_t) + chunkCount * 2) / 4 - 1);
        _size += chunkCount * 2;
    }

    xr->length = htons(_size / 4 - 1);

    if (size)
        *size = _size;

    return 0;
}


int ARSTREAM2_RTCP_ProcessExtendedReport(const uint8_t *buffer, unsigned int bufferSize,
                                         uint64_t receptionTimestamp, uint32_t receiverSsrc, uint32_t senderSsrc,
                                         ARSTREAM2_RTCP_LossReportContext_t *lossReportCtx,
                                         ARSTREAM2_RTCP_DjbReportContext_t *djbReportCtx,
                                         int *gotLossReport, int *gotDjbReport)
{
    const ARSTREAM2_RTCP_ExtendedReport_t *xr = (const ARSTREAM2_RTCP_ExtendedReport_t*)buffer;
    int ret = 0;

    if (gotLossReport)
    {
        *gotLossReport = 0;
    }

    if ((!buffer) || (!lossReportCtx) || (!djbReportCtx))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }
    if (bufferSize < sizeof(ARSTREAM2_RTCP_ExtendedReport_t))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size");
        return -1;
    }

    uint8_t version = (xr->flags >> 6) & 0x3;
    if (version != 2)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid extended report packet protocol version (%d)", version);
        return -1;
    }

    if (xr->packetType != ARSTREAM2_RTCP_EXTENDED_REPORT_PACKET_TYPE)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid extended report packet type (%d)", xr->packetType);
        return -1;
    }

    uint32_t ssrc = ntohl(xr->ssrc);
    if (ssrc != receiverSsrc)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "Unexpected peer SSRC");
        return -1;
    }

    uint16_t length = ntohs(xr->length);
    if ((unsigned int)length * 4 + 4 > bufferSize)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid length (%d -> %d bytes) for %d bytes buffer size", length, (unsigned int)length * 4 + 4, bufferSize);
        return -1;
    }
    if (length < sizeof(ARSTREAM2_RTCP_ExtendedReport_t) / 4 - 1)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid extended report packet length (%d)", length);
        return -1;
    }

    uint16_t processedLen = sizeof(ARSTREAM2_RTCP_ExtendedReport_t) / 4 - 1;
    while (processedLen < length)
    {
        if (*(buffer + processedLen * 4 + 4) == ARSTREAM2_RTCP_LOSS_RLE_REPORT_BLOCK_TYPE)
        {
            const ARSTREAM2_RTCP_LossRleReportBlock_t *lossRle = (const ARSTREAM2_RTCP_LossRleReportBlock_t*)(buffer + processedLen * 4 + 4);

            if (bufferSize - (processedLen * 4 + 4) < sizeof(ARSTREAM2_RTCP_LossRleReportBlock_t))
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size");
                ret = -1;
                break;
            }

            if (lossRle->thinning != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "Thinning is not supported");
                ret = -1;
                break;
            }

            uint32_t ssrc2 = ntohl(lossRle->ssrc);
            if (ssrc2 != senderSsrc)
            {
                ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "Unexpected sender SSRC");
                ret = -1;
                break;
            }

            uint16_t blockLen = ntohs(lossRle->length);
            if ((unsigned int)blockLen * 4 + 4 > bufferSize - (processedLen * 4 + 4))
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid length (%d -> %d bytes) for %d bytes remaining buffer size", blockLen, (unsigned int)blockLen * 4 + 4, bufferSize - (processedLen * 4 + 4));
                ret = -1;
                break;
            }
            if (blockLen < sizeof(ARSTREAM2_RTCP_LossRleReportBlock_t) / 4 - 1)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid loss RLE block length (%d)", blockLen);
                ret = -1;
                break;
            }

            uint16_t beginSeq = ntohs(lossRle->beginSeq);
            uint16_t endSeq = ntohs(lossRle->endSeq);

            lossReportCtx->startSeqNum = beginSeq;
            lossReportCtx->endSeqNum = endSeq;
            if (endSeq < beginSeq)
            {
                lossReportCtx->endSeqNum += 65536;
            }
            lossReportCtx->endSeqNum--;
            lossReportCtx->count = lossReportCtx->endSeqNum - lossReportCtx->startSeqNum + 1;

            /* Realloc the receivedFlag buffer if necessary */
            if ((!lossReportCtx->receivedFlag) || (lossReportCtx->count >= lossReportCtx->wordCount * 32))
            {
                if (lossReportCtx->wordCount == 0)
                {
                    lossReportCtx->wordCount = ARSTREAM2_RTCP_LOSS_REPORT_INITIAL_WORD_COUNT;
                }
                else
                {
                    lossReportCtx->wordCount *= 2;
                    if (lossReportCtx->wordCount >= 65536 / 32)
                    {
                        /* Loss RLE blocks cannot account for more than 65534 packets (RFC3611) */
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Word count is too large (%d), resetting", lossReportCtx->wordCount);
                        ARSTREAM2_RTCP_LossReportReset(lossReportCtx);
                        return -1;
                    }
                }
                uint32_t *receivedFlag = realloc(lossReportCtx->receivedFlag, lossReportCtx->wordCount * sizeof(uint32_t));
                if (!receivedFlag)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Allocation failed (size %zu)", lossReportCtx->wordCount * sizeof(uint32_t));
                    free(lossReportCtx->receivedFlag);
                    lossReportCtx->receivedFlag = NULL;
                    lossReportCtx->wordCount = 0;
                    ARSTREAM2_RTCP_LossReportReset(lossReportCtx);
                    return -1;
                }
                else
                {
                    lossReportCtx->receivedFlag = receivedFlag;
                }
            }

            ret = ARSTREAM2_RTCP_LossReportReset(lossReportCtx);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "ARSTREAM2_RTCP_LossReportReset() failed (%d)", ret);
                ret = -1;
                break;
            }
            lossReportCtx->count = lossReportCtx->endSeqNum - lossReportCtx->startSeqNum + 1;

            int i, j, k, chunkCount = (blockLen - 2) * 2;
            uint16_t *chunkPtr = (uint16_t*)lossRle + 6;
            uint16_t chunk;
            for (i = 0, k = 0; i < chunkCount; i++, chunkPtr++)
            {
                if (k >= lossReportCtx->count)
                {
                    break;
                }
                chunk = htons(*chunkPtr);
                if ((chunk & 0x8000) == 0x8000)
                {
                    /* Bit vector chunk */
                    for (j = 0; j < 15; j++, k++)
                    {
                        if (k >= lossReportCtx->count)
                        {
                            break;
                        }
                        int wordIdx = k >> 5;
                        int bitIdx = 31 - (k & 0x1F);
                        lossReportCtx->receivedFlag[wordIdx] |= (((chunk >> (14 - j)) & 1) << bitIdx);
                    }
                }
                else
                {
                    /* Run length chunk */
                    int bitVal = ((chunk >> 14) & 1), len = (chunk & 0x3FFF);
                    /* if len == 0 this is in fact a terminating null chunk */
                    for (j = 0; j < len; j++, k++)
                    {
                        if (k >= lossReportCtx->count)
                        {
                            break;
                        }
                        int wordIdx = k >> 5;
                        int bitIdx = 31 - (k & 0x1F);
                        lossReportCtx->receivedFlag[wordIdx] |= (bitVal << bitIdx);
                    }
                }
            }

            lossReportCtx->lastReceptionTimestamp = receptionTimestamp;
            if (gotLossReport)
            {
                *gotLossReport = 1;
            }
        }
        else if (*(buffer + processedLen * 4 + 4) == ARSTREAM2_RTCP_DJB_METRICS_REPORT_BLOCK_TYPE)
        {
            const ARSTREAM2_RTCP_DjbMetricsReportBlock_t *djbReport = (const ARSTREAM2_RTCP_DjbMetricsReportBlock_t*)(buffer + processedLen * 4 + 4);

            if (bufferSize - (processedLen * 4 + 4) < sizeof(ARSTREAM2_RTCP_DjbMetricsReportBlock_t))
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size");
                ret = -1;
                break;
            }

            uint32_t ssrc2 = ntohl(djbReport->ssrc);
            if (ssrc2 != senderSsrc)
            {
                ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "Unexpected sender SSRC");
                ret = -1;
                break;
            }

            uint16_t blockLen = ntohs(djbReport->length);
            if ((unsigned int)blockLen * 4 + 4 > bufferSize - (processedLen * 4 + 4))
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid length (%d -> %d bytes) for %d bytes remaining buffer size", blockLen, (unsigned int)blockLen * 4 + 4, bufferSize - (processedLen * 4 + 4));
                ret = -1;
                break;
            }
            if (blockLen < sizeof(ARSTREAM2_RTCP_DjbMetricsReportBlock_t) / 4 - 1)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid DJB report block length (%d)", blockLen);
                ret = -1;
                break;
            }

            djbReportCtx->djbNominal = ntohs(djbReport->djbNominal);
            djbReportCtx->djbMax = ntohs(djbReport->djbMax);
            djbReportCtx->djbHighWatermark = ntohs(djbReport->djbHighWatermark);
            djbReportCtx->djbLowWatermark = ntohs(djbReport->djbLowWatermark);
            if ((djbReport->djbNominal == 0xFFFF) && (djbReport->djbMax == 0xFFFF) && (djbReport->djbHighWatermark == 0xFFFF) && (djbReport->djbLowWatermark == 0xFFFF))
            {
                djbReportCtx->djbMetricsAvailable = 0;
            }
            else
            {
                djbReportCtx->djbMetricsAvailable = 1;
            }

            djbReportCtx->lastReceptionTimestamp = receptionTimestamp;
            if (gotDjbReport)
            {
                *gotDjbReport = 1;
            }
        }
        processedLen += ntohs(*((uint16_t*)(buffer + processedLen * 4 + 4 + 2))) + 1;
    }

    return ret;
}


int ARSTREAM2_RTCP_GetApplicationPacketSubtype(const uint8_t *buffer, unsigned int bufferSize)
{
    const ARSTREAM2_RTCP_Application_t *app = (const ARSTREAM2_RTCP_Application_t*)buffer;

    if (!buffer)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }
    if (bufferSize < sizeof(ARSTREAM2_RTCP_Application_t))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size");
        return -1;
    }

    uint8_t version = (app->flags >> 6) & 0x3;
    if (version != 2)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet protocol version (%d)", version);
        return -1;
    }

    if (app->packetType != ARSTREAM2_RTCP_APP_PACKET_TYPE)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet type (%d)", app->packetType);
        return -1;
    }

    uint32_t name = ntohl(app->name);
    if (name != ARSTREAM2_RTCP_APP_PACKET_NAME)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet name (0x%08X)", name);
        return -1;
    }

    uint16_t length = ntohs(app->length);
    if ((unsigned int)length * 4 + 4 > bufferSize)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid length (%d -> %d bytes) for %d bytes buffer size", length, (unsigned int)length * 4 + 4, bufferSize);
        return -1;
    }
    if (length < 2)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet length");
        return -1;
    }

    uint8_t subType = (app->flags & 0x1F);

    return subType;
}


int ARSTREAM2_RTCP_GenerateApplicationClockDelta(ARSTREAM2_RTCP_Application_t *app, ARSTREAM2_RTCP_ClockDelta_t *clockDelta,
                                                 uint64_t sendTimestamp, uint32_t ssrc,
                                                 ARSTREAM2_RTCP_ClockDeltaContext_t *context)
{
    if ((!app) || (!clockDelta) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }

    app->flags = (2 << 6) | (ARSTREAM2_RTCP_APP_PACKET_CLOCKDELTA_SUBTYPE & 0x1F);
    app->packetType = ARSTREAM2_RTCP_APP_PACKET_TYPE;
    app->length = htons((sizeof(ARSTREAM2_RTCP_Application_t) + sizeof(ARSTREAM2_RTCP_ClockDelta_t)) / 4 - 1);
    app->ssrc = htonl(ssrc);
    app->name = htonl(ARSTREAM2_RTCP_APP_PACKET_NAME);

    clockDelta->originateTimestampH = htonl((uint32_t)(context->nextPeerOriginateTimestamp >> 32));
    clockDelta->originateTimestampL = htonl((uint32_t)(context->nextPeerOriginateTimestamp & 0xFFFFFFFF));
    clockDelta->receiveTimestampH = htonl((uint32_t)(context->nextReceiveTimestamp >> 32));
    clockDelta->receiveTimestampL = htonl((uint32_t)(context->nextReceiveTimestamp & 0xFFFFFFFF));
    clockDelta->transmitTimestampH = htonl((uint32_t)(sendTimestamp >> 32));
    clockDelta->transmitTimestampL = htonl((uint32_t)(sendTimestamp & 0xFFFFFFFF));

    context->expectedOriginateTimestamp = sendTimestamp;

    return 0;
}


int ARSTREAM2_RTCP_ProcessApplicationClockDelta(const uint8_t *buffer, unsigned int bufferSize,
                                                uint64_t receptionTimestamp, uint32_t peerSsrc,
                                                ARSTREAM2_RTCP_ClockDeltaContext_t *context)
{
    const ARSTREAM2_RTCP_Application_t *app = (const ARSTREAM2_RTCP_Application_t*)buffer;
    const ARSTREAM2_RTCP_ClockDelta_t *clockDelta = (const ARSTREAM2_RTCP_ClockDelta_t*)(buffer + sizeof(ARSTREAM2_RTCP_Application_t));

    if ((!buffer) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }
    if (bufferSize < sizeof(ARSTREAM2_RTCP_Application_t) + sizeof(ARSTREAM2_RTCP_ClockDelta_t))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size");
        return -1;
    }

    uint8_t version = (app->flags >> 6) & 0x3;
    if (version != 2)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet protocol version (%d)", version);
        return -1;
    }

    if (app->packetType != ARSTREAM2_RTCP_APP_PACKET_TYPE)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet type (%d)", app->packetType);
        return -1;
    }

    uint32_t name = ntohl(app->name);
    if (name != ARSTREAM2_RTCP_APP_PACKET_NAME)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet name (0x%08X)", name);
        return -1;
    }

    uint8_t subType = (app->flags & 0x1F);
    if (subType != ARSTREAM2_RTCP_APP_PACKET_CLOCKDELTA_SUBTYPE)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet subtype (%d)", subType);
        return -1;
    }

    uint32_t ssrc = ntohl(app->ssrc);
    if (ssrc != peerSsrc)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "Unexpected peer SSRC");
        return -1;
    }

    uint16_t length = ntohs(app->length);
    if ((unsigned int)length * 4 + 4 > bufferSize)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid length (%d -> %d bytes) for %d bytes buffer size", length, (unsigned int)length * 4 + 4, bufferSize);
        return -1;
    }
    if (length != (sizeof(ARSTREAM2_RTCP_Application_t) + sizeof(ARSTREAM2_RTCP_ClockDelta_t)) / 4 - 1)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet length (%d)", length);
        return -1;
    }

    uint32_t originateTimestampH = ntohl(clockDelta->originateTimestampH);
    uint32_t originateTimestampL = ntohl(clockDelta->originateTimestampL);
    uint64_t originateTimestamp = ((uint64_t)originateTimestampH << 32) + ((uint64_t)originateTimestampL & 0xFFFFFFFF);
    uint32_t receiveTimestampH = ntohl(clockDelta->receiveTimestampH);
    uint32_t receiveTimestampL = ntohl(clockDelta->receiveTimestampL);
    uint64_t peerReceiveTimestamp = ((uint64_t)receiveTimestampH << 32) + ((uint64_t)receiveTimestampL & 0xFFFFFFFF);
    uint32_t transmitTimestampH = ntohl(clockDelta->transmitTimestampH);
    uint32_t transmitTimestampL = ntohl(clockDelta->transmitTimestampL);
    uint64_t peerTransmitTimestamp = ((uint64_t)transmitTimestampH << 32) + ((uint64_t)transmitTimestampL & 0xFFFFFFFF);

    if ((receptionTimestamp != 0) && (originateTimestamp != 0) && (peerReceiveTimestamp != 0) && (peerTransmitTimestamp != 0)
            && (peerTransmitTimestamp >= peerReceiveTimestamp + ARSTREAM2_RTCP_CLOCKDELTA_MIN_TS_DELTA)
            && (receptionTimestamp >= originateTimestamp + ARSTREAM2_RTCP_CLOCKDELTA_MIN_TS_DELTA)
            && (originateTimestamp == context->expectedOriginateTimestamp))
    {
        int64_t rtDelay, clockDelta;
        int64_t peer2meDelay, me2peerDelay, oneWayDelayDiff;
        int ws = 0;
        rtDelay = ((int64_t)receptionTimestamp - (int64_t)originateTimestamp) - ((int64_t)peerTransmitTimestamp - (int64_t)peerReceiveTimestamp);
        clockDelta = ((int64_t)peerReceiveTimestamp + (int64_t)peerTransmitTimestamp - (int64_t)originateTimestamp - (int64_t)receptionTimestamp + 1) / 2;
        peer2meDelay = originateTimestamp - peerReceiveTimestamp + context->clockDeltaAvg;
        peer2meDelay = (peer2meDelay < 0) ? -peer2meDelay : peer2meDelay;
        me2peerDelay = receptionTimestamp - peerTransmitTimestamp + context->clockDeltaAvg;
        me2peerDelay = (me2peerDelay < 0) ? -me2peerDelay : me2peerDelay;
        oneWayDelayDiff = peer2meDelay - me2peerDelay;
        oneWayDelayDiff = (oneWayDelayDiff < 0) ? -oneWayDelayDiff : oneWayDelayDiff;

        if (rtDelay > 0)
        {
            /* Average RTD */
            if (context->rtDelayAvg == 0)
            {
                context->rtDelayAvg = rtDelay;
            }
            else
            {
                /* Sliding average, alpha = 1 / ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA_LONG */
                context->rtDelayAvg = context->rtDelayAvg + (rtDelay - context->rtDelayAvg + ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA_LONG / 2) / ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA_LONG;
            }
            if (context->clockDeltaAvg != 0)
            {
                /* Average peer2me delay */
                if (context->p2mDelayAvg == 0)
                {
                    context->p2mDelayAvg = peer2meDelay;
                }
                else
                {
                    /* Sliding average, alpha = 1 / ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA_LONG */
                    context->p2mDelayAvg = context->p2mDelayAvg + (peer2meDelay - context->p2mDelayAvg + ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA_LONG / 2) / ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA_LONG;
                }

                /* Average me2peer delay */
                if (context->m2pDelayAvg == 0)
                {
                    context->m2pDelayAvg = me2peerDelay;
                }
                else
                {
                    /* Sliding average, alpha = 1 / ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA_LONG */
                    context->m2pDelayAvg = context->m2pDelayAvg + (me2peerDelay - context->m2pDelayAvg + ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA_LONG / 2) / ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA_LONG;
                }
            }

            if ((context->clockDeltaAvg == 0) || ((float)oneWayDelayDiff <= (float)rtDelay * ARSTREAM2_RTCP_CLOCKDELTA_ONE_WAY_DELAY_DIFF_THRES))
            {
                /* Initialize the window */
                if (context->windowSize == 0)
                {
                    context->windowStartTimestamp = receptionTimestamp;
                }

                /* Fill the window */
                context->clockDeltaWindow[context->windowSize] = clockDelta;
                context->rtDelayWindow[context->windowSize] = rtDelay;
                context->windowSize++;

                if ((context->windowSize >= ARSTREAM2_RTCP_CLOCKDELTA_WINDOW_SIZE) || ((context->windowSize >= ARSTREAM2_RTCP_CLOCKDELTA_WINDOW_SIZE / 2) && (receptionTimestamp >= context->windowStartTimestamp + ARSTREAM2_RTCP_CLOCKDELTA_WINDOW_TIMEOUT)))
                {
                    /* Window is full or half-full and on timeout */
                    int i;
                    int64_t minRtDelay = 10000000;
                    ws = context->windowSize;

                    /* Find the minimum round trip delay */
                    for (i = 0; i < context->windowSize; i++)
                    {
                        if (context->rtDelayWindow[i] < minRtDelay)
                        {
                            minRtDelay = context->rtDelayWindow[i];
                            context->clockDelta = context->clockDeltaWindow[i];
                        }
                    }

                    if (minRtDelay < ARSTREAM2_RTCP_CLOCKDELTA_MAX_RTDELAY)
                    {
                        /* Min RTD is acceptable */

                        /* Average min RTD */
                        if (context->rtDelayMinAvg == 0)
                        {
                            context->rtDelayMinAvg = minRtDelay;
                        }
                        else
                        {
                            /* Sliding average, alpha = 1 / ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA */
                            context->rtDelayMinAvg = context->rtDelayMinAvg + (minRtDelay - context->rtDelayMinAvg + ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA / 2) / ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA;
                        }

                        if (minRtDelay <= context->rtDelayMinAvg * 2)
                        {
                            /* Min RTD is less than 200% of the average RTD */

                            /* Average clock delta */
                            if (context->clockDeltaAvg == 0)
                            {
                                context->clockDeltaAvg = context->clockDelta;
                            }
                            else
                            {
                                /* Sliding average, alpha = 1 / ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA */
                                context->clockDeltaAvg = context->clockDeltaAvg + (context->clockDelta - context->clockDeltaAvg + ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA / 2) / ARSTREAM2_RTCP_CLOCKDELTA_AVG_ALPHA;
                            }
                        }
                    }

                    /* Reset the window */
                    context->windowSize = 0;
                }
            }
        }

        context->expectedOriginateTimestamp = 0;
    }

    context->nextPeerOriginateTimestamp = peerTransmitTimestamp;
    context->nextReceiveTimestamp = receptionTimestamp;

    return 0;
}


int ARSTREAM2_RTCP_GenerateApplicationVideoStats(ARSTREAM2_RTCP_Application_t *app, ARSTREAM2_RTCP_VideoStats_t *videoStats,
                                                 unsigned int maxSize, uint64_t sendTimestamp,
                                                 uint32_t ssrc, ARSTREAM2_RTCP_VideoStatsContext_t *context, unsigned int *size)
{
    uint32_t i, j;

    if ((!app) || (!videoStats) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }
    if (!context->videoStats.mbStatusClassCount)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid class count");
        return -1;
    }
    if (!context->videoStats.mbStatusZoneCount)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid zone count");
        return -1;
    }

    unsigned int _size = sizeof(ARSTREAM2_RTCP_Application_t) + sizeof(ARSTREAM2_RTCP_VideoStats_t) + (context->videoStats.mbStatusClassCount * context->videoStats.mbStatusZoneCount + context->videoStats.mbStatusZoneCount) * 4;
    if (_size > maxSize)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Buffer is too small for video stats");
        return -1;
    }

    app->flags = (2 << 6) | (ARSTREAM2_RTCP_APP_PACKET_VIDEOSTATS_SUBTYPE & 0x1F);
    app->packetType = ARSTREAM2_RTCP_APP_PACKET_TYPE;
    app->length = htons(_size / 4 - 1);
    app->ssrc = htonl(ssrc);
    app->name = htonl(ARSTREAM2_RTCP_APP_PACKET_NAME);

    videoStats->version = ARSTREAM2_RTCP_VIDEOSTATS_VERSION;
    videoStats->rssi = context->videoStats.rssi;
    videoStats->reserved1 = videoStats->reserved2 = 0;
    videoStats->timestampH = htonl((uint32_t)(context->videoStats.timestamp >> 32));
    videoStats->timestampL = htonl((uint32_t)(context->videoStats.timestamp & 0xFFFFFFFF));
    videoStats->totalFrameCount = htonl(context->videoStats.totalFrameCount);
    videoStats->outputFrameCount = htonl(context->videoStats.outputFrameCount);
    videoStats->erroredOutputFrameCount = htonl(context->videoStats.erroredOutputFrameCount);
    videoStats->missedFrameCount = htonl(context->videoStats.missedFrameCount);
    videoStats->discardedFrameCount = htonl(context->videoStats.discardedFrameCount);
    videoStats->timestampDeltaIntegralH = htonl((uint32_t)(context->videoStats.timestampDeltaIntegral >> 32));
    videoStats->timestampDeltaIntegralL = htonl((uint32_t)(context->videoStats.timestampDeltaIntegral & 0xFFFFFFFF));
    videoStats->timestampDeltaIntegralSqH = htonl((uint32_t)(context->videoStats.timestampDeltaIntegralSq >> 32));
    videoStats->timestampDeltaIntegralSqL = htonl((uint32_t)(context->videoStats.timestampDeltaIntegralSq & 0xFFFFFFFF));
    videoStats->timingErrorIntegralH = htonl((uint32_t)(context->videoStats.timingErrorIntegral >> 32));
    videoStats->timingErrorIntegralL = htonl((uint32_t)(context->videoStats.timingErrorIntegral & 0xFFFFFFFF));
    videoStats->timingErrorIntegralSqH = htonl((uint32_t)(context->videoStats.timingErrorIntegralSq >> 32));
    videoStats->timingErrorIntegralSqL = htonl((uint32_t)(context->videoStats.timingErrorIntegralSq & 0xFFFFFFFF));
    videoStats->estimatedLatencyIntegralH = htonl((uint32_t)(context->videoStats.estimatedLatencyIntegral >> 32));
    videoStats->estimatedLatencyIntegralL = htonl((uint32_t)(context->videoStats.estimatedLatencyIntegral & 0xFFFFFFFF));
    videoStats->estimatedLatencyIntegralSqH = htonl((uint32_t)(context->videoStats.estimatedLatencyIntegralSq >> 32));
    videoStats->estimatedLatencyIntegralSqL = htonl((uint32_t)(context->videoStats.estimatedLatencyIntegralSq & 0xFFFFFFFF));
    videoStats->erroredSecondCount = htonl(context->videoStats.erroredSecondCount);
    videoStats->mbStatusClassCount = htonl(context->videoStats.mbStatusClassCount);
    videoStats->mbStatusZoneCount = htonl(context->videoStats.mbStatusZoneCount);
    uint32_t *videoStatsArr = (uint32_t*)&videoStats->mbStatusZoneCount + 1;
    for (i = 0; i < context->videoStats.mbStatusZoneCount; i++)
    {
        *videoStatsArr++ = htonl(context->videoStats.erroredSecondCountByZone[i]);
    }
    for (j = 0; j < context->videoStats.mbStatusClassCount; j++)
    {
        for (i = 0; i < context->videoStats.mbStatusZoneCount; i++)
        {
            *videoStatsArr++ = htonl(context->videoStats.macroblockStatus[j][i]);
        }
    }

    if (size)
        *size = _size;

    return 0;
}


int ARSTREAM2_RTCP_ProcessApplicationVideoStats(const uint8_t *buffer, unsigned int bufferSize,
                                                uint64_t receptionTimestamp, uint32_t peerSsrc,
                                                ARSTREAM2_RTCP_VideoStatsContext_t *context, int *gotVideoStats)
{
    const ARSTREAM2_RTCP_Application_t *app = (const ARSTREAM2_RTCP_Application_t*)buffer;
    const ARSTREAM2_RTCP_VideoStats_t *videoStats = (const ARSTREAM2_RTCP_VideoStats_t*)(buffer + sizeof(ARSTREAM2_RTCP_Application_t));
    uint32_t i, j;

    if (gotVideoStats)
    {
        *gotVideoStats = 0;
    }

    if ((!buffer) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }
    if (bufferSize < sizeof(ARSTREAM2_RTCP_Application_t) + sizeof(ARSTREAM2_RTCP_VideoStats_t))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid buffer size");
        return -1;
    }

    uint8_t version = (app->flags >> 6) & 0x3;
    if (version != 2)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet protocol version (%d)", version);
        return -1;
    }

    if (app->packetType != ARSTREAM2_RTCP_APP_PACKET_TYPE)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet type (%d)", app->packetType);
        return -1;
    }

    uint32_t name = ntohl(app->name);
    if (name != ARSTREAM2_RTCP_APP_PACKET_NAME)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet name (0x%08X)", name);
        return -1;
    }

    uint8_t subType = (app->flags & 0x1F);
    if (subType != ARSTREAM2_RTCP_APP_PACKET_VIDEOSTATS_SUBTYPE)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet subtype (%d)", subType);
        return -1;
    }

    uint32_t ssrc = ntohl(app->ssrc);
    if (ssrc != peerSsrc)
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTCP_TAG, "Unexpected peer SSRC");
        return -1;
    }

    if (videoStats->version != ARSTREAM2_RTCP_VIDEOSTATS_VERSION)
    {
        ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTCP_TAG, "Unexpected video stats version");
        return 0;
    }

    uint16_t length = ntohs(app->length);
    if ((unsigned int)length * 4 + 4 > bufferSize)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid length (%d -> %d bytes) for %d bytes buffer size", length, (unsigned int)length * 4 + 4, bufferSize);
        return -1;
    }
    if (length < (sizeof(ARSTREAM2_RTCP_Application_t) + sizeof(ARSTREAM2_RTCP_VideoStats_t)) / 4 - 1)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet length (%d)", length);
        return -1;
    }

    context->videoStats.rssi = videoStats->rssi;
    context->videoStats.timestamp = ((uint64_t)ntohl(videoStats->timestampH) << 32) + (uint64_t)ntohl(videoStats->timestampL);
    context->videoStats.totalFrameCount = ntohl(videoStats->totalFrameCount);
    context->videoStats.outputFrameCount = ntohl(videoStats->outputFrameCount);
    context->videoStats.erroredOutputFrameCount = ntohl(videoStats->erroredOutputFrameCount);
    context->videoStats.missedFrameCount = ntohl(videoStats->missedFrameCount);
    context->videoStats.discardedFrameCount = ntohl(videoStats->discardedFrameCount);
    context->videoStats.timestampDeltaIntegral = ((uint64_t)ntohl(videoStats->timestampDeltaIntegralH) << 32) + (uint64_t)ntohl(videoStats->timestampDeltaIntegralL);
    context->videoStats.timestampDeltaIntegralSq = ((uint64_t)ntohl(videoStats->timestampDeltaIntegralSqH) << 32) + (uint64_t)ntohl(videoStats->timestampDeltaIntegralSqL);
    context->videoStats.timingErrorIntegral = ((uint64_t)ntohl(videoStats->timingErrorIntegralH) << 32) + (uint64_t)ntohl(videoStats->timingErrorIntegralL);
    context->videoStats.timingErrorIntegralSq = ((uint64_t)ntohl(videoStats->timingErrorIntegralSqH) << 32) + (uint64_t)ntohl(videoStats->timingErrorIntegralSqL);
    context->videoStats.estimatedLatencyIntegral = ((uint64_t)ntohl(videoStats->estimatedLatencyIntegralH) << 32) + (uint64_t)ntohl(videoStats->estimatedLatencyIntegralL);
    context->videoStats.estimatedLatencyIntegralSq = ((uint64_t)ntohl(videoStats->estimatedLatencyIntegralSqH) << 32) + (uint64_t)ntohl(videoStats->estimatedLatencyIntegralSqL);
    context->videoStats.erroredSecondCount = ntohl(videoStats->erroredSecondCount);
    context->videoStats.mbStatusClassCount = ntohl(videoStats->mbStatusClassCount);
    context->videoStats.mbStatusZoneCount = ntohl(videoStats->mbStatusZoneCount);

    if (context->videoStats.mbStatusClassCount > ARSTREAM2_H264_MB_STATUS_CLASS_MAX_COUNT)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid video stats class count (%d)", context->videoStats.mbStatusClassCount);
        return -1;
    }
    if (context->videoStats.mbStatusZoneCount > ARSTREAM2_H264_MB_STATUS_ZONE_MAX_COUNT)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid video stats zone count (%d)", context->videoStats.mbStatusZoneCount);
        return -1;
    }

    if (length < (sizeof(ARSTREAM2_RTCP_Application_t) + sizeof(ARSTREAM2_RTCP_VideoStats_t) + (context->videoStats.mbStatusClassCount * context->videoStats.mbStatusZoneCount + context->videoStats.mbStatusZoneCount) * 4) / 4 - 1)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid application packet length (%d)", length);
        return -1;
    }

    uint32_t *videoStatsArr = (uint32_t*)&videoStats->mbStatusZoneCount + 1;
    for (i = 0; i < context->videoStats.mbStatusZoneCount; i++)
    {
        context->videoStats.erroredSecondCountByZone[i] = ntohl(*videoStatsArr++);
    }

    for (j = 0; j < context->videoStats.mbStatusClassCount; j++)
    {
        for (i = 0; i < context->videoStats.mbStatusZoneCount; i++)
        {
            context->videoStats.macroblockStatus[j][i] = ntohl(*videoStatsArr++);
        }
    }

    if (gotVideoStats)
    {
        *gotVideoStats = 1;
    }

    return 0;
}


int ARSTREAM2_RTCP_Sender_GenerateCompoundPacket(uint8_t *packet, unsigned int maxPacketSize,
                                                 uint64_t sendTimestamp, int generateSenderReport,
                                                 int generateSourceDescription, int generateApplicationClockDelta,
                                                 uint32_t packetCount, uint64_t byteCount,
                                                 ARSTREAM2_RTCP_SenderContext_t *context,
                                                 unsigned int *size)
{
    int ret = 0;
    unsigned int totalSize = 0;

    if ((!packet) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }

    if (maxPacketSize == 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid max packet size");
        return -1;
    }

    if ((ret == 0) && (generateSenderReport) && (totalSize + sizeof(ARSTREAM2_RTCP_SenderReport_t) <= maxPacketSize))
    {
        ret = ARSTREAM2_RTCP_Sender_GenerateSenderReport((ARSTREAM2_RTCP_SenderReport_t*)(packet + totalSize),
                                                         sendTimestamp, packetCount, byteCount, context);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to generate sender report (%d)", ret);
        }
        else
        {
            totalSize += sizeof(ARSTREAM2_RTCP_SenderReport_t);
        }
    }

    if ((ret == 0) && (generateSourceDescription))
    {
        unsigned int sdesSize = 0;
        ret = ARSTREAM2_RTCP_GenerateSourceDescription((ARSTREAM2_RTCP_Sdes_t*)(packet + totalSize), maxPacketSize - totalSize,
                                                       context->senderSsrc, sendTimestamp,
                                                       context->sdesItem, context->sdesItemCount, &sdesSize);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to generate source description (%d)", ret);
        }
        else
        {
            totalSize += sdesSize;
        }
    }

    if ((ret == 0) && (generateApplicationClockDelta) && (totalSize + sizeof(ARSTREAM2_RTCP_Application_t) + sizeof(ARSTREAM2_RTCP_ClockDelta_t) <= maxPacketSize))
    {
        if ((context->clockDeltaCtx.expectedOriginateTimestamp == 0) || (sendTimestamp >= context->clockDeltaCtx.expectedOriginateTimestamp + ARSTREAM2_RTCP_CLOCKDELTA_TIMEOUT))
        {
            ret = ARSTREAM2_RTCP_GenerateApplicationClockDelta((ARSTREAM2_RTCP_Application_t*)(packet + totalSize),
                                                               (ARSTREAM2_RTCP_ClockDelta_t*)(packet + totalSize + sizeof(ARSTREAM2_RTCP_Application_t)),
                                                               sendTimestamp, context->senderSsrc,
                                                               &context->clockDeltaCtx);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to generate application defined clock delta (%d)", ret);
            }
            else
            {
                totalSize += sizeof(ARSTREAM2_RTCP_Application_t) + sizeof(ARSTREAM2_RTCP_ClockDelta_t);
            }
        }
    }

    if (size) *size = totalSize;
    return ret;
}


int ARSTREAM2_RTCP_Receiver_GenerateCompoundPacket(uint8_t *packet, unsigned int maxPacketSize,
                                                   uint64_t sendTimestamp, int generateReceiverReport,
                                                   int generateSourceDescription, int generateApplicationClockDelta,
                                                   int generateApplicationVideoStats, int generateLossReport, int generateDjbReport,
                                                   ARSTREAM2_RTCP_ReceiverContext_t *context, unsigned int *size)
{
    int ret = 0;
    unsigned int totalSize = 0;

    if ((!packet) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }

    if (maxPacketSize == 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid max packet size");
        return -1;
    }

    if ((ret == 0) && (generateReceiverReport) && (totalSize + sizeof(ARSTREAM2_RTCP_ReceiverReport_t) + sizeof(ARSTREAM2_RTCP_ReceptionReportBlock_t) <= maxPacketSize))
    {
        unsigned int rrSize = 0;
        ret = ARSTREAM2_RTCP_Receiver_GenerateReceiverReport((ARSTREAM2_RTCP_ReceiverReport_t*)(packet + totalSize),
                                                             (ARSTREAM2_RTCP_ReceptionReportBlock_t*)(packet + totalSize + sizeof(ARSTREAM2_RTCP_ReceiverReport_t)),
                                                             sendTimestamp, context, &rrSize);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to generate receiver report (%d)", ret);
        }
        else
        {
            totalSize += rrSize;
        }
    }

    if ((ret == 0) && ((generateLossReport) || (generateDjbReport)))
    {
        unsigned int extendedReportSize = 0;
        ret = ARSTREAM2_RTCP_GenerateExtendedReport((ARSTREAM2_RTCP_ExtendedReport_t*)(packet + totalSize),
                                                    maxPacketSize - totalSize, sendTimestamp, context->receiverSsrc, context->senderSsrc,
                                                    (generateLossReport) ? &context->lossReportCtx : NULL,
                                                    (generateDjbReport) ? &context->djbReportCtx : NULL, &extendedReportSize);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to generate extended report (%d)", ret);
        }
        else
        {
            totalSize += extendedReportSize;
        }
    }

    if ((ret == 0) && (generateSourceDescription))
    {
        unsigned int sdesSize = 0;
        ret = ARSTREAM2_RTCP_GenerateSourceDescription((ARSTREAM2_RTCP_Sdes_t*)(packet + totalSize), maxPacketSize - totalSize,
                                                       context->receiverSsrc, sendTimestamp,
                                                       context->sdesItem, context->sdesItemCount, &sdesSize);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to generate source description (%d)", ret);
        }
        else
        {
            totalSize += sdesSize;
        }
    }

    if ((ret == 0) && (generateApplicationClockDelta) && (totalSize + sizeof(ARSTREAM2_RTCP_Application_t) + sizeof(ARSTREAM2_RTCP_ClockDelta_t) <= maxPacketSize))
    {
        if ((context->clockDeltaCtx.expectedOriginateTimestamp == 0) || (sendTimestamp >= context->clockDeltaCtx.expectedOriginateTimestamp + ARSTREAM2_RTCP_CLOCKDELTA_TIMEOUT))
        {
            ret = ARSTREAM2_RTCP_GenerateApplicationClockDelta((ARSTREAM2_RTCP_Application_t*)(packet + totalSize),
                                                               (ARSTREAM2_RTCP_ClockDelta_t*)(packet + totalSize + sizeof(ARSTREAM2_RTCP_Application_t)),
                                                               sendTimestamp, context->receiverSsrc,
                                                               &context->clockDeltaCtx);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to generate application defined clock delta (%d)", ret);
            }
            else
            {
                totalSize += sizeof(ARSTREAM2_RTCP_Application_t) + sizeof(ARSTREAM2_RTCP_ClockDelta_t);
            }
        }
    }

    if ((ret == 0) && (generateApplicationVideoStats))
    {
        unsigned int videoStatsSize = 0;
        ret = ARSTREAM2_RTCP_GenerateApplicationVideoStats((ARSTREAM2_RTCP_Application_t*)(packet + totalSize),
                                                           (ARSTREAM2_RTCP_VideoStats_t*)(packet + totalSize + sizeof(ARSTREAM2_RTCP_Application_t)),
                                                           maxPacketSize - totalSize, sendTimestamp, context->receiverSsrc,
                                                           &context->videoStatsCtx, &videoStatsSize);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to generate application defined video stats (%d)", ret);
        }
        else
        {
            totalSize += videoStatsSize;
        }
    }

    if (size) *size = totalSize;
    return ret;
}


int ARSTREAM2_RTCP_Sender_ProcessCompoundPacket(const uint8_t *buffer, unsigned int bufferSize,
                                                uint64_t receptionTimestamp,
                                                ARSTREAM2_RTCP_SenderContext_t *context,
                                                int *gotReceptionReport, int *gotVideoStats,
                                                int *gotLossReport, int *gotDjbReport)
{
    unsigned int readSize = 0, size = 0;
    int receptionReportCount = 0, type, subType, ret, _ret = 0;

    if ((!buffer) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }

    while (readSize < bufferSize)
    {
        type = ARSTREAM2_RTCP_GetPacketType(buffer, bufferSize - readSize, &receptionReportCount, &size);
        if ((type < 0) || (readSize + size > bufferSize))
        {
            _ret = -1;
            break;
        }
        switch (type)
        {
            case ARSTREAM2_RTCP_RECEIVER_REPORT_PACKET_TYPE:
                if (receptionReportCount > 0)
                {
                    ret = ARSTREAM2_RTCP_Sender_ProcessReceiverReport(buffer, bufferSize - readSize,
                                                                      receptionTimestamp,
                                                                      context, gotReceptionReport);
                    if (ret != 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to process receiver report (%d)", ret);
                    }
                    else
                    {
                        /*ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTCP_TAG, "Receiver state: RTD=%.1fms interarrivalJitter=%.1fms lost=%d lastLossRate=%.1f%% highestSeqNum=%d",
                                    (float)context->roundTripDelay / 1000., (float)context->interarrivalJitter / 1000.,
                                    context->receiverLostCount, (float)context->receiverFractionLost * 100. / 256.,
                                    context->receiverExtHighestSeqNum);*/
                    }
                }
                break;
            case ARSTREAM2_RTCP_EXTENDED_REPORT_PACKET_TYPE:
                ret = ARSTREAM2_RTCP_ProcessExtendedReport(buffer, bufferSize - readSize, receptionTimestamp,
                                                           context->receiverSsrc, context->senderSsrc,
                                                           &context->lossReportCtx, &context->djbReportCtx,
                                                           gotLossReport, gotDjbReport);
                if (ret != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to process extended report (%d)", ret);
                }
                break;
            case ARSTREAM2_RTCP_SDES_PACKET_TYPE:
                ret = ARSTREAM2_RTCP_ProcessSourceDescription(buffer, bufferSize - readSize, context->peerSdesItem,
                                                              ARSTREAM2_RTCP_SDES_ITEM_MAX_COUNT, &context->peerSdesItemCount);
                if (ret != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to process source description (%d)", ret);
                }
                break;
            case ARSTREAM2_RTCP_APP_PACKET_TYPE:
                subType = ARSTREAM2_RTCP_GetApplicationPacketSubtype(buffer, bufferSize - readSize);
                switch (subType)
                {
                    case ARSTREAM2_RTCP_APP_PACKET_CLOCKDELTA_SUBTYPE:
                        ret = ARSTREAM2_RTCP_ProcessApplicationClockDelta(buffer, bufferSize - readSize,
                                                                          receptionTimestamp, context->receiverSsrc,
                                                                          &context->clockDeltaCtx);
                        if (ret != 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to process application clock delta (%d)", ret);
                        }
                        else
                        {
                            /*ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTCP_TAG, "Clock delta: delta=%lli RTD=%lli",
                                        context->clockDeltaCtx.clockDelta, context->clockDeltaCtx.rtDelay);*/
                        }
                        break;
                    case ARSTREAM2_RTCP_APP_PACKET_VIDEOSTATS_SUBTYPE:
                        ret = ARSTREAM2_RTCP_ProcessApplicationVideoStats(buffer, bufferSize - readSize,
                                                                          receptionTimestamp, context->receiverSsrc,
                                                                          &context->videoStatsCtx, gotVideoStats);
                        if (ret != 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to process application video stats (%d)", ret);
                        }
                        else
                        {
                            /*ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTCP_TAG, "Video stats");*/
                        }
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
        readSize += size;
        buffer += size;
    }

    return _ret;
}


int ARSTREAM2_RTCP_Receiver_ProcessCompoundPacket(const uint8_t *buffer, unsigned int bufferSize,
                                                  uint64_t receptionTimestamp,
                                                  ARSTREAM2_RTCP_ReceiverContext_t *context)
{
    unsigned int readSize = 0, size = 0;
    int type, subType, ret, _ret = 0;

    if ((!buffer) || (!context))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Invalid pointer");
        return -1;
    }

    while (readSize < bufferSize)
    {
        type = ARSTREAM2_RTCP_GetPacketType(buffer, bufferSize - readSize, NULL, &size);
        if ((type < 0) || (readSize + size > bufferSize))
        {
            _ret = -1;
            break;
        }
        switch (type)
        {
            case ARSTREAM2_RTCP_SENDER_REPORT_PACKET_TYPE:
                ret = ARSTREAM2_RTCP_Receiver_ProcessSenderReport(buffer, bufferSize - readSize,
                                                                  receptionTimestamp, context);
                if (ret != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to process sender report (%d)", ret);
                }
                else
                {
                    /*ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTCP_TAG, "Sender state: interval=%.1fms packetRate=%.1fpacket/s bitrate=%.2fkbit/s",
                                (float)context->lastSrInterval / 1000., (float)context->srIntervalPacketCount * 1000000. / (float)context->lastSrInterval,
                                (float)context->srIntervalByteCount * 8000. / (float)context->lastSrInterval);*/
                }
                break;
            case ARSTREAM2_RTCP_SDES_PACKET_TYPE:
                ret = ARSTREAM2_RTCP_ProcessSourceDescription(buffer, bufferSize - readSize, context->peerSdesItem,
                                                              ARSTREAM2_RTCP_SDES_ITEM_MAX_COUNT, &context->peerSdesItemCount);
                if (ret != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to process source description (%d)", ret);
                }
                break;
            case ARSTREAM2_RTCP_APP_PACKET_TYPE:
                subType = ARSTREAM2_RTCP_GetApplicationPacketSubtype(buffer, bufferSize - readSize);
                switch (subType)
                {
                    case ARSTREAM2_RTCP_APP_PACKET_CLOCKDELTA_SUBTYPE:
                        ret = ARSTREAM2_RTCP_ProcessApplicationClockDelta(buffer, bufferSize - readSize,
                                                                          receptionTimestamp, context->senderSsrc,
                                                                          &context->clockDeltaCtx);
                        if (ret != 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTCP_TAG, "Failed to process application clock delta (%d)", ret);
                        }
                        else
                        {
                            /*ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTCP_TAG, "Clock delta: delta=%lli RTD=%lli",
                                        context->clockDeltaCtx.clockDeltaAvg, context->clockDeltaCtx.rtDelay);*/
                        }
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
        readSize += size;
        buffer += size;
    }

    return _ret;
}
