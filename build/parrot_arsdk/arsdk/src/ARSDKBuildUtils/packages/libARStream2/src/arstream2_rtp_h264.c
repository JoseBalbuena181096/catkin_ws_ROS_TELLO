/**
 * @file arstream2_rtp_h264.c
 * @brief Parrot Streaming Library - RTP H.264 payloading implementation
 * @date 04/25/2016
 * @author aurelien.barre@parrot.com
 */

#include "arstream2_rtp_h264.h"

#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARStream2/arstream2_stream_sender.h>


/**
 * Tag for ARSAL_PRINT
 */
#define ARSTREAM2_RTPH264_TAG "ARSTREAM2_Rtp"


//TODO replace with FifoDequeue+FifoPushFreeItem
static int ARSTREAM2_RTPH264_FifoDequeueNalu(ARSTREAM2_H264_NaluFifo_t *fifo, ARSTREAM2_H264_NalUnit_t *nalu)
{
    ARSTREAM2_H264_NaluFifoItem_t* cur;

    if ((!fifo) || (!nalu))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Invalid pointer");
        return -1;
    }

    if ((!fifo->head) || (!fifo->count))
    {
        //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTPH264_TAG, "NALU FIFO is empty");
        return -2;
    }

    cur = ARSTREAM2_H264_NaluFifoDequeueItem(fifo);
    if (!cur)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to dequeue FIFO item");
        return -1;
    }

    memcpy(nalu, &cur->nalu, sizeof(ARSTREAM2_H264_NalUnit_t));

    int ret = ARSTREAM2_H264_NaluFifoPushFreeItem(fifo, cur);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to push free FIFO item");
        return -1;
    }

    return 0;
}


static int ARSTREAM2_RTPH264_Sender_SingleNaluPacket(ARSTREAM2_RTP_SenderContext_t *context,
                                                     ARSTREAM2_H264_NalUnit_t *nalu,
                                                     ARSTREAM2_RTP_PacketFifo_t *packetFifo,
                                                     ARSTREAM2_RTP_PacketFifoQueue_t *packetFifoQueue, uint64_t curTime)
{
    int ret = 0;

    ARSTREAM2_RTP_PacketFifoBuffer_t *buffer = ARSTREAM2_RTP_PacketFifoGetBuffer(packetFifo);
    ARSTREAM2_RTP_PacketFifoItem_t *item = ARSTREAM2_RTP_PacketFifoPopFreeItem(packetFifo);
    if ((item) && (buffer))
    {
        unsigned int offsetInBuffer = 0;
        uint8_t *payload = NULL, *headerExtension = NULL;
        unsigned int payloadSize = 0, headerExtensionSize = 0;
        ARSTREAM2_RTP_PacketReset(&item->packet);
        item->packet.buffer = buffer;

        if ((context->useRtpHeaderExtensions) && (nalu->metadata) && (nalu->metadataSize > 4))
        {
            uint32_t decodedHeaderExtensionSize = (uint32_t)ntohs(*((uint16_t*)nalu->metadata + 1)) * 4 + 4;
            if (decodedHeaderExtensionSize != nalu->metadataSize)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "RTP extension header size error: expected %d bytes, got length of %d bytes", nalu->metadataSize, decodedHeaderExtensionSize);
            }
            else
            {
                if (nalu->metadataSize <= context->maxPacketSize)
                {
                    memcpy(item->packet.buffer->buffer + offsetInBuffer, nalu->metadata, nalu->metadataSize);
                    headerExtension = item->packet.buffer->buffer + offsetInBuffer;
                    headerExtensionSize = nalu->metadataSize;
                    offsetInBuffer += nalu->metadataSize;
                }
                else
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Header extension size exceeds max packet size (%d)", context->maxPacketSize);
                }
            }
        }
        if (offsetInBuffer + nalu->naluSize <= context->maxPacketSize)
        {
            memcpy(item->packet.buffer->buffer + offsetInBuffer, nalu->nalu, nalu->naluSize);
            payload = item->packet.buffer->buffer + offsetInBuffer;
            payloadSize = nalu->naluSize;
            offsetInBuffer += nalu->naluSize;
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Payload size exceeds max packet size (%d)", context->maxPacketSize);
        }

        context->seqNum += nalu->seqNumForcedDiscontinuity;
        ret = ARSTREAM2_RTP_Sender_GeneratePacket(context, &item->packet,
                                                  payload, payloadSize,
                                                  (headerExtensionSize > 0) ? headerExtension : NULL, headerExtensionSize,
                                                  nalu->ntpTimestamp, nalu->inputTimestamp, nalu->timeoutTimestamp,
                                                  context->seqNum, nalu->isLastInAu, nalu->importance, nalu->priority);

        context->packetCount += nalu->seqNumForcedDiscontinuity + 1;
        context->byteCount += payloadSize;
        context->seqNum++;

        if (ret == 0)
        {
            ret = ARSTREAM2_RTP_PacketFifoEnqueueItemOrderedByPriority(packetFifoQueue, item);
            if (ret != 0)
            {
                ARSTREAM2_RTP_PacketFifoUnrefBuffer(packetFifo, item->packet.buffer);
                ARSTREAM2_RTP_PacketFifoPushFreeItem(packetFifo, item);
            }
        }
    }
    else
    {
        if (buffer) ARSTREAM2_RTP_PacketFifoUnrefBuffer(packetFifo, buffer);
        if (item) ARSTREAM2_RTP_PacketFifoPushFreeItem(packetFifo, item);
        int flushRet = ARSTREAM2_RTP_Sender_PacketFifoFlush(context, packetFifo, curTime);
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Packet FIFO is full => flush to recover (%d packets flushed)", flushRet);
        ret = -1;
    }

    return ret;
}


static int ARSTREAM2_RTPH264_Sender_FuAPackets(ARSTREAM2_RTP_SenderContext_t *context,
                                               ARSTREAM2_H264_NalUnit_t *nalu,
                                               unsigned int fragmentCount,
                                               ARSTREAM2_RTP_PacketFifo_t *packetFifo,
                                               ARSTREAM2_RTP_PacketFifoQueue_t *packetFifoQueue, uint64_t curTime)
{
    int ret, status = 0;
    unsigned int i, offset;
    uint8_t fuIndicator, fuHeader;
    fuIndicator = fuHeader = *nalu->nalu;
    fuIndicator &= ~0x1F;
    fuIndicator |= ARSTREAM2_RTPH264_NALU_TYPE_FUA;
    fuHeader &= ~0xE0;
    unsigned int meanFragmentSize = (nalu->naluSize + ((context->useRtpHeaderExtensions) ? nalu->metadataSize : 0) + fragmentCount / 2) / fragmentCount;

    for (i = 0, offset = 1; (i < fragmentCount) && (status == 0); i++)
    {
        unsigned int fragmentSize = (i == fragmentCount - 1) ? nalu->naluSize - offset : meanFragmentSize;
        unsigned int fragmentOffset = 0;
        do
        {
            unsigned int packetSize = (fragmentSize - fragmentOffset > context->maxPacketSize - 2) ? context->maxPacketSize - 2 : fragmentSize - fragmentOffset;
            if ((context->useRtpHeaderExtensions) && (offset == 1) && (nalu->metadataSize < packetSize))
            {
                packetSize -= nalu->metadataSize;
            }

            if (packetSize + 2 <= context->maxPacketSize)
            {
                ARSTREAM2_RTP_PacketFifoBuffer_t *buffer = ARSTREAM2_RTP_PacketFifoGetBuffer(packetFifo);
                ARSTREAM2_RTP_PacketFifoItem_t *item = ARSTREAM2_RTP_PacketFifoPopFreeItem(packetFifo);
                if ((item) && (buffer))
                {
                    unsigned int offsetInBuffer = 0;
                    uint8_t *payload = NULL, *headerExtension = NULL;
                    unsigned int payloadSize = 0, headerExtensionSize = 0;
                    uint8_t startBit = 0, endBit = 0;
                    ARSTREAM2_RTP_PacketReset(&item->packet);
                    item->packet.buffer = buffer;

                    if ((context->useRtpHeaderExtensions) && (offset == 1) && (nalu->metadata) && (nalu->metadataSize > 4) && (nalu->metadataSize < packetSize))
                    {
                        uint32_t decodedHeaderExtensionSize = (uint32_t)ntohs(*((uint16_t*)nalu->metadata + 1)) * 4 + 4;
                        if (decodedHeaderExtensionSize != nalu->metadataSize)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "RTP extension header size error: expected %d bytes, got length of %d bytes", nalu->metadataSize, decodedHeaderExtensionSize);
                        }
                        else
                        {
                            if (nalu->metadataSize <= context->maxPacketSize)
                            {
                                memcpy(item->packet.buffer->buffer + offsetInBuffer, nalu->metadata, nalu->metadataSize);
                                headerExtension = item->packet.buffer->buffer + offsetInBuffer;
                                headerExtensionSize = nalu->metadataSize;
                                offsetInBuffer += nalu->metadataSize;
                            }
                            else
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Header extension size exceeds max packet size (%d)", context->maxPacketSize);
                            }
                        }
                    }
                    if (offsetInBuffer + packetSize + 2 <= context->maxPacketSize)
                    {
                        memcpy(item->packet.buffer->buffer + offsetInBuffer + 2, nalu->nalu + offset, packetSize);
                        *(item->packet.buffer->buffer + offsetInBuffer) = fuIndicator;
                        startBit = (offset == 1) ? 0x80 : 0;
                        endBit = ((i == fragmentCount - 1) && (fragmentOffset + packetSize == fragmentSize)) ? 0x40 : 0;
                        *(item->packet.buffer->buffer + offsetInBuffer + 1) = fuHeader | startBit | endBit;
                        payload = item->packet.buffer->buffer + offsetInBuffer;
                        payloadSize = packetSize + 2;
                        offsetInBuffer += packetSize + 2;
                    }
                    else
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Payload size exceeds max packet size (%d)", context->maxPacketSize);
                    }

                    if (offset == 1) context->seqNum += nalu->seqNumForcedDiscontinuity;
                    ret = ARSTREAM2_RTP_Sender_GeneratePacket(context, &item->packet,
                                                              payload, payloadSize,
                                                              (headerExtensionSize > 0) ? headerExtension : NULL, headerExtensionSize,
                                                              nalu->ntpTimestamp, nalu->inputTimestamp, nalu->timeoutTimestamp,
                                                              context->seqNum, ((nalu->isLastInAu) && (endBit)) ? 1 : 0,
                                                              nalu->importance, nalu->priority);

                    context->packetCount += nalu->seqNumForcedDiscontinuity + 1;
                    context->byteCount += payloadSize;
                    context->seqNum++;

                    if (ret == 0)
                    {
                        ret = ARSTREAM2_RTP_PacketFifoEnqueueItemOrderedByPriority(packetFifoQueue, item);
                        if (ret != 0)
                        {
                            ARSTREAM2_RTP_PacketFifoUnrefBuffer(packetFifo, item->packet.buffer);
                            ARSTREAM2_RTP_PacketFifoPushFreeItem(packetFifo, item);
                        }
                    }
                }
                else
                {
                    if (buffer) ARSTREAM2_RTP_PacketFifoUnrefBuffer(packetFifo, buffer);
                    if (item) ARSTREAM2_RTP_PacketFifoPushFreeItem(packetFifo, item);
                    int flushRet = ARSTREAM2_RTP_Sender_PacketFifoFlush(context, packetFifo, curTime);
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Packet FIFO is full => flush to recover (%d packets flushed)", flushRet);
                    status = -1;
                    break;
                }
            }
            else
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Time %"PRIu64": FU-A, packetSize + 2 > maxPacketSize (packetSize=%d)", nalu->ntpTimestamp, packetSize);
            }

            fragmentOffset += packetSize;
            offset += packetSize;
        }
        while (fragmentOffset != fragmentSize);
    }

    return status;
}


static int ARSTREAM2_RTPH264_Sender_BeginStapAPacket(ARSTREAM2_RTP_SenderContext_t *context,
                                                     ARSTREAM2_H264_NalUnit_t *nalu,
                                                     ARSTREAM2_RTP_PacketFifo_t *packetFifo, uint64_t curTime)
{
    int ret = 0;

    ARSTREAM2_RTP_PacketFifoBuffer_t *buffer = ARSTREAM2_RTP_PacketFifoGetBuffer(packetFifo);
    context->stapItem = ARSTREAM2_RTP_PacketFifoPopFreeItem(packetFifo);
    if ((context->stapItem) && (buffer))
    {
        ARSTREAM2_RTP_PacketReset(&context->stapItem->packet);
        context->stapItem->packet.buffer = buffer;
        context->stapFirstNalu = 1;
        context->stapMaxNri = 0;
        context->stapImportance = 0;
        context->stapPriority = 0;
        context->stapOffsetInBuffer = 0;
        context->stapHeaderExtension = NULL;
        context->stapHeaderExtensionSize = 0;
        context->stapPayload = NULL;
        context->stapPayloadSize = 0;
        context->stapSeqNumForcedDiscontinuity = nalu->seqNumForcedDiscontinuity;
        context->stapNtpTimestamp = nalu->ntpTimestamp;
        context->stapInputTimestamp = nalu->inputTimestamp;
        context->stapTimeoutTimestamp = nalu->timeoutTimestamp;
        if ((context->useRtpHeaderExtensions) && (nalu->metadata) && (nalu->metadataSize > 4))
        {
            uint32_t headerExtensionSize = (uint32_t)ntohs(*((uint16_t*)nalu->metadata + 1)) * 4 + 4;
            if (headerExtensionSize != nalu->metadataSize)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "RTP extension header size error: expected %d bytes, got length of %d bytes", nalu->metadataSize, headerExtensionSize);
            }
            else
            {
                if (nalu->metadataSize <= context->maxPacketSize)
                {
                    memcpy(context->stapItem->packet.buffer->buffer + context->stapOffsetInBuffer, nalu->metadata, nalu->metadataSize);
                    context->stapHeaderExtension = context->stapItem->packet.buffer->buffer + context->stapOffsetInBuffer;
                    context->stapHeaderExtensionSize = nalu->metadataSize;
                    context->stapOffsetInBuffer += nalu->metadataSize;
                }
                else
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Header extension size exceeds max packet size (%d)", context->maxPacketSize);
                }
            }
        }
        if (context->stapOffsetInBuffer + 1 <= context->maxPacketSize)
        {
            context->stapPayload = context->stapItem->packet.buffer->buffer + context->stapOffsetInBuffer;
            context->stapPayloadSize = 1;
            context->stapOffsetInBuffer++;
            context->stapPending = 1;
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Payload size exceeds max packet size (%d)", context->maxPacketSize);
        }
    }
    else
    {
        if (buffer) ARSTREAM2_RTP_PacketFifoUnrefBuffer(packetFifo, buffer);
        if (context->stapItem) ARSTREAM2_RTP_PacketFifoPushFreeItem(packetFifo, context->stapItem);
        int flushRet = ARSTREAM2_RTP_Sender_PacketFifoFlush(context, packetFifo, curTime);
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Packet FIFO is full => flush to recover (%d packets flushed)", flushRet);
        ret = -1;
    }

    return ret;
}


static int ARSTREAM2_RTPH264_Sender_AppendToStapAPacket(ARSTREAM2_RTP_SenderContext_t *context,
                                                        ARSTREAM2_H264_NalUnit_t *nalu)
{
    int ret = 0;

    if (context->stapOffsetInBuffer + 2 + nalu->naluSize <= context->maxPacketSize)
    {
        uint8_t nri = ((uint8_t)(*(nalu->nalu)) >> 5) & 0x3;
        if (nri > context->stapMaxNri) context->stapMaxNri = nri;
        if (context->stapFirstNalu)
        {
            context->stapImportance = nalu->importance;
        }
        else
        {
            if (nalu->importance < context->stapImportance) context->stapImportance = nalu->importance;
        }
        if (context->stapFirstNalu)
        {
            context->stapPriority = nalu->priority;
        }
        else
        {
            if (nalu->priority < context->stapPriority) context->stapPriority = nalu->priority;
        }
        *(context->stapItem->packet.buffer->buffer + context->stapOffsetInBuffer) = ((nalu->naluSize >> 8) & 0xFF);
        *(context->stapItem->packet.buffer->buffer + context->stapOffsetInBuffer + 1) = (nalu->naluSize & 0xFF);
        context->stapPayloadSize += 2;
        context->stapOffsetInBuffer += 2;
        memcpy(context->stapItem->packet.buffer->buffer + context->stapOffsetInBuffer, nalu->nalu, nalu->naluSize);
        context->stapPayloadSize += nalu->naluSize;
        context->stapOffsetInBuffer += nalu->naluSize;
        context->stapFirstNalu = 0;
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Payload size exceeds max packet size (%d)", context->maxPacketSize);
    }

    return ret;
}


static int ARSTREAM2_RTPH264_Sender_FinishStapAPacket(ARSTREAM2_RTP_SenderContext_t *context,
                                                      ARSTREAM2_RTP_PacketFifo_t *packetFifo,
                                                      ARSTREAM2_RTP_PacketFifoQueue_t *packetFifoQueue, int markerBit)
{
    int ret = 0;
    uint8_t stapHeader;

    stapHeader = ARSTREAM2_RTPH264_NALU_TYPE_STAPA | ((context->stapMaxNri & 3) << 5);
    *(context->stapPayload) = stapHeader;
    context->seqNum += context->stapSeqNumForcedDiscontinuity;
    ret = ARSTREAM2_RTP_Sender_GeneratePacket(context, &context->stapItem->packet,
                                              context->stapPayload, context->stapPayloadSize,
                                              (context->stapHeaderExtensionSize > 0) ? context->stapHeaderExtension : NULL, context->stapHeaderExtensionSize,
                                              context->stapNtpTimestamp, context->stapInputTimestamp, context->stapTimeoutTimestamp,
                                              context->seqNum, markerBit, context->stapImportance, context->stapPriority);

    context->packetCount += context->stapSeqNumForcedDiscontinuity + 1;
    context->byteCount += context->stapPayloadSize;
    context->seqNum++;

    if (ret == 0)
    {
        ret = ARSTREAM2_RTP_PacketFifoEnqueueItemOrderedByPriority(packetFifoQueue, context->stapItem);
        if (ret != 0)
        {
            ARSTREAM2_RTP_PacketFifoUnrefBuffer(packetFifo, context->stapItem->packet.buffer);
            ARSTREAM2_RTP_PacketFifoPushFreeItem(packetFifo, context->stapItem);
        }
    }
    context->stapPayloadSize = 0;
    context->stapHeaderExtensionSize = 0;
    context->stapPending = 0;

    return ret;
}


int ARSTREAM2_RTPH264_Sender_NaluFifoToPacketFifo(ARSTREAM2_RTP_SenderContext_t *context,
                                                  ARSTREAM2_H264_NaluFifo_t *naluFifo,
                                                  ARSTREAM2_RTP_PacketFifo_t *packetFifo,
                                                  ARSTREAM2_RTP_PacketFifoQueue_t *packetFifoQueue,
                                                  uint64_t curTime, int dropOnTimeout, int *newPacketsCount)
{
    ARSTREAM2_H264_NalUnit_t nalu;
    int ret = 0, fifoRes, naluCount = 0, err;
    int initialPacketCount = packetFifoQueue->count;

    while ((fifoRes = ARSTREAM2_RTPH264_FifoDequeueNalu(naluFifo, &nalu)) == 0) //TODO replace with FifoDequeue+FifoPushFreeItem
    {
        naluCount++;
        if ((context->previousTimestamp != 0) && (nalu.ntpTimestamp != context->previousTimestamp))
        {
            if (context->stapPending)
            {
                /* Finish the previous STAP-A packet */
                err = ARSTREAM2_RTPH264_Sender_FinishStapAPacket(context, packetFifo, packetFifoQueue, 0); // do not set the marker bit
                if (err != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Sender_FinishStapAPacket() failed (%d)", err);
                }
            }

            if (context->auCallback != NULL)
            {
                /* new Access Unit: do we need to call the auCallback? */
                if (context->previousTimestamp != context->lastAuCallbackTimestamp)
                {
                    context->lastAuCallbackTimestamp = context->previousTimestamp;

                    /* call the auCallback */
                    ((ARSTREAM2_StreamSender_AuCallback_t)context->auCallback)(ARSTREAM2_STREAM_SENDER_STATUS_SENT, context->previousAuUserPtr, context->auCallbackUserPtr);
                }
            }
        }

        /* check that the NALU is not too old */
        if ((!dropOnTimeout) || ((nalu.timeoutTimestamp == 0) || (nalu.timeoutTimestamp > curTime)))
        {
            /* If target packet size is null, do not use aggregation (STAP-A):
             * only single-NALU and fragmentation (FU-A) to meet the maxPacketSize
             * requirement.
             * If target packet size is not null, use single-NALU, aggregation
             * (STAP-A) up to targetPacketSize and fragmentation (FU-A) to meet
             * both the targetPacketSize and maxPacketSize requirements.
             */

            /* Fragments count evaluation */
            unsigned int fragmentCount = (context->targetPacketSize)
                    ? (nalu.naluSize + ((context->useRtpHeaderExtensions) ? nalu.metadataSize : 0) + context->targetPacketSize / 2) / context->targetPacketSize
                    : 0;

            if ((fragmentCount > 1) || (nalu.naluSize > context->maxPacketSize))
            {
                /* Fragmentation (FU-A) */

                if (context->stapPending)
                {
                    /* Finish the previous STAP-A packet */
                    err = ARSTREAM2_RTPH264_Sender_FinishStapAPacket(context, packetFifo, packetFifoQueue, 0); // do not set the marker bit
                    if (err != 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Sender_FinishStapAPacket() failed (%d)", err);
                    }
                }

                err = ARSTREAM2_RTPH264_Sender_FuAPackets(context, &nalu, fragmentCount, packetFifo, packetFifoQueue, curTime);
                if (err != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Sender_FuAPackets() failed (%d)", err);
                }
            }
            else
            {
                unsigned int newStapSize = ((!context->stapPending) ? sizeof(ARSTREAM2_RTP_Header_t) + ((context->useRtpHeaderExtensions) ? nalu.metadataSize : 0) + 1 : 0) + 2 + nalu.naluSize;
                if ((context->stapPayloadSize + context->stapHeaderExtensionSize + newStapSize >= context->maxPacketSize)
                        || (context->stapPayloadSize + context->stapHeaderExtensionSize + newStapSize > context->targetPacketSize)
                        || (nalu.seqNumForcedDiscontinuity))
                {
                    if (context->stapPending)
                    {
                        /* Finish the previous STAP-A packet */
                        err = ARSTREAM2_RTPH264_Sender_FinishStapAPacket(context, packetFifo, packetFifoQueue, 0); // do not set the marker bit
                        if (err != 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Sender_FinishStapAPacket() failed (%d)", err);
                        }
                    }
                }

                if ((context->stapPayloadSize + context->stapHeaderExtensionSize + newStapSize >= context->maxPacketSize)
                        || (context->stapPayloadSize + context->stapHeaderExtensionSize + newStapSize > context->targetPacketSize))
                {
                    /* Single NAL unit */
                    err = ARSTREAM2_RTPH264_Sender_SingleNaluPacket(context, &nalu, packetFifo, packetFifoQueue, curTime);
                    if (err != 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Sender_SingleNaluPacket() failed (%d)", err);
                    }
                }
                else
                {
                    /* Aggregation (STAP-A) */
                    if (!context->stapPending)
                    {
                        /* Start a new STAP-A packet */
                        err = ARSTREAM2_RTPH264_Sender_BeginStapAPacket(context, &nalu, packetFifo, curTime);
                        if (err != 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Sender_BeginStapAPacket() failed (%d)", err);
                        }
                    }
                    if (context->stapPending)
                    {
                        /* Append to the current STAP-A packet */
                        err = ARSTREAM2_RTPH264_Sender_AppendToStapAPacket(context, &nalu);
                        if (err != 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Sender_AppendToStapAPacket() failed (%d)", err);
                        }

                        if (nalu.isLastInAu)
                        {
                            /* Finish the STAP-A packet */
                            err = ARSTREAM2_RTPH264_Sender_FinishStapAPacket(context, packetFifo, packetFifoQueue, 1); // set the marker bit
                            if (err != 0)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Sender_FinishStapAPacket() failed (%d)", err);
                            }
                        }
                    }
                }
            }

            /* call the naluCallback */
            if (context->naluCallback != NULL)
            {
                ((ARSTREAM2_StreamSender_NaluCallback_t)context->naluCallback)(ARSTREAM2_STREAM_SENDER_STATUS_SENT, nalu.naluUserPtr, context->naluCallbackUserPtr);
            }
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTPH264_TAG, "Time %"PRIu64": dropped NALU (%.1fms late) (seqNum = %d)",
                        nalu.ntpTimestamp, (float)(curTime - nalu.timeoutTimestamp) / 1000., context->seqNum - 1);

            ret = ARSTREAM2_RTPH264_Sender_NaluDrop(context, &nalu, curTime);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Sender_NaluDrop() failed (%d)", ret);
            }
        }

        /* last NALU in the Access Unit: call the auCallback */
        if ((context->auCallback != NULL) && (nalu.isLastInAu))
        {
            if (nalu.ntpTimestamp != context->lastAuCallbackTimestamp)
            {
                context->lastAuCallbackTimestamp = nalu.ntpTimestamp;

                /* call the auCallback */
                ((ARSTREAM2_StreamSender_AuCallback_t)context->auCallback)(ARSTREAM2_STREAM_SENDER_STATUS_SENT, nalu.auUserPtr, context->auCallbackUserPtr);
            }
        }

        context->previousTimestamp = nalu.ntpTimestamp;
        context->previousAuUserPtr = nalu.auUserPtr;
    }

    /*ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTPH264_TAG, "Processed %d NALUs (packet FIFO count: %d)",
                naluCount, packetFifoQueue->count);*/ //TODO: debug

    if (newPacketsCount) *newPacketsCount = packetFifoQueue->count - initialPacketCount;

    return ret;
}


int ARSTREAM2_RTPH264_Sender_FifoFlush(ARSTREAM2_RTP_SenderContext_t *context,
                                       ARSTREAM2_H264_NaluFifo_t *naluFifo,
                                       uint64_t curTime)
{
    ARSTREAM2_H264_NalUnit_t nalu;
    int ret = 0, fifoRes, naluCount = 0;

    while ((fifoRes = ARSTREAM2_RTPH264_FifoDequeueNalu(naluFifo, &nalu)) == 0) //TODO replace with FifoDequeue+FifoPushFreeItem
    {
        naluCount++;

        ret = ARSTREAM2_RTPH264_Sender_NaluDrop(context, &nalu, curTime);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Sender_NaluDrop() failed (%d)", ret);
        }

        /* last NALU in the Access Unit: call the auCallback */
        if ((context->auCallback != NULL) && (nalu.isLastInAu))
        {
            if (nalu.ntpTimestamp != context->lastAuCallbackTimestamp)
            {
                context->lastAuCallbackTimestamp = nalu.ntpTimestamp;

                /* call the auCallback */
                ((ARSTREAM2_StreamSender_AuCallback_t)context->auCallback)(ARSTREAM2_STREAM_SENDER_STATUS_SENT, nalu.auUserPtr, context->auCallbackUserPtr);
            }
        }
    }

    ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTPH264_TAG, "Flushed %d NALUs from FIFO", naluCount); //TODO: debug

    return ret;
}


int ARSTREAM2_RTPH264_Sender_NaluDrop(ARSTREAM2_RTP_SenderContext_t *context,
                                      ARSTREAM2_H264_NalUnit_t *nalu,
                                      uint64_t curTime)
{
    if ((!context) || (!nalu))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Invalid pointer");
        return -1;
    }
    if (!curTime)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Invalid current time");
        return -1;
    }

    /* increment the sequence number to let the receiver know that we dropped something */
    context->seqNum += nalu->seqNumForcedDiscontinuity + 1;
    context->packetCount += nalu->seqNumForcedDiscontinuity + 1;
    context->byteCount += nalu->naluSize;

    context->droppedPacketCount++;
    context->droppedByteIntegral += nalu->naluSize;
    context->droppedByteIntegralSq += (nalu->naluSize * nalu->naluSize);
    context->inputToDroppedTimeIntegral += (curTime - nalu->inputTimestamp);
    context->inputToDroppedTimeIntegralSq += ((curTime - nalu->inputTimestamp) * (curTime - nalu->inputTimestamp));

    /* call the monitoringCallback */
    if (context->monitoringCallback != NULL)
    {
        uint32_t rtpTimestamp = (nalu->ntpTimestamp * context->rtpClockRate + (uint64_t)context->rtpTimestampOffset + 500000) / 1000000;

        context->monitoringCallback(nalu->inputTimestamp, curTime, nalu->ntpTimestamp, rtpTimestamp, context->seqNum - 1,
                                    nalu->isLastInAu, nalu->importance, nalu->priority,
                                    0, nalu->naluSize, context->monitoringCallbackUserPtr);
    }

    /* call the naluCallback */
    if (context->naluCallback != NULL)
    {
        ((ARSTREAM2_StreamSender_NaluCallback_t)context->naluCallback)(ARSTREAM2_STREAM_SENDER_STATUS_CANCELLED, nalu->naluUserPtr, context->naluCallbackUserPtr);
    }

    return 0;
}


static int ARSTREAM2_RTPH264_Receiver_SingleNaluPacket(ARSTREAM2_RTPH264_ReceiverContext_t *context,
                                                       ARSTREAM2_RTP_Packet_t *packet,
                                                       uint32_t missingPacketsBefore)
{
    int ret = 0, err;

    if (!context->auItem)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Invalid pointer");
        return -1;
    }

    /* metadata as RTP header extension */
    if ((packet->headerExtension) && (packet->headerExtensionSize > 0)
            && (packet->headerExtensionSize <= context->auItem->au.buffer->metadataBufferSize))
    {
        memcpy(context->auItem->au.buffer->metadataBuffer, packet->headerExtension, packet->headerExtensionSize);
        context->auItem->au.metadataSize = packet->headerExtensionSize;
    }

    ARSTREAM2_H264_NaluFifoItem_t *item = ARSTREAM2_H264_AuNaluFifoPopFreeItem(&context->auItem->au);
    if (item)
    {
        ARSTREAM2_H264_NaluReset(&item->nalu);
        err = ARSTREAM2_H264_AuCheckSizeRealloc(&context->auItem->au, context->startCodeLength + packet->payloadSize);
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Access unit buffer is too small");
            err = ARSTREAM2_H264_AuNaluFifoPushFreeItem(&context->auItem->au, item);
            if (err < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to push free FIFO item");
            }
            return -1;
        }

        /* NALU data */
        item->nalu.nalu = context->auItem->au.buffer->auBuffer + context->auItem->au.auSize;
        item->nalu.naluSize = 0;
        if (context->startCodeLength > 0)
        {
            memcpy(context->auItem->au.buffer->auBuffer + context->auItem->au.auSize, &context->startCode, context->startCodeLength);
            item->nalu.naluSize += context->startCodeLength;
            context->auItem->au.auSize += context->startCodeLength;
        }
        memcpy(context->auItem->au.buffer->auBuffer + context->auItem->au.auSize, packet->payload, packet->payloadSize);
        item->nalu.naluSize += packet->payloadSize;
        context->auItem->au.auSize += packet->payloadSize;

        item->nalu.inputTimestamp = packet->inputTimestamp;
        item->nalu.timeoutTimestamp = packet->timeoutTimestamp;
        item->nalu.ntpTimestamp = packet->ntpTimestamp;
        item->nalu.ntpTimestampRaw = packet->ntpTimestampRaw;
        item->nalu.ntpTimestampLocal = packet->ntpTimestampLocal;
        item->nalu.extRtpTimestamp = packet->extRtpTimestamp;
        item->nalu.rtpTimestamp = packet->rtpTimestamp;
        item->nalu.isLastInAu = packet->markerBit;
        item->nalu.missingPacketsBefore = missingPacketsBefore;

        err = ARSTREAM2_H264_AuEnqueueNalu(&context->auItem->au, item);
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to enqueue NALU item in AU");
            context->auItem->au.auSize -= item->nalu.naluSize;
            err = ARSTREAM2_H264_AuNaluFifoPushFreeItem(&context->auItem->au, item);
            if (err != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to push free FIFO item");
            }
        }
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "AU-NALU FIFO is full");
        ret = -1;
    }

    return ret;
}


static int ARSTREAM2_RTPH264_Receiver_StapAPacket(ARSTREAM2_RTPH264_ReceiverContext_t *context,
                                                  ARSTREAM2_RTP_Packet_t *packet,
                                                  uint32_t missingPacketsBefore)
{
    int ret = 0, err, first;
    uint8_t *packetBuf = packet->payload + 1;
    unsigned int sizeLeft = packet->payloadSize - 1, naluSize;

    if (!context->auItem)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Invalid pointer");
        return -1;
    }

    /* metadata as RTP header extension */
    if ((packet->headerExtension) && (packet->headerExtensionSize > 0)
            && (packet->headerExtensionSize <= context->auItem->au.buffer->metadataBufferSize))
    {
        memcpy(context->auItem->au.buffer->metadataBuffer, packet->headerExtension, packet->headerExtensionSize);
        context->auItem->au.metadataSize = packet->headerExtensionSize;
    }

    naluSize = (((uint16_t)(*packetBuf) << 8) & 0xFF00) | (((uint16_t)(*(packetBuf + 1))) & 0x00FF);
    packetBuf += 2;
    sizeLeft -= 2;
    first = 1;

    while ((naluSize > 0) && (sizeLeft >= naluSize))
    {
        ARSTREAM2_H264_NaluFifoItem_t *item = ARSTREAM2_H264_AuNaluFifoPopFreeItem(&context->auItem->au);
        if (item)
        {
            ARSTREAM2_H264_NaluReset(&item->nalu);
            err = ARSTREAM2_H264_AuCheckSizeRealloc(&context->auItem->au, context->startCodeLength + naluSize);
            if (err != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Access unit buffer is too small");
                err = ARSTREAM2_H264_AuNaluFifoPushFreeItem(&context->auItem->au, item);
                if (err < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to push free FIFO item");
                }
                return -1;
            }

            /* NALU data */
            item->nalu.nalu = context->auItem->au.buffer->auBuffer + context->auItem->au.auSize;
            item->nalu.naluSize = 0;
            if (context->startCodeLength > 0)
            {
                memcpy(context->auItem->au.buffer->auBuffer + context->auItem->au.auSize, &context->startCode, context->startCodeLength);
                item->nalu.naluSize += context->startCodeLength;
                context->auItem->au.auSize += context->startCodeLength;
            }
            memcpy(context->auItem->au.buffer->auBuffer + context->auItem->au.auSize, packetBuf, naluSize);
            item->nalu.naluSize += naluSize;
            context->auItem->au.auSize += naluSize;

            item->nalu.inputTimestamp = packet->inputTimestamp;
            item->nalu.timeoutTimestamp = packet->timeoutTimestamp;
            item->nalu.ntpTimestamp = packet->ntpTimestamp;
            item->nalu.ntpTimestampRaw = packet->ntpTimestampRaw;
            item->nalu.ntpTimestampLocal = packet->ntpTimestampLocal;
            item->nalu.extRtpTimestamp = packet->extRtpTimestamp;
            item->nalu.rtpTimestamp = packet->rtpTimestamp;
            item->nalu.isLastInAu = (sizeLeft - naluSize >= 2) ? 0 : packet->markerBit;
            item->nalu.missingPacketsBefore = (first) ? missingPacketsBefore : 0;

            err = ARSTREAM2_H264_AuEnqueueNalu(&context->auItem->au, item);
            if (err != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to enqueue NALU item in AU");
                context->auItem->au.auSize -= item->nalu.naluSize;
                err = ARSTREAM2_H264_AuNaluFifoPushFreeItem(&context->auItem->au, item);
                if (err != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to push free FIFO item");
                }
                return -1;
            }
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "AU-NALU FIFO is full");
            ret = -1;
        }

        packetBuf += naluSize;
        sizeLeft -= naluSize;

        /* next NALU */
        if (sizeLeft >= 2)
        {
            naluSize = (((uint16_t)(*packetBuf) << 8) & 0xFF00) | (((uint16_t)(*(packetBuf + 1))) & 0x00FF);
            packetBuf += 2;
            sizeLeft -= 2;
        }
        else
        {
            naluSize = 0;
        }
        first = 0;
    }

    return ret;
}


static int ARSTREAM2_RTPH264_Receiver_BeginFuAPackets(ARSTREAM2_RTPH264_ReceiverContext_t *context,
                                                       ARSTREAM2_RTP_Packet_t *packet,
                                                       uint32_t missingPacketsBefore)
{
    int ret = 0, err;

    if (!context->auItem)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Invalid pointer");
        return -1;
    }

    /* metadata as RTP header extension */
    if ((packet->headerExtension) && (packet->headerExtensionSize > 0)
            && (packet->headerExtensionSize <= context->auItem->au.buffer->metadataBufferSize))
    {
        memcpy(context->auItem->au.buffer->metadataBuffer, packet->headerExtension, packet->headerExtensionSize);
        context->auItem->au.metadataSize = packet->headerExtensionSize;
    }

    context->fuNaluItem = ARSTREAM2_H264_AuNaluFifoPopFreeItem(&context->auItem->au);
    if (context->fuNaluItem)
    {
        ARSTREAM2_H264_NaluReset(&context->fuNaluItem->nalu);
        if (context->startCodeLength > 0)
        {
            err = ARSTREAM2_H264_AuCheckSizeRealloc(&context->auItem->au, context->startCodeLength);
            if (err != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Access unit buffer is too small");
                err = ARSTREAM2_H264_AuNaluFifoPushFreeItem(&context->auItem->au, context->fuNaluItem);
                if (err < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to push free FIFO item");
                }
                context->fuNaluItem = NULL;
                return -1;
            }
        }

        context->fuNaluItem->nalu.nalu = context->auItem->au.buffer->auBuffer + context->auItem->au.auSize;
        context->fuNaluItem->nalu.naluSize = 0;

        context->fuNaluItem->nalu.inputTimestamp = packet->inputTimestamp;
        context->fuNaluItem->nalu.timeoutTimestamp = packet->timeoutTimestamp;
        context->fuNaluItem->nalu.ntpTimestamp = packet->ntpTimestamp;
        context->fuNaluItem->nalu.ntpTimestampRaw = packet->ntpTimestampRaw;
        context->fuNaluItem->nalu.ntpTimestampLocal = packet->ntpTimestampLocal;
        context->fuNaluItem->nalu.extRtpTimestamp = packet->extRtpTimestamp;
        context->fuNaluItem->nalu.rtpTimestamp = packet->rtpTimestamp;
        context->fuNaluItem->nalu.missingPacketsBefore = missingPacketsBefore;

        if (context->startCodeLength > 0)
        {
            memcpy(context->auItem->au.buffer->auBuffer + context->auItem->au.auSize, &context->startCode, context->startCodeLength);
            context->fuNaluItem->nalu.naluSize += context->startCodeLength;
            context->auItem->au.auSize += context->startCodeLength;
        }

        context->fuPending = 1;
        context->fuPacketCount = 0;
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "AU-NALU FIFO is full");
        ret = -1;
    }

    return ret;
}


static int ARSTREAM2_RTPH264_Receiver_AppendPacketToFuA(ARSTREAM2_RTPH264_ReceiverContext_t *context,
                                                        ARSTREAM2_RTP_Packet_t *packet,
                                                        int isFirst, uint8_t headerByte)
{
    int ret = 0, err;
    uint8_t *packetBuf = packet->payload + ((isFirst) ? 1 : 2);
    unsigned int packetSize = packet->payloadSize - ((isFirst) ? 1 : 2);

    if ((!context->auItem) || (!context->fuNaluItem))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Invalid pointer");
        return -1;
    }

    uint8_t *oldPtr = context->auItem->au.buffer->auBuffer;
    err = ARSTREAM2_H264_AuCheckSizeRealloc(&context->auItem->au, packetSize);
    if (err != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Access unit buffer is too small");
        return -1;
    }
    uint8_t *newPtr = context->auItem->au.buffer->auBuffer;

    if (newPtr != oldPtr)
    {
        /* translate the current FU-A NALU pointer to the new AU buffer */
        unsigned int offset = (unsigned int)(context->fuNaluItem->nalu.nalu - oldPtr);
        if (offset < context->auItem->au.buffer->auBufferSize)
        {
            context->fuNaluItem->nalu.nalu = newPtr + offset;
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Invalid NALU offset in AU buffer (%d)", offset);
            context->fuNaluItem->nalu.nalu = NULL;
            return -1;
        }
    }

    /* NALU data */
    memcpy(context->auItem->au.buffer->auBuffer + context->auItem->au.auSize, packetBuf, packetSize);
    if (isFirst)
    {
        /* restore the NALU header byte */
        *(context->auItem->au.buffer->auBuffer + context->auItem->au.auSize) = headerByte;
    }
    context->fuNaluItem->nalu.naluSize += packetSize;
    context->auItem->au.auSize += packetSize;
    context->fuPacketCount++;

    return ret;
}


static int ARSTREAM2_RTPH264_Receiver_FinishFuAPackets(ARSTREAM2_RTPH264_ReceiverContext_t *context,
                                                       ARSTREAM2_RTP_Packet_t *packet)
{
    int ret = 0, err;

    if ((!context->auItem) || (!context->fuNaluItem))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Invalid pointer");
        return -1;
    }

    context->fuNaluItem->nalu.isLastInAu = packet->markerBit;

    err = ARSTREAM2_H264_AuEnqueueNalu(&context->auItem->au, context->fuNaluItem);
    if (err != 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to enqueue NALU item in AU");
        return -1;
    }

    context->fuPending = 0;
    context->fuPacketCount = 0;
    context->fuNaluItem = NULL;

    return ret;
}


static int ARSTREAM2_RTPH264_Receiver_DropFuAPackets(ARSTREAM2_RTPH264_ReceiverContext_t *context)
{
    int err;

    if (!context->fuNaluItem)
    {
        return 0;
    }

    if ((context->auItem) && (context->fuNaluItem->nalu.naluSize <= context->auItem->au.auSize))
    {
        context->auItem->au.auSize -= context->fuNaluItem->nalu.naluSize;
    }

    context->fuPending = 0;
    context->fuPacketCount = 0;

    err = ARSTREAM2_H264_AuNaluFifoPushFreeItem(&context->auItem->au, context->fuNaluItem);
    if (err < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to push free FIFO item");
        return -1;
    }

    context->fuNaluItem = NULL;

    return 0;
}


int ARSTREAM2_RTPH264_Receiver_PacketFifoToAuFifo(ARSTREAM2_RTPH264_ReceiverContext_t *context,
                                                  ARSTREAM2_RTP_PacketFifo_t *packetFifo,
                                                  ARSTREAM2_RTP_PacketFifoQueue_t *packetFifoQueue,
                                                  ARSTREAM2_H264_AuFifo_t *auFifo,
                                                  uint64_t curTime, ARSTREAM2_RTCP_ReceiverContext_t *rtcpContext)
{
    ARSTREAM2_RTP_PacketFifoItem_t *packetItem;
    int ret = 0, packetCount = 0, peekCount = 0, err;

    while ((packetItem = ARSTREAM2_RTP_PacketFifoPeekItem(packetFifoQueue)) != NULL)
    {
        peekCount++;
        if ((context->previousDepayloadExtSeqNum == -1)
                || ((int64_t)packetItem->packet.extSeqNum == context->previousDepayloadExtSeqNum + 1)
                || (curTime >= packetItem->packet.timeoutTimestamp))
        {
            packetItem = ARSTREAM2_RTP_PacketFifoDequeueItem(packetFifoQueue);
            if (packetItem)
            {
                if ((int64_t)packetItem->packet.extSeqNum > context->previousDepayloadExtSeqNum)
                {
                    ARSTREAM2_RTP_Packet_t *packet = &packetItem->packet;
                    uint32_t missingPacketsBefore;
                    if (context->previousDepayloadExtSeqNum == -1)
                    {
                        missingPacketsBefore = 0;
                    }
                    else
                    {
                        missingPacketsBefore = (uint32_t)((int64_t)packet->extSeqNum - context->previousDepayloadExtSeqNum - 1 + context->missingBeforePending);
                        context->missingBeforePending = 0;
                        rtcpContext->packetsLost += missingPacketsBefore;
                        rtcpContext->packetsReceived -= missingPacketsBefore;
                    }
                    err = ARSTREAM2_RTCP_LossReportSet(&rtcpContext->lossReportCtx, packet->extSeqNum);
                    if (err != 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTCP_LossReportSet() failed (%d)", err);
                    }

                    /* AU change detection */
                    if ((ret == 0) && (context->auItem != NULL) && (context->previousDepayloadExtRtpTimestamp != 0)
                            && (packet->extRtpTimestamp != context->previousDepayloadExtRtpTimestamp))
                    {
                        /* drop the previous incomplete FU-A */
                        //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTPH264_TAG, "Incomplete FU-A packet before extSeqNum %d", packet->extSeqNum);
                        context->missingBeforePending += context->fuPacketCount;
                        err = ARSTREAM2_RTPH264_Receiver_DropFuAPackets(context);
                        if (err != 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_DropFuAPacket() failed (%d)", err);
                        }

                        /* change of RTP timestamp without the marker bit set on previous packet => output the access unit */
                        if (context->auCallback)
                        {
                            err = context->auCallback(context->auItem, context->auCallbackUserPtr);
                            if (err != 0)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to output the access unit");
                                ret = -1;
                            }
                            context->auItem = NULL;
                        }
                        else
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Invalid access unit callback function");
                            ret = -1;

                            /* free the access unit */
                            ret = ARSTREAM2_H264_AuFifoUnrefBuffer(auFifo, context->auItem->au.buffer);
                            if (ret != 0)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to unref buffer (%d)", ret);
                            }
                            ret = ARSTREAM2_H264_AuFifoPushFreeItem(auFifo, context->auItem);
                            if (ret != 0)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to push free item in the AU FIFO (%d)", ret);
                            }
                            context->auItem = NULL;
                        }
                    }

                    if ((ret == 0) && (context->auItem == NULL))
                    {
                        ARSTREAM2_H264_AuFifoBuffer_t *buffer = ARSTREAM2_H264_AuFifoGetBuffer(auFifo);
                        context->auItem = ARSTREAM2_H264_AuFifoPopFreeItem(auFifo);
                        if ((!buffer) || (!context->auItem))
                        {
                            if (buffer) ARSTREAM2_H264_AuFifoUnrefBuffer(auFifo, buffer);
                            if (context->auItem) ARSTREAM2_H264_AuFifoPushFreeItem(auFifo, context->auItem);
                            err = ARSTREAM2_H264_AuFifoFlush(auFifo);
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Access unit FIFO is full => flush to recover (%d AU flushed)", err);
                            ret = -1;
                        }
                        else
                        {
                            ARSTREAM2_H264_AuReset(&context->auItem->au);
                            context->auItem->au.buffer = buffer;
                        }
                    }

                    if ((ret == 0) && (context->auItem != NULL))
                    {
                        if ((packet->payload) && (packet->payloadSize >= 1))
                        {
                            uint8_t headByte = *(packet->payload);

                            if ((headByte & 0x1F) == ARSTREAM2_RTPH264_NALU_TYPE_FUA)
                            {
                                /* Fragmentation (FU-A) */
                                if (packet->payloadSize >= 2)
                                {
                                    uint8_t fuIndicator, fuHeader, startBit, endBit;
                                    fuIndicator = headByte;
                                    fuHeader = *(packet->payload + 1);
                                    startBit = fuHeader & 0x80;
                                    endBit = fuHeader & 0x40;

                                    if ((context->fuPending) && (startBit))
                                    {
                                        /* drop the previous incomplete FU-A */
                                        //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTPH264_TAG, "Incomplete FU-A packet before extSeqNum %d", packet->extSeqNum);
                                        context->missingBeforePending += context->fuPacketCount;
                                        err = ARSTREAM2_RTPH264_Receiver_DropFuAPackets(context);
                                        if (err != 0)
                                        {
                                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_DropFuAPacket() failed (%d)", err);
                                        }
                                    }

                                    if (startBit)
                                    {
                                        err = ARSTREAM2_RTPH264_Receiver_BeginFuAPackets(context, packet, missingPacketsBefore + context->missingBeforePending);
                                        if (err != 0)
                                        {
                                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_BeginFuAPackets() failed (%d)", err);
                                            context->missingBeforePending++;
                                        }
                                        else
                                        {
                                            context->missingBeforePending = 0;
                                        }
                                    }
                                    else if (missingPacketsBefore + context->missingBeforePending)
                                    {
                                        /* drop the FU-A if there is a seqNum discontinuity */
                                        //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTPH264_TAG, "Incomplete FU-A packet at extSeqNum %d", packet->extSeqNum);
                                        context->missingBeforePending += context->fuPacketCount + 1;
                                        err = ARSTREAM2_RTPH264_Receiver_DropFuAPackets(context);
                                        if (err != 0)
                                        {
                                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_DropFuAPacket() failed (%d)", err);
                                        }
                                    }

                                    if (context->fuPending)
                                    {
                                        err = ARSTREAM2_RTPH264_Receiver_AppendPacketToFuA(context, packet, startBit, (fuIndicator & 0xE0) | (fuHeader & 0x1F));
                                        if (err != 0)
                                        {
                                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_AppendPacketToFuA() failed (%d)", err);
                                            context->missingBeforePending += context->fuPacketCount + 1;
                                            err = ARSTREAM2_RTPH264_Receiver_DropFuAPackets(context);
                                            if (err != 0)
                                            {
                                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_DropFuAPacket() failed (%d)", err);
                                            }
                                        }

                                        if (endBit)
                                        {
                                            err = ARSTREAM2_RTPH264_Receiver_FinishFuAPackets(context, packet);
                                            if (err != 0)
                                            {
                                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_FinishFuAPackets() failed (%d)", err);
                                                context->missingBeforePending += context->fuPacketCount;
                                                err = ARSTREAM2_RTPH264_Receiver_DropFuAPackets(context);
                                                if (err != 0)
                                                {
                                                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_DropFuAPacket() failed (%d)", err);
                                                }
                                            }
                                        }
                                    }
                                }
                                else
                                {
                                    ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTPH264_TAG, "Invalid payload size (%d) for FU-A packet at extSeqNum %d", packet->payloadSize, packet->extSeqNum);
                                }
                            }
                            else if ((headByte & 0x1F) == ARSTREAM2_RTPH264_NALU_TYPE_STAPA)
                            {
                                /* Aggregation (STAP-A) */

                                if (context->fuPending)
                                {
                                    /* drop the previous incomplete FU-A */
                                    //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTPH264_TAG, "Incomplete FU-A packet before extSeqNum %d", packet->extSeqNum);
                                    context->missingBeforePending += context->fuPacketCount;
                                    err = ARSTREAM2_RTPH264_Receiver_DropFuAPackets(context);
                                    if (err != 0)
                                    {
                                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_DropFuAPacket() failed (%d)", err);
                                    }
                                }

                                if (packet->payloadSize >= 3)
                                {
                                    err = ARSTREAM2_RTPH264_Receiver_StapAPacket(context, packet, missingPacketsBefore + context->missingBeforePending);
                                    if (err != 0)
                                    {
                                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_StapAPacket() failed (%d)", err);
                                        context->missingBeforePending++;
                                    }
                                    else
                                    {
                                        context->missingBeforePending = 0;
                                    }
                                }
                                else
                                {
                                    ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTPH264_TAG, "Invalid payload size (%d) for STAP-A packet at extSeqNum %d", packet->payloadSize, packet->extSeqNum);
                                }
                            }
                            else
                            {
                                /* Single NAL unit */

                                if (context->fuPending)
                                {
                                    /* drop the previous incomplete FU-A */
                                    //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTPH264_TAG, "Incomplete FU-A packet before extSeqNum %d", packet->extSeqNum);
                                    context->missingBeforePending += context->fuPacketCount;
                                    err = ARSTREAM2_RTPH264_Receiver_DropFuAPackets(context);
                                    if (err != 0)
                                    {
                                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_DropFuAPacket() failed (%d)", err);
                                    }
                                }

                                err = ARSTREAM2_RTPH264_Receiver_SingleNaluPacket(context, packet, missingPacketsBefore + context->missingBeforePending);
                                if (err != 0)
                                {
                                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "ARSTREAM2_RTPH264_Receiver_SingleNaluPacket() failed (%d)", err);
                                    context->missingBeforePending++;
                                }
                                else
                                {
                                    context->missingBeforePending = 0;
                                }
                            }
                        }
                        else
                        {
                            ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTPH264_TAG, "Invalid payload size (%d) for packet at extSeqNum %d", packet->payloadSize, packet->extSeqNum);
                        }
                    }

                    if ((ret == 0) && (context->auItem != NULL) && (packet->markerBit))
                    {
                        /* the marker bit is set => output the access unit */
                        if (context->auCallback)
                        {
                            err = context->auCallback(context->auItem, context->auCallbackUserPtr);
                            if (err != 0)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to output the access unit");
                                ret = -1;
                            }
                            context->auItem = NULL;
                        }
                        else
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Invalid access unit callback function");
                            ret = -1;

                            /* free the access unit */
                            ret = ARSTREAM2_H264_AuFifoUnrefBuffer(auFifo, context->auItem->au.buffer);
                            if (ret != 0)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to unref buffer (%d)", ret);
                            }
                            ret = ARSTREAM2_H264_AuFifoPushFreeItem(auFifo, context->auItem);
                            if (ret != 0)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to push free item in the AU FIFO (%d)", ret);
                            }
                            context->auItem = NULL;
                        }
                    }

                    rtcpContext->packetsReceived++;
                    context->previousDepayloadExtSeqNum = packet->extSeqNum;
                    context->previousDepayloadExtRtpTimestamp = packet->extRtpTimestamp;
                    packetCount++;
                }

                err = ARSTREAM2_RTP_PacketFifoUnrefBuffer(packetFifo, packetItem->packet.buffer);
                if (err < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to unref packet buffer (%d)", err);
                }
                err = ARSTREAM2_RTP_PacketFifoPushFreeItem(packetFifo, packetItem);
                if (err < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to push free item in the packet FIFO (%d)", err);
                }
                if (ret < 0)
                {
                    break;
                }
            }
            else
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTPH264_TAG, "Failed to dequeue packet from FIFO");
                ret = -1;
                break;
            }
        }
        else
        {
            break;
        }
    }

    //ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_RTPH264_TAG, "Processed %d packets", packetCount); //TODO: debug

    return ret;
}
