/**
 * @file arstream2_rtp.c
 * @brief Parrot Streaming Library - RTP implementation
 * @date 04/25/2016
 * @author aurelien.barre@parrot.com
 */

#include "arstream2_rtp.h"
#include "arstream2_rtcp.h"

#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <libARSAL/ARSAL_Print.h>


/**
 * Tag for ARSAL_PRINT
 */
#define ARSTREAM2_RTP_TAG "ARSTREAM2_Rtp"


void ARSTREAM2_RTP_PacketReset(ARSTREAM2_RTP_Packet_t *packet)
{
    if (!packet)
    {
        return;
    }

    packet->inputTimestamp = 0;
    packet->timeoutTimestamp = 0;
    packet->ntpTimestamp = 0;
    packet->ntpTimestampRaw = 0;
    packet->ntpTimestampLocal = 0;
    packet->extRtpTimestamp = 0;
    packet->rtpTimestamp = 0;
    packet->seqNum = 0;
    packet->extSeqNum = 0;
    packet->markerBit = 0;
    packet->header = NULL;
    packet->headerExtension = NULL;
    packet->headerExtensionSize = 0;
    packet->payload = NULL;
    packet->payloadSize = 0;
    packet->importance = 0;
    packet->priority = 0;
    packet->msgIovLength = 0;
}


void ARSTREAM2_RTP_PacketCopy(ARSTREAM2_RTP_Packet_t *dst, const ARSTREAM2_RTP_Packet_t *src)
{
    if ((!src) || (!dst))
    {
        return;
    }

    dst->buffer = src->buffer;
    dst->inputTimestamp = src->inputTimestamp;
    dst->timeoutTimestamp = src->timeoutTimestamp;
    dst->ntpTimestamp = src->ntpTimestamp;
    dst->ntpTimestampRaw = src->ntpTimestampRaw;
    dst->ntpTimestampLocal = src->ntpTimestampLocal;
    dst->extRtpTimestamp = src->extRtpTimestamp;
    dst->rtpTimestamp = src->rtpTimestamp;
    dst->seqNum = src->seqNum;
    dst->extSeqNum = src->extSeqNum;
    dst->markerBit = src->markerBit;
    dst->header = src->header;
    dst->headerExtension = src->headerExtension;
    dst->headerExtensionSize = src->headerExtensionSize;
    dst->payload = src->payload;
    dst->payloadSize = src->payloadSize;
    dst->importance = src->importance;
    dst->priority = src->priority;
    dst->msgIovLength = src->msgIovLength;
}


int ARSTREAM2_RTP_PacketFifoInit(ARSTREAM2_RTP_PacketFifo_t *fifo, int itemMaxCount, int bufferMaxCount, int packetBufferSize)
{
    int i;
    ARSTREAM2_RTP_PacketFifoItem_t* curItem = NULL;
    ARSTREAM2_RTP_PacketFifoBuffer_t* curBuffer = NULL;

    if (!fifo)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }
    if (itemMaxCount <= 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid item max count (%d)", itemMaxCount);
        return -1;
    }
    if (bufferMaxCount <= 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid buffer max count (%d)", bufferMaxCount);
        return -1;
    }

    memset(fifo, 0, sizeof(ARSTREAM2_RTP_PacketFifo_t));

    fifo->itemPoolSize = itemMaxCount;
    fifo->itemPool = malloc(itemMaxCount * sizeof(ARSTREAM2_RTP_PacketFifoItem_t));
    if (!fifo->itemPool)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "FIFO allocation failed (size %zu)", itemMaxCount * sizeof(ARSTREAM2_RTP_PacketFifoItem_t));
        ARSTREAM2_RTP_PacketFifoFree(fifo);
        return -1;
    }
    memset(fifo->itemPool, 0, itemMaxCount * sizeof(ARSTREAM2_RTP_PacketFifoItem_t));

    for (i = 0; i < itemMaxCount; i++)
    {
        curItem = &fifo->itemPool[i];
        if (fifo->itemFree)
        {
            fifo->itemFree->prev = curItem;
        }
        curItem->next = fifo->itemFree;
        curItem->prev = NULL;
        fifo->itemFree = curItem;
    }

    fifo->bufferPoolSize = bufferMaxCount;
    fifo->bufferPool = malloc(bufferMaxCount * sizeof(ARSTREAM2_RTP_PacketFifoBuffer_t));
    if (!fifo->bufferPool)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "FIFO allocation failed (size %zu)", bufferMaxCount * sizeof(ARSTREAM2_RTP_PacketFifoBuffer_t));
        ARSTREAM2_RTP_PacketFifoFree(fifo);
        return -1;
    }
    memset(fifo->bufferPool, 0, bufferMaxCount * sizeof(ARSTREAM2_RTP_PacketFifoBuffer_t));

    for (i = 0; i < bufferMaxCount; i++)
    {
        curBuffer = &fifo->bufferPool[i];
        if (fifo->bufferFree)
        {
            fifo->bufferFree->prev = curBuffer;
        }
        curBuffer->next = fifo->bufferFree;
        curBuffer->prev = NULL;
        fifo->bufferFree = curBuffer;
    }

    if (packetBufferSize > 0)
    {
        for (i = 0; i < bufferMaxCount; i++)
        {
            fifo->bufferPool[i].buffer = malloc(packetBufferSize);
            if (!fifo->bufferPool[i].buffer)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "FIFO packet buffer allocation failed (size %d)", packetBufferSize);
                ARSTREAM2_RTP_PacketFifoFree(fifo);
                return -1;
            }
            fifo->bufferPool[i].bufferSize = packetBufferSize;
        }
    }

    for (i = 0; i < bufferMaxCount; i++)
    {
        fifo->bufferPool[i].header = malloc(sizeof(ARSTREAM2_RTP_Header_t));
        if (!fifo->bufferPool[i].header)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "FIFO packet buffer allocation failed (size %zu)", sizeof(ARSTREAM2_RTP_Header_t));
            ARSTREAM2_RTP_PacketFifoFree(fifo);
            return -1;
        }
        fifo->bufferPool[i].headerSize = sizeof(ARSTREAM2_RTP_Header_t);
    }

    return 0;
}


int ARSTREAM2_RTP_PacketFifoFree(ARSTREAM2_RTP_PacketFifo_t *fifo)
{
    int i;

    if (!fifo)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    free(fifo->itemPool);

    if (fifo->bufferPool)
    {
        for (i = 0; i < fifo->bufferPoolSize; i++)
        {
            free(fifo->bufferPool[i].buffer);
            fifo->bufferPool[i].buffer = NULL;
            free(fifo->bufferPool[i].header);
            fifo->bufferPool[i].header = NULL;
        }

        free(fifo->bufferPool);
    }

    memset(fifo, 0, sizeof(ARSTREAM2_RTP_PacketFifo_t));

    return 0;
}


int ARSTREAM2_RTP_PacketFifoAddQueue(ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoQueue_t *queue)
{
    if ((!fifo) || (!queue))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    queue->count = 0;
    queue->head = NULL;
    queue->tail = NULL;

    queue->prev = NULL;
    queue->next = fifo->queue;
    if (queue->next)
    {
        queue->next->prev = queue;
    }
    fifo->queue = queue;
    fifo->queueCount++;

    return 0;
}


int ARSTREAM2_RTP_PacketFifoRemoveQueue(ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoQueue_t *queue)
{
    if ((!fifo) || (!queue))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    if (queue->prev)
    {
        queue->prev->next = queue->next;
    }
    if (queue->next)
    {
        queue->next->prev = queue->prev;
    }
    if (fifo->queue == queue)
    {
        fifo->queue = queue->next;
    }
    fifo->queueCount--;
    queue->prev = NULL;
    queue->next = NULL;
    queue->count = 0;
    queue->head = NULL;
    queue->tail = NULL;

    return 0;
}


ARSTREAM2_RTP_PacketFifoBuffer_t* ARSTREAM2_RTP_PacketFifoGetBuffer(ARSTREAM2_RTP_PacketFifo_t *fifo)
{
    if (!fifo)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return NULL;
    }

    if (fifo->bufferFree)
    {
        ARSTREAM2_RTP_PacketFifoBuffer_t* cur = fifo->bufferFree;
        fifo->bufferFree = cur->next;
        if (cur->next) cur->next->prev = NULL;
        cur->prev = NULL;
        cur->next = NULL;
        cur->refCount = 1;
        return cur;
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "No free buffer in pool");
        return NULL;
    }
}


int ARSTREAM2_RTP_PacketFifoBufferAddRef(ARSTREAM2_RTP_PacketFifoBuffer_t *buffer)
{
    if (!buffer)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    buffer->refCount++;

    return 0;
}


int ARSTREAM2_RTP_PacketFifoUnrefBuffer(ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoBuffer_t *buffer)
{
    if ((!fifo) || (!buffer))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    if (buffer->refCount != 0)
    {
        buffer->refCount--;
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTP_TAG, "FIXME! Ref count is already null, this should not happen!");
    }

    if (buffer->refCount == 0)
    {
        if (fifo->bufferFree)
        {
            fifo->bufferFree->prev = buffer;
            buffer->next = fifo->bufferFree;
        }
        else
        {
            buffer->next = NULL;
        }
        fifo->bufferFree = buffer;
        buffer->prev = NULL;
    }

    return 0;
}


ARSTREAM2_RTP_PacketFifoItem_t* ARSTREAM2_RTP_PacketFifoPopFreeItem(ARSTREAM2_RTP_PacketFifo_t *fifo)
{
    if (!fifo)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return NULL;
    }

    if (fifo->itemFree)
    {
        ARSTREAM2_RTP_PacketFifoItem_t* cur = fifo->itemFree;
        fifo->itemFree = cur->next;
        if (cur->next) cur->next->prev = NULL;
        cur->prev = NULL;
        cur->next = NULL;
        return cur;
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Packet FIFO is full");
        return NULL;
    }
}


int ARSTREAM2_RTP_PacketFifoPushFreeItem(ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoItem_t *item)
{
    if ((!fifo) || (!item))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    if (fifo->itemFree)
    {
        fifo->itemFree->prev = item;
        item->next = fifo->itemFree;
    }
    else
    {
        item->next = NULL;
    }
    fifo->itemFree = item;
    item->prev = NULL;

    return 0;
}


int ARSTREAM2_RTP_PacketFifoEnqueueItem(ARSTREAM2_RTP_PacketFifoQueue_t *queue, ARSTREAM2_RTP_PacketFifoItem_t *item)
{
    if ((!queue) || (!item))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    item->next = NULL;
    if (queue->tail)
    {
        queue->tail->next = item;
        item->prev = queue->tail;
    }
    else
    {
        item->prev = NULL;
    }
    queue->tail = item;
    if (!queue->head)
    {
        queue->head = item;
    }
    queue->count++;

    return 0;
}


int ARSTREAM2_RTP_PacketFifoEnqueueItemOrderedByPriority(ARSTREAM2_RTP_PacketFifoQueue_t *queue, ARSTREAM2_RTP_PacketFifoItem_t *item)
{
    ARSTREAM2_RTP_PacketFifoItem_t* cur;

    if ((!queue) || (!item))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    for (cur = queue->tail; cur; cur = cur->prev)
    {
        if (cur->packet.rtpTimestamp == item->packet.rtpTimestamp)
        {
            if (cur->packet.priority <= item->packet.priority)
            {
                break;
            }
        }
        else if (cur->packet.rtpTimestamp < item->packet.rtpTimestamp)
        {
            break;
        }
    }

    if (cur)
    {
        /* insert after cur */
        item->next = cur->next;
        if (item->next)
        {
            item->next->prev = item;
        }
        else
        {
            queue->tail = item;
        }
        item->prev = cur;
        cur->next = item;
        queue->count++;
    }
    else
    {
        /* insert at head */
        item->next = queue->head;
        if (queue->head)
        {
            queue->head->prev = item;
        }
        item->prev = NULL;
        queue->head = item;
        if (!queue->tail)
        {
            queue->tail = item;
        }
        queue->count++;
    }

    return 0;
}


int ARSTREAM2_RTP_PacketFifoEnqueueItemOrderedBySeqNum(ARSTREAM2_RTP_PacketFifoQueue_t *queue, ARSTREAM2_RTP_PacketFifoItem_t *item)
{
    ARSTREAM2_RTP_PacketFifoItem_t* cur;
    int outOfOrder = 0, duplicate = 0;

    if ((!queue) || (!item))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    for (cur = queue->tail; cur; cur = cur->prev)
    {
        if (cur->packet.extSeqNum == item->packet.extSeqNum)
        {
            duplicate = 1;
            break;
        }
        else if (cur->packet.extSeqNum < item->packet.extSeqNum)
        {
            break;
        }
        else
        {
            outOfOrder = 1;
        }
    }

    if (duplicate)
    {
        return -3;
    }

    if (cur)
    {
        /* insert after cur */
        item->next = cur->next;
        if (item->next)
        {
            item->next->prev = item;
        }
        else
        {
            queue->tail = item;
        }
        item->prev = cur;
        cur->next = item;
        queue->count++;
    }
    else
    {
        /* insert at head */
        item->next = queue->head;
        if (queue->head)
        {
            queue->head->prev = item;
        }
        item->prev = NULL;
        queue->head = item;
        if (!queue->tail)
        {
            queue->tail = item;
        }
        queue->count++;
    }

    return (outOfOrder) ? 1 : 0;
}


ARSTREAM2_RTP_PacketFifoItem_t* ARSTREAM2_RTP_PacketFifoDequeueItem(ARSTREAM2_RTP_PacketFifoQueue_t *queue)
{
    ARSTREAM2_RTP_PacketFifoItem_t* cur;

    if (!queue)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return NULL;
    }

    if ((!queue->head) || (!queue->count))
    {
        //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTP_TAG, "Packet FIFO is empty");
        return NULL;
    }

    cur = queue->head;
    if (cur->next)
    {
        cur->next->prev = NULL;
        queue->head = cur->next;
        queue->count--;
    }
    else
    {
        queue->head = NULL;
        queue->count = 0;
        queue->tail = NULL;
    }
    cur->prev = NULL;
    cur->next = NULL;

    return cur;
}


ARSTREAM2_RTP_PacketFifoItem_t* ARSTREAM2_RTP_PacketFifoPeekItem(ARSTREAM2_RTP_PacketFifoQueue_t *queue)
{
    ARSTREAM2_RTP_PacketFifoItem_t* cur;

    if (!queue)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return NULL;
    }

    if ((!queue->head) || (!queue->count))
    {
        //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTP_TAG, "Packet FIFO is empty");
        return NULL;
    }

    cur = queue->head;

    return cur;
}


ARSTREAM2_RTP_PacketFifoItem_t* ARSTREAM2_RTP_PacketFifoDuplicateItem(ARSTREAM2_RTP_PacketFifo_t *fifo,
                                                                      ARSTREAM2_RTP_PacketFifoItem_t *item)
{
    ARSTREAM2_RTP_PacketFifoItem_t *copyItem;

    if ((!fifo) || (!item))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return NULL;
    }

    copyItem = ARSTREAM2_RTP_PacketFifoPopFreeItem(fifo);
    if (copyItem)
    {
        ARSTREAM2_RTP_PacketCopy(&copyItem->packet, &item->packet);
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Failed to pop free item from the AU FIFO");
    }

    return copyItem;
}


int ARSTREAM2_RTP_Sender_PacketFifoFillMsgVec(ARSTREAM2_RTP_PacketFifoQueue_t *queue, struct mmsghdr *msgVec, unsigned int msgVecCount, void *msgName, socklen_t msgNamelen)
{
    ARSTREAM2_RTP_PacketFifoItem_t* cur = NULL;
    unsigned int i;

    if ((!queue) || (!msgVec))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }
    if (msgVecCount == 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid msgVecCount");
        return -1;
    }

    if ((!queue->head) || (!queue->count))
    {
        //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTP_TAG, "Packet FIFO is empty");
        return -2;
    }

    for (cur = queue->head, i = 0; ((cur) && (i < msgVecCount)); cur = cur->next, i++)
    {
        msgVec[i].msg_hdr.msg_name = msgName;
        msgVec[i].msg_hdr.msg_namelen = msgNamelen;
        msgVec[i].msg_hdr.msg_iov = cur->packet.buffer->msgIov;
        msgVec[i].msg_hdr.msg_iovlen = cur->packet.msgIovLength;
        msgVec[i].msg_hdr.msg_control = NULL;
        msgVec[i].msg_hdr.msg_controllen = 0;
        msgVec[i].msg_hdr.msg_flags = 0;
        msgVec[i].msg_len = 0;
    }

    return i;
}


int ARSTREAM2_RTP_Sender_PacketFifoCleanFromMsgVec(ARSTREAM2_RTP_SenderContext_t *context,
                                                   ARSTREAM2_RTP_PacketFifo_t *fifo,
                                                   ARSTREAM2_RTP_PacketFifoQueue_t *queue,
                                                   struct mmsghdr *msgVec, unsigned int msgVecCount, uint64_t curTime)
{
    ARSTREAM2_RTP_PacketFifoItem_t* cur = NULL;
    unsigned int i;

    if ((!context) || (!fifo) || (!queue) || (!msgVec))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    if ((!queue->head) || (!queue->count))
    {
        //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTP_TAG, "Packet FIFO is empty");
        return -2;
    }

    for (cur = queue->head, i = 0; ((cur != NULL) && (i < msgVecCount)); cur = queue->head, i++)
    {
        size_t k, len;
        for (k = 0, len = 0; k < (size_t)msgVec[i].msg_hdr.msg_iovlen; k++)
        {
            len += msgVec[i].msg_hdr.msg_iov[k].iov_len;
        }
        if (msgVec[i].msg_len != len)
        {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTP_TAG, "Sent size (%d) does not match message iov total size (%zu)", msgVec[i].msg_len, len);
        }

        int ret = ARSTREAM2_RTP_Sender_FinishPacket(context, &cur->packet, curTime, 0);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_Sender_FinishPacket() failed (%d)", ret);
        }

        if (cur->next)
        {
            cur->next->prev = NULL;
            queue->head = cur->next;
            queue->count--;
        }
        else
        {
            queue->head = NULL;
            queue->count = 0;
            queue->tail = NULL;
        }

        if (cur->packet.buffer)
        {
            ret = ARSTREAM2_RTP_PacketFifoUnrefBuffer(fifo, cur->packet.buffer);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoUnrefBuffer() failed (%d)", ret);
            }
        }
        ret = ARSTREAM2_RTP_PacketFifoPushFreeItem(fifo, cur);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Failed to push free FIFO item");
            return -1;
        }
    }

    return (int)i;
}


int ARSTREAM2_RTP_Sender_PacketFifoCleanFromTimeout(ARSTREAM2_RTP_SenderContext_t *context,
                                                    ARSTREAM2_RTP_PacketFifo_t *fifo,
                                                    ARSTREAM2_RTP_PacketFifoQueue_t *queue, uint64_t curTime,
                                                    unsigned int *dropCount, unsigned int importanceLevelCount)
{
    ARSTREAM2_RTP_PacketFifoItem_t *cur = NULL, *next = NULL;
    int count;
    unsigned int i;

    if ((dropCount) && (importanceLevelCount > 0))
    {
        for (i = 0; i < importanceLevelCount; i++)
        {
            dropCount[i] = 0;
        }
    }

    if ((!context) || (!fifo) || (!queue))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }
    if (!curTime)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid current time");
        return -1;
    }

    if ((!queue->head) || (!queue->count))
    {
        //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTP_TAG, "Packet FIFO is empty");
        return -2;
    }

    for (cur = queue->head, count = 0; cur != NULL; cur = next)
    {
        if ((cur->packet.timeoutTimestamp != 0) && (cur->packet.timeoutTimestamp <= curTime))
        {
            if ((dropCount) && (cur->packet.importance < importanceLevelCount))
            {
                dropCount[cur->packet.importance]++;
            }
            int ret = ARSTREAM2_RTP_Sender_FinishPacket(context, &cur->packet, curTime, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_Sender_FinishPacket() failed (%d)", ret);
            }

            if (cur->next)
            {
                cur->next->prev = cur->prev;
            }
            else
            {
                queue->tail = cur->prev;
            }
            if (cur->prev)
            {
                cur->prev->next = cur->next;
            }
            else
            {
                queue->head = cur->next;
            }
            queue->count--;
            count++;

            next = cur->next;

            if (cur->packet.buffer)
            {
                ret = ARSTREAM2_RTP_PacketFifoUnrefBuffer(fifo, cur->packet.buffer);
                if (ret != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoUnrefBuffer() failed (%d)", ret);
                }
            }
            ret = ARSTREAM2_RTP_PacketFifoPushFreeItem(fifo, cur);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Failed to push free FIFO item");
                return -1;
            }
        }
        else
        {
            next = cur->next;
        }
    }

    return count;
}


int ARSTREAM2_RTP_Sender_PacketFifoRandomDrop(ARSTREAM2_RTP_SenderContext_t *context,
                                              ARSTREAM2_RTP_PacketFifo_t *fifo,
                                              ARSTREAM2_RTP_PacketFifoQueue_t *queue, float ratio, uint64_t curTime)
{
    ARSTREAM2_RTP_PacketFifoItem_t *cur = NULL, *next = NULL;
    int count;

    if ((!context) || (!fifo) || (!queue))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }
    if (!curTime)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid current time");
        return -1;
    }

    if ((!queue->head) || (!queue->count))
    {
        //ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTP_TAG, "Packet FIFO is empty");
        return -2;
    }

    for (cur = queue->head, count = 0; cur != NULL; cur = next)
    {
        if (rand() <= RAND_MAX * ratio)
        {
            int ret = ARSTREAM2_RTP_Sender_FinishPacket(context, &cur->packet, curTime, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_Sender_FinishPacket() failed (%d)", ret);
            }

            if (cur->next)
            {
                cur->next->prev = cur->prev;
            }
            else
            {
                queue->tail = cur->prev;
            }
            if (cur->prev)
            {
                cur->prev->next = cur->next;
            }
            else
            {
                queue->head = cur->next;
            }
            queue->count--;
            count++;

            next = cur->next;

            if (cur->packet.buffer)
            {
                ret = ARSTREAM2_RTP_PacketFifoUnrefBuffer(fifo, cur->packet.buffer);
                if (ret != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoUnrefBuffer() failed (%d)", ret);
                }
            }
            ret = ARSTREAM2_RTP_PacketFifoPushFreeItem(fifo, cur);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Failed to push free FIFO item");
                return -1;
            }
        }
        else
        {
            next = cur->next;
        }
    }

    return count;
}


int ARSTREAM2_RTP_Sender_PacketFifoFlushQueue(ARSTREAM2_RTP_SenderContext_t *context,
                                              ARSTREAM2_RTP_PacketFifo_t *fifo,
                                              ARSTREAM2_RTP_PacketFifoQueue_t *queue, uint64_t curTime)
{
    ARSTREAM2_RTP_PacketFifoItem_t* item;
    int count = 0, fifoErr;

    if ((!fifo) || (!queue))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }
    if (!curTime)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid current time");
        return -1;
    }

    do
    {
        item = ARSTREAM2_RTP_PacketFifoDequeueItem(queue);
        if (item)
        {
            int ret = ARSTREAM2_RTP_Sender_FinishPacket(context, &item->packet, curTime, 1);
            if (ret < 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_Sender_FinishPacket() failed (%d)", ret);
            }

            if (item->packet.buffer)
            {
                fifoErr = ARSTREAM2_RTP_PacketFifoUnrefBuffer(fifo, item->packet.buffer);
                if (fifoErr != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoUnrefBuffer() failed (%d)", fifoErr);
                }
            }

            fifoErr = ARSTREAM2_RTP_PacketFifoPushFreeItem(fifo, item);
            if (fifoErr != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoPushFreeItem() failed (%d)", fifoErr);
            }
            count++;
        }
    }
    while (item);

    return count;
}


int ARSTREAM2_RTP_Sender_PacketFifoFlush(ARSTREAM2_RTP_SenderContext_t *context,
                                         ARSTREAM2_RTP_PacketFifo_t *fifo, uint64_t curTime)
{
    ARSTREAM2_RTP_PacketFifoQueue_t *queue;
    ARSTREAM2_RTP_PacketFifoItem_t* item;
    int count = 0, fifoErr;

    if (!fifo)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }
    if (!curTime)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid current time");
        return -1;
    }

    if (!fifo->queue)
    {
        return 0;
    }

    for (queue = fifo->queue; queue; queue = queue->next)
    {
        do
        {
            item = ARSTREAM2_RTP_PacketFifoDequeueItem(queue);
            if (item)
            {
                int ret = ARSTREAM2_RTP_Sender_FinishPacket(context, &item->packet, curTime, 1);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_Sender_FinishPacket() failed (%d)", ret);
                }

                if (item->packet.buffer)
                {
                    fifoErr = ARSTREAM2_RTP_PacketFifoUnrefBuffer(fifo, item->packet.buffer);
                    if (fifoErr != 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoUnrefBuffer() failed (%d)", fifoErr);
                    }
                }

                fifoErr = ARSTREAM2_RTP_PacketFifoPushFreeItem(fifo, item);
                if (fifoErr != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoPushFreeItem() failed (%d)", fifoErr);
                }
                count++;
            }
        }
        while (item);
    }

    return count;
}


int ARSTREAM2_RTP_Sender_GeneratePacket(ARSTREAM2_RTP_SenderContext_t *context, ARSTREAM2_RTP_Packet_t *packet,
                                        uint8_t *payload, unsigned int payloadSize,
                                        uint8_t *headerExtension, unsigned int headerExtensionSize,
                                        uint64_t ntpTimestamp, uint64_t inputTimestamp,
                                        uint64_t timeoutTimestamp, uint16_t seqNum, uint32_t markerBit,
                                        uint32_t importance, uint32_t priority)
{
    uint16_t flags;

    if ((!context) || (!packet) || (!payload))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    if (payloadSize == 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid payload size (%d)", payloadSize);
        return -1;
    }

    /* Timestamps and sequence number */
    packet->inputTimestamp = inputTimestamp;
    packet->timeoutTimestamp = timeoutTimestamp;
    packet->ntpTimestamp = ntpTimestamp;
    packet->rtpTimestamp = (ntpTimestamp * context->rtpClockRate + (uint64_t)context->rtpTimestampOffset + 500000) / 1000000;
    packet->seqNum = seqNum;
    packet->markerBit = markerBit;
    packet->importance = importance;
    packet->priority = priority;

    /* Data */
    if ((headerExtension) && (headerExtensionSize > 0))
    {
        packet->headerExtension = headerExtension;
        packet->headerExtensionSize = headerExtensionSize;
    }
    packet->payload = payload;
    packet->payloadSize = payloadSize;

    /* Fill RTP packet header */
    packet->header = (ARSTREAM2_RTP_Header_t*)packet->buffer->header;
    flags = 0x8060; /* with PT=96 */
    if (headerExtensionSize > 0)
    {
        /* set the extention bit */
        flags |= (1 << 12);
    }
    if (markerBit)
    {
        /* set the marker bit */
        flags |= (1 << 7);
    }
    packet->header->flags = htons(flags);
    packet->header->seqNum = htons(seqNum);
    packet->header->timestamp = htonl(packet->rtpTimestamp);
    packet->header->ssrc = htonl(context->senderSsrc);

    /* Fill the IOV array */
    packet->msgIovLength = 0;
    packet->buffer->msgIov[packet->msgIovLength].iov_base = (void*)packet->header;
    packet->buffer->msgIov[packet->msgIovLength].iov_len = (size_t)sizeof(ARSTREAM2_RTP_Header_t);
    packet->msgIovLength++;
    if (headerExtensionSize > 0)
    {
        packet->buffer->msgIov[packet->msgIovLength].iov_base = (void*)packet->headerExtension;
        packet->buffer->msgIov[packet->msgIovLength].iov_len = (size_t)headerExtensionSize;
        packet->msgIovLength++;
    }
    packet->buffer->msgIov[packet->msgIovLength].iov_base = (void*)packet->payload;
    packet->buffer->msgIov[packet->msgIovLength].iov_len = (size_t)payloadSize;
    packet->msgIovLength++;

    return 0;
}


int ARSTREAM2_RTP_Sender_FinishPacket(ARSTREAM2_RTP_SenderContext_t *context, ARSTREAM2_RTP_Packet_t *packet, uint64_t curTime, int dropped)
{
    if ((!context) || (!packet))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }
    if (!curTime)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid current time");
        return -1;
    }

    if (dropped)
    {
        context->droppedPacketCount++;
        context->droppedByteIntegral += packet->payloadSize;
        context->droppedByteIntegralSq += (packet->payloadSize * packet->payloadSize);
        context->inputToDroppedTimeIntegral += (curTime - packet->inputTimestamp);
        context->inputToDroppedTimeIntegralSq += ((curTime - packet->inputTimestamp) * (curTime - packet->inputTimestamp));
    }
    else
    {
        context->sentPacketCount++;
        context->sentByteIntegral += packet->payloadSize;
        context->sentByteIntegralSq += (packet->payloadSize * packet->payloadSize);
        context->inputToSentTimeIntegral += (curTime - packet->inputTimestamp);
        context->inputToSentTimeIntegralSq += ((curTime - packet->inputTimestamp) * (curTime - packet->inputTimestamp));
    }

    /* call the monitoringCallback */
    if (context->monitoringCallback != NULL)
    {
        context->monitoringCallback(packet->inputTimestamp, curTime, packet->ntpTimestamp, packet->rtpTimestamp, packet->seqNum,
                                    packet->markerBit, packet->importance, packet->priority,
                                    (dropped) ? 0 : packet->payloadSize, (dropped) ? packet->payloadSize : 0,
                                    context->monitoringCallbackUserPtr);
    }

    return 0;
}


/* WARNING: the call sequence ARSTREAM2_RTP_Receiver_PacketFifoFillMsgVec -> recvmmsg -> ARSTREAM2_RTP_Receiver_PacketFifoAddFromMsgVec
   must not be broken (no change made to the free items list) */
int ARSTREAM2_RTP_Receiver_PacketFifoFillMsgVec(ARSTREAM2_RTP_PacketFifo_t *fifo, struct mmsghdr *msgVec, unsigned int msgVecCount)
{
    ARSTREAM2_RTP_PacketFifoBuffer_t* cur = NULL;
    unsigned int i;

    if (!fifo)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    if (!fifo->bufferFree)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Packet FIFO is full => flush to recover");
        int ret = ARSTREAM2_RTP_Receiver_PacketFifoFlush(fifo);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_Receiver_PacketFifoFlush() failed (%d)", ret);
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "%d packets flushed", ret);
        }
    }

    for (cur = fifo->bufferFree, i = 0; ((cur) && (i < msgVecCount)); cur = cur->next, i++)
    {
        /* RTP header */
        cur->msgIov[0].iov_base = cur->header;
        cur->msgIov[0].iov_len = cur->headerSize;

        /* RTP payload */
        cur->msgIov[1].iov_base = cur->buffer;
        cur->msgIov[1].iov_len = cur->bufferSize;

        msgVec[i].msg_hdr.msg_name = NULL;
        msgVec[i].msg_hdr.msg_namelen = 0;
        msgVec[i].msg_hdr.msg_iov = cur->msgIov;
        msgVec[i].msg_hdr.msg_iovlen = 2;
        msgVec[i].msg_hdr.msg_control = NULL;
        msgVec[i].msg_hdr.msg_controllen = 0;
        msgVec[i].msg_hdr.msg_flags = 0;
        msgVec[i].msg_len = 0;
    }

    return i;
}


static int ARSTREAM2_RTP_Receiver_PacketFifoResendEnqueue(ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoQueue_t *queue, ARSTREAM2_RTP_PacketFifoItem_t *item, uint64_t curTime, uint32_t timeout)
{
    int err = 0, ret = 0, needUnref = 0, needFree = 0;
    ARSTREAM2_RTP_PacketFifoItem_t *resendItem = NULL;

    /* add ref to packet buffer */
    ret = ARSTREAM2_RTP_PacketFifoBufferAddRef(item->packet.buffer);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoBufferAddRef() failed (%d)", ret);
    }
    if (ret == 0)
    {
        /* duplicate the packet item */
        resendItem = ARSTREAM2_RTP_PacketFifoDuplicateItem(fifo, item);
        if (resendItem)
        {
            ARSTREAM2_RTP_Packet_t *packet = &resendItem->packet;
            packet->buffer = item->packet.buffer;
            packet->timeoutTimestamp = curTime + timeout; //TODO: compute the expected arrival time
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Failed to pop free item from the packet FIFO");
            ret = -1;
            needUnref = 1;
        }
    }

    if ((ret == 0) && (resendItem))
    {
        /* enqueue the AU */
        ret = ARSTREAM2_RTP_PacketFifoEnqueueItem(queue, resendItem);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoEnqueueItem() failed (%d)", ret);
            err = -1;
            needUnref = 1;
            needFree = 1;
        }
    }
    else
    {
        err = -1;
    }

    /* error handling */
    if (needFree)
    {
        ret = ARSTREAM2_RTP_PacketFifoPushFreeItem(fifo, resendItem);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Failed to push free item in the packet FIFO (%d)", ret);
        }
        needFree = 0;
    }
    if (needUnref)
    {
        ret = ARSTREAM2_RTP_PacketFifoUnrefBuffer(fifo, item->packet.buffer);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Failed to unref buffer (%d)", ret);
        }
        needUnref = 0;
    }

    return err;
}


/* WARNING: the call sequence ARSTREAM2_RTP_Receiver_PacketFifoFillMsgVec -> recvmmsg -> ARSTREAM2_RTP_Receiver_PacketFifoAddFromMsgVec
   must not be broken (no change made to the free items list) */
int ARSTREAM2_RTP_Receiver_PacketFifoAddFromMsgVec(ARSTREAM2_RTP_ReceiverContext_t *context,
                                                   ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoQueue_t *queue,
                                                   ARSTREAM2_RTP_PacketFifoQueue_t **resendQueue, uint32_t *resendTimeout, unsigned int resendCount,
                                                   struct mmsghdr *msgVec, unsigned int msgVecCount, uint64_t curTime,
                                                   ARSTREAM2_RTCP_ReceiverContext_t *rtcpContext)
{
    ARSTREAM2_RTP_PacketFifoItem_t* item = NULL;
    ARSTREAM2_RTP_PacketFifoBuffer_t *buffer = NULL;
    ARSTREAM2_RTP_PacketFifoItem_t* garbage = NULL;
    int ret = 0, resendRet, garbageCount = 0, garbageCount2 = 0;
    unsigned int i, k, popCount = 0, enqueueCount = 0;

    if ((!context) || (!fifo) || (!rtcpContext))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    if ((resendCount > 0) && (!resendQueue))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid resend queue list");
        return -1;
    }

    if (!msgVecCount)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Empty msgVec array");
        return -2;
    }

    uint64_t recvRtpTimestamp = (curTime * context->rtpClockRate + 500000) / 1000000;

    for (i = 0; i < msgVecCount; i++)
    {
        buffer = ARSTREAM2_RTP_PacketFifoGetBuffer(fifo);
        item = ARSTREAM2_RTP_PacketFifoPopFreeItem(fifo);
        if ((item) && (buffer))
        {
            ARSTREAM2_RTP_PacketReset(&item->packet);
            item->packet.buffer = buffer;
            popCount++;
            if (msgVec[i].msg_len > sizeof(ARSTREAM2_RTP_Header_t))
            {
                uint16_t flags;
                int seqNumDelta = 0;

                item->packet.header = (ARSTREAM2_RTP_Header_t*)item->packet.buffer->header;
                item->packet.inputTimestamp = curTime;
                item->packet.rtpTimestamp = ntohl(item->packet.header->timestamp);
                item->packet.seqNum = ntohs(item->packet.header->seqNum);
                item->packet.importance = 0; //TODO: how to get this value on the receiver side for resenders?
                item->packet.priority = 0; //TODO: how to get this value on the receiver side for resenders?
                if (context->previousExtSeqNum != -1)
                {
                    item->packet.extSeqNum = (context->extHighestSeqNum & 0xFFFF0000) | ((uint32_t)item->packet.seqNum & 0xFFFF);
                    if ((int64_t)item->packet.extSeqNum - (int64_t)context->previousExtSeqNum < -32768)
                    {
                        item->packet.extSeqNum += 65536;
                    }
                    else if ((int64_t)item->packet.extSeqNum - (int64_t)context->previousExtSeqNum > 32768)
                    {
                        item->packet.extSeqNum -= 65536;
                    }
                    seqNumDelta = item->packet.extSeqNum - context->extHighestSeqNum;
                    if (item->packet.extSeqNum > context->extHighestSeqNum)
                    {
                        context->extHighestSeqNum = item->packet.extSeqNum;
                        rtcpContext->extHighestSeqNum = context->extHighestSeqNum;
                    }
                    item->packet.extRtpTimestamp = (context->extHighestRtpTimestamp & 0xFFFFFFFF00000000ULL) | ((uint64_t)item->packet.rtpTimestamp & 0xFFFFFFFFULL);
                    if ((int64_t)item->packet.extRtpTimestamp - (int64_t)context->previousExtRtpTimestamp < -2147483648LL)
                    {
                        item->packet.extRtpTimestamp += 0x100000000ULL;
                    }
                    else if ((int64_t)item->packet.extRtpTimestamp - (int64_t)context->previousExtRtpTimestamp > 2147483648LL)
                    {
                        item->packet.extRtpTimestamp -= 0x100000000ULL;
                    }
                    if (item->packet.extRtpTimestamp > context->extHighestRtpTimestamp)
                    {
                        context->extHighestRtpTimestamp = item->packet.extRtpTimestamp;
                    }
                }
                else
                {
                    /* first packet received */
                    item->packet.extSeqNum = item->packet.seqNum;
                    item->packet.extRtpTimestamp = item->packet.rtpTimestamp;
                    context->extHighestSeqNum = item->packet.extSeqNum;
                    context->extHighestRtpTimestamp = item->packet.extRtpTimestamp;
                    context->previousRecvRtpTimestamp = recvRtpTimestamp;
                    context->previousExtRtpTimestamp = item->packet.extRtpTimestamp;
                    context->firstRecvRtpTimestamp = recvRtpTimestamp;
                    context->firstExtRtpTimestamp = item->packet.extRtpTimestamp;
                    rtcpContext->senderSsrc = ntohl(item->packet.header->ssrc);
                    rtcpContext->firstSeqNum = item->packet.seqNum;
                    rtcpContext->extHighestSeqNum = context->extHighestSeqNum;
                    rtcpContext->packetsReceived = 0;
                    rtcpContext->packetsLost = 0;
                }
                context->previousExtSeqNum = (int32_t)item->packet.extSeqNum;
                flags = ntohs(item->packet.header->flags);
                if (flags & (1 << 7))
                {
                    /* the marker bit is set */
                    item->packet.markerBit = 1;
                }
                if (flags & (1 << 12))
                {
                    /* the extention bit is set */
                    item->packet.headerExtension = item->packet.buffer->buffer;
                    uint16_t length = ntohs(*((uint16_t*)(item->packet.headerExtension + 2)));
                    item->packet.headerExtensionSize = length * 4 + 4;
                }
                else
                {
                    item->packet.headerExtension = NULL;
                    item->packet.headerExtensionSize = 0;
                }
                item->packet.payload = item->packet.buffer->buffer + item->packet.headerExtensionSize;
                item->packet.payloadSize = msgVec[i].msg_len - sizeof(ARSTREAM2_RTP_Header_t) - item->packet.headerExtensionSize;
                item->packet.buffer->msgIov[0].iov_len = sizeof(ARSTREAM2_RTP_Header_t);
                item->packet.buffer->msgIov[1].iov_len = item->packet.headerExtensionSize + item->packet.payloadSize;
                item->packet.msgIovLength = 2;

                ret = ARSTREAM2_RTP_PacketFifoEnqueueItemOrderedBySeqNum(queue, item);
                if (ret < 0)
                {
                    if (ret == -3)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTP_TAG, "Duplicate RTP packet received (seqNum %d, extSeqNum %d)",
                                    item->packet.seqNum, item->packet.extSeqNum);
                    }
                    else
                    {
                        rtcpContext->packetsLost++;
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoEnqueueItem() failed (%d)", ret);
                    }
                    /* failed to enqueue, flag the item for garbage collection */
                    garbageCount++;
                    if (!garbage)
                    {
                        garbage = item;
                    }
                    else
                    {
                        item->next = garbage;
                        garbage->prev = item;
                        garbage = item;
                    }
                }
                else if (ret == 1)
                {
                    /*ARSAL_PRINT(ARSAL_PRINT_VERBOSE, ARSTREAM2_RTP_TAG, "Out of order RTP packet received (seqNum %d, extSeqNum %d, delta %d)",
                                item->packet.seqNum, item->packet.extSeqNum, -seqNumDelta); //TODO: debug */
                    enqueueCount++;
                }
                else
                {
                    if ((recvRtpTimestamp != context->previousRecvRtpTimestamp) && (item->packet.extRtpTimestamp != context->previousExtRtpTimestamp))
                    {
                        /* clock skew computation */
                        int64_t clockSkew = ((int64_t)recvRtpTimestamp - (int64_t)context->firstRecvRtpTimestamp)
                                            - ((int64_t)item->packet.extRtpTimestamp - (int64_t)context->firstExtRtpTimestamp);

                        /* initialize the window */
                        if (context->clockSkewWindowSize == 0)
                        {
                            context->clockSkewWindowStartTimestamp = curTime;
                        }

                        /* fill the window */
                        context->clockSkewWindow[context->clockSkewWindowSize] = clockSkew;
                        context->clockSkewWindowSize++;

                        if ((context->clockSkewWindowSize >= ARSTREAM2_RTP_CLOCKSKEW_WINDOW_SIZE)
                                || ((context->clockSkewWindowSize >= ARSTREAM2_RTP_CLOCKSKEW_WINDOW_SIZE / 2) && (curTime >= context->clockSkewWindowStartTimestamp + ARSTREAM2_RTP_CLOCKSKEW_WINDOW_TIMEOUT)))
                        {
                            /* window is full or half-full and on timeout */
                            int i;
                            context->clockSkewMin = context->clockSkewWindow[0];

                            /* Find the minimum clock skew in the window */
                            for (i = 0; i < context->clockSkewWindowSize; i++)
                            {
                                if (context->clockSkewWindow[i] < context->clockSkewMin)
                                {
                                    context->clockSkewMin = context->clockSkewWindow[i];
                                }
                            }

                            /* Average min clock skew */
                            if (!context->clockSkewInit)
                            {
                                context->clockSkewOffset = context->clockSkewMin;
                                context->clockSkewMinAvg = context->clockSkewMin - context->clockSkewOffset;
                                context->clockSkewInit = 1;
                            }
                            else
                            {
                                /* Sliding average, alpha = 1 / ARSTREAM2_RTP_CLOCKSKEW_AVG_ALPHA */
                                context->clockSkewMinAvg = context->clockSkewMinAvg
                                                           + (context->clockSkewMin - context->clockSkewOffset - context->clockSkewMinAvg + ARSTREAM2_RTP_CLOCKSKEW_AVG_ALPHA / 2) / ARSTREAM2_RTP_CLOCKSKEW_AVG_ALPHA;
                            }
                            context->clockSkew = (context->clockSkewMinAvg * 1000000 + context->rtpClockRate / 2) / context->rtpClockRate;

                            /* Reset the window */
                            context->clockSkewWindowSize = 0;
                        }
                    }

                    /* interarrival jitter computation */
                    int64_t d = ((int64_t)context->previousRecvRtpTimestamp - (int64_t)context->previousExtRtpTimestamp)
                                - ((int64_t)recvRtpTimestamp - (int64_t)item->packet.extRtpTimestamp);
                    if (d < 0) d = -d;
                    rtcpContext->interarrivalJitter = (uint32_t)((int64_t)rtcpContext->interarrivalJitter
                                                      + (d - (int64_t)rtcpContext->interarrivalJitter) / 16);

                    context->previousRecvRtpTimestamp = recvRtpTimestamp;
                    context->previousExtRtpTimestamp = item->packet.extRtpTimestamp;
                    enqueueCount++;
                }
                item->packet.ntpTimestampRaw = (item->packet.extRtpTimestamp * 1000000 + context->rtpClockRate / 2) / context->rtpClockRate;
                item->packet.ntpTimestampRawUnskewed = ((int64_t)item->packet.ntpTimestampRaw + context->clockSkew >= 0) ? item->packet.ntpTimestampRaw + context->clockSkew : 0;
                item->packet.ntpTimestamp = ARSTREAM2_RTCP_Receiver_GetNtpTimestampFromRtpTimestamp(rtcpContext, item->packet.extRtpTimestamp);
                item->packet.ntpTimestampUnskewed = ((int64_t)item->packet.ntpTimestamp + context->clockSkew >= 0) ? item->packet.ntpTimestamp + context->clockSkew : 0;
                item->packet.ntpTimestampLocal = ((rtcpContext->clockDeltaCtx.clockDeltaAvg != 0) && (item->packet.ntpTimestamp != 0)) ? (item->packet.ntpTimestamp - rtcpContext->clockDeltaCtx.clockDeltaAvg) : 0;
                item->packet.timeoutTimestamp = curTime + context->nominalDelay; //TODO: compute the expected arrival time

                if (ret >= 0)
                {
                    for (k = 0; k < resendCount; k++)
                    {
                        resendRet = ARSTREAM2_RTP_Receiver_PacketFifoResendEnqueue(fifo, resendQueue[k], item, curTime, resendTimeout[k]);
                        if (resendRet != 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Failed to enqueue packet into resend queue #%d (%d)", k, resendRet);
                        }
                    }
                }
            }
            else
            {
                /* invalid payload, flag the item for garbage collection */
                rtcpContext->packetsLost++;
                garbageCount++;
                if (!garbage)
                {
                    garbage = item;
                }
                else
                {
                    item->next = garbage;
                    garbage->prev = item;
                    garbage = item;
                }
            }
        }
        else
        {
            if (buffer) ARSTREAM2_RTP_PacketFifoUnrefBuffer(fifo, buffer);
            if (item) ARSTREAM2_RTP_PacketFifoPushFreeItem(fifo, item);
            break;
        }
    }

    while (garbage)
    {
        garbageCount2++;
        ARSTREAM2_RTP_PacketFifoItem_t* next = garbage->next;
        int garbageRet;
        if (garbage->packet.buffer)
        {
            garbageRet = ARSTREAM2_RTP_PacketFifoUnrefBuffer(fifo, garbage->packet.buffer);
            if (garbageRet != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoUnrefBuffer() failed (%d)", garbageRet);
            }
        }
        garbageRet = ARSTREAM2_RTP_PacketFifoPushFreeItem(fifo, garbage);
        if (garbageRet < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoPushFreeItem() failed (%d)", garbageRet);
        }
        garbage = next;
    }
    if (garbageCount != garbageCount2)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Garbage count mismatch: %d vs. %d", garbageCount, garbageCount2);
    }
    if (popCount != enqueueCount + garbageCount)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Pop count mismatch: %d vs. %d", popCount, enqueueCount + garbageCount);
    }
    //ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTP_TAG, "popCount=%d, enqueueCount=%d, garbageCount=%d", popCount, enqueueCount, garbageCount); //TODO: debug

    return 0;
}


int ARSTREAM2_RTP_Receiver_PacketFifoFlushQueue(ARSTREAM2_RTP_PacketFifo_t *fifo, ARSTREAM2_RTP_PacketFifoQueue_t *queue)
{
    ARSTREAM2_RTP_PacketFifoItem_t* item;
    int count = 0, fifoErr;

    if ((!fifo) || (!queue))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    do
    {
        item = ARSTREAM2_RTP_PacketFifoDequeueItem(queue);
        if (item)
        {
            if (item->packet.buffer)
            {
                fifoErr = ARSTREAM2_RTP_PacketFifoUnrefBuffer(fifo, item->packet.buffer);
                if (fifoErr != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoUnrefBuffer() failed (%d)", fifoErr);
                }
            }

            fifoErr = ARSTREAM2_RTP_PacketFifoPushFreeItem(fifo, item);
            if (fifoErr != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoPushFreeItem() failed (%d)", fifoErr);
            }
            count++;
        }
    }
    while (item);

    return count;
}


int ARSTREAM2_RTP_Receiver_PacketFifoFlush(ARSTREAM2_RTP_PacketFifo_t *fifo)
{
    ARSTREAM2_RTP_PacketFifoQueue_t *queue;
    ARSTREAM2_RTP_PacketFifoItem_t* item;
    int count = 0, fifoErr;

    if (!fifo)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "Invalid pointer");
        return -1;
    }

    if (!fifo->queue)
    {
        return 0;
    }

    for (queue = fifo->queue; queue; queue = queue->next)
    {
        do
        {
            item = ARSTREAM2_RTP_PacketFifoDequeueItem(queue);
            if (item)
            {
                if (item->packet.buffer)
                {
                    fifoErr = ARSTREAM2_RTP_PacketFifoUnrefBuffer(fifo, item->packet.buffer);
                    if (fifoErr != 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoUnrefBuffer() failed (%d)", fifoErr);
                    }
                }

                fifoErr = ARSTREAM2_RTP_PacketFifoPushFreeItem(fifo, item);
                if (fifoErr != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_TAG, "ARSTREAM2_RTP_PacketFifoPushFreeItem() failed (%d)", fifoErr);
                }
                count++;
            }
        }
        while (item);
    }

    return count;
}
