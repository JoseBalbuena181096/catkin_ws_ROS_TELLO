/**
 * @file arstream2_rtp_resender.h
 * @brief Parrot Streaming Library - RTP Resender
 * @date 10/30/2016
 * @author aurelien.barre@parrot.com
 */

#ifndef _ARSTREAM2_RTP_RESENDER_H_
#define _ARSTREAM2_RTP_RESENDER_H_

#include <config.h>

#include <inttypes.h>

#include <libARStream2/arstream2_error.h>
#include "arstream2_rtp_sender.h"
#include "arstream2_rtp.h"
#include "arstream2_rtp_h264.h"
#include "arstream2_rtcp.h"
#include "arstream2_h264.h"


/**
 * Default stream socket send buffer size (100ms at 10 Mbit/s)
 */
#define ARSTREAM2_RTP_RESENDER_DEFAULT_STREAM_SOCKET_SEND_BUFFER_SIZE (10000000 * 100 / 1000 / 8)


typedef struct ARSTREAM2_RtpResender_s
{
    ARSTREAM2_RtpSender_t *sender;
    ARSTREAM2_RTP_PacketFifoQueue_t packetFifoQueue;
    int streamSocketSendBufferSize;
    uint32_t maxNetworkLatencyUs;

    struct ARSTREAM2_RtpResender_s *prev;
    struct ARSTREAM2_RtpResender_s *next;

} ARSTREAM2_RtpResender_t;


#endif /* _ARSTREAM2_RTP_RESENDER_H_ */
