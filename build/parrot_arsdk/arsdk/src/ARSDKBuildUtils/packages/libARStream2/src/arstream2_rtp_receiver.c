/**
 * @file arstream2_rtp_receiver.c
 * @brief Parrot Streaming Library - RTP Receiver
 * @date 04/16/2015
 * @author aurelien.barre@parrot.com
 */


#include "arstream2_rtp_receiver.h"


#define ARSTREAM2_RTP_RECEIVER_TAG "ARSTREAM2_RtpReceiver"


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


static int ARSTREAM2_RtpReceiver_SetSocketSendBufferSize(ARSTREAM2_RtpReceiver_t *receiver, int socket, int size)
{
    int ret = 0, err;
    socklen_t size2 = sizeof(size2);

    err = setsockopt(socket, SOL_SOCKET, SO_SNDBUF, (void*)&size, sizeof(size));
    if (err != 0)
    {
        ret = -1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to set socket send buffer size to 2*%d bytes: error=%d (%s)", size, errno, strerror(errno));
    }

    size = -1;
    err = getsockopt(socket, SOL_SOCKET, SO_SNDBUF, (void*)&size, &size2);
    if (err != 0)
    {
        ret = -1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to get socket send buffer size: error=%d (%s)", errno, strerror(errno));
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_RTP_RECEIVER_TAG, "Socket send buffer size is 2*%d bytes", size);
    }

    return ret;
}


static int ARSTREAM2_RtpReceiver_SetSocketReceiveBufferSize(ARSTREAM2_RtpReceiver_t *receiver, int socket, int size)
{
    int ret = 0, err;
    socklen_t size2 = sizeof(size2);

    err = setsockopt(socket, SOL_SOCKET, SO_RCVBUF, (void*)&size, sizeof(size));
    if (err != 0)
    {
        ret = -1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to set socket receive buffer size to 2*%d bytes: error=%d (%s)", size, errno, strerror(errno));
    }

    size = -1;
    err = getsockopt(socket, SOL_SOCKET, SO_RCVBUF, (void*)&size, &size2);
    if (err != 0)
    {
        ret = -1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to get socket receive buffer size: error=%d (%s)", errno, strerror(errno));
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_RTP_RECEIVER_TAG, "Socket receive buffer size is 2*%d bytes", size);
    }

    return ret;
}

static int ARSTREAM2_RtpReceiver_StreamMuxSetup(ARSTREAM2_RtpReceiver_t *receiver)
{
#if BUILD_LIBMUX
    int ret, r2;

    if (receiver == NULL || receiver->mux.mux == NULL)
        return -EINVAL;

    ret = mux_channel_open(receiver->mux.mux, MUX_ARSDK_CHANNEL_ID_STREAM_DATA,
                           NULL, NULL);
    if (ret != 0)
        goto fail;

    ret = mux_channel_alloc_queue(receiver->mux.mux,
                                  MUX_ARSDK_CHANNEL_ID_STREAM_DATA,
                                  0,
                                  &(receiver->mux.data));

    if (ret != 0)
        goto close_channel;


    return 0;

close_channel:
    r2 = mux_channel_close(receiver->mux.mux, MUX_ARSDK_CHANNEL_ID_STREAM_DATA);
    if (r2 != 0)
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG,
                    "Error while closing mux channel in error handler: %s (%d)",
                    strerror(-r2), r2);
fail:
    receiver->mux.data = NULL;
    return ret;
#else
    return -ENOSYS;
#endif
}


static int ARSTREAM2_RtpReceiver_StreamSocketSetup(ARSTREAM2_RtpReceiver_t *receiver)
{
    int ret = 0;
    struct sockaddr_in recvSin;
    int err;

    /* create socket */
    receiver->net.streamSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (receiver->net.streamSocket < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to create stream socket");
        ret = -1;
    }

#if HAVE_DECL_SO_NOSIGPIPE
    if (ret == 0)
    {
        /* remove SIGPIPE */
        int set = 1;
        err = setsockopt(receiver->net.streamSocket, SOL_SOCKET, SO_NOSIGPIPE, (void*)&set, sizeof(int));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Error on setsockopt: error=%d (%s)", errno, strerror(errno));
        }
    }
#endif

    if (ret == 0)
    {
        int tos = receiver->net.classSelector;
        err = ARSAL_Socket_Setsockopt(receiver->net.streamSocket, IPPROTO_IP, IP_TOS, (void*)&tos, sizeof(int));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Error on setsockopt: error=%d (%s)", errno, strerror(errno));
        }
    }

    if (ret == 0)
    {
        /* set to non-blocking */
        int flags = fcntl(receiver->net.streamSocket, F_GETFL, 0);
        err = fcntl(receiver->net.streamSocket, F_SETFL, flags | O_NONBLOCK);
        if (err < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to set to non-blocking: error=%d (%s)", errno, strerror(errno));
        }

        memset(&recvSin, 0, sizeof(struct sockaddr_in));
        recvSin.sin_family = AF_INET;
        recvSin.sin_port = htons(receiver->net.clientStreamPort);
        recvSin.sin_addr.s_addr = htonl(INADDR_ANY);

        if (receiver->net.isMulticast)
        {
            int addrFirst = atoi(receiver->net.serverAddr);
            if ((addrFirst >= 224) && (addrFirst <= 239))
            {
                /* multicast */
                struct ip_mreq mreq;
                memset(&mreq, 0, sizeof(mreq));
                err = inet_pton(AF_INET, receiver->net.serverAddr, &(mreq.imr_multiaddr.s_addr));
                if (err <= 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to convert address '%s'", receiver->net.serverAddr);
                    ret = -1;
                }

                if (ret == 0)
                {
                    if ((receiver->net.mcastIfaceAddr) && (strlen(receiver->net.mcastIfaceAddr) > 0))
                    {
                        err = inet_pton(AF_INET, receiver->net.mcastIfaceAddr, &(mreq.imr_interface.s_addr));
                        if (err <= 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to convert address '%s'", receiver->net.mcastIfaceAddr);
                            ret = -1;
                        }
                    }
                    else
                    {
                        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
                    }
                }

                if (ret == 0)
                {
                    /* join the multicast group */
                    err = setsockopt(receiver->net.streamSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq));
                    if (err != 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to join multacast group: error=%d (%s)", errno, strerror(errno));
                        ret = -1;
                    }
                }
            }
            else
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Invalid multicast address '%s'", receiver->net.serverAddr);
                ret = -1;
            }
        }
    }

    if (ret == 0)
    {
        /* allow multiple sockets to use the same port */
        unsigned int yes = 1;
        err = setsockopt(receiver->net.streamSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to set socket option SO_REUSEADDR: error=%d (%s)", errno, strerror(errno));
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* bind the socket */
        err = bind(receiver->net.streamSocket, (struct sockaddr*)&recvSin, sizeof(recvSin));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Error on stream socket bind port=%d: error=%d (%s)", receiver->net.clientStreamPort, errno, strerror(errno));
            ret = -1;
        }
    }

    if (ret != 0)
    {
        if (receiver->net.streamSocket >= 0)
        {
            while (((err = close(receiver->net.streamSocket)) == -1) && (errno == EINTR));
        }
        receiver->net.streamSocket = -1;
    }

    return ret;
}

static int ARSTREAM2_RtpReceiver_StreamMuxTeardown(ARSTREAM2_RtpReceiver_t *receiver)
{
#if BUILD_LIBMUX
    int ret;
    if (receiver == NULL || receiver->mux.mux == NULL)
        return -EINVAL;

    if (receiver->mux.data == NULL)
        return 0;

    ret = mux_channel_close(receiver->mux.mux,
                            MUX_ARSDK_CHANNEL_ID_STREAM_DATA);
    if (ret == 0)
        receiver->mux.data = NULL;

    return ret;
#else
    return -ENOSYS;
#endif
}

static int ARSTREAM2_RtpReceiver_StreamSocketTeardown(ARSTREAM2_RtpReceiver_t *receiver)
{
    if (receiver == NULL)
        return -EINVAL;

    if (receiver->net.streamSocket != -1)
    {
        int err;
        while (((err = close(receiver->net.streamSocket)) == -1) && (errno == EINTR));
        receiver->net.streamSocket = -1;
    }

    return 0;
}

static int ARSTREAM2_RtpReceiver_ControlMuxSetup(ARSTREAM2_RtpReceiver_t *receiver)
{
#if BUILD_LIBMUX
    int ret, r2;
    if (receiver == NULL || receiver->mux.mux == NULL)
        return -EINVAL;

    ret = mux_channel_open(receiver->mux.mux,
                           MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL,
                           NULL, NULL);
    if (ret != 0)
        goto fail;

    ret = mux_channel_alloc_queue(receiver->mux.mux,
                                  MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL,
                                  0,
                                  &(receiver->mux.control));

    if (ret != 0)
        goto close_channel;


    return 0;

close_channel:
    r2 = mux_channel_close(receiver->mux.mux,
                           MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL);
    if (r2 != 0)
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG,
                    "Error while closing mux channel in error handler: %s (%d)",
                    strerror(-r2), r2);
fail:
    receiver->mux.control = NULL;
    return ret;
#else
    return -ENOSYS;
#endif
}


static int ARSTREAM2_RtpReceiver_ControlSocketSetup(ARSTREAM2_RtpReceiver_t *receiver)
{
    int ret = 0;
    struct sockaddr_in recvSin;
    int err;

    if (ret == 0)
    {
        /* create socket */
        receiver->net.controlSocket = socket(AF_INET, SOCK_DGRAM, 0);
        if (receiver->net.controlSocket < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to create control socket");
            ret = -1;
        }
    }

#if HAVE_DECL_SO_NOSIGPIPE
    if (ret == 0)
    {
        /* remove SIGPIPE */
        int set = 1;
        err = setsockopt(receiver->net.controlSocket, SOL_SOCKET, SO_NOSIGPIPE, (void*)&set, sizeof(int));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Error on setsockopt: error=%d (%s)", errno, strerror(errno));
        }
    }
#endif

    if (ret == 0)
    {
        int tos = receiver->net.classSelector;
        err = ARSAL_Socket_Setsockopt(receiver->net.controlSocket, IPPROTO_IP, IP_TOS, (void*)&tos, sizeof(int));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Error on setsockopt: error=%d (%s)", errno, strerror(errno));
        }
    }

    if (ret == 0)
    {
        /* set to non-blocking */
        int flags = fcntl(receiver->net.controlSocket, F_GETFL, 0);
        err = fcntl(receiver->net.controlSocket, F_SETFL, flags | O_NONBLOCK);
        if (err < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to set to non-blocking: error=%d (%s)", errno, strerror(errno));
        }

        /* receive address */
        memset(&recvSin, 0, sizeof(struct sockaddr_in));
        recvSin.sin_family = AF_INET;
        recvSin.sin_port = htons(receiver->net.clientControlPort);
        recvSin.sin_addr.s_addr = htonl(INADDR_ANY);

        if (receiver->net.isMulticast)
        {
            int addrFirst = atoi(receiver->net.serverAddr);
            if ((addrFirst >= 224) && (addrFirst <= 239))
            {
                /* multicast */
                struct ip_mreq mreq;
                memset(&mreq, 0, sizeof(mreq));
                err = inet_pton(AF_INET, receiver->net.serverAddr, &(mreq.imr_multiaddr.s_addr));
                if (err <= 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to convert address '%s'", receiver->net.serverAddr);
                    ret = -1;
                }

                if (ret == 0)
                {
                    if ((receiver->net.mcastIfaceAddr) && (strlen(receiver->net.mcastIfaceAddr) > 0))
                    {
                        err = inet_pton(AF_INET, receiver->net.mcastIfaceAddr, &(mreq.imr_interface.s_addr));
                        if (err <= 0)
                        {
                            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to convert address '%s'", receiver->net.mcastIfaceAddr);
                            ret = -1;
                        }
                    }
                    else
                    {
                        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
                    }
                }

                if (ret == 0)
                {
                    /* join the multicast group */
                    err = setsockopt(receiver->net.controlSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq));
                    if (err != 0)
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to join multacast group: error=%d (%s)", errno, strerror(errno));
                        ret = -1;
                    }
                }
            }
            else
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Invalid multicast address '%s'", receiver->net.serverAddr);
                ret = -1;
            }
        }
    }

    if (ret == 0)
    {
        /* allow multiple sockets to use the same port */
        unsigned int yes = 1;
        err = setsockopt(receiver->net.controlSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to set socket option SO_REUSEADDR: error=%d (%s)", errno, strerror(errno));
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* bind the socket */
        err = bind(receiver->net.controlSocket, (struct sockaddr*)&recvSin, sizeof(recvSin));
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Error on control socket bind port=%d: error=%d (%s)", receiver->net.clientControlPort, errno, strerror(errno));
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* send address */
        memset(&receiver->net.controlSendSin, 0, sizeof(struct sockaddr_in));
        receiver->net.controlSendSin.sin_family = AF_INET;
        receiver->net.controlSendSin.sin_port = htons(receiver->net.serverControlPort);
        err = inet_pton(AF_INET, receiver->net.serverAddr, &(receiver->net.controlSendSin.sin_addr));
        if (err <= 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to convert address '%s'", receiver->net.serverAddr);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        /* set the socket buffer size */
        err = ARSTREAM2_RtpReceiver_SetSocketSendBufferSize(receiver, receiver->net.controlSocket, 4096);
        if (err != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to set the send socket buffer size");
            ret = -1;
        }
    }

    if (ret != 0)
    {
        if (receiver->net.controlSocket >= 0)
        {
            while (((err = close(receiver->net.controlSocket)) == -1) && (errno == EINTR));
        }
        receiver->net.controlSocket = -1;
    }

    return ret;
}

static int ARSTREAM2_RtpReceiver_ControlMuxTeardown(ARSTREAM2_RtpReceiver_t *receiver)
{
#if BUILD_LIBMUX
    int ret;
    if (receiver == NULL || receiver->mux.mux == NULL)
        return -EINVAL;

    if (receiver->mux.control == NULL)
        return 0;

    ret = mux_channel_close(receiver->mux.mux,
                            MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL);
    if (ret == 0)
        receiver->mux.control = NULL;

    return ret;
#else
    return -ENOSYS;
#endif
}

static int ARSTREAM2_RtpReceiver_ControlSocketTeardown(ARSTREAM2_RtpReceiver_t *receiver)
{
    if (receiver == NULL)
        return -EINVAL;

    if (receiver->net.controlSocket != -1)
    {
        int err;
        while (((err = close(receiver->net.controlSocket)) == -1) && (errno == EINTR));
        receiver->net.controlSocket = -1;
    }

    return 0;
}

static void ARSTREAM2_RtpReceiver_UpdateMonitoring(ARSTREAM2_RtpReceiver_t *receiver, uint64_t recvTimestamp, uint32_t rtpTimestamp, uint64_t ntpTimestamp, uint64_t ntpTimestampLocal, uint16_t seqNum, uint16_t markerBit, uint32_t bytes)
{
    ARSAL_Mutex_Lock(&(receiver->monitoringMutex));

    if (receiver->monitoringCount < ARSTREAM2_RTP_RECEIVER_MONITORING_MAX_POINTS)
    {
        receiver->monitoringCount++;
    }
    receiver->monitoringIndex = (receiver->monitoringIndex + 1) % ARSTREAM2_RTP_RECEIVER_MONITORING_MAX_POINTS;
    receiver->monitoringPoint[receiver->monitoringIndex].bytes = bytes;
    receiver->monitoringPoint[receiver->monitoringIndex].rtpTimestamp = rtpTimestamp;
    receiver->monitoringPoint[receiver->monitoringIndex].ntpTimestamp = ntpTimestamp;
    receiver->monitoringPoint[receiver->monitoringIndex].ntpTimestampLocal = ntpTimestampLocal;
    receiver->monitoringPoint[receiver->monitoringIndex].seqNum = seqNum;
    receiver->monitoringPoint[receiver->monitoringIndex].markerBit = markerBit;
    receiver->monitoringPoint[receiver->monitoringIndex].recvTimestamp = recvTimestamp;

    ARSAL_Mutex_Unlock(&(receiver->monitoringMutex));
}

static int ARSTREAM2_RtpReceiver_MuxRecvMmsg(ARSTREAM2_RtpReceiver_t *receiver, struct mmsghdr *msgvec, unsigned int vlen, int blocking)
{
#if BUILD_LIBMUX
    int ret = 0;
    unsigned int i, count;

    if ((!receiver) || (!receiver->mux.data) || (!msgvec))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Invalid pointer");
        return -1;
    }

    for (i = 0, count = 0; i < vlen; i++)
    {
        int ret2;
        struct pomp_buffer *buffer;
        const void *pb_data;
        size_t k, pb_len, left, offset;

        if ((blocking) && (i == 0))
        {
            ret2 = mux_queue_get_buf(receiver->mux.data, &buffer);
        }
        else
        {
            ret2 = mux_queue_try_get_buf(receiver->mux.data, &buffer);
        }
        if (ret2 != 0)
        {
            if (ret2 == -EPIPE)
            {
                ret = ret2;
            }
            break;
        }

        ret = pomp_buffer_get_cdata(buffer, &pb_data, &pb_len, NULL);
        if (ret != 0)
        {
            pomp_buffer_unref(buffer);
            break;
        }

        for (k = 0, offset = 0, left = pb_len; ((k < (size_t)msgvec[i].msg_hdr.msg_iovlen) && (left > 0)); k++)
        {
            size_t sz = (msgvec[i].msg_hdr.msg_iov[k].iov_len < left) ? msgvec[i].msg_hdr.msg_iov[k].iov_len : left;
            memcpy(msgvec[i].msg_hdr.msg_iov[k].iov_base, (uint8_t*)pb_data + offset, sz);
            offset += sz;
            left -= sz;
        }

        if (left != 0)
        {
            pomp_buffer_unref(buffer);
            ret = -E2BIG;
            break;
        }

        pomp_buffer_unref(buffer);
        msgvec[i].msg_len = (unsigned int)pb_len;
        count++;
    }

    return (ret == 0) ? (int)count : ret;
#else
    return -ENOSYS;
#endif
}


#ifndef HAS_MMSG
static int recvmmsg(int sockfd, struct mmsghdr *msgvec, unsigned int vlen,
                    unsigned int flags, struct timespec *timeout)
{
    unsigned int i, count;
    ssize_t ret;

    if (!msgvec)
    {
        return -1;
    }

    for (i = 0, count = 0; i < vlen; i++)
    {
        while (((ret = recvmsg(sockfd, &msgvec[i].msg_hdr, flags)) == -1) && (errno == EINTR));
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

static int ARSTREAM2_RtpReceiver_NetRecvMmsg(ARSTREAM2_RtpReceiver_t *receiver, struct mmsghdr *msgvec, unsigned int vlen, int blocking)
{
    //TODO: blocking does not work, but should not be needed
    int ret;

    if ((!receiver) || (!msgvec))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Invalid pointer");
        return -1;
    }

    while (((ret = recvmmsg(receiver->net.streamSocket, msgvec, vlen, 0, NULL)) == -1) && (errno == EINTR));
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Stream socket - recvmmsg error (%d): %s", errno, strerror(errno));
        return -1;
    }
    else
    {
        return ret;
    }
}

static int ARSTREAM2_RtpReceiver_MuxSendControlData(ARSTREAM2_RtpReceiver_t *receiver,
                                                    uint8_t *buffer,
                                                    int size)
{
#if BUILD_LIBMUX
    int ret;
    struct pomp_buffer *pbuffer;

    if (receiver == NULL ||
        receiver->mux.mux == NULL ||
        buffer == NULL)
        return -EINVAL;

    pbuffer = pomp_buffer_new_with_data(buffer,
                                        size);

    if (pbuffer == NULL)
        return -ENOMEM;

    ret = mux_encode(receiver->mux.mux,
                     MUX_ARSDK_CHANNEL_ID_STREAM_CONTROL,
                     pbuffer);

    pomp_buffer_unref(pbuffer);

    /* On success, return the number of bytes sent */
    if (ret == 0)
        ret = size;

    return ret;
#else
    return -ENOSYS;
#endif
}

static int ARSTREAM2_RtpReceiver_NetSendControlData(ARSTREAM2_RtpReceiver_t *receiver, uint8_t *buffer, int size)
{
    int ret;
    if (receiver->net.controlSendSin.sin_port == 0)
        return -EINVAL;
    while (((ret = sendto(receiver->net.controlSocket, buffer, size, 0, (struct sockaddr*)&receiver->net.controlSendSin, sizeof(receiver->net.controlSendSin))) == -1) && (errno == EINTR));
    if (ret < 0)
        ret = -errno;
    return ret;
}

static int ARSTREAM2_RtpReceiver_MuxReadControlData(ARSTREAM2_RtpReceiver_t *receiver,
                                                    uint8_t *buffer,
                                                    int size)
{
#if BUILD_LIBMUX
    int ret;
    struct pomp_buffer *pbuffer;
    const void *pb_data;
    size_t pb_len;

    if (receiver == NULL ||
        receiver->mux.control == NULL ||
        buffer == NULL)
        return -EINVAL;

    ret = mux_queue_try_get_buf(receiver->mux.control,
                                &pbuffer);

    if (ret != 0)
        return 0;

    ret = pomp_buffer_get_cdata(pbuffer,
                                &pb_data,
                                &pb_len,
                                NULL);

    if (ret != 0)
        goto unref_buffer;

    if (pb_len > (size_t)size) {
        ret = -E2BIG;
        goto unref_buffer;
    }

    memcpy(buffer, pb_data, pb_len);

    /* On success, return the number of bytes read */
    if (ret == 0)
        ret = pb_len;

unref_buffer:
    pomp_buffer_unref(pbuffer);
    return ret;
#else
    return -ENOSYS;
#endif
}

static int ARSTREAM2_RtpReceiver_NetReadControlData(ARSTREAM2_RtpReceiver_t *receiver, uint8_t *buffer, int size)
{
    ssize_t bytes;
    struct sockaddr_in srcaddr;
    socklen_t addrlen = sizeof(srcaddr);
    memset(&srcaddr, 0, sizeof(srcaddr));

    while (((bytes = recvfrom(receiver->net.controlSocket, buffer, size, 0, (struct sockaddr *)&srcaddr, &addrlen)) == -1) && (errno == EINTR));

    if ((bytes > 0) && (receiver->net.controlSendSin.sin_port == 0) && (srcaddr.sin_port > 0))
    {
        /* Save the control port if it was not known */
        receiver->net.controlSendSin.sin_port = srcaddr.sin_port;
        ARSAL_PRINT(ARSAL_PRINT_INFO, ARSTREAM2_RTP_RECEIVER_TAG, "Server control port: %d", ntohs(srcaddr.sin_port));
    }

    return (int)bytes;
}


void ARSTREAM2_RtpReceiver_Stop(ARSTREAM2_RtpReceiver_t *receiver)
{
    int ret;

    if (receiver != NULL)
    {
        if (receiver->useMux) {
            /* To stop the mux threads, we have to teardown the channels here.
               The second teardown, done at the end of the thread loops, will
               have no effect, but can be useful when using net backend */
            ret = receiver->ops.streamChannelTeardown(receiver);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to teardown the stream channel (error %d : %s).\n", -ret, strerror(-ret));
            }
            ret = receiver->ops.controlChannelTeardown(receiver);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to teardown the control channel (error %d : %s).", -ret, strerror(-ret));
            }
        }
    }
}


ARSTREAM2_RtpReceiver_t* ARSTREAM2_RtpReceiver_New(ARSTREAM2_RtpReceiver_Config_t *config,
                                                   ARSTREAM2_RtpReceiver_NetConfig_t *net_config,
                                                   ARSTREAM2_RtpReceiver_MuxConfig_t *mux_config,
                                                   eARSTREAM2_ERROR *error)
{
    ARSTREAM2_RtpReceiver_t *retReceiver = NULL;
    int monitoringMutexWasInit = 0;
    eARSTREAM2_ERROR internalError = ARSTREAM2_OK;

    /* ARGS Check */
    if (config == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "No config provided");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retReceiver;
    }
    if ((config->canonicalName == NULL) || (!strlen(config->canonicalName)))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Config: no canonical name provided");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retReceiver;
    }
    if ((!config->packetFifo) || (!config->packetFifoQueue))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "No packet FIFO provided");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retReceiver;
    }
    if (!config->auFifo)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "No access unit FIFO provided");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retReceiver;
    }
    if (!config->auCallback)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "No access unit callback function provided");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retReceiver;
    }

    if (net_config == NULL && mux_config == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "No net/mux config provided");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retReceiver;
    }

    if (net_config != NULL && mux_config != NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Both net/mux config provided. Cannot use both !");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retReceiver;
    }


    if (net_config != NULL)
    {
        if (((net_config->serverAddr == NULL) || (!strlen(net_config->serverAddr)))
                && ((net_config->mcastAddr == NULL) || (!strlen(net_config->mcastAddr))))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Config: no server address provided");
            SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
            return retReceiver;
        }
    }

    if (mux_config != NULL)
    {
#if BUILD_LIBMUX
        if (mux_config->mux == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Config: no mux context provided");
            SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
            return retReceiver;
        }
#else
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Config: library built without mux support");
        SET_WITH_CHECK(error, ARSTREAM2_ERROR_BAD_PARAMETERS);
        return retReceiver;
#endif
    }

    /* Alloc new receiver */
    retReceiver = malloc(sizeof(ARSTREAM2_RtpReceiver_t));
    if (retReceiver == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Allocation failed");
        internalError = ARSTREAM2_ERROR_ALLOC;
    }

    /* Initialize the receiver and copy parameters */
    if (internalError == ARSTREAM2_OK)
    {
        memset(retReceiver, 0, sizeof(ARSTREAM2_RtpReceiver_t));
        if (config->canonicalName)
        {
            retReceiver->canonicalName = strndup(config->canonicalName, 40);
        }
        if (config->friendlyName)
        {
            retReceiver->friendlyName = strndup(config->friendlyName, 40);
        }
        if (config->applicationName)
        {
            retReceiver->applicationName = strndup(config->applicationName, 40);
        }
        retReceiver->auFifo = config->auFifo;
        retReceiver->packetFifo = config->packetFifo;
        retReceiver->packetFifoQueue = config->packetFifoQueue;
        retReceiver->msgVecCount = retReceiver->packetFifo->bufferPoolSize;
        retReceiver->rtpStatsCallback = config->rtpStatsCallback;
        retReceiver->rtpStatsCallbackUserPtr = config->rtpStatsCallbackUserPtr;
        retReceiver->rtph264ReceiverContext.auCallback = config->auCallback;
        retReceiver->rtph264ReceiverContext.auCallbackUserPtr = config->auCallbackUserPtr;
        retReceiver->rtpReceiverContext.maxPacketSize = (config->maxPacketSize > 0) ? config->maxPacketSize - ARSTREAM2_RTP_TOTAL_HEADERS_SIZE : ARSTREAM2_RTP_MAX_PAYLOAD_SIZE;
        retReceiver->insertStartCodes = (config->insertStartCodes > 0) ? 1 : 0;
        retReceiver->generateReceiverReports = (config->generateReceiverReports > 0) ? 1 : 0;
        retReceiver->rtpReceiverContext.rtpClockRate = 90000;
        retReceiver->rtpReceiverContext.nominalDelay = 30000; //TODO
        retReceiver->rtpReceiverContext.previousExtSeqNum = -1;
        retReceiver->rtph264ReceiverContext.previousDepayloadExtSeqNum = -1;
        retReceiver->rtph264ReceiverContext.previousDepayloadExtRtpTimestamp = 0;
        retReceiver->rtph264ReceiverContext.startCode = (retReceiver->insertStartCodes) ? htonl(ARSTREAM2_H264_BYTE_STREAM_NALU_START_CODE) : 0;
        retReceiver->rtph264ReceiverContext.startCodeLength = (retReceiver->insertStartCodes) ? ARSTREAM2_H264_BYTE_STREAM_NALU_START_CODE_LENGTH : 0;
        retReceiver->rtcpReceiverContext.receiverSsrc = ARSTREAM2_RTP_RECEIVER_SSRC;
        retReceiver->rtcpReceiverContext.rtcpByteRate = ARSTREAM2_RTCP_RECEIVER_DEFAULT_BITRATE / 8;
        retReceiver->rtcpReceiverContext.rtpClockRate = 90000;
        retReceiver->rtcpReceiverContext.sdesItemCount = 0;
        if ((retReceiver->canonicalName) && (strlen(retReceiver->canonicalName)))
        {
            retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].type = ARSTREAM2_RTCP_SDES_CNAME_ITEM;
            snprintf(retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].value, 256, "%s", retReceiver->canonicalName);
            retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].sendTimeInterval = 0;
            retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].lastSendTime = 0;
            retReceiver->rtcpReceiverContext.sdesItemCount++;
        }
        if ((retReceiver->friendlyName) && (strlen(retReceiver->friendlyName)))
        {
            retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].type = ARSTREAM2_RTCP_SDES_NAME_ITEM;
            snprintf(retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].value, 256, "%s", retReceiver->friendlyName);
            retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].sendTimeInterval = 5000000;
            retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].lastSendTime = 0;
            retReceiver->rtcpReceiverContext.sdesItemCount++;
        }
        if ((retReceiver->applicationName) && (strlen(retReceiver->applicationName)))
        {
            retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].type = ARSTREAM2_RTCP_SDES_TOOL_ITEM;
            snprintf(retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].value, 256, "%s", retReceiver->applicationName);
            retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].sendTimeInterval = 5000000;
            retReceiver->rtcpReceiverContext.sdesItem[retReceiver->rtcpReceiverContext.sdesItemCount].lastSendTime = 0;
            retReceiver->rtcpReceiverContext.sdesItemCount++;
        }
        retReceiver->rtcpReceiverContext.videoStatsCtx.sendTimeInterval = config->videoStatsSendTimeInterval;
        retReceiver->rtcpReceiverContext.lossReportCtx.sendTimeInterval = config->lossReportSendTimeInterval;
        retReceiver->rtcpReceiverContext.djbReportCtx.sendTimeInterval = config->djbReportSendTimeInterval;
        retReceiver->rtcpReceiverContext.djbReportCtx.djbMetricsAvailable = 0;

        if (retReceiver->rtpReceiverContext.maxPacketSize < sizeof(ARSTREAM2_RTCP_ReceiverReport_t) + sizeof(ARSTREAM2_RTCP_ReceptionReportBlock_t))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Config: max packet size is too small to hold a receiver report");
            internalError = ARSTREAM2_ERROR_BAD_PARAMETERS;
        }

        if (net_config)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_RTP_RECEIVER_TAG, "New RTP Receiver using sockets");
            retReceiver->net.isMulticast = 0;
            retReceiver->net.streamSocket = -1;
            retReceiver->net.controlSocket = -1;

            if (net_config->mcastAddr)
            {
                retReceiver->net.isMulticast = 1;
                retReceiver->generateReceiverReports = 0; // Force not sending RTCP receiver reports in multicast mode
                retReceiver->net.serverAddr = strndup(net_config->mcastAddr, 16);
            }
            if (net_config->mcastIfaceAddr)
            {
                retReceiver->net.mcastIfaceAddr = strndup(net_config->mcastIfaceAddr, 16);
            }
            if ((!retReceiver->net.serverAddr) && (net_config->serverAddr))
            {
                retReceiver->net.serverAddr = strndup(net_config->serverAddr, 16);
            }
            retReceiver->net.serverStreamPort = net_config->serverStreamPort;
            retReceiver->net.serverControlPort = net_config->serverControlPort;
            retReceiver->net.clientStreamPort = (net_config->clientStreamPort > 0) ? net_config->clientStreamPort : ARSTREAM2_RTP_RECEIVER_DEFAULT_CLIENT_STREAM_PORT;
            retReceiver->net.clientControlPort = (net_config->clientControlPort > 0) ? net_config->clientControlPort : ARSTREAM2_RTP_RECEIVER_DEFAULT_CLIENT_CONTROL_PORT;
            retReceiver->net.classSelector = net_config->classSelector;

            retReceiver->useMux = 0;

            retReceiver->ops.streamChannelSetup = ARSTREAM2_RtpReceiver_StreamSocketSetup;
            retReceiver->ops.streamChannelRecvMmsg = ARSTREAM2_RtpReceiver_NetRecvMmsg;
            retReceiver->ops.streamChannelTeardown = ARSTREAM2_RtpReceiver_StreamSocketTeardown;

            retReceiver->ops.controlChannelSetup = ARSTREAM2_RtpReceiver_ControlSocketSetup;
            retReceiver->ops.controlChannelSend = ARSTREAM2_RtpReceiver_NetSendControlData;
            retReceiver->ops.controlChannelRead = ARSTREAM2_RtpReceiver_NetReadControlData;
            retReceiver->ops.controlChannelTeardown = ARSTREAM2_RtpReceiver_ControlSocketTeardown;
        }

#if BUILD_LIBMUX
        if (mux_config)
        {
            ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_RTP_RECEIVER_TAG, "New RTP Receiver using mux");
            retReceiver->mux.mux = mux_config->mux;
            mux_ref(retReceiver->mux.mux);
            retReceiver->useMux = 1;

            retReceiver->ops.streamChannelSetup = ARSTREAM2_RtpReceiver_StreamMuxSetup;
            retReceiver->ops.streamChannelRecvMmsg = ARSTREAM2_RtpReceiver_MuxRecvMmsg;
            retReceiver->ops.streamChannelTeardown = ARSTREAM2_RtpReceiver_StreamMuxTeardown;

            retReceiver->ops.controlChannelSetup = ARSTREAM2_RtpReceiver_ControlMuxSetup;
            retReceiver->ops.controlChannelSend = ARSTREAM2_RtpReceiver_MuxSendControlData;
            retReceiver->ops.controlChannelRead = ARSTREAM2_RtpReceiver_MuxReadControlData;
            retReceiver->ops.controlChannelTeardown = ARSTREAM2_RtpReceiver_ControlMuxTeardown;
        }
#endif

    }

    /* Setup internal mutexes/sems */
    if (internalError == ARSTREAM2_OK)
    {
        int mutexInitRet = ARSAL_Mutex_Init(&(retReceiver->monitoringMutex));
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
        if (retReceiver->msgVecCount > 0)
        {
            retReceiver->msgVec = malloc(retReceiver->msgVecCount * sizeof(struct mmsghdr));
            if (!retReceiver->msgVec)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "FIFO allocation failed (size %ld)", (long)retReceiver->msgVecCount * sizeof(struct mmsghdr));
                internalError = ARSTREAM2_ERROR_ALLOC;
            }
            else
            {
                memset(retReceiver->msgVec, 0, retReceiver->msgVecCount * sizeof(struct mmsghdr));
            }
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Invalid msgVecCount: %d", retReceiver->msgVecCount);
            internalError = ARSTREAM2_ERROR_BAD_PARAMETERS;
        }
    }

    /* Stream channel setup */
    if (internalError == ARSTREAM2_OK)
    {
        int ret = retReceiver->ops.streamChannelSetup(retReceiver);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to setup the stream channel (error %d)", ret);
            internalError = ARSTREAM2_ERROR_RESOURCE_UNAVAILABLE;
        }
    }

    /* Control channel setup */
    if (internalError == ARSTREAM2_OK)
    {
        int ret = retReceiver->ops.controlChannelSetup(retReceiver);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to setup the control channel (error %d)", ret);
            internalError = ARSTREAM2_ERROR_RESOURCE_UNAVAILABLE;
        }
    }

    /* RTCP message buffer */
    if (internalError == ARSTREAM2_OK)
    {
        retReceiver->rtcpMsgBuffer = malloc(retReceiver->rtpReceiverContext.maxPacketSize);
        if (retReceiver->rtcpMsgBuffer == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Memory allocation failed (%d)", retReceiver->rtpReceiverContext.maxPacketSize);
            internalError = ARSTREAM2_ERROR_ALLOC;
        }
    }

    if ((internalError != ARSTREAM2_OK) &&
        (retReceiver != NULL))
    {
        int ret = retReceiver->ops.streamChannelTeardown(retReceiver);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to teardown the stream channel (error %d : %s).\n", -ret, strerror(-ret));
        }
        ret = retReceiver->ops.controlChannelTeardown(retReceiver);
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to teardown the control channel (error %d : %s).\n", -ret, strerror(-ret));
        }
        if (monitoringMutexWasInit == 1)
        {
            ARSAL_Mutex_Destroy(&(retReceiver->monitoringMutex));
        }
        free(retReceiver->msgVec);
        free(retReceiver->rtcpMsgBuffer);
        free(retReceiver->canonicalName);
        free(retReceiver->friendlyName);
        free(retReceiver->applicationName);
        free(retReceiver->net.serverAddr);
        free(retReceiver->net.mcastIfaceAddr);

#if BUILD_LIBMUX
        if ((retReceiver) && (retReceiver->mux.mux))
        {
            mux_unref(retReceiver->mux.mux);
        }
#endif

        free(retReceiver);
        retReceiver = NULL;
    }

    SET_WITH_CHECK(error, internalError);
    return retReceiver;
}


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_Delete(ARSTREAM2_RtpReceiver_t **receiver)
{
    eARSTREAM2_ERROR retVal = ARSTREAM2_ERROR_BAD_PARAMETERS;
    if ((receiver != NULL) &&
        (*receiver != NULL))
    {
        int ret = (*receiver)->ops.streamChannelTeardown((*receiver));
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to teardown the stream channel (error %d : %s).\n", -ret, strerror(-ret));
        }
        ret = (*receiver)->ops.controlChannelTeardown((*receiver));
        if (ret != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to teardown the control channel (error %d : %s).\n", -ret, strerror(-ret));
        }
        ARSAL_Mutex_Destroy(&((*receiver)->monitoringMutex));
        free((*receiver)->msgVec);
        free((*receiver)->rtcpMsgBuffer);
        free((*receiver)->canonicalName);
        free((*receiver)->friendlyName);
        free((*receiver)->applicationName);
        free((*receiver)->net.serverAddr);
        free((*receiver)->net.mcastIfaceAddr);
        free((*receiver)->rtcpReceiverContext.lossReportCtx.receivedFlag);

#if BUILD_LIBMUX
        if ((*receiver)->mux.mux)
        {
            mux_unref((*receiver)->mux.mux);
        }
#endif

        free(*receiver);
        *receiver = NULL;
        retVal = ARSTREAM2_OK;
    }
    return retVal;
}


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_GetSelectParams(ARSTREAM2_RtpReceiver_t *receiver, fd_set **readSet, fd_set **writeSet, fd_set **exceptSet, int *maxFd, uint32_t *nextTimeout)
{
    eARSTREAM2_ERROR retVal = ARSTREAM2_OK;
    int _maxFd = 0;

    // Args check
    if (receiver == NULL)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if (!receiver->useMux)
    {
        _maxFd = -1;
        if (receiver->net.streamSocket > _maxFd) _maxFd = receiver->net.streamSocket;
        if (receiver->net.controlSocket > _maxFd) _maxFd = receiver->net.controlSocket;
        if (readSet)
        {
            FD_SET(receiver->net.streamSocket, *readSet);
            FD_SET(receiver->net.controlSocket, *readSet);
        }
        if (exceptSet)
        {
            FD_SET(receiver->net.streamSocket, *exceptSet);
            FD_SET(receiver->net.controlSocket, *exceptSet);
        }
    }
    else
    {
        if (readSet)
            *readSet = NULL;
        if (writeSet)
            *writeSet = NULL;
        if (exceptSet)
            *exceptSet = NULL;
    }

    if (maxFd) *maxFd = _maxFd;
    if (nextTimeout) *nextTimeout = (receiver->generateReceiverReports) ? ((receiver->nextRrDelay < ARSTREAM2_RTP_RECEIVER_TIMEOUT_US) ? receiver->nextRrDelay : ARSTREAM2_RTP_RECEIVER_TIMEOUT_US) : ARSTREAM2_RTP_RECEIVER_TIMEOUT_US;

    return retVal;
}


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_ProcessRtp(ARSTREAM2_RtpReceiver_t *receiver, int selectRet, fd_set *readSet, fd_set *writeSet, fd_set *exceptSet,
                                                  int *shouldStop, ARSTREAM2_RTP_PacketFifoQueue_t **resendQueue, uint32_t *resendTimeout, unsigned int resendCount)
{
    eARSTREAM2_ERROR retVal = ARSTREAM2_OK;
    struct timespec t1;
    uint64_t curTime;
    int ret;

    // Args check
    if (receiver == NULL)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if ((exceptSet) && (FD_ISSET(receiver->net.streamSocket, exceptSet)))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Exception on stream socket");
    }

    ARSAL_Time_GetTime(&t1);
    curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

    /* RTP packets reception */
    if ((!readSet) || ((selectRet >= 0) && (FD_ISSET(receiver->net.streamSocket, readSet))))
    {
        ret = ARSTREAM2_RTP_Receiver_PacketFifoFillMsgVec(receiver->packetFifo, receiver->msgVec, receiver->msgVecCount);
        if (ret < 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "ARSTREAM2_RTP_Receiver_PacketFifoFillMsgVec() failed (%d)", ret);
        }
        else if (ret > 0)
        {
            unsigned int msgCount = (unsigned  int)ret;

            ret = receiver->ops.streamChannelRecvMmsg(receiver, receiver->msgVec, msgCount, receiver->useMux);
            if (ret < 0)
            {
                if (ret == -EPIPE && receiver->useMux == 1)
                {
                    /* EPIPE with the mux means that we should no longer use the channel */
                    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_RTP_RECEIVER_TAG, "Got an EPIPE for stream channel, stopping thread");
                    if (shouldStop) *shouldStop = 1;
                }
                if (ret != -ETIMEDOUT)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to read data (%d)", ret);
                }
            }
            else if (ret > 0)
            {
                unsigned int recvMsgCount = (unsigned int)ret;

                ret = ARSTREAM2_RTP_Receiver_PacketFifoAddFromMsgVec(&receiver->rtpReceiverContext, receiver->packetFifo,
                                                                     receiver->packetFifoQueue, resendQueue, resendTimeout, resendCount,
                                                                     receiver->msgVec, recvMsgCount, curTime,
                                                                     &receiver->rtcpReceiverContext);
                if (ret < 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "ARSTREAM2_RTP_Receiver_PacketFifoAddFromMsgVec() failed (%d)", ret);
                }
            }
        }
    }

    /* RTP packets processing */
    ret = ARSTREAM2_RTPH264_Receiver_PacketFifoToAuFifo(&receiver->rtph264ReceiverContext, receiver->packetFifo,
                                                        receiver->packetFifoQueue, receiver->auFifo,
                                                        curTime, &receiver->rtcpReceiverContext);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "ARSTREAM2_RTPH264_Receiver_PacketFifoToAuFifo() failed (%d)", ret);
    }

    return retVal;
}


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_ProcessRtcp(ARSTREAM2_RtpReceiver_t *receiver, int selectRet, fd_set *readSet, fd_set *writeSet, fd_set *exceptSet, int *shouldStop)
{
    eARSTREAM2_ERROR retVal = ARSTREAM2_OK;
    struct timespec t1;
    uint64_t curTime;
    uint32_t rrDelay = 0;
    int ret;

    // Args check
    if (receiver == NULL)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    if ((exceptSet) && (FD_ISSET(receiver->net.controlSocket, exceptSet)))
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Exception on control socket");
    }

    ARSAL_Time_GetTime(&t1);
    curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

    /* RTCP sender reports */
    if ((!readSet) || ((selectRet >= 0) && (FD_ISSET(receiver->net.controlSocket, readSet))))
    {
        /* The control channel is ready for reading */
        ssize_t bytes = receiver->ops.controlChannelRead(receiver, receiver->rtcpMsgBuffer, receiver->rtpReceiverContext.maxPacketSize);
        if ((bytes < 0) && (errno != EAGAIN))
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Control channel - read error (%d): %s", errno, strerror(errno));
            if (bytes == -EPIPE && receiver->useMux == 1)
            {
                /* For the mux case, EPIPE means that the channel should not be used again */
                ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_RTP_RECEIVER_TAG, "Got an EPIPE for control channel, stopping thread");
                if (shouldStop) *shouldStop = 1;
            }
        }
        while (bytes > 0)
        {
            ret = ARSTREAM2_RTCP_Receiver_ProcessCompoundPacket(receiver->rtcpMsgBuffer, (unsigned int)bytes,
                                                                curTime, &receiver->rtcpReceiverContext);
            if (ret != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Failed to process compound RTCP packet (%d)", ret);
            }

            bytes = receiver->ops.controlChannelRead(receiver, receiver->rtcpMsgBuffer, receiver->rtpReceiverContext.maxPacketSize);
            if ((bytes < 0) && (errno != EAGAIN))
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Control channel - read error (%d): %s", errno, strerror(errno));
                if (bytes == -EPIPE && receiver->useMux == 1)
                {
                    /* For the mux case, EPIPE means that the channel should not be used again */
                    ARSAL_PRINT(ARSAL_PRINT_DEBUG, ARSTREAM2_RTP_RECEIVER_TAG, "Got an EPIPE for control channel, stopping thread");
                    if (shouldStop) *shouldStop = 1;
                }
            }
        }
    }

    /* RTCP receiver reports */
    if (receiver->generateReceiverReports)
    {
        rrDelay = (uint32_t)(curTime - receiver->rtcpReceiverContext.lastRtcpTimestamp);
        if ((rrDelay >= receiver->nextRrDelay) && (receiver->rtcpReceiverContext.prevSrNtpTimestamp != 0))
        {
            unsigned int size = 0;
            int generateVideoStats = 0;
            int generateLossReport = 0;
            int generateDjbReport = 0;

            if ((receiver->rtcpReceiverContext.videoStatsCtx.updatedSinceLastTime)
                    && (receiver->rtcpReceiverContext.videoStatsCtx.sendTimeInterval > 0)
                    && ((receiver->rtcpReceiverContext.videoStatsCtx.lastSendTime == 0)
                        || (curTime >= receiver->rtcpReceiverContext.videoStatsCtx.lastSendTime + receiver->rtcpReceiverContext.videoStatsCtx.sendTimeInterval)))
            {
                generateVideoStats = 1;
                receiver->rtcpReceiverContext.videoStatsCtx.lastSendTime = curTime;
                receiver->rtcpReceiverContext.videoStatsCtx.updatedSinceLastTime = 0;
            }

            if ((receiver->rtcpReceiverContext.lossReportCtx.sendTimeInterval > 0)
                    && ((receiver->rtcpReceiverContext.lossReportCtx.lastSendTime == 0)
                        || (curTime >= receiver->rtcpReceiverContext.lossReportCtx.lastSendTime + receiver->rtcpReceiverContext.lossReportCtx.sendTimeInterval)))
            {
                generateLossReport = 1;
                receiver->rtcpReceiverContext.lossReportCtx.lastSendTime = curTime;
            }

            if ((receiver->rtcpReceiverContext.djbReportCtx.sendTimeInterval > 0)
                    && ((receiver->rtcpReceiverContext.djbReportCtx.lastSendTime == 0)
                        || (curTime >= receiver->rtcpReceiverContext.djbReportCtx.lastSendTime + receiver->rtcpReceiverContext.djbReportCtx.sendTimeInterval)))
            {
                generateDjbReport = 1;
                receiver->rtcpReceiverContext.djbReportCtx.lastSendTime = curTime;
            }

            ret = ARSTREAM2_RTCP_Receiver_GenerateCompoundPacket(receiver->rtcpMsgBuffer, receiver->rtpReceiverContext.maxPacketSize, curTime,
                                                                 1, 1, 1, generateVideoStats, generateLossReport, generateDjbReport,
                                                                 &receiver->rtcpReceiverContext, &size);
            if ((ret == 0) && (size > 0))
            {
                receiver->rtcpDropStatsTotalPackets++;
                ssize_t bytes = receiver->ops.controlChannelSend(receiver, receiver->rtcpMsgBuffer, size);
                if (bytes < 0)
                {
                    if (errno == EAGAIN)
                    {
                        /* Log drops once in a while */
                        receiver->rtcpDropCount++;
                        if (receiver->rtcpDropLogStartTime)
                        {
                            if (curTime >= receiver->rtcpDropLogStartTime + (uint64_t)ARSTREAM2_RTP_RECEIVER_RTCP_DROP_LOG_INTERVAL * 1000000)
                            {
                                ARSAL_PRINT(ARSAL_PRINT_WARNING, ARSTREAM2_RTP_RECEIVER_TAG, "Dropped %d RTCP packets out of %d (%.1f%%) on socket buffer full in last %.1f seconds",
                                            receiver->rtcpDropCount, receiver->rtcpDropStatsTotalPackets, (float)receiver->rtcpDropCount * 100. / (float)receiver->rtcpDropStatsTotalPackets,
                                            (float)(curTime - receiver->rtcpDropLogStartTime) / 1000000.);
                                receiver->rtcpDropCount = 0;
                                receiver->rtcpDropStatsTotalPackets = 0;
                                receiver->rtcpDropLogStartTime = 0;
                            }
                        }
                        else
                        {
                            receiver->rtcpDropLogStartTime = curTime;
                        }
                    }
                    else
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "Control channel - send error (%d): %s", errno, strerror(errno));
                    }
                }

                if (receiver->rtpStatsCallback != NULL)
                {
                    ARSTREAM2_RTP_RtpStats_t rtpStats;

                    memset(&rtpStats, 0, sizeof(ARSTREAM2_RTP_RtpStats_t));
                    if (receiver->rtcpReceiverContext.lastSrReceptionTimestamp != 0)
                    {
                        rtpStats.senderReport.timestamp = receiver->rtcpReceiverContext.lastSrReceptionTimestamp;
                        rtpStats.senderReport.lastInterval = receiver->rtcpReceiverContext.lastSrInterval;
                        rtpStats.senderReport.intervalPacketCount = receiver->rtcpReceiverContext.srIntervalPacketCount;
                        rtpStats.senderReport.intervalByteCount = receiver->rtcpReceiverContext.srIntervalByteCount;
                    }
                    if (receiver->rtcpReceiverContext.lastRrTimestamp != 0)
                    {
                        rtpStats.receiverReport.timestamp = receiver->rtcpReceiverContext.lastRrTimestamp;
                        rtpStats.receiverReport.roundTripDelay = 0; // unable to compute on the receiver side
                        rtpStats.receiverReport.interarrivalJitter = receiver->rtcpReceiverContext.lastRrInterarrivalJitter;
                        rtpStats.receiverReport.receiverLostCount = receiver->rtcpReceiverContext.lastRrPacketsLost;
                        rtpStats.receiverReport.receiverFractionLost = receiver->rtcpReceiverContext.lastRrFractionLost;
                        rtpStats.receiverReport.receiverExtHighestSeqNum = receiver->rtcpReceiverContext.lastRrExtHighestSeqNum;
                    }
                    if ((generateLossReport) && (receiver->rtcpReceiverContext.lossReportCtx.lastSendTime != 0))
                    {
                        rtpStats.lossReport.timestamp = receiver->rtcpReceiverContext.lossReportCtx.lastSendTime;
                        rtpStats.lossReport.startSeqNum = (uint16_t)(receiver->rtcpReceiverContext.lossReportCtx.startSeqNum & 0xFFFF);
                        rtpStats.lossReport.endSeqNum = (uint16_t)(receiver->rtcpReceiverContext.lossReportCtx.endSeqNum & 0xFFFF);
                        rtpStats.lossReport.receivedFlag = receiver->rtcpReceiverContext.lossReportCtx.receivedFlag;
                    }
                    if ((receiver->rtcpReceiverContext.djbReportCtx.djbMetricsAvailable) && (receiver->rtcpReceiverContext.djbReportCtx.lastSendTime != 0))
                    {
                        rtpStats.djbMetricsReport.timestamp = receiver->rtcpReceiverContext.djbReportCtx.lastSendTime;
                        rtpStats.djbMetricsReport.djbNominal = (receiver->rtcpReceiverContext.djbReportCtx.djbNominal <= 0xFFFD) ?
                                                               receiver->rtcpReceiverContext.djbReportCtx.djbNominal : 0xFFFE;
                        rtpStats.djbMetricsReport.djbMax = (receiver->rtcpReceiverContext.djbReportCtx.djbMax <= 0xFFFD) ?
                                                           receiver->rtcpReceiverContext.djbReportCtx.djbMax : 0xFFFE;
                        rtpStats.djbMetricsReport.djbHighWatermark = (receiver->rtcpReceiverContext.djbReportCtx.djbHighWatermark <= 0xFFFD) ?
                                                                     receiver->rtcpReceiverContext.djbReportCtx.djbHighWatermark : 0xFFFE;
                        rtpStats.djbMetricsReport.djbLowWatermark = (receiver->rtcpReceiverContext.djbReportCtx.djbLowWatermark <= 0xFFFD) ?
                                                                    receiver->rtcpReceiverContext.djbReportCtx.djbLowWatermark : 0xFFFE;
                    }
                    rtpStats.clockDelta.peerClockDelta = receiver->rtcpReceiverContext.clockDeltaCtx.clockDeltaAvg;
                    rtpStats.clockDelta.roundTripDelay = (uint32_t)receiver->rtcpReceiverContext.clockDeltaCtx.rtDelayAvg;
                    rtpStats.clockDelta.peer2meDelay = (uint32_t)receiver->rtcpReceiverContext.clockDeltaCtx.p2mDelayAvg;
                    rtpStats.clockDelta.me2peerDelay = (uint32_t)receiver->rtcpReceiverContext.clockDeltaCtx.m2pDelayAvg;

                    /* Call the RTP stats callback function */
                    receiver->rtpStatsCallback(&rtpStats, receiver->rtpStatsCallbackUserPtr);
                }
            }

            if (generateLossReport)
            {
                int err = ARSTREAM2_RTCP_LossReportReset(&receiver->rtcpReceiverContext.lossReportCtx);
                if (err != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, ARSTREAM2_RTP_RECEIVER_TAG, "ARSTREAM2_RTCP_LossReportReset() failed (%d)", err);
                }
            }

            receiver->rtcpReceiverContext.lastRtcpTimestamp = curTime;
            rrDelay = 0;
            receiver->nextRrDelay = (size + ARSTREAM2_RTP_UDP_HEADER_SIZE + ARSTREAM2_RTP_IP_HEADER_SIZE) * 1000000 / receiver->rtcpReceiverContext.rtcpByteRate;
            if (receiver->nextRrDelay < ARSTREAM2_RTCP_RECEIVER_MIN_PACKET_TIME_INTERVAL) receiver->nextRrDelay = ARSTREAM2_RTCP_RECEIVER_MIN_PACKET_TIME_INTERVAL;
        }
    }

    return retVal;
}


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_ProcessEnd(ARSTREAM2_RtpReceiver_t *receiver, int queueOnly)
{
    eARSTREAM2_ERROR retVal = ARSTREAM2_OK;

    // Args check
    if (receiver == NULL)
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    /* flush the packet FIFO */
    if (queueOnly)
        ARSTREAM2_RTP_Receiver_PacketFifoFlushQueue(receiver->packetFifo, receiver->packetFifoQueue);
    else
        ARSTREAM2_RTP_Receiver_PacketFifoFlush(receiver->packetFifo);

    return retVal;
}


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_UpdateVideoStats(ARSTREAM2_RtpReceiver_t *receiver, const ARSTREAM2_H264_VideoStats_t *videoStats)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;

    if ((receiver == NULL) || (videoStats == 0))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    memcpy(&receiver->rtcpReceiverContext.videoStatsCtx.videoStats, videoStats, sizeof(ARSTREAM2_H264_VideoStats_t));
    receiver->rtcpReceiverContext.videoStatsCtx.updatedSinceLastTime = 1;

    return ret;
}


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_GetSdesItem(ARSTREAM2_RtpReceiver_t *receiver, uint8_t type, const char *prefix, char **value, uint32_t *sendInterval)
{
    int k, found;

    if ((receiver == NULL) || (value == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((type == ARSTREAM2_RTCP_SDES_PRIV_ITEM) && (prefix == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    for (k = 0, found = 0; k < receiver->rtcpReceiverContext.sdesItemCount; k++)
    {
        if (type == receiver->rtcpReceiverContext.sdesItem[k].type)
        {
            if (type == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
            {
                if (!strncmp(prefix, receiver->rtcpReceiverContext.sdesItem[k].prefix, 256))
                {
                    *value = receiver->rtcpReceiverContext.sdesItem[k].value;
                    if (sendInterval) *sendInterval = receiver->rtcpReceiverContext.sdesItem[k].sendTimeInterval;
                    found = 1;
                    break;
                }
            }
            else
            {
                *value = receiver->rtcpReceiverContext.sdesItem[k].value;
                if (sendInterval) *sendInterval = receiver->rtcpReceiverContext.sdesItem[k].sendTimeInterval;
                found = 1;
                break;
            }
        }
    }

    return (found) ? ARSTREAM2_OK : ARSTREAM2_ERROR_NOT_FOUND;
}


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_SetSdesItem(ARSTREAM2_RtpReceiver_t *receiver, uint8_t type, const char *prefix, const char *value, uint32_t sendInterval)
{
    int k, found;

    if ((receiver == NULL) || (value == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((type == ARSTREAM2_RTCP_SDES_PRIV_ITEM) && (prefix == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    for (k = 0, found = 0; k < receiver->rtcpReceiverContext.sdesItemCount; k++)
    {
        if (type == receiver->rtcpReceiverContext.sdesItem[k].type)
        {
            if (type == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
            {
                if (!strncmp(prefix, receiver->rtcpReceiverContext.sdesItem[k].prefix, 256))
                {
                    snprintf(receiver->rtcpReceiverContext.sdesItem[k].value, 256, "%s", value);
                    receiver->rtcpReceiverContext.sdesItem[k].sendTimeInterval = sendInterval;
                    receiver->rtcpReceiverContext.sdesItem[k].lastSendTime = 0;
                    found = 1;
                    break;
                }
            }
            else
            {
                snprintf(receiver->rtcpReceiverContext.sdesItem[k].value, 256, "%s", value);
                receiver->rtcpReceiverContext.sdesItem[k].sendTimeInterval = sendInterval;
                receiver->rtcpReceiverContext.sdesItem[k].lastSendTime = 0;
                found = 1;
                break;
            }
        }
    }

    if (!found)
    {
        if (receiver->rtcpReceiverContext.sdesItemCount >= ARSTREAM2_RTCP_SDES_ITEM_MAX_COUNT)
        {
            return ARSTREAM2_ERROR_ALLOC;
        }
        else
        {
            k = receiver->rtcpReceiverContext.sdesItemCount;
            receiver->rtcpReceiverContext.sdesItem[k].type = type;
            snprintf(receiver->rtcpReceiverContext.sdesItem[k].value, 256, "%s", value);
            if (type == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
            {
                snprintf(receiver->rtcpReceiverContext.sdesItem[k].prefix, 256, "%s", prefix);
            }
            receiver->rtcpReceiverContext.sdesItem[k].sendTimeInterval = sendInterval;
            receiver->rtcpReceiverContext.sdesItem[k].lastSendTime = 0;
            receiver->rtcpReceiverContext.sdesItemCount++;
        }
    }

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_GetPeerSdesItem(ARSTREAM2_RtpReceiver_t *receiver, uint8_t type, const char *prefix, char **value)
{
    int k, found;

    if ((receiver == NULL) || (value == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }
    if ((type == ARSTREAM2_RTCP_SDES_PRIV_ITEM) && (prefix == NULL))
    {
        return ARSTREAM2_ERROR_BAD_PARAMETERS;
    }

    for (k = 0, found = 0; k < receiver->rtcpReceiverContext.peerSdesItemCount; k++)
    {
        if (type == receiver->rtcpReceiverContext.peerSdesItem[k].type)
        {
            if (type == ARSTREAM2_RTCP_SDES_PRIV_ITEM)
            {
                if (!strncmp(prefix, receiver->rtcpReceiverContext.peerSdesItem[k].prefix, 256))
                {
                    *value = receiver->rtcpReceiverContext.peerSdesItem[k].value;
                    found = 1;
                    break;
                }
            }
            else
            {
                *value = receiver->rtcpReceiverContext.peerSdesItem[k].value;
                found = 1;
                break;
            }
        }
    }

    return (found) ? ARSTREAM2_OK : ARSTREAM2_ERROR_NOT_FOUND;
}


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_GetMonitoring(ARSTREAM2_RtpReceiver_t *receiver, uint64_t startTime, uint32_t timeIntervalUs, uint32_t *realTimeIntervalUs, uint32_t *receptionTimeJitter,
                                                     uint32_t *bytesReceived, uint32_t *meanPacketSize, uint32_t *packetSizeStdDev, uint32_t *packetsReceived, uint32_t *packetsMissed)
{
    eARSTREAM2_ERROR ret = ARSTREAM2_OK;
    uint64_t endTime, curTime, previousTime, auTimestamp, receptionTimeSum = 0, receptionTimeVarSum = 0, packetSizeVarSum = 0;
    uint32_t bytes, bytesSum = 0, _meanPacketSize = 0, receptionTime = 0, meanReceptionTime = 0, _receptionTimeJitter = 0, _packetSizeStdDev = 0;
    int currentSeqNum, previousSeqNum = -1, seqNumDelta, gapsInSeqNum = 0;
    int points = 0, usefulPoints = 0, idx, i, firstUsefulIdx = -1;

    if ((receiver == NULL) || (timeIntervalUs == 0))
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

    ARSAL_Mutex_Lock(&(receiver->monitoringMutex));

    if (receiver->monitoringCount > 0)
    {
        idx = receiver->monitoringIndex;
        previousTime = startTime;

        while (points < receiver->monitoringCount)
        {
            curTime = receiver->monitoringPoint[idx].recvTimestamp;
            if (curTime > startTime)
            {
                points++;
                idx = (idx - 1 >= 0) ? idx - 1 : ARSTREAM2_RTP_RECEIVER_MONITORING_MAX_POINTS - 1;
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
            idx = (idx - 1 >= 0) ? idx - 1 : ARSTREAM2_RTP_RECEIVER_MONITORING_MAX_POINTS - 1;
            curTime = receiver->monitoringPoint[idx].recvTimestamp;
            bytes = receiver->monitoringPoint[idx].bytes;
            bytesSum += bytes;
            auTimestamp = receiver->monitoringPoint[idx].ntpTimestampLocal;
            receptionTime = curTime - auTimestamp;
            receptionTimeSum += receptionTime;
            currentSeqNum = receiver->monitoringPoint[idx].seqNum;
            seqNumDelta = (previousSeqNum != -1) ? (previousSeqNum - currentSeqNum) : 1;
            if (seqNumDelta < -32768) seqNumDelta += 65536; /* handle seqNum 16 bits loopback */
            gapsInSeqNum += seqNumDelta - 1;
            previousSeqNum = currentSeqNum;
            previousTime = curTime;
            usefulPoints++;
            points++;
            idx = (idx - 1 >= 0) ? idx - 1 : ARSTREAM2_RTP_RECEIVER_MONITORING_MAX_POINTS - 1;
        }

        endTime = previousTime;
        _meanPacketSize = (usefulPoints) ? (bytesSum / usefulPoints) : 0;
        meanReceptionTime = (usefulPoints) ? (uint32_t)(receptionTimeSum / usefulPoints) : 0;

        if ((receptionTimeJitter) || (packetSizeStdDev))
        {
            for (i = 0, idx = firstUsefulIdx; i < usefulPoints; i++)
            {
                idx = (idx - 1 >= 0) ? idx - 1 : ARSTREAM2_RTP_RECEIVER_MONITORING_MAX_POINTS - 1;
                curTime = receiver->monitoringPoint[idx].recvTimestamp;
                bytes = receiver->monitoringPoint[idx].bytes;
                auTimestamp = receiver->monitoringPoint[idx].ntpTimestampLocal;
                receptionTime = curTime - auTimestamp;
                packetSizeVarSum += ((bytes - _meanPacketSize) * (bytes - _meanPacketSize));
                receptionTimeVarSum += ((receptionTime - meanReceptionTime) * (receptionTime - meanReceptionTime));
            }
            _receptionTimeJitter = (usefulPoints) ? (uint32_t)(sqrt((double)receptionTimeVarSum / usefulPoints)) : 0;
            _packetSizeStdDev = (usefulPoints) ? (uint32_t)(sqrt((double)packetSizeVarSum / usefulPoints)) : 0;
        }
    }

    ARSAL_Mutex_Unlock(&(receiver->monitoringMutex));

    if (realTimeIntervalUs)
    {
        *realTimeIntervalUs = (startTime - endTime);
    }
    if (receptionTimeJitter)
    {
        *receptionTimeJitter = _receptionTimeJitter;
    }
    if (bytesReceived)
    {
        *bytesReceived = bytesSum;
    }
    if (meanPacketSize)
    {
        *meanPacketSize = _meanPacketSize;
    }
    if (packetSizeStdDev)
    {
        *packetSizeStdDev = _packetSizeStdDev;
    }
    if (packetsReceived)
    {
        *packetsReceived = usefulPoints;
    }
    if (packetsMissed)
    {
        *packetsMissed = gapsInSeqNum;
    }

    return ret;
}
