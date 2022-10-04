/**
 * @file arstream2_rtp_receiver.h
 * @brief Parrot Streaming Library - RTP Receiver
 * @date 04/16/2015
 * @author aurelien.barre@parrot.com
 */

#ifndef _ARSTREAM2_RTP_RECEIVER_H_
#define _ARSTREAM2_RTP_RECEIVER_H_

#include <config.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#define __USE_GNU
#include <sys/socket.h>
#undef __USE_GNU
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>

#include <libARStream2/arstream2_error.h>
#include "arstream2_rtp_sender.h"
#include "arstream2_rtp.h"
#include "arstream2_rtp_h264.h"
#include "arstream2_rtcp.h"
#include "arstream2_h264.h"

#include <libARSAL/ARSAL_Print.h>
#include <libARSAL/ARSAL_Mutex.h>

#if BUILD_LIBMUX
#include <libmux.h>
#include <libmux-arsdk.h>
#include <libpomp.h>
#endif


#define ARSTREAM2_RTP_RECEIVER_DEFAULT_CLIENT_STREAM_PORT     (55004)
#define ARSTREAM2_RTP_RECEIVER_DEFAULT_CLIENT_CONTROL_PORT    (55005)

#define ARSTREAM2_RTP_RECEIVER_TIMEOUT_US (100 * 1000)
#define ARSTREAM2_RTP_RECEIVER_MUX_TIMEOUT_US (10 * 1000)

#define ARSTREAM2_RTP_RECEIVER_STREAM_DATAREAD_TIMEOUT_MS (500)
#define ARSTREAM2_RTP_RECEIVER_CONTROL_DATAREAD_TIMEOUT_MS (500)

#define ARSTREAM2_RTP_RECEIVER_MONITORING_MAX_POINTS (2048)

#define ARSTREAM2_RTP_RECEIVER_RTCP_DROP_LOG_INTERVAL (10)


/**
 * @brief Callback function for RTP stats
 * This callback function is called when an RTCP compound packet has been sent.
 *
 * @param[in] rtpStats Pointer to RTP stats data
 * @param[in] userPtr Global callback user pointer
 */
typedef void (*ARSTREAM2_RtpReceiver_RtpStatsCallback_t) (const ARSTREAM2_RTP_RtpStats_t *rtpStats, void *userPtr);


/**
 * @brief RtpReceiver net configuration parameters
 */
typedef struct ARSTREAM2_RtpReceiver_NetConfig_t
{
    const char *serverAddr;                         /**< Server address */
    const char *mcastAddr;                          /**< Multicast receive address (optional, NULL for no multicast) */
    const char *mcastIfaceAddr;                     /**< Multicast input interface address (required if mcastAddr is not NULL) */
    int serverStreamPort;                           /**< Server stream port, @see ARSTREAM2_RTP_SENDER_DEFAULT_SERVER_STREAM_PORT */
    int serverControlPort;                          /**< Server control port, @see ARSTREAM2_RTP_SENDER_DEFAULT_SERVER_CONTROL_PORT */
    int clientStreamPort;                           /**< Client stream port */
    int clientControlPort;                          /**< Client control port */
    eARSAL_SOCKET_CLASS_SELECTOR classSelector;     /**< Type of Service class selector */
} ARSTREAM2_RtpReceiver_NetConfig_t;

// Forward declaration of the mux_ctx structure
struct mux_ctx;

/**
 * @brief RtpReceiver mux configuration parameters
 */
typedef struct ARSTREAM2_RtpReceiver_MuxConfig_t
{
    struct mux_ctx *mux;                            /**< libmux context */
} ARSTREAM2_RtpReceiver_MuxConfig_t;
/**
 * @brief RtpReceiver configuration parameters
 */
typedef struct ARSTREAM2_RtpReceiver_Config_t
{
    const char *canonicalName;                      /**< RTP participant canonical name (CNAME SDES item) */
    const char *friendlyName;                       /**< RTP participant friendly name (NAME SDES item) (optional, can be NULL) */
    const char *applicationName;                    /**< RTP participant application name (TOOL SDES item) (optional, can be NULL) */
    ARSTREAM2_RTP_PacketFifo_t *packetFifo;         /**< User-provided packet FIFO */
    ARSTREAM2_RTP_PacketFifoQueue_t *packetFifoQueue;  /**< User-provided packet FIFO queue */
    ARSTREAM2_H264_AuFifo_t *auFifo;                /**< User-provided access unit FIFO */
    ARSTREAM2_H264_ReceiverAuCallback_t auCallback;
    void *auCallbackUserPtr;
    ARSTREAM2_RtpReceiver_RtpStatsCallback_t rtpStatsCallback;   /**< RTP stats callback function (optional, can be NULL) */
    void *rtpStatsCallbackUserPtr;                  /**< RTP stats callback function user pointer (optional, can be NULL) */
    int maxPacketSize;                              /**< Maximum network packet size in bytes (should be provided by the server, if 0 the maximum UDP packet size is used) */
    int insertStartCodes;                           /**< Boolean-like (0-1) flag: if active insert a start code prefix before NAL units */
    int generateReceiverReports;                    /**< Boolean-like (0-1) flag: if active generate RTCP receiver reports */
    uint32_t videoStatsSendTimeInterval;            /**< Time interval for sending video stats in compound RTCP packets (optional, can be null) */
    uint32_t lossReportSendTimeInterval;            /**< Time interval for sending loss reports in compound RTCP packets (optional, can be null) */
    uint32_t djbReportSendTimeInterval;             /**< Time interval for sending de-jitter buffer metrics reports in compound RTCP packets (optional, can be null) */
} ARSTREAM2_RtpReceiver_Config_t;


/**
 * @brief An RtpReceiver instance to allow receiving H.264 video over a network
 */
typedef struct ARSTREAM2_RtpReceiver_t ARSTREAM2_RtpReceiver_t;


typedef struct ARSTREAM2_RtpReceiver_MonitoringPoint_s {
    uint64_t recvTimestamp;
    uint64_t ntpTimestamp;
    uint64_t ntpTimestampLocal;
    uint32_t rtpTimestamp;
    uint16_t seqNum;
    uint16_t markerBit;
    uint32_t bytes;
} ARSTREAM2_RtpReceiver_MonitoringPoint_t;


struct ARSTREAM2_RtpReceiver_NetInfos_t {
    char *serverAddr;
    char *mcastIfaceAddr;
    int serverStreamPort;
    int serverControlPort;
    int clientStreamPort;
    int clientControlPort;
    int classSelector;

    /* Sockets */
    int isMulticast;
    int streamSocket;
    int controlSocket;
    struct sockaddr_in controlSendSin;
};

struct ARSTREAM2_RtpReceiver_MuxInfos_t {
    struct mux_ctx *mux;
    struct mux_queue *control;
    struct mux_queue *data;
};

struct ARSTREAM2_RtpReceiver_Ops_t {
    /* Stream channel */
    int (*streamChannelSetup)(ARSTREAM2_RtpReceiver_t *);
    int (*streamChannelTeardown)(ARSTREAM2_RtpReceiver_t *);
    int (*streamChannelRecvMmsg)(ARSTREAM2_RtpReceiver_t *,
                                 struct mmsghdr *,
                                 unsigned int,
                                 int);


    /* Control channel */
    int (*controlChannelSetup)(ARSTREAM2_RtpReceiver_t *);
    int (*controlChannelTeardown)(ARSTREAM2_RtpReceiver_t *);
    int (*controlChannelSend)(ARSTREAM2_RtpReceiver_t *,
                              uint8_t *,
                              int);
    int (*controlChannelRead)(ARSTREAM2_RtpReceiver_t *,
                              uint8_t *,
                              int);
};

struct ARSTREAM2_RtpReceiver_t {
    /* Configuration on New */
    int useMux;
    struct ARSTREAM2_RtpReceiver_NetInfos_t net;
    struct ARSTREAM2_RtpReceiver_MuxInfos_t mux;
    struct ARSTREAM2_RtpReceiver_Ops_t ops;

    /* Process context */
    ARSTREAM2_RTP_ReceiverContext_t rtpReceiverContext;
    ARSTREAM2_RTPH264_ReceiverContext_t rtph264ReceiverContext;
    ARSTREAM2_RTCP_ReceiverContext_t rtcpReceiverContext;
    ARSTREAM2_RtpReceiver_RtpStatsCallback_t rtpStatsCallback;
    void *rtpStatsCallbackUserPtr;

    char *canonicalName;
    char *friendlyName;
    char *applicationName;
    int insertStartCodes;
    int generateReceiverReports;
    uint8_t *rtcpMsgBuffer;
    uint32_t nextRrDelay;

    /* Packet and access unit FIFO */
    ARSTREAM2_H264_AuFifo_t *auFifo;
    ARSTREAM2_RTP_PacketFifo_t *packetFifo;
    ARSTREAM2_RTP_PacketFifoQueue_t *packetFifoQueue;
    struct mmsghdr *msgVec;
    unsigned int msgVecCount;

    /* Monitoring */
    ARSAL_Mutex_t monitoringMutex;
    int monitoringCount;
    int monitoringIndex;
    ARSTREAM2_RtpReceiver_MonitoringPoint_t monitoringPoint[ARSTREAM2_RTP_RECEIVER_MONITORING_MAX_POINTS];

    unsigned int rtcpDropCount;
    unsigned int rtcpDropStatsTotalPackets;
    uint64_t rtcpDropLogStartTime;
};


/**
 * @brief Creates a new RtpReceiver
 * @warning This function allocates memory. The receiver must be deleted by a call to ARSTREAM2_RtpReceiver_Delete()
 *
 * @param[in] config Pointer to a configuration parameters structure
 * @param[out] error Optionnal pointer to an eARSTREAM2_ERROR to hold any error information
 *
 * @return A pointer to the new ARSTREAM2_RtpReceiver_t, or NULL if an error occured
 *
 * @see ARSTREAM2_RtpReceiver_Stop()
 * @see ARSTREAM2_RtpReceiver_Delete()
 */
ARSTREAM2_RtpReceiver_t* ARSTREAM2_RtpReceiver_New(ARSTREAM2_RtpReceiver_Config_t *config,
                                                   ARSTREAM2_RtpReceiver_NetConfig_t *net_config,
                                                   ARSTREAM2_RtpReceiver_MuxConfig_t *mux_config,
                                                   eARSTREAM2_ERROR *error);


/**
 * @brief Stops a running RtpReceiver
 * @warning Once stopped, a receiver cannot be restarted
 *
 * @param[in] receiver The receiver instance
 *
 * @note Calling this function multiple times has no effect
 */
void ARSTREAM2_RtpReceiver_Stop(ARSTREAM2_RtpReceiver_t *receiver);


/**
 * @brief Deletes an RtpReceiver
 * @warning This function should NOT be called on a running receiver
 *
 * @param receiver Pointer to the ARSTREAM2_RtpReceiver_t* to delete
 *
 * @return ARSTREAM2_OK if the receiver was deleted
 * @return ARSTREAM2_ERROR_BUSY if the receiver is still busy and can not be stopped now (probably because ARSTREAM2_RtpReceiver_Stop() has not been called yet)
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if receiver does not point to a valid ARSTREAM2_RtpReceiver_t
 *
 * @note The function uses a double pointer, so it can set *receiver to NULL after freeing it
 */
eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_Delete(ARSTREAM2_RtpReceiver_t **receiver);


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_GetSelectParams(ARSTREAM2_RtpReceiver_t *receiver, fd_set **readSet, fd_set **writeSet, fd_set **exceptSet, int *maxFd, uint32_t *nextTimeout);


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_ProcessRtp(ARSTREAM2_RtpReceiver_t *receiver, int selectRet, fd_set *readSet, fd_set *writeSet, fd_set *exceptSet,
                                                  int *shouldStop, ARSTREAM2_RTP_PacketFifoQueue_t **resendQueue, uint32_t *resendTimeout, unsigned int resendCount);


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_ProcessRtcp(ARSTREAM2_RtpReceiver_t *receiver, int selectRet, fd_set *readSet, fd_set *writeSet, fd_set *exceptSet, int *shouldStop);


eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_ProcessEnd(ARSTREAM2_RtpReceiver_t *receiver, int queueOnly);


/**
 * @brief Update the video stats
 * The video stats are provided by the upper layer to be sent in RTCP compound packets.
 *
 * @param[in] receiver The receiver instance
 * @param[in] videoStats Video stats data
 *
 * @return ARSTREAM2_OK if no error occured.
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if either the receiver or videoStats is invalid.
 */
eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_UpdateVideoStats(ARSTREAM2_RtpReceiver_t *receiver, const ARSTREAM2_H264_VideoStats_t *videoStats);


/**
 * @brief Get a RTCP Source Description item
 *
 * @param[in] receiver The receiver instance
 * @param[in] type SDES item type
 * @param[in] prefix SDES item prefix (only for private extension type)
 * @param[in] value Pointer to the SDES item value
 * @param[in] sendInterval Pointer to the SDES item minimum send interval in microseconds
 *
 * @return ARSTREAM2_OK if no error happened
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the receiver or value pointers are invalid
 * @return ARSTREAM2_ERROR_NOT_FOUND it the item has not been found
 */
eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_GetSdesItem(ARSTREAM2_RtpReceiver_t *receiver, uint8_t type, const char *prefix, char **value, uint32_t *sendInterval);


/**
 * @brief Set a RTCP Source Description item
 *
 * @param[in] receiver The receiver instance
 * @param[in] type SDES item type
 * @param[in] prefix SDES item prefix (only for private extension type)
 * @param[in] value SDES item value
 * @param[in] sendInterval SDES item minimum send interval in microseconds
 *
 * @return ARSTREAM2_OK if no error happened
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the receiver or value pointers are invalid
 * @return ARSTREAM2_ERROR_ALLOC if the max number of SDES items has been reached
 */
eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_SetSdesItem(ARSTREAM2_RtpReceiver_t *receiver, uint8_t type, const char *prefix, const char *value, uint32_t sendInterval);


/**
 * @brief Get a peer RTCP Source Description item
 *
 * @param[in] receiver The receiver instance
 * @param[in] type SDES item type
 * @param[in] prefix SDES item prefix (only for private extension type)
 * @param[in] value Pointer to the SDES item value
 * @param[in] sendInterval Pointer to the SDES item minimum send interval in microseconds
 *
 * @return ARSTREAM2_OK if no error happened
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the receiver or value pointers are invalid
 * @return ARSTREAM2_ERROR_NOT_FOUND it the item has not been found
 */
eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_GetPeerSdesItem(ARSTREAM2_RtpReceiver_t *receiver, uint8_t type, const char *prefix, char **value);


/**
 * @brief Get the stream monitoring
 * The monitoring data is computed form the time startTime and back timeIntervalUs microseconds at most.
 * If startTime is 0 the start time is the current time.
 * If monitoring data is not available up to timeIntervalUs, the monitoring is computed on less time and the real interval is output to realTimeIntervalUs.
 * Pointers to monitoring parameters that are not required can be left NULL.
 *
 * @param[in] receiver The receiver instance
 * @param[in] startTime Monitoring start time in microseconds (0 means current time)
 * @param[in] timeIntervalUs Monitoring time interval (back from startTime) in microseconds
 * @param[out] realTimeIntervalUs Real monitoring time interval in microseconds (optional, can be NULL)
 * @param[out] receptionTimeJitter Network reception time jitter during realTimeIntervalUs in microseconds (optional, can be NULL)
 * @param[out] bytesReceived Bytes received during realTimeIntervalUs (optional, can be NULL)
 * @param[out] meanPacketSize Mean packet size during realTimeIntervalUs (optional, can be NULL)
 * @param[out] packetSizeStdDev Packet size standard deviation during realTimeIntervalUs (optional, can be NULL)
 * @param[out] packetsReceived Packets received during realTimeIntervalUs (optional, can be NULL)
 * @param[out] packetsMissed Packets missed during realTimeIntervalUs (optional, can be NULL)
 *
 * @return ARSTREAM2_OK if no error occured.
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the receiver is invalid or if timeIntervalUs is 0.
 */
eARSTREAM2_ERROR ARSTREAM2_RtpReceiver_GetMonitoring(ARSTREAM2_RtpReceiver_t *receiver, uint64_t startTime, uint32_t timeIntervalUs, uint32_t *realTimeIntervalUs, uint32_t *receptionTimeJitter,
                                                     uint32_t *bytesReceived, uint32_t *meanPacketSize, uint32_t *packetSizeStdDev, uint32_t *packetsReceived, uint32_t *packetsMissed);


#endif /* _ARSTREAM2_RTP_RECEIVER_H_ */
