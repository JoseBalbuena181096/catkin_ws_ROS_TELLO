/**
 * @file arstream2_rtp_sender.h
 * @brief Parrot Streaming Library - RTP Sender
 * @date 04/17/2015
 * @author aurelien.barre@parrot.com
 */

#ifndef _ARSTREAM2_RTP_SENDER_H_
#define _ARSTREAM2_RTP_SENDER_H_

#ifdef __cplusplus
extern "C" {
#endif /* #ifdef __cplusplus */

#include <inttypes.h>
#include <libARStream2/arstream2_error.h>
#include <libARStream2/arstream2_stream_sender.h>
#include "arstream2_rtp.h"
#include "arstream2_rtcp.h"
#include "arstream2_h264.h"


/**
 * @brief Default server-side stream port
 */
#define ARSTREAM2_RTP_SENDER_DEFAULT_SERVER_STREAM_PORT     (5004)


/**
 * @brief Default server-side control port
 */
#define ARSTREAM2_RTP_SENDER_DEFAULT_SERVER_CONTROL_PORT    (5005)


/**
 * @brief Callback function for RTP stats
 * This callback function is called when an RTCP receiver report has been received.
 *
 * @param[in] rtpStats Pointer to RTP stats data
 * @param[in] userPtr Global callback user pointer
 */
typedef void (*ARSTREAM2_RtpSender_RtpStatsCallback_t) (const ARSTREAM2_RTP_RtpStats_t *rtpStats, void *userPtr);


/**
 * @brief Callback function for video stats
 * This callback function is called when video stats has been received though RTCP.
 *
 * @param[in] videoStats Pointer to video stats data
 * @param[in] userPtr Global callback user pointer
 */
typedef void (*ARSTREAM2_RtpSender_VideoStatsCallback_t) (const ARSTREAM2_H264_VideoStats_t *videoStats, void *userPtr);


/**
 * @brief RtpSender configuration parameters
 */
typedef struct ARSTREAM2_RtpSender_Config_t
{
    const char *canonicalName;                      /**< RTP participant canonical name (CNAME SDES item) */
    const char *friendlyName;                       /**< RTP participant friendly name (NAME SDES item) (optional, can be NULL) */
    const char *applicationName;                    /**< RTP participant application name (TOOL SDES item) (optional, can be NULL) */
    const char *clientAddr;                         /**< Client address */
    const char *mcastAddr;                          /**< Multicast send address (optional, NULL for no multicast) */
    const char *mcastIfaceAddr;                     /**< Multicast output interface address (required if mcastAddr is not NULL) */
    int serverStreamPort;                           /**< Server stream port, @see ARSTREAM2_RTP_SENDER_DEFAULT_SERVER_STREAM_PORT */
    int serverControlPort;                          /**< Server control port, @see ARSTREAM2_RTP_SENDER_DEFAULT_SERVER_CONTROL_PORT */
    int clientStreamPort;                           /**< Client stream port */
    int clientControlPort;                          /**< Client control port */
    eARSAL_SOCKET_CLASS_SELECTOR classSelector;     /**< Type of Service class selector */
    int streamSocketSendBufferSize;                 /**< Send buffer size for the stream socket (optional, can be 0) */
    ARSTREAM2_StreamSender_AuCallback_t auCallback;       /**< Access unit callback function (optional, can be NULL) */
    void *auCallbackUserPtr;                        /**< Access unit callback function user pointer (optional, can be NULL) */
    ARSTREAM2_StreamSender_NaluCallback_t naluCallback;   /**< NAL unit callback function (optional, can be NULL) */
    void *naluCallbackUserPtr;                      /**< NAL unit callback function user pointer (optional, can be NULL) */
    ARSTREAM2_RtpSender_RtpStatsCallback_t rtpStatsCallback;   /**< RTP stats callback function (optional, can be NULL) */
    void *rtpStatsCallbackUserPtr;                  /**< RTP stats callback function user pointer (optional, can be NULL) */
    ARSTREAM2_RtpSender_VideoStatsCallback_t videoStatsCallback;   /**< Video stats callback function (optional, can be NULL) */
    void *videoStatsCallbackUserPtr;                /**< Video stats callback function user pointer (optional, can be NULL) */
    ARSTREAM2_StreamSender_DisconnectionCallback_t disconnectionCallback;     /**< Disconnection callback function (optional, can be NULL) */
    void *disconnectionCallbackUserPtr;             /**< Disconnection callback function user pointer (optional, can be NULL) */
    ARSTREAM2_H264_NaluFifo_t *naluFifo;            /**< Optional user-provided NALU FIFO */
    ARSTREAM2_RTP_PacketFifo_t *packetFifo;         /**< User-provided packet FIFO */
    ARSTREAM2_RTP_PacketFifoQueue_t *packetFifoQueue;  /**< User-provided packet FIFO queue */
    int maxPacketSize;                              /**< Maximum network packet size in bytes (example: the interface MTU) */
    int targetPacketSize;                           /**< Target network packet size in bytes */
    int maxBitrate;                                 /**< Maximum streaming bitrate in bit/s (optional, can be 0) */
    int useRtpHeaderExtensions;                     /**< Boolean-like (0-1) flag: if active insert access unit metadata as RTP header extensions */
    const char *dateAndTime;
    const char *debugPath;

} ARSTREAM2_RtpSender_Config_t;


/**
 * @brief RtpSender dynamic configuration parameters
 */
typedef struct ARSTREAM2_RtpSender_DynamicConfig_t
{
    int targetPacketSize;                           /**< Target network packet size in bytes */
    int streamSocketSendBufferSize;                 /**< Send buffer size for the stream socket (optional, can be 0) */
    int maxBitrate;                                 /**< Maximum streaming bitrate in bit/s (optional, can be 0) */

} ARSTREAM2_RtpSender_DynamicConfig_t;


/**
 * @brief An RtpSender instance to allow streaming H.264 video over a network
 */
typedef struct ARSTREAM2_RtpSender_t ARSTREAM2_RtpSender_t;


/**
 * @brief Creates a new RtpSender
 * @warning This function allocates memory. The sender must be deleted by a call to ARSTREAM2_RtpSender_Delete()
 *
 * @param[in] config Pointer to a configuration parameters structure
 * @param[out] error Optionnal pointer to an eARSTREAM2_ERROR to hold any error information
 *
 * @return A pointer to the new ARSTREAM2_RtpSender_t, or NULL if an error occured
 *
 * @see ARSTREAM2_RtpSender_Stop()
 * @see ARSTREAM2_RtpSender_Delete()
 */
ARSTREAM2_RtpSender_t* ARSTREAM2_RtpSender_New(const ARSTREAM2_RtpSender_Config_t *config, eARSTREAM2_ERROR *error);


/**
 * @brief Deletes an RtpSender
 *
 * @param sender Pointer to the ARSTREAM2_RtpSender_t* to delete
 *
 * @return ARSTREAM2_OK if the sender was deleted
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if sender does not point to a valid ARSTREAM2_RtpSender_t
 *
 * @note The function uses a double pointer, so it can set *sender to NULL after freeing it
 */
eARSTREAM2_ERROR ARSTREAM2_RtpSender_Delete(ARSTREAM2_RtpSender_t **sender);


/**
 * @brief Flush all currently queued NAL units
 *
 * @param[in] sender The sender instance
 *
 * @return ARSTREAM2_OK if no error occured.
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the sender is invalid.
 */
eARSTREAM2_ERROR ARSTREAM2_RtpSender_FlushNaluQueue(ARSTREAM2_RtpSender_t *sender);


eARSTREAM2_ERROR ARSTREAM2_RtpSender_GetSelectParams(ARSTREAM2_RtpSender_t *sender, fd_set **readSet, fd_set **writeSet, fd_set **exceptSet, int *maxFd, uint32_t *nextTimeout);


eARSTREAM2_ERROR ARSTREAM2_RtpSender_ProcessRtp(ARSTREAM2_RtpSender_t *sender, int selectRet, fd_set *readSet, fd_set *writeSet, fd_set *exceptSet);


eARSTREAM2_ERROR ARSTREAM2_RtpSender_ProcessRtcp(ARSTREAM2_RtpSender_t *sender, int selectRet, fd_set *readSet, fd_set *writeSet, fd_set *exceptSet);


eARSTREAM2_ERROR ARSTREAM2_RtpSender_ProcessEnd(ARSTREAM2_RtpSender_t *sender, int queueOnly);


/**
 * @brief Get the current dynamic configuration parameters
 *
 * @param[in] sender The sender instance
 * @param[out] config Pointer to a dynamic config structure to fill
 *
 * @return ARSTREAM2_OK if no error happened
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the sender or config pointers are invalid
 */
eARSTREAM2_ERROR ARSTREAM2_RtpSender_GetDynamicConfig(ARSTREAM2_RtpSender_t *sender, ARSTREAM2_RtpSender_DynamicConfig_t *config);


/**
 * @brief Set the current dynamic configuration parameters
 *
 * @param[in] sender The sender instance
 * @param[in] config Pointer to a dynamic config structure
 *
 * @return ARSTREAM2_OK if no error happened
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the sender or config pointers are invalid
 */
eARSTREAM2_ERROR ARSTREAM2_RtpSender_SetDynamicConfig(ARSTREAM2_RtpSender_t *sender, const ARSTREAM2_RtpSender_DynamicConfig_t *config);


/**
 * @brief Get a RTCP Source Description item
 *
 * @param[in] sender The sender instance
 * @param[in] type SDES item type
 * @param[in] prefix SDES item prefix (only for private extension type)
 * @param[in] value Pointer to the SDES item value
 * @param[in] sendInterval Pointer to the SDES item minimum send interval in microseconds
 *
 * @return ARSTREAM2_OK if no error happened
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the sender or value pointers are invalid
 * @return ARSTREAM2_ERROR_NOT_FOUND it the item has not been found
 */
eARSTREAM2_ERROR ARSTREAM2_RtpSender_GetSdesItem(ARSTREAM2_RtpSender_t *sender, uint8_t type, const char *prefix, char **value, uint32_t *sendInterval);


/**
 * @brief Set a RTCP Source Description item
 *
 * @param[in] sender The sender instance
 * @param[in] type SDES item type
 * @param[in] prefix SDES item prefix (only for private extension type)
 * @param[in] value SDES item value
 * @param[in] sendInterval SDES item minimum send interval in microseconds
 *
 * @return ARSTREAM2_OK if no error happened
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the sender or value pointers are invalid
 * @return ARSTREAM2_ERROR_ALLOC if the max number of SDES items has been reached
 */
eARSTREAM2_ERROR ARSTREAM2_RtpSender_SetSdesItem(ARSTREAM2_RtpSender_t *sender, uint8_t type, const char *prefix, const char *value, uint32_t sendInterval);


/**
 * @brief Get a peer RTCP Source Description item
 *
 * @param[in] sender The sender instance
 * @param[in] type SDES item type
 * @param[in] prefix SDES item prefix (only for private extension type)
 * @param[in] value Pointer to the SDES item value
 * @param[in] sendInterval Pointer to the SDES item minimum send interval in microseconds
 *
 * @return ARSTREAM2_OK if no error happened
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the sender or value pointers are invalid
 * @return ARSTREAM2_ERROR_NOT_FOUND it the item has not been found
 */
eARSTREAM2_ERROR ARSTREAM2_RtpSender_GetPeerSdesItem(ARSTREAM2_RtpSender_t *sender, uint8_t type, const char *prefix, char **value);


/**
 * @brief Get the stream monitoring
 * The monitoring data is computed form the time startTime and back timeIntervalUs microseconds at most.
 * If startTime is 0 the start time is the current time.
 * If monitoring data is not available up to timeIntervalUs, the monitoring is computed on less time and the real interval is output to realTimeIntervalUs.
 * Pointers to monitoring parameters that are not required can be left NULL.
 *
 * @param[in] sender The sender instance
 * @param[in] startTime Monitoring start time in microseconds (0 means current time)
 * @param[in] timeIntervalUs Monitoring time interval (back from startTime) in microseconds
 * @param[out] monitoringData Pointer to a monitoring data structure to fill
 *
 * @return ARSTREAM2_OK if no error occured.
 * @return ARSTREAM2_ERROR_BAD_PARAMETERS if the sender is invalid or if timeIntervalUs is 0.
 */
eARSTREAM2_ERROR ARSTREAM2_RtpSender_GetMonitoring(ARSTREAM2_RtpSender_t *sender, uint64_t startTime, uint32_t timeIntervalUs,
                                                   ARSTREAM2_StreamSender_MonitoringData_t *monitoringData);


#ifdef __cplusplus
}
#endif /* #ifdef __cplusplus */

#endif /* _ARSTREAM2_RTP_SENDER_H_ */
