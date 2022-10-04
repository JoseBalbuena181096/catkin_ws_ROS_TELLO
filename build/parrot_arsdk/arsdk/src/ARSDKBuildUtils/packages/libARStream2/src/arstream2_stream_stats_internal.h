/**
 * @file arstream2_stream_stats.h
 * @brief Parrot Streaming Library - Stream Stats
 * @date 10/04/2016
 * @author aurelien.barre@parrot.com
 */

#ifndef _ARSTREAM2_STREAM_STATS_INTERNAL_H_
#define _ARSTREAM2_STREAM_STATS_INTERNAL_H_

#include "arstream2_rtp.h"
#include "arstream2_h264.h"


typedef struct
{
    uint64_t fileOutputTimestamp;
    FILE *outputFile;

} ARSTREAM2_StreamStats_VideoStatsContext_t;


typedef struct
{
    uint64_t fileOutputTimestamp;
    FILE *outputFile;
    ARSTREAM2_RTP_RtpStats_t rtpStatsCumulated;
    unsigned int senderReportCumulatedCount;
    unsigned int receiverReportCumulatedCount;
    unsigned int djbMetricsReportCumulatedCount;

} ARSTREAM2_StreamStats_RtpStatsContext_t;


typedef struct
{
    FILE *outputFile;

} ARSTREAM2_StreamStats_RtpLossContext_t;


void ARSTREAM2_StreamStats_VideoStatsFileOpen(ARSTREAM2_StreamStats_VideoStatsContext_t *context, const char *debugPath, const char *friendlyName,
                                              const char *dateAndTime, uint32_t mbStatusZoneCount, uint32_t mbStatusClassCount);
void ARSTREAM2_StreamStats_VideoStatsFileClose(ARSTREAM2_StreamStats_VideoStatsContext_t *context);
void ARSTREAM2_StreamStats_VideoStatsFileWrite(ARSTREAM2_StreamStats_VideoStatsContext_t *context, const ARSTREAM2_H264_VideoStats_t *videoStats);

void ARSTREAM2_StreamStats_RtpStatsFileOpen(ARSTREAM2_StreamStats_RtpStatsContext_t *context, const char *debugPath,
                                            const char *friendlyName, const char *dateAndTime);
void ARSTREAM2_StreamStats_RtpStatsFileClose(ARSTREAM2_StreamStats_RtpStatsContext_t *context);
void ARSTREAM2_StreamStats_RtpStatsFileWrite(ARSTREAM2_StreamStats_RtpStatsContext_t *context, const ARSTREAM2_RTP_RtpStats_t *rtpStats, uint64_t curTime);

void ARSTREAM2_StreamStats_RtpLossFileOpen(ARSTREAM2_StreamStats_RtpLossContext_t *context, const char *debugPath,
                                            const char *friendlyName, const char *dateAndTime);
void ARSTREAM2_StreamStats_RtpLossFileClose(ARSTREAM2_StreamStats_RtpLossContext_t *context);
void ARSTREAM2_StreamStats_RtpLossFileWrite(ARSTREAM2_StreamStats_RtpLossContext_t *context, const ARSTREAM2_RTP_RtpStats_t *rtpStats);


#endif /* _ARSTREAM2_STREAM_STATS_INTERNAL_H_ */
