#!/usr/bin/python

# @file stream_video_stats.py
# @brief Streaming video stats script
# @date 09/21/2016
# @author aurelien.barre@parrot.com


import sys, getopt, math
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


################
#  Full stats  #
################

def videoStats(inFile, outFile, simple):
    # input file
    f = open(inFile, 'r')
    firstLine = f.readline()
    f.close()
    if firstLine != '' and firstLine[0] == '#':
        title = firstLine[1:]
        title = title.strip()
    else:
        title = inFile
    if firstLine != '' and firstLine.find(',') != -1:
        sep = ','
    else:
        sep = ' '

    data = pd.read_csv(inFile, sep=sep, comment='#', skip_blank_lines=True)


    # for compatibility with old stats files and files from telemetry blackbox
    if 'erroredSecondCount' in data.columns:
        lblEsr = 'erroredSecondCount'
        if 'erroredSecondCountByZone[0]' in data.columns:
            lblEsrZ0 = 'erroredSecondCountByZone[0]'
            lblEsrZ1 = 'erroredSecondCountByZone[1]'
            lblEsrZ2 = 'erroredSecondCountByZone[2]'
            lblEsrZ3 = 'erroredSecondCountByZone[3]'
            lblEsrZ4 = 'erroredSecondCountByZone[4]'
        else:
            lblEsrZ0 = 'erroredSecondCountByZone_0'
            lblEsrZ1 = 'erroredSecondCountByZone_1'
            lblEsrZ2 = 'erroredSecondCountByZone_2'
            lblEsrZ3 = 'erroredSecondCountByZone_3'
            lblEsrZ4 = 'erroredSecondCountByZone_4'
    else:
        lblEsr = 'errorSecondCount'
        lblEsrZ0 = 'errorSecondCountByZone[0]'
        lblEsrZ1 = 'errorSecondCountByZone[1]'
        lblEsrZ2 = 'errorSecondCountByZone[2]'
        lblEsrZ3 = 'errorSecondCountByZone[3]'
        lblEsrZ4 = 'errorSecondCountByZone[4]'
    if 'erroredOutputFrameCount' in data.columns:
        lblErroredOutputFrameCount = 'erroredOutputFrameCount'
    else:
        lblErroredOutputFrameCount = ''
    if 'macroblockStatus[0][0]' in data.columns:
        lblMbStatus0_0 = 'macroblockStatus[0][0]'
        lblMbStatus0_1 = 'macroblockStatus[0][1]'
        lblMbStatus0_2 = 'macroblockStatus[0][2]'
        lblMbStatus0_3 = 'macroblockStatus[0][3]'
        lblMbStatus0_4 = 'macroblockStatus[0][4]'
        lblMbStatus1_0 = 'macroblockStatus[1][0]'
        lblMbStatus1_1 = 'macroblockStatus[1][1]'
        lblMbStatus1_2 = 'macroblockStatus[1][2]'
        lblMbStatus1_3 = 'macroblockStatus[1][3]'
        lblMbStatus1_4 = 'macroblockStatus[1][4]'
        lblMbStatus2_0 = 'macroblockStatus[2][0]'
        lblMbStatus2_1 = 'macroblockStatus[2][1]'
        lblMbStatus2_2 = 'macroblockStatus[2][2]'
        lblMbStatus2_3 = 'macroblockStatus[2][3]'
        lblMbStatus2_4 = 'macroblockStatus[2][4]'
        lblMbStatus3_0 = 'macroblockStatus[3][0]'
        lblMbStatus3_1 = 'macroblockStatus[3][1]'
        lblMbStatus3_2 = 'macroblockStatus[3][2]'
        lblMbStatus3_3 = 'macroblockStatus[3][3]'
        lblMbStatus3_4 = 'macroblockStatus[3][4]'
        lblMbStatus4_0 = 'macroblockStatus[4][0]'
        lblMbStatus4_1 = 'macroblockStatus[4][1]'
        lblMbStatus4_2 = 'macroblockStatus[4][2]'
        lblMbStatus4_3 = 'macroblockStatus[4][3]'
        lblMbStatus4_4 = 'macroblockStatus[4][4]'
        lblMbStatus5_0 = 'macroblockStatus[5][0]'
        lblMbStatus5_1 = 'macroblockStatus[5][1]'
        lblMbStatus5_2 = 'macroblockStatus[5][2]'
        lblMbStatus5_3 = 'macroblockStatus[5][3]'
        lblMbStatus5_4 = 'macroblockStatus[5][4]'
    else:
        lblMbStatus0_0 = 'macroblockStatus_0'
        lblMbStatus0_1 = 'macroblockStatus_1'
        lblMbStatus0_2 = 'macroblockStatus_2'
        lblMbStatus0_3 = 'macroblockStatus_3'
        lblMbStatus0_4 = 'macroblockStatus_4'
        lblMbStatus1_0 = 'macroblockStatus_5'
        lblMbStatus1_1 = 'macroblockStatus_6'
        lblMbStatus1_2 = 'macroblockStatus_7'
        lblMbStatus1_3 = 'macroblockStatus_8'
        lblMbStatus1_4 = 'macroblockStatus_9'
        lblMbStatus2_0 = 'macroblockStatus_10'
        lblMbStatus2_1 = 'macroblockStatus_11'
        lblMbStatus2_2 = 'macroblockStatus_12'
        lblMbStatus2_3 = 'macroblockStatus_13'
        lblMbStatus2_4 = 'macroblockStatus_14'
        lblMbStatus3_0 = 'macroblockStatus_15'
        lblMbStatus3_1 = 'macroblockStatus_16'
        lblMbStatus3_2 = 'macroblockStatus_17'
        lblMbStatus3_3 = 'macroblockStatus_18'
        lblMbStatus3_4 = 'macroblockStatus_19'
        lblMbStatus4_0 = 'macroblockStatus_20'
        lblMbStatus4_1 = 'macroblockStatus_21'
        lblMbStatus4_2 = 'macroblockStatus_22'
        lblMbStatus4_3 = 'macroblockStatus_23'
        lblMbStatus4_4 = 'macroblockStatus_24'
        lblMbStatus5_0 = 'macroblockStatus_25'
        lblMbStatus5_1 = 'macroblockStatus_26'
        lblMbStatus5_2 = 'macroblockStatus_27'
        lblMbStatus5_3 = 'macroblockStatus_28'
        lblMbStatus5_4 = 'macroblockStatus_29'


    ####################
    #  Overall frames  #
    ####################

    overallTotalFrameCount = float(data['totalFrameCount'].iat[-1])
    if lblErroredOutputFrameCount != '':
        overallErroredOutputFrameCount = float(data[lblErroredOutputFrameCount].iat[-1])
    else:
        overallErroredOutputFrameCount = 0
    overallNoErrorOutputFrameCount = float(data['outputFrameCount'].iat[-1]) - overallErroredOutputFrameCount
    overallDiscardedFrameCount = float(data['discardedFrameCount'].iat[-1])
    overallMissedFrameCount = float(data['missedFrameCount'].iat[-1]) - overallDiscardedFrameCount
    overallErroredFrameRatio = (overallErroredOutputFrameCount + overallDiscardedFrameCount + overallMissedFrameCount) / overallTotalFrameCount * 100.
    overallTotalFrameCount2 = overallErroredOutputFrameCount + overallNoErrorOutputFrameCount + overallDiscardedFrameCount + overallMissedFrameCount
    if overallTotalFrameCount2 != overallTotalFrameCount:
        print "Warning - overall frame count missmatch: " + str(int(overallTotalFrameCount)) + " vs. " + str(int(overallTotalFrameCount2))


    ########################
    #  Frames across time  #
    ########################

    firstTime = data['timestamp'][1]
    dataTime = []
    for i, val in enumerate(data['timestamp']):
        if i > 0:
            dataTime.append((float(val) - firstTime) / 1000000.)
    dataTotalFrameDelta = []
    for i, val in enumerate(data['totalFrameCount']):
        if i > 0:
            dataTotalFrameDelta.append(float(val) - float(data['totalFrameCount'][i - 1]))
    if lblErroredOutputFrameCount != '':
        dataErroredOutputFrameDelta = np.subtract(data[lblErroredOutputFrameCount][1:], data[lblErroredOutputFrameCount][:-1])
    else:
        dataErroredOutputFrameDelta = np.empty(len(dataTotalFrameDelta))
        dataErroredOutputFrameDelta.fill(0)
    dataNoErrorOutputFrameDelta = np.subtract(np.subtract(data['outputFrameCount'][1:], data['outputFrameCount'][:-1]), dataErroredOutputFrameDelta)
    dataDiscardedFrameDelta = np.subtract(data['discardedFrameCount'][1:], data['discardedFrameCount'][:-1])
    dataMissedFrameDelta = np.subtract(np.subtract(data['missedFrameCount'][1:], data['missedFrameCount'][:-1]), dataDiscardedFrameDelta)

    noErrorOutputFrames = np.divide(dataNoErrorOutputFrameDelta, dataTotalFrameDelta) * 100
    erroredOutputFrames = np.divide(dataErroredOutputFrameDelta, dataTotalFrameDelta) * 100
    discardedFrames = np.divide(dataDiscardedFrameDelta, dataTotalFrameDelta) * 100
    missedFrames = np.divide(dataMissedFrameDelta, dataTotalFrameDelta) * 100

    discardedFrames = np.add(discardedFrames, missedFrames)
    erroredOutputFrames = np.add(erroredOutputFrames, discardedFrames)
    noErrorOutputFrames = np.add(noErrorOutputFrames, erroredOutputFrames)


    ####################
    #  Frames by RSSI  #
    ####################

    rssiMin = np.amin(data['rssi'])
    rssiMax = np.amax(data['rssi'])
    rssiCount = rssiMax - rssiMin + 1
    dataRssi = np.array(data['rssi'][1:])

    totalFrameCountByRssi = np.empty(rssiCount)
    totalFrameCountByRssi.fill(0)
    for i, val in enumerate(dataTotalFrameDelta):
        totalFrameCountByRssi[dataRssi[i] - rssiMin] += float(val)

    erroredOutputFrameCountByRssi = np.empty(rssiCount)
    erroredOutputFrameCountByRssi.fill(0)
    for i, val in enumerate(dataErroredOutputFrameDelta):
        erroredOutputFrameCountByRssi[dataRssi[i] - rssiMin] += float(val)

    noErrorOutputFrameCountByRssi = np.empty(rssiCount)
    noErrorOutputFrameCountByRssi.fill(0)
    for i, val in enumerate(dataNoErrorOutputFrameDelta):
        noErrorOutputFrameCountByRssi[dataRssi[i] - rssiMin] += float(val)

    discardedFrameCountByRssi = np.empty(rssiCount)
    discardedFrameCountByRssi.fill(0)
    for i, val in enumerate(dataDiscardedFrameDelta):
        discardedFrameCountByRssi[dataRssi[i] - rssiMin] += float(val)

    missedFrameCountByRssi = np.empty(rssiCount)
    missedFrameCountByRssi.fill(0)
    for i, val in enumerate(dataMissedFrameDelta):
        missedFrameCountByRssi[dataRssi[i] - rssiMin] += float(val)

    overallTotalFrameNear = 0.
    overallTotalFrameFar = 0.
    for i, val in enumerate(totalFrameCountByRssi):
        if i + rssiMin >= -70:
            overallTotalFrameNear += val
        else:
            overallTotalFrameFar += val
    overallErroredFrameNear = 0.
    overallErroredFrameFar = 0.
    for i, val in enumerate(erroredOutputFrameCountByRssi):
        if totalFrameCountByRssi[i] > 0.:
            if i + rssiMin >= -70:
                overallErroredFrameNear += val
            else:
                overallErroredFrameFar += val
    for i, val in enumerate(discardedFrameCountByRssi):
        if totalFrameCountByRssi[i] > 0.:
            if i + rssiMin >= -70:
                overallErroredFrameNear += val
            else:
                overallErroredFrameFar += val
    for i, val in enumerate(missedFrameCountByRssi):
        if totalFrameCountByRssi[i] > 0.:
            if i + rssiMin >= -70:
                overallErroredFrameNear += val
            else:
                overallErroredFrameFar += val
    if overallTotalFrameNear > 0.:
        overallErroredFrameNear = overallErroredFrameNear / overallTotalFrameNear * 100.
    if overallTotalFrameFar > 0.:
        overallErroredFrameFar = overallErroredFrameFar / overallTotalFrameFar * 100.

    erroredOutputFrameCountByRssi = np.divide(erroredOutputFrameCountByRssi, totalFrameCountByRssi)
    noErrorOutputFrameCountByRssi = np.divide(noErrorOutputFrameCountByRssi, totalFrameCountByRssi)
    discardedFrameCountByRssi = np.divide(discardedFrameCountByRssi, totalFrameCountByRssi)
    missedFrameCountByRssi = np.divide(missedFrameCountByRssi, totalFrameCountByRssi)

    totalFrameCountByRssi = np.divide(totalFrameCountByRssi, overallTotalFrameCount)


    #################
    #  ESR by zone  #
    #################

    totalTime = float(data['timestamp'].iat[-1] - data['timestamp'].iat[0] + 1000000) / 1000000.
    totalEsr = float(data[lblEsr].iat[-1]) / totalTime * 100.
    zoneEsr = [float(data[lblEsrZ4].iat[-1]) / totalTime * 100., float(data[lblEsrZ3].iat[-1]) / totalTime * 100., float(data[lblEsrZ2].iat[-1]) / totalTime * 100., float(data[lblEsrZ1].iat[-1]) / totalTime * 100., float(data[lblEsrZ0].iat[-1]) / totalTime * 100.]
    zoneEsrYPos = np.arange(5)
    zoneEsrLabel = ('5', '4', '3', '2', '1')

    dataTimeDelta = []
    for i, val in enumerate(data['timestamp']):
        if i > 0:
            dataTimeDelta.append((float(val) - float(data['timestamp'][i - 1])) / 1000000.)
    timeByRssi = np.empty(rssiCount)
    timeByRssi.fill(0)
    for i, val in enumerate(dataTimeDelta):
        timeByRssi[dataRssi[i] - rssiMin] += float(val)

    dataEsr = []
    for i, val in enumerate(data[lblEsr]):
        if i > 0:
            dataEsr.append(float(val) - float(data[lblEsr][i - 1]))
    esrByRssi = np.empty(rssiCount)
    esrByRssi.fill(0)
    for i, val in enumerate(dataEsr):
        esrByRssi[dataRssi[i] - rssiMin] += float(val)

    overallTimeNear = 0.
    overallTimeFar = 0.
    for i, val in enumerate(timeByRssi):
        if i + rssiMin >= -70:
            overallTimeNear += val
        else:
            overallTimeFar += val
    overallEsrNear = 0.
    overallEsrFar = 0.
    for i, val in enumerate(esrByRssi):
        if totalFrameCountByRssi[i] > 0.:
            if i + rssiMin >= -70:
                overallEsrNear += val
            else:
                overallEsrFar += val
    if overallTimeNear > 0.:
        overallEsrNear = overallEsrNear / overallTimeNear * 100.
    if overallTimeFar > 0.:
        overallEsrFar = overallEsrFar / overallTimeFar * 100.


    #####################
    #  Timings by RSSI  #
    #####################

    dataOutputFrameCount = np.subtract(data['outputFrameCount'][1:], data['outputFrameCount'][:-1])
    if 'timestampDeltaIntegral' in data.columns:
        dataTimestampDelta = np.subtract(data['timestampDeltaIntegral'][1:], data['timestampDeltaIntegral'][:-1])
        dataTimestampDeltaSq = np.subtract(data['timestampDeltaIntegralSq'][1:], data['timestampDeltaIntegralSq'][:-1])
    else:
        dataTimestampDelta = []
        dataTimestampDeltaSq = []
    if 'timingErrorIntegral' in data.columns:
        dataTimingError = np.subtract(data['timingErrorIntegral'][1:], data['timingErrorIntegral'][:-1])
        dataTimingErrorSq = np.subtract(data['timingErrorIntegralSq'][1:], data['timingErrorIntegralSq'][:-1])
    else:
        dataTimingError = []
        dataTimingErrorSq = []
    if 'estimatedLatencyIntegral' in data.columns:
        dataEstimatedLatency = np.subtract(data['estimatedLatencyIntegral'][1:], data['estimatedLatencyIntegral'][:-1])
        dataEstimatedLatencySq = np.subtract(data['estimatedLatencyIntegralSq'][1:], data['estimatedLatencyIntegralSq'][:-1])
    else:
        dataEstimatedLatency = []
        dataEstimatedLatencySq = []

    outputFrameCountByRssi = np.empty(rssiCount)
    outputFrameCountByRssi.fill(0)
    for i, val in enumerate(dataOutputFrameCount):
        outputFrameCountByRssi[dataRssi[i] - rssiMin] += float(val)

    timestampDeltaByRssi = np.empty(rssiCount)
    timestampDeltaByRssi.fill(0)
    for i, val in enumerate(dataTimestampDelta):
        timestampDeltaByRssi[dataRssi[i] - rssiMin] += float(val) / 1000.
    timestampDeltaByRssi = np.divide(timestampDeltaByRssi, outputFrameCountByRssi)
    timestampDeltaByRssiStd = np.empty(rssiCount)
    timestampDeltaByRssiStd.fill(0)
    for i, val in enumerate(dataTimestampDeltaSq):
        timestampDeltaByRssiStd[dataRssi[i] - rssiMin] += float(val) / 1000000.
    timestampDeltaByRssiStd = np.sqrt(np.abs(np.subtract(np.divide(timestampDeltaByRssiStd, outputFrameCountByRssi), np.square(timestampDeltaByRssi))))

    timingErrorByRssi = np.empty(rssiCount)
    timingErrorByRssi.fill(0)
    for i, val in enumerate(dataTimingError):
        timingErrorByRssi[dataRssi[i] - rssiMin] += float(val) / 1000.
    timingErrorByRssi = np.divide(timingErrorByRssi, outputFrameCountByRssi)
    timingErrorByRssiStd = np.empty(rssiCount)
    timingErrorByRssiStd.fill(0)
    for i, val in enumerate(dataTimingErrorSq):
        timingErrorByRssiStd[dataRssi[i] - rssiMin] += float(val) / 1000000.
    timingErrorByRssiStd = np.sqrt(np.abs(np.subtract(np.divide(timingErrorByRssiStd, outputFrameCountByRssi), np.square(timingErrorByRssi))))

    estimatedLatencyByRssi = np.empty(rssiCount)
    estimatedLatencyByRssi.fill(0)
    for i, val in enumerate(dataEstimatedLatency):
        estimatedLatencyByRssi[dataRssi[i] - rssiMin] += float(val) / 1000.
    estimatedLatencyByRssi = np.divide(estimatedLatencyByRssi, outputFrameCountByRssi)
    estimatedLatencyByRssiStd = np.empty(rssiCount)
    estimatedLatencyByRssiStd.fill(0)
    for i, val in enumerate(dataEstimatedLatencySq):
        estimatedLatencyByRssiStd[dataRssi[i] - rssiMin] += float(val) / 1000000.
    estimatedLatencyByRssiStd = np.sqrt(np.abs(np.subtract(np.divide(estimatedLatencyByRssiStd, outputFrameCountByRssi), np.square(estimatedLatencyByRssi))))


    #########################
    #  Timings across time  #
    #########################

    timestampDelta = []
    timestampDeltaStd = []
    for i, val in enumerate(data['outputFrameCount']):
        if i > 0:
            if val != 0:
                timestampDelta.append(float(dataTimestampDelta[i]) / dataOutputFrameCount[i] / 1000.)
                timestampDeltaStd.append(np.sqrt(np.abs(float(dataTimestampDeltaSq[i]) / dataOutputFrameCount[i] / 1000000. - (timestampDelta[-1] * timestampDelta[-1]))))
            else:
                timestampDelta.append(0.)
                timestampDeltaStd.append(0.)
    timingError = []
    timingErrorStd = []
    for i, val in enumerate(data['outputFrameCount']):
        if i > 0:
            if val != 0:
                timingError.append(float(dataTimingError[i]) / dataOutputFrameCount[i] / 1000.)
                timingErrorStd.append(np.sqrt(np.abs(float(dataTimingErrorSq[i]) / dataOutputFrameCount[i] / 1000000. - (timingError[-1] * timingError[-1]))))
            else:
                timingError.append(0.)
                timingErrorStd.append(0.)
    estimatedLatency = []
    estimatedLatencyStd = []
    for i, val in enumerate(data['outputFrameCount']):
        if i > 0:
            if val != 0:
                estimatedLatency.append(float(dataEstimatedLatency[i]) / dataOutputFrameCount[i] / 1000.)
                estimatedLatencyStd.append(np.sqrt(np.abs(float(dataEstimatedLatencySq[i]) / dataOutputFrameCount[i] / 1000000. - (estimatedLatency[-1] * estimatedLatency[-1]))))
            else:
                estimatedLatency.append(0.)
                estimatedLatencyStd.append(0.)


    #####################
    #  Overall timings  #
    #####################

    if 'timestampDeltaIntegral' in data.columns:
        overallTimestampDelta = float(data['timestampDeltaIntegral'].iat[-1]) / float(data['outputFrameCount'].iat[-1]) / 1000.
        overallTimestampDeltaStd = math.sqrt(float(data['timestampDeltaIntegralSq'].iat[-1]) / float(data['outputFrameCount'].iat[-1]) / 1000000. - overallTimestampDelta * overallTimestampDelta)
        overallTimingError = float(data['timingErrorIntegral'].iat[-1]) / float(data['outputFrameCount'].iat[-1]) / 1000.
        overallTimingErrorStd = math.sqrt(float(data['timingErrorIntegralSq'].iat[-1]) / float(data['outputFrameCount'].iat[-1]) / 1000000. - overallTimingError * overallTimingError)
        overallEstimatedLatency = float(data['estimatedLatencyIntegral'].iat[-1]) / float(data['outputFrameCount'].iat[-1]) / 1000.
        overallEstimatedLatencyStd = math.sqrt(float(data['estimatedLatencyIntegralSq'].iat[-1]) / float(data['outputFrameCount'].iat[-1]) / 1000000. - overallEstimatedLatency * overallEstimatedLatency)
    else:
        overallTimestampDelta = 0.
        overallTimestampDeltaStd = 0.
        overallTimingError = 0.
        overallTimingErrorStd = 0.
        overallEstimatedLatency = 0.
        overallEstimatedLatencyStd = 0.


    #########################
    #  Overall macroblocks  #
    #########################

    overallUnknownMbCount = float(data[lblMbStatus0_0].iat[-1] + data[lblMbStatus0_1].iat[-1] + data[lblMbStatus0_2].iat[-1] + data[lblMbStatus0_3].iat[-1] + data[lblMbStatus0_4].iat[-1])
    overallValidISliceMbCount = float(data[lblMbStatus1_0].iat[-1] + data[lblMbStatus1_1].iat[-1] + data[lblMbStatus1_2].iat[-1] + data[lblMbStatus1_3].iat[-1] + data[lblMbStatus1_4].iat[-1])
    overallValidPSliceMbCount = float(data[lblMbStatus2_0].iat[-1] + data[lblMbStatus2_1].iat[-1] + data[lblMbStatus2_2].iat[-1] + data[lblMbStatus2_3].iat[-1] + data[lblMbStatus2_4].iat[-1])
    overallMissingConcealedMbCount = float(data[lblMbStatus3_0].iat[-1] + data[lblMbStatus3_1].iat[-1] + data[lblMbStatus3_2].iat[-1] + data[lblMbStatus3_3].iat[-1] + data[lblMbStatus3_4].iat[-1])
    overallMissingMbCount = float(data[lblMbStatus4_0].iat[-1] + data[lblMbStatus4_1].iat[-1] + data[lblMbStatus4_2].iat[-1] + data[lblMbStatus4_3].iat[-1] + data[lblMbStatus4_4].iat[-1])
    overallErrorPropagationMbCount = float(data[lblMbStatus5_0].iat[-1] + data[lblMbStatus5_1].iat[-1] + data[lblMbStatus5_2].iat[-1] + data[lblMbStatus5_3].iat[-1] + data[lblMbStatus5_4].iat[-1])
    overallValidTotalMbCount = overallValidISliceMbCount + overallValidPSliceMbCount
    overallInvalidTotalMbCount = overallUnknownMbCount + overallMissingConcealedMbCount + overallMissingMbCount + overallErrorPropagationMbCount
    overallTotalMbCount = overallValidTotalMbCount + overallInvalidTotalMbCount
    overallErroredMbRatio = overallInvalidTotalMbCount / overallTotalMbCount * 100.


    #############################
    #  Macroblocks across time  #
    #############################

    dataUnknownMb0 = np.subtract(data[lblMbStatus0_0][1:], data[lblMbStatus0_0][:-1])
    dataUnknownMb1 = np.subtract(data[lblMbStatus0_1][1:], data[lblMbStatus0_1][:-1])
    dataUnknownMb2 = np.subtract(data[lblMbStatus0_2][1:], data[lblMbStatus0_2][:-1])
    dataUnknownMb3 = np.subtract(data[lblMbStatus0_3][1:], data[lblMbStatus0_3][:-1])
    dataUnknownMb4 = np.subtract(data[lblMbStatus0_4][1:], data[lblMbStatus0_4][:-1])
    dataUnknownMb = np.add(dataUnknownMb0, np.add(dataUnknownMb1, np.add(dataUnknownMb2, np.add(dataUnknownMb3, dataUnknownMb4))))
    dataValidISliceMb0 = np.subtract(data[lblMbStatus1_0][1:], data[lblMbStatus1_0][:-1])
    dataValidISliceMb1 = np.subtract(data[lblMbStatus1_1][1:], data[lblMbStatus1_1][:-1])
    dataValidISliceMb2 = np.subtract(data[lblMbStatus1_2][1:], data[lblMbStatus1_2][:-1])
    dataValidISliceMb3 = np.subtract(data[lblMbStatus1_3][1:], data[lblMbStatus1_3][:-1])
    dataValidISliceMb4 = np.subtract(data[lblMbStatus1_4][1:], data[lblMbStatus1_4][:-1])
    dataValidISliceMb = np.add(dataValidISliceMb0, np.add(dataValidISliceMb1, np.add(dataValidISliceMb2, np.add(dataValidISliceMb3, dataValidISliceMb4))))
    dataValidPSliceMb0 = np.subtract(data[lblMbStatus2_0][1:], data[lblMbStatus2_0][:-1])
    dataValidPSliceMb1 = np.subtract(data[lblMbStatus2_1][1:], data[lblMbStatus2_1][:-1])
    dataValidPSliceMb2 = np.subtract(data[lblMbStatus2_2][1:], data[lblMbStatus2_2][:-1])
    dataValidPSliceMb3 = np.subtract(data[lblMbStatus2_3][1:], data[lblMbStatus2_3][:-1])
    dataValidPSliceMb4 = np.subtract(data[lblMbStatus2_4][1:], data[lblMbStatus2_4][:-1])
    dataValidPSliceMb = np.add(dataValidPSliceMb0, np.add(dataValidPSliceMb1, np.add(dataValidPSliceMb2, np.add(dataValidPSliceMb3, dataValidPSliceMb4))))
    dataMissingConcealedMb0 = np.subtract(data[lblMbStatus3_0][1:], data[lblMbStatus3_0][:-1])
    dataMissingConcealedMb1 = np.subtract(data[lblMbStatus3_1][1:], data[lblMbStatus3_1][:-1])
    dataMissingConcealedMb2 = np.subtract(data[lblMbStatus3_2][1:], data[lblMbStatus3_2][:-1])
    dataMissingConcealedMb3 = np.subtract(data[lblMbStatus3_3][1:], data[lblMbStatus3_3][:-1])
    dataMissingConcealedMb4 = np.subtract(data[lblMbStatus3_4][1:], data[lblMbStatus3_4][:-1])
    dataMissingConcealedMb = np.add(dataMissingConcealedMb0, np.add(dataMissingConcealedMb1, np.add(dataMissingConcealedMb2, np.add(dataMissingConcealedMb3, dataMissingConcealedMb4))))
    dataMissingMb0 = np.subtract(data[lblMbStatus4_0][1:], data[lblMbStatus4_0][:-1])
    dataMissingMb1 = np.subtract(data[lblMbStatus4_1][1:], data[lblMbStatus4_1][:-1])
    dataMissingMb2 = np.subtract(data[lblMbStatus4_2][1:], data[lblMbStatus4_2][:-1])
    dataMissingMb3 = np.subtract(data[lblMbStatus4_3][1:], data[lblMbStatus4_3][:-1])
    dataMissingMb4 = np.subtract(data[lblMbStatus4_4][1:], data[lblMbStatus4_4][:-1])
    dataMissingMb = np.add(dataMissingMb0, np.add(dataMissingMb1, np.add(dataMissingMb2, np.add(dataMissingMb3, dataMissingMb4))))
    dataErrorPropagationMb0 = np.subtract(data[lblMbStatus5_0][1:], data[lblMbStatus5_0][:-1])
    dataErrorPropagationMb1 = np.subtract(data[lblMbStatus5_1][1:], data[lblMbStatus5_1][:-1])
    dataErrorPropagationMb2 = np.subtract(data[lblMbStatus5_2][1:], data[lblMbStatus5_2][:-1])
    dataErrorPropagationMb3 = np.subtract(data[lblMbStatus5_3][1:], data[lblMbStatus5_3][:-1])
    dataErrorPropagationMb4 = np.subtract(data[lblMbStatus5_4][1:], data[lblMbStatus5_4][:-1])
    dataErrorPropagationMb = np.add(dataErrorPropagationMb0, np.add(dataErrorPropagationMb1, np.add(dataErrorPropagationMb2, np.add(dataErrorPropagationMb3, dataErrorPropagationMb4))))
    dataTotalMb = []
    for i, val in enumerate(data['timestamp']):
        if i > 0:
            dataTotalMb.append(float(dataUnknownMb[i]) + float(dataValidISliceMb[i]) + float(dataValidPSliceMb[i]) + float(dataMissingConcealedMb[i]) + float(dataMissingMb[i]) + float(dataErrorPropagationMb[i]))

    validISliceMb = np.divide(dataValidISliceMb, dataTotalMb) * 100
    validPSliceMb = np.divide(dataValidPSliceMb, dataTotalMb) * 100
    errorPropagationMb = np.divide(dataErrorPropagationMb, dataTotalMb) * 100
    missingConcealedMb = np.divide(dataMissingConcealedMb, dataTotalMb) * 100
    missingMb = np.divide(dataMissingMb, dataTotalMb) * 100
    unknownMb = np.divide(dataUnknownMb, dataTotalMb) * 100

    missingMb = np.add(missingMb, unknownMb)
    missingConcealedMb = np.add(missingConcealedMb, missingMb)
    errorPropagationMb = np.add(errorPropagationMb, missingConcealedMb)
    validPSliceMb = np.add(validPSliceMb, errorPropagationMb)
    validISliceMb = np.add(validISliceMb, validPSliceMb)


    #########################
    #  Macroblocks by RSSI  #
    #########################

    totalMbCountByRssi = np.empty(rssiCount)
    totalMbCountByRssi.fill(0)
    for i, val in enumerate(dataTotalMb):
        totalMbCountByRssi[dataRssi[i] - rssiMin] += float(val)

    unknownMbByRssi = np.empty(rssiCount)
    unknownMbByRssi.fill(0)
    for i, val in enumerate(dataUnknownMb):
        unknownMbByRssi[dataRssi[i] - rssiMin] += float(val)

    validISliceMbByRssi = np.empty(rssiCount)
    validISliceMbByRssi.fill(0)
    for i, val in enumerate(dataValidISliceMb):
        validISliceMbByRssi[dataRssi[i] - rssiMin] += float(val)

    validPSliceMbByRssi = np.empty(rssiCount)
    validPSliceMbByRssi.fill(0)
    for i, val in enumerate(dataValidPSliceMb):
        validPSliceMbByRssi[dataRssi[i] - rssiMin] += float(val)

    missingConcealedMbByRssi = np.empty(rssiCount)
    missingConcealedMbByRssi.fill(0)
    for i, val in enumerate(dataMissingConcealedMb):
        missingConcealedMbByRssi[dataRssi[i] - rssiMin] += float(val)

    missingMbByRssi = np.empty(rssiCount)
    missingMbByRssi.fill(0)
    for i, val in enumerate(dataMissingMb):
        missingMbByRssi[dataRssi[i] - rssiMin] += float(val)

    errorPropagationMbByRssi = np.empty(rssiCount)
    errorPropagationMbByRssi.fill(0)
    for i, val in enumerate(dataErrorPropagationMb):
        errorPropagationMbByRssi[dataRssi[i] - rssiMin] += float(val)

    overallTotalMbNear = 0.
    overallTotalMbFar = 0.
    for i, val in enumerate(totalMbCountByRssi):
        if i + rssiMin >= -70:
            overallTotalMbNear += val
        else:
            overallTotalMbFar += val
    overallErroredMbNear = 0.
    overallErroredMbFar = 0.
    for i, val in enumerate(unknownMbByRssi):
        if totalMbCountByRssi[i] > 0.:
            if i + rssiMin >= -70:
                overallErroredMbNear += val
            else:
                overallErroredMbFar += val
    for i, val in enumerate(missingConcealedMbByRssi):
        if totalMbCountByRssi[i] > 0.:
            if i + rssiMin >= -70:
                overallErroredMbNear += val
            else:
                overallErroredMbFar += val
    for i, val in enumerate(missingMbByRssi):
        if totalMbCountByRssi[i] > 0.:
            if i + rssiMin >= -70:
                overallErroredMbNear += val
            else:
                overallErroredMbFar += val
    for i, val in enumerate(errorPropagationMbByRssi):
        if totalMbCountByRssi[i] > 0.:
            if i + rssiMin >= -70:
                overallErroredMbNear += val
            else:
                overallErroredMbFar += val
    if overallTotalMbNear > 0.:
        overallErroredMbNear = overallErroredMbNear / overallTotalMbNear * 100.
    if overallTotalMbFar > 0.:
        overallErroredMbFar = overallErroredMbFar / overallTotalMbFar * 100.

    unknownMbByRssi = np.divide(unknownMbByRssi, totalMbCountByRssi)
    validISliceMbByRssi = np.divide(validISliceMbByRssi, totalMbCountByRssi)
    validPSliceMbByRssi = np.divide(validPSliceMbByRssi, totalMbCountByRssi)
    missingConcealedMbByRssi = np.divide(missingConcealedMbByRssi, totalMbCountByRssi)
    missingMbByRssi = np.divide(missingMbByRssi, totalMbCountByRssi)
    errorPropagationMbByRssi = np.divide(errorPropagationMbByRssi, totalMbCountByRssi)

    totalMbCountByRssi = np.divide(totalMbCountByRssi, overallTotalMbCount)


    if simple:
        ###################
        #  Simple graphs  #
        ###################

        fig1 = plt.figure(figsize=(1000. / 96., 800. / 96.), dpi=96)
        plt.subplots_adjust(left=0.03, right=0.97, bottom=0.03, top=0.92)
        plt.suptitle('Video stats (' + title + ')', fontsize=16, color='0.4')

        ####################
        #  Overall frames  #
        ####################

        axOverallFrames = fig1.add_subplot(2, 3, 1)
        axOverallFrames.set_title("Overall frames", color='0.4')
        axOverallFrames.pie([overallNoErrorOutputFrameCount, overallErroredOutputFrameCount, overallDiscardedFrameCount, overallMissedFrameCount], labels=['Output (no errors)', 'Output (with errors)', 'Discarded', 'Missed'], colors=['lightgreen', 'cadetblue', 'salmon', 'firebrick'], autopct='%1.1f%%', startangle=90)
        axOverallFrames.set_aspect('equal')


        #################
        #  ESR by zone  #
        #################

        axZoneEsr = fig1.add_subplot(2, 3, 3)
        axZoneEsr.set_xlim(0, 100)
        axZoneEsr.set_title("ESR by zone", color='0.4')
        axZoneEsr.barh(zoneEsrYPos, zoneEsr, align='center', alpha=0.8, color='cadetblue')
        axZoneEsr.set_yticks(zoneEsrYPos)
        axZoneEsr.set_yticklabels(zoneEsrLabel)
        axZoneEsr.set_xlabel('ESR (%)')
        axZoneEsr.set_ylabel('Zone')


        #########################
        #  Overall macroblocks  #
        #########################

        axOverallMbs = fig1.add_subplot(2, 3, 4)
        axOverallMbs.set_title("Overall macroblocks", color='0.4')
        axOverallMbs.pie([overallValidISliceMbCount, overallValidPSliceMbCount, overallMissingConcealedMbCount, overallMissingMbCount, overallErrorPropagationMbCount, overallUnknownMbCount], labels=['Valid (I)', 'Valid (P)', 'Missing (concealed)', 'Missing', 'Error propagation', 'Unknown'], colors=['mediumseagreen', 'lightgreen', 'salmon', 'firebrick', 'cadetblue', '0.3'], autopct='%1.1f%%', startangle=90)
        axOverallMbs.set_aspect('equal')

        axOverallErrorMbs = fig1.add_subplot(2, 3, 5)
        axOverallErrorMbs.set_title("Overall error macroblocks", color='0.4')
        axOverallErrorMbs.pie([overallMissingConcealedMbCount, overallMissingMbCount, overallErrorPropagationMbCount, overallUnknownMbCount], labels=['Missing (concealed)', 'Missing', 'Error propagation', 'Unknown'], colors=['salmon', 'firebrick', 'cadetblue', '0.3'], autopct='%1.1f%%', startangle=90)
        axOverallErrorMbs.set_aspect('equal')


        #####################
        #  Overall timings  #
        #####################

        axTimings = fig1.add_subplot(2, 3, 6)
        axTimings.set_title("Overall timings", color='0.4')
        width = 0.8
        axTimings.bar(1 - width / 2, overallTimestampDelta, width, yerr=overallTimestampDeltaStd, color='mediumseagreen')
        axTimings.bar(2 - width / 2, overallTimingError, width, yerr=overallTimingErrorStd, color='firebrick')
        axTimings.bar(3 - width / 2, overallEstimatedLatency, width, yerr=overallEstimatedLatencyStd, color='cadetblue')
        axTimings.set_xticks([1, 2, 3])
        axTimings.set_xticklabels(['Time delta', 'Output error', 'Est. latency'])
        axTimings.set_ylabel('(ms)')


        ###################
        #  Overall stats  #
        ###################

        axText = fig1.add_subplot(2, 3, 2)
        axText.axis('off')
        axText.set_title("Summary", color='0.4')
        axText.text(0., .7, 'Video ESR:\n' + '%.1f' % totalEsr + '% / ' + '%.1f' % overallEsrNear + '% / ' + '%.1f' % overallEsrFar + '%', color='0.4', fontsize=12)
        axText.text(0., .5, 'Errored frames:\n' + '%.1f' % overallErroredFrameRatio + '% / ' + '%.1f' % overallErroredFrameNear + '% / ' + '%.1f' % overallErroredFrameFar + '%', color='0.4', fontsize=12)
        axText.text(0., .3, 'Errored MBs:\n' + '%.1f' % overallErroredMbRatio + '% / ' + '%.1f' % overallErroredMbNear + '% / ' + '%.1f' % overallErroredMbFar + '%', color='0.4', fontsize=12)
        axText.text(0., .1, '(overall) / (RSSI>=-70) / (RSSI<-70)', color='0.4', fontsize=9)

    else:
        #################
        #  Full graphs  #
        #################

        fig1 = plt.figure(figsize=(1920. / 96., 1080. / 96.), dpi=96)
        plt.subplots_adjust(left=0.035, right=0.965, bottom=0.05, top=0.92)
        plt.suptitle('Video stats (' + title + ')', fontsize=16, color='0.4')


        ####################
        #  Overall frames  #
        ####################

        axOverallFrames = fig1.add_subplot(3, 6, 3)
        axOverallFrames.set_title("Overall frames", color='0.4')
        axOverallFrames.pie([overallNoErrorOutputFrameCount, overallErroredOutputFrameCount, overallDiscardedFrameCount, overallMissedFrameCount], labels=['Output (no errors)', 'Output (with errors)', 'Discarded', 'Missed'], colors=['lightgreen', 'cadetblue', 'salmon', 'firebrick'], autopct='%1.1f%%', startangle=90)
        axOverallFrames.set_aspect('equal')


        ########################
        #  Frames across time  #
        ########################

        axFramesByTime = fig1.add_subplot(3, 3, 1)
        axFramesByTime.set_title("Frames across time", color='0.4')
        axFramesByTime.plot(dataTime, noErrorOutputFrames, color='lightgreen')
        axFramesByTime.fill_between(dataTime, erroredOutputFrames, noErrorOutputFrames, facecolor='lightgreen', alpha=0.75)
        axFramesByTime.plot(dataTime, erroredOutputFrames, color='cadetblue')
        axFramesByTime.fill_between(dataTime, discardedFrames, erroredOutputFrames, facecolor='cadetblue', alpha=0.75)
        axFramesByTime.plot(dataTime, discardedFrames, color='salmon')
        axFramesByTime.fill_between(dataTime, missedFrames, discardedFrames, facecolor='salmon', alpha=0.75)
        axFramesByTime.plot(dataTime, missedFrames, color='firebrick')
        axFramesByTime.fill_between(dataTime, 0, missedFrames, facecolor='firebrick', alpha=0.75)
        axFramesByTime2 = axFramesByTime.twinx()
        axFramesByTime2.plot(dataTime, data['rssi'][1:], color='0.4')
        axFramesByTime.set_ylim(0, 100)
        axFramesByTime.set_xlim(0, np.amax(dataTime))
        axFramesByTime.set_xlabel('Time (s)')
        axFramesByTime.set_yticks([0, 20, 40, 60, 80, 100])
        axFramesByTime.set_ylabel('(%)')
        axFramesByTime2.set_ylim(-100, 0)
        axFramesByTime2.set_yticks([-100, -80, -60, -40, -20, 0])
        axFramesByTime2.set_ylabel('(dBm)')


        ####################
        #  Frames by RSSI  #
        ####################

        sum1 = np.add(missedFrameCountByRssi, discardedFrameCountByRssi)
        sum2 = np.add(sum1, erroredOutputFrameCountByRssi)
        axFramesByRssi = fig1.add_subplot(3, 3, 3)
        axFramesByRssi.set_title("Frames by RSSI", color='0.4')
        width = 0.45
        rssiVal = np.arange(rssiMin, rssiMax + 1)
        axFramesByRssi.bar(rssiVal - width, missedFrameCountByRssi * 100., width, color='firebrick')
        axFramesByRssi.bar(rssiVal - width, discardedFrameCountByRssi * 100., width, color='salmon', bottom=missedFrameCountByRssi*100.)
        axFramesByRssi.bar(rssiVal - width, erroredOutputFrameCountByRssi * 100., width, color='cadetblue', bottom=sum1*100.)
        axFramesByRssi.bar(rssiVal - width, noErrorOutputFrameCountByRssi * 100., width, color='lightgreen', bottom=sum2*100.)
        axFramesByRssi.bar(rssiVal, totalFrameCountByRssi * 100., width, color='0.4')
        rssiMax2 = rssiMin
        rssiMin2 = 0
        for i, val in enumerate(totalFrameCountByRssi):
            if val > 0:
                rssiMax2 = rssiVal[i]
                if rssiMin2 == 0:
                    rssiMin2 = rssiVal[i]
        axFramesByRssi.set_ylim(0, 100)
        axFramesByRssi.set_xlim(rssiMin2 - 1, rssiMax2 + 1)
        axFramesByRssi.set_xlabel('RSSI (dBm)')
        axFramesByRssi.set_ylabel('(%)')


        #################
        #  ESR by zone  #
        #################

        axZoneEsr = fig1.add_subplot(3, 6, 4)
        axZoneEsr.set_xlim(0, 100)
        axZoneEsr.set_title("ESR by zone", color='0.4')
        axZoneEsr.barh(zoneEsrYPos, zoneEsr, align='center', alpha=0.8, color='cadetblue')
        axZoneEsr.set_yticks(zoneEsrYPos)
        axZoneEsr.set_yticklabels(zoneEsrLabel)
        axZoneEsr.set_xlabel('ESR (%)')
        axZoneEsr.set_ylabel('Zone')


        #####################
        #  Timings by RSSI  #
        #####################

        axTimingsByRssi = fig1.add_subplot(3, 3, 6)
        axTimingsByRssi.set_title("Timings by RSSI", color='0.4')
        width = 0.24
        rssiVal = np.arange(rssiMin, rssiMax + 1)
        axTimingsByRssi.bar(rssiVal - width * 2, timestampDeltaByRssi, width, yerr=timestampDeltaByRssiStd, color='mediumseagreen')
        axTimingsByRssi.bar(rssiVal - width, timingErrorByRssi, width, yerr=timingErrorByRssiStd, color='firebrick')
        axTimingsByRssi.bar(rssiVal, estimatedLatencyByRssi, width, yerr=estimatedLatencyByRssiStd, color='cadetblue')
        axTimingsByRssi2 = axTimingsByRssi.twinx()
        axTimingsByRssi2.bar(rssiVal + width, outputFrameCountByRssi / float(data['outputFrameCount'].iat[-1]) * 100., width, color='0.4')
        rssiMax2 = rssiMin
        rssiMin2 = 0
        for i, val in enumerate(outputFrameCountByRssi):
            if val > 0:
                rssiMax2 = rssiVal[i]
                if rssiMin2 == 0:
                    rssiMin2 = rssiVal[i]
        axTimingsByRssi.set_xlim(rssiMin2 - 1, rssiMax2 + 1)
        axTimingsByRssi.set_xlabel('RSSI (dBm)')
        a, b = axTimingsByRssi.get_ylim()
        axTimingsByRssi.set_ylim(0, b)
        axTimingsByRssi2.set_ylim(0, 100)
        axTimingsByRssi2.set_yticks([0, 50, 100])
        axTimingsByRssi.set_ylabel('(ms)')
        axTimingsByRssi2.set_ylabel('(%)')


        #########################
        #  Timings across time  #
        #########################

        axTimingsByTime = fig1.add_subplot(3, 3, 4)
        axTimingsByTime.set_title("Timings across time", color='0.4')
        axTimingsByTime.plot(dataTime, timestampDelta, color='mediumseagreen')
        axTimingsByTime.fill_between(dataTime, timestampDelta, np.subtract(timestampDelta, timestampDeltaStd), color='mediumseagreen', facecolor='mediumseagreen', alpha=0.5)
        axTimingsByTime.fill_between(dataTime, timestampDelta, np.add(timestampDelta, timestampDeltaStd), color='mediumseagreen', facecolor='mediumseagreen', alpha=0.5)
        axTimingsByTime.plot(dataTime, timingError, color='firebrick')
        axTimingsByTime.fill_between(dataTime, timingError, np.subtract(timingError, timingErrorStd), color='firebrick', facecolor='firebrick', alpha=0.5)
        axTimingsByTime.fill_between(dataTime, timingError, np.add(timingError, timingErrorStd), color='firebrick', facecolor='firebrick', alpha=0.5)
        axTimingsByTime.plot(dataTime, estimatedLatency, color='cadetblue')
        axTimingsByTime.fill_between(dataTime, estimatedLatency, np.subtract(estimatedLatency, estimatedLatencyStd), color='cadetblue', facecolor='cadetblue', alpha=0.5)
        axTimingsByTime.fill_between(dataTime, estimatedLatency, np.add(estimatedLatency, estimatedLatencyStd), color='cadetblue', facecolor='cadetblue', alpha=0.5)
        axTimingsByTime2 = axTimingsByTime.twinx()
        axTimingsByTime2.plot(dataTime, data['rssi'][1:], color='0.4')
        axTimingsByTime.set_xlim(0, np.amax(dataTime))
        axTimingsByTime.set_xlabel('Time (s)')
        a, b = axTimingsByRssi.get_ylim()
        axTimingsByTime.set_ylim(0, b)
        axTimingsByTime.set_ylabel('(ms)')
        axTimingsByTime2.set_ylim(-100, 0)
        axTimingsByTime2.set_yticks([-100, -50, 0])
        axTimingsByTime2.set_ylabel('(dBm)')


        #####################
        #  Overall timings  #
        #####################

        axTimings = fig1.add_subplot(3, 6, 10)
        axTimings.set_title("Overall timings", color='0.4')
        width = 0.8
        axTimings.bar(1 - width / 2, overallTimestampDelta, width, yerr=overallTimestampDeltaStd, color='mediumseagreen')
        axTimings.bar(2 - width / 2, overallTimingError, width, yerr=overallTimingErrorStd, color='firebrick')
        axTimings.bar(3 - width / 2, overallEstimatedLatency, width, yerr=overallEstimatedLatencyStd, color='cadetblue')
        axTimings.set_xticks([1, 2, 3])
        axTimings.set_xticklabels(['Time delta', 'Output error', 'Est. latency'])
        a, b = axTimingsByRssi.get_ylim()
        axTimings.set_ylim(a, b)
        axTimings.set_ylabel('(ms)')


        #########################
        #  Overall macroblocks  #
        #########################

        axOverallMbs = fig1.add_subplot(3, 6, 15)
        axOverallMbs.set_title("Overall macroblocks", color='0.4')
        axOverallMbs.pie([overallValidISliceMbCount, overallValidPSliceMbCount, overallMissingConcealedMbCount, overallMissingMbCount, overallErrorPropagationMbCount, overallUnknownMbCount], labels=['Valid (I)', 'Valid (P)', 'Missing (concealed)', 'Missing', 'Error propagation', 'Unknown'], colors=['mediumseagreen', 'lightgreen', 'salmon', 'firebrick', 'cadetblue', '0.3'], autopct='%1.1f%%', startangle=90)
        axOverallMbs.set_aspect('equal')

        axOverallErrorMbs = fig1.add_subplot(3, 6, 16)
        axOverallErrorMbs.set_title("Overall error macroblocks", color='0.4')
        axOverallErrorMbs.pie([overallMissingConcealedMbCount, overallMissingMbCount, overallErrorPropagationMbCount, overallUnknownMbCount], labels=['Missing (concealed)', 'Missing', 'Error propagation', 'Unknown'], colors=['salmon', 'firebrick', 'cadetblue', '0.3'], autopct='%1.1f%%', startangle=90)
        axOverallErrorMbs.set_aspect('equal')


        #############################
        #  Macroblocks across time  #
        #############################

        axMacroblocksByTime = fig1.add_subplot(3, 3, 7)
        axMacroblocksByTime.set_title("Macroblocks across time", color='0.4')
        axMacroblocksByTime.plot(dataTime, validISliceMb, color='mediumseagreen')
        axMacroblocksByTime.fill_between(dataTime, validPSliceMb, validISliceMb, facecolor='mediumseagreen', alpha=0.75)
        axMacroblocksByTime.plot(dataTime, validPSliceMb, color='lightgreen')
        axMacroblocksByTime.fill_between(dataTime, errorPropagationMb, validPSliceMb, facecolor='lightgreen', alpha=0.75)
        axMacroblocksByTime.plot(dataTime, errorPropagationMb, color='cadetblue')
        axMacroblocksByTime.fill_between(dataTime, missingConcealedMb, errorPropagationMb, facecolor='cadetblue', alpha=0.75)
        axMacroblocksByTime.plot(dataTime, missingConcealedMb, color='salmon')
        axMacroblocksByTime.fill_between(dataTime, missingMb, missingConcealedMb, facecolor='salmon', alpha=0.75)
        axMacroblocksByTime.plot(dataTime, missingMb, color='firebrick')
        axMacroblocksByTime.fill_between(dataTime, 0, missingMb, facecolor='firebrick', alpha=0.75)
        axMacroblocksByTime2 = axMacroblocksByTime.twinx()
        axMacroblocksByTime2.plot(dataTime, data['rssi'][1:], color='0.4')
        axMacroblocksByTime.set_ylim(0, 100)
        axMacroblocksByTime.set_xlim(0, np.amax(dataTime))
        axMacroblocksByTime.set_xlabel('Time (s)')
        axMacroblocksByTime.set_yticks([0, 20, 40, 60, 80, 100])
        axMacroblocksByTime.set_ylabel('(%)')
        axMacroblocksByTime2.set_ylim(-100, 0)
        axMacroblocksByTime2.set_yticks([-100, -80, -60, -40, -20, 0])
        axMacroblocksByTime2.set_ylabel('(dBm)')


        #########################
        #  Macroblocks by RSSI  #
        #########################

        sum1 = np.add(unknownMbByRssi, missingMbByRssi)
        sum2 = np.add(sum1, missingConcealedMbByRssi)
        sum3 = np.add(sum2, errorPropagationMbByRssi)
        sum4 = np.add(sum3, validPSliceMbByRssi)

        axMacroblocksByRssi = fig1.add_subplot(3, 3, 9)
        axMacroblocksByRssi.set_title("Macroblocks by RSSI", color='0.4')
        width = 0.45
        axMacroblocksByRssi.bar(rssiVal - width, unknownMbByRssi * 100., width, color='0.3')
        axMacroblocksByRssi.bar(rssiVal - width, missingMbByRssi * 100., width, color='firebrick', bottom=unknownMbByRssi*100.)
        axMacroblocksByRssi.bar(rssiVal - width, missingConcealedMbByRssi * 100., width, color='salmon', bottom=sum1*100.)
        axMacroblocksByRssi.bar(rssiVal - width, errorPropagationMbByRssi * 100., width, color='cadetblue', bottom=sum2*100.)
        axMacroblocksByRssi.bar(rssiVal - width, validPSliceMbByRssi * 100., width, color='lightgreen', bottom=sum3*100.)
        axMacroblocksByRssi.bar(rssiVal - width, validISliceMbByRssi * 100., width, color='mediumseagreen', bottom=sum4*100.)
        axMacroblocksByRssi.bar(rssiVal, totalMbCountByRssi * 100., width, color='0.4')
        rssiMax2 = rssiMin
        rssiMin2 = 0
        for i, val in enumerate(totalMbCountByRssi):
            if val > 0:
                rssiMax2 = rssiVal[i]
                if rssiMin2 == 0:
                    rssiMin2 = rssiVal[i]
        axMacroblocksByRssi.set_ylim(0, 100)
        axMacroblocksByRssi.set_xlim(rssiMin2 - 1, rssiMax2 + 1)
        axMacroblocksByRssi.set_xlabel('RSSI (dBm)')
        axMacroblocksByRssi.set_ylabel('(%)')


        ###################
        #  Overall stats  #
        ###################

        axText = fig1.add_subplot(3, 6, 9)
        axText.axis('off')
        axText.set_title("Summary", color='0.4')
        axText.text(0., .7, 'Video ESR:\n' + '%.1f' % totalEsr + '% / ' + '%.1f' % overallEsrNear + '% / ' + '%.1f' % overallEsrFar + '%', color='0.4', fontsize=12)
        axText.text(0., .5, 'Errored frames:\n' + '%.1f' % overallErroredFrameRatio + '% / ' + '%.1f' % overallErroredFrameNear + '% / ' + '%.1f' % overallErroredFrameFar + '%', color='0.4', fontsize=12)
        axText.text(0., .3, 'Errored MBs:\n' + '%.1f' % overallErroredMbRatio + '% / ' + '%.1f' % overallErroredMbNear + '% / ' + '%.1f' % overallErroredMbFar + '%', color='0.4', fontsize=12)
        axText.text(0., .1, '(overall) / (RSSI>=-70) / (RSSI<-70)', color='0.4', fontsize=9)

    plt.draw()
    if outFile != '':
        plt.savefig(outFile)
    else:
        plt.show()


def usage():
    print "Usage:"
    print "    " + sys.argv[0] + " -i | --infile <input_file>"
    print "Options:"
    print "    -o | --outfile <output_file>    Output to file instead of GUI"
    print "    -s | --simple                   Simple output (less graphs)"
    print "    -h | --help                     Print this message"


def main(argv):
    inFile = ''
    outFile = ''
    simple = False

    # command-line arguments
    try:
        opts, args = getopt.getopt(argv,"hsi:o:",["help", "infile=", "outfile=", "simple"])
    except getopt.GetoptError:
        usage()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            usage()
            sys.exit()
        elif opt in ("-i", "--infile"):
            inFile = arg
        elif opt in ("-o", "--outfile"):
            outFile = arg
        elif opt in ("-s", "--simple"):
            simple = True

    if inFile == '':
        usage()
        sys.exit(2)

    if inFile != '':
        videoStats(inFile, outFile, simple)


if __name__ == '__main__':
    main(sys.argv[1:])
