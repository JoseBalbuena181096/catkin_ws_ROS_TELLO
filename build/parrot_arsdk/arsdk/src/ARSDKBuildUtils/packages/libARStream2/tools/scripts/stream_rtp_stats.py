#!/usr/bin/python

# @file stream_rtp_stats.py
# @brief Streaming rtp stats script
# @date 09/21/2016
# @author aurelien.barre@parrot.com


import sys, getopt, math
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


################
#  Full stats  #
################

def rtpStats(statsFile, lossFile, outFile, simple):
    # stats file
    f = open(statsFile, 'r')
    firstLine = f.readline()
    f.close()
    if firstLine != '' and firstLine[0] == '#':
        title = firstLine[1:]
        title = title.strip()
    else:
        title = statsFile
    if firstLine != '' and firstLine.find(',') != -1:
        sep = ','
    else:
        sep = ' '

    data = pd.read_csv(statsFile, sep=sep, comment='#', skip_blank_lines=True)


    # loss file
    if lossFile != '':
        f = open(lossFile, 'r')
        firstLine = f.readline()
        f.close()
        if firstLine != '' and firstLine.find(',') != -1:
            sep = ','
        else:
            sep = ' '

        dataLoss = pd.read_csv(lossFile, sep=sep, comment='#', skip_blank_lines=True)


    ##################
    #  Sender stats  #
    ##################

    # TODO: ignore lines with senderStatsTimestamp == 0
    firstSenderStatsTime = data['senderStatsTimestamp'][1]
    dataSenderStatsTime = []
    dataSenderStatsTime.append(0.)
    dataSenderStatsTimeDelta = []
    dataSenderStatsTimeDelta.append(0.)
    for i, val in enumerate(data['senderStatsTimestamp']):
        if i > 0:
            dataSenderStatsTime.append((float(data['senderStatsTimestamp'][i]) - firstSenderStatsTime) / 1000000.)
            dataSenderStatsTimeDelta.append((float(data['senderStatsTimestamp'][i]) - float(data['senderStatsTimestamp'][i - 1])) / 1000000.)

    dataSenderStatsSentPacketCount = np.subtract(data['senderStatsSentPacketCount'][1:], data['senderStatsSentPacketCount'][:-1])
    dataSenderStatsSentPacketCount[0] = 0
    dataSenderStatsDroppedPacketCount = np.subtract(data['senderStatsDroppedPacketCount'][1:], data['senderStatsDroppedPacketCount'][:-1])
    dataSenderStatsDroppedPacketCount[0] = 0
    dataSenderStatsSentByteCount = np.subtract(data['senderStatsSentByteIntegral'][1:], data['senderStatsSentByteIntegral'][:-1])
    dataSenderStatsSentByteCount[0] = 0
    dataSenderStatsSentByteCountSq = np.subtract(data['senderStatsSentByteIntegralSq'][1:], data['senderStatsSentByteIntegralSq'][:-1])
    dataSenderStatsSentByteCountSq[0] = 0
    dataSenderStatsDroppedByteCount = np.subtract(data['senderStatsDroppedByteIntegral'][1:], data['senderStatsDroppedByteIntegral'][:-1])
    dataSenderStatsDroppedByteCount[0] = 0
    dataSenderStatsDroppedByteCountSq = np.subtract(data['senderStatsDroppedByteIntegralSq'][1:], data['senderStatsDroppedByteIntegralSq'][:-1])
    dataSenderStatsDroppedByteCountSq[0] = 0
    dataSenderStatsInputToSentTime = np.subtract(data['senderStatsInputToSentTimeIntegral'][1:], data['senderStatsInputToSentTimeIntegral'][:-1])
    dataSenderStatsInputToSentTime[0] = 0
    dataSenderStatsInputToSentTimeSq = np.subtract(data['senderStatsInputToSentTimeIntegralSq'][1:], data['senderStatsInputToSentTimeIntegralSq'][:-1])
    dataSenderStatsInputToSentTimeSq[0] = 0
    dataSenderStatsInputToDroppedTime = np.subtract(data['senderStatsInputToDroppedTimeIntegral'][1:], data['senderStatsInputToDroppedTimeIntegral'][:-1])
    dataSenderStatsInputToDroppedTime[0] = 0
    dataSenderStatsInputToDroppedTimeSq = np.subtract(data['senderStatsInputToDroppedTimeIntegralSq'][1:], data['senderStatsInputToDroppedTimeIntegralSq'][:-1])
    dataSenderStatsInputToDroppedTimeSq[0] = 0

    dataSenderStatsSentBitrate = []
    dataSenderStatsDroppedBitrate = []
    dataSenderStatsSentPacketRate = []
    dataSenderStatsDroppedPacketRate = []
    for i, val in enumerate(dataSenderStatsTimeDelta):
        if dataSenderStatsTimeDelta[i] != 0:
            dataSenderStatsSentBitrate.append(float(dataSenderStatsSentByteCount[i]) / dataSenderStatsTimeDelta[i] * 8. / 1000.)
            dataSenderStatsDroppedBitrate.append(float(dataSenderStatsDroppedByteCount[i]) / dataSenderStatsTimeDelta[i] * 8. / 1000.)
            dataSenderStatsSentPacketRate.append(float(dataSenderStatsSentPacketCount[i]) / dataSenderStatsTimeDelta[i])
            dataSenderStatsDroppedPacketRate.append(float(dataSenderStatsDroppedPacketCount[i]) / dataSenderStatsTimeDelta[i])
        else:
            dataSenderStatsSentBitrate.append(0.)
            dataSenderStatsDroppedBitrate.append(0.)
            dataSenderStatsSentPacketRate.append(0.)
            dataSenderStatsDroppedPacketRate.append(0.)

    dataSenderStatsSentPacketSizeMean = []
    dataSenderStatsSentPacketSizeStd = []
    for i, val in enumerate(dataSenderStatsSentPacketCount):
        if dataSenderStatsSentPacketCount[i] != 0:
            dataSenderStatsSentPacketSizeMean.append(float(dataSenderStatsSentByteCount[i]) / dataSenderStatsSentPacketCount[i])
            dataSenderStatsSentPacketSizeStd.append(np.sqrt(np.abs(float(dataSenderStatsSentByteCountSq[i]) / dataSenderStatsSentPacketCount[i] - (dataSenderStatsSentPacketSizeMean[-1] * dataSenderStatsSentPacketSizeMean[-1]))))
        else:
            dataSenderStatsSentPacketSizeMean.append(0.)
            dataSenderStatsSentPacketSizeStd.append(0.)

    dataSenderStatsDroppedPacketSize = []
    dataSenderStatsDroppedPacketSizeStd = []
    for i, val in enumerate(dataSenderStatsDroppedPacketCount):
        if dataSenderStatsDroppedByteCount[i] != 0:
            dataSenderStatsDroppedPacketSize.append(float(dataSenderStatsDroppedByteCount[i]) / dataSenderStatsDroppedPacketCount[i])
            dataSenderStatsDroppedPacketSizeStd.append(np.sqrt(np.abs(float(dataSenderStatsDroppedByteCountSq[i]) / dataSenderStatsDroppedPacketCount[i] - (dataSenderStatsDroppedPacketSize[-1] * dataSenderStatsDroppedPacketSize[-1]))))
        else:
            dataSenderStatsDroppedPacketSize.append(0.)
            dataSenderStatsDroppedPacketSizeStd.append(0.)

    dataSenderStatsInputToSentTimeMean = []
    dataSenderStatsInputToSentTimeStd = []
    for i, val in enumerate(dataSenderStatsSentPacketCount):
        if dataSenderStatsSentPacketCount[i] != 0:
            dataSenderStatsInputToSentTimeMean.append(float(dataSenderStatsInputToSentTime[i]) / dataSenderStatsSentPacketCount[i] / 1000.)
            dataSenderStatsInputToSentTimeStd.append(np.sqrt(np.abs(float(dataSenderStatsInputToSentTimeSq[i]) / dataSenderStatsSentPacketCount[i] - (dataSenderStatsInputToSentTimeMean[-1] * dataSenderStatsInputToSentTimeMean[-1]))) / 1000.)
        else:
            dataSenderStatsInputToSentTimeMean.append(0.)
            dataSenderStatsInputToSentTimeStd.append(0.)

    dataSenderStatsInputToDroppedTimeMean = []
    dataSenderStatsInputToDroppedTimeStd = []
    for i, val in enumerate(dataSenderStatsDroppedPacketCount):
        if dataSenderStatsDroppedPacketCount[i] != 0:
            dataSenderStatsInputToDroppedTimeMean.append(float(dataSenderStatsInputToDroppedTime[i]) / dataSenderStatsDroppedPacketCount[i] / 1000.)
            dataSenderStatsInputToDroppedTimeStd.append(np.sqrt(np.abs(float(dataSenderStatsInputToDroppedTimeSq[i]) / dataSenderStatsDroppedPacketCount[i] - (dataSenderStatsInputToDroppedTimeMean[-1] * dataSenderStatsInputToDroppedTimeMean[-1]))) / 1000.)
        else:
            dataSenderStatsInputToDroppedTimeMean.append(0.)
            dataSenderStatsInputToDroppedTimeStd.append(0.)

    ####################
    #  Sender reports  #
    ####################

    # TODO: ignore lines with senderReportTimestamp == 0
    firstSenderReportTime = data['senderReportTimestamp'][1]
    dataSenderReportTime = []
    dataSenderReportTime.append(0.)
    for i, val in enumerate(data['senderReportTimestamp']):
        if i > 0:
            dataSenderReportTime.append((float(data['senderReportTimestamp'][i]) - firstSenderReportTime) / 1000000.)
    dataSenderReportPacketRate = []
    for i, val in enumerate(data['senderReportTimestamp']):
        if data['senderReportLastInterval'][i] > 0:
            dataSenderReportPacketRate.append((float(data['senderReportIntervalPacketCount'][i]) / float(data['senderReportLastInterval'][i]) * 1000000.))
        else:
            dataSenderReportPacketRate.append(0.)
    dataSenderReportBitrate = []
    for i, val in enumerate(data['senderReportTimestamp']):
        if data['senderReportLastInterval'][i] > 0:
            dataSenderReportBitrate.append((float(data['senderReportIntervalByteCount'][i]) / float(data['senderReportLastInterval'][i]) * 8. * 1000.))
        else:
            dataSenderReportBitrate.append(0.)


    ######################
    #  Receiver reports  #
    ######################

    # TODO: ignore lines with receiverReportTimestamp == 0
    firstReceiverReportTime = data['receiverReportTimestamp'][1]
    dataReceiverReportTime = []
    dataReceiverReportTime.append(0.)
    dataReceiverReportTimeDelta = []
    dataReceiverReportTimeDelta.append(0.)
    for i, val in enumerate(data['receiverReportTimestamp']):
        if i > 0:
            dataReceiverReportTime.append((float(data['receiverReportTimestamp'][i]) - firstReceiverReportTime) / 1000000.)
            dataReceiverReportTimeDelta.append((float(data['receiverReportTimestamp'][i]) - float(data['receiverReportTimestamp'][i - 1])) / 1000000.)

    dataReceiverReportTotalPacketCount = np.subtract(data['receiverReportReceiverExtHighestSeqNum'][1:], data['receiverReportReceiverExtHighestSeqNum'][:-1])
    dataReceiverReportTotalPacketCount[0] = 0
    dataReceiverReportLostPacketCount = np.subtract(data['receiverReportReceiverLostCount'][1:], data['receiverReportReceiverLostCount'][:-1])
    dataReceiverReportLostPacketCount[0] = 0

    dataReceiverReportTotalPacketRate = []
    dataReceiverReportLostPacketRate = []
    for i, val in enumerate(dataReceiverReportTimeDelta):
        if dataReceiverReportTimeDelta[i] != 0:
            dataReceiverReportTotalPacketRate.append(float(dataReceiverReportTotalPacketCount[i]) / dataReceiverReportTimeDelta[i])
            dataReceiverReportLostPacketRate.append(float(dataReceiverReportLostPacketCount[i]) / dataReceiverReportTimeDelta[i])
        else:
            dataReceiverReportTotalPacketRate.append(0.)
            dataReceiverReportLostPacketRate.append(0.)

    #####################################
    #  De-jitter buffer metrics report  #
    #####################################

    # TODO: ignore lines with djbMetricsReportTimestamp == 0
    firstDjbMetricsReportTime = data['djbMetricsReportTimestamp'][1]
    dataDjbMetricsReportTime = []
    dataDjbMetricsReportTime.append(0.)
    for i, val in enumerate(data['djbMetricsReportTimestamp']):
        if i > 0:
            dataDjbMetricsReportTime.append((float(data['djbMetricsReportTimestamp'][i]) - firstDjbMetricsReportTime) / 1000000.)


    #####################
    #  RTP loss report  #
    #####################

    if lossFile != '':
        if 'receivedFlag_0' in dataLoss.columns:
            # CSV file from telemetry
            dataReceivedFlag = []
            for i, val in enumerate(dataLoss['time_us']):
                strReceived = ""
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_0'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_1'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_2'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_3'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_4'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_5'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_6'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_7'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_8'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_9'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_10'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_11'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_12'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_13'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_14'][i])
                strReceived = strReceived + '{0:032b}'.format(dataLoss['receivedFlag_15'][i])
                startSeq = dataLoss['startSeqNum'][i]
                endSeq = dataLoss['endSeqNum'][i]
                count = endSeq - startSeq + 1
                if count <= 0:
                    count = count + 65536
                dataReceivedFlag.append(strReceived[:count])
        else:
            dataReceivedFlag = dataLoss['receivedFlag']

        maxCount = 1000
        recv = []
        loss = []
        recvCount = 0
        lossCount = 0
        prevBitVal = '0'
        prevEndSeq = -1

        for i, val in enumerate(dataReceivedFlag):
            startSeq = dataLoss['startSeqNum'][i]
            endSeq = dataLoss['endSeqNum'][i]
            count = endSeq - startSeq + 1
            if count <= 0:
                count = count + 65536
            gap = startSeq - prevEndSeq
            if gap <= 0:
                gap = gap + 65536
            prevEndSeq = endSeq

            # Handle gaps in seqNums between lines (ignore TS intervals > 1s because it is probably a lost report packet)
            if gap > 1 and i > 0 and dataLoss['timestamp'][i] - dataLoss['timestamp'][i - 1] < 1000000:
                bitVal = '0'
                lossCount += (gap - 1)
                if bitVal != prevBitVal and recvCount != 0:
                    while recvCount - maxCount > 0:
                        recv.append(maxCount)
                        recvCount = recvCount - maxCount
                    recv.append(recvCount)
                    recvCount = 0
                prevBitVal = bitVal

            # Handle the line
            for j, bitVal in enumerate(str(val)):
                if bitVal == '0':
                    lossCount += 1
                    if bitVal != prevBitVal and recvCount != 0:
                        while recvCount - maxCount > 0:
                            recv.append(maxCount)
                            recvCount = recvCount - maxCount
                        recv.append(recvCount)
                        recvCount = 0
                else:
                    recvCount += 1
                    if bitVal != prevBitVal and lossCount != 0:
                        while lossCount - maxCount > 0:
                            loss.append(maxCount)
                            lossCount = lossCount - maxCount
                        loss.append(lossCount)
                        lossCount = 0
                prevBitVal = bitVal

        # Last values
        while recvCount - maxCount > 0:
            recv.append(maxCount)
            recvCount = recvCount - maxCount
        recv.append(recvCount)
        while lossCount - maxCount > 0:
            loss.append(maxCount)
            lossCount = lossCount - maxCount
        loss.append(lossCount)


    if simple:
        ###################
        #  Simple graphs  #
        ###################

        fig1 = plt.figure(figsize=(1000. / 96., 800. / 96.), dpi=96)
        plt.subplots_adjust(left=0.03, right=0.97, bottom=0.03, top=0.92)
        plt.suptitle('RTP stats (' + title + ')', fontsize=16, color='0.4')

        #TODO

    else:
        #################
        #  Full graphs  #
        #################

        fig1 = plt.figure(figsize=(1920. / 96., 1080. / 96.), dpi=96)
        plt.subplots_adjust(left=0.035, right=0.965, bottom=0.05, top=0.92)
        plt.suptitle('RTP stats (' + title + ')', fontsize=16, color='0.4')

        axPackets = fig1.add_subplot(3, 3, 4)
        axPackets.set_title("Packets", color='0.4')
        axPackets.plot(dataReceiverReportTime, dataReceiverReportTotalPacketRate, color='mediumseagreen')
        axPackets.plot(dataSenderReportTime, dataSenderReportPacketRate, color='lightgreen')
        axPackets.plot(dataSenderStatsTime, dataSenderStatsSentPacketRate, color='cadetblue')
        axPackets.plot(dataReceiverReportTime, dataReceiverReportLostPacketRate, color='salmon')
        axPackets.plot(dataSenderStatsTime, dataSenderStatsDroppedPacketRate, color='firebrick')
        axPackets.set_xlabel('Time (s)')
        axPackets.set_ylabel('Packet/s')

        axBytes = fig1.add_subplot(3, 3, 1)
        axBytes.set_title("Bitrates", color='0.4')
        axBytes.plot(dataSenderStatsTime, np.add(dataSenderStatsSentBitrate, dataSenderStatsDroppedBitrate), color='mediumseagreen')
        axBytes.plot(dataSenderReportTime, dataSenderReportBitrate, color='lightgreen')
        axBytes.plot(dataSenderStatsTime, dataSenderStatsSentBitrate, color='cadetblue')
        axBytes.plot(dataSenderStatsTime, dataSenderStatsDroppedBitrate, color='salmon')
        axBytes.set_xlabel('Time (s)')
        axBytes.set_ylabel('Kbit/s')

        axPacketSize = fig1.add_subplot(3, 3, 7)
        axPacketSize.set_title("Packet size", color='0.4')
        axPacketSize.plot(dataSenderStatsTime, dataSenderStatsSentPacketSizeMean, color='mediumseagreen')
        axPacketSize.fill_between(dataSenderStatsTime, dataSenderStatsSentPacketSizeMean, np.subtract(dataSenderStatsSentPacketSizeMean, dataSenderStatsSentPacketSizeStd), color='mediumseagreen', facecolor='mediumseagreen', alpha=0.5)
        axPacketSize.fill_between(dataSenderStatsTime, dataSenderStatsSentPacketSizeMean, np.add(dataSenderStatsSentPacketSizeMean, dataSenderStatsSentPacketSizeStd), color='mediumseagreen', facecolor='mediumseagreen', alpha=0.5)
        axPacketSize.plot(dataSenderStatsTime, dataSenderStatsDroppedPacketSize, color='salmon')
        axPacketSize.fill_between(dataSenderStatsTime, dataSenderStatsDroppedPacketSize, np.subtract(dataSenderStatsDroppedPacketSize, dataSenderStatsDroppedPacketSizeStd), color='salmon', facecolor='salmon', alpha=0.5)
        axPacketSize.fill_between(dataSenderStatsTime, dataSenderStatsDroppedPacketSize, np.add(dataSenderStatsDroppedPacketSize, dataSenderStatsDroppedPacketSizeStd), color='salmon', facecolor='salmon', alpha=0.5)
        axPacketSize.set_xlabel('Time (s)')
        axPacketSize.set_ylabel('Packet size (bytes)')

        axTimings = fig1.add_subplot(3, 3, 3)
        axTimings.set_title("Timings", color='0.4')
        axTimings.plot(dataSenderStatsTime, dataSenderStatsInputToSentTimeMean, color='mediumseagreen')
        axTimings.fill_between(dataSenderStatsTime, dataSenderStatsInputToSentTimeMean, np.subtract(dataSenderStatsInputToSentTimeMean, dataSenderStatsInputToSentTimeStd), color='mediumseagreen', facecolor='mediumseagreen', alpha=0.5)
        axTimings.fill_between(dataSenderStatsTime, dataSenderStatsInputToSentTimeMean, np.add(dataSenderStatsInputToSentTimeMean, dataSenderStatsInputToSentTimeStd), color='mediumseagreen', facecolor='mediumseagreen', alpha=0.5)
        axTimings.plot(dataSenderStatsTime, dataSenderStatsInputToDroppedTimeMean, color='salmon')
        axTimings.fill_between(dataSenderStatsTime, dataSenderStatsInputToDroppedTimeMean, np.subtract(dataSenderStatsInputToDroppedTimeMean, dataSenderStatsInputToDroppedTimeStd), color='salmon', facecolor='salmon', alpha=0.5)
        axTimings.fill_between(dataSenderStatsTime, dataSenderStatsInputToDroppedTimeMean, np.add(dataSenderStatsInputToDroppedTimeMean, dataSenderStatsInputToDroppedTimeStd), color='salmon', facecolor='salmon', alpha=0.5)
        axTimings.set_xlabel('Time (s)')
        axTimings.set_ylabel('Input to output time (ms)')

        axDelays = fig1.add_subplot(3, 3, 6)
        axDelays.set_title("Delays", color='0.4')
        axDelays.plot(dataReceiverReportTime, data['receiverReportRoundTripDelay'] / 1000., color='mediumseagreen')
        axDelays.plot(dataReceiverReportTime, data['clockDeltaRoundTripDelay'] / 1000., color='cadetblue')
        axDelays.plot(dataReceiverReportTime, data['clockDeltaPeer2meDelay'] / 1000., color='0.4')
        axDelays.plot(dataReceiverReportTime, data['clockDeltaMe2peerDelay'] / 1000., color='0.6')
        axDelays.plot(dataReceiverReportTime, data['receiverReportInterarrivalJitter'] / 1000., color='salmon')
        axDelays.set_xlabel('Time (s)')
        axDelays.set_ylabel('Delay (ms)')

        axDjb = fig1.add_subplot(3, 3, 9)
        axDjb.set_title("De-jitter buffer", color='0.4')
        axDjb.plot(dataDjbMetricsReportTime, data['djbMetricsReportDjbNominal'] / 1000., color='mediumseagreen')
        axDjb.plot(dataDjbMetricsReportTime, data['djbMetricsReportDjbMax'] / 1000., color='cadetblue')
        axDjb.plot(dataDjbMetricsReportTime, data['djbMetricsReportDjbLowWatermark'] / 1000., color='salmon')
        axDjb.plot(dataDjbMetricsReportTime, data['djbMetricsReportDjbHighWatermark'] / 1000., color='firebrick')
        axDjb.set_xlabel('Time (s)')
        axDjb.set_ylabel('DJB delay (ms)')

        if lossFile != '':
            axRecv = fig1.add_subplot(3, 3, 5)
            axRecv.set_title("Reception", color='0.4')
            recvBins = min(np.max(recv), 200)
            if recvBins > 0:
                axRecv.hist(recv, recvBins, color='mediumseagreen')

            axLoss = fig1.add_subplot(3, 3, 8)
            axLoss.set_title("Loss", color='0.4')
            lossBins = min(np.max(loss), 200)
            if lossBins > 0:
                axLoss.hist(loss, lossBins, color='firebrick')


    plt.draw()
    if outFile != '':
        plt.savefig(outFile)
    else:
        plt.show()


def usage():
    print "Usage:"
    print "    " + sys.argv[0] + " -i | --statsfile <input_file>"
    print "Options:"
    print "    -o | --outfile <output_file>    Output to file instead of GUI"
    print "    -s | --simple                   Simple output (less graphs)"
    print "    -h | --help                     Print this message"


def main(argv):
    statsFile = ''
    lossFile = ''
    outFile = ''
    simple = False

    # command-line arguments
    try:
        opts, args = getopt.getopt(argv,"hsi:l:o:",["help", "statsfile=", "lossfile=", "outfile=", "simple"])
    except getopt.GetoptError:
        usage()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            usage()
            sys.exit()
        elif opt in ("-i", "--statsfile"):
            statsFile = arg
        elif opt in ("-l", "--lossfile"):
            lossFile = arg
        elif opt in ("-o", "--outfile"):
            outFile = arg
        elif opt in ("-s", "--simple"):
            simple = True

    if statsFile == '':
        usage()
        sys.exit(2)

    if statsFile != '':
        rtpStats(statsFile, lossFile, outFile, simple)


if __name__ == '__main__':
    main(sys.argv[1:])
