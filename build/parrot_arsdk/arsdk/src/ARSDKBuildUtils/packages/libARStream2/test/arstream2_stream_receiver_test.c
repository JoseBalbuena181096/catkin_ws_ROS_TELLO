/**
 * @file arstream2_stream_receiver_test.h
 * @brief Parrot Streaming Library - Stream Receiver test program
 * @date 07/27/2016
 * @author aurelien.barre@parrot.com
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>

#include <libARSAL/ARSAL.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetwork/ARNetwork.h>
#include <libARNetworkAL/ARNetworkAL.h>
#include <libARCommands/ARCommands.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARStream2/arstream2_stream_receiver.h>
#include <json-c/json.h>

#include "arstream2_stream_receiver_test.h"


#define TAG "ARSTREAM2_StreamReceiver_Test"

#define BD_DISCOVERY_PORT 44444
#define BD_C2D_PORT 54321 // should be read from Json
#define BD_D2C_PORT 43210

#define BD_NET_CD_NONACK_ID 10
#define BD_NET_CD_ACK_ID 11
#define BD_NET_CD_EMERGENCY_ID 12
#define BD_NET_DC_NAVDATA_ID 127
#define BD_NET_DC_EVENT_ID 126

#define BD_CLIENT_STREAM_PORT 55004
#define BD_CLIENT_CONTROL_PORT 55005

#define BD_AU_BUFFER_SIZE (1920 * 1088 * 3 / 2)


static const char short_options[] = "hk:K:i:s:c:S:C:";


static const struct option
long_options[] = {
    { "help"            , no_argument        , NULL, 'h' },
    { "arsdk"           , required_argument  , NULL, 'k' },
    { "arsdk-start"     , required_argument  , NULL, 'K' },
    { "ip"              , required_argument  , NULL, 'i' },
    { "sstrmp"          , required_argument  , NULL, 's' },
    { "sctrlp"          , required_argument  , NULL, 'c' },
    { "cstrmp"          , required_argument  , NULL, 'S' },
    { "cctrlp"          , required_argument  , NULL, 'C' },
    { 0, 0, 0, 0 }
};


static ARNETWORK_IOBufferParam_t c2dParams[] = {
    /* Non-acknowledged commands. */
    {
        .ID = BD_NET_CD_NONACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 2,
        .dataCopyMaxSize = 128,
        .isOverwriting = 1,
    },
    /* Acknowledged commands. */
    {
        .ID = BD_NET_CD_ACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    /* Emergency commands. */
    {
        .ID = BD_NET_CD_EMERGENCY_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 10,
        .ackTimeoutMs = 100,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 1,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
};

static const size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);


static ARNETWORK_IOBufferParam_t d2cParams[] = {
    {
        .ID = BD_NET_DC_NAVDATA_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    {
        .ID = BD_NET_DC_EVENT_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
};

static const size_t numD2cParams = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);


static int commandBufferIds[] = {
    BD_NET_DC_NAVDATA_ID,
    BD_NET_DC_EVENT_ID,
};

static const size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);


static int stopping = 0;


void sighandler(int signum)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- Stopping... --");
    stopping = 1;
}


void *readerRun(void* data)
{
    BD_MANAGER_t *deviceManager = NULL;
    int bufferId = 0;
    int failed = 0;
    
    // Allocate some space for incoming data.
    const size_t maxLength = 128 * 1024;
    void *readData = malloc(maxLength);
    if (readData == NULL)
    {
        failed = 1;
    }
    
    if (!failed)
    {
        // get thread data.
        if (data != NULL)
        {
            bufferId = ((READER_THREAD_DATA_t *)data)->readerBufferId;
            deviceManager = ((READER_THREAD_DATA_t *)data)->deviceManager;
            
            if (deviceManager == NULL)
            {
                failed = 1;
            }
        }
        else
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        while (deviceManager->run)
        {
            eARNETWORK_ERROR netError = ARNETWORK_OK;
            int length = 0;
            int skip = 0;
            
            // read data
            netError = ARNETWORK_Manager_ReadDataWithTimeout(deviceManager->netManager, bufferId, readData, maxLength, &length, 1000);
            if (netError != ARNETWORK_OK)
            {
                if (netError != ARNETWORK_ERROR_BUFFER_EMPTY)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNETWORK_Manager_ReadDataWithTimeout () failed: %s", ARNETWORK_Error_ToString(netError));
                }
                skip = 1;
            }
            
            if (!skip)
            {
                // Forward data to the CommandsManager
                eARCOMMANDS_DECODER_ERROR cmdError = ARCOMMANDS_DECODER_OK;
                cmdError = ARCOMMANDS_Decoder_DecodeCommand(deviceManager->decoder, (uint8_t*)readData, length);
                if ((cmdError != ARCOMMANDS_DECODER_OK) && (cmdError != ARCOMMANDS_DECODER_ERROR_NO_CALLBACK))
                {
                    char msg[128];
                    ARCOMMANDS_Decoder_DescribeBuffer((uint8_t*)readData, length, msg, sizeof(msg));
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARCOMMANDS_Decoder_DecodeBuffer () failed: %d %s", cmdError, msg);
                }
            }
        }
    }
    
    if (readData != NULL)
    {
        free(readData);
        readData = NULL;
    }
    
    return NULL;
}


static void usage(int argc, char *argv[])
{
    printf("Usage: %s [options]\n"
            "Options:\n"
            "-h | --help                        Print this message\n"
            "-k | --arsdk <ip_address>          ARSDK connection to drone with its IP address\n"
            "-K | --arsdk-start <ip_address>    ARSDK connection to drone with its IP address (connect only, do not start stream)\n"
            "-i | --ip <ip_address>             Direct RTP/AVP reception with an IP address\n"
            "-s | --sstrmp <port>               Server stream port for direct RTP/AVP reception\n"
            "-c | --sctrlp <port>               Server control port for direct RTP/AVP reception\n"
            "-S | --cstrmp <port>               Client stream port for direct RTP/AVP reception\n"
            "-C | --cctrlp <port>               Client control port for direct RTP/AVP reception\n"
            "\n",
            argv[0]);
}


int main(int argc, char *argv[])
{
    /* local declarations */
    int failed = 0;
    int idx, c;
    BD_MANAGER_t *deviceManager;

    printf("ARStream2 Stream Receiver Test\n\n");

    if (argc < 2)
    {
        usage(argc, argv);
        exit(-1);
    }

    deviceManager = malloc(sizeof(BD_MANAGER_t));
    if (deviceManager != NULL)
    {
        // initialize deviceManager
        memset(deviceManager, 0, sizeof(BD_MANAGER_t));
        deviceManager->d2cPort = BD_D2C_PORT;
        deviceManager->c2dPort = BD_C2D_PORT; // Will be read from json
        deviceManager->arstream2ClientStreamPort = BD_CLIENT_STREAM_PORT;
        deviceManager->arstream2ClientControlPort = BD_CLIENT_CONTROL_PORT;
        deviceManager->run = 1;
    }
    else
    {
        failed = 1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "deviceManager alloc error!");
    }

    while ((c = getopt_long(argc, argv, short_options, long_options, &idx)) != -1)
    {
        switch (c)
        {
            case 0:
                break;

            case 'h':
                usage(argc, argv);
                exit(0);
                break;

            case 'k':
                strncpy(deviceManager->addr, optarg, sizeof(deviceManager->addr));
                deviceManager->arsdk = 1;
                deviceManager->startStream = 1;
                deviceManager->receiveStream = 1;
                break;

            case 'K':
                strncpy(deviceManager->addr, optarg, sizeof(deviceManager->addr));
                deviceManager->arsdk = 1;
                deviceManager->startStream = 1;
                break;

            case 'i':
                strncpy(deviceManager->addr, optarg, sizeof(deviceManager->addr));
                deviceManager->receiveStream = 1;
                break;

            case 's':
                sscanf(optarg, "%d", &deviceManager->arstream2ServerStreamPort);
                break;

            case 'c':
                sscanf(optarg, "%d", &deviceManager->arstream2ServerControlPort);
                break;

            case 'S':
                sscanf(optarg, "%d", &deviceManager->arstream2ClientStreamPort);
                break;

            case 'C':
                sscanf(optarg, "%d", &deviceManager->arstream2ClientControlPort);
                break;

            default:
                usage(argc, argv);
                exit(-1);
                break;
        }
    }


    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- Starting --");

    signal(SIGINT, sighandler);

    if (!failed)
    {
        deviceManager->auBufferSize = BD_AU_BUFFER_SIZE;
        deviceManager->auBuffer = malloc(deviceManager->auBufferSize);
        if (!deviceManager->auBuffer)
        {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "AU buffer alloc error!");
        }
    }

    if ((!failed) && (deviceManager->arsdk))
    {
        failed = ardiscoveryConnect(deviceManager);
    }

    if ((!failed) && (deviceManager->arsdk))
    {
        /* start */
        failed = startNetwork(deviceManager);
    }

    if ((!failed) && (deviceManager->receiveStream))
    {
        /* start */
        failed = startVideo(deviceManager);
    }

    if ((!failed) && (deviceManager->arsdk))
    {
        int cmdSend = 0;

        cmdSend = sendDateAndTime(deviceManager);
    }

    if ((!failed) && (deviceManager->arsdk))
    {
        int cmdSend = 0;

        cmdSend = sendAllStates(deviceManager);
    }

    if ((!failed) && (deviceManager->arsdk) && (deviceManager->startStream))
    {
        int cmdSend = 0;

        cmdSend = sendBeginStream(deviceManager);
    }

    if ((!failed) && (deviceManager->arsdk))
    {
        // create the decoder
        deviceManager->decoder = ARCOMMANDS_Decoder_NewDecoder(NULL);
        if (deviceManager->decoder == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to create decoder");
            failed = 1;
        }

        // allocate reader thread array.
        deviceManager->readerThreads = calloc(numOfCommandBufferIds, sizeof(ARSAL_Thread_t));
        
        if (deviceManager->readerThreads == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Allocation of reader threads failed");
            failed = 1;
        }

        if (!failed)
        {
            // allocate reader thread data array.
            deviceManager->readerThreadsData = calloc(numOfCommandBufferIds, sizeof(READER_THREAD_DATA_t));
            
            if (deviceManager->readerThreadsData == NULL)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Allocation of reader threads data failed");
                failed = 1;
            }
        }

        if (!failed)
        {
            // Create and start reader threads.
            size_t readerThreadIndex = 0;
            for (readerThreadIndex = 0 ; readerThreadIndex < numOfCommandBufferIds ; readerThreadIndex++)
            {
                // initialize reader thread data
                deviceManager->readerThreadsData[readerThreadIndex].deviceManager = deviceManager;
                deviceManager->readerThreadsData[readerThreadIndex].readerBufferId = commandBufferIds[readerThreadIndex];

                if (ARSAL_Thread_Create(&(deviceManager->readerThreads[readerThreadIndex]), readerRun, &(deviceManager->readerThreadsData[readerThreadIndex])) != 0)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of reader thread failed");
                    failed = 1;
                }
            }
        }
    }

    // Run until CTRL+C
    while (!stopping)
    {
        sleep(1);
    }

    if (deviceManager != NULL)
    {
        deviceManager->run = 0; // break threads loops

        /* stop */
        if (deviceManager->readerThreads != NULL)
        {
            // Stop reader Threads
            size_t readerThreadIndex = 0;
            for (readerThreadIndex = 0 ; readerThreadIndex < numOfCommandBufferIds ; readerThreadIndex++)
            {
                if (deviceManager->readerThreads[readerThreadIndex] != NULL)
                {
                    ARSAL_Thread_Join(deviceManager->readerThreads[readerThreadIndex], NULL);
                    ARSAL_Thread_Destroy(&(deviceManager->readerThreads[readerThreadIndex]));
                    deviceManager->readerThreads[readerThreadIndex] = NULL;
                }
            }
            
            // free reader thread array
            free(deviceManager->readerThreads);
            deviceManager->readerThreads = NULL;
        }
        
        if (deviceManager->readerThreadsData != NULL)
        {
            // free reader thread data array
            free(deviceManager->readerThreadsData);
            deviceManager->readerThreadsData = NULL;
        }

        ARCOMMANDS_Decoder_DeleteDecoder(&deviceManager->decoder);
        stopVideo(deviceManager);
        stopNetwork(deviceManager);
        free(deviceManager->auBuffer);
        free(deviceManager);
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- END --");

    return 0;
}


int ardiscoveryConnect(BD_MANAGER_t *deviceManager)
{
    int failed = 0;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- ARDiscovery Connection");

    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;
    ARDISCOVERY_Connection_ConnectionData_t *discoveryData = ARDISCOVERY_Connection_New(ARDISCOVERY_Connection_SendJsonCallback, ARDISCOVERY_Connection_ReceiveJsonCallback, deviceManager, &err);
    if (discoveryData == NULL || err != ARDISCOVERY_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while creating discoveryData: %s", ARDISCOVERY_Error_ToString(err));
        failed = 1;
    }

    if (!failed)
    {
        eARDISCOVERY_ERROR err = ARDISCOVERY_Connection_ControllerConnection(discoveryData, BD_DISCOVERY_PORT, deviceManager->addr);
        if (err != ARDISCOVERY_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while opening discovery connection: %s", ARDISCOVERY_Error_ToString(err));
            failed = 1;
        }
    }

    ARDISCOVERY_Connection_Delete(&discoveryData);

    return failed;
}


int startNetwork(BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARNETWORK_ERROR netError = ARNETWORK_OK;
    eARNETWORKAL_ERROR netAlError = ARNETWORKAL_OK;
    int pingDelay = 0; // 0 means default, -1 means no ping

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Start ARNetwork");

    // Create the ARNetworkALManager
    deviceManager->alManager = ARNETWORKAL_Manager_New(&netAlError);
    if (netAlError != ARNETWORKAL_OK)
    {
        failed = 1;
    }

    if (!failed)
    {
        // Initilize the ARNetworkALManager
        netAlError = ARNETWORKAL_Manager_InitWifiNetwork(deviceManager->alManager, deviceManager->addr, deviceManager->c2dPort, BD_D2C_PORT, 1);
        if (netAlError != ARNETWORKAL_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        if (deviceManager->qosMode == 1)
        {
            netAlError = ARNETWORKAL_Manager_SetSendClassSelector(deviceManager->alManager, ARSAL_SOCKET_CLASS_SELECTOR_CS6);
            if (netAlError != ARNETWORKAL_OK)
            {
                failed = 1;
            }
            netAlError = ARNETWORKAL_Manager_SetRecvClassSelector(deviceManager->alManager, ARSAL_SOCKET_CLASS_SELECTOR_CS6);
            if (netAlError != ARNETWORKAL_OK)
            {
                failed = 1;
            }
        }
    }

    if (!failed)
    {
        // Create the ARNetworkManager.
        deviceManager->netManager = ARNETWORK_Manager_New(deviceManager->alManager, numC2dParams, c2dParams, numD2cParams, d2cParams, pingDelay, onDisconnectNetwork, deviceManager, &netError);
        if (netError != ARNETWORK_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create and start Tx and Rx threads.
        if (ARSAL_Thread_Create(&(deviceManager->rxThread), ARNETWORK_Manager_ReceivingThreadRun, deviceManager->netManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of Rx thread failed");
            failed = 1;
        }

        if (ARSAL_Thread_Create(&(deviceManager->txThread), ARNETWORK_Manager_SendingThreadRun, deviceManager->netManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of Tx thread failed");
            failed = 1;
        }
    }

    // Print net error
    if (failed)
    {
        if (netAlError != ARNETWORKAL_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNetWorkAL Error: %s", ARNETWORKAL_Error_ToString(netAlError));
        }

        if (netError != ARNETWORK_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNetWork Error: %s", ARNETWORK_Error_ToString(netError));
        }
    }

    return failed;
}


void stopNetwork(BD_MANAGER_t *deviceManager)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Stop ARNetwork");

    // ARNetwork cleanup
    if (deviceManager->netManager != NULL)
    {
        ARNETWORK_Manager_Stop(deviceManager->netManager);
        if (deviceManager->rxThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->rxThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->rxThread));
            deviceManager->rxThread = NULL;
        }

        if (deviceManager->txThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->txThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->txThread));
            deviceManager->txThread = NULL;
        }
    }

    if (deviceManager->alManager != NULL)
    {
        ARNETWORKAL_Manager_Unlock(deviceManager->alManager);

        ARNETWORKAL_Manager_CloseWifiNetwork(deviceManager->alManager);
    }

    ARNETWORK_Manager_Delete(&(deviceManager->netManager));
    ARNETWORKAL_Manager_Delete(&(deviceManager->alManager));
}


void onDisconnectNetwork(ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "onDisconnectNetwork ...");
}


int sendDateAndTime(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send date and time");

    char strDate[30];
    char strTime[30];
    time_t rawDate;
    struct tm* tmDateTime;

    time(&rawDate);
    tmDateTime = localtime(&rawDate);
    strftime(strDate, 30, "%F", tmDateTime);
    strftime(strTime, 30, "T%H%M%S%z", tmDateTime);

    // Send date command
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentDate(cmdBuffer, sizeof(cmdBuffer), &cmdSize, strDate);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send date command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    // Send time command
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentTime(cmdBuffer, sizeof(cmdBuffer), &cmdSize, strTime);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send time command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}


int sendAllStates(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send All States");

    // Send all states command
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonAllStates(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send all states command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}


int sendBeginStream(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send Streaming Begin");
    
    // Send Streaming begin command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3MediaStreamingVideoEnable(cmdBuffer, sizeof(cmdBuffer), &cmdSize, 1);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}


int startVideo(BD_MANAGER_t *deviceManager)
{
    int ret = 0;
    eARSTREAM2_ERROR err;
    ARSTREAM2_StreamReceiver_Config_t streamReceiverConfig;
    ARSTREAM2_StreamReceiver_NetConfig_t streamReceiverNetConfig;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Start ARStream2");

    if (ret == 0)
    {
        memset(&streamReceiverConfig, 0, sizeof(streamReceiverConfig));
        memset(&streamReceiverNetConfig, 0, sizeof(streamReceiverNetConfig));

        streamReceiverNetConfig.serverAddr = deviceManager->addr;
        int addrFirst = atoi(deviceManager->addr);
        streamReceiverNetConfig.mcastAddr = ((addrFirst >= 224) && (addrFirst <= 239)) ? deviceManager->addr : NULL;
        streamReceiverNetConfig.mcastIfaceAddr = NULL;
        streamReceiverNetConfig.serverStreamPort = deviceManager->arstream2ServerStreamPort;
        streamReceiverNetConfig.serverControlPort = deviceManager->arstream2ServerControlPort;
        streamReceiverNetConfig.clientStreamPort = deviceManager->arstream2ClientStreamPort;
        streamReceiverNetConfig.clientControlPort = deviceManager->arstream2ClientControlPort;
        streamReceiverNetConfig.classSelector = (deviceManager->qosMode == 1) ? ARSAL_SOCKET_CLASS_SELECTOR_CS4 : ARSAL_SOCKET_CLASS_SELECTOR_UNSPECIFIED;
        streamReceiverConfig.canonicalName = "StreamReceiverTest";
        streamReceiverConfig.friendlyName = "StreamReceiverTest";
        streamReceiverConfig.applicationName = "StreamReceiverTest";
        streamReceiverConfig.maxPacketSize = deviceManager->arstream2MaxPacketSize;
        streamReceiverConfig.generateReceiverReports = 1;
        streamReceiverConfig.waitForSync = 1;
        streamReceiverConfig.outputIncompleteAu = 0;
        streamReceiverConfig.filterOutSpsPps = 0;
        streamReceiverConfig.filterOutSei = 1;
        streamReceiverConfig.replaceStartCodesWithNaluSize = 0;
        streamReceiverConfig.generateSkippedPSlices = 1;
        streamReceiverConfig.generateFirstGrayIFrame = 1;
        streamReceiverConfig.debugPath = "./streamdebug";

        err = ARSTREAM2_StreamReceiver_Init(&deviceManager->streamReceiver, &streamReceiverConfig, &streamReceiverNetConfig, NULL);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "ARSTREAM2_StreamReceiver_Init() failed: %s", ARSTREAM2_Error_ToString(err));
            ret = -1;
        }
    }

    if (ret == 0)
    {
        int thErr = ARSAL_Thread_Create(&deviceManager->streamNetworkThread, ARSTREAM2_StreamReceiver_RunNetworkThread, (void*)deviceManager->streamReceiver);
        if (thErr != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Stream network thread creation failed (%d)", thErr);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        int thErr = ARSAL_Thread_Create(&deviceManager->streamOutputThread, ARSTREAM2_StreamReceiver_RunAppOutputThread, (void*)deviceManager->streamReceiver);
        if (thErr != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Stream output thread creation failed (%d)", thErr);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        err = ARSTREAM2_StreamReceiver_StartAppOutput(deviceManager->streamReceiver, spsPpsCallback, deviceManager,
                                                      getAuBufferCallback, deviceManager,
                                                      auReadyCallback, deviceManager);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "ARSTREAM2_StreamReceiver_StartAppOutput() failed: %s", ARSTREAM2_Error_ToString(err));
            ret = -1;
        }
    }

    return ret;
}


void stopVideo(BD_MANAGER_t *deviceManager)
{
    int ret = 0;
    eARSTREAM2_ERROR err;
    int thErr;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Stop ARStream2");

    if (ret == 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARSTREAM2_StreamReceiver_StopAppOutput");
        err = ARSTREAM2_StreamReceiver_StopAppOutput(deviceManager->streamReceiver);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "ARSTREAM2_StreamReceiver_StopAppOutput() failed: %s", ARSTREAM2_Error_ToString(err));
            ret = -1;
        }
    }

    if (ret == 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARSTREAM2_StreamReceiver_Stop");
        err = ARSTREAM2_StreamReceiver_Stop(deviceManager->streamReceiver);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "ARSTREAM2_StreamReceiver_Stop() failed: %s", ARSTREAM2_Error_ToString(err));
            ret = -1;
        }
    }

    if (ret == 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Threads delete");
        thErr = ARSAL_Thread_Join(deviceManager->streamNetworkThread, NULL);
        if (thErr != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARSAL_Thread_Join() failed (%d)", thErr);
            ret = -1;
        }
        thErr = ARSAL_Thread_Destroy(&deviceManager->streamNetworkThread);
        if (thErr != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARSAL_Thread_Destroy() failed (%d)", thErr);
            ret = -1;
        }
        deviceManager->streamNetworkThread = NULL;
        thErr = ARSAL_Thread_Join(deviceManager->streamOutputThread, NULL);
        if (thErr != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARSAL_Thread_Join() failed (%d)", thErr);
            ret = -1;
        }
        thErr = ARSAL_Thread_Destroy(&deviceManager->streamOutputThread);
        if (thErr != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARSAL_Thread_Destroy() failed (%d)", thErr);
            ret = -1;
        }
        deviceManager->streamOutputThread = NULL;
    }

    if (ret == 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARSTREAM2_StreamReceiver_Free");
        err = ARSTREAM2_StreamReceiver_Free(&deviceManager->streamReceiver);
        if (err != ARSTREAM2_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "ARSTREAM2_StreamReceiver_Stop() failed: %s", ARSTREAM2_Error_ToString(err));
            ret = -1;
        }
    }
}


eARSTREAM2_ERROR spsPpsCallback(uint8_t *spsBuffer, int spsSize, uint8_t *ppsBuffer, int ppsSize, void *userPtr)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "SPS/PPS received: SPSSize=%d, PPSSize=%d", spsSize, ppsSize);

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR getAuBufferCallback(uint8_t **auBuffer, int *auBufferSize, void **auBufferUserPtr, void *userPtr)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)userPtr;

    *auBuffer = deviceManager->auBuffer;
    *auBufferSize = deviceManager->auBufferSize;
    *auBufferUserPtr = NULL;

    return ARSTREAM2_OK;
}


eARSTREAM2_ERROR auReadyCallback(uint8_t *auBuffer, int auSize,
                                 ARSTREAM2_StreamReceiver_AuReadyCallbackTimestamps_t *auTimestamps,
                                 eARSTREAM2_STREAM_RECEIVER_AU_SYNC_TYPE auSyncType,
                                 ARSTREAM2_StreamReceiver_AuReadyCallbackMetadata_t *auMetadata,
                                 void *auBufferUserPtr, void *userPtr)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Frame received: RawTS=%"PRIu64", NtpTS=%"PRIu64", complete=%d, errors=%d, ref=%d, size=%d",
                auTimestamps->auNtpTimestampRaw, auTimestamps->auNtpTimestamp, auMetadata->isComplete, auMetadata->hasErrors, auMetadata->isRef, auSize);

    return ARSTREAM2_OK;
}


eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause)
{
    eARNETWORK_MANAGER_CALLBACK_RETURN retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT;

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "    - arnetworkCmdCallback %d, cause:%d ", buffer_id, cause);

    if (cause == ARNETWORK_MANAGER_CALLBACK_STATUS_TIMEOUT)
    {
        retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DATA_POP;
    }

    return retval;
}


eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback(uint8_t *dataTx, uint32_t *dataTxSize, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataTx != NULL) && (dataTxSize != NULL) && (deviceManager != NULL))
    {
        *dataTxSize = sprintf((char*)dataTx, "{ \"%s\": %d, \"%s\": \"%s\", \"%s\": \"%s\", \"%s\": %d, \"%s\": %d, \"%s\": %d, \"%s\": %d }",
                              ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY, deviceManager->d2cPort,
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY, "ARStream2StreamReceiverTest",
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY, "Unix",
                              ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY, 1,
                              ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_STREAM_PORT_KEY, deviceManager->arstream2ClientStreamPort,
                              ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_CONTROL_PORT_KEY, deviceManager->arstream2ClientControlPort,
                              ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SUPPORTED_METADATA_VERSION_KEY, 1) + 1;
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}


eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback(uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataRx != NULL) && (dataRxSize != 0) && (deviceManager != NULL))
    {
        char *json = malloc(dataRxSize + 1);
        strncpy(json, (char*)dataRx, dataRxSize);
        json[dataRxSize] = '\0';

        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - ReceiveJson:%s ", json);

        free(json);


        int error = 0;
        json_object* jsonObj_All;
        json_object* jsonObj_Item;
        json_bool jsonRet;
        int value;

        /* Parse the whole Rx buffer */
        if (error == 0)
        {
            jsonObj_All = json_tokener_parse((const char*)dataRx);
            if (jsonObj_All == NULL)
            {
                error = -1;
            }
        }

        /* Find the port node */
        if (error == 0)
        {
            jsonRet = json_object_object_get_ex(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_C2DPORT_KEY, &jsonObj_Item);
            if ((jsonRet) && (jsonObj_Item != NULL))
            {
                value = json_object_get_int(jsonObj_Item);
                deviceManager->c2dPort = value;
                ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - c2dPort = %d", deviceManager->c2dPort);
            }
        }

        /* Find the QoS mode */
        if (error == 0)
        {
            jsonRet = json_object_object_get_ex(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY, &jsonObj_Item);
            if ((jsonRet) && (jsonObj_Item != NULL))
            {
                value = json_object_get_int(jsonObj_Item);
                deviceManager->qosMode = value;
                ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - qosMode = %d", deviceManager->qosMode);
            }
        }

        /* Find the arstream2ServerStreamPort node */
        if (error == 0)
        {
            jsonRet = json_object_object_get_ex(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_STREAM_PORT_KEY, &jsonObj_Item);
            if ((jsonRet) && (jsonObj_Item != NULL))
            {
                value = json_object_get_int(jsonObj_Item);
                deviceManager->arstream2ServerStreamPort = value;
                ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - arstream2ServerStreamPort = %d", deviceManager->arstream2ServerStreamPort);
            }
        }

        /* Find the arstream2ServerControlPort node */
        if (error == 0)
        {
            jsonRet = json_object_object_get_ex(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_CONTROL_PORT_KEY, &jsonObj_Item);
            if ((jsonRet) && (jsonObj_Item != NULL))
            {
                value = json_object_get_int(jsonObj_Item);
                deviceManager->arstream2ServerControlPort = value;
                ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - arstream2ServerControlPort = %d", deviceManager->arstream2ServerControlPort);
            }
        }

        /* Find the arstream2MaxPacketSize node */
        if (error == 0)
        {
            jsonRet = json_object_object_get_ex(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_MAX_PACKET_SIZE_KEY, &jsonObj_Item);
            if ((jsonRet) && (jsonObj_Item != NULL))
            {
                value = json_object_get_int(jsonObj_Item);
                deviceManager->arstream2MaxPacketSize = value;
                ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - arstream2MaxPacketSize = %d", deviceManager->arstream2MaxPacketSize);
            }
        }
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}
