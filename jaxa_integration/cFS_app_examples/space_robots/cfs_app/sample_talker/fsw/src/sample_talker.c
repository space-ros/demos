/*
**   Include Files:
*/
#include "sample_talker.h"
#include "sample_talker_perfids.h"
#include "sample_talker_msgids.h"
#include "sample_talker_msg.h"
#include "racs2_user_msg.h"
#include "sample_talker_events.h"
#include "sample_talker_version.h"
#include "RACS2Brdige_std_msgs.pb-c.h"

/*
** Global Data
*/
sample_hk_tlm_t    SAMPLE_TALKER_HkTelemetryPkt;
racs2_user_msg_t   RACS2_UserMsgPkt;
CFE_SB_PipeId_t    SAMPLE_TALKER_CommandPipe;
CFE_SB_MsgPtr_t    SAMPLE_TALKER_MsgPtr;

static CFE_EVS_BinFilter_t SAMPLE_EventFilters[] = {
    {SAMPLE_STARTUP_INF_EID, 0x0000},
    {SAMPLE_COMMAND_ERR_EID, 0x0000},
    {SAMPLE_COMMANDNOP_INF_EID, 0x0000},
    {SAMPLE_COMMANDRST_INF_EID, 0x0000},
};

/*
** Name: SAMPLE_TALKER_Main
**
** Purpose: Entry point and main loop of the application. It generates a random
**          integer between 0 and 4 and sends it as a message to the ROS2 bridge.
**          The process repeats every 5 seconds.
**
*/
void SAMPLE_TALKER_Main(void)
{
    int32 status;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;
    int num;
    int count = 0;

    OS_printf("SAMPLE_TALKER_Main starts.\n");

    // Log the performance entry
    CFE_ES_PerfLogEntry(SAMPLE_APP_PERF_ID);

    // Initialize the application
    SAMPLE_TAKLKER_Init();

    // Seed the random number generator
    srand(time(NULL));

    /*
    ** SAMPLE_TALKER Runloop: Generates a random number and sends messages.
    */
    while (CFE_ES_RunLoop(&RunStatus) == true)
    {
        CFE_ES_PerfLogExit(SAMPLE_APP_PERF_ID);
        CFE_ES_PerfLogEntry(SAMPLE_APP_PERF_ID);

        // Generate a random number between 0 and 7
        num = rand() % 8;

        // Set the ROS2 topic name
        strcpy(RACS2_UserMsgPkt.ros2_topic_name, "/Recv/RACS2Bridge");

        // Create the message content
        int string_length = 22;
        char buf[32];
        sprintf(buf, "Message To ROS2 :%5d", num);

        RACS2BridgeStdMsgs *message = (RACS2BridgeStdMsgs *)malloc(sizeof(RACS2BridgeStdMsgs));
        racs2_bridge_std_msgs__init(message);

        message->string_data = (char *)malloc(string_length);
        strncpy(message->string_data, buf, string_length);

        // Serialize the message
        int len = racs2_bridge_std_msgs__get_packed_size(message);
        void *buffer = malloc(len);
        racs2_bridge_std_msgs__pack(message, buffer);

        // Set the body data and length
        strncpy(RACS2_UserMsgPkt.body_data, buffer, len);
        RACS2_UserMsgPkt.body_data_length = len;

        // Timestamp the message and send it
        CFE_SB_TimeStampMsg((CFE_SB_Msg_t *)&RACS2_UserMsgPkt);
        status = CFE_SB_SendMsg((CFE_SB_Msg_t *)&RACS2_UserMsgPkt);

        // Check for errors in sending
        if (status != CFE_SUCCESS)
        {
            OS_printf("SAMPLE_TALKER: Error: sending failed. status = 0x%x\n", status);
        }

        // Clean up memory
        free(buffer);
        free(message->string_data);
        free(message);
        memset(buf, '\0', sizeof(buf));

        // Wait for 5 seconds before sending the next message
        sleep(5);
    }

    CFE_ES_ExitApp(RunStatus);
}

/*
** Name: SAMPLE_TAKLKER_Init
**
** Purpose: Initializes the application by registering it with executive services,
**          registering events, resetting counters, and initializing message packets.
**
*/
void SAMPLE_TAKLKER_Init(void)
{
    // Register the application with Executive services
    CFE_ES_RegisterApp();

    // Register events with event services
    CFE_EVS_Register(SAMPLE_EventFilters,
                     sizeof(SAMPLE_EventFilters) / sizeof(CFE_EVS_BinFilter_t),
                     CFE_EVS_EventFilter_BINARY);

    // Reset counters
    SAMPLE_TALKER_ResetCounters();

    // Initialize housekeeping and user message packets
    CFE_SB_InitMsg(&SAMPLE_TALKER_HkTelemetryPkt, SAMPLE_TALKER_HK_TLM_MID, SAMPLE_APP_HK_TLM_LNGTH, true);
    CFE_SB_InitMsg(&RACS2_UserMsgPkt, RACS2_BRIDGE_MID, RACS2_USER_MSG_LNGTH, false);

    // Send startup event
    CFE_EVS_SendEvent(SAMPLE_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "SAMPLE_TALKER App Initialized. Version %d.%d.%d.%d",
                      SAMPLE_APP_MAJOR_VERSION,
                      SAMPLE_APP_MINOR_VERSION,
                      SAMPLE_APP_REVISION,
                      SAMPLE_APP_MISSION_REV);
}

/*
** Name: SAMPLE_TALKER_ProcessCommandPacket
**
** Purpose: Processes packets received on the SAMPLE_TALKER command pipe by
**          identifying the message ID and directing it to the appropriate handler.
**
*/
void SAMPLE_TALKER_ProcessCommandPacket(void)
{
    CFE_SB_MsgId_t MsgId;

    MsgId = CFE_SB_GetMsgId(SAMPLE_TALKER_MsgPtr);

    switch (MsgId)
    {
        case SAMPLE_TALKER_CMD_MID:
            SAMPLE_TALKER_ProcessGroundCommand();
            break;

        case SAMPLE_TALKER_SEND_HK_MID:
            SAMPLE_TALKER_ReportHousekeeping();
            break;

        default:
            SAMPLE_TALKER_HkTelemetryPkt.sample_command_error_count++;
            CFE_EVS_SendEvent(SAMPLE_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "SAMPLE_TALKER: invalid command packet,MID = 0x%x", MsgId);
            break;
    }
}

/*
** Name: SAMPLE_TALKER_ProcessGroundCommand
**
** Purpose: Processes ground commands such as NOOP and Reset Counters, updating telemetry
**          or taking the appropriate action based on the command code.
**
*/
void SAMPLE_TALKER_ProcessGroundCommand(void)
{
    uint16 CommandCode;

    CommandCode = CFE_SB_GetCmdCode(SAMPLE_TALKER_MsgPtr);

    switch (CommandCode)
    {
        case SAMPLE_APP_NOOP_CC:
            SAMPLE_TALKER_HkTelemetryPkt.sample_command_count++;
            CFE_EVS_SendEvent(SAMPLE_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "SAMPLE_TALKER: NOOP command");
            break;

        case SAMPLE_APP_RESET_COUNTERS_CC:
            SAMPLE_TALKER_ResetCounters();
            break;

        default:
            break;
    }
}

/*
** Name: SAMPLE_TALKER_ReportHousekeeping
**
** Purpose: Gathers the application's telemetry, packetizes it, and sends it to
**          the housekeeping task via the software bus.
**
*/
void SAMPLE_TALKER_ReportHousekeeping(void)
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t *)&SAMPLE_TALKER_HkTelemetryPkt);
    CFE_SB_SendMsg((CFE_SB_Msg_t *)&SAMPLE_TALKER_HkTelemetryPkt);
}

/*
** Name: SAMPLE_TALKER_ResetCounters
**
** Purpose: Resets all global counter variables that are part of the task telemetry.
**
*/
void SAMPLE_TALKER_ResetCounters(void)
{
    SAMPLE_TALKER_HkTelemetryPkt.sample_command_count = 0;
    SAMPLE_TALKER_HkTelemetryPkt.sample_command_error_count = 0;

    CFE_EVS_SendEvent(SAMPLE_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "SAMPLE_TALKER: RESET command");
}

/*
** Name: SAMPLE_TALKER_VerifyCmdLength
**
** Purpose: Verifies that the length of a command packet matches the expected length.
**          If the lengths do not match, an error event is sent.
**
** Returns: true if lengths match, false otherwise.
**
*/
bool SAMPLE_TALKER_VerifyCmdLength(CFE_SB_MsgPtr_t msg, uint16 ExpectedLength)
{
    bool result = true;
    uint16 ActualLength = CFE_SB_GetTotalMsgLength(msg);

    if (ExpectedLength != ActualLength)
    {
        CFE_SB_MsgId_t MessageID = CFE_SB_GetMsgId(msg);
        uint16 CommandCode = CFE_SB_GetCmdCode(msg);

        CFE_EVS_SendEvent(SAMPLE_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid msg length: ID = 0x%X, CC = %d, Len = %d, Expected = %d",
                          MessageID, CommandCode, ActualLength, ExpectedLength);
        result = false;
        SAMPLE_TALKER_HkTelemetryPkt.sample_command_error_count++;
    }

    return result;
}

