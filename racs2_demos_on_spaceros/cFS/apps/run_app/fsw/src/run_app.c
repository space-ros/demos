/*
**   Include Files:
*/

#include "run_app.h"
#include "run_app_perfids.h"
#include "run_app_msgids.h"
#include "run_app_msg.h"
#include "racs2_user_msg.h"
#include "run_app_events.h"
#include "run_app_version.h"
#include "RACS2Bridge_geometry_msgs.pb-c.h"

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>

/*
** global data
*/

sample_hk_tlm_t    RUN_APP_HkTelemetryPkt;
racs2_user_msg_t   RACS2_UserMsgPkt;
CFE_SB_PipeId_t    RUN_APP_CommandPipe;
CFE_SB_MsgPtr_t    RUN_APP_MsgPtr;

typedef enum {
    FORWARD,
    BACKWARD,
    TURN_RIGHT,
    TURN_LEFT,
    STOP
} Command;

Command current_command = STOP;
struct termios original_termios;

double twist_linear_x = 0.0;
double twist_linear_y = 0.0;
double twist_linear_z = 0.0;
double twist_angular_x = 0.0;
double twist_angular_y = 0.0;
double twist_angular_z = 0.0;

static CFE_EVS_BinFilter_t  SAMPLE_EventFilters[] =
       {  /* Event ID    mask */
          {SAMPLE_STARTUP_INF_EID,       0x0000},
          {SAMPLE_COMMAND_ERR_EID,       0x0000},
          {SAMPLE_COMMANDNOP_INF_EID,    0x0000},
          {SAMPLE_COMMANDRST_INF_EID,    0x0000},
       };


void send_command(Command cmd) {
    switch (cmd) {
        case FORWARD:
            printf("Moving Forward\n");
            twist_linear_x = 2.0;
            break;
        case BACKWARD:
            printf("Moving Backward\n");
            twist_linear_x = -2.0;
            break;
        case TURN_RIGHT:
            printf("Turning Right\n");
            twist_linear_x = 1.0;
            twist_angular_z = -0.4;
            break;
        case TURN_LEFT:
            printf("Turning Left\n");
            twist_linear_x = 1.0;
            twist_angular_z = 0.4;
            break;
        case STOP:
            printf("Stopping\n");
            twist_linear_x = 0.0;
            twist_linear_y = 0.0;
            twist_linear_z = 0.0;
            twist_angular_x = 0.0;
            twist_angular_y = 0.0;
            twist_angular_z = 0.0;
            break;
    }
}

void process_input(char input) {
    switch (input) {
        case 'w':
            current_command = FORWARD;
            printf("###### Moving Forward ###### \n");
            twist_linear_x = 2.0;
            break;
        case 's':
            current_command = BACKWARD;
            printf("###### Moving Backward ###### \n");
            twist_linear_x = -2.0;
            break;
        case 'd':
            current_command = TURN_RIGHT;
            printf("###### Turning Right ###### \n");
            twist_linear_x = 1.0;
            twist_angular_z = -0.4;
            break;
        case 'a':
            current_command = TURN_LEFT;
            printf("###### Turning Left ###### \n");
            twist_linear_x = 1.0;
            twist_angular_z = 0.4;
            break;
        case 'x':
            current_command = STOP;
            printf("###### Stopping ###### \n");
            twist_linear_x = 0.0;
            twist_linear_y = 0.0;
            twist_linear_z = 0.0;
            twist_angular_x = 0.0;
            twist_angular_y = 0.0;
            twist_angular_z = 0.0;
            break;
        default:
            printf("!!!!! Invalid input !!!!! \n");
            return;
    }
}

void set_non_canonical_mode(int fd) {
    struct termios termios;

    // 現在の端末設定を取得
    tcgetattr(fd, &original_termios);

    // 設定を変更
    termios = original_termios;
    termios.c_lflag &= ~(ICANON | ECHO); // 非カノニカルモード、エコーを無効にする

    // 変更を適用
    tcsetattr(fd, TCSANOW, &termios);
}

void restore_canonical_mode(int fd) {
    tcsetattr(fd, TCSANOW, &original_termios);
}

void signal_handler(int signum) {
    // 端末設定を元に戻す
    restore_canonical_mode(fileno(stdin));
    printf("\nTerminating program.\n");
    exit(0);
}


/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* RUN_APP_Main() -- Application entry point and main process loop          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void RUN_APP_Main( void )
{
    int32  status;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    OS_printf("RUN_APP_Main starts.\n");

    CFE_ES_PerfLogEntry(SAMPLE_APP_PERF_ID);

    SAMPLE_TAKLKER_Init();

    struct termios original_termios;
    char input;
    signal(SIGINT, signal_handler);

    /*
    ** RUN_APP Runloop
    */
    
    int fd = fileno(stdin);
    set_non_canonical_mode(fd);

    int count = 0;
    while (CFE_ES_RunLoop(&RunStatus) == true)
    {
        if (read(fd, &input, 1) == 1) {
            process_input(input);
        }
        
        CFE_ES_PerfLogExit(SAMPLE_APP_PERF_ID);

        // /* Pend on receipt of command packet -- timeout set to 500 millisecs */
        // status = CFE_SB_RcvMsg(&RUN_APP_MsgPtr, RUN_APP_CommandPipe, 500);
        
        CFE_ES_PerfLogEntry(SAMPLE_APP_PERF_ID);

        // if (status == CFE_SUCCESS)
        // {
        //     RUN_APP_ProcessCommandPacket();
        // }
        sleep(2);

        // send message
        // set topic name
        strcpy(RACS2_UserMsgPkt.ros2_topic_name, "/cmd_vel");
        // define serialized body data 
        uint8_t *buffer;

        RACS2BridgeGeometryMsgsVector3 linear = RACS2_BRIDGE_GEOMETRY_MSGS__VECTOR3__INIT;
        RACS2BridgeGeometryMsgsVector3 angular = RACS2_BRIDGE_GEOMETRY_MSGS__VECTOR3__INIT;
        RACS2BridgeGeometryMsgsTwist message = RACS2_BRIDGE_GEOMETRY_MSGS__TWIST__INIT;

        message.linear = &linear;
        message.angular = &angular;
        message.linear->x = twist_linear_x;
        message.linear->y = twist_linear_y;
        message.linear->z = twist_linear_z;
        message.angular->x = twist_angular_x;
        message.angular->y = twist_angular_y;
        message.angular->z = twist_angular_z;
        // message.linear->x = 2.0;
        // message.linear->y = 0.0;
        // message.linear->z = 0.0;
        // message.angular->x = 0.0;
        // message.angular->y = 0.0;
        // message.angular->z = 0.0;

        OS_printf("RUN_APP: [Send][MsgID=0x%x][linear: x = %5.2f, y = %5.2f, z = %5.2f][angular: x = %5.2f, y = %5.2f, z = %5.2f]\n", 
            RACS2_BRIDGE_MID,
            message.linear->x,
            message.linear->y,
            message.linear->z,
            message.angular->x,
            message.angular->y,
            message.angular->z
            );
      
        size_t len = protobuf_c_message_get_packed_size((const ProtobufCMessage *) &message);
        buffer=malloc(len);
        size_t packed_len = protobuf_c_message_pack((const ProtobufCMessage *)&message, buffer);

        // set body data
        memcpy(RACS2_UserMsgPkt.body_data, buffer, len);
        // set body data length
        RACS2_UserMsgPkt.body_data_length = len;

        // send data
        CFE_SB_TimeStampMsg((CFE_SB_Msg_t *) &RACS2_UserMsgPkt);
        status = CFE_SB_SendMsg((CFE_SB_Msg_t *) &RACS2_UserMsgPkt);
        // OS_printf("RUN_APP: Sent message, MID = [0x%x], sample_command_count = %d\n", 
        //     CFE_SB_GetMsgId((CFE_SB_MsgPtr_t) &RACS2_UserMsgPkt),
        //     RACS2_UserMsgPkt.sample_command_count
        //     );
        if (status != CFE_SUCCESS) {
            OS_printf("RUN_APP: Error: sending is failed. status = 0x%x\n", status);
        }
        
        free(buffer);

        count++;
    }

    CFE_ES_ExitApp(RunStatus);

} /* End of RUN_APP_Main() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* SAMPLE_TAKLKER_Init() --  initialization                                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void SAMPLE_TAKLKER_Init(void)
{
    /*
    ** Register the app with Executive services
    */
    CFE_ES_RegisterApp() ;

    /*
    ** Register the events
    */ 
    CFE_EVS_Register(SAMPLE_EventFilters,
                     sizeof(SAMPLE_EventFilters)/sizeof(CFE_EVS_BinFilter_t),
                     CFE_EVS_EventFilter_BINARY);

    RUN_APP_ResetCounters();

    CFE_SB_InitMsg(&RUN_APP_HkTelemetryPkt,
                   RUN_APP_HK_TLM_MID,
                   SAMPLE_APP_HK_TLM_LNGTH, true);

    CFE_SB_InitMsg(&RACS2_UserMsgPkt, RACS2_BRIDGE_MID, RACS2_USER_MSG_LNGTH, false);

    CFE_EVS_SendEvent (SAMPLE_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION,
               "RUN_APP App Initialized. Version %d.%d.%d.%d",
                SAMPLE_APP_MAJOR_VERSION,
                SAMPLE_APP_MINOR_VERSION, 
                SAMPLE_APP_REVISION, 
                SAMPLE_APP_MISSION_REV);
				
} /* End of SAMPLE_TAKLKER_Init() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  RUN_APP_ProcessCommandPacket                                 */
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the RUN_APP */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void RUN_APP_ProcessCommandPacket(void)
{
    CFE_SB_MsgId_t  MsgId;

    MsgId = CFE_SB_GetMsgId(RUN_APP_MsgPtr);
    
    switch (MsgId)
    {
        case RUN_APP_CMD_MID:
            RUN_APP_ProcessGroundCommand();
            break;

        case RUN_APP_SEND_HK_MID:
            RUN_APP_ReportHousekeeping();
            break;

        default:
            RUN_APP_HkTelemetryPkt.sample_command_error_count++;
            CFE_EVS_SendEvent(SAMPLE_COMMAND_ERR_EID,CFE_EVS_EventType_ERROR,
			"RUN_APP: invalid command packet,MID = 0x%x", MsgId);
            break;
    }

    return;

} /* End RUN_APP_ProcessCommandPacket */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* RUN_APP_ProcessGroundCommand() -- RUN_APP ground commands      */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/

void RUN_APP_ProcessGroundCommand(void)
{
    uint16 CommandCode;

    CommandCode = CFE_SB_GetCmdCode(RUN_APP_MsgPtr);

    /* Process "known" RUN_APP app ground commands */
    switch (CommandCode)
    {
        case SAMPLE_APP_NOOP_CC:
            RUN_APP_HkTelemetryPkt.sample_command_count++;
            CFE_EVS_SendEvent(SAMPLE_COMMANDNOP_INF_EID,
                        CFE_EVS_EventType_INFORMATION,
			"RUN_APP: NOOP command");
            break;

        case SAMPLE_APP_RESET_COUNTERS_CC:
            RUN_APP_ResetCounters();
            break;

        /* default case already found during FC vs length test */
        default:
            break;
    }
    return;

} /* End of RUN_APP_ProcessGroundCommand() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  RUN_APP_ReportHousekeeping                                   */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void RUN_APP_ReportHousekeeping(void)
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t *) &RUN_APP_HkTelemetryPkt);
    CFE_SB_SendMsg((CFE_SB_Msg_t *) &RUN_APP_HkTelemetryPkt);
    return;

} /* End of RUN_APP_ReportHousekeeping() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  RUN_APP_ResetCounters                                        */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void RUN_APP_ResetCounters(void)
{
    /* Status of commands processed by the RUN_APP App */
    RUN_APP_HkTelemetryPkt.sample_command_count       = 0;
    RUN_APP_HkTelemetryPkt.sample_command_error_count = 0;

    CFE_EVS_SendEvent(SAMPLE_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION,
		"RUN_APP: RESET command");
    return;

} /* End of RUN_APP_ResetCounters() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* RUN_APP_VerifyCmdLength() -- Verify command packet length            */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool RUN_APP_VerifyCmdLength(CFE_SB_MsgPtr_t msg, uint16 ExpectedLength)
{     
    bool result = true;

    uint16 ActualLength = CFE_SB_GetTotalMsgLength(msg);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        CFE_SB_MsgId_t MessageID   = CFE_SB_GetMsgId(msg);
        uint16         CommandCode = CFE_SB_GetCmdCode(msg);

        CFE_EVS_SendEvent(SAMPLE_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
           "Invalid msg length: ID = 0x%X,  CC = %d, Len = %d, Expected = %d",
              MessageID, CommandCode, ActualLength, ExpectedLength);
        result = false;
        RUN_APP_HkTelemetryPkt.sample_command_error_count++;
    }

    return(result);

} /* End of RUN_APP_VerifyCmdLength() */

