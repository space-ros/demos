/*
 * run_app: keyboard teleoperation for the Curiosity rover, running on the
 * ground cFS node.
 *
 * Ported to cFS v7 from run_app in space-ros/demos racs2_demos_on_spaceros.
 * Each key press sends one RACS2 user message (topic /cmd_vel, protobuf
 * geometry_msgs/Twist) on the Software Bus. In this demo BPNode picks that
 * message up and carries it to the rover as a DTN bundle.
 *
 * Keys are read from the terminal running core-cpu1 and from the named pipe
 * /tmp/ground_keys, so they can also be sent with: echo w > /tmp/ground_keys
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "cfe.h"
#include "racs2_user_msg.h"
#include "RACS2Bridge_geometry_msgs.pb-c.h"

#define RUN_APP_KEY_FIFO      "/tmp/ground_keys"
#define RUN_APP_POLL_MSEC     100

static racs2_user_msg_t RUN_APP_Msg;
static struct termios   RUN_APP_OrigTermios;
static bool             RUN_APP_TermiosSaved = false;
static int              RUN_APP_FifoFd       = -1;

static double LinearX  = 0.0;
static double AngularZ = 0.0;

static void RUN_APP_Timestamp(char *Buf, size_t Size)
{
    struct timespec Now;
    struct tm       Tm;

    clock_gettime(CLOCK_REALTIME, &Now);
    gmtime_r(&Now.tv_sec, &Tm);
    snprintf(Buf, Size, "%02d:%02d:%02d.%03ldZ", Tm.tm_hour, Tm.tm_min, Tm.tm_sec, Now.tv_nsec / 1000000L);
}

/* Returns true if the key changed the command */
static bool RUN_APP_ProcessKey(char Key, const char **Name)
{
    switch (Key)
    {
        case 'w':
            LinearX  = 2.0;
            AngularZ = 0.0;
            *Name    = "forward";
            break;
        case 's':
            LinearX  = -2.0;
            AngularZ = 0.0;
            *Name    = "backward";
            break;
        case 'd':
            LinearX  = 1.0;
            AngularZ = -0.4;
            *Name    = "turn right";
            break;
        case 'a':
            LinearX  = 1.0;
            AngularZ = 0.4;
            *Name    = "turn left";
            break;
        case 'x':
            LinearX  = 0.0;
            AngularZ = 0.0;
            *Name    = "stop";
            break;
        default:
            return false;
    }
    return true;
}

static void RUN_APP_SendTwist(const char *Name)
{
    RACS2BridgeGeometryMsgsVector3 Linear  = RACS2_BRIDGE_GEOMETRY_MSGS__VECTOR3__INIT;
    RACS2BridgeGeometryMsgsVector3 Angular = RACS2_BRIDGE_GEOMETRY_MSGS__VECTOR3__INIT;
    RACS2BridgeGeometryMsgsTwist   Twist   = RACS2_BRIDGE_GEOMETRY_MSGS__TWIST__INIT;
    size_t                         Len;
    CFE_Status_t                   Status;
    char                           Stamp[32];

    Linear.x      = LinearX;
    Angular.z     = AngularZ;
    Twist.linear  = &Linear;
    Twist.angular = &Angular;

    Len = protobuf_c_message_get_packed_size((const ProtobufCMessage *)&Twist);
    if (Len > BODY_DATA_MAX_LNGTH)
    {
        OS_printf("RUN_APP: encoded Twist too large (%lu bytes)\n", (unsigned long)Len);
        return;
    }

    memset(RUN_APP_Msg.ros2_topic_name, 0, sizeof(RUN_APP_Msg.ros2_topic_name));
    strncpy(RUN_APP_Msg.ros2_topic_name, "/cmd_vel", sizeof(RUN_APP_Msg.ros2_topic_name) - 1);
    protobuf_c_message_pack((const ProtobufCMessage *)&Twist, RUN_APP_Msg.body_data);
    RUN_APP_Msg.body_data_length = (uint8)Len;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(RUN_APP_Msg.TelemetryHeader));
    Status = CFE_SB_TransmitMsg(CFE_MSG_PTR(RUN_APP_Msg.TelemetryHeader), true);

    RUN_APP_Timestamp(Stamp, sizeof(Stamp));
    if (Status == CFE_SUCCESS)
    {
        OS_printf("[ground %s] sent '%s' (linear.x=%.1f angular.z=%.1f) as MsgID 0x%04X\n", Stamp, Name, LinearX,
                  AngularZ, RACS2_BRIDGE_MID);
    }
    else
    {
        OS_printf("[ground %s] failed to send '%s', status 0x%08X\n", Stamp, Name, (unsigned int)Status);
    }
}

static void RUN_APP_Init(void)
{
    struct termios Raw;

    CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);

    CFE_MSG_Init(CFE_MSG_PTR(RUN_APP_Msg.TelemetryHeader), CFE_SB_ValueToMsgId(RACS2_BRIDGE_MID),
                 sizeof(RUN_APP_Msg));

    /* Terminal input, if core-cpu1 runs in a terminal: no line buffering, no echo, non-blocking */
    if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &RUN_APP_OrigTermios) == 0)
    {
        Raw = RUN_APP_OrigTermios;
        Raw.c_lflag &= ~(ICANON | ECHO);
        Raw.c_cc[VMIN]  = 0;
        Raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &Raw);
        RUN_APP_TermiosSaved = true;
    }

    /* Named pipe input */
    if (mkfifo(RUN_APP_KEY_FIFO, 0666) != 0 && errno != EEXIST)
    {
        OS_printf("RUN_APP: cannot create %s: %s\n", RUN_APP_KEY_FIFO, strerror(errno));
    }
    else
    {
        /* O_RDWR keeps the pipe open when no writer is attached */
        RUN_APP_FifoFd = open(RUN_APP_KEY_FIFO, O_RDWR | O_NONBLOCK);
    }

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
                      "RUN_APP ready: keys w/s/a/d/x on the console or via %s", RUN_APP_KEY_FIFO);
    OS_printf("RUN_APP: drive the rover with w (forward), s (backward), a (left), d (right), x (stop)\n");
}

static void RUN_APP_PollFd(int Fd)
{
    char        Key;
    const char *Name;

    if (Fd < 0)
    {
        return;
    }
    while (read(Fd, &Key, 1) == 1)
    {
        if (RUN_APP_ProcessKey(Key, &Name))
        {
            RUN_APP_SendTwist(Name);
        }
    }
}

void RUN_APP_Main(void)
{
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    RUN_APP_Init();

    while (CFE_ES_RunLoop(&RunStatus) == true)
    {
        if (RUN_APP_TermiosSaved)
        {
            RUN_APP_PollFd(STDIN_FILENO);
        }
        RUN_APP_PollFd(RUN_APP_FifoFd);
        OS_TaskDelay(RUN_APP_POLL_MSEC);
    }

    if (RUN_APP_TermiosSaved)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &RUN_APP_OrigTermios);
    }
    if (RUN_APP_FifoFd >= 0)
    {
        close(RUN_APP_FifoFd);
    }
    CFE_ES_ExitApp(RunStatus);
}
