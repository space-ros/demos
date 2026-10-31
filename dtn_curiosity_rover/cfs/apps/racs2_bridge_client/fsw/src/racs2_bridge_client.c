/*
 * racs2_bridge_client: forwards RACS2 user messages between the cFS Software
 * Bus and the RACS2 ROS 2 bridge (bridge_py_s) over a websocket.
 *
 * cFS v7 port of racs2_bridge_client from jaxa/racs2_bridge v1.1.1
 * (Apache-2.0), keeping its websocket frame format:
 *
 *   cFS -> ROS 2: 32-byte ROS 2 topic name, then the serialized body
 *   ROS 2 -> cFS: 2-byte cFS message ID (big endian) in a 32-byte header,
 *                 then the serialized body
 *
 * In this demo it runs on the rover cFS node, where BPNode delivers the
 * messages that arrived from the ground as DTN bundles.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <libwebsockets.h>

#include "cfe.h"
#include "racs2_user_msg.h"

#define RACS2_BRIDGE_CONFIG_FILE   "cf/racs2_bridge_config.txt"
#define RACS2_BRIDGE_HEADER_LNGTH  32
#define RACS2_BRIDGE_PIPE_DEPTH    32
#define RACS2_BRIDGE_RCV_TIMEOUT   50 /* msec */
#define RACS2_BRIDGE_RETRY_SEC     5  /* seconds between connect attempts */
#define RACS2_BRIDGE_MAX_PENDING   16

typedef struct
{
    uint8  Data[LWS_PRE + RACS2_BRIDGE_HEADER_LNGTH + BODY_DATA_MAX_LNGTH];
    size_t Len;
} RACS2_BRIDGE_Frame_t;

static char                 WssUri[64];
static uint16               WssPort;
static CFE_SB_PipeId_t      CmdPipe;
static racs2_user_msg_t     InboundMsg;
static struct lws_context  *LwsContext;
static struct lws          *WebSocket;
static bool                 Connected;
static RACS2_BRIDGE_Frame_t Pending[RACS2_BRIDGE_MAX_PENDING];
static uint32               PendingHead;
static uint32               PendingCount;

static void RACS2_BRIDGE_Timestamp(char *Buf, size_t Size)
{
    struct timespec Now;
    struct tm       Tm;

    clock_gettime(CLOCK_REALTIME, &Now);
    gmtime_r(&Now.tv_sec, &Tm);
    snprintf(Buf, Size, "%02d:%02d:%02d.%03ldZ", Tm.tm_hour, Tm.tm_min, Tm.tm_sec, Now.tv_nsec / 1000000L);
}

/* ROS 2 -> cFS: publish the frame body on the Software Bus under the message ID in its header */
static void RACS2_BRIDGE_PublishInbound(const uint8 *In, size_t Len)
{
    uint16 MsgIdValue;
    size_t BodyLen;

    if (Len < RACS2_BRIDGE_HEADER_LNGTH)
    {
        return; /* e.g. the server's greeting */
    }

    MsgIdValue = (uint16)((In[0] << 8) | In[1]);
    BodyLen    = Len - RACS2_BRIDGE_HEADER_LNGTH;
    if (BodyLen > BODY_DATA_MAX_LNGTH)
    {
        OS_printf("RACS2_BRIDGE_CLIENT: dropping %lu-byte body for MsgID 0x%04X\n", (unsigned long)BodyLen,
                  MsgIdValue);
        return;
    }

    memset(&InboundMsg, 0, sizeof(InboundMsg));
    CFE_MSG_Init(CFE_MSG_PTR(InboundMsg.TelemetryHeader), CFE_SB_ValueToMsgId(MsgIdValue), sizeof(InboundMsg));
    InboundMsg.body_data_length = (uint8)BodyLen;
    memcpy(InboundMsg.body_data, In + RACS2_BRIDGE_HEADER_LNGTH, BodyLen);
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(InboundMsg.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(InboundMsg.TelemetryHeader), true);
}

static int RACS2_BRIDGE_Callback(struct lws *Wsi, enum lws_callback_reasons Reason, void *User, void *In,
                                 size_t Len)
{
    RACS2_BRIDGE_Frame_t *Frame;

    (void)User;

    switch (Reason)
    {
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            Connected = true;
            OS_printf("RACS2_BRIDGE_CLIENT: connected to ROS 2 bridge at %s:%u\n", WssUri, WssPort);
            lws_callback_on_writable(Wsi);
            break;

        case LWS_CALLBACK_CLIENT_RECEIVE:
            RACS2_BRIDGE_PublishInbound((const uint8 *)In, Len);
            break;

        case LWS_CALLBACK_CLIENT_WRITEABLE:
            if (PendingCount > 0)
            {
                Frame = &Pending[PendingHead];
                lws_write(Wsi, &Frame->Data[LWS_PRE], Frame->Len, LWS_WRITE_BINARY);
                PendingHead = (PendingHead + 1) % RACS2_BRIDGE_MAX_PENDING;
                PendingCount--;
                if (PendingCount > 0)
                {
                    lws_callback_on_writable(Wsi);
                }
            }
            break;

        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
        case LWS_CALLBACK_CLIENT_CLOSED:
            if (Connected)
            {
                OS_printf("RACS2_BRIDGE_CLIENT: connection to ROS 2 bridge closed, retrying\n");
            }
            Connected = false;
            WebSocket = NULL;
            break;

        default:
            break;
    }

    return 0;
}

static const struct lws_protocols RACS2_BRIDGE_Protocols[] = {
    {"example-protocol", RACS2_BRIDGE_Callback, 0, 256, 0, NULL, 0},
    {NULL, NULL, 0, 0, 0, NULL, 0} /* terminator */
};

/* cFS -> ROS 2: queue a websocket frame for a RACS2 user message from the Software Bus */
static void RACS2_BRIDGE_QueueOutbound(const racs2_user_msg_t *Msg)
{
    RACS2_BRIDGE_Frame_t *Frame;
    uint8                *Payload;
    size_t                BodyLen = Msg->body_data_length;
    char                  Stamp[32];
    char                  Topic[ROS2_TOPIC_NAME_LNGTH + 1];

    if (BodyLen > BODY_DATA_MAX_LNGTH)
    {
        BodyLen = BODY_DATA_MAX_LNGTH;
    }

    memcpy(Topic, Msg->ros2_topic_name, ROS2_TOPIC_NAME_LNGTH);
    Topic[ROS2_TOPIC_NAME_LNGTH] = '\0';
    RACS2_BRIDGE_Timestamp(Stamp, sizeof(Stamp));

    if (PendingCount == RACS2_BRIDGE_MAX_PENDING)
    {
        OS_printf("[rover %s] bridge queue full, dropping message for %s\n", Stamp, Topic);
        return;
    }

    Frame   = &Pending[(PendingHead + PendingCount) % RACS2_BRIDGE_MAX_PENDING];
    Payload = &Frame->Data[LWS_PRE];
    memset(Payload, 0, RACS2_BRIDGE_HEADER_LNGTH);
    memcpy(Payload, Msg->ros2_topic_name, ROS2_TOPIC_NAME_LNGTH);
    memcpy(Payload + RACS2_BRIDGE_HEADER_LNGTH, Msg->body_data, BodyLen);
    Frame->Len = RACS2_BRIDGE_HEADER_LNGTH + BodyLen;
    PendingCount++;

    OS_printf("[rover %s] received %s command from the ground, body #%08x, forwarding to ROS 2%s\n", Stamp, Topic,
              (unsigned int)racs2_body_hash(Msg->body_data, BodyLen), Connected ? "" : " (bridge not connected yet, queued)");

    if (WebSocket != NULL)
    {
        lws_callback_on_writable(WebSocket);
    }
}

static int32 RACS2_BRIDGE_ReadConfig(void)
{
    FILE *Fp;
    char  Line[128];
    char  Key[32];
    char  Value[64];
    bool  HaveUri  = false;
    bool  HavePort = false;

    Fp = fopen(RACS2_BRIDGE_CONFIG_FILE, "r");
    if (Fp == NULL)
    {
        OS_printf("RACS2_BRIDGE_CLIENT: cannot open %s\n", RACS2_BRIDGE_CONFIG_FILE);
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    while (fgets(Line, sizeof(Line), Fp) != NULL)
    {
        if (sscanf(Line, "%31[^=]=%63s", Key, Value) != 2)
        {
            continue;
        }
        if (strcmp(Key, "wss_uri") == 0)
        {
            snprintf(WssUri, sizeof(WssUri), "%s", Value);
            HaveUri = true;
        }
        else if (strcmp(Key, "wss_port") == 0)
        {
            unsigned long Port = strtoul(Value, NULL, 10);
            if (Port > 0 && Port <= 65535UL)
            {
                WssPort  = (uint16)Port;
                HavePort = true;
            }
        }
    }
    fclose(Fp);

    if (!HaveUri || !HavePort)
    {
        OS_printf("RACS2_BRIDGE_CLIENT: %s needs wss_uri and wss_port\n", RACS2_BRIDGE_CONFIG_FILE);
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    return CFE_SUCCESS;
}

static int32 RACS2_BRIDGE_Init(void)
{
    struct lws_context_creation_info Info;
    int32                            Status;

    CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);

    Status = RACS2_BRIDGE_ReadConfig();
    if (Status != CFE_SUCCESS)
    {
        return Status;
    }

    Status = CFE_SB_CreatePipe(&CmdPipe, RACS2_BRIDGE_PIPE_DEPTH, "RACS2_TO_ROS");
    if (Status == CFE_SUCCESS)
    {
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(RACS2_BRIDGE_MID), CmdPipe);
    }
    if (Status != CFE_SUCCESS)
    {
        OS_printf("RACS2_BRIDGE_CLIENT: Software Bus setup failed, status 0x%08X\n", (unsigned int)Status);
        return Status;
    }

    lws_set_log_level(LLL_ERR, NULL);
    memset(&Info, 0, sizeof(Info));
    Info.port      = CONTEXT_PORT_NO_LISTEN;
    Info.protocols = RACS2_BRIDGE_Protocols;
    Info.gid       = -1;
    Info.uid       = -1;
    LwsContext     = lws_create_context(&Info);
    if (LwsContext == NULL)
    {
        OS_printf("RACS2_BRIDGE_CLIENT: lws_create_context failed\n");
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION, "RACS2_BRIDGE_CLIENT ready, ROS 2 bridge at %s:%u",
                      WssUri, WssPort);
    return CFE_SUCCESS;
}

static void RACS2_BRIDGE_Connect(void)
{
    struct lws_client_connect_info Info;

    memset(&Info, 0, sizeof(Info));
    Info.context  = LwsContext;
    Info.address  = WssUri;
    Info.port     = WssPort;
    Info.path     = "/";
    Info.host     = WssUri;
    Info.origin   = "origin";
    Info.protocol = RACS2_BRIDGE_Protocols[0].name;
    WebSocket     = lws_client_connect_via_info(&Info);
}

void RACS2_BRIDGE_Main(void)
{
    uint32           RunStatus = CFE_ES_RunStatus_APP_RUN;
    CFE_SB_Buffer_t *BufPtr;
    CFE_SB_MsgId_t   MsgId;
    CFE_Status_t     Status;
    time_t           LastAttempt = 0;

    if (RACS2_BRIDGE_Init() != CFE_SUCCESS)
    {
        RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&RunStatus) == true)
    {
        /* A connect attempt can block while the bridge's host name does not resolve, so
        ** retry only every few seconds */
        if (WebSocket == NULL && time(NULL) - LastAttempt >= RACS2_BRIDGE_RETRY_SEC)
        {
            LastAttempt = time(NULL);
            RACS2_BRIDGE_Connect();
        }

        /* Take every message waiting on the pipe, so a burst (such as the commands an
        ** orbiter relay delivers in one pass) is not spread over several loops */
        Status = CFE_SB_ReceiveBuffer(&BufPtr, CmdPipe, RACS2_BRIDGE_RCV_TIMEOUT);
        while (Status == CFE_SUCCESS)
        {
            CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
            if (CFE_SB_MsgIdToValue(MsgId) == RACS2_BRIDGE_MID)
            {
                RACS2_BRIDGE_QueueOutbound((const racs2_user_msg_t *)BufPtr);
            }
            Status = CFE_SB_ReceiveBuffer(&BufPtr, CmdPipe, CFE_SB_POLL);
        }

        lws_service(LwsContext, 0);
    }

    if (LwsContext != NULL)
    {
        lws_context_destroy(LwsContext);
    }
    CFE_ES_ExitApp(RunStatus);
}
