/*
 * RACS2 user message, as carried on the cFS Software Bus between run_app,
 * BPNode and racs2_bridge_client.
 *
 * Same fields as racs2_user_msg.h in jaxa/racs2_bridge (Apache-2.0), with the
 * telemetry header type of cFS v7.
 */
#ifndef RACS2_USER_MSG_H
#define RACS2_USER_MSG_H

#include "cfe.h"

/* Message ID the RACS2 bridge client forwards to ROS 2 (same as jaxa/racs2_bridge) */
#define RACS2_BRIDGE_MID 0x1EFE

#define ROS2_TOPIC_NAME_LNGTH 32
#define BODY_DATA_MAX_LNGTH   128

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    char                      ros2_topic_name[ROS2_TOPIC_NAME_LNGTH];
    uint8                     body_data_length;
    uint8                     body_data[BODY_DATA_MAX_LNGTH];
} racs2_user_msg_t;

#endif /* RACS2_USER_MSG_H */
