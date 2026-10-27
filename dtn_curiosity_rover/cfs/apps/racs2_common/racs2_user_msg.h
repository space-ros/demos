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

/*
 * Short fingerprint of a message body (32-bit FNV-1a), logged by run_app and
 * racs2_bridge_client so a command can be matched to its arrival on the rover.
 */
static inline uint32 racs2_body_hash(const uint8 *Data, size_t Len)
{
    uint32 Hash = 2166136261u;
    size_t i;

    for (i = 0; i < Len; i++)
    {
        Hash = (Hash ^ Data[i]) * 16777619u;
    }
    return Hash;
}

#endif /* RACS2_USER_MSG_H */
