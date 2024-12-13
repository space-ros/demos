#ifndef _racs2_user_msg_h_
#define _racs2_user_msg_h_

/*
** Type definition (user data format for racs2 bridge)
*/
#define ROS2_TOPIC_NAME_LNGTH   32
#define BODY_DATA_MAX_LNGTH   128

typedef struct 
{
    uint8              TlmHeader[CFE_SB_TLM_HDR_SIZE];
    char               ros2_topic_name[ROS2_TOPIC_NAME_LNGTH];
    uint8              body_data_length;
    uint8              body_data[BODY_DATA_MAX_LNGTH];

}   OS_PACK racs2_user_msg_t  ;

#define RACS2_USER_MSG_LNGTH   sizeof ( racs2_user_msg_t )
#define RACS2_BRIDGE_USER_LISTENER_LNGTH   sizeof ( racs2_user_msg_t )

#endif /* _racs2_user_msg_h_ */
