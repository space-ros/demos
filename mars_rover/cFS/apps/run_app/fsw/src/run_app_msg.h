#ifndef _run_app_msg_h_
#define _run_app_msg_h_

/*
** SAMPLE App command codes
*/
#define SAMPLE_APP_NOOP_CC                 0
#define SAMPLE_APP_RESET_COUNTERS_CC       1

/*************************************************************************/
/*
** Type definition (generic "no arguments" command)
*/
typedef struct
{
   uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];

} SAMPLE_NoArgsCmd_t;

/*************************************************************************/
/*
** Type definition (SAMPLE App housekeeping)
*/
typedef struct 
{
    uint8              TlmHeader[CFE_SB_TLM_HDR_SIZE];
    uint8              sample_command_error_count;
    uint8              sample_command_count;
    uint8              spare[2];

}   OS_PACK sample_hk_tlm_t  ;

#define SAMPLE_APP_HK_TLM_LNGTH   sizeof ( sample_hk_tlm_t )
#define RUN_APP_LISTENER_LNGTH   sizeof ( sample_hk_tlm_t )

#endif /* _run_app_msg_h_ */
