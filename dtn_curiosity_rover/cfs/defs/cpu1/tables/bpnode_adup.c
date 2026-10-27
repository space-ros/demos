/*
 * NASA Docket No. GSC-19,559-1, and identified as "Delay/Disruption Tolerant Networking 
 * (DTN) Bundle Protocol (BP) v7 Core Flight System (cFS) Application Build 7.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this 
 * file except in compliance with the License. You may obtain a copy of the License at 
 *
 * http://www.apache.org/licenses/LICENSE-2.0 
 *
 * Unless required by applicable law or agreed to in writing, software distributed under 
 * the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF 
 * ANY KIND, either express or implied. See the License for the specific language 
 * governing permissions and limitations under the License. The copyright notice to be 
 * included in the software is as follows: 
 *
 * Copyright 2025 United States Government as represented by the Administrator of the 
 * National Aeronautics and Space Administration. All Rights Reserved.
 *
 */

/*
 * Modified for the space-ros/demos dtn_curiosity_rover demo: channel 0 carries the RACS2 user messages (MsgID 0x1EFE, rover drive commands) to the rover.
 */

#include "cfe.h"
#include "fwp_adup.h"
#include "cfe_tbl_filedef.h" /* Required to obtain the CFE_TBL_FILEDEF macro definition */
#include "bpnode_msgids.h"

BPA_ADUP_Config_t ADUProxyTable[BPLIB_MAX_NUM_CHANNELS] = {
    {
        .SendToMsgId = CFE_SB_MSGID_WRAP_VALUE(BPNODE_ADU_OUT_SEND_TO_MID),
        .NumRecvFrmMsgIds = 1,
        .RecvFrmMsgIds = {
            CFE_SB_MSGID_WRAP_VALUE(0x1EFE),
        },
        .MsgLims = {
            10,
        }
    },
    {
        .SendToMsgId = CFE_SB_MSGID_WRAP_VALUE(BPNODE_ADU_OUT_SEND_TO_MID),
        .NumRecvFrmMsgIds = 0
    }
};

CFE_TBL_FILEDEF(ADUProxyTable, BPNODE.ADUProxyTable, ADU Proxy Config Table, bpnode_adup.tbl)
