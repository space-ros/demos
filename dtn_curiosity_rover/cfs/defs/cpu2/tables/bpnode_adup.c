/*
 * NASA Docket No. GSC-18,587-1 and identified as “The Bundle Protocol Core Flight
 * System Application (BP) v6.5”
 *
 * Copyright © 2020 United States Government as represented by the Administrator of
 * the National Aeronautics and Space Administration. All Rights Reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * Modified for the space-ros/demos dtn_curiosity_rover demo: nothing is sent from the rover node; it only receives on channel 0.
 */

#include "cfe.h"
#include "fwp_adup.h"
#include "cfe_tbl_filedef.h" /* Required to obtain the CFE_TBL_FILEDEF macro definition */
#include "bpnode_msgids.h"

BPA_ADUP_Config_t ADUProxyTable[BPLIB_MAX_NUM_CHANNELS] = {
    {
        .SendToMsgId = CFE_SB_MSGID_WRAP_VALUE(BPNODE_ADU_OUT_SEND_TO_MID),
        .NumRecvFrmMsgIds = 0
    },
    {
        .SendToMsgId = CFE_SB_MSGID_WRAP_VALUE(BPNODE_ADU_OUT_SEND_TO_MID),
        .NumRecvFrmMsgIds = 0
    }
};

CFE_TBL_FILEDEF(ADUProxyTable, BPNODE.ADUProxyTable, ADU Proxy Config Table, bpnode_adup.tbl)
