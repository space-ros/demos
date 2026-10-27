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
 * Modified for the space-ros/demos dtn_curiosity_rover demo: contact 0 routes all of ipn:100 to 172.28.10.10 (the other node on the demo's DTN link), with a 30 s custody retransmit timeout and 1 s custody signal trigger.
 */

/*
** \file
**   This file contains the source code for the Contacts Table
*/

#include "cfe_tbl_filedef.h" /* Required to obtain the CFE_TBL_FILEDEF macro definition */
#include "bpnode_tbl.h"
#include "bplib.h"


BPLib_CLA_ContactsTable_t ContactsTable = {
    .ContactSet = {
            {
                .DestEIDs = {
                    {
                        .Scheme       = BPLIB_EID_SCHEME_IPN,
                        .IpnSspFormat = BPLIB_EID_IPN_SSP_FORMAT_TWO_DIGIT,
                        .MaxAllocator = 0,
                        .MinAllocator = 0,
                        .MaxNode      = 100,
                        .MinNode      = 100,
                        .MaxService   = 255,
                        .MinService   = 0
                    }
                },
                .CLAType             = BPLib_UDP_CLA,
                .ClaInAddr           = "0.0.0.0",
                .ClaOutAddr          = "172.28.10.10",
                .ClaInPort           = 4551,
                .ClaOutPort          = 4501,
                .RetransmitTimeout   = 30000,    /* 30 secs, must exceed the round-trip light time */
                .CSTimeTrigger       = 1000,     /* 1 sec */
                .CSSizeTrigger       = (BPLIB_MINIMUM_ENCODED_CCS_LEN + 10),
                .IngressBitsPerCycle = 20000000, /* Ingress rate, 20 Mb per cycle */
                .EgressBitsPerCycle  = 20000000  /* Egress rate, 20 Mb per cycle */
            },
            {
                .DestEIDs = {
                    {
                        .Scheme       = BPLIB_EID_SCHEME_IPN,
                        .IpnSspFormat = BPLIB_EID_IPN_SSP_FORMAT_TWO_DIGIT,
                        .MaxAllocator = 0,
                        .MinAllocator = 0,
                        .MaxNode      = BPNODE_EID_NODE_NUM_FOR_CONTACT_1,
                        .MinNode      = BPNODE_EID_NODE_NUM_FOR_CONTACT_1,
                        .MaxService   = BPNODE_EID_SERVICE_NUM_FOR_CONTACT_1,
                        .MinService   = BPNODE_EID_SERVICE_NUM_FOR_CONTACT_1
                    }
                },
                .CLAType                = BPLib_SB_CLA,
                .ClaInAddr              = "",
                .ClaOutAddr             = "",
                .ClaInPort              = 0,
                .ClaOutPort             = 0,
                .RetransmitTimeout      = 30000,    /* 30 secs */
                .CSTimeTrigger          = 10000,    /* milliseconds */
                .CSSizeTrigger          = (BPLIB_MINIMUM_ENCODED_CCS_LEN + 10),
                .IngressBitsPerCycle    = 20000000, /* Ingress rate, 20 Mb per cycle */
                .EgressBitsPerCycle     = 20000000  /* Egress rate, 20 Mb per cycle */
            },
            {
                .DestEIDs = {
                    {
                        .Scheme       = BPLIB_EID_SCHEME_IPN,
                        .IpnSspFormat = BPLIB_EID_IPN_SSP_FORMAT_TWO_DIGIT,
                        .MaxAllocator = 0,
                        .MinAllocator = 0,
                        .MaxNode      = 100,
                        .MinNode      = 100,
                        .MaxService   = 0,
                        .MinService   = 0
                    }
                },
                .CLAType             = BPLib_UDP_CLA,
                .ClaInAddr           = "0.0.0.0",
                .ClaOutAddr          = "127.0.0.1",
                .ClaInPort           = 4552,
                .ClaOutPort          = 4502,
                .RetransmitTimeout   = 30000,    /* 30 secs */
                .CSTimeTrigger       = 10000,    /* milliseconds */
                .CSSizeTrigger       = (BPLIB_MINIMUM_ENCODED_CCS_LEN + 10),
                .IngressBitsPerCycle = 20000000,      /* Ingress rate, 20 Mb per cycle */
                .EgressBitsPerCycle  = 20000000       /* Egress rate, 20 Mb per cycle */
            }
        }
};

/*
** The macro below identifies:
**    1) the data structure type to use as the table image format
**    2) the name of the table to be placed into the cFE Example Table File Header
**    3) a brief description of the contents of the file image
**    4) the desired name of the table image binary file that is cFE compatible
*/
CFE_TBL_FILEDEF(ContactsTable, BPNODE.ContactsTable, Contacts Setup Table, bpnode_contacts.tbl)