#ifndef _sample_talker_h_
#define _sample_talker_h_

/*
** Required header files.
*/
#include "cfe.h"
#include "cfe_error.h"
#include "cfe_evs.h"
#include "cfe_sb.h"
#include "cfe_es.h"

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>


#define SAMPLE_PIPE_DEPTH                     32

/*
** Local function prototypes.
**
** Note: Except for the entry point (SAMPLE_TALKER_Main), these
**       functions are not called from any other source module.
*/
void SAMPLE_TALKER_Main(void);
void SAMPLE_TAKLKER_Init(void);
void SAMPLE_TALKER_ProcessCommandPacket(void);
void SAMPLE_TALKER_ProcessGroundCommand(void);
void SAMPLE_TALKER_ReportHousekeeping(void);
void SAMPLE_TALKER_ResetCounters(void);

bool SAMPLE_TALKER_VerifyCmdLength(CFE_SB_MsgPtr_t msg, uint16 ExpectedLength);

#endif /* _sample_talker_h_ */
