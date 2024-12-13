#ifndef _run_app_h_
#define _run_app_h_

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

/***********************************************************************/

#define SAMPLE_PIPE_DEPTH                     32

/************************************************************************
** Type Definitions
*************************************************************************/

/****************************************************************************/
/*
** Local function prototypes.
**
** Note: Except for the entry point (RUN_APP_Main), these
**       functions are not called from any other source module.
*/
void RUN_APP_Main(void);
void SAMPLE_TAKLKER_Init(void);
void RUN_APP_ProcessCommandPacket(void);
void RUN_APP_ProcessGroundCommand(void);
void RUN_APP_ReportHousekeeping(void);
void RUN_APP_ResetCounters(void);

bool RUN_APP_VerifyCmdLength(CFE_SB_MsgPtr_t msg, uint16 ExpectedLength);

#endif /* _run_app_h_ */
