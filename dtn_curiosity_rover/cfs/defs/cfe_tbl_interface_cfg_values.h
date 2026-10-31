/*
 * Table name limits for the DTN demo.
 *
 * BPNode registers a table named "MIBConfigPNTable" (16 characters), which
 * does not fit cFE's default CFE_MISSION_TBL_MAX_NAME_LENGTH of 16
 * (the limit includes the terminating NUL).
 */
#ifndef CFE_TBL_INTERFACE_CFG_VALUES_H
#define CFE_TBL_INTERFACE_CFG_VALUES_H

#define CFE_MISSION_TBL_CFGVAL(x)               DTN_CFE_MISSION_TBL_##x
#define DTN_CFE_MISSION_TBL_MAX_NAME_LENGTH     24
#define DTN_CFE_MISSION_TBL_MAX_FULL_NAME_LEN   48

#endif
