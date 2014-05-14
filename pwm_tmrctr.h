/*****
 * pwm_tmrctr.h - Header file for PWM API for Xilinx timer/counter (tmrctr)
 *
 * Copyright Roy Kravitz, 2009-2013, 2014, 2015
 *
 * 
 * Author:	Roy Kravitz
 * Version:	1.2
 * Date:	29-March-14
 *
 * Revision History
 * ================
 * 05-Jan-09	RK		Created the first version
 * 16-Apr-13	RK		Assigned PLB_CLOCK_FREQ_HZ from xparameters.h instead of hardwiring it to 50MHz
 * 29-Mar-14	RK		Minor edits.  No functional changes
 *
 *
 * Description:
 * ============
 * This file contain the constant definitions and function prototypes for pwm_tmrctr.c.  
 * pwm_tmrctrc.c provides an API for Pulse-width modulation using the Xilinx timer/counter (tmrctr IP).  The API is
 * provided because the Xilinx timer/counter driver does not support PWM mode.  This driver is based on the
 * high-level driver model supported by Xilinx and borrows/adapts code from the tmrctr driver source code.
 *
 *****/
 
#ifndef PWM_TMRCTR_H	/* prevent circular inclusions */
#define PWM_TMRCTR_H	/* by using protection macros */

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#include "stdbool.h"
#include "math.h"
#include "xbasic_types.h"
#include "xstatus.h"
#include "xparameters.h"
#include "xtmrctr.h"

/************************** Constant Definitions *****************************/
#define PLB_CLOCK_FREQ_HZ	XPAR_PROC_BUS_0_FREQ_HZ
#define PWM_TIMER_WIDTH		32
#define PWM_MAXCNT			4294967295.00

#define PWM_PERIOD_TIMER	0
#define PWM_DUTY_TIMER		1

/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/
XStatus PWM_Initialize(XTmrCtr *InstancePtr, Xuint16 DeviceId, bool EnableInterrupts);
XStatus PWM_Start(XTmrCtr *InstancePtr);
XStatus PWM_Stop(XTmrCtr *InstancePtr);
XStatus PWM_SetParams(XTmrCtr *InstancePtr, Xuint32 freq, Xuint32 dutyfactor);
XStatus PWM_GetParams(XTmrCtr *InstancePtr, Xuint32 *freq, Xuint32 *dutyfactor);

/************************** Variable Definitions *****************************/

#ifdef __cplusplus
}
#endif

#endif /* end of protection macro */
