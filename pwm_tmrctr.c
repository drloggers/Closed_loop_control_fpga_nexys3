/*****
 * pwm_tmrctr.c - PWM API for Xilinx timer/counter (tmrctr)
 *
 * Copyright Roy Kravitz, 2009-2013, 2014, 2015
 *
 * 
 * Author:	Roy Kravitz
 * Version:	1.2
 * Date:	29-March-2014
 *
 * Revision History
 * ================
 * 05-Jan-09	RK		Created the first version
 * 16-Apr-13	RK		No changes in this file but changed pwm_tmrctr.h to take the bus
 *						frequency from xparameters.h instead of hardwiring it to 50MHz.
 *29-Mar-14		RK		Minor edits.  No functional changes.
 *
 *
 * Description:
 * ============
 * This file provides an API for Pulse-width modulation using the Xilinx timer/counter (tmrctr IP).  It is
 * provided because the Xilinx timer/counter driver does not support PWM mode.  This driver is based on the
 * high-level driver model supported by Xilinx and borrows/adapts code from the tmrctr driver source code.
 *
 *****/
 
/***************************** Include Files *********************************/
#include "pwm_tmrctr.h"


/************************** Constant Definitions *****************************/


/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/


/************************** Variable Definitions *****************************/


/*****************************************************************************/
/**
* PWM_Initialize()- Initializes a  timer/counter instance/driver for PWM use. 
*
* Initialize fields of the XTmrCtr structure and set the control bits for PWM usage.
* Uses both high level and low level tmrctr driver functions
*
* @param    InstancePtr is a pointer to the XTmrCtr instance to be used for PWM.
* @param    DeviceId is the unique id of the device controlled by this XTmrCtr
*           component.  Passing in a device id associates the generic XTmrCtr
*           component to a specific device, as chosen by the caller or
*           application developer.
* @param	EnableInterrupts - says to enable interrupts for the PWM timer/countere
*
* @return
*
*   - XST_SUCCESS if initialization was successful
*   - XST_DEVICE_IS_STARTED if the device has already been started
*   - XST_DEVICE_NOT_FOUND if the device doesn't exist
*
* @note
*
* None.
*
******************************************************************************/
XStatus PWM_Initialize(XTmrCtr *InstancePtr, Xuint16 DeviceId, bool EnableInterrupts)
{
    Xuint32 StatusReg;
    u32		PWM_BaseAddress;
    u32		ctlbits;
    
    // Initialize the timer/counter instance
    // This clears  both timer registers and any pending interrupts
    StatusReg = XTmrCtr_Initialize(InstancePtr, DeviceId);
    if (StatusReg != XST_SUCCESS) // failed to initialize.  Return the reason
    {
	    return StatusReg;
    }

    // successfully initialized the timer/ctr instance
	// initialize timer to PWM mode with interrupts enabled
	PWM_BaseAddress = InstancePtr->BaseAddress;
	if (EnableInterrupts)
	{
		ctlbits = XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK  | XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_DOWN_COUNT_MASK | XTC_CSR_ENABLE_INT_MASK;
	}
	else
	{
		ctlbits = XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK  | XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_DOWN_COUNT_MASK;
	}	
	XTmrCtr_SetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER, ctlbits);
	XTmrCtr_SetControlStatusReg(PWM_BaseAddress, PWM_DUTY_TIMER, ctlbits);
	return XST_SUCCESS;
}


/*****************************************************************************/
/**
* PWM_Start() - Starts the specified PWM instance
*
* Starts the specified PWM instance of the device such that it starts running.
* The timer counter is reset before it is started and the reset value is
* loaded into the timer counter.  Assumes that the PWM timer instance has been
* initialized successfully and that the compare (Load) registers have been
* loaded with the period (TLR0) and duty cycle (TLR1)
*
* @param    InstancePtr is a pointer to the PWM instance to be worked on.
*
* @return
*
*   - XST_SUCCESS if the PWM timers were started
*   - XST_FAILURE if the PWM instance is not initialized
*
* @note
*
* None.
*
******************************************************************************/
XStatus PWM_Start(XTmrCtr *InstancePtr)
{
	u32		ctlbits;
    u32		PWM_BaseAddress;

    if (InstancePtr->IsReady != XCOMPONENT_IS_READY) // check that instance is initialized
    {
	    return XST_FAILURE;
    }
	
    // instance was initialized - reset (load TLRx) the timers 
    PWM_BaseAddress = InstancePtr->BaseAddress;
    XTmrCtr_LoadTimerCounterReg(PWM_BaseAddress, PWM_PERIOD_TIMER);
	ctlbits = XTmrCtr_GetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER) & 0xFFFFFFDF;  // clear load bits
	XTmrCtr_SetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER, ctlbits);
		  
    XTmrCtr_LoadTimerCounterReg(PWM_BaseAddress, PWM_DUTY_TIMER);
	ctlbits = XTmrCtr_GetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER) & 0xFFFFFFDF;  // clear load bits
	XTmrCtr_SetControlStatusReg(PWM_BaseAddress, PWM_DUTY_TIMER, ctlbits);
	
	// and enable (start) both timers  - ENABLE-ALL is shadowed in both TCSR registers
	ctlbits = XTmrCtr_GetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER);
	ctlbits |= XTC_CSR_ENABLE_ALL_MASK;
	XTmrCtr_SetControlStatusReg(PWM_BaseAddress, PWM_PERIOD_TIMER, ctlbits);
	return XST_SUCCESS;								
}


/*****************************************************************************/
/**
*
* PWM_Stop() - Stops the specified PWM instance
*
* Stops the specified PWM instance of the device. Assumes that the PWM timer 
* instance has been initialized successfully.
*
* @param    InstancePtr is a pointer to the PWM instance to be worked on.
*
* @return
*
*   - XST_SUCCESS if the PWM timers were stopped
*   - XST_FAILURE if the PWM instance is not initialized
*
* @note
*
* None.
*
******************************************************************************/
XStatus PWM_Stop(XTmrCtr *InstancePtr)
{
    u32		PWM_BaseAddress;

    if (InstancePtr->IsReady != XCOMPONENT_IS_READY) // check that instance is initialized
    {
	    return XST_FAILURE;
    }
	
    // instance was initialized - stop the timers
    PWM_BaseAddress = InstancePtr->BaseAddress;
	XTmrCtr_Disable(PWM_BaseAddress, PWM_PERIOD_TIMER);
	XTmrCtr_Disable(PWM_BaseAddress, PWM_DUTY_TIMER);
	return XST_SUCCESS;
}


/*****************************************************************************/
/**
*
* PWM_SetParams() - Set the PWM parameters
*
* Sets the frequency and duty cycle for the PWM.  Stops the PWM timers but does not
* restart them.  Assumes that the PWM timer instance has been initialized and that the 
* timer is running at the PLB bus frequency (PLB_CLOCK_FREQ_HZ) as defined in pwm_tmrctr.h 
*
* @param    InstancePtr is a pointer to the PWM instance to be worked on.
* @param    PWM frequency (in Hz).
* @param	PWM high time (in pct of PWM period - 0 to 100)
*
* @return
*
*   - XST_SUCCESS if the PWM parameters were loaded
*   - XST_FAILURE if the PWM instance is not initialized
*	- XST_INVALID_PARAM if one or both of the parameters is invalid
*
* @note
*
* Formulas for calculating counts (PWM counters are configured as down counters):
* 	TLR0 (PWM period count) = (PWM_PERIOD / PLB_CLOCK_PERIOD) - 2
* 	TLR1 (PWM duty cycle count) = MAX( 0, (((PWM_PERIOD * (DUTY CYCLE / 100)) / PLB_CLOCK_PERIOD) - 2) )
* 
******************************************************************************/
XStatus PWM_SetParams(XTmrCtr *InstancePtr, Xuint32 freq, Xuint32 dutyfactor)
{
	u32		PWM_BaseAddress;
	float	plb_clock_period,	
			pwm_period,
			pwm_dc,
			tlr0,
			tlr1;
     	
    if (InstancePtr->IsReady != XCOMPONENT_IS_READY) // check that instance is initialized
    {
	    return XST_FAILURE;
    }
    	   
    // calculate the PWM period and high time
	plb_clock_period = 1.0 / (float) PLB_CLOCK_FREQ_HZ;
	pwm_period = 1.0 / freq;
	tlr0 = (pwm_period / plb_clock_period) - 2;
	
	pwm_dc = dutyfactor / 100.00;
	tlr1 = ((pwm_period * pwm_dc) / plb_clock_period) - 2;
	if (tlr1 < 0)   // duty cycle cannot be less than 0%
	{
		tlr1 = 0.0;
	}

	// check to see if parameters are valid
    if (dutyfactor > 100)  // cannot have a duty cylce > 100%
    {
	   return XST_INVALID_PARAM;
	}
	if ((tlr0 > PWM_MAXCNT) || (tlr1 > PWM_MAXCNT))  // period or high time is too big for the timer/counter registers
	{
		return XST_INVALID_PARAM;
	}
	   
	// period and duty cycle are within range of timer - stop timer and write values to load registers   
    PWM_Stop(InstancePtr);
    PWM_BaseAddress = InstancePtr->BaseAddress;
    XTmrCtr_SetLoadReg(PWM_BaseAddress, PWM_PERIOD_TIMER, (u32) tlr0);
  	XTmrCtr_SetLoadReg(PWM_BaseAddress, PWM_DUTY_TIMER, (u32) tlr1);
	return XST_SUCCESS;
}


/*****************************************************************************/
/**
*
* PWM_GetParams() - Get the PWM parameters
*
* Returns the frequency (Hz) and duty cycle (%) for the PWM.  Stops the PWM timers but does not
* restart them.  Assumes that the PWM timer instance has been initialized and that the
* timer is running at the PLB bus frequency (PLB_CLOCK_FREQ_HZ) as defined in pwm_tmrctr.h 
*
* @param    InstancePtr is a pointer to the PWM instance to be worked on.
* @param    pointer to PWM frequency (in Hz).
* @param	pointer to PWM high time (in pct of PWM period - 0 to 100)
*
* @return
*
*   - XST_SUCCESS if the PWM parameters were loaded
*   - XST_FAILURE if the PWM instance is not initialized
*	- XST_INVALID_PARAM if one or both of the parameters is invalid
*
* @note
*
* Formulas for calculating counts (PWM counters are configured as down counters):
*		PLB_CLOCK_PERIOD = 1 / PLB_CLOCK_FREQ
*		PWM_PERIOD = (TLR0 + 2) x (1 / PLB_CLOCK_FREQ)
*		PWM_HIGH_TIME = (TLR1 + 2) x (1 / PLB_CLOCK_FREQ)
*
******************************************************************************/
XStatus PWM_GetParams(XTmrCtr *InstancePtr, Xuint32 *freq, Xuint32 *dutyfactor)
{
	u32		PWM_BaseAddress;
	float	plb_clock_period,	
			pwm_period,
			pwm_dc,
			tlr0,
			tlr1;
			
	u32		tlr;
     	
    if (InstancePtr->IsReady != XCOMPONENT_IS_READY) // check that instance is initialized
    {
	    return XST_FAILURE;
    }
    
    // first stop the PWM timers and get Base Address of timer registers   
	PWM_Stop(InstancePtr);
	PWM_BaseAddress = InstancePtr->BaseAddress;

	// next read the load registers to get the period and high time 
 	tlr = XTmrCtr_GetLoadReg(PWM_BaseAddress, PWM_PERIOD_TIMER);
 	tlr0 = (float) tlr; 
 	tlr = XTmrCtr_GetLoadReg(PWM_BaseAddress, PWM_DUTY_TIMER);
 	tlr1 = (float) tlr;  	   

    // calculate the PWM period and high time
	plb_clock_period = 1.0 / PLB_CLOCK_FREQ_HZ;
	pwm_period = (tlr0 + 2) * plb_clock_period;
	pwm_dc = tlr1 / tlr0;
	
	// round the values and return them
	*freq = lroundf(1.00 / pwm_period);
	*dutyfactor = lroundf(pwm_dc * 100.00);
	return XST_SUCCESS;
}
        	
    	
