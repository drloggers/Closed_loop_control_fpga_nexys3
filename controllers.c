/*
 * controllers.c
 *
 *  Created on: May 13, 2014
 *
 *  Author : Sanket Borhade & Sameer Ghewari
 *
 *  Description :
 *  			Contains function which implement closed loop function. Different controllers
 *  			are implemented and values are global values to help change using custom
 *  			display and control setup.
 */
#include "xbasic_types.h"
#include "xtmrctr.h"
#include "hardware_pwm_detect_driver.h"

#define NUM_FRQ_SAMPLES			150
// Min and Max duty cycle for step and characterization tests
#define STEPDC_MIN				1
#define STEPDC_MAX				99

extern float 	prev_error ;
extern float 	deriv;
extern float 	integral;
extern int	 	offset, prop_gain, int_gain, deri_gain;
extern Xfloat32 set_point;
extern int 		pwm_duty;
extern int      pwm_freq;
extern XTmrCtr  PWMTimerInst;

/***********************************************/
/*					Controllers				   */
/***********************************************/

/**************
 *
 * ON-OFF (Bang-Bang controller)
 *
 */
void on_off_controller(u16 count){
	Xfloat32 actual_value;
	XStatus status;

	actual_value = count2vtg(count);
	if(actual_value < set_point)
		pwm_duty = STEPDC_MAX;
	else
		pwm_duty = STEPDC_MIN;
	status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
	if (status == XST_SUCCESS)	{
			PWM_Start(&PWMTimerInst);
	}
}

/*************
 *
 * Proportional Controller
 *
 */
void proportional_controller(u16 count){
	Xfloat32 actual_value;
		XStatus status;

		actual_value = count2vtg(count);
		pwm_duty = offset + (set_point - actual_value) * prop_gain;
		if(pwm_duty > STEPDC_MAX)
			pwm_duty = STEPDC_MAX;
		if(pwm_duty < STEPDC_MIN)
			pwm_duty = STEPDC_MIN;
		status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
		if (status == XST_SUCCESS)	{
				PWM_Start(&PWMTimerInst);
		}
}

/*************
 *
 * Proportional - Integral - Derivitive Controller
 *
 */
void pid_controller(u16 count){
	Xfloat32 actual_value;
	float error;
	XStatus status;

	actual_value = count2vtg(count);
	error = set_point - actual_value;
	deriv = error - prev_error;
	prev_error = error;

	if(error < (set_point/5))
		integral += error;
	else
		integral = 0;

	pwm_duty = offset + (error * prop_gain) + (deriv * deri_gain) + (integral * int_gain);
	if(pwm_duty > STEPDC_MAX)
		pwm_duty = STEPDC_MAX;
	if(pwm_duty < STEPDC_MIN)
		pwm_duty = STEPDC_MIN;
	status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
	if (status == XST_SUCCESS)	{
		PWM_Start(&PWMTimerInst);
	}
}
