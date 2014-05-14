/*
 * hardware_pwm_detect_driver.c
 *
 *  Created on: May 4, 2014
 *
 *  Author: Sanket Borhade & Sameer Ghewari
 *
 *  Revision : v1.0
 *
 *  Description:
 *  			Driver API for the frequency detection hardware.
 */

#include "hardware_pwm_detect_driver.h"
#include "hardware_pwm_detect.h"

/******* Number of sample value for averaging the signal*********/
#define SAMPLE_VAL 40



/******
 * Initializes the pwm detection hardware
 *
 */
XStatus init_hardware_pwm_detect(u32 baseAddress){
	// perform self test
	XStatus status;
	status = HARDWARE_PWM_DETECT_SelfTest(&baseAddress);
	if (status != XST_SUCCESS){
		return (XST_FAILURE);
	}
	//reset the device to start operation
	HARDWARE_PWM_DETECT_mReset(baseAddress);
	return XST_SUCCESS;
}

/*****
 * returns high count of the input signal
 *
 */
u32 getHighCount(u32 baseAddress){
	return HARDWARE_PWM_DETECT_mReadReg(baseAddress,HARDWARE_PWM_DETECT_SLV_REG0_OFFSET);
}

/*****
 * returns low count of the input signal
 *
 */
u32 getLowCount(u32 baseAddress){
	return HARDWARE_PWM_DETECT_mReadReg(baseAddress,HARDWARE_PWM_DETECT_SLV_REG1_OFFSET);
}

/*
 * calculates duty cycle based on the high level count and low level counts of the
 * input signal
 *
 */
Xfloat32 count2duty_cycle(u32 highCount,u32 lowCount){
	 return highCount *100/ (highCount + lowCount);
}

/*
 * returns frequency considering the processor run on 66.66MHz and requires high count
 * and low count of the input signal
 *
 */
float count2freq(u32 highCount,u32 lowCount){
	 return (66660000/((float)(highCount + lowCount)));
 }

/*
 * returns average value of the detected sensor readings, number of samples depends on
 * constant defined
 *
 */
u16 getSensorAvgCount(u32 baseAddress){
	char temp_idx;
			int freq_temp[SAMPLE_VAL],avg;

			for(temp_idx =0;temp_idx<SAMPLE_VAL;temp_idx++){
				freq_temp[temp_idx] = (count2freq(getHighCount(baseAddress),getLowCount(baseAddress)));
			}
			avg =0 ;
			for(temp_idx =0;temp_idx<SAMPLE_VAL;temp_idx++){
				avg += freq_temp[temp_idx];
			}
			avg/=SAMPLE_VAL;
			return (avg/23.72);
}

/*
 * returns the value of detected sensor value when it settles to nearly constant value
 *
 */
u16 getStableCount(u32 baseAddress){
	int previous=0;
	int current;
	while(1){
	current = count2freq(getHighCount(baseAddress),getLowCount(baseAddress));
	if(abs(previous - current) < 5000)
		previous = current;
	else
		return current;
	}
	return current;
}

/*
 * returns voltage between 0-3.3v depending on the input frequency
 * the range is scaled based on charactersitics curve
 */
float freq2vtg(u32 frequency){

	 return ((frequency+5383.5)*(0.0000326765)) ;
}

/*
 * returns count value scaled to 0-4095 for Serial Charter software using frequency
 *
 */
u16 freq2count(u32 frequency){
	 return frequency/23.72;
}

/*
 * returns converted voltage based on frequency counter value
 */
float count2vtg(u16 count){
	 return ((0.0007718937)* (count+227.91));
}
