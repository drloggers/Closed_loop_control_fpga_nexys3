/*
 * hardware_pwm_detect_driver.h
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

#ifndef HARDWARE_PWM_DETECT_DRIVER_H_
#define HARDWARE_PWM_DETECT_DRIVER_H_

#include "xbasic_types.h"
#include "xstatus.h"

XStatus init_hardware_pwm_detect(u32);

u32 getHighCount(u32);

u32 getLowCount(u32);

u16 getSensorAvgCount(u32);

u16 getStableCount(u32);

u16   freq2count(u32 );

float count2duty_cycle(u32 ,u32 );

float count2freq(u32 ,u32);

float freq2vtg(u32 );

float count2vtg(u16);

#endif /* HARDWARE_PWM_DETECT_DRIVER_H_ */
