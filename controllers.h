/*
 * controllers.h
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

#ifndef CONTROLLERS_H_
#define CONTROLLERS_H_

void on_off_controller(u16);

void proportional_controller(u16);

void pid_controller(u16);


#endif /* CONTROLLERS_H_ */
