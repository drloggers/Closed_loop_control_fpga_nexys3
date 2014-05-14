/*****
 * Closed_loop_system.c - Program for Closed loop Control Systems to control the LED intensity.
 *
 * Copyright Roy Kravitz, 2013, 2014
 *
 *
 * Author:			Roy Kravitz
 * Modified by:		Sanket Borhade & Sameer Ghewari
 * Version:			4.0
 * Date:			6-May-2014
 *
 * Revision History
 * ================
 * 18-Apr-13	RK		Created the first version from test_PmodCtlSys_r1
 * 23-Apr-13	RK		Moved PmodCtlSys functions to their own driver and header files
 * 24-Apr-14    RM      Modified to remove references to PmodCtlSys.h calls and use student-made peripheral instead
 * 06-May-14   BS,GS    Modified to add on-off, proportional and PID controllers to form closed loop system
 *
 * Description:
 * ============
 * This program implements on-off (BANG BANG), Proportional and PID controllers to control the LED intensity at specific user
 * defined set-point forming a closed loop system. The system consist of custom hardware peripheral added using the XPS
 * custom peripheral wizard. The driver for this system is included in hardware_pwm_detect_driver files.
 *
 *
 * Switch operations:
 *
 *		sw[1:0] = 00:		Use rotary knob to dial in the set-point in volts. Display the set value in volts.
 *							Using On-Off (BANG-BANG) controller control the system at set-point.Press and hold
 *							the rotary encoder pushbutton to start the test.Release the button when the "Run"
 *							(rightmost) LED turns off to upload the data via serial port to a PC.
 *
 * 		sw[1:0] = 01:		Use rotary knob to dial in the set-point in volts. Display the set value in volts,
 * 							proportional gain and the offset value. Using Proportional controller control the
 * 							system at set-point. Press and hold the rotary encoder pushbutton to start the test.
 *                          Release the button when the "Run" (rightmost) LED turns off to upload the
 *                          data via serial port to a PC. The step function goes from 1% duty cycle to
 *                          99% duty cycle.
 *
 *`		sw[1:0] = 10:		Display shows various PID gain and offset setting values. Use north button to toggle
 *`							between different parameters. East and west button can be used to increase or decrease
 *`							the underlined parameter value. Press and hold the rotary encoder pushbutton to start
 *`							the test. Release the button when the "Run" (rightmost) LED turns off to upload the
 *                          data via serial port to a PC. The step function goes from 99% duty cycle to
 *                          1% duty cycle.
 *
 *		sw[1:0] = 11:		Characterizes the response to the system by stepping the PWM duty cycle from
 *							min (1%) to max (99%) after allowing the light sensor output to settle. Press and hold
 *							the rotary encoder pushbutton to start the test.  Release the button when the
 *							"Run" (rightmost) LED turns off to upload the data via serial port to a PC
 *
 *****/

/********** Include Files ***********/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "xparameters.h"
#include "xbasic_types.h"
#include "xtmrctr.h"
#include "xintc.h"
#include "xgpio.h"
#include "n3eif.h"
#include "pwm_tmrctr.h"
#include "mb_interface.h"
#include "controllers.h"
#include "hardware_pwm_detect_driver.h"

/************************** Macros and Constants ******************************/
// Microblaze Cache Parameters
#define	USE_ICACHE				XPAR_MICROBLAZE_0_USE_ICACHE
#define	USE_DCACHE				XPAR_MICROBLAZE_0_USE_DCACHE
#define USE_DCACHE_WRITEBACK	XPAR_MICROBLAZE_DCACHE_USE_WRITEBACK

// GPIO and N3EIF parameters
#define GPIO_DEVICE_ID			XPAR_XPS_GPIO_0_DEVICE_ID
#define GPIO_BASEADDR			XPAR_XPS_GPIO_0_BASEADDR
#define GPIO_HIGHADDR			XPAR_XPS_GPIO_0_HIGHADDR
#define GPIO_OUTPUT_CHANNEL		1

#define GPIO1_BASEADDR			XPAR_XPS_GPIO_1_BASEADDR
#define GPIO1_HIGHADDR			XPAR_XPS_GPIO_1_HIGHADDR
#define GPIO1_DEVICE_ID			XPAR_XPS_GPIO_1_DEVICE_ID
#define GPIO1_INPUT_CHANNEL0	1
#define GPIO1_INPUT_CHANNEL1	2

#define N3EIF_DEVICEID			XPAR_N3EIF_0_DEVICE_ID
#define N3EIF_BASEADDR			XPAR_N3EIF_0_BASEADDR
#define N3EIF_HIGHADDR			XPAR_N3EIF_0_HIGHADDR

// PWM timer parameters
// Set PWM frequency = 10KHz, duty cycle increments by 5%
#define PWM_TIMER_DEVICE_ID		XPAR_XPS_TIMER_0_DEVICE_ID
#define PWM_TIMER_BASEADDR		XPAR_XPS_TIMER_0_BASEADDR
#define PWM_TIMER_HIGHADDR		XPAR_XPS_TIMER_0_HIGHADDR
#define PWM_FREQUENCY			10000
#define PWM_VIN					3.3
#define DUTY_CYCLE_CHANGE		2

// Min and Max duty cycle for step and characterization tests
#define STEPDC_MIN				1
#define STEPDC_MAX				99

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_XPS_INTC_0_DEVICE_ID
#define INTC_BASEADDR			XPAR_XPS_INTC_0_BASEADDR
#define INTC_HIGHADDR			XPAR_XPS_INTC_0_HIGHADDR
#define TIMER_INTERRUPT_ID		XPAR_XPS_INTC_0_XPS_TIMER_0_INTERRUPT_INTR
#define FIT_INTERRUPT_ID		XPAR_XPS_INTC_0_FIT_TIMER_0_INTERRUPT_INTR

// Fixed Interval timer - 66.67MHz input clock, 5KHz output clock
// 1 msec time interval for FIT interrupt handler
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	XPAR_PROC_BUS_0_FREQ_HZ
#define FIT_COUNT				13333
#define FIT_CLOCK_FREQ_HZ		(FIT_IN_CLOCK_FREQ_HZ / FIT_COUNT)
#define FIT_COUNT_1MSEC			(FIT_CLOCK_FREQ_HZ / 1000)

// Hardware parameters to detect frequency
#define hw_pwm_det_ID 		XPAR_HARDWARE_PWM_DETECT_0_DEVICE_ID
#define hw_pwm_det_BASEADDR XPAR_HARDWARE_PWM_DETECT_0_BASEADDR
#define hw_pwm_det_HIGHADDR XPAR_HARDWARE_PWM_DETECT_0_HIGHADDR

// sample settings
#define NUM_FRQ_SAMPLES			150

// macro functions
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

/*****************************************************************************/

/****************************** typedefs and structures **********************/
typedef enum {BANG_BANG = 0x0, PROPORTIONAL = 0x01, PID = 0x02,
				TEST_CHARACTERIZE = 0x03, TEST_INVALID = 0xFF} Test_t;

/*****************************************************************************/

/************************** Global Variables *********************************/
// Microblaze peripheral instances
XIntc 	IntrptCtlrInst;								// Interrupt Controller instance
XTmrCtr	PWMTimerInst;								// PWM timer instance
XGpio	GPIOInst;									// GPIO instance
XGpio 	GPIO1Inst;									// GPIO instance for pwm detect

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
volatile unsigned long	timestamp;					// timestamp since the program began
volatile u32			gpio_port = 0;				// GPIO port register - maintained in program

// The following variables are shared between the functions in the program
// such that they must be global
u32						sample[NUM_FRQ_SAMPLES];	// sample array
u32                     duty_reg[NUM_FRQ_SAMPLES];
int						smpl_idx;					// index into sample array
int						frq_smple_interval;			// approximate sample interval

int						pwm_freq;					// PWM frequency
int						pwm_duty;					// PWM duty cycle

// Light Sensor Peripheral parameters
u32 frequency,duty;
/******
 * controller parameters
 ******/
float 					prev_error = 0;
float 					deriv = 0;
float 					integral = 0;
char 					x=1,y=7,prev_mode;
int 					offset = 30, prop_gain = 50, int_gain = 80, deri_gain = 85;
int 					prev_offset = 0, prev_prop_gain = 0, prev_int_gain = 0, prev_deri_gain = 0;
Xfloat32 				set_point = 0.1,prev_set_point = 0;

/*---------------------------------------------------------------------------*/
int						debugen = 0;				// debug level/flag
/*---------------------------------------------------------------------------*/

/*****************************************************************************/
/************************** Function Prototypes ******************************/
XStatus			DoTest_Characterize(void);									// Perform Characterization test
XStatus			do_init(void);												// initialize system
void			delay_msecs(u32 msecs);										// busy-wait delay for "msecs" milliseconds
void			voltstostrng(Xfloat32 v, char* s);							// converts volts to a string
void			FIT_Handler(void);											// fixed interval timer interrupt handler

/*****************************************************************************/

void user_menu(char);

void menu_display(char);

/*********************************************/
/*              Main Program                 */
/*********************************************/
int main()
{
	XStatus 	Status;
	u32			btnsw = 0x00000000, old_btnsw = 0x000000FF;
	int			rotcnt, old_rotcnt = 0x1000;
	Test_t		test, next_test;
	char        mode = 'P';
	char 		button_press_flag = 0,lcd_modify = 1,pwm_start = 0;

	// initialize devices and set up interrupts, etc.
 	Status = do_init();
 	if (Status != XST_SUCCESS) 	{
 		LCD_setcursor(1,0);
 		LCD_wrstring("****** ERROR *******");
 		LCD_setcursor(2,0);
 		LCD_wrstring("INIT FAILED- EXITING");
 		exit(XST_FAILURE);
 	}
 	Status = LCD_onCursor();
	// initialize the variables
	timestamp = 0;
	pwm_freq = PWM_FREQUENCY;
	pwm_duty = STEPDC_MIN;
	next_test = TEST_INVALID;

	// Enable the Microblaze caches and kick off the processing by enabling the Microblaze
	// interrupt.  This starts the FIT timer which updates the timestamp.
	if (USE_ICACHE == 1){
		microblaze_invalidate_icache();
		microblaze_enable_icache();
	}
	if (USE_DCACHE == 1){
		microblaze_invalidate_dcache();
		microblaze_enable_dcache();
	}
	microblaze_enable_interrupts();

 	// display the greeting
    LCD_setcursor(1,0);
    LCD_wrstring("    CLSystem    ");
	LCD_setcursor(2,0);
	LCD_wrstring("BY SANKET,SAMEER");
	NX3_writeleds(0xFF);
	//Run the LED characterization routine to establish sensor min's and max's
    DoTest_Characterize();
	NX3_writeleds(0x00);
	LCD_clrd();
	delay_msecs(100);

    // main loop - there is no exit except by hardware reset
	while (1){

		// read sw[1:0] to get the task to perform.
		NX3_readBtnSw(&btnsw);

		test = btnsw & (msk_SWITCH1 | msk_SWITCH0);

		pwm_start = btnsw & msk_SWITCH3;

		if (test == BANG_BANG)  // BANG BANG controller
		{
			if(test != next_test){
				menu_display('B');
				next_test = test;
			}
			lcd_modify = 0;

			user_menu('B');
			if((btnsw & msk_BTN_ROT) == 0x80)
				button_press_flag = 1;
			if((btnsw ^ old_btnsw) && (msk_BTN_ROT & btnsw)){
				lcd_modify = 0;
			Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_start==0? STEPDC_MIN:STEPDC_MAX);
			if (Status == XST_SUCCESS)	{
				PWM_Start(&PWMTimerInst);
			}
			delay_msecs(800);
			int smpl_idx = 0;
			while (smpl_idx < NUM_FRQ_SAMPLES){
				sample[smpl_idx] = getSensorAvgCount(hw_pwm_det_BASEADDR);
				on_off_controller(sample[smpl_idx++]);
			}
			prev_prop_gain = 0;
			prev_int_gain = 0;
			prev_deri_gain = 0;
			prev_offset = 0;
			prev_set_point = 0;
			NX3_writeleds(0x02);
			LCD_clrd();
			LCD_setcursor(1, 0);
			LCD_wrstring("Sending Data....");
			LCD_setcursor(2, 0);
			LCD_wrstring("S:    DATA:     ");
			Xfloat32 v;
			char s[10];
			// trigger the serial charter program
			xil_printf("\n Plot for BANG BANG controller response");
			xil_printf("===STARTPLOT===\n");
			// start with the second sample.  The first sample is not representative of
			// the data.  This will pretty-up the graph a bit
			for (smpl_idx = 1; smpl_idx < NUM_FRQ_SAMPLES; smpl_idx++)
			{
				u16 count;
				count = sample[smpl_idx];
				v = count2vtg(count);
				voltstostrng(v, s);
				xil_printf("%d\t%d\t%s\t%d\n\r", smpl_idx, count, s,pwm_duty);
				LCD_setcursor(2, 2);
				LCD_wrstring("   ");
				LCD_setcursor(2, 2);
				LCD_putnum(smpl_idx, 10);
				LCD_setcursor(2, 11);
				LCD_wrstring("     ");
				LCD_setcursor(2, 11);
				LCD_putnum(count, 10);
			}
			// stop the serial charter program
			xil_printf("===ENDPLOT===\n");
			NX3_writeleds(0x00);
			next_test = PROPORTIONAL;
		}
	}
	else if (test == PID){
		if(test != next_test){
			menu_display('P');
			next_test = test;
		}
		lcd_modify = 0;
		user_menu('P');
		if((btnsw ^ old_btnsw) && (msk_BTN_ROT & btnsw)){
			Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_start==0? STEPDC_MIN:STEPDC_MAX);
			if (Status == XST_SUCCESS)	{
				PWM_Start(&PWMTimerInst);
			}
			delay_msecs(800);
			int smpl_idx = 0;
			while (smpl_idx < NUM_FRQ_SAMPLES){
				sample[smpl_idx] = getSensorAvgCount(hw_pwm_det_BASEADDR);
				pid_controller(sample[smpl_idx++]);
			}
			prev_prop_gain = 0;
			prev_int_gain = 0;
			prev_deri_gain = 0;
			prev_offset = 0;
			prev_set_point = 0;
			NX3_writeleds(0x02);
			LCD_clrd();
			LCD_setcursor(1, 0);
			LCD_wrstring("Sending Data....");
			LCD_setcursor(2, 0);
			LCD_wrstring("S:    DATA:     ");
			Xfloat32 v;
			char s[10];
			// trigger the serial charter program
			xil_printf("\n Plot for PID controller response");
			xil_printf("===STARTPLOT===\n");
			// start with the second sample.  The first sample is not representative of
			// the data.
			for (smpl_idx = 1; smpl_idx < NUM_FRQ_SAMPLES; smpl_idx++){
				u16 count;
				count = sample[smpl_idx];
				v = count2vtg(count);
				voltstostrng(v, s);
				xil_printf("%d\t%d\t%s\t%d\t%d\n\r", smpl_idx, count, s,pwm_duty);
				LCD_setcursor(2, 2);
				LCD_wrstring("   ");
				LCD_setcursor(2, 2);
				LCD_putnum(smpl_idx, 10);
				LCD_setcursor(2, 11);
				LCD_wrstring("     ");
				LCD_setcursor(2, 11);
				LCD_putnum(count, 10);
			}
			// stop the serial charter program
			xil_printf("===ENDPLOT===\n");
			NX3_writeleds(0x00);
			next_test = PROPORTIONAL;
		}
	}
	else if(test == PROPORTIONAL){
		if(test != next_test){
			menu_display('D');
			next_test = test;
		}
		lcd_modify = 0;
		NX3_writeleds(0x00);
		user_menu('D');
		if((btnsw ^ old_btnsw) && (msk_BTN_ROT & btnsw)){
			lcd_modify = 1;
			Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_start==0? STEPDC_MIN:STEPDC_MAX);
			if (Status == XST_SUCCESS)	{
				PWM_Start(&PWMTimerInst);
			}
			delay_msecs(800);
			int smpl_idx = 0;
			while (smpl_idx < NUM_FRQ_SAMPLES){
				sample[smpl_idx] = getSensorAvgCount(hw_pwm_det_BASEADDR);
				proportional_controller(sample[smpl_idx++]);
			}
			prev_prop_gain = 0;
			prev_int_gain = 0;
			prev_deri_gain = 0;
			prev_offset = 0;
			prev_set_point = 0;
			NX3_writeleds(0x02);
			LCD_clrd();
			LCD_setcursor(1, 0);
			LCD_wrstring("Sending Data....");
			LCD_setcursor(2, 0);
			LCD_wrstring("S:    DATA:     ");
			Xfloat32 v;
			char s[10];
			// trigger the serial charter program
			xil_printf("\n Plot for Proportional controller response");
			xil_printf("===STARTPLOT===\n");
			// start with the second sample.  The first sample is not representative of
			// the data.
			for (smpl_idx = 1; smpl_idx < NUM_FRQ_SAMPLES; smpl_idx++){
				u16 count;
				count = sample[smpl_idx];
				v = count2vtg(count);
				voltstostrng(v, s);
				xil_printf("%d\t%d\t%s\t%d\t%d\n\r", smpl_idx, count, s,pwm_duty);
				LCD_setcursor(2, 2);
				LCD_wrstring("   ");
				LCD_setcursor(2, 2);
				LCD_putnum(smpl_idx, 10);
				LCD_setcursor(2, 11);
				LCD_wrstring("     ");
				LCD_setcursor(2, 11);
				LCD_putnum(count, 10);
			}
			// stop the serial charter program
			xil_printf("===ENDPLOT===\n");
			NX3_writeleds(0x00);
			next_test = PID;
		}
	}
	else if (test == TEST_CHARACTERIZE){  // Characterize Response
		if (test != next_test){
			LCD_clrd();
			LCD_setcursor(1,0);
			LCD_wrstring("|CHAR|Press RBtn");
			LCD_setcursor(2,0);
			LCD_wrstring("LED OFF-Release ");
		}
		// start the test on the rising edge of the Rotary Encoder button press
		// the test will write the samples into the global "sample[]"
		// the samples will be sent to stdout when the Rotary Encoder button
		// is released
		if ((btnsw ^ old_btnsw) && (msk_BTN_ROT & btnsw)) { // do the step test and dump data
			// light "Run" (rightmost) LED to show the test has begun
			// and do the test.  The test will return when the measured samples array
			// has been filled.  Turn off the rightmost LED after the data has been
			// captured to let the user know he/she can release the button
			NX3_writeleds(0x01);
			DoTest_Characterize();
			NX3_writeleds(0x00);
			// wait for the Rotary Encoder button to be released
			// and then send the sample data to stdout
			do{
				NX3_readBtnSw(&btnsw);
				delay_msecs(10);
			} while ((btnsw & msk_BTN_ROT) == 0x80);
			// light "Transfer" LED to indicate that data is being transmitted
			// Show the traffic on the LCD
			NX3_writeleds(0x02);
			LCD_clrd();
			LCD_setcursor(1, 0);
			LCD_wrstring("Sending Data....");
			LCD_setcursor(2, 0);
			LCD_wrstring("S:    DATA:     ");
			xil_printf("\n\rCharacterization Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);
			// trigger the serial charter program
			xil_printf("===STARTPLOT===\n\r");
			for (smpl_idx = STEPDC_MIN; smpl_idx <= 99; smpl_idx++)	{
				u16 		count;
				Xfloat32	v;
				char		s[10];
				count = sample[smpl_idx];
				v = count2vtg(count);
				voltstostrng(v, s);
				xil_printf("%d\t%d\t%s\t%d\n\r", smpl_idx, count, s,pwm_duty);

				LCD_setcursor(2, 2);
				LCD_wrstring("   ");
				LCD_setcursor(2, 2);
				LCD_putnum(smpl_idx, 10);
				LCD_setcursor(2, 11);
				LCD_wrstring("     ");
				LCD_setcursor(2, 11);
				LCD_putnum(count, 10);
			}
			// stop the serial charter program
			xil_printf("===ENDPLOT===\n\r");
			NX3_writeleds(0x00);
			old_btnsw = btnsw;
			next_test = TEST_INVALID;
		}  // do the step test and dump data
		else{
			next_test = test;
		}
	} // Test 3 - Characterize Response
	else { // outside the current test range - blink LED's and hang
		// should never get here but just in case
		NX3_writeleds(0xFF);
		delay_msecs(2000);
		NX3_writeleds(0x00);
	}
	// wait a bit and start again
	delay_msecs(100);
	}  // while(1) loop
 }  // end main()


/*****
 * DoTest_Characterize() - Perform the Characterization test
 *
 * This function starts the duty cycle at the minimum duty cycle and
 * then sweeps it to the max duty cycle for the test.
 * Samples are collected into the global array sample[].
 * The function toggles the TEST_RUNNING signal high for the duration
 * of the test as a debug aid and adjusts the global "pwm_duty"
 *
 * The test also sets the global frequency count min and max counts to
 * help limit the counts to the active range for the circuit
 *****/
XStatus DoTest_Characterize(void)
{
	XStatus		Status;					// Xilinx return status
	unsigned	tss;					// starting timestamp

	int			n;						// number of samples

	n = 0;
		tss = timestamp;
	// stabilize the PWM output (and thus the lamp intensity) at the
	// minimum before starting the test
	pwm_duty = STEPDC_MIN;
	Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
	if (Status == XST_SUCCESS)
	{
		PWM_Start(&PWMTimerInst);
	}
	else
	{
		return -1;
	}
	//Wait for the LED output to settle before starting
    delay_msecs(1500);

	// sweep the duty cycle from STEPDC_MIN to STEPDC_MAX
	smpl_idx = STEPDC_MIN;
	n = 0;
	tss = timestamp;
	while (smpl_idx <= STEPDC_MAX)
	{
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, smpl_idx);
		if (Status == XST_SUCCESS)
		{
			PWM_Start(&PWMTimerInst);
		}
		else
		{
			return -1;
		}

		sample[smpl_idx++] = getSensorAvgCount(hw_pwm_det_BASEADDR);
		n++;
		delay_msecs(50);
	}
	frq_smple_interval = (timestamp - tss) / smpl_idx;

    return n;
}


/*********************************************/
/*            Support Functions              */
/*********************************************/

/*****
 * do_init() - initialize the system
 *
 * This function is executed once at start-up and after a reset.  It initializes
 * the peripherals and registers the interrupt handlers
 *****/
XStatus do_init(void)
{
	XStatus 	Status;				// status from Xilinx Lib calls

	// initialize the N3EIF hardware and driver
	// rotary encoder is set to increment duty cycle from 0 by 5 w/
	// no negative counts
 	N3EIF_init(N3EIF_BASEADDR);
 	ROT_init(DUTY_CYCLE_CHANGE, true);
	ROT_clear();

	// initialize the GPIO instance
	Status = XGpio_Initialize(&GPIOInst, GPIO_DEVICE_ID);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO channel 1 is an 8-bit output port that your application can
	// use.  None of the bits are used by this program
	XGpio_SetDataDirection(&GPIOInst, GPIO_OUTPUT_CHANNEL, 0xF0);
	XGpio_DiscreteWrite(&GPIOInst, GPIO_OUTPUT_CHANNEL, gpio_port);


	Status = XGpio_Initialize(&GPIO1Inst, GPIO1_DEVICE_ID);
		if (Status != XST_SUCCESS)
		{
			return XST_FAILURE;
		}
		XGpio_SetDataDirection(&GPIO1Inst, GPIO1_INPUT_CHANNEL0, 0x0);
		XGpio_DiscreteWrite(&GPIO1Inst, GPIO1_INPUT_CHANNEL0, gpio_port);
		XGpio_SetDataDirection(&GPIO1Inst, GPIO1_INPUT_CHANNEL1, 0x0);
		XGpio_DiscreteWrite(&GPIO1Inst, GPIO1_INPUT_CHANNEL1, gpio_port);


	// initialize the PWM timer/counter instance but do not start it
	// do not enable PWM interrupts
	Status = PWM_Initialize(&PWMTimerInst, PWM_TIMER_DEVICE_ID, false);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize hardware to detect frequency
	Status = init_hardware_pwm_detect(hw_pwm_det_BASEADDR);
	if (Status != XST_SUCCESS){
	       return XST_FAILURE;
	}

	// initialize the interrupt controller
	Status = XIntc_Initialize(&IntrptCtlrInst,INTC_DEVICE_ID);
    if (Status != XST_SUCCESS)
    {
       return XST_FAILURE;
    }

	// connect the fixed interval timer (FIT) handler to the interrupt
    Status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
                           (XInterruptHandler)FIT_Handler,
                           (void *)0);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

 	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts, specifically real mode so that
	// the the  FIT can cause interrupts thru the interrupt controller.
    Status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

 	// enable the FIT interrupt
    XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);

	return XST_SUCCESS;
}


/*****
 * delay_msecs() - delay execution for "n" msecs
 *
 *		timing is approximate but we're not looking for precision here, just
 *		a uniform delay function.  The function uses the global "timestamp" which
 *		is incremented every msec by FIT_Handler().
 *
 * NOTE:  Assumes that this loop is running faster than the fit_interval ISR (every msec)
 *****/
void delay_msecs(u32 msecs)
{
	unsigned long target;

	if ( msecs == 0 )
	{
		return;
	}
	target = timestamp + msecs;
	while ( timestamp != target )
	{
		// spin until delay is over
	}
}


/*****
 * voltstostrng() - converts volts to a fixed format string
 *
 * accepts an Xfloat32 voltage reading and turns it into a 5 character string
 * of the following format:
 *		(+/-)x.yy
 * where (+/-) is the sign, x is the integer part of the voltage and yy is
 * the decimal part of the voltage.
 *
 * NOTE:  Assumes that s points to an array of at least 6 bytes.
 *****/
 void	voltstostrng(Xfloat32 v, char* s)
 {
	Xfloat32	dpf, ipf;
	Xuint32		dpi;
	Xuint32		ones, tenths, hundredths;

	 // form the fixed digits
	 dpf = modff(v, &ipf);
	 dpi = dpf * 100;
	 ones = abs(ipf) + '0';
	 tenths = (dpi / 10) + '0';
	 hundredths = (dpi - ((tenths - '0') * 10)) + '0';

	 // form the string and return
	 *s++ = ipf == 0 ? ' ' : (ipf > 0 ? '+' : '-');
	 *s++ = (char) ones;
	 *s++ = '.';
	 *s++ = (char) tenths;
	 *s++ = (char) hundredths;
	 *s   = 0;
	 return;
 }


/*********************************************/
/*            Interrupt Handlers             */
/*********************************************/

/*****
 * FIT_Handler() - Fixed interval timer interrupt handler
 *
 * updates the global "timestamp" every millisecond.  "timestamp" is used for the delay_msecs() function
 * and as a time stamp for data collection and reporting
 *****/
void FIT_Handler(void)
{
	static	int			ts_interval = 0;			// interval counter for incrementing timestamp

	// update timestamp
	ts_interval++;
	if (ts_interval > FIT_COUNT_1MSEC)
	{
		timestamp++;
		ts_interval = 1;
	}
}


//***************************************************
// Function to display user menu

void user_menu(char mode)
{
	static char prev_mode;
	u32 btnsw;
	int			rotcnt;
	static int old_rotcnt = 0x1000;
	XStatus status;
	char s[10];

	NX3_readBtnSw(&btnsw);

	if((btnsw)&(msk_BTN_NORTH|msk_BTN_WEST|msk_BTN_EAST))
		if((x==1) & (y==7)){
			if((btnsw)&(msk_BTN_WEST))
				prop_gain--;
			else if((btnsw)&(msk_BTN_EAST))
				prop_gain++;
			if((btnsw)&(msk_BTN_NORTH)){
				x=1;
				y=11;
			}
		}
		else if((x==1) & (y==11)){
			if((btnsw)&(msk_BTN_WEST))
				int_gain--;
			else if((btnsw)&(msk_BTN_EAST))
				int_gain++;
			if((btnsw)&(msk_BTN_NORTH)){
				x=1;
				y=15;
			}
		}
		else if((x==1) & (y==15))
		{
			if((btnsw)&(msk_BTN_WEST))
			    deri_gain--;
			else if((btnsw)&(msk_BTN_EAST))
				deri_gain++;
			if((btnsw)&(msk_BTN_NORTH)){
				x=2;
				y=15;
			}
		}
		else{
			if((btnsw)&(msk_BTN_WEST))
				offset--;
			else if((btnsw)&(msk_BTN_EAST))
				offset++;
			if((btnsw)&(msk_BTN_NORTH)){
				x=1;
				y=7;
			}
		}
		LCD_setcursor(x,y);
		ROT_readRotcnt(&rotcnt);
		if (rotcnt != old_rotcnt){
			if (abs(rotcnt - old_rotcnt) > 10){
				if(rotcnt > old_rotcnt)
					set_point += 0.1;
				else
					set_point -=0.1;
			}
			else{
				if(rotcnt > old_rotcnt)
					set_point += 0.01;
				else
					set_point -=0.01;
			}
			set_point = MAX(0.0, MIN(set_point, 3.4));
			old_rotcnt = rotcnt;
		}
		if(prop_gain != prev_prop_gain || int_gain != prev_int_gain || deri_gain != prev_deri_gain || offset != prev_offset || set_point != prev_set_point){
			switch(mode){
				case 'P':
					LCD_setcursor(1,6);
					LCD_wrstring("   ");
					LCD_setcursor(1,6);
					LCD_putnum(prop_gain,10);
					LCD_setcursor(1,10);
					LCD_wrstring("   ");
					LCD_setcursor(1,10);
					LCD_putnum(int_gain,10);
					LCD_setcursor(1,14);
					LCD_wrstring("   ");
					LCD_setcursor(1,14);
					LCD_putnum(deri_gain,10);
					LCD_setcursor(2,14);
					LCD_wrstring("   ");
					LCD_setcursor(2,14);
					LCD_putnum(offset,10);

					voltstostrng(set_point,s);
					LCD_setcursor(2,4);
					LCD_wrstring("   ");
					LCD_setcursor(2,4);
					LCD_wrstring(s);
					break;

				case 'B':
					voltstostrng(set_point,s);
					LCD_setcursor(2,4);
					LCD_wrstring("   ");
					LCD_setcursor(2,4);
					LCD_wrstring(s);
					break;

				case 'D':
					LCD_setcursor(1,14);
					LCD_wrstring("  ");
					LCD_setcursor(1,14);
					LCD_putnum(prop_gain,10);
					voltstostrng(set_point,s);
					LCD_setcursor(2,4);
					LCD_wrstring("   ");
					LCD_setcursor(2,4);
					LCD_wrstring(s);
					break;
			}
			prev_prop_gain = prop_gain;
			prev_int_gain = int_gain;
			prev_deri_gain = deri_gain;
			prev_offset = offset;
			prev_set_point = set_point;
		}
		LCD_setcursor(x,y);

}

void menu_display(char mode)
{
	if(mode != prev_mode){
		LCD_clrd();
	switch(mode){
		case 'P':
			LCD_setcursor(1,0);
			LCD_wrstring("PID -P");
			LCD_setcursor(1,9);
			LCD_wrstring("I");
			LCD_setcursor(1,13);
			LCD_wrstring("D");
			LCD_setcursor(2,0);
			LCD_wrstring("SP:");
			LCD_setcursor(2,10);
			LCD_wrstring("OFF:");
			x=1;
			y=7;
			break;
		case 'B':
			LCD_setcursor(1,0);
			LCD_wrstring("   BANG BANG!   ");
			LCD_setcursor(2,0);
			LCD_wrstring("SP:");
			break;
		case 'D':
			LCD_setcursor(1,0);
			LCD_wrstring("Proportional G  ");
			LCD_setcursor(2,0);
			LCD_wrstring("SP:");
			LCD_setcursor(2,10);
			LCD_wrstring("OFF:");
			x=1;
			y=7;
			break;
		}
	mode = prev_mode;
	}
}

