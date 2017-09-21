/**
 * @Author: Nishad Saraf & Chaitanya Deshpande
 * @University: Portland State University, Oregon.
 *
 * Description:
 * This C program is written for Digilent's Nexys 4 DDR board.
 * Program implements code to control DC motor in closed loop. PID control logic drive the motor in closed loop.
 * Equivalent graph between desired speed and actual speed is drawn on PmodOLED in real time.
 * Serial plotter can also be used to view graphs in real time on PC through UART running at 115200.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "xparameters.h"
#include "xstatus.h"
#include "nexys4IO.h"
#include "pmodENC.h"
#include "xgpio.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "PmodOLEDrgb.h"
#include "PMODHB3.h"


/************************** Constant Definitions ****************************/
// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0

// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_S00_AXI_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODHB3
#define PMODHB3_DEVICE_ID		XPAR_PMODHB3IP_0_DEVICE_ID
#define PMODHB3_BASEADDR		XPAR_PMODHB3IP_0_S00_AXI_BASEADDR
#define PMODHB3_HIGHADDR		XPAR_PMODHB3IP_0_S00_AXI_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_INPUT_0_CHANNEL		1
#define GPIO_0_OUTPUT_0_CHANNEL		2


// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/

/************************** Function Prototypes *****************************/
void usleep(u32 usecs);

void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
void FIT_Handler(void);										//
void DCControl(void);

int do_init_nx4io(u32 BaseAddress);
int do_init_pmdio(u32 BaseAddress);
int AXI_Timer_initialize(void);
int do_init();

PmodENC 	pmodENC_inst;				// PmodENC instance ref
PmodOLEDrgb	pmodOLEDrgb_inst;			// PmodOLED instance ref
XGpio		GPIOInst0;					// GPIO instance
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				// PWM timer instance

XGpio		GPIOInst0,GPIOInst1,GPIOInst2,GPIOInst3;		// GPIO instance

uint8_t 	R,G,B;

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
// "clkfit" toggles each time the FIT interrupt handler is called so its frequency will
// be 1/2 FIT_CLOCK_FREQ_HZ.  timestamp increments every 1msec and is used in delay_msecs()

u16  RotaryCnt;

/************************** MAIN PROGRAM ************************************/
int main()
{
	int sts;
	init_platform();

	sts = do_init();		// initialize the peripherals
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	microblaze_enable_interrupts();		// enable the interrupts
	DCControl();

	// blank the display digits and turn off the decimal points
	NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);

	// Clear all the display digits and the OLED display at the end of the program
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_end(&pmodOLEDrgb_inst);

	cleanup_platform();
	exit(0);
}


/**
 * Function Name: do_init()
 *
 * Return: XST_FAILURE or XST_SUCCESS
 *
 * Description: Initialize the AXI timer, gpio, interrupt, FIT timer, Encoder,
 * 				OLED display
 */
int do_init()
{
	int sts = 0;
	int status;

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (sts == XST_FAILURE)
	{
		exit(1);
	}

	// initialize the PMod544IO driver and the PmodENC and PmodCLP
	status = pmodENC_initialize(&pmodENC_inst, PMODENC_BASEADDR);
	if (sts == XST_FAILURE)
	{
		exit(1);
	}

	// initialize the PModHB3 driver
	status = PMODHB3_initialize(PMODHB3_BASEADDR);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}

	// Initialize the AXI Timer 
	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	// Initialize the OLED display 
	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	return XST_SUCCESS;
}

/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}

/************************ TEST FUNCTIONS ************************************/

void DCControl(void){
	int  RotaryIncr;
	uint32_t high_count = 0;
	bool RotaryNoNeg;
	u8 dutyRead;
	u8 stopduty = 0;
	int KP = 0;
	volatile int32_t set_rpm = 0;

	volatile int32_t actual_speed = 0;
	volatile int8_t error = 0;
	volatile int32_t scaled_rpm= 0;
	int direction = 0;
	u16 ledvalue;

	// test the rotary encoder functions
	RotaryIncr = 1;
	RotaryNoNeg = false;

	// Initialize the rotary encoder
	// clear the counter of the encoder if initialized to garbage value on power on
	pmodENC_init(&pmodENC_inst, RotaryIncr, RotaryNoNeg);
	pmodENC_clear_count(&pmodENC_inst);


	while(1)
	{
		// read the switches and write them to the LEDs and SSEGLO
		ledvalue = NX4IO_getSwitches();
		NX4IO_setLEDs(ledvalue);

		// check if the rotary encoder pushbutton or BTNC is pressed
		// exit the loop if either one is pressed.
		if ( pmodENC_is_button_pressed(&pmodENC_inst) )
		{
			break;
		}
		if (RotaryNoNeg){	//No Neg was enabled
			pmodENC_init(&pmodENC_inst, RotaryIncr, false);
			RotaryNoNeg = false;
		}else{	// No Neg was disabled
			pmodENC_init(&pmodENC_inst, RotaryIncr, true);
			RotaryNoNeg = true;		// enable the no negative mode of the rotary encoder to stop at 0
		}
		// High priority - to turn off motor with KP = 1
		if (NX4IO_isPressed(BTNC)){

			PMODHB3_setDutyCycle(stopduty);

			KP = 1;
		}
		// Up button pressed to increase KP by 1
		if (NX4IO_isPressed(BTNU)){
			KP == 255 ? KP = 255 : KP++;
			usleep(50000);
		}
		// Down button pressed to decrease KP by 1
		if (NX4IO_isPressed(BTND)){
			KP == 0 ? KP = 0 : KP--;
			usleep(50000);
		}

		// read the new value from the rotary encoder and show it on the display
		if(ledvalue == 0x0001){										// 00 - increment by 1
			pmodENC_init(&pmodENC_inst, 1, false);
		}else if(ledvalue == 0x0002){								// 01 - increment by 5
			pmodENC_init(&pmodENC_inst, 5, false);
		}else if(ledvalue == 0x0003 || ledvalue == 0x0004){			// 1x - increment by 10
			pmodENC_init(&pmodENC_inst, 10, false);
		}

		if(RotaryCnt > 255)
			RotaryCnt = 255;
		else if(RotaryCnt < 0)
			RotaryCnt = 0;

		PMODHB3_setDirection(0);
		pmodENC_read_count(&pmodENC_inst, &RotaryCnt);
		if(RotaryCnt < 256){
			PMODHB3_setDutyCycle(RotaryCnt);
			usleep(50000);
			high_count = PMODHB3_read_SA_Highcount();
			usleep(50000);
			set_rpm = RotaryCnt * 47;			// Scaled input rotary count to Max RPM approximately 11800. 11800/255 (Max RPM/Max Rotary Count) = approx. 47

			scaled_rpm = (high_count*0.0085); 	// Scaled RPM value for high count to range from 0 - 255. max high count considered is 30000. so scaled RPM = 255/30000 = 0.0085
			actual_speed=(high_count*0.32);		// converting high count to RPM
			error = ((RotaryCnt-scaled_rpm));
			//Error can be negative
			RotaryCnt = RotaryCnt + (KP*error)+10;
			PMODHB3_setDutyCycle(RotaryCnt);
			xil_printf("$%d %d %d;", KP, actual_speed ,set_rpm);
		}else if(RotaryCnt > 256 || RotaryCnt == 256){
			PMODHB3_setDutyCycle(255);
			usleep(50000);
			high_count = PMODHB3_read_SA_Highcount();
			usleep(50000);
			set_rpm = RotaryCnt * 47;			// Scaled input rotary count to Max RPM approximately 11800. 11800/255 (Max RPM/Max Rotary Count) = approx. 47

			scaled_rpm = (high_count*0.0085); 	// Scaled RPM value for high count to range from 0 - 255. max high count considered is 30000. so scaled RPM = 255/30000 = 0.0085
			actual_speed=(high_count*0.32);		// converting high count to RPM
			error = ((RotaryCnt-scaled_rpm));
			//Error can be negative
			RotaryCnt = RotaryCnt + (KP*error)+10;
			PMODHB3_setDutyCycle(RotaryCnt);
			// xil_printf("$%d %d %d;", KP, actual_speed ,set_rpm); // used to plot graphs on Serial plotter
		}
		usleep(50000);

		// Displaying KP and RPM on 7-segment display
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT7,(int) (KP/10)%10);
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT6,(int) (KP)%10);
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT4,(int) (actual_speed/10000)%10);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT3,(int) (actual_speed/1000)%10);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT2,(int) (actual_speed/100) %10 ) ;
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT1,(int) (actual_speed%100)/10);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT0,(int)  actual_speed%10);

		// calculate the desired speed and actual speed to draw graph on OLED
		int i, difference_Desired_speed, difference_Actual_speed;
		int previous_Desired_speed, previous_Actual_speed, new_count_Desired_Speed, new_count_Actual_Speed;

		if(previous_Actual_speed != actual_speed){
			difference_Actual_speed = actual_speed - previous_Actual_speed;
			new_count_Actual_Speed = new_count_Actual_Speed + difference_Actual_speed;
		}
		if(previous_Desired_speed != set_rpm){
			difference_Desired_speed = set_rpm - previous_Desired_speed;
			new_count_Desired_Speed = new_count_Desired_Speed + difference_Desired_speed;
		}
		i++;
		// If I get to the edge of the screen start from zero again
		if(i >= 90){
			OLEDrgb_Clear(&pmodOLEDrgb_inst);
			i = 0;
		}
		// appropriately scale the value to fit inside the OLED.
		OLEDrgb_DrawPixel(&pmodOLEDrgb_inst, i, new_count_Actual_Speed/800, OLEDrgb_BuildRGB(255,0,0));
		OLEDrgb_DrawPixel(&pmodOLEDrgb_inst, i, new_count_Desired_Speed/800, OLEDrgb_BuildRGB(255,255,0));
		OLEDrgb_DrawPixel(&pmodOLEDrgb_inst, i, 63-KP, OLEDrgb_BuildRGB(255,255,255));
		OLEDrgb_DrawPixel(&pmodOLEDrgb_inst, i, 31-(error/10), OLEDrgb_BuildRGB(0,0,255));

		previous_Desired_speed = set_rpm;
		previous_Actual_speed = actual_speed;

	} // rotary button has been pressed - exit the loop
	return;
}

/*********************** HELPER FUNCTIONS ***********************************/

/****************************************************************************/
/**
 * insert delay (in microseconds) between instructions.
 *
 * This function should be in libc but it seems to be missing.  This emulation implements
 * a delay loop with (really) approximate timing; not perfect but it gets the job done.
 *
 * @param	usec is the requested delay in microseconds
 *
 * @return	*NONE*
 *
 * @note
 * This emulation assumes that the microblaze is running @ 100MHz and takes 15 clocks
 * per iteration - this is probably totally bogus but it's a start.
 *
 *****************************************************************************/

static const u32	DELAY_1US_CONSTANT	= 15;	// constant for 1 microsecond delay

void usleep(u32 usec)
{
	volatile u32 i, j;

	for (i = 0; i < usec; i++)
	{
		for (j = 0; j < DELAY_1US_CONSTANT; j++);
	}
	return;
}


/****************************************************************************/
/**
 * initialize the Nexys4 LEDs and seven segment display digits
 *
 * Initializes the NX4IO driver, turns off all of the LEDs and blanks the seven segment display
 *
 * @param	BaseAddress is the memory mapped address of the start of the Nexys4 registers
 *
 * @return	XST_SUCCESS if initialization succeeds.  XST_FAILURE otherwise
 *
 * @note
 * The NX4IO_initialize() function calls the NX4IO self-test.  This could
 * cause the program to hang if the hardware was not configured properly
 *
 *****************************************************************************/
int do_init_nx4io(u32 BaseAddress)
{
	int sts;

	// initialize the NX4IO driver
	sts = NX4IO_initialize(BaseAddress);
	if (sts == XST_FAILURE)
		return XST_FAILURE;

	// turn all of the LEDs off using the "raw" set functions
	// functions should mask out the unused bits..something to check w/
	// the debugger when we bring the drivers up for the first time
	NX4IO_setLEDs(0xFFF0000);
	NX4IO_RGBLED_setRGB_DATA(RGB1, 0xFF000000);
	NX4IO_RGBLED_setRGB_DATA(RGB2, 0xFF000000);
	NX4IO_RGBLED_setRGB_CNTRL(RGB1, 0xFFFFFFF0);
	NX4IO_RGBLED_setRGB_CNTRL(RGB2, 0xFFFFFFFC);

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	return XST_SUCCESS;

}


/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
 * Converts an integer to ASCII characters
 *
 * algorithm borrowed from ReactOS system libraries
 *
 * Converts an integer to ASCII in the specified base.  Assumes string[] is
 * long enough to hold the result plus the terminating null
 *
 * @param 	value is the integer to convert
 * @param 	*string is a pointer to a buffer large enough to hold the converted number plus
 *  			the terminating null
 * @param	radix is the base to use in conversion,
 *
 * @return  *NONE*
 *
 * @note
 * No size check is done on the return string size.  Make sure you leave room
 * for the full string plus the terminating null in string
 *****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

	while (v || tp == tmp)
	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

	return;
}


/****************************************************************************/
/**
 * Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
 *
 * Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
 * cursor position.
 *
 * @param num is the number to display as a hex value
 *
 * @return  *NONE*
 *
 * @note
 * No size checking is done to make sure the string will fit into a single line,
 * or the entire display, for that matter.  Watch your string sizes.
 *****************************************************************************/
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
	char  buf[9];
	int32_t   cnt;
	char  *ptr;
	int32_t  digit;

	ptr = buf;
	for (cnt = 7; cnt >= 0; cnt--) {
		digit = (num >> (cnt * 4)) & 0xF;

		if (digit <= 9)
		{
			*ptr++ = (char) ('0' + digit);
		}
		else
		{
			*ptr++ = (char) ('a' - 10 + digit);
		}
	}

	*ptr = (char) 0;
	OLEDrgb_PutString(InstancePtr,buf);

	return;
}


/****************************************************************************/
/**
 * Write a 32-bit number in Radix "radix" to LCD display
 *
 * Writes a 32-bit number to the LCD display starting at the current
 * cursor position. "radix" is the base to output the number in.
 *
 * @param num is the number to display
 *
 * @param radix is the radix to display number in
 *
 * @return *NONE*
 *
 * @note
 * No size checking is done to make sure the string will fit into a single line,
 * or the entire display, for that matter.  Watch your string sizes.
 *****************************************************************************/
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
	char  buf[16];

	PMDIO_itoa(num, buf, radix);
	OLEDrgb_PutString(InstancePtr,buf);

	return;
}
