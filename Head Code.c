#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"

#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/pwm.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "driverlib/debug.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"
#include <string.h>
#include "inc/tm4c123gh6pm.h"

// FACE
//	R - PA2
// 	G - PA3
//	B - PA4

// DOWN
//	R - PB0
//	G - PB1
//	B - PB2

// UP
//	R - PA5
//	G - PD7
//	B - PD6

volatile bool fadeUp = true;
volatile bool fadeDown = true;
volatile bool fadeUpDown = true;
volatile unsigned long increment = 10;
volatile unsigned long pwmNow = 20;
volatile unsigned long pwmNow1 = 310;

volatile unsigned long upDown0 = 20;

volatile bool fadeUp0 = true;
volatile bool fadeUp1 = true;
volatile bool fadeUp2 = true;
volatile bool fadeUp3 = true;
volatile bool fadeUp4 = true;
volatile bool fadeUp5 = true;
volatile bool fadeUp6 = true;
volatile bool fadeUp7 = true;
volatile unsigned long test0 = 20;
volatile unsigned long test1 = 60;
volatile unsigned long test2 = 100;
volatile unsigned long test3 = 140;
volatile unsigned long test4 = 180;
volatile unsigned long test5 = 220;
volatile unsigned long test6 = 260;
volatile unsigned long test7 = 300;

//Boolean for interrupt from 'Master' Tiva 
volatile static bool idle=true;
volatile static bool sadFace=false;
void Timer0_Init(unsigned long period);

void PortFunctionInit(void){
		//HWREG(GPIO_PORTA_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
		//HWREG(GPIO_PORTA_BASE + GPIO_
	
		//
    // Enable Peripheral Clocks 
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //For interrupt from 'Master'

    //
    // Enable pin PA5 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);

    //
    // Enable pin PD6 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);

    //
    // Enable pin PA4 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);

    //
    // Enable pin PD7 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);

    //
    // Enable pin PA2 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);

    //
    // Enable pin PA3 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);

    //
    // Enable pin PB2 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);
		
		//
    // Enable pin PB1 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1);
		
		//
    // Enable pin PB0 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);
		
		//Enable PA7 for GPIOInput
		GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);//For interrupt from 'Master' when URF is reading
		//Enable PA6 for GPIOInput
		GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);//For when URF from 'Master' is not reading
		//Enable PUR
		HWREG(GPIO_PORTA_BASE+GPIO_O_LOCK)=GPIO_LOCK_KEY;
		HWREG(GPIO_PORTA_BASE+GPIO_O_CR)|=0x01;
		HWREG(GPIO_PORTA_BASE+GPIO_O_LOCK)=0;
}

void Happy(void){

		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2,	0);
	
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x08);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0x02);
		
}

void Sad(void){
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2,	0);
	
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0X10);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0X40);
}

void Angry(void){
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2,	0);
	
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0x04);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x20);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0x01);
}
void Off(void){
		// Turn off the Output pins
		PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
		PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
		PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
		PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
		PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
		PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, false);
		PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, false);
		PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false); 
}
void delayMS(int ms){
	SysCtlDelay((SysCtlClockGet()/(3*1000))*ms);
}

void Fade(void){
		// Turn on the Output pins
		PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
		//PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
		PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
		//PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
		PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
		//PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
		PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
		//PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
	
		delayMS(20);
		if (fadeUp) {
				pwmNow += increment;
				if (pwmNow >= 320) { fadeUp = false; }
		}
		else {
				pwmNow -= increment;
				if (pwmNow <= 10) { fadeUp = true; }
		}
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,pwmNow);
		//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,pwmNow);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,pwmNow);
		//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,pwmNow);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,pwmNow);
		//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5,pwmNow);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,pwmNow);
		//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,pwmNow);
}

void FadeIn(void){
		// Turn on the Output pins
		//PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
		PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
		//PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
		PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
		//PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
		PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
		//PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
		PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
	
		delayMS(20);
		if (fadeDown) {
				pwmNow1 += increment;
				if (pwmNow1 >= 320) { fadeDown = false; }
		}
		else {
				pwmNow1 -= increment;
				if (pwmNow1 <= 10) { fadeDown = true; }
		}
	
		//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,pwmNow);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,pwmNow1);
		//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,pwmNow);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,pwmNow1);
		//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,pwmNow);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5,pwmNow1);
		//PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,pwmNow);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,pwmNow1);
}

void Test0(void){
		// Turn on the Output pins
		PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	
		delayMS(5);
		if (fadeUp0) {
				test0 += increment;
				if (test0 >= 320) { fadeUp0 = false; }
		}
		else {
				test0 -= increment;
				if (test0 <= 10) { fadeUp0 = true; }
		}
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,test0);
		
}

void Test1(void){
		// Turn on the Output pins
		PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
	
		delayMS(5);
		if (fadeUp1) {
				test1 += increment;
				if (test1 >= 320) { fadeUp1 = false; }
		}
		else {
				test1 -= increment;
				if (test1 <= 10) { fadeUp1 = true; }
		}
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,test1);
		
}

void Test2(void){
		// Turn on the Output pins
		PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
	
		delayMS(5);
		if (fadeUp2) {
				test2 += increment;
				if (test2 >= 320) { fadeUp2 = false; }
		}
		else {
				test2 -= increment;
				if (test2 <= 10) { fadeUp2 = true; }
		}
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,test2);
		
}


void Test3(void){
		// Turn on the Output pins
		PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
	
		delayMS(5);
		if (fadeUp3) {
				test3 += increment;
				if (test3 >= 320) { fadeUp3 = false; }
		}
		else {
				test3 -= increment;
				if (test3 <= 10) { fadeUp3 = true; }
		}
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,test3);
		
}

void Test4(void){
		// Turn on the Output pins
		PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
	
		delayMS(5);
		if (fadeUp4) {
				test4 += increment;
				if (test4 >= 320) { fadeUp4 = false; }
		}
		else {
				test4 -= increment;
				if (test4 <= 10) { fadeUp4 = true; }
		}
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,test4);
		
}

void Test5(void){
		// Turn on the Output pins
		PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
	
		delayMS(5);
		if (fadeUp5) {
				test5 += increment;
				if (test5 >= 320) { fadeUp5 = false; }
		}
		else {
				test5 -= increment;
				if (test5 <= 10) { fadeUp5 = true; }
		}
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5,test5);
		
}

void Test6(void){
		// Turn on the Output pins
		PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
	
		delayMS(5);
		if (fadeUp6) {
				test6 += increment;
				if (test6 >= 320) { fadeUp6 = false; }
		}
		else {
				test6 -= increment;
				if (test6 <= 10) { fadeUp6 = true; }
		}
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,test6);
		
}

void Test7(void){
		// Turn on the Output pins
		PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
	
		delayMS(5);
		if (fadeUp7) {
				test7 += increment;
				if (test7 >= 320) { fadeUp7 = false; }
		}
		else {
				test7 -= increment;
				if (test7 <= 10) { fadeUp7 = true; }
		}
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,test7);
		
}





void Tester(void){//Put Test0-7 in this method for convenience
	Test0();
	Test1();
	Test2();
	Test3();
	Test4();
	Test5(); 
	Test6();
	Test7();
}
void interrupt_init(void){//initialize interrupt for PA7 - 'Slave'
	IntEnable(INT_GPIOA);
	IntPrioritySet(INT_GPIOA,0x00);
	GPIO_PORTA_IM_R|=0xC0;
	GPIO_PORTA_IS_R&=~0xC0;
	GPIO_PORTA_IBE_R |=0xC0;
	IntMasterEnable();
}
void MasterHandler(void){
	if(GPIO_PORTA_RIS_R&0x80){
		GPIO_PORTA_ICR_R=0x80;
		idle=false;
		sadFace=false;
	}
	else{
		GPIO_PORTA_ICR_R=0x40;
		idle=true;
	}
}
void Timer0_Init(unsigned long period){//Timer for it to be 'too idle' then go to Blue Face
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE,TIMER_A,period-1);
	IntPrioritySet(INT_TIMER0A,0x01);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER0_BASE,TIMER_A);
}
void Timer0_Handler(void){
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	if(idle)
		sadFace=true;
}
int main(void)
{
		unsigned long period = 16000000*5; //1 Second * 30 = 30 sec
		PortFunctionInit();
		//interrupt_init();
		//Timer0_Init(period);
		IntMasterEnable();
		//Set the clock
		SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

		//Configure PWM Clock to match system
		SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

		// Enable the peripherals used by this program.
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins

		//Configure PF1,PF2,PF3 Pins as PWM
		GPIOPinConfigure(GPIO_PB6_M0PWM0);
		GPIOPinConfigure(GPIO_PB7_M0PWM1);
		GPIOPinConfigure(GPIO_PB4_M0PWM2);
		GPIOPinConfigure(GPIO_PB5_M0PWM3);
		GPIOPinConfigure(GPIO_PE4_M0PWM4);
		GPIOPinConfigure(GPIO_PE5_M0PWM5);
		GPIOPinConfigure(GPIO_PC4_M0PWM6);
		GPIOPinConfigure(GPIO_PC5_M0PWM7);
		GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5);
		GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
		GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

		//Configure PWM Options
		//PWM_GEN_2 Covers M1PWM4 and M1PWM5
		//PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
		PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); 
		PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); 
		PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); 
		PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); 

		//Set the Period (expressed in clock ticks)
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 320);
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 320);
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 320);
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 320);

		//Set PWM duty-50% (Period /2)
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5,100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,100);

		// Enable the PWM generator
		PWMGenEnable(PWM0_BASE, PWM_GEN_0);
		PWMGenEnable(PWM0_BASE, PWM_GEN_1);
		PWMGenEnable(PWM0_BASE, PWM_GEN_2);
		PWMGenEnable(PWM0_BASE, PWM_GEN_3);
		Timer0_Init(period);
		while(1)
		{
			if(idle && !sadFace){
				//Off();
				Happy();
				Tester();
				interrupt_init();
			}
			else if(!idle && !sadFace){
				//Off();
				Angry();
				Fade();
				FadeIn();
			}
			//Case 3:
			if(sadFace){
				//Off();
				Sad();
				Fade();
				FadeIn();
			}
			// ********************************
			// Case1:
			// Neutral State. The face remains happy when no actions are happening and idle timer is still running.
			// Methods used:
			//		Happy(); 
			//		Test0();
			//		Test1();
			// 		Test2();
			// 		Test3();
			// 		Test4();
			// 		Test5(); 
			// 		Test6();
			// 		Test7();
			// ********************************
			
			// ********************************
			// Case2:
			// Angry State: The face turns angry after sensor is activated. The Tiva controlling the body will
			// send an interrupt to the tiva controlling the face. 
			// Methods used:
			//		Angry();
			//	  Fade();
			//		FadeIn();
			// ********************************
			
			// ********************************
			// Case3:
			// Sad State: The face turns sad after a prolonged time of no actions. The periodic timer will enable an interrupt
			// Methods used:
			//		Sad();
			//		Fade();
			//		FadeIn();
			// ********************************
			
			//** If the LEDS stay on and overlap with other methods then the Off() method should be used
			
			// ********************************
			// NOTES:
			// Everything is labeled on the LED boards and is self-explanitory
			
			// FACE
			//	R - PA2
			// 	G - PA3
			//	B - PA4

			// DOWN
			//	R - PB0
			//	G - PB1
			//	B - PB2

			// UP
			//	R - PA5
			//	G - PD7
			//	B - PD6
			
			
			// ********************************
		}

	
}
