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
#define PWM_FREQUENCY 55

//For servo motors
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile int resolution = 1000;
volatile static bool isFighting;
volatile static bool isReading;

uint32_t dcMotorAnalogIn[1];
volatile uint32_t dcMotorReader;
volatile uint32_t dcMotorSpeed;
//"ATTACK COMBOS"
void rightArmReady(void);
void leftArmReady(void);
void rightArmPunchGrab(void);
void oneTwo(void);
void shutDown(void);
void Timer0_Init(unsigned long period);
void Timer0_Handler(void);
volatile static bool canAttack=false;

//For URF
const double t=1.0/80.0;
volatile uint32_t pulse=0;
volatile uint8_t echo=0;
void useConsole(void);
void inInterrupt(void);
void initCapture(void);
void urf(void);
uint32_t pulseArray[5];

//For IR Emitter and Sensor
uint32_t ui32ADC0Value[1];
volatile uint32_t pe3Analog;//for PD3
void ADC0_Handler(void);
void irSensor(void);
volatile static bool isAttacked=false;

//ADC0 initializaiton
void ADC0_Init(void)
{
	
		//SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // configure the system clock to be 40MHz
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	//activate the clock of ADC0
		SysCtlDelay(2);	//insert a few cycles after enabling the peripheral to allow the clock to be fully activated.

		//initialize port E
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
	
	
		//Note: ADC0_BASE, 3 == ADC0 SS3
		ADCSequenceDisable(ADC0_BASE, 3); //disable ADC0 before the configuration is complete
		ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); // will use ADC0, SS3, processor-trigger, priority 0
		//ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_TS); //ADC0 SS3 Step 0, sample from internal temperature sensor
		//ADCSequenceStepConfigure(ADC0_BASE, 3, 1, ADC_CTL_TS); //ADC0 SS3 Step 1, sample from internal temperature sensor
		//ADCSequenceStepConfigure(ADC0_BASE, 3, 2, ADC_CTL_TS); //ADC0 SS3 Step 2, sample from internal temperature sensor
		//ADC0 SS1 Step 0, sample from internal temperature sensor, completion of this step will set RIS, last sample of the sequence
		//Only need one to signal interrupt trigger because SS3 FIFO only takes 1 sample
		
		//For Task 1
		//ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END); 
		//For Task 2
		ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END); 
	
		IntPrioritySet(INT_ADC0SS3, 0x02);  	 // configure ADC0 SS3 interrupt priority as 0
		IntEnable(INT_ADC0SS3);    				// enable interrupt 31 in NVIC (ADC0 SS1)
		ADCIntEnableEx(ADC0_BASE, ADC_INT_SS3);      // arm interrupt of ADC0 SS1
	
		ADCSequenceEnable(ADC0_BASE, 3); //enable ADC0
		
}
void hardwareConfigure(void){
	//Run CPU at 80MHz
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);//run PWM clock at 1250kHz
	
	/*
	Enable PWM1,
	ADC0,ADC1 (For analog)
	Timer (for URF)
	GPIOA(for dc motor (PA6,PA7 for PWM left track**Disabled),
	GPIOB (for gun LED and piezo buzzer)
	GPIOC(for URF),
	GPIOD(for PD0 output), 
	and GPIOF(for SW1 and SW2) (PF2,PF3 for DC motor pwm right track**Disabled)
	*/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  //SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	SysCtlDelay(3);
	
	/********************************************************************************************
			SERVO MOTOR
	********************************************************************************************/
	/*Servo PWM*/
	//Configure PD0 as PWM and output as PWM generator 0, found in schematic
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	//Configure PD1 as PWM and output as PWM generator 0, found in schematic
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);
	//Configure PA6 as PWM and output as PWM Generator 1
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinConfigure(GPIO_PA6_M1PWM2);
	//Configure PF1 as PWM and output as PWM generator 0, found in schematic
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PF1_M1PWM5);
	//Configure PF2 as PWM and output as PWM Generator 3
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
	GPIOPinConfigure(GPIO_PF2_M1PWM6);
	//Configure PF3 as PWM and output 
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
	GPIOPinConfigure(GPIO_PF3_M1PWM7);
	
	//Unlock GPIO Register and configure PUR
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	
	/********************************************************************************************
		ULTRASONIC RANGE FINDER
	********************************************************************************************/
	//configure the timer
	initCapture();
	SysCtlDelay(3);
	//Enable PC5 as output for URF *Trigger pin
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
	//Enable PC6 as input for URF *Echo pin
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2,GPIO_BOTH_EDGES);
	GPIOIntRegister(GPIO_PORTA_BASE, inInterrupt);
	IntPrioritySet(INT_GPIOA,0x01);
	
	/********************************************************************************************
		"MASTER-SLAVE"
	********************************************************************************************/
	//Configure PF0 as PWM and output 
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PF0_M1PWM4);
	
	/********************************************************************************************
		PWM Settings
	********************************************************************************************/
	//Configure module 1 PWM generator 0 as a down-counter and load count value
	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ui32Load);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);
	
	PWMGenEnable(PWM1_BASE, PWM_GEN_0); //(PD0,PD1)
	PWMGenEnable(PWM1_BASE, PWM_GEN_1); //(PA6,PA7)
	PWMGenEnable(PWM1_BASE, PWM_GEN_2); //(PF1)
	PWMGenEnable(PWM1_BASE, PWM_GEN_3);	//(PF2,PF3)
}

//ADC handler
void ADC0_Handler(void)
{
	ADCIntClear(ADC0_BASE, 3);
		if(!canAttack){
			ADCIntClear(ADC0_BASE, 3);
			ADCProcessorTrigger(ADC0_BASE, 3);
			ADCSequenceDataGet(ADC0_BASE, 3, ui32ADC0Value);
			
			pe3Analog=ui32ADC0Value[0];
			if(pe3Analog>3000){
				isAttacked=true;
				canAttack=true;
			}
		
		}
}

int main(void)
{
		volatile uint8_t ui8Adjust;
	
		hardwareConfigure();
		ADC0_Init();
		IntMasterEnable();       		// globally enable interrupt
		ADCProcessorTrigger(ADC0_BASE, 3);
		
		volatile int resolution = 500;
		resolution=1000;
		unsigned long waitBeforeAttackTime = 80000000*10; //10 seconds
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
		useConsole();
		initCapture();
	
		Timer0_Init(waitBeforeAttackTime);
		while(1)
		{
			UARTprintf("%d\n",pe3Analog);
			urf();
			if(isFighting&&!canAttack){ //Fighting stance.
				PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT , true);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 100 * ui32Load / resolution);
				leftArmReady();
				rightArmReady();
				//rightArmPunchGrab();
				//PF0 turn off to signal not fighting. --Used in 'slave'
				PWMOutputState(PWM1_BASE, PWM_OUT_4_BIT , false);
				//ADC0_Init();
			}	
			else{
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 1 * ui32Load / resolution);//PF2 - CLAW Rest
				SysCtlDelay(3000000);
			}
			if(canAttack){
				PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT , true);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 100 * ui32Load / resolution);
				if(isAttacked){
					//oneTwo();
					PWMOutputState(PWM1_BASE, PWM_OUT_4_BIT , false);
					rightArmPunchGrab();
					isAttacked=false;
					canAttack=false;
					ADCProcessorTrigger(ADC0_BASE, 3);
				}
				else{
					for(int x=0;x<3;x++){
						rightArmPunchGrab();
						SysCtlDelay(1000000);
					}
					//SysCtlDelay(1000000);
					canAttack=false;
				  ADCProcessorTrigger(ADC0_BASE, 3);
				}
			}
			else{
				PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT , false);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 100 * ui32Load / resolution);
			}
		}
}



//Servo motor arm 'attack combos'
void rightArmReady(void){
	//Right Arm Set output state
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT , true);//PF1
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT , true);//PD0
	PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT , true);//PF2 - CLAW
	//Right Arm Set pulse width
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 70 * ui32Load / resolution);//PF1 - Right Shoulder
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 70 * ui32Load / resolution);//PD0 - Right Elbow
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 80 * ui32Load / resolution);//PF2 - CLAW
}
void leftArmReady(void){
	//Left Arm Set output state
	PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT , true);//PD1
	PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT , true);//PF3
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT , true);//PA6 - Bottom Servo
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 110 * ui32Load / resolution );//PD1
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 110 * ui32Load / resolution);//PF3
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 120 * ui32Load / resolution);//PA6 //Bottom Servo
}

void rightArmPunchGrab(void){
	//Right Arm Set output state
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT , true);//PF1
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT , true);//PD0
	PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT , true);//PF2 - CLAW
	//Disable Left Arm
	PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT , false);//PF3
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT , true);//PA6 - Bottom Servo
	
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 40 * ui32Load / resolution);//PF1 - Right Shoulder
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 85 * ui32Load / resolution);//PD0 - Right Elbow
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 80 * ui32Load / resolution);//PF2 - CLAW
	SysCtlDelay(10000000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 105 * ui32Load / resolution);//PF1 - Right Shoulder
	SysCtlDelay(2000000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 20 * ui32Load / resolution);//PD0 - Right Elbow
	SysCtlDelay(1000000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 1 * ui32Load / resolution);//PF2 - CLAW
	SysCtlDelay(3000000);
}
void oneTwo(void){
	PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT , true);//PD1
	PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT , true);//PF3
	SysCtlDelay(10000000);
	rightArmPunchGrab();
	SysCtlDelay(10000000);
	rightArmPunchGrab();
	SysCtlDelay(10000000);
	PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT , true);//PD1
	PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT , true);//PF3
	//Disable Right Arm
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT , false);//PF1
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT , false);//PD0
	PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT , false);//PF2 - CLAW
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 130 * ui32Load / resolution);//PF3 Left Shoulder
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 50 * ui32Load / resolution );//PD1 Left Elbow
	SysCtlDelay(10000000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 40 * ui32Load / resolution);//PF3
	SysCtlDelay(2000000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 110 * ui32Load / resolution );//PD1
	SysCtlDelay(10000000);
}
//Atack Timer combos triggered when URF detects
void Timer0_Init(unsigned long period){//Timer for it to be 'too idle' then go to Blue Face
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE,TIMER_A,period-1);
	IntPrioritySet(INT_TIMER0A,0x00);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER0_BASE,TIMER_A);
}
void Timer0_Handler(void){
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	if(isFighting)
		canAttack=true;
	//isFighting=true;4
	UARTprintf("Can Attack? %d\n",canAttack);
	//shutDown();
}
void shutDown(void){
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 5 * ui32Load / resolution);//PF2 - CLAW
	SysCtlDelay(10000);
	//Right Arm set output state
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT , false);
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT , false);
	PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT , false);
	//Left Arm set output state
	PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT , false);
	PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT , false);
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT , false);
	
}

void useConsole(void){
	//Enable GPIO Port A for UART 0 pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//Pin muxing for UART0 on A0 and A1
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	
	//Enable UART0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//Use internal 16MHz oscillator as UART clock source
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	
	//Initialize UART for console I/O, *Parameters 2,3: Baud rate and clock rate
	UARTStdioConfig(0, 115200, 16000000);
}

void inInterrupt(void){ //URF interrupt handler
	/*
		This is the interrupt for the pulses sent out by the URF Trigger
	*/
	//Clear interrupt flag
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);
	
	//GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
	/*
		If rising edge then set timer to 0
		It's periodic so it is some random value
	*/
	if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2)==GPIO_PIN_2){
		HWREG(TIMER2_BASE + TIMER_O_TAV)=0;//load value 0 into timer
		TimerEnable(TIMER2_BASE,TIMER_A);
		echo=1;
	}
	else{
		/*
			if falling edge was detected, get value of counter
		*/
		pulse=TimerValueGet(TIMER2_BASE,TIMER_A);//get value
		TimerDisable(TIMER2_BASE,TIMER_A);
		pulse = (uint32_t)(t*pulse);
		pulse /=58;	
		if(pulse<11 && pulse>=3)
			isReading=true;
		else if(pulse>30)
			isReading=false;
		//UARTprintf("%2d\n",isReading);
		echo=0;
	}
}
void initCapture(void){
	/*
		Set a periodic timer
	*/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	SysCtlDelay(3);
	TimerConfigure(TIMER2_BASE,TIMER_CFG_PERIODIC_UP);
	TimerEnable(TIMER2_BASE,TIMER_A);
}

void urf(void){
	//check if reading pulse
	if(echo!=1){
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3,GPIO_PIN_3);
		SysCtlDelay(266);//delay for a bit
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3,~GPIO_PIN_3);
	}
	//while(echo!=0);
	
	//pulse = (uint32_t)(t*pulse);
	//pulse /=58;
	/*
		Can put motor stuff here
	*/
	//volatile bool isTrue;
	for(int x=0;x<15;x++){
			pulseArray[x]=pulse;
		}
		uint32_t newP=0;
		for(int x=0;x<5;x++){
			newP+=pulseArray[x];
		}
		pulse=newP/5;
	if(!isReading){
		isFighting=false;
		pulse=0;
		//Set to rest position
		//Right Arm set pulse width
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 50 * ui32Load / resolution);//PF1
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 40 * ui32Load / resolution);//PD0
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 5 * ui32Load / resolution);//PF2 - CLAW
		//Left arm set pulse width
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 130 * ui32Load / resolution );//PD1
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 130 * ui32Load / resolution );//PF3
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 1 * ui32Load / resolution);//PA6
		PWMOutputState(PWM1_BASE, PWM_OUT_4_BIT , true);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, 130 * ui32Load / resolution);//PF0 for 'Slave' Interrupt
		SysCtlDelay(5000000);
		//Right Arm set output state
		PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT , false);
		PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT , false);
		PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT , false);
		//Left Arm set output state
		PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT , false);
		PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT , false);
		PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT , false);
	}
	else{
		isFighting=true;
	}
	//UARTprintf("distance=%2dcm\n",pulse);
	SysCtlDelay(400000);
}
