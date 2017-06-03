/*************************************************
 Sample onboard temprature sensor, and
 print the temp value on UART terminal,
 with FPU enabled.

 By Dr. Xinrong Li, xinrong@UNT.EDU, Sept. 10, 2014
 *************************************************/



#include <stdint.h> // Variable definitions for the C99 standard.
#include <stdio.h> // Input and output facilities for the C99 standard.
#include <stdbool.h> // Boolean definitions for the C99 standard.
#include <stdlib.h>

#include "inc/tm4c123gh6pm.h" // Definitions for the interrupt and register assignments.
#include "inc/hw_memmap.h" // Memory map definitions of the Tiva C Series device.
#include "inc/hw_types.h" // Definitions of common types and macros.
#include "inc/hw_gpio.h" // Defines and Macros for GPIO hardware.

#include "driverlib/sysctl.h" // Definitions and macros for System Control API of DriverLib.
#include "driverlib/interrupt.h" // Defines and macros for NVIC Controller API of DriverLib.
#include "driverlib/gpio.h" // Definitions and macros for GPIO API of DriverLib.
#include "driverlib/timer.h" // Defines and macros for Timer API of DriverLib.
#include "driverlib/pin_map.h" //Mapping of peripherals to pins for all parts.
#include "driverlib/uart.h" // Definitions and macros for UART API of DriverLib.
#include "driverlib/adc.h" // Definitions for ADC API of DriverLib.
#include "driverlib/fpu.h" // Prototypes for the FPU manipulation routines.

#include "utils/uartstdio.h" // Prototypes for the UART console functions.
							 // Needs to add "utils/uartstdio.c" through a relative link.

#define TIMER0_FREQ    2 // Freqency/2 in Hz
#define TIMER1_FREQ    200 //1 // 1/2 Hz
#define UART0_BAUDRATE    115200 // UART baudrate in bps
#define UART1_BAUDRATE    115200 // UART baudrate in bps

#define NUM_BUTTONS    2
#define LEFT_BUTTON    GPIO_PIN_4
#define RIGHT_BUTTON    GPIO_PIN_0
#define NUM_DEBOUNCE_CHECKS    10 // For 50 msec debounce time.

#define ADC0_SEQ_NUM 0 // ADC Sample Sequence Number

#define RED_LED    GPIO_PIN_1
#define BLUE_LED    GPIO_PIN_2
#define GREEN_LED    GPIO_PIN_3

#define DISP_TEXT_LINE_NUM    4
#define TEMP_STR_LEN    20

// function prototypes
void init_LEDs(void);
void init_timer(void);
void Timer0_ISR(void);
void Timer1_ISR(void);
void init_UART0(void);
void init_UART1(void);
void init_ADC(void);
void init_GPIOE(void);
void init_buttons(void);
void set_button_states(void);

extern void UARTStdioIntHandler(void);

// global variables
uint32_t sys_clock;
uint8_t cur_LED = RED_LED;
uint32_t E = 0;
uint32_t D = 0;
char temp_str[TEMP_STR_LEN];

volatile uint8_t raw_button_states[NUM_DEBOUNCE_CHECKS]; // Raw button states in circular buffer.
volatile uint32_t raw_button_states_index=0;
volatile uint8_t button_states=0;

float D_float = 0;
char temp_str[TEMP_STR_LEN];

const char *disp_text[DISP_TEXT_LINE_NUM] = {
		"\n",
		"UART Demo\n",
		"\n",
		"> " };

unsigned char buff[100];
unsigned char user_cmd;
char s[100];
uint32_t n=0;
uint32_t i=0;

int main(void)
{
	//uint32_t m=0;
	//unsigned char user_cmd;
	uint8_t saved_button_states=0, cur_button_states;

	// Configure system clock at 40 MHz.
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	sys_clock = SysCtlClockGet();

	// Enable the floating-point unit (FPU).
	FPUEnable();
	// Configure FPU to perform lazy stacking of the floating-point state.
	FPULazyStackingEnable();

	init_LEDs();
	init_ADC();
	//init_UART0();
	init_UART1();
	init_timer();
	init_GPIOE();
	init_buttons();

	// Enable the processor to respond to interrupts.
	IntMasterEnable();

	// Start the timer by enabling operation of the timer module.
	TimerEnable(TIMER1_BASE, TIMER_A);

	UARTprintf("AT\r\n"); // test
	SysCtlDelay(2.33e6);
	UARTprintf("AT+RST\r\n"); // reset
	SysCtlDelay(2.33e6);
	UARTprintf("AT+CWQAP\r\n");
	SysCtlDelay(4.66e6);

	//UARTprintf("AT+CWJAP=\"FiOS-LS70J\",\"also47nog9383dance\"\r\n"); // connect to wifi router
	UARTprintf("AT+CWJAP=\"SM-J320A 1921\",\"nxmt8142\"\r\n"); // connect to mobile hotspot

	while(1) {

		// Read user inputs from UART if available.
				/*if(UARTRxBytesAvail())
			        user_cmd = UARTgetc();
				else
					user_cmd = 0;*/

		// Read user inputs from UART if available.
		if(UARTRxBytesAvail())
		{
			// store character in buffer
			buff[i]=UARTgetc();
			// increment counter
			i++;

			if(buff[i-1]==13)
			{
				// do something with buff here...

				UARTprintf("You entered: ");
				for(n=0;n<i;n++)
				{
					UARTprintf("%c", buff[n]);
					buff[n]=0;
				}
				UARTprintf("\r\n");
				i=0;
			}

			if(i>=100)
			{
				UARTprintf("\r\nMaximum amount of characters reached\r\n");
				// erase data
				for(n=0;n<100;n++)
				{
					buff[n]=0;
				}

				// reset counter
				i=0;
			}

			if(buff[i-1]<32||buff[i-1]>126)
			{
				--i;
			}
		}
		else
		{
			buff[i]=0;
			//i++;
		}


		// Check button states.
		cur_button_states = button_states;
		if(saved_button_states != cur_button_states){
			if((~saved_button_states & LEFT_BUTTON) && (cur_button_states & LEFT_BUTTON)) {
				//UARTprintf("Left button pushed down.\n> ");

				UARTprintf("AT+CWMODE=3\r\n");
				SysCtlDelay(1e6);
				UARTprintf("AT+CIPMUX=1\r\n");
				SysCtlDelay(1e6);
				UARTprintf("AT+CIPSERVER=1,80\r\n");
				SysCtlDelay(1e6);
/*
				UARTprintf("AT\r\n"); // test
				SysCtlDelay(1e6);
				UARTprintf("AT+RST\r\n"); // reset
				SysCtlDelay(1e6);
				//UARTprintf("AT+GMR\r\n"); // check firmware version
				UARTprintf("AT+CWQAP\r\n");
				SysCtlDelay(1e6);

				UARTprintf("AT+CWJAP=\"SM-J320A 1921\",\"nxmt8142\"\r\n"); // connect to mobile hotspot
*/
			}
			if((saved_button_states & LEFT_BUTTON) && (~cur_button_states & LEFT_BUTTON)) {
				//UARTprintf("Left button released.\n> ");

				//UARTprintf("AT+CWJAP\r\n");
				//UARTprintf("AT+CWMODE=3\r\n"); //
				//UARTprintf("AT+CWJAP?\r\n"); //
				//UARTprintf("AT+RST\r\n"); // reset

				//UARTprintf("AT+CWJAP=\"FiOS-LS70J\",\"also47nog9383dance\"\r\n");

				/*UARTprintf("AT+CWJAP=\"FiOS-LS70J\",\"also47nog9383dance\"\r\n"); // connect to wifi router
				UARTprintf("AT+CIPSTART=\"TCP\",\"47.189.70.7\",80\r\n"); // connect to IP/Website
				UARTprintf("AT+CIPSEND=12\r\n"); // specify number of bytes to send (12 for hello world)
				UARTprintf("Hello World!"); // print hello world*/

				//UARTprintf("AT+CWQAP\r\n");
				//UARTprintf("AT+CWJAP=\"SM-J320A 1921\",\"nxmt8142\"\r\n"); // connect to hotspot
				//UARTprintf("AT\r\n");


			}
			if((~saved_button_states & RIGHT_BUTTON) && (cur_button_states & RIGHT_BUTTON)) {
				//UARTprintf("Right button pushed down.\n> ");

				//UARTprintf("AT+CIOBAUD?\r\n"); // get baud rate

			}
			if((saved_button_states & RIGHT_BUTTON) && (~cur_button_states & RIGHT_BUTTON)) {
				//UARTprintf("Right button released.\n> ");

				static bool t=0;

				if(t==0)
				{
					TimerEnable(TIMER0_BASE, TIMER_A);
					t=1;
				}
				else if(t==1)
				{
					TimerDisable(TIMER0_BASE, TIMER_A);
					t=0;
				}
				//UARTprintf("AT+CIPSTART=0,\"TCP\",\"192.168.43.176\",80\r\n"); // connect to IP/Website

				/*UARTprintf("AT+CIPSEND=0,12\r\n"); // specify number of bytes to send (12 for hello world)
				UARTprintf("Hello World!\r\n"); // print hello world
				UARTprintf("AT+CIPCLOSE=0\r\n"); // close channel(send data)*/


				//UARTprintf("AT+CIPMUX=1\r\n");
				//UARTprintf("AT+CIPSERVER=1,80\r\n");
				//UARTprintf("AT+CIFSR\r\n"); // get ip of esp


				//UARTprintf("AT+CWJAP?\r\n"); // return ssid
				//UARTprintf("AT+CIPSTATUS\r\n"); //
				//UARTprintf("AT+CIFSR\r\n"); // get ip of esp


				// connecting method

				//UARTprintf("AT\r\n"); // test
				//UARTprintf("AT+RST\r\n"); // reset
				//UARTprintf("AT+CWQAP\r\n"); // disconnect from Wi-Fi
				//UARTprintf("AT+GMR\r\n"); // check firmware version
				//UARTprintf("AT+CWMODE?\r\n"); // query mode
				//UARTprintf("AT+CWJAP?\r\n"); // list Wi-Fi connection
				//UARTprintf("AT+CWJAP=\"SM-J320A 1921\",\"nxmt8142\"\r\n"); // connect to hotspot
				//UARTprintf("AT+CIFSR\r\n"); // list ip and mac addresses

				//UARTprintf("AT+CWMODE=3\r\n"); // set mode
				//UARTprintf("AT+CIPMUX=1\r\n");
				//UARTprintf("AT+CWJAP=\"SM-J320A 1921\",\"nxmt8142\"\r\n");
				//UARTprintf("AT+CIPSTART=4,\"TCP\",\"192.168.4.1\",80\r\n"); // TCP
				//UARTprintf("AT+CIPSTATUS\r\n");

			}
			if(cur_button_states == (LEFT_BUTTON | RIGHT_BUTTON)) {
				//UARTprintf("Both buttons held down.\n> ");
			}

			saved_button_states = cur_button_states;
		}

	}
}


void init_LEDs(void)
{
	// Enable and configure LED peripheral.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable GPIO Port F.
	// Three onboard LEDs, R:PF1, B:PF2, G:PF3.
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}


void init_timer(void)
{
	// Enable and configure timer peripheral.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

	// Configure Timer0 as a 32-bit timer in periodic mode.
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	// Initialize timer load register.
	TimerLoadSet(TIMER0_BASE, TIMER_A, sys_clock/TIMER0_FREQ -1);

	// Configure Timer1 as a 32-bit timer in periodic mode.
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	// Initialize timer load register.
	TimerLoadSet(TIMER1_BASE, TIMER_A, sys_clock/TIMER1_FREQ -1);


	// Registers a function to be called when the interrupt occurs.
	IntRegister(INT_TIMER0A, Timer0_ISR);
	// The specified interrupt is enabled in the interrupt controller.
	IntEnable(INT_TIMER0A);
	// Enable the indicated timer interrupt source.
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Registers a function to be called when the interrupt occurs.
	IntRegister(INT_TIMER1A, Timer1_ISR);
	// The specified interrupt is enabled in the interrupt controller.
	IntEnable(INT_TIMER1A);
	// Enable the indicated timer interrupt source.
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}


void init_UART0(void)
{
	// Enable and configure UART0 for debugging printouts.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Registers a function to be called when the interrupt occurs.
	IntRegister(INT_UART0, UARTStdioIntHandler);
	UARTStdioConfig(0, UART0_BAUDRATE, sys_clock);
}

void init_UART1(void)
{
// Enable and configure UART1 for debugging printouts.
SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
GPIOPinConfigure(GPIO_PB0_U1RX);
GPIOPinConfigure(GPIO_PB1_U1TX);
GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

// Registers a function to be called when the interrupt occurs.
IntRegister(INT_UART1, UARTStdioIntHandler);
UARTStdioConfig(1, UART1_BAUDRATE, sys_clock);
}

void init_ADC(void)
{
	// Enable and configure ADC0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0);

	ADCSequenceConfigure(ADC0_BASE, ADC0_SEQ_NUM, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH7);
	ADCSequenceEnable(ADC0_BASE, ADC0_SEQ_NUM);
}

void init_GPIOE(void)
{
	// Enable and configure
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	// input for GPIOE P4
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4);

	// output for GPIOE P5
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);
	// set pin high
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
}

// Timer0 interrupt service routine
void Timer0_ISR(void)
{
	static bool toggle=0;

	// Clear the timer interrupt.
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	//TimerDisable(TIMER0_BASE, TIMER_A);

	//snprintf(temp_str, TEMP_STR_LEN, "\nCount = %.5f", count);

	if(toggle==0)
	{
		UARTprintf("AT+CIPSEND=0,14\r\n");
		toggle=1;
	}
	else
	{
	UARTprintf("\nHello World!");
	toggle=0;
	}

}

// Timer1 interrupt service routine
void Timer1_ISR(void)
{
	// Clear the timer interrupt.
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	set_button_states();
}


void init_buttons(void)
{
    uint32_t i;

	// Enable the GPIO port connected to the two onboard pushbuttons.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // PF0 is muxed with NMI, so unlock it first to configure as a GPIO input.
    // Then re-lock it to prevent further changes.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    // Set each of the button GPIO pins as an input with a pull-up.
    GPIODirModeSet(GPIO_PORTF_BASE, LEFT_BUTTON|RIGHT_BUTTON, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, LEFT_BUTTON|RIGHT_BUTTON,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Initialize global variable.
    for(i=0; i<NUM_DEBOUNCE_CHECKS; i++)
        raw_button_states[i] = 0;
}


// This function should be called regularly in a timer ISR.
void set_button_states(void)
{
	uint32_t i;
	uint8_t states = LEFT_BUTTON|RIGHT_BUTTON;

	raw_button_states[raw_button_states_index] = ~ GPIOPinRead(GPIO_PORTF_BASE, LEFT_BUTTON|RIGHT_BUTTON);

	if(raw_button_states_index >= NUM_DEBOUNCE_CHECKS-1)
		raw_button_states_index = 0;
	else
		raw_button_states_index ++;

	for(i=0; i<NUM_DEBOUNCE_CHECKS; i++)
	    states = states & raw_button_states[i];

	button_states = states;
}



//
