/*
Program:		ARES Embedded.c
Authors:		Stuart Pollmann & Patricia Munn

Use Case:		Flashed to ATMega 2560 (Arduino Mega) on-board the rover.
				Code receives 5 byte messages serially from the on-board Jetson
				Orin Nano before sending those on as PWM signals to command the
				motors and linear actuators.  Code also prints out (Txs) what
				commands are being sent so that the user can monitor the system
				from the UI laptop.

				This C code was used in final version of the rover, worked correctly.
				Commented out is the right joystick code that would have controlled
				the camera gimble, but that code never tested or used bc the team
				never got the camera gimble mounted.  Also commented out is the
				setDIR_serial() function, which works and can be used to send commands
				to the rover using keyboard buttons rather than the controller.


Functionality:	Program is interrupt driven.  When Arduino receives a byte from
				the Jetson, the ISR(USART0_RX_vect) is triggered, which stores
				the byte in the heard_msg.data array.  Once all 5 bytes of the 
				message are received and stored in the array, the MSG_handler()
				function has a switch case that looks at the message type (the 
				first byte received, which is heard_msg.data[0] ) and processes
				the message based on what type it is.
				The message type can be either '1' for buttons or '2' for left
				joystick movement.  If the message type is neither of these 
				values, the default case is used, which throws away the received
				message and prints 'dflt' to the terminal.

				There also exists a watchdog timer that goes off every ~6 s and
				kill all rover movement.  This mitigates the risk of a runaway
				situation (where the rover continually runs the last command
				received), which can occur if the data sync is thrown off or
				if the rover loses network connection.


Room for		Currently, the code is robust with the robot acting on all gamepad commands with very little latency.  Rarely do 'dflt' cases occur and
Improvements:	they do not throw off the communication sync between the Jetson & Arduino.
				Though making code changes on the C or Python side seem to
				cause more and more 'dflt' cases to occur and throw off the communication
				synchronization between the Jetson & Arduino.

				Root cause of this: the Jetson is sending bytes faster than the
				Arduino is able to store and take them out of the Rx buffer.  The 
				Arduino's Rx buffer can hold 2 bytes before it is full.  Once full,
				the next byte will be thrown out and lost (a data overrun condition).
				A byte being lost throws off the data synchronization between the 
				Jetson & Arduino: for all subsequent messages, the message type
				won't be correctly stored at heard_msg.data[0], causing the 'dflt'
				case to occur.  To observe if a data overrun condition happens,
				print out DOR0 (Data Overrun), Bit 3 of the UCSR0A register, 
				after a UART ISR trigger.

				Proposed fixes: maybe a longer time.sleep() delay being added
				between byte sends in Python could mitigate this.  A smarter 
				and more robust fix would be to add error checking and correction
				in the C code.  When an error in synchronization is detected, the
				Arduino could tell the Jetson there's an error, throw away that
				data, and reset both devices to send/expect to receive the 
				message type byte next.

Datasheets
used:			https://ww1.microchip.com/downloads/en/devicedoc/atmel-2549-8-bit-avr-microcontroller-atmega640-1280-1281-2560-2561_datasheet.pdf													
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>


//-----TIMER 1 FAST PWM SETTING MACROS----------------------//
#define T1A_COM_HI		(TCCR1A	|=  (1<<COM1A1)	| (1<<COM1B1)| (1<<COM1C1));
#define T1A_COM_LO		(TCCR1A	&= ~(1<<COM1A0)	&~(1<<COM1B0)&~(1<<COM1C0));								   
#define T1A_WGM_HI		(TCCR1A	|=	(1<<WGM11));
#define T1A_WGM_LO		(TCCR1A	&= ~(1<<WGM10));
#define T1B_WGM_HI		(TCCR1B	|=	(1<<WGM12) |  (1<<WGM13));
#define T1A_CS__LO		(TCCR1B	&= ~(1<< CS12)	&~(1<<CS10));
#define T1B_CS__HI		(TCCR1B	|=	(1<< CS11));

#define FPWM_1A_OUT		(DDRB	|=	(1<<PINB5));
#define FPWM_1B_OUT		(DDRB	|=	(1<<PINB6));
#define FPWM_1C_OUT		(DDRB	|=	(1<<PINB7));


//-----TIMER 3 FAST PWM SETTING MACROS----------------------//
#define T3A_COM_HI		(TCCR3A	|=  (1<<COM3A1)	| (1<<COM3B1)| (1<<COM3C1));
#define T3A_COM_LO		(TCCR3A	&= ~(1<<COM3A0)	&~(1<<COM3B0)&~(1<<COM3C0));
#define T3A_WGM_HI		(TCCR3A	|=	(1<<WGM31));
#define T3A_WGM_LO		(TCCR3A	&= ~(1<<WGM30));
#define T3B_WGM_HI		(TCCR3B	|=	(1<<WGM32) |  (1<<WGM33));
#define T3A_CS__LO		(TCCR3B	&= ~(1<< CS32)	&~(1<<CS30));
#define T3B_CS__HI		(TCCR3B	|=	(1<< CS31));

#define FPWM_4A_OUT		(DDRE	|=	(1<<PINE3));
#define FPWM_4B_OUT		(DDRE	|=	(1<<PINE4));
#define FPWM_4C_OUT		(DDRE	|=	(1<<PINE5));


//-----TIMER 4 FAST PWM SETTING MACROS----------------------//
#define T4A_COM_HI		(TCCR4A	|=  (1<<COM4A1)	| (1<<COM4B1)| (1<<COM4C1));
#define T4A_COM_LO		(TCCR4A	&= ~(1<<COM4A0)	&~(1<<COM4B0)&~(1<<COM4C0));
#define T4A_WGM_HI		(TCCR4A	|=	(1<<WGM41));
#define T4A_WGM_LO		(TCCR4A	&= ~(1<<WGM40));
#define T4B_WGM_HI		(TCCR4B	|=	(1<<WGM42) |  (1<<WGM43));
#define T4A_CS__LO		(TCCR4B	&= ~(1<< CS42)	&~(1<< CS40));
#define T4B_CS__HI		(TCCR4B	|=	(1<< CS41));

#define FPWM_2A_OUT		(DDRH	|=	(1<<PINH3));
#define FPWM_2B_OUT		(DDRH	|=	(1<<PINH4));
#define FPWM_2C_OUT		(DDRH	|=	(1<<PINB5));


//-----TIMER 5 FAST PWM SETTING MACROS----------------------//
#define T5A_COM_HI		(TCCR5A	|=  (1<<COM5A1)	| (1<<COM5B1)| (1<<COM5C1));
#define T5A_COM_LO		(TCCR5A	&= ~(1<<COM5A0)	&~(1<<COM5B0)&~(1<<COM5C0));
#define T5A_WGM_HI		(TCCR5A	|=	(1<<WGM51));
#define T5A_WGM_LO		(TCCR5A	&= ~(1<<WGM50));
#define T5B_WGM_HI		(TCCR5B	|=	(1<<WGM52) |  (1<<WGM53));
#define T5A_CS__LO		(TCCR5B	&= ~(1<< CS52)	&~(1<< CS50));
#define T5B_CS__HI		(TCCR5B	|=	(1<< CS51));

#define FPWM_3A_OUT		(DDRL	|=	(1<<PINL3));
#define FPWM_3B_OUT		(DDRL	|=	(1<<PINL4));
#define FPWM_3C_OUT		(DDRL	|=	(1<<PINL5));


//-----SABERTOOTH CONTROLLER MACROS-------------------------//
#define FWD_DUTY16	 3999
#define STOP_DUTY16	 2999
#define RVRS_DUTY16  1999
#define HALF_FWD_DUTY16		3499
#define HALF_RVRS_DUTY16	2499

#define TOP_40HZ	39999

#define DRIVE_L		OCR1A	// Arduino pin 11, PB5
#define DRIVE_R		OCR1B	// pin 12, PB6
// #define SERVO_L		OCR1C	// pin 13, PB7
// #define SERVO_R		OCR3A	// pin 5, PE3
#define DEPO_TILT	OCR5B	// pin 45, PL4
#define EXC_TILT	OCR4A	// pin 6, PH3
#define EXC_CHAIN	OCR4B	// pin 7, PH4
#define EXC_HEIGHT	OCR4C	// pin 8, PH5

volatile	uint8_t DIRencoder = 0X00;
volatile	char	key		   =    0;


//-----WATCHDOG OVERRIDE MACROS---------------------------//
#define		WATCH_DOG_EN		(TIMSK5|=(1<<TOIE5));		// EN timer5 overflow interrupt
volatile	uint8_t		WatchToken = 0;


//-----MSG HANDLING MACROS--------------------------------//
#define MAX_MSG_LENGTH					5					// max # of words in a msg
volatile uint8_t buttons[8]		=	{0};					// button bit field

typedef struct message						// struct holding variables related to msgs
{
		volatile uint8_t werd_count;		
		volatile uint8_t data[MAX_MSG_LENGTH];						//array to store all the words in a message
} message;

volatile struct message heard_msg;		// creates a message struct instance


//-----FUNCTION PROTOTYPES--------------------------------//
void serial_transmit(uint8_t data);
void term_Send_Val_as_Digits(uint8_t val);
void printBin8(uint8_t stuff);
void stop_motors();
void stop_linear_actuators();
void signal_linear_actuators();
void MSG_handler();



//-----INIT FUNCTIONS----------------------------------------//

void uart_init (void)						//initialize UART
{
	UBRR0L = 1;								//BAUD 500000
	
	UCSR0B	|=	(1<<	TXEN0)
			|	(1<<	RXEN0)					
			|	(1<<	RXCIE0);			//EN: Interrupt
											//EN: receiver/transmitter
	
	UCSR0C	|=	(1<<	UCSZ00)
			|	(1<<	UCSZ01);			//8-bit char size
}


void gpio_init()							//in's and out's
{
	SCK_SIG_OUT
	HX711_DAT_IN
	
	NORTH_SENS_IN
	SOUTH_SENS_IN
	
	FPWM_1A_OUT
	FPWM_1B_OUT
	FPWM_1C_OUT
	
	FPWM_2A_OUT
	FPWM_2B_OUT
	FPWM_2C_OUT
	
	FPWM_3A_OUT
	FPWM_3B_OUT
	FPWM_3C_OUT
	
	FPWM_4A_OUT
	FPWM_4B_OUT
	FPWM_4C_OUT
}


void timer1_init()						// For Drivetrain & Servo Left
{
	T1A_WGM_HI								//FPWM Mode
	T1A_WGM_LO
	T1B_WGM_HI
	T1A_CS__LO								//P.S. 8
	T1B_CS__HI	
	T1A_COM_HI								//Match:Clear
	T1A_COM_LO								//Bottom:Set
	
	ICR1	=	TOP_40HZ;					//40Hz 1.5ms Pulse
	
	OCR1A	=	STOP_DUTY16;				// DRIVE_L
	OCR1B	=	STOP_DUTY16;				// DRIVE_R	
	OCR1C	=	STOP_DUTY16;				// SERVO_L
}


void timer3_init()						// For Servo Right
{
	T3A_WGM_HI								//FPWM Mode
	T3A_WGM_LO
	T3B_WGM_HI
	T3A_CS__LO								//P.S. 8
	T3B_CS__HI
	T3A_COM_HI								//Match:Clear
	T3A_COM_LO								//Bottom:Set

	ICR3	=	TOP_40HZ;					//40Hz 1.5ms Pulse
	
	OCR3A	=	STOP_DUTY16;				// SERVO_R
	OCR3B	=	STOP_DUTY16;				//Available
	OCR3C	=	STOP_DUTY16;				//Available
}


void timer4_init()						// For Excavation
{
	T4A_WGM_HI								//FPWM Mode
	T4A_WGM_LO
	T4B_WGM_HI
	T4A_CS__LO								//P.S. 8
	T4B_CS__HI
	T4A_COM_HI								//Match:Clear
	T4A_COM_LO								//Bottom:Set

	ICR4	=	TOP_40HZ;					//40Hz 1.5ms Pulse
	
	OCR4A	=	STOP_DUTY16;				// EXC_TILT
	OCR4B	=	STOP_DUTY16;				// EXC_CHAIN
	OCR4C	=	STOP_DUTY16;				// EXC_HEIGHT
}


void timer5_init()						// For Watchdog & Depo
{
	T5A_WGM_HI								//FPWM Mode
	T5A_WGM_LO
	T5B_WGM_HI
	T5A_CS__LO								//P.S. 8
	T5B_CS__HI
	T5A_COM_HI								//Match:Clear
	T5A_COM_LO								//Bottom:Set

	WATCH_DOG_EN							//Watchdog on

	ICR5	=	TOP_40HZ;					//40Hz 1.5ms Pulse
	
	OCR5A	=	STOP_DUTY16;				//Available
	OCR5B	=	STOP_DUTY16;				// DEPO_TILT
	OCR5C	=	STOP_DUTY16;				//Available
}



//-----UART TX FUNCTIONS-----------------------------------------//

void serial_transmit (uint8_t data)		//Tx serial
{
	//UDR0 |= (0 << TXB80);
	while (!( UCSR0A & (1<<UDRE0)));		//w8 b4 read;
											//UDREn is read when 1
	UDR0 = data; 							//write the data in register
}


void term_Send_Val_as_Digits(uint8_t val)	//Decimal
{
	uint8_t digit = '0';					//Initialize
	
	while(val >= 100)						//incoming hundred's
	{
		digit	+= 1;						//count
		val		-= 100;						//chunks of hundred
	}
	
	serial_transmit(digit);					//Send to serial
	digit = '0';							//initialize
	
	while(val >= 10)						//Remaining ten's
	{
		digit	+= 1;						//count
		val		-= 10;						//chunks of ten
	}
	
	serial_transmit(digit);					//Send to Serial
	serial_transmit('0' + val);				//Send remainder Value
}


void printBin8(uint8_t stuff)				//Binary Print
{
	for(uint8_t n = 0; n < 8; n++)			//Print data: MSB to LSB
	{
		uint8_t tm_ = (stuff >> (7 - n)) & (0x01);		// shift highest AND remaining

		if(tm_)
		{
			serial_transmit('1');			//Send 1
		}
		else
		{
			serial_transmit('0');			//Send 0
		}
	}
}



//-----HELPER FUNCTIONS--------------------------------------------//

void stop_motors()		// initialize all motor direction to be stopped
{
	DRIVE_L		= STOP_DUTY16;
	DRIVE_R		= STOP_DUTY16;
	// SERVO_L		= STOP_DUTY16;
	// SERVO_R		= STOP_DUTY16;
}


void stop_linear_actuators() 
{
	DEPO_TILT = STOP_DUTY16;
	EXC_TILT = STOP_DUTY16;
	EXC_CHAIN = STOP_DUTY16;
	EXC_HEIGHT = STOP_DUTY16;
}


void signal_linear_actuators()
{
	switch (heard_msg.data[1])
	{
		case 0x80:					// exc chain back  X
			EXC_CHAIN = RVRS_DUTY16;
			serial_transmit('c');
			serial_transmit('b');			
		break;
			
			
		case 0x40:					// exc height down  A
			EXC_HEIGHT = HALF_FWD_DUTY16;
			serial_transmit('e');
			serial_transmit('d');
			break;
			
			
		case 0x20:					// exc chain fwd  B
			EXC_CHAIN = FWD_DUTY16;
			serial_transmit('c');
			serial_transmit('f');
		break;
			
			
		case 0x10:					// exc height up  Y
			EXC_HEIGHT = HALF_RVRS_DUTY16;
			serial_transmit('e');
			serial_transmit('u');
		break;
		
		
		case 0x08:					// depo tilt fwd  LB
			DEPO_TILT = FWD_DUTY16;
			serial_transmit('d');
			serial_transmit('f');
		break;
		
		
		case 0x04:					// exc tilt fwd  RB
			EXC_TILT = HALF_FWD_DUTY16;
			serial_transmit('e');
			serial_transmit('f');
		break;
		
		
		case 0x02:					// depo tilt back  LT
			DEPO_TILT = RVRS_DUTY16;
			serial_transmit('d');
			serial_transmit('b');
		break;
		
		
		case 0x01:					// exc tilt back  RT
			EXC_TILT = HALF_RVRS_DUTY16;
			serial_transmit('e');
			serial_transmit('b');
		break;

		// Pressing two buttons at once:
		case 0x60:					// exc height down + exc chain fwd    A + B
			EXC_HEIGHT = HALF_FWD_DUTY16;
			EXC_CHAIN = FWD_DUTY16;
			serial_transmit('e');
			serial_transmit('d');
			serial_transmit('&');
			serial_transmit('c');
			serial_transmit('f');
		break;
		

		case 0x30:					// exc height up + exc chain fwd    Y + B
			EXC_HEIGHT = HALF_RVRS_DUTY16;
			EXC_CHAIN = FWD_DUTY16;
			serial_transmit('e');
			serial_transmit('u');
			serial_transmit('&');
			serial_transmit('c');
			serial_transmit('f');
		break;
		

		case 0xC0:					// exc height down + exc chain backward    A + X
			EXC_HEIGHT = HALF_FWD_DUTY16;
			EXC_CHAIN = RVRS_DUTY16;
			serial_transmit('e');
			serial_transmit('d');
			serial_transmit('&');
			serial_transmit('c');
			serial_transmit('b');
		break;
		

		case 0x90:					// exc height up + exc chain backward    Y + X
			EXC_HEIGHT = HALF_RVRS_DUTY16;
			EXC_CHAIN = RVRS_DUTY16;
			serial_transmit('e');
			serial_transmit('u');
			serial_transmit('&');
			serial_transmit('c');
			serial_transmit('b');
		break;
		
		
		default:
			serial_transmit('s');
			serial_transmit('t');
			serial_transmit('a');
			serial_transmit('h');
			serial_transmit('p');
			stop_linear_actuators();
		break;
	}
}



//-----CONTROLS----------------------------------------------//

//void setDIR_serial(char input)		// controls using the keyboard
//{
	//char tosend='A';						//Place holder
	//
	////-----DRIVETRAIN COMMAND FUNCTIONS-------------------//
	//
	//if(input=='a')
	//{										//Crck Scrw L
		//
		//DRIVE_L = FWD_DUTY16;
		//DRIVE_R = RVRS_DUTY16;
		//
		//DIRencoder = 0x01;					//Encrypt DIR set
		//tosend='L';							//Encrypt State Measure
		//
	//}
	//
	//else if (input=='d')
	//{										//Crck Scrw R
		//DRIVE_L = RVRS_DUTY16;
		//DRIVE_R= FWD_DUTY16;
		//
		//DIRencoder = 0X08;					//Encrypt DIR set
		//tosend='R';							//Encrypt State Measure
//
	//}
	//
	//else if (input=='w')
	//{										//Forward
		//DRIVE_L = FWD_DUTY16;
		//DRIVE_R = FWD_DUTY16;
		//
		//DIRencoder	=	0X0D;				//Encrypt DIR set
		//tosend		=	'F';				//Encrypt State Measure
		//
	//}
	//
	//else if (input=='s')					//Backward
	//{
		//
		//DRIVE_L = RVRS_DUTY16;
		//DRIVE_R = RVRS_DUTY16;
		//
		//DIRencoder = 0X0C;					//Encrypt DIR set
		//tosend='B';							//Encrypt State Measure
	//}
	//else if (input=='m')				// left joystick is in deadzone, Stop
	//{
		//DRIVE_L = STOP_DUTY16;
		//DRIVE_R = STOP_DUTY16;
	//}
	//
	////-----EXCAVATION COMMAND FUNCTIONS-----------------------//
	//
	//else if (input == '1')
	//{
		//EXC_TILT = RVRS_DUTY16;
		//tosend='D';							//down
	//}
	//
	//else if (input == '2')
	//{
		//EXC_TILT = STOP_DUTY16;
		//tosend='S';							//stop
	//}
	//
	//else if (input == '3')
	//{
		//EXC_TILT =	FWD_DUTY16;
		//tosend='U';							//up
	//}
	//
	//else if (input == '7')
	//{
		//EXC_CHAIN = FWD_DUTY16;i
		//tosend='W';							//CW
	//}
	//else if (input == '8')
	//{
		//EXC_CHAIN =	RVRS_DUTY16;
		//tosend='C';							//CCW
	//}
	//else if (input == '9')
	//{
		//EXC_CHAIN = STOP_DUTY16;
		//tosend='S';							//stop
	//}
	//
	//else if (input == 'j')
	//{
		//DEPO_TILT = FWD_DUTY16;
		//tosend='U';							//up
	//}
	//else if (input == 'k')
	//{
		//DEPO_TILT = STOP_DUTY16;
		//tosend='S';							//stop
	//}
	//else if (input == 'l')
	//{
		//DEPO_TILT = RVRS_DUTY16;
		//tosend='D';							//down
	//}
	//
	//else
	//{
		//init_DIR();
		//
		//DIRencoder = 0X00;					//Encrypt DIR set
		//tosend='S';							//Encrypt State Measure
	//}
	//
	//serial_transmit(tosend);				//Send State to serial
	//serial_transmit('\n');
	//serial_transmit('\r');
	//
//}


void MSG_handler()			// reads msg_type and takes appropriate actions
{
	serial_transmit('m');
	serial_transmit('s');
	serial_transmit('g');
	serial_transmit(':');
	serial_transmit(' ');
	
	uint8_t msg_type = heard_msg.data[0];

	switch(msg_type)				// decodes the message, based on what type of message it is
	{
		case '1':										// message was for buttons
	
			serial_transmit('B');
			serial_transmit('\t');		
			
			signal_linear_actuators();
			WatchToken = 0;
		
		break;
	
		case '2':								// message was for the left joystick
		
			serial_transmit('L');
			serial_transmit('\t');
		
			// sets duty cycles of the left & right motors
			DRIVE_L = (heard_msg.data[1] << 8) | heard_msg.data[2];
			DRIVE_R = (heard_msg.data[3] << 8) | heard_msg.data[4];

			//for (uint8_t i = 0; i < 5; i++)		// can check your data received by printing them out in binary
			//{
				//printBin8(heard_msg.data[i]);
				//serial_transmit(' ');
			//}
			WatchToken = 0;		// reset watchdog count

		break;
	
		// case '3':								// message was for the right joystick
		
		// 	serial_transmit('R');
		// 	serial_transmit('\t');
		
		// 	// sets duty cycles of the left & right servos for the gimble
		// 	SERVO_L = (heard_msg.data[1] << 8) | heard_msg.data[2];
		// 	SERVO_R = (heard_msg.data[3] << 8) | heard_msg.data[4];

		// 	//for (uint8_t i = 0; i < 5; i++)		// can check your data received by printing them out in binary
		// 	//{
		// 		//printBin8(heard_msg.data[i]);
		// 		//serial_transmit(' ');
		// 	//}
		// 	WatchToken = 0;		// reset watchdog count

		// break;
	
		default:								// if the msg_type is not a recognizable value
			
			serial_transmit('\t');
			serial_transmit('\t');
	
			serial_transmit('d');
			serial_transmit('f');
			serial_transmit('l');
			serial_transmit('t');

		break;
	}
		
	serial_transmit('\n');	// formatting
	serial_transmit('\n');
	serial_transmit('\r');
	
	heard_msg.data[0] = 0;		// reset vals to 0, just in case
	heard_msg.data[1] = 0;
	heard_msg.data[2] = 0;
	heard_msg.data[3] = 0;
	heard_msg.data[4] = 0;
}



//-----ISRs-------------------------------------------------//

ISR(TIMER5_OVF_vect)		// for watchdog
{
	cli();

	if(WatchToken == 240)					//kill operations
	{
		stop_motors();
		stop_linear_actuators();
		WatchToken = 0;
		
		serial_transmit('\n');
		
		serial_transmit('Z');				//notify Terminal
		serial_transmit('O');
		serial_transmit('I');
		serial_transmit('N');
		serial_transmit('K');
		serial_transmit('S');

		serial_transmit('\n');				//Print nxt on
		serial_transmit('\r');				//new line
	}
	
	WatchToken++;

	sei();
}


ISR(USART0_RX_vect)		// for receiving bytes from the Jetson
{
	cli();									//disable interrupts

	heard_msg.data[heard_msg.werd_count] = UDR0;
	
	serial_transmit(heard_msg.werd_count + '0');
	serial_transmit(' ');
	//printBin8(heard_msg.data[heard_msg.werd_count]);
	//serial_transmit(' ');
	serial_transmit(heard_msg.data[heard_msg.werd_count]);
	serial_transmit(' ');
	//printBin8(UCSR0A);
	
	serial_transmit('\n');
	serial_transmit('\r');
	
	heard_msg.werd_count++;
	
	if (heard_msg.werd_count >= MAX_MSG_LENGTH)		// handle msg & reset werd_count
	{
		MSG_handler(&heard_msg);
		heard_msg.werd_count = 0;
	}

	sei();
}



//-----MAIN------------------------------------------------//

int main(void)
{
	cli();

	gpio_init();
	stop_motors();
	stop_linear_actuators();
	uart_init();
	timer1_init();
	timer3_init();
	timer4_init();
	timer5_init();
	heard_msg.werd_count = 0;

	serial_transmit('i');
	serial_transmit('n');
	serial_transmit('i');
	serial_transmit('t');
	
	serial_transmit('\n');
	
    sei();
	
    while (1) 
    {
		
    }
}
