/*
 * ARES_Embedded_v2.c
 *
 * Created: 4/2/2024 11:23:21 AM
 * Author : Patricia
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

//#include "serialTxfunctions.h"
#include "current_sensors.h"



//-----PROXIMITY SENSOR MACROS------------------------------//

#define NORTH_SENS_IN		(DDRE &=	  ~(1<<PINE5));
#define SOUTH_SENS_IN		(DDRE &=	  ~(1<<PINE4));

volatile uint8_t	NorthIsClear = 1;
volatile uint8_t	SouthIsClear = 1;

//----------------------------------------------------------


//-----HXX711 LOAD CELL MACROS------------------------------//
#define SCK_SIG_OUT			 (DDRL |=	  (1<<PINL7));
#define HX711_DAT_IN		 (DDRL &=	 ~(1<<PINL6));

#define HX711_DATA_LINE		  PINL6
#define HX711_DATA_RDY	   (!(PINL  &	  (1<<PINL6) )     )
#define HX711_DAT_PENDING	 (PINL  &	  (1<<PINL6) )
#define HX711_DATA_VALUE   ( (PINL  &	  (1<<PINL6) ) >> 4)

#define SCK_LINE			  PINL7
#define SCK_HIGH			 (PORTL |=    (1<<PINL7) )
#define SCK_LOW				 (PORTL &=   ~(1<<PINL7) )

#define GAIN128					1
#define GAIN32					2
#define GAIN64					3

volatile uint32_t	hx711_data		= 0;
volatile uint8_t	DepBinLoading	= 1;
//----------------------------------------------------------


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
//----------------------------------------------------------


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
//----------------------------------------------------------


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
//----------------------------------------------------------



//-----SABERTOOTH CONTROLLER MACROS-------------------------//

#define FWD_DUTY16	 3999
#define STOP_DUTY16	 2999
#define RVRS_DUTY16  1999

#define HALF_FWD_DUTY16		3499
#define HALF_RVRS_DUTY16	2499

#define TOP_40HZ	49999

#define DRIVE_L		OCR1A	// pin 11, PB5
#define DRIVE_R		OCR1B	// pin 12, PB6
#define DEPO_TILT	OCR5B	// pin 39, PL4
#define EXC_TILT	OCR4A	// pin 6, PH3
#define EXC_CHAIN	OCR4B	// pin 7, PH4
#define EXC_HEIGHT	OCR4C	// pin 8, PH5


volatile	uint8_t DIRencoder = 0X00;
volatile	char	key		   =    0;
//----------------------------------------------------------


//-----WATCHDOG OVERRIDE MACROS---------------------------//

#define		WATCH_DOG_EN		(TIMSK5|=(1<<TOIE5));	// EN timer5 overflow interrupt
volatile	uint8_t		WatchToken = 0;
//----------------------------------------------------------


//-----Communication Protocol-----------------------------//

#define MAX_MSG_LENGTH					5		// max # of words in a msg
#define CMD_BYTE						0

volatile uint8_t buttons[8]		=	{0};					// button bit field

//----------------------------------------------------------



//---------Prototype Function Declarations----------//

void serial_transmit (uint8_t data);
void printBin8(uint8_t stuff);
void stop_linear_actuators();

//----------------------------------------




typedef struct message						// struct holding variables related to messages (a data Tx)
{
	volatile uint8_t werd_count;
	volatile uint8_t data[MAX_MSG_LENGTH];						//array to store all the words in a message
} message;


//enum message_type												// data[0]=the message type
//{
//KILL,
//RES0,		// TODO, what is RES0 going to be used for?
//CTRL_BUTT,
//RES1,		// TODO "
//CTRL_JOY_L_STICK,
//};


volatile struct message heard_msg;		// creates a message struct instance


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
		
		// Two buttons at once:
		case 0x60:					// exc height down + exc chain fwd    A + B
		EXC_HEIGHT = HALF_FWD_DUTY16;
		EXC_CHAIN = HALF_FWD_DUTY16;
		serial_transmit('e');
		serial_transmit('d');
		serial_transmit('&');
		serial_transmit('c');
		serial_transmit('f');
		break;
		
		case 0x30:					// exc height up + exc chain fwd    Y + B
		EXC_HEIGHT = HALF_RVRS_DUTY16;
		EXC_CHAIN = HALF_FWD_DUTY16;
		serial_transmit('e');
		serial_transmit('u');
		serial_transmit('&');
		serial_transmit('c');
		serial_transmit('f');
		break;
		
		case 0xC0:					// exc height down + exc chain backward    A + X
		EXC_HEIGHT = HALF_FWD_DUTY16;
		EXC_CHAIN = HALF_RVRS_DUTY16;
		serial_transmit('e');
		serial_transmit('d');
		serial_transmit('&');
		serial_transmit('c');
		serial_transmit('b');
		break;
		
		case 0x90:					// exc height up + exc chain backward    Y + X
		EXC_HEIGHT = HALF_RVRS_DUTY16;
		EXC_CHAIN = HALF_RVRS_DUTY16;
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


void MSG_handler ()		// points to the addr of a message struct
{
	//check_sum()&data[CHK_SUM]); here					//best be a good reason
	
	/*
	The watch dog is 5 or 6 seconds, so if it takes 5 to get here- which it doesn't-
	then we'd have bigger problems. That said, it's annoying but we should clear tokens
	every message switch condition that's valid
	*/
		
	serial_transmit('m');
	serial_transmit('s');
	serial_transmit('g');
	serial_transmit(':');
	serial_transmit(' ');
	
	uint8_t msg_type = heard_msg.data[0];		// gets msg_type
	//msg_type = '2';

	switch(msg_type)				// decodes the message, based on what type of message it is
	{
	
		//case '0':											// message was a kill command
			////TODO handle_kill();									//Destroy the Child
		//break;
	
	
		case '1':										// message was for buttons
	
			serial_transmit('b');
			serial_transmit('u');
			serial_transmit('t');
			serial_transmit('t');

			serial_transmit('\t');		
			
			signal_linear_actuators();
			
			WatchToken = 0;
		
		break;
	
	
		case '2':								// message was for the left joystick
			// heard_msg.werd_count = 5;	// addr werd + 4 data werds 
	
			serial_transmit('j');
			serial_transmit('o');
			serial_transmit('y');

			serial_transmit('\t');
		
			// sets duty cycles of the left & right motors
			DRIVE_L = (heard_msg.data[1] << 8) | heard_msg.data[2];
			DRIVE_R = (heard_msg.data[3] << 8) | heard_msg.data[4];

			for (uint8_t i = 0; i < 5; i++)
			{
				printBin8(heard_msg.data[i]);
				serial_transmit(' ');
			}
	
			WatchToken = 0;		// reset watchdog count

		break;
	
	
		default:											// if the msg_type is not a recognizable value
			//didnt_hear(boo_hoo);								// message is bunk, dump it, we're doing connectionless Tx
			serial_transmit('\t');
			serial_transmit('\t');
	
			serial_transmit('d');
			serial_transmit('f');
			serial_transmit('l');
			serial_transmit('t');

		break;
		}
		
		serial_transmit('\n');
		serial_transmit('\n');
		serial_transmit('\r');
		
		heard_msg.data[0] = 0;		// reset vals to 0
		heard_msg.data[1] = 0;
		heard_msg.data[2] = 0;
		heard_msg.data[3] = 0;
		heard_msg.data[4] = 0;
	
}


//-----UART FUNCTIONS-------------------------------------//

void uart_init (void)						//initialize UART
{

	UBRR0L = 1;								// BAUD 500000
	
	UCSR0B	|=	(1<<	TXEN0)				// en transmitter
	|	(1<<	RXEN0)				// en receiver
	|	(1<<	RXCIE0);			// en interrupt
	
	UCSR0C	|=	(1<<	UCSZ00)				//
	|	(1<<	UCSZ01);			//8-bit char size

}



//-----END OF UART FUNCTIONS------------------------------//


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
	
	//FPWM_3A_OUT
	FPWM_3B_OUT
	//FPWM_3C_OUT
}

//void Prox_ISR_EN()							//EXT interrupt
//{
//EICRB |=  (ISC50) |  (1<<ISC40);
//EICRB &= ~(ISC51) & ~(1<<ISC41);
//EIMSK |=  (INT5)  |	 (INT4);			//North and South
//}

//void use_HX711()
//{
//static uint8_t sensor_IP =1;
//
//for (uint8_t i = 0; i < 24; i++)		//ACQUISITION
//{
//
//SCK_HIGH;
//
//
////asm volatile(" nop");				//62.5 nano seconds
////asm volatile(" nop");
////asm volatile(" nop");
////asm volatile(" nop");				//250th nano second
//
//
//hx711_data   |= HX711_DATA_VALUE;		//3 ops to read
//hx711_data <<= 1;						//4 Scooch for nxt rd
//
//SCK_LOW;
//
//asm volatile(" nop");				//62.5 nano seconds
//asm volatile(" nop");
//asm volatile(" nop");
//asm volatile(" nop");				//250th nano second
//}
//
//
//SCK_HIGH;								//for Gain 128
//asm volatile(" nop");				//62.5 nano seconds
//asm volatile(" nop");
//asm volatile(" nop");
//asm volatile(" nop");				//250th nano second
//
//SCK_LOW;
//
//asm volatile(" nop");				//62.5 nano seconds
//asm volatile(" nop");
//asm volatile(" nop");
//asm volatile(" nop");				//250th nano second
//
//sensor_IP = 0;
//
//
//}

void stop_motors()		// initialize all motor direction to be stopped
{
	DRIVE_L		= STOP_DUTY16;
	DRIVE_R		= STOP_DUTY16;
}


void stop_linear_actuators()
{
	DEPO_TILT = STOP_DUTY16;
	EXC_TILT = STOP_DUTY16;
	EXC_CHAIN = STOP_DUTY16;
	EXC_HEIGHT = STOP_DUTY16;
}



ISR (TIMER5_OVF_vect)
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

void timer1_init()							//For Drive n Dep
{

	T1A_WGM_HI								//FPWM Mode
	T1A_WGM_LO
	T1B_WGM_HI
	T1A_CS__LO								//P.S. 8
	T1B_CS__HI
	T1A_COM_HI								//Match:Clear
	T1A_COM_LO								//Bottom:Set
	
	
	ICR1	=	TOP_40HZ;					//40Hz 1.5ms Pulse
	
	OCR1A	=	STOP_DUTY16;				//DriveTrain Left
	OCR1B	=	STOP_DUTY16;				//DriveTrain Right
	OCR1C	=	STOP_DUTY16;				//Deposition Tilt
	
}

void timer4_init()							//For Excavation
{
	
	T4A_WGM_HI								//FPWM Mode
	T4A_WGM_LO
	T4B_WGM_HI
	T4A_CS__LO								//P.S. 8
	T4B_CS__HI
	T4A_COM_HI								//Match:Clear
	T4A_COM_LO								//Bottom:Set

	ICR4	=	TOP_40HZ;					//40Hz 1.5ms Pulse
	
	OCR4A	=	STOP_DUTY16;				//Excavation Tilt
	OCR4B	=	STOP_DUTY16;				//Excavation Drive
	OCR4C	=	STOP_DUTY16;				//Excavation Depth
	
}

void timer5_init()		// used for watchdog
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
	OCR5B	=	STOP_DUTY16;				//Available
	OCR5C	=	STOP_DUTY16;				//Available
	
}


//ISR(INT5_vect)								//for North
//{
//stop_motors();								//Halt Motion
//NorthIsClear ^= 1;						//toggle flag
//}
//
//ISR(INT4_vect)								//For South
//{
//stop_motors();								//Halt Motion
//SouthIsClear ^= 1;						//toggle flag
//}

//ISR(USART1_RX_vect)							//Serial In
//{
////rx_isrmaybe(); stuff goes here
//cli();									//disable interrupts
//uint8_t datz;							//Temp Var for Capture
//
//while ( !(UCSR0A & (1<<RXC0)) );		//w8 for buffer clear
//datz=UDR0;								//then read data
//key = datz;								//send for Global access
//
//setDIR_serial(key);						//process for motion
//
//WatchToken = HeelDog;					//Acknowledge connection
//use_HX711();							//spend ~20us LdCell
//
//sei();
//
//}

ISR (USART0_RX_vect)
{
	cli();									//disable interrupts
	
	heard_msg.data[heard_msg.werd_count] = UDR0;
	
	serial_transmit(heard_msg.werd_count + '0');
	serial_transmit(' ');
	printBin8(heard_msg.data[heard_msg.werd_count]);
	serial_transmit(' ');
	serial_transmit(heard_msg.data[heard_msg.werd_count]);
	serial_transmit(' ');
	//printBin8(UCSR0A);
	
	serial_transmit('\n');
	serial_transmit('\r');
	
	heard_msg.werd_count++;

	
	//if ((UCSR0A >> DOR0) & 1) == 1))		// if Data Overrun has occurred (RX buffer is full)
	//{
	//heard_msg.werd_count = 0;	// reset Arduino to expect msg_type next and do not use previous data received
	//serial_transmit('>');		// alert Jetson that it should send msg_type next
	//}
	
	
	if (heard_msg.werd_count >= MAX_MSG_LENGTH)		// handle msg & reset werd_count
	{
		MSG_handler(&heard_msg);
		heard_msg.werd_count = 0;
	}
	
	sei();
}

//----------------------------------------------------------



int main(void)
{
	gpio_init();
	//Prox_ISR_EN();
	stop_motors();
	stop_linear_actuators();
	
	uart_init();
	sensor_ADC_init();
	
	timer1_init();
	timer4_init();
	timer5_init();
	
	serial_transmit('i');
	serial_transmit('n');
	serial_transmit('i');
	serial_transmit('t');
	serial_transmit('\n');
	
	heard_msg.werd_count = 0;
	
	
	DDRF &= ~(1<<1); //  makes PF1 an input for ADC1 (for mega)
	ADCSRA |= (1<<ADSC);	// start first conversion
	
	sei();
	
	while (1)
	{
		
	}
	
}
