/*
PINB5 is Timer 1 Compare Match A PWM	-	Drivetrain Left  Motors
PINB6 is Timer 1 Compare Match B PWM	-	Drivetrain Right Motors
PINB7 is Timer 1 Compare Match C PWM	-	Deposition Tilt Lin Actuators
					
PINH3 is Timer 4 Compare Match A PWM	-	Excavation Tilt  Motors
PINH4 is Timer 4 Compare Match B PWM	-	Excavation Drive Motors
PINH5 is Timer 4 Compare Match C PWM	-	Excavation Depth Motors

PINL3 is Timer 5 Compare Match A PWM	-	Available PWM Channels	
PINL4 is Timer 5 Compare Match B PWM	-	Available PWM Channels
PINL5 is Timer 5 Compare Match C PWM	-	Available PWM Channels

PINE5 is the Proximity Sensor Due North
PINE4 is the Proximity Sensor Due South

PINL6 is the Arduino capture of the Data Out pin of the HX711 
PINL7 is the Arduino's output SCK signal to the HX711

Author:												Stuart Pollmann
Program:											ATMEGA 2560 Processor
Application:										Task manager for Arduino Mega
Use Case:											Operate ARES Lunar Construction Bot
Functionality:																				
		
			Program is Interrupt Driven, when a user sends a serial instruction,
		some mechanical motion is induced by high impedance signal from Arduino
		to Sabertooth 2x12 motor controllers. The signal generated from this
		program allows the motor controllers to commutate the speed and direction
		of the robot's DC motors.
			Optical sensor circuitry is configured in shmit trigger output. These
		sensors will signal an external interrupt request to the ATMEGA, which
		will suspend the motions of the robot drivetrain in the direction of
		a detected obstructrion, and resume the motions when the robot's sensors
		indicate the direction is clear of the obstruction via persuation of the
		pivot command.
			Program also reads values from an HX711 Programmable Gain Amplifier
		module which signals it's ready state by transitioning from a high signal
		to a low signal. Upon the falling edge of the ready signal, this program
		detects such a state, requests the data output from the module via a clock
		signal's rising generated edge from the arduino, to the hx711. A compare
		match is performed in the acquisition state of the program which stores
		each bit of the requested data. No less than 200 nano seconds shall pass
		before an attempt is made to read the sensor's data. to optimize the timing
		in-line assembly language instructions sufficiently delay the ATMEGA's 
		16MHz clock such that the reading is performed. When the data is captured
		its magnitude is checked for a maximum condition, upon which excavation
		commands are suspended, indicating the completion of the excavation process
			This program is intended to be in line with the signal path of an
		SSH connection from a user's laptop to the ARES on-board Jetson nano, which
		communicates with this intended Arduino module via USB connection between
		the COM port of the Jetson Orin Nano, and the Arduino MEGA
													
*/




#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>


//-----PROXIMITY SENSOR MACROS------------------------------//

#define NORTH_SENS_IN		(DDRE &=	  ~(1<<PINE5));
#define SOUTH_SENS_IN		(DDRE &=	  ~(1<<PINE4));

volatile uint8_t	NorthIsClear = 1;
volatile uint8_t	SouthIsClear = 1;


//-----HXX711 LOAD CELL MACROS------------------------------//

#define SCK_SIG_OUT			 (DDRL |=	  (1<<PINL6));
#define HX711_DAT_IN		 (DDRL &=	 ~(1<<PINL7));

#define HX711_DATA_LINE		  PINL7								//ORANGE
#define HX711_DATA_RDY	   (!(PINL  &	  (1<<PINL7) )     )
#define HX711_DAT_PENDING	 (PINL  &	  (1<<PINL7) )
#define HX711_DATA_VALUE   ( (PINL  &	  (1<<PINL7) ) >> 4)

#define SCK_LINE			  PINL6								//PURPLE
#define SCK_HIGH			 (PORTL |=    (1<<PINL6) )
#define SCK_LOW				 (PORTL &=   ~(1<<PINL6) )

#define GAIN128					1
#define GAIN32					2
#define GAIN64					3

volatile uint32_t	hx711_data			= 0;
volatile uint16_t	sense_read			= 0;
volatile uint8_t	tare_flag			= 0;
volatile uint16_t	known_grams			= 3470;
volatile uint16_t	known_bit			= 3720;
volatile uint16_t	grams				= 0;
volatile uint16_t	Tare_it_bruh		= 0;
volatile uint16_t	known_Delta_Ref		= 0;			//Associative bits to 516 Grams
volatile uint8_t	DepBinLoading		= 1;
volatile uint8_t	sensor_IP			= 0;

volatile uint16_t	scale_log[8]={0};
volatile uint8_t	history=0;
volatile uint32_t	sum=0;



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
#define SERVO_L		OCR1C	// pin 13, PB7
#define SERVO_R		OCR3A	// pin 5, PE3
#define DEPO_TILT	OCR5B	// pin 45, PL4
#define EXC_TILT	OCR4A	// pin 6, PH3
#define EXC_CHAIN	OCR4B	// pin 7, PH4
#define EXC_HEIGHT	OCR4C	// pin 8, PH5


volatile	uint8_t DIRencoder = 0X00;
volatile	char	key		   =    0;

//-----WATCHDOG OVERRIDE MACROS---------------------------//

#define		WATCH_DOG_EN		(TIMSK5|=(1<<TOIE5));	// EN timer5 overflow interrupt
volatile	uint8_t		WatchToken = 0;

//-----Communication Protocol-----------------------------//

#define MAX_MSG_LENGTH					5		// max # of words in a msg
#define CMD_BYTE						0

volatile uint8_t buttons[8]		=	{0};					// button bit field



//---------Function Declarations----------//
void serial_transmit (uint8_t data);
void printBin8(uint8_t stuff);		
void stop_linear_actuators();
//----------------------------------------//

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


void MSG_handler ()		
{
	//check_sum()&data[CHK_SUM]); here					//best be a good reason
			
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

			//for (uint8_t i = 0; i < 5; i++)
			//{
				//printBin8(heard_msg.data[i]);
				//serial_transmit(' ');
			//}
			WatchToken = 0;		// reset watchdog count

		break;
	
		case '3':								// message was for the right joystick
		
			serial_transmit('R');
			serial_transmit('\t');
		
			// sets duty cycles of the left & right servos for the gimble
			SERVO_L = (heard_msg.data[1] << 8) | heard_msg.data[2];
			SERVO_R = (heard_msg.data[3] << 8) | heard_msg.data[4];

			//for (uint8_t i = 0; i < 5; i++)
			//{
				//printBin8(heard_msg.data[i]);
				//serial_transmit(' ');
			//}
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

	UBRR0L = 1;								//BAUD 500000
	
	UCSR0B	|=	(1<<	RXEN0)				
			|	(1<<	TXEN0)					
			|	(1<<	RXCIE0);			//EN: Interrupt
	
	UCSR0C	|=	(1<<	UCSZ00)
			|	(1<<	UCSZ01);			//8-bit char size
			//
	//PRR0 &= ~(1<<PRUSART0);
	//PRR1 &= ~(1<<PRUSART1);
	//PRR1 &= ~(1<<PRUSART2);
	//PRR1 &= ~(1<<PRUSART3);
	
	//UCSR0A  &=  ~(1<<	DOR0)
		    //&   ~(1<<	UPE0);
	//UCSR0A	|=	(1<<	RXC0)
			//|	(1<<	U2X0);					//p219 ref
	
}

void serial_transmit (uint8_t data)	//Tx serial
{
	//UDR0 |= (0 << TXB80);
	while (!( UCSR0A & (1<<UDRE0)));		//w8 b4 read;
											//UDREn is read when 1
	UDR0 = data; 							//write the data in register
}


unsigned char uart_recieve (void)			//Rx serial
{
	
	while(!((UCSR0A) & (1<<RXC0)));			//w8t while data being received
	return UDR0;							//return 8-bit data read

}


// unsigned char uart_receive_16 (void)		// Rx serial for 16 bit values (i.e. joystick values)
// {

// 	while(!((UCSR0A) & (1<<RXC0)));			//w8t while high data bits being received
// 	uint16_t joy_data = UDR0 << 8;			//store 8-bit high data
// 	while(!((UCSR0A) & (1<<RXC0)));			//w8t while low data bits being received
// 	joy_data = joy_data | UDR0;				//store 8-bit low data


// }

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
											//from ascii: '0'
}


void term_Send_16_as_Digits(uint16_t val){
	uint8_t digit = '0';
	

	
	while(val >= 10000){
		digit += 1;
		val -= 10000;
	}
	serial_transmit(digit);
	
	digit = '0';
	
	while(val >= 1000){
		digit += 1;
		val -= 1000;
	}
	serial_transmit(digit);
	
	digit = '0';
	
	while(val >= 100){
		digit += 1;
		val -= 100;
	}
	serial_transmit(digit);
	
	digit = '0';
	
	while(val >= 10){
		digit += 1;
		val -= 10;
	}
	serial_transmit(digit);
	
	serial_transmit('0' + val);
}



void printBin8(uint8_t stuff)				//Binary Print
{
	for(uint8_t n = 0; n < 8; n++)			//Print data: MSB to LSB
	{
		uint8_t tm_ = (stuff >> (7 - n)) & (0x01);
		
											//shift highest
											//AND remaining
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

//-----END OF UART FUNCTIONS------------------------------//


void gpio_init()							//in's and out's
{
	PORTL |= (1<<7);
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




void tare_scale()
{
	tare_flag	=	1;
}

//void Prox_ISR_EN()							//EXT interrupt
//{
	//EICRB |=  (ISC50) |  (1<<ISC40);
	//EICRB &= ~(ISC51) & ~(1<<ISC41);
	//EIMSK |=  (INT5)  |	 (INT4);			//North and South
//}


void stop_motors()		// initialize all motor direction to be stopped
{
	DRIVE_L		= STOP_DUTY16;
	DRIVE_R		= STOP_DUTY16;
	SERVO_L		= STOP_DUTY16;
	SERVO_R		= STOP_DUTY16;
}


void stop_linear_actuators() 
{
	DEPO_TILT = STOP_DUTY16;
	EXC_TILT = STOP_DUTY16;
	EXC_CHAIN = STOP_DUTY16;
	EXC_HEIGHT = STOP_DUTY16;
}

//void setDIR_serial(char input)		// set motor direction based on serial data Rx'd from Jetson
//{
	//char tosend='A';						//Place holder
	//
	////-----DRIVETRAIN COMMAND FUNCTIONS------------------------------//
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
	//else if (input=='w' && NorthIsClear)
	//{										//Forward
		//DRIVE_L = FWD_DUTY16;
		//DRIVE_R = FWD_DUTY16;
		//
		//DIRencoder	=	0X0D;				//Encrypt DIR set
		//tosend		=	'F';				//Encrypt State Measure
		//
	//}
	//
	//else if (input=='s' && SouthIsClear)	//Backward
	//{
		//
		//DRIVE_L = RVRS_DUTY16;
		//DRIVE_R = RVRS_DUTY16;
		//
		//DIRencoder = 0X0C;					//Encrypt DIR set
		//tosend='B';							//Encrypt State Measure
	//}
	//else if (input=='m')		// left joystick is in deadzone, Stop
	//{
		//DRIVE_L = STOP_DUTY16;
		//DRIVE_R = STOP_DUTY16;
	//}
	//
	////-----EXCAVATION COMMAND FUNCTIONS------------------------------//
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
	//else if (input == '7' && DepBinLoading)
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

void timer1_init()							//For Drivetrain & Servo Left
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
	
	OCR4A	=	STOP_DUTY16;				// EXC_TILT
	OCR4B	=	STOP_DUTY16;				// EXC_CHAIN
	OCR4C	=	STOP_DUTY16;				// EXC_HEIGHT
	
}

void timer5_init()		// For Watchdog & Depo
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

//void ADC_init()								//Analogue Digital Conversion Set
//{
	//ADCSRA	|=	(1<<	ADEN)
			//|(1<<		ADPS0)
			//|(1<<		ADPS1)
			//|(1<<		ADPS2);			
//
	//ADMUX	&= ~(1<<	REFS1);
	//ADMUX	|=	(1<<	REFS0);				//Ref V cc setting
//
	//ADMUX	&= ~(1<<	MUX0)
			//&  ~(1<<	MUX3);
	//ADMUX	|=	(1<<	MUX1)
			//|	(1<<	MUX2);				//ADC6 single ended input for
	//ADMUX	|=	(1<<	ADLAR);				//Left justified
	//ADCSRB	&=	~(1<<	ADTS0)
			//&	~(1<<	ADTS1)
			//&	~(1<<	ADTS2);				//ADTS0:2 = 0X00
//}

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

//ISR(USART0_RX_vect, ISR_BLOCK)				//Serial In
//{
	//cli();									//disable interrupts
	//char datz='a';							//Temp Var for Capture
	//datz=UDR0;						//then read data
//
	//if (datz=='t')
	//{
		//tare_scale();
	//}
	//
	////	WatchToken = HeelDog;					//Acknowledge connection
	//
	////use_HX711();							//spend ~20us LdCell
	//
	//sei();
//}


ISR (USART0_RX_vect)
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
	
	
	
	if(sensor_IP)
	{
		serial_transmit('L');				//notify Terminal
		serial_transmit('O');
		serial_transmit('L');
		serial_transmit('\n');
		serial_transmit('\r');
		
		
		scale_log[history]=scale_log[history-1];
	}
	
	char datz='a';							//Temp Var for Capture
	datz=UDR0;						//then read data

	if (datz=='t')
	{
		tare_scale();
	}
	
	//	WatchToken = HeelDog;					//Acknowledge connection
	
	//use_HX711();							//spend ~20us LdCell
	
	sei();
}



void use_HX711()
{
	
	cli	();
	sensor_IP =1;

	for (uint8_t i = 0; i < 24; i++)		//ACQUISITION
	{
		
		SCK_HIGH;
		
		
		asm volatile(" nop");				//62.5 nano seconds
		asm volatile(" nop");
		asm volatile(" nop");
		asm volatile(" nop");				//250th nano second
		
		
		hx711_data   |= HX711_DATA_VALUE;	//3 ops to read
		hx711_data <<= 1;					//4 Scooch for nxt rd
		
		SCK_LOW;
		
		asm volatile(" nop");				//62.5 nano seconds
		asm volatile(" nop");
		asm volatile(" nop");
		asm volatile(" nop");				//250th nano second
	}
	
	
	SCK_HIGH;								//for Gain 128
	asm volatile(" nop");					//62.5 nano seconds
	asm volatile(" nop");
	asm volatile(" nop");
	asm volatile(" nop");					//250th nano second
	
	SCK_LOW;
	
	asm volatile(" nop");					//62.5 nano seconds
	asm volatile(" nop");
	asm volatile(" nop");
	asm volatile(" nop");					//250th nano second

	//sensor_IP = 0;

	hx711_data >>= 1;
	
	sei();

}




int main(void)
{
	gpio_init();
	//Prox_ISR_EN();
	stop_motors();
	stop_linear_actuators();
	
	uart_init();
	serial_transmit('i');
	serial_transmit('n');
	serial_transmit('i');
	serial_transmit('t');

	serial_transmit('\n');
	
	timer1_init();
	timer3_init();
	timer4_init();
	timer5_init();
	heard_msg.werd_count = 0;
	
    sei();
	
    while (1) 
    {
		if(HX711_DATA_RDY)				//could be expensive wrt time if in function invktn
		{
			use_HX711();				//costs ~32us/1 read
			
			
			hx711_data|=0xFF000000;
			hx711_data = ~hx711_data;
			hx711_data +=1;				//undo 2's comp
			
			hx711_data>>=8;
			sense_read|=hx711_data;
			

			if (tare_flag)
			{
				Tare_it_bruh = sense_read;
			}
			
			sense_read= sense_read-(Tare_it_bruh);
			
			//grams=(sense_read<<1);
			
			grams = ((uint32_t)sense_read*(uint32_t)known_grams)/known_bit;
			
			// subtract the oldest value,
			//  because its about to be replaced
			//   with the newest value
			sum-=scale_log[history];
			
			scale_log[history]=grams;
			
			sum += scale_log[history];
			history++;
			
			history=history%8;

			grams=sum>>3;
			//lopass avg opt1

			
			
			
			//term_Send_16_as_Digits(grams);
			
			//term_Send_16_as_Digits(sense_read);
			
			//serial_transmit('\n');				//Print nxt on
			//serial_transmit('\r');				//new line
			

			
			if (grams>50000)						//more resillience could be had here to
			{										//hold if valid weigh brings close to 50kg
				grams = 0;
				
				term_Send_16_as_Digits(grams);
				
				sense_read=0;
				
				//term_Send_16_as_Digits(sense_read);
				
				
				serial_transmit('G');				//notify Terminal
				serial_transmit('R');
				serial_transmit('A');
				serial_transmit('M');
				serial_transmit('S');
				serial_transmit('\n');				//Print nxt on
				serial_transmit('\r');				//new line
				sense_read		= 0;
			}
			else
			{
				
				//lopass avg opt2
				
				term_Send_16_as_Digits(grams);
				
				//term_Send_16_as_Digits(sense_read);
				
				serial_transmit('G');				//notify Terminal
				serial_transmit('R');
				serial_transmit('A');
				serial_transmit('M');
				serial_transmit('S');
				serial_transmit('\n');				//Print nxt on
				serial_transmit('\r');				//new line
				
				sense_read		= 0;
				
			}
			
			
			sensor_IP = 0;

			tare_flag=0;
			sense_read		= 0;
			hx711_data =0;
			
		}
    }
	
}
