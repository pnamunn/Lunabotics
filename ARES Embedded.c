/*
PINB5 is Timer 1 Compare Match A PWM	-	Drivetrain Left  Motors
PINB6 is Timer 1 Compare Match B PWM	-	Drivetrain Right Motors
PINB7 is Timer 1 Compare Match C PWM	-	Deposition Tilt Lin Actuators
					
PINH3 is Timer 4 Compare Match A PWM
PINH4 is Timer 4 Compare Match B PWM
PINH5 is Timer 4 Compare Match C PWM

PINL3 is Timer 0 Compare Match A PWM
PINL4 is Timer 0 Compare Match B PWM
PINL5 is Timer 0 Compare Match C PWM

PORT C Pin 1										**Used for IN1A
PORT C Pin 2										**Used for IN2A

PORT C PIN 3										**Used for IN1B
PORT D PIND 3										**used for IN2B

PD 6 OCR0A TIMER INTERRUPT
PD


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
		match is performed in the aqcuisition state of the program which stores
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

#define F_CPU 16000000
#define BAUD_9_6K 9600								// define baud
#define BAUD_115K	115200
#define BAUDRATE ((F_CPU/(16UL*BAUD_9_6K))-1)		// set baud rate

#include <avr/io.h>
#include <avr/interrupt.h>

//-----PROXIMITY SENSOR MACROS------------------------------//

#define NORTH_SENS_IN		(DDRE &=	  ~(1<<PINE5));
#define SOUTH_SENS_IN		(DDRE &=	  ~(1<<PINE4));

volatile uint8_t	NorthIsClear = 1;
volatile uint8_t	SouthIsClear = 1;

//-----PROXIMITY SENSOR MACROS------------------------------//

//-----HXX711 LOAD CELL MACROS------------------------------//
#define SCK_SIG_OUT			(DDRB |=	  (1<<PINB5));
#define HX711_DAT_IN		(DDRB &=	 ~(1<<PINB4));

#define HX711_DATA_LINE		  PINB4
#define HX711_DATA_RDY	   (!(PINB  &	  (1<<PINB4) )     )
#define HX711_DAT_PENDING	 (PINB  &	  (1<<PINB4) )
#define HX711_DATA_VALUE   ( (PINB  &	  (1<<PINB4) ) >> 4) 

#define SCK_LINE			  PINB5
#define SCK_HIGH			 (PORTB |=    (1<<PINB5) )
#define SCK_LOW				 (PORTB &=   ~(1<<PINB5) )

#define GAIN128					1
#define GAIN32					2
#define GAIN64					3

volatile uint32_t	hx711_data		= 0;
volatile uint8_t	DepBinLoading	= 1;

//-----END OF HXX711 LOAD CELL MACROS-----------------------//

//-----TIMER 1 FAST PWM SETTING MACROS----------------------//

#define	T1_TCCRA_FPWM	(TCCR1A |= 0b10101010);
							    
#define T1B_WGM_HI		(TCCR1B |=   (1<<WGM13) |  (1<<WGM12));
#define T1B_CS_LO		(TCCR1B &=   ~(1<<CS12) & ~(1<< CS10));
#define	T1B_CS_HI		(TCCR1B |=    (1<<CS11));
#define T1_CS			   T1B_CS_HI	   T1B_CS_LO 
#define T1_TCCRB_FPWM	   T1_CS		   T1B_WGM_HI
										   
#define T1_FPWM_SET		(   (T1_TCCRA_FPWM)  (T1_TCCRB_FPWM))

#define FPWM_1A_OUT		(DDRB	|=	(1<<PINB5));
#define FPWM_1B_OUT		(DDRB	|=	(1<<PINB6));
#define FPWM_1C_OUT		(DDRB	|=	(1<<PINB7));

//-----TIMER 4 FAST PWM SETTING MACROS----------------------//

#define	T4_TCCRA_FPWM	(TCCR4A|=0b10101010);
#define T4B_WGM_HI		(TCCR4B|=   (1<<WGM43) |  (1<<WGM42));
#define T4B_CS_LO		(TCCR4B&=   ~(1<<CS42) & ~(1<< CS40));
#define	T4B_CS_HI		(TCCR4B|=    (1<<CS41));
#define T4_CS			(   T4B_CS_HI	  T4B_CS_LO )
#define T4_TCCRB_FPWM	(   T4_CS		  T4B_WGM_HI)
#define T4_FPWM_SET		(   T4_TCCRA_FPWM T4_TCCRB_FPWM)

#define FPWM_2A_OUT		(DDRH	|=	(1<<PINH3));
#define FPWM_2B_OUT		(DDRH	|=	(1<<PINH4));
#define FPWM_2C_OUT		(DDRH	|=	(1<<PINB5));

//-----TIMER 5 FAST PWM SETTING MACROS----------------------//

#define	T5_TCCRA_FPWM	(TCCR5A	|=0b10101010);
#define T5B_WGM_HI		(TCCR5B	|=   (1<<WGM53) |  (1<<WGM52));
#define T5B_CS_LO		(TCCR5B	&=   ~(1<<CS52) & ~(1<< CS50));
#define	T5B_CS_HI		(TCCR5B	|=    (1<<CS51));
#define T5_CS			(   T5B_CS_HI	  T5B_CS_LO )
#define T5_TCCRB_FPWM	(   T5_CS		  T5B_WGM_HI)
#define T5_FPWM_SET		(   T5_TCCRA_FPWM T5_TCCRB_FPWM)

#define FPWM_3A_OUT		(DDRL	|=	(1<<PINL3));
#define FPWM_3B_OUT		(DDRL	|=	(1<<PINL4));
#define FPWM_3C_OUT		(DDRL	|=	(1<<PINL5));

//-----SABERTOOTH CONTROLLER MACROS-------------------------//

#define FWD_DUTY16	 4000
#define STOP_DUTY16	 3000
#define RVRS_DUTY16  2000
#define TOP_40HZ	50000

#define DRIVE_L		OCR1A
#define DRIVE_R		OCR1B

#define DEPOS_TILT	OCR1C

#define EX_ACTUATOR	OCR4A
#define EX_DRIVE	OCR4B
#define EX_DESCEND	OCR4C


volatile	uint8_t DIRencoder = 0X00;
volatile	char	key		   =    0;

//-----WATCHDOG OVERRIDE MACROS---------------------------//

#define		WATCH_DOG_EN		(TIMSK5|=(1<<TOIE5));
#define		HeelDog							  0;
volatile	uint8_t		WatchToken =		  0;

//-----UART FUNCTIONS-------------------------------------//

void uart_init (void)							//initialize UART
{

	UBRR0L = 8;
	
	UCSR0B	|=	(1<<	TXEN0)
			|	(1<<	RXEN0)					
			|	(1<<	RXCIE0);				//EN: Interrupt
												//EN: receiver/transmitter
	
	UCSR0C	|=	(1<<	UCSZ00)
			|	(1<<	UCSZ01);				//8-bit char size

}

void serial_transmit (unsigned char data)		//Tx serial
{

	while (!( UCSR0A & (1<<UDRE0)));			//w8 b4 read;
												//UDREn is read when 1
	UDR0 = data; 								//write the data in register

}

unsigned char uart_recieve (void)				//Rx serial
{
	
	while(!((UCSR0A) & (1<<RXC0)));				//w8t while data being received
	return UDR0;								//return 8-bit data read

}

void term_Send_Val_as_Digits(uint8_t val)		//Decimal
{
	uint8_t digit = '0';						//Initialize
	
	while(val >= 100)							//incoming hundred's
	{
		digit	+= 1;							//count
		val		-= 100;							//chunks of hundred
	}
	
	serial_transmit(digit);						//Send to serial

	digit = '0';								//initialize
	
	while(val >= 10)							//Remaining ten's
	{
		digit	+= 1;							//count
		val		-= 10;							//chunks of ten
	}
	
	serial_transmit(digit);						//Send to Serial
	
	serial_transmit('0' + val);					//Send remainder Value
	//from ascii: '0'
}

void printBin8(uint8_t stuff)					//Binary Print
{
	for(uint8_t n = 0; n < 8; n++)				//Print data: MSB to LSB
	{
		uint8_t tm_ = (stuff >> (7 - n)) & (0x01);
												//shift highest
												//AND remaining
		if(tm_)
		{
			serial_transmit('1');				//Send 1
		}
		else
		{
			serial_transmit('0');				//Send 0
		}
	}
}

//-----END OF UART FUNCTIONS------------------------------//

void gpio_init()								//in's and out's
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
}

void Prox_ISR_EN()								//EXT interrupt
{
	EICRB |=  (ISC50) |  (1<<ISC40);
	EICRB &= ~(ISC51) & ~(1<<ISC41);
	EIMSK |=  (INT5)  |	 (INT4);				//North and South
}

void use_HX711()
{
	static uint8_t sensor_IP =1;					//flag
	
	static enum HX711_State {STANDBY, ACTIVE, SEND} hx711_state = STANDBY;
	
	while(sensor_IP)								//sensor stt mach
	{
		
		switch(hx711_state)							//~2 ticks
		{
			
			case STANDBY:							//max busy: 76us
			while (HX711_DAT_PENDING);				//System vulnerable
			hx711_state = ACTIVE;					//62.5 nano seconds
			break;
			
			
			case ACTIVE:							//ACQUISITION
			for (uint8_t i = 0; i < 24; i++)		//7 ticks ~437.5ns
			{
				
				SCK_HIGH;
				
				__asm__ __volatile__("nop");		//stall
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");		//250th nano sec
				
				
				hx711_data   = HX711_DATA_VALUE;	//187.5 nano sec
				hx711_data <<= 1;					//250th nano sec
				
				SCK_LOW;
				
				__asm__ __volatile__("nop");		//stall
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");		//250th nano second
			}										//Exit: 18 micro
			
			for (uint8_t i = 0; i < GAIN128; i++)	//7 ticks ~437.5ns
			{
				
				SCK_HIGH;
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");		//250th nano se
				
				SCK_LOW;
				
				__asm__ __volatile__("nop");		//62.5 nano sec
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");		//250th nano second
				
			}										//18.875  micro
			hx711_state = SEND;						//1 tick
			break;									//18.9375 micro
			
			
			case SEND:								//print longest
			 
			 if(hx711_data&0xFFFFFF)
			 DepBinLoading	=	0;					//clear flag
			 
			 else
			 DepBinLoading	=	1;					//Not Full
			
			term_Send_Val_as_Digits((uint8_t)(hx711_data>>24));
			term_Send_Val_as_Digits((uint8_t)(hx711_data>>16));
			term_Send_Val_as_Digits((uint8_t)(hx711_data>> 8));
			term_Send_Val_as_Digits((uint8_t)(hx711_data>> 0));
			serial_transmit('\n');
			serial_transmit('\r');
			
			hx711_state=STANDBY;
			sensor_IP = 0;
			break;
		}
	}

}

void init_DIR()
{
	DRIVE_L		= STOP_DUTY16;
	DRIVE_R		= STOP_DUTY16;
	EX_ACTUATOR	= STOP_DUTY16;
	EX_DRIVE	= STOP_DUTY16;
	EX_DESCEND	= STOP_DUTY16;
}

void setDIR_serial(char input)					//And speed atm
{
	char tosend='A';							//Place holder
	
//-----DRIVETRAIN COMMAND FUNCTIONS------------------------------//
	
	if(input=='a')
	{											//Crck Scrw L
		
		DRIVE_L = FWD_DUTY16;
		DRIVE_R = RVRS_DUTY16;
		
		DIRencoder = 0x01;						//Encrypt DIR set
		tosend='L';								//Encrypt State Measure
		
	}
	
	else if (input=='d')
	{											//Crck Scrw R
		DRIVE_L = RVRS_DUTY16;
		DRIVE_R= FWD_DUTY16;
		
		DIRencoder = 0X08;						//Encrypt DIR set
		tosend='R';								//Encrypt State Measure

	}
	
	else if (input=='w' && NorthIsClear)
	{											//Forward
		DRIVE_L = FWD_DUTY16;
		DRIVE_R = FWD_DUTY16;
		
		DIRencoder	=	0X0D;					//Encrypt DIR set
		tosend		=	'F';					//Encrypt State Measure
		
	}
	
	else if (input=='s' && SouthIsClear)
	{
		
		DRIVE_L = RVRS_DUTY16;
		DRIVE_R = RVRS_DUTY16;
		
		DIRencoder = 0X0C;						//Encrypt DIR set
		tosend='B';								//Encrypt State Measure
	}
	
//-----EXCAVATION COMMAND FUNCTIONS------------------------------//	
	
		else if (input == '1')
		{
			EX_ACTUATOR = RVRS_DUTY16;
			tosend='D';						//down
		}
		
		else if (input == '2')
		{
			EX_ACTUATOR = STOP_DUTY16;
			tosend='S';						//stop
		}
		
		else if (input == '3')
		{
			EX_ACTUATOR =	FWD_DUTY16;
			tosend='U';						//up
		}
		
			else if (input == '7' && DepBinLoading)
			{
				EX_DRIVE = FWD_DUTY16;
				tosend='W';					//CW
			}
			else if (input == '8')
			{
				EX_DRIVE =	RVRS_DUTY16;
				tosend='C';					//CCW
			}
			else if (input == '9')
			{
				EX_DRIVE = STOP_DUTY16;
				tosend='S';					//stop
			}
			
				else if (input == 'j')
				{
					DEPOS_TILT = FWD_DUTY16;
					tosend='U';				//up
				}
				else if (input == 'k')
				{
					DEPOS_TILT = STOP_DUTY16;
					tosend='S';				//stop
				}
				else if (input == 'l')
				{
					DEPOS_TILT = RVRS_DUTY16;
					tosend='D';				//down
				}
	
	else
	{
		init_DIR();
		
		DIRencoder = 0X00;						//Encrypt DIR set
		tosend='S';								//Encrypt State Measure
	}
	
	serial_transmit(tosend);				//Send State to serial
	serial_transmit('\n');
	serial_transmit('\r');
	
}

void BewareOfDog()							//watchdog
{
	if(WatchToken == 240)					//kill operations
	{
		init_DIR();
		
		WatchToken	=		HeelDog
		
		serial_transmit('Z');
		serial_transmit('O');
		serial_transmit('I');
		serial_transmit('N');
		serial_transmit('K');
		serial_transmit('S');

		serial_transmit('\n');
		serial_transmit('\r');
	}
	
	WatchToken++;
}

void timer1_init()
{
	T1_FPWM_SET									//Fast PWM
												//P.S. 8
												//compare match
												//A:C non-invrt

	ICR1	=	TOP_40HZ;
	
	OCR1A	=	STOP_DUTY16;
	OCR1B	=	STOP_DUTY16;						
	OCR1C	=	STOP_DUTY16;
	
}

void timer4_init()
{
	T4_FPWM_SET									//Fast PWM
												//P.S. 8
												//compare match
												//A:C non-invrt

	ICR4	=	TOP_40HZ;	
	OCR4A	=	STOP_DUTY16;
	OCR4B	=	STOP_DUTY16;
	OCR4C	=	STOP_DUTY16;
	
}

void timer5_init()
{
	T5_FPWM_SET									//Fast PWM
												//P.S. 8
												//compare match
												//A:C non-invrt

	WATCH_DOG_EN								//Watchdog on

	ICR5	=	TOP_40HZ;
	OCR5A	=	STOP_DUTY16;
	OCR5B	=	STOP_DUTY16;
	OCR5C	=	STOP_DUTY16;
	
}

void ADC_init()									//Analogue Digital Conversion Set
{
	ADCSRA	|=	(1<<	ADEN)
			|(1<<		ADPS0)
			|(1<<		ADPS1)
			|(1<<		ADPS2);			

	ADMUX	&= ~(1<<	REFS1);
	ADMUX	|=	(1<<	REFS0);					//Ref V cc setting

	ADMUX	&= ~(1<<	MUX0)
			&  ~(1<<	MUX3);
	ADMUX	|=	(1<<	MUX1)
			|	(1<<	MUX2);					//ADC6 single ended input for
	ADMUX	|=	(1<<	ADLAR);					//Left justified
	ADCSRB	&=	~(1<<	ADTS0)
			&	~(1<<	ADTS1)
			&	~(1<<	ADTS2);					//ADTS0:2 = 0X00
}

ISR(INT5_vect)									//for North
{
	init_DIR();
	NorthIsClear ^= 1;
}

ISR(INT4_vect)									//For South
{
	init_DIR();
	SouthIsClear ^= 1;
}

ISR(USART1_RX_vect)								//Serial In
{
	cli();
	
	uint8_t datz;
	
	while ( !(UCSR0A & (1<<RXC0)) );
	
	datz=UDR0;
	
	key = datz;
	
	setDIR_serial(key);
	
	WatchToken = HeelDog;				//heel Watchdog
	
	use_HX711();
	
	sei();
	
}

int main(void)
{
	gpio_init();
	Prox_ISR_EN();
	init_DIR();
	
	uart_init();
	
	timer1_init();
	timer4_init();
	timer5_init();
	
    sei();
	
    while (1) 
    {
		
    }
	
}

