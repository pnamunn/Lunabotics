/*
PD6 is Timer 0 Compare Match A Event
PD5 is Timer 0 Compare Match B Event

PB1 is Timer 1 Compare Match A Event
PB0 is Timer 1 Compare Match B Event

PB3 is Timer 2 Compare Match A Event				**Used For PWM
PD3 is Timer 2 Compare Match B Event		

PORT C Pin 1										**Used for IN1A
PORT C Pin 2										**Used for IN2A

PORT C PIN 3										**Used for IN1B
PORT D PIND 3										**used for IN2B

PD 6 OCR0A TIMER INTERRUPT
PD


Author:												Stuart Pollmann
Program:											TB6612FNG H-Bridge driver
Application:										Joystick Steering Drive train
Use Case:											Brush DC Motor Control 
Functionality:										joystick x/y axis multiplexed 
													ADC Read channels, direction
													Pin-states are realized on 
													physical joy position, if a
													position,thus terminal voltage											
													
													
*/

#define F_CPU 16000000
#define BAUD_9_6K 9600								// define baud
#define BAUD_115K	115200
#define BAUDRATE ((F_CPU/(16UL*BAUD_9_6K))-1)		// set baud rate value for UBRR

#include <avr/io.h>
#include <avr/interrupt.h>

#define IN1AHI		(PORTC |= (1<<PINC1));			//For L Motors
#define IN2AHI		(PORTC |= (1<<PINC2));			//For L Motors
													//For L Motors
#define IN1ALOW		(PORTC &= ~(1<<PINC1));			//For L Motors
#define IN2ALOW		(PORTC &= ~(1<<PINC2));			//For L Motors

#define IN1BHI		(PORTC |= (1<<	PINC3));		//For R Motors
#define IN2BHI		(PORTD |= (1<<	PIND4));		//For R Motors
													//For R Motors
#define IN1BLOW		(PORTC &= ~(1<<	PINC3));		//For R Motors
#define IN2BLOW		(PORTD &= ~(1<<	PIND4));		//For R Motors

#define FWD_DUTY	31;							//2ms
#define STOP_DUTY	23;							//1.5ms
#define RVRS_DUTY	15;							//1ms

#define DRIVE_L		OCR2A
#define DRIVE_R		OCR2B

#define EX_ACTUATOR	OCR0A
#define EX_ACTIVE	(PORTB |= (1<<PINB4));
#define EX_INACTIVE	(PORTB &= ~(1<<PINB4));

#define	EX_IN_PROG	(PORTD |= (1<<PIND7));

#define EX_CHAIN	OCR0B

#define zynOverDose	0;

volatile	uint8_t zyn4Scoobz = 0;
volatile	uint8_t DIRencoder = 0X00;
volatile	char key = 0;

/*
#define ADC128		(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)
#define StartADC	ADCSRA	|=	(1<<ADSC);
#define	ADCfreeRun	~(1<<ADTS0)&~(1<<ADTS1)&~(1<<ADTS2)

volatile uint8_t data_log[3] = {0,0,0};				//to hold packet content
char control[4]={'s', 'd', 'a', 'w'};	
uint8_t indexBuff = 0;
uint8_t com_state= 0;

#define JOY_R(x,y)	(((int8_t)y-127)-(((int8_t)x-127)>>1))
#define JOY_L(x,y)	(((int8_t)y-127)+(((int8_t)x-127)>>1))
*/

void ADC_init()									//Analogue Digital Conversion Set
{
	ADCSRA	|=	(1<<	ADEN)
			|(1<<		ADPS0)
			|(1<<		ADPS1)
			|(1<<		ADPS2)
			|(1<<		ADIE);					//Interrupt en

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

//void timer1_init(){}							//Watchdog

void timer2_init()								//Wave Form Generation	
{
	TCCR2A	|=	(1<<	WGM21);					//Fast PWM
	TCCR2A	|=	(1<<	WGM20);
	TCCR2B	&= ~(1<<	WGM22);
	
	TCCR2B	|=	(1<<	CS22)
			|	(1<<	CS21)
			|	(1<<	CS20);					//PS 1024					
	
	TCCR2A	|=	(1<<	COM2B1);				//setting clears OCRB on
	TCCR2A	&= ~(1<<	COM2B0);				//compare match as well as
	TCCR2A	|=	(1<<	COM2A1);				//clears OCRA on compare
	TCCR2A	&= ~(1<<	COM2A0);				//match (CTC)
	
	TIMSK2	|=	(1<<	TOIE2);					//Max over flow enable
	
	OCR2A = STOP_DUTY
	OCR2B = STOP_DUTY
	
}

void timer0_init ()
{
	TCCR0A |=	(1<<WGM00)
		   |	(1<<WGM01);
		   
	TCCR0B	&= ~(1<<WGM02);
	
	TCCR0A |=  (1<< COM0A1)
		   |   (1<< COM0B1);
		   
	TCCR0A &= ~(1<<	COM0A0)
		   &  ~(1<<	COM0B0);
		   
	TCCR0B |=  (1<<	CS02)
		   |   (1<<	CS00);  
	TCCR0B &= ~(1<<	CS01);						//P.S. 1024
	
	OCR0A = STOP_DUTY
	OCR0B = STOP_DUTY
	
}

void timer1_init()
{
	
	TCCR1A	|=	(1<<	COM1A1	) 
			|	(1<<	COM1B1	);
			
	TCCR1A	&= ~(1<<	COM1A0	) 
			&  ~(1<<	COM1B0	);
	
	TCCR1A	|=	(1<<	WGM11	);
	TCCR1A	&=	~(1<<	WGM10	);
	TCCR1B	|=	(1<<	WGM12	)
			|	(1<<	WGM13	);
	
	TCCR1B	|=	(1<<	CS11	);
	TCCR1B	&= ~(1<<	CS10	)
			&  ~(1<<	CS12	);		//P.S.256
			
	//ICR1	=	0XC350;					//50,000 dec
	ICR1	=	5000;
	
	//OCR1A	=	25000;					//1.5ms
	OCR1A	=	2500;
	OCR1B	=	3000;					//3000 dec
	
}


void GPIO_init()								//Data Direction Set
{
	DDRC &=	~(1<<0);							//set for ADC Read
	DDRB|=	 (1<<3)
		|	 (1<<2)
		|	 (1<<1)
		|	 (1<<0);							//set for COMP2A PWM out
	DDRD|=	 (1<<3)
		|	 (1<<4)
		|	 (1<<5)
		|	 (1<<6);							//set for COMP2B PWM out
	DDRC|=	 (1<<1)
		|	 (1<<2)
		|	 (1<<3);							//set for Logic out IN1&2
	
}
											
//** UART FUNCTIONS **//					
void uart_init (void)							// function to initialize UART
{

	//UBRR0H = (BAUDRATE>>8);					//shift the register right by 8 bits
	UBRR0L = 8;									// set lower baud rate to lower reg
												//USBSn =0 for 1 stop bit by default
												//UPMn1; UPMn0 are 0 by default pairty disabled
	
	UCSR0B	|=	(1<<	TXEN0)
			|	(1<<	RXEN0)
			|	(1<<	RXCIE0);				// enable receiver and transmitter
	
	UCSR0C	|=	(1<<	UCSZ00)
			|	(1<<	UCSZ01);				//reg C UCSZn1&n0 should be 1 for 8-bit char size

}

void serial_transmit (unsigned char data)		//Tx serial
{ 

	while (!( UCSR0A & (1<<UDRE0)));			//w8 b4 read;
												//UDREn is read when 1	
	UDR0 = data; 								//write the data in register

}

unsigned char uart_recieve (void)				//Rx serial
{
	
	while(!((UCSR0A) & (1<<RXC0)));				// wait while data is being received
	return UDR0;								// return 8-bit data read

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
//** END UART FUNCTIONS **//															

void init_DIR()									//Initiallize DIR to stop
{
		IN1ALOW
		IN2ALOW
		IN1BLOW
		IN2BLOW
}

/*
void setDIR (uint8_t X, uint8_t Y)				//Direction set
{
	int DZ = 40;								//Dead Zone buffer
	char tosend='A';							//Place holder
	
	if(X<(127-DZ) && ((127-DZ)<Y && Y<(127+DZ)))					
	{											//Crck Scrw L
												//low x, not y
												//x	= 01, y= 00									
		IN1AHI									//Set H-bride Ch1	
		IN2ALOW										
		
		IN1BLOW									//Set H-Bridge Ch2
		IN2BHI
		
		DIRencoder = 0x01;						//Encrypt DIR set
		tosend='L';								//Encrypt State Measure
		
	}
	
	else if ( X>(127+DZ) && ( (127-DZ)<Y && Y < (127+DZ)))			
	{											//Crck Scrw R
												//high x, not y																
												//x= 10, y= 00
		
		IN1ALOW									//Set H-bride Ch1	
		IN2AHI										
		
		IN1BHI									//Set H-bride Ch2
		IN2BLOW
		
		DIRencoder = 0X08;						//Encrypt DIR set
		tosend='R';								//Encrypt State Measure
		
	}
	
	else if (Y>(127+DZ) && ((127-DZ)<X && X<(127+DZ)))					
	{											//Forward
												//high y, and x
												//x = 11, y = 10
																				
		IN1AHI									//Set H-bride Ch1	
		IN2ALOW									
		
		IN1BHI									//Set H-bride Ch2
		IN2BLOW	
		
		DIRencoder	=	0X0D;					//Encrypt DIR set
		tosend		=	'F';					//Encrypt State Measure
		
	}
	
	else if (Y<(127-DZ) && ((127-DZ)<X && X<(127+DZ)))	
	{									
												//reverse
												//low y, and x
												//x= 11, y=01	
												
		IN1ALOW									//Set H-Bridge Ch 1
		IN2AHI						
		
		IN1BLOW									//Set H-Bridge Ch 2
		IN2BHI 
		
		DIRencoder = 0X0C;						//Encrypt DIR set
		tosend='B';								//Encrypt State Measure
	}	
	
	else											
	{												
												//Null
												//x=00, y=00
												
		IN1ALOW									//Turn off Dir Ch A
		IN2ALOW
		
		IN1BLOW									//Turn off Ch B
		IN2BLOW
		
		DIRencoder = 0X00;						//Encrypt DIR set
		tosend='S';								//Encrypt State Measure
	}
	
	serial_transmit(tosend);					//Send State to serial
	serial_transmit('\n');
}
*/


void setDIR_serial(char input)					//And speed atm
{
	char tosend='A';							//Place holder
		
	if(input=='a')
	{											//Crck Scrw L
		//low x, not y
		//x	= 01, y= 00
		
		IN1AHI									//Set H-bride Ch1
		IN2ALOW
		
		IN1BLOW									//Set H-Bridge Ch2
		IN2BHI
		
		
		DRIVE_L = FWD_DUTY
		DRIVE_R = RVRS_DUTY
		
		DIRencoder = 0x01;						//Encrypt DIR set
		tosend='L';								//Encrypt State Measure
		
	}
	
	else if (input=='d')
	{											//Crck Scrw R
		//high x, not y
		//x= 10, y= 00
		
		IN1ALOW									//Set H-bride Ch1
		IN2AHI
		
		IN1BHI									//Set H-bride Ch2
		IN2BLOW
		
		DRIVE_L = RVRS_DUTY
		DRIVE_R= FWD_DUTY
		
		DIRencoder = 0X08;						//Encrypt DIR set
		tosend='R';								//Encrypt State Measure

		
	}
	
	else if (input=='w')
	{											//Forward
		//high y, and x
		//x = 11, y = 10
		
		
		IN1AHI									//Set H-bride Ch1
		IN2ALOW
		
		IN1BHI									//Set H-bride Ch2
		IN2BLOW
		
		
		DRIVE_L = FWD_DUTY
		DRIVE_R = FWD_DUTY
		
		DIRencoder	=	0X0D;					//Encrypt DIR set
		tosend		=	'F';					//Encrypt State Measure
		
	}
	
	else if (input=='s')
	{
		//reverse
		//low y, and x
		//x= 11, y=01
		
		IN1ALOW									//Set H-Bridge Ch 1
		IN2AHI
		
		IN1BLOW									//Set H-Bridge Ch 2
		IN2BHI
		

		DRIVE_L = RVRS_DUTY
		DRIVE_R = RVRS_DUTY		
		
		DIRencoder = 0X0C;						//Encrypt DIR set
		tosend='B';								//Encrypt State Measure
	}
	
		else if (input == '1')
		{
			EX_ACTUATOR = RVRS_DUTY
			tosend='D';
		}
		
		else if (input == '2')
		{
			EX_ACTUATOR = STOP_DUTY
			tosend='S';
		}
		
		else if (input == '3')
		{
			EX_ACTUATOR =	FWD_DUTY
			tosend='U';
		}
			
			else if (input == '8')
			{
				EX_CHAIN = FWD_DUTY
				tosend='O';
			}
			else if (input == '9')
			{
				EX_CHAIN = STOP_DUTY
				tosend='h';
				}
	
	else
	{
		//Null
		//x=00, y=00
		
		IN1ALOW									//Turn off Dir Ch A
		IN2ALOW
		
		IN1BLOW									//Turn off Ch B
		IN2BLOW
		
		DRIVE_L = STOP_DUTY
		DRIVE_R = STOP_DUTY
		EX_ACTUATOR = STOP_DUTY
		EX_CHAIN	= STOP_DUTY
		
		DIRencoder = 0X00;						//Encrypt DIR set
		tosend='S';								//Encrypt State Measure
	}
	
		serial_transmit(tosend);					//Send State to serial

		serial_transmit('\n');
		serial_transmit('\r');
}


uint8_t abs_i(int8_t val)						//Absolute Value Function
{
	if(val<0)									//force neg to Posi
	{
		return (uint8_t)(val*(-1));
	}
	else										//else nbd
	{
		return (uint8_t)val;
	}
	}

//void setSpd(uint8_t x, uint8_t y)				//Speed Set
void setSpd()
{
	/*
	if(DIRencoder==0x01)						//L
	{
		OCR2A = abs_i(JOY_R(x,y));				//Still Being Developed
		OCR2B = abs_i(JOY_L(x,y));				//Still Being Developed
	}		
							  
	else if(DIRencoder==0x08)					//R	
	{							
		OCR2A = abs_i(JOY_R(x,y));				//Still Being Developed
		OCR2B = abs_i(JOY_L(x,y));				//Still Being Developed
	}	
								 
	else if(DIRencoder==0x0D)					//F	
	{											
		OCR2A = abs_i(JOY_R(x,y));				//Still Being Developed
		OCR2B = abs_i(JOY_L(x,y));				//Still Being Developed
	}	
								  
	else if(DIRencoder==0x0C)					//B	 
	{											
		OCR2A = abs_i(JOY_R(x,y));				//Still Being Developed
		OCR2B = abs_i(JOY_L(x,y));				//Still Being Developed
	}
	*/
	
	if (DIRencoder)								//If a valid control sig
	{
		OCR2A=255;								//too slow?
		OCR2B=255;								//SPI UART to Saber
	}
	else
	{
		OCR2A=0;								//Null State
		OCR2B=0;
	}	
}



/*

ISR(ADC_vect)									//Interrupt Sub Routine
{
	static	uint8_t	x = 0;
	static	uint8_t y = 0;
	
	
	if		((ADMUX	&	0X0F) ==	7)			//bot have been read
	{
		
		x	=	ADCH;							//This is the final multiplex
		setDIR(x,y);							//set direction state
		setSpd(x,y);							//set speed
		
		serial_transmit('A');					//spit out quick Label
		serial_transmit('D');					//to serial monitor
		serial_transmit('C');
		serial_transmit('\t');
		
		term_Send_Val_as_Digits(x);				//print x
		serial_transmit(',');
		term_Send_Val_as_Digits(y);				//print y
		serial_transmit('\t');
		
		printBin8(ADMUX);						//print current ADMUX CHannel
		serial_transmit('\n');
		
		ADMUX &= ~(1);							//Change MUX channel
	}
	
	else if	((ADMUX	&	0X0F) ==	6)			//Poor place for the first read
	{
		y	=	ADCH;							//read Y ADCH
		ADMUX|= (1);							//and move on to x
	}
	
	else										//Force starting condition
	{
		ADMUX |= (6u);
	}
	
	ADCSRA	|=	(1<<ADSC);						//Start next conversion

}
*/

/*
 void conducrtor()
 {
	 
 }
 
 */


void scoobyGottaGun()
{
	if(zyn4Scoobz == 240)					//kill operations
	{
		DRIVE_L		=		STOP_DUTY
		DRIVE_R		=		STOP_DUTY
		EX_ACTUATOR =		STOP_DUTY
		EX_CHAIN	=		STOP_DUTY
		zyn4Scoobz	=		zynOverDose	
		
			serial_transmit('Z');
			serial_transmit('O');
			serial_transmit('I');
			serial_transmit('N');
			serial_transmit('K');
			serial_transmit('S');

			serial_transmit('\n');
			serial_transmit('\r');
	}
	
	zyn4Scoobz++;
}


ISR(TIMER2_OVF_vect)
{

	//if(zyn4Scoobz == 240)					//kill operations
	//{
	//	DRIVE_L		=		STOP_DUTY
	//	DRIVE_R		=		STOP_DUTY
	//	EX_ACTUATOR =		STOP_DUTY
	//	EX_CHAIN	=		STOP_DUTY
	//	zyn4Scoobz	=		zynOverDose
	//	
	//	key = 'g';
	//}
	
	scoobyGottaGun();
	//zyn4Scoobz++;
	
	
}


ISR(USART_RX_vect)
{
	uint8_t datz;
	
	while ( !(UCSR0A & (1<<RXC0)) );
	
	datz=UDR0;
	
	key = datz;
	
	setDIR_serial(key);
	
	zyn4Scoobz = zynOverDose;
	
		/*
		setSpd();
		if (!com_state)									//if not data state
		{
			indexBuff=datz;								//store index
			  
		}
		else
		{
		
		control[indexBuff]=datz;	
		}
		*/
	
}



int main(void) 
{

	GPIO_init();
	init_DIR();
	uart_init();
	timer2_init();
	timer1_init();	
	timer0_init();
	
	//ADC_init();
	
/*	Should NEVER print (it did ty Joe)
	issue: set TIMSK for COM 2A&B w no ISR
			to jump to, processor would
			soft restart and print from here
	Functionality: Sanity Check
	
	serial_transmit('Z');
	serial_transmit('Y');
	serial_transmit('N');
	term_Send_Val_as_Digits(k++);
	serial_transmit('\n');
*/
	
	//ADCSRA	|=	(1<<ADSC);
	//char c;
	
	sei();

	while (1) 
	{
			
	}
}



