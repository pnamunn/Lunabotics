
/*
PD6 is Timer 0 Compare Match A Event
PD5 is Timer 0 Compare Match B Event

PB1 is Timer 1 Compare Match A Event
PB0 is Timer 1 Compare Match B Event

PB3 is Timer 2 Compare Match A Event			**Used For PWM
PD3 is Timer 2 Compare Match B Event		

PORT C Pin 1									**Used for IN1
PORT C Pin 2									**Used for IN2

Author:			Stuart Pollmann
Program:		DC Motor Control
Application:	Driving TB6612FNG H-Bridge
Use Case:		Direction and Speed control
Functionality:	Fast PWM duty cycle set by adjusting
				duty cycle via ADC of pot position. 
				Direction is set on conditional, 2 
				Pins required for direction logic
*/

#define F_CPU 16000000
#define BAUD_9_6K 9600								// define baud
#define BAUD_115K	115200
#define BAUDRATE ((F_CPU/(16UL*BAUD_9_6K))-1)		// set baud rate value for UBRR

#include <avr/io.h>
#include <avr/interrupt.h>

//#define LEDBUILTIN	PINB5

#define IN1AHI		(PORTC |= (1<<PINC1));
#define IN2AHI		(PORTC |= (1<<PINC2));

#define IN1BHI		(PORTC |= (1<<	PINC3));
#define IN2BHI		(PORTD |= (1<<	PIND4));

#define IN1BLOW		(PORTC &= ~(1<<	PINC3));		//For the R Motor[s]
#define IN2BLOW		(PORTD &= ~(1<<	PIND4));

#define IN1ALOW		(PORTC &= ~(1<<PINC1));
#define IN2ALOW		(PORTC &= ~(1<<PINC2));

#define ADC128		(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)
#define StartADC	ADCSRA	|=	(1<<ADSC);
#define	ADCfreeRun	~(1<<ADTS0)&~(1<<ADTS1)&~(1<<ADTS2)

#define JOY_R(x,y)	(((int8_t)y-127)-(((int8_t)x-127)>>1))
#define JOY_L(x,y)	(((int8_t)y-127)+(((int8_t)x-127)>>1))


volatile	uint8_t ledSelect=0;					// Stores which LED is currently selected
volatile	uint8_t escDutyCycleValue=0;			// Stores the duty cycle value for the ESC
volatile	uint8_t DIRencoder = 0X00;




void ADC_init()
{
	ADCSRA	|=	(1<<	ADEN)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)|(1<<	ADIE);

	ADMUX	&= ~(1<<	REFS1);						
	ADMUX	|=	(1<<	REFS0);						//Ref V cc setting

	ADMUX	&= ~(1<<	MUX0)							
			&  ~(1<<	MUX3);						 
	ADMUX	|=	(1<<	MUX1)
			|	(1<<	MUX2);						//ADC6 single ended input for
	ADMUX	|=	(1<<	ADLAR);						//Left justified
	ADCSRB	&=	~(1<<ADTS0)&~(1<<ADTS1)&~(1<<ADTS2);					//ADTS0:2 = 0X00
	DIDR0	|=	(1<<	ADC0D);						//Meh.. save power
	//StartADC

}

void timer2_init()									
{
	TCCR2A |=(1<<WGM21);							//Fast PWM
	TCCR2A |=(1<<WGM20);
	TCCR2B &= ~(1<<WGM22);
	TCCR2B |=(1<<CS22)| (1<<CS21);					//PS 256
	TCCR2B &= ~(1<<CS20);
	TCCR2A |=(1<<COM2B1);							//setting clears OCRB on
	TCCR2A &= ~(1<<COM2B0);							//compare match as well as
	TCCR2A |= (1<<COM2A1);							//clears OCRA on compare
	TCCR2A &= ~(1<<COM2A0);							//match (CTC)
	//TIMSK2 |= (1<<OCIE2A);							//Compare match A and B
	//TIMSK2 |= (1<<OCIE2B);							//interrupt vector enabled


	OCR2A = 0;										//(200hz desired)/
													//(244HZ(top w p.s)
	OCR2B = 0;										//(0.8192 ratio)*(255size)=
													//208 OCRA top
}

void GPIO_init()
{
	DDRC &= ~(1<<0);								//set for ADC Read
	DDRB |= (1<<3)|(1<<0)|(1<<1);					//set for COMP2A PWM out
	DDRD|=(1<<3)|(1<<4);									//set for COMP2B PWM out
	DDRC|= (1<<1)|(1<<2)|(1<<3);							//set for Logic out IN1&2
	
}
					
void uart_init (void)						// function to initialize UART
{

	//UBRR0H = (BAUDRATE>>8);					// shift the register right by 8 bits
	UBRR0L = 8;						// set lower baud rate to lower reg
											//USBSn =0 for 1 stop bit by default
											//UPMn1; UPMn0 are 0 by default pairty disabled
	
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0);			// enable receiver and transmitter
	UCSR0C|= (1<<UCSZ00)|(1<<UCSZ01);		//reg C UCSZn1&n0 should be 1 for 8-bit char size

}

void serial_transmit (unsigned char data)	//Tx serial
{ 
	//cli();

	while (!( UCSR0A & (1<<UDRE0)));	//w8 b4 read;
										//UDREn is read when 1	
	UDR0 = data; 						//write the data in register

	//sei();
}

unsigned char uart_recieve (void)			//Rx serial
{
	//cli();
	
	while(!((UCSR0A) & (1<<RXC0)));		// wait while data is being received
	return UDR0;						// return 8-bit data read
	
	//sei();
}

void term_Send_Val_as_Digits(uint8_t val)
{
	uint8_t digit = '0';

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

void printBin8(uint8_t data){
	for(uint8_t n = 0; n < 8; n++){
		uint8_t tm_ = (data >> (7 - n)) & (0x01);
		if(tm_){
			serial_transmit('1');
			} else {
			serial_transmit('0');
		}
	}
}

void init_DIR()
{
		IN1ALOW
		IN2ALOW
		IN1BLOW
		IN2BLOW
}

void setDIR (uint8_t x, uint8_t y)
{
	int DZ = 40;
	
	char tosend='A';
	
	if(x<(127-DZ) && ((127-DZ)<y && y<(127+DZ)))					//Crck Scrw L
	{										//low x, not y
		IN1AHI								//x	= 01, y= 00
		IN2ALOW
		
		IN1BLOW
		IN2BHI
		DIRencoder = 0x01;
		//serial_transmit('L');
		tosend='L';
	}
	
	else if (x>(127+DZ) && ((127-DZ)<y && y<(127+DZ)))			//Crck Scrw R
	{										//high x, not y
		IN1ALOW								//x= 10, y= 00
		IN2AHI
		
		IN1BHI
		IN2BLOW
		DIRencoder = 0X08;
		//serial_transmit('R');
		tosend='R';
	}
	
	else if (y>(127+DZ) && ((127-DZ)<x && x<(127+DZ)))					//Forward
	{										//high y, and x
		IN1AHI								//x = 11, y = 10
		IN2ALOW
		
		IN1BHI
		IN2BLOW	
		DIRencoder=0X0D;
		//serial_transmit('F');
		tosend='F';
	}
	else if (y<(127-DZ) && ((127-DZ)<x && x<(127+DZ)))					//reverse
	{										//low y, and x
		IN1ALOW								//x= 11, y=01
		IN2AHI
		
		IN1BLOW
		IN2BHI 
		DIRencoder = 0X0C;
		//serial_transmit('R');
		tosend='B';
	}	
	else									//Null
	{										//x=00, y=00
		IN1ALOW
		IN2ALOW
		IN1BLOW
		IN2BLOW
		DIRencoder = 0X00;
		//serial_transmit('S');
		tosend='S';
	}
	
	serial_transmit(tosend);
	serial_transmit('\n');
	//serial_transmit('\r');
	
}

uint8_t abs_i(int8_t val){
	if(val<0){
		return (uint8_t)(val*(-1));
	}
	else
	{
		return (uint8_t)val;
	}
}

void setSpd(uint8_t x, uint8_t y)
{
	if(DIRencoder==0x01)//L
	{
		OCR2A = abs_i(JOY_R(x,y));//((Ymag-127)<<1)-((Xmag-127));
		OCR2B = abs_i(JOY_L(x,y));	//(((Ymag-127)-((Xmag-127)>>1)));
	}
	else if(DIRencoder==0x08)//R
	{
		OCR2A = abs_i(JOY_R(x,y));//((Ymag-127)<<1)-((Xmag-127));
		OCR2B = abs_i(JOY_L(x,y));	//(((Ymag-127)-((Xmag-127)>>1)));
	}
	else if(DIRencoder==0x0D)//F
	{
		OCR2A = abs_i(JOY_R(x,y));//((Ymag-127)<<1)-((Xmag-127));
		OCR2B = abs_i(JOY_L(x,y));	//(((Ymag-127)-((Xmag-127)>>1)));
	}
	else if(DIRencoder==0x0C)//B
	{
		OCR2A = abs_i(JOY_R(x,y));//((Ymag-127)<<1)-((Xmag-127));
		OCR2B = abs_i(JOY_L(x,y));	//(((Ymag-127)-((Xmag-127)>>1)));
	}
	
	else
	{
		OCR2A=0;
		OCR2B=0;
	}
	
}


ISR(ADC_vect)
{
	static	uint8_t	x = 0;
	static	uint8_t y = 0;
	
	//static uint8_t L = 128;
	//static uint8_t R = 128;
	
	if		((ADMUX	&	0X0F) ==	7)
	{
		x	=	ADCH;
		setDIR(x,y);
		setSpd(x,y);
		
		serial_transmit('A');
		serial_transmit('D');
		serial_transmit('C');
		serial_transmit('\t');
		term_Send_Val_as_Digits(x);
		serial_transmit(',');
		term_Send_Val_as_Digits(y);
		serial_transmit('\t');
		printBin8(ADMUX);
		
		serial_transmit('\t');
		printBin8(ADMUX);
		serial_transmit('\n');
		ADMUX &= ~(1);
		
	}
	else if	((ADMUX	&	0X0F) ==	6)
	{
		y	=	ADCH;
		setDIR(x,y);
		setSpd(x,y);
		ADMUX|= (1);
	}
	
	else
	{
		ADMUX |= (6u);
	}
	
	ADCSRA	|=	(1<<ADSC);
	//StartADC
	//uint32_t a=0;
	//for(;a<0xFFFF;++a);
	
}



int main(void) 
{

	GPIO_init();
	init_DIR();
	uart_init();
	timer2_init();	
	ADC_init();
	static uint8_t k=0;
	
	serial_transmit('Z');
	serial_transmit('Y');
	serial_transmit('N');
	term_Send_Val_as_Digits(k++);
	serial_transmit('\n');
	

	
	ADCSRA	|=	(1<<ADSC);
	sei();

	while (1) 
	{
		static uint8_t a = 0;
		a++;
		__asm("nop");
	}
}







/*
ISR(ADC_vect)
{
	//sei();
	escDutyCycleValue = ADCH;
	

	if(escDutyCycleValue >= 128+2)
	{
		
		IN1AHI
		IN2ALOW
		IN1BHI
		IN2BLOW
		
		OCR2A = ((escDutyCycleValue-128) << 1);
		OCR2B=OCR2A;
	}
	
	else if (escDutyCycleValue < 128-2)
	{
		IN1ALOW
		IN2AHI
		IN1BLOW
		IN2BHI

		
		OCR2A = ((127-escDutyCycleValue) << 1);
		OCR2B=OCR2A;
	}
	
	else
	{
		IN1ALOW
		IN2ALOW
		IN1BLOW
		IN2BLOW
	}

}
*/

/*
void timer0_init()									//WG for polling
{

													//CTC Mode
	TCCR0A &= ~(1<<0);								//WGM00 = 0
	TCCR0A |= (1<<1);								//WGM01 = 1
	TCCR0B &= ~(1<<3);								//WGM02 = 0
	OCR0A = 0x1F;									//31 proportion to 2ms
	OCR0B = 0X00;									//initialize
	TIMSK0 |= (1<<OCIE0A);							//enable ISR COMPA vector
													//p.s = 1024
	TCCR0B |= (1<<0);								//CS00
	TCCR0B &= ~(1<<1);								//CS01
	TCCR0B |= (1<<2);								//CS02
}
*/