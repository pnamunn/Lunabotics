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
#define BAUD 9600								// define baud
#define BAUDRATE ((F_CPU/(16UL*BAUD))-1)		// set baud rate value for UBRR

#include <avr/io.h>
#include <avr/interrupt.h>

#define LEDBUILTIN	PINB5
#define IN1LOW		PORTC &= ~(1<<PINC1);
#define IN2LOW		PORTC &= ~(1<<PINC2);
#define IN1HI		PORTC |= (1<<PINC1);
#define IN2HI		PORTC |= (1<<PINC2);


volatile uint8_t ledSelect=0;					// Stores which LED is currently selected
volatile uint8_t escDutyCycleValue=0;			// Stores the duty cycle value for the ESC

void init_serial(unsigned int someVal)
{												// Debug for use with development boards

	//UBRR0L = 3;								// 250k BAUD
	UBRR0L = 8;									// 115200 BAUD
												
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);		// Enable Tx and Rx
												// Setup 8N2 format // Change to 8N1 later							
	UCSR0C = (3 << UCSZ00);						// 8N1 Format
}


void serialWrite(unsigned char data)
{
	cli();
	while(!(UCSR0A & (1 << UDRE0)));			// Wait for empty transmit buffer
	UDR0 = data;
	sei();
}


void ADC_init()
{

	ADCSRA	|=(1<<ADEN)								
	| (1<<ADATE)									
	| (1<<ADIE)										
	| (1<<ADPS0)									
	| (1<<ADPS1)
	| (1<<ADPS2);									/*ADC enable*/
													/*ADC auto trigger enable*/
													/*interrupt vector enable*/
													/*Pre-scalar 1024*/

	ADMUX &= ~(1<<REFS1);							//Ref V cc setting
	ADMUX |= (1<< REFS0);

	ADMUX &= ~(1<<MUX0)								
	&  ~(1<<MUX1)
	&  ~(1<<MUX2)
	&  ~(1<<MUX3);									/*ADC0 single ended input for PINC0 (my Y)*/

	ADMUX |= (1<<ADLAR);							/*Left justified*/

	ADCSRB &= ~(1<< ADTS2);							//trigger set to timer 0 compare A match
	ADCSRB |= (1<<ADTS1) | (1<<ADTS0);
	ADCSRA |= (1<<ADSC);

	DIDR0 |= (1<<ADC0D);							//Meh.. save power

}

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
	TIMSK2 |= (1<<OCIE2A);							//Compare match A and B
	TIMSK2 |= (1<<OCIE2B);							//interrupt vector enabled


	OCR2A = 0;										//(200hz desired)/
													//(244HZ(top w p.s)
	OCR2B = 0;										//(0.8192 ratio)*(255size)=
													//208 OCRA top
}

void GPIO_init()
{
	DDRC &= ~(1<<0);								//set for ADC Read
	DDRB |= (1<<3)|(1<<0)|(1<<1);					//set for COMP2A PWM out
	DDRD|=(1<<3);									//set for COMP2B PWM out
	DDRC|= (1<<1)|(1<<2);							//set for Logic out IN1&2
	
}


int main(void) 
{

	
	GPIO_init();
	timer0_init();
	timer2_init();	
ADC_init();
	sei();

	while (1) 
	{
	
	}
}



ISR(ADC_vect)
{
	//sei();
	escDutyCycleValue = ADCH;
	

	if(escDutyCycleValue >= 0x80)
	{
		
		IN1HI
		IN2LOW
		OCR2A = ((escDutyCycleValue-128) << 1);
		OCR2B=0;
	}
	else if (escDutyCycleValue < 0x80)
	{
		IN1LOW
		IN2HI
		OCR2A = ((127-escDutyCycleValue) << 1);
		OCR2B=0;
	}

}

/*
ISR(ADC_vect)
{
	//sei();
	escDutyCycleValue = ADCH;
	
	serialWrite('A');
	serialWrite(10);
	if(escDutyCycleValue >= 0x80)
	{
		OCR2B = 0;
		OCR2A = ((escDutyCycleValue-128) << 1);
	}
	else if (escDutyCycleValue < 0x80)
	{
		OCR2A = 0;
		OCR2B = ((127-escDutyCycleValue) << 1);
	}

}
*/


/*

	//#define GREEN_LED 0					//PB3
	//#define RED_LED   1					//PD3
	//#define LEDON(a)  PORTB |= (1 << a)
	//#define LEDOFF(a) PORTB &= ~(1 << a)

	// Set Baud rate
	//UBRR0H = (unsigned char)(someVal >> 8);
	//UBRR0L = (unsigned char)(someVal);

	DDRB|=(1<<LEDBUILTIN);
	PORTB|=(1<<LEDBUILTIN);
	init_serial(8);
while (1) 
	{


	(escDutyCycleValue & 0x80) ? 
	grnSelect(&escDutyCycleValue): 
	redSelect(&escDutyCycleValue);



	
	//PINB|=(1<<LEDBUILTIN);

	//serialWrite('B');
	//serialWrite(10);

	//PORTB|=(1<<0);
	//OCR2B = escDutyCycleValue;
	}

void grnSelect (uint8_t *adcIN)
{
	OCR2B = 0;
	OCR2A = ((adcIN-127)<<1);
}

void redSelect(uint8_t *adcIn)
{
	OCR2A=0;
	OCR2B = ((127-adcIn)<<1);
}

ISR(ADC_vect)
{
	//sei();
	escDutyCycleValue = ADCH;
	//ledSelect = (escDutyCycleValue & 0x80) ? GREEN_LED : RED_LED;
}
*/
