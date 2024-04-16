/*
 * ASC758 driver.c
 *
 * Created: 2/15/2024 10:25:48 PM
 * Author : Main
 */ 

#define F_CPU 16000000
#define BAUD_9_6K 9600								// define baud
#define BAUD_115K	115200
#define BAUDRATE ((F_CPU/(16UL*BAUD_9_6K))-1)		// set baud rate value for UBRR

#include <avr/io.h>
#include <avr/interrupt.h>


void ADC_init()									//Analogue Digital Conversion Set
{
	ADCSRA	|=	(1<<	ADEN)
			|	(1<<	ADPS0)
			|	(1<<	ADPS1)
			|	(1<<	ADPS2)
			|	(1<<	ADIE);					//Interrupt en

	ADMUX	&= ~(1<<	REFS1);
	ADMUX	|=	(1<<	REFS0);					//Ref V cc setting

	ADMUX	&= ~(1<<	MUX0)
			&  ~(1<<	MUX3);
	
	ADMUX	|=	(1<<	MUX1)
			|	(1<<	MUX2);					//ADC6 single ended input for
	
	//ADMUX	|=	(1<<	ADLAR);					//Left justified
	ADMUX	&=	~(1<<	ADLAR);					//Right justified
		
	ADCSRB	&=	~(1<<	ADTS0)
			&	~(1<<	ADTS1)
			&	~(1<<	ADTS2);					//ADTS0:2 = 0X00
}



//** UART FUNCTIONS **//
void uart_init (void)							// function to initialize UART
{

	UBRR0L = 8;									// set lower baud rate to lower reg

	
	UCSR0B	|=	(1<<	TXEN0)
			|	(1<<	RXEN0);

	
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

void term_Send_16Val_as_Digits(uint16_t val)
{
	
	uint8_t digit = '0';						//Initialize

	while(val >= 10000)							//incoming hundred's
	{
		digit	+= 1;							//count
		val		-= 10000;						//chunks of ten thou
	}
	
	serial_transmit(digit);
	digit = '0';								//initialize
		
	while(val >= 1000)							//incoming hundred's
	{
		digit	+= 1;							//count
		val		-= 1000;						//chunks of thou
	}
	
	serial_transmit(digit);
	digit = '0';								//initialize
			
			
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

int main(void) 
{
	uart_init();
	ADC_init();
	
/*	Should NEVER print
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
	
	ADCSRA	|=	(1<<ADSC);
	
	sei();
	while(1)
	{
		
	}
}


ISR(ADC_vect)									//Interrupt Sub Routine
{
	//static	uint8_t	ASC7581 = 0;				//this is just a resistor div
	//static	uint8_t ASC7582 = 0;
	
	static	uint16_t ASC7581 = 0;				//this is just a resistor div
	static	uint16_t ASC7582 = 0;
	uint8_t ASC_L;
	uint8_t ASC_H;
	
	
	if		((ADMUX	&	0X0F) ==	7)			//bot have been read
	{
		
		//ASC7582 = ADCH;
		
		ASC_L = ADCL;
		ASC_H = ADCH;
		
		ASC7582	=	(ASC_H<<8)|ASC_L;			//This is the final multiplex
		
		serial_transmit('A');					//spit out quick Label
		serial_transmit('D');					//to serial monitor
		serial_transmit('C');
		serial_transmit('1');
		serial_transmit(':');
		serial_transmit(' ');			
		//term_Send_Val_as_Digits(ASC7581);		//print x
		term_Send_16Val_as_Digits(ASC7581);
		serial_transmit('\n');					//Print nxt on
		serial_transmit('\r');					//new line
		
		serial_transmit('A');					//spit out quick Label
		serial_transmit('D');					//to serial monitor
		serial_transmit('C');
		serial_transmit('2');		
		serial_transmit(':');
		serial_transmit(' ');			
		//term_Send_Val_as_Digits(ASC7582);		//print y
		term_Send_16Val_as_Digits(ASC7582);
		serial_transmit('\n');					//Print nxt on
		serial_transmit('\r');					//new line
		
		serial_transmit('C');					
		serial_transmit('H');					
		serial_transmit('1');					
		serial_transmit(':');					
		serial_transmit(' ');					
		printBin8(ADMUX);						//print current ADMUX CHannel
		serial_transmit('\n');					//Print nxt on
		serial_transmit('\r');					//new line
		
		ADMUX &= ~(1);							//Change MUX channel 6
		serial_transmit('C');
		serial_transmit('H');
		serial_transmit('2');
		serial_transmit(':');
		serial_transmit(' ');
		printBin8(ADMUX);						//print current ADMUX CHannel
		serial_transmit('\n');					//Print nxt on
		serial_transmit('\r');					//new line	
	}
	
	else if	((ADMUX	&	0X0F) ==	6)			//Poor place for the first read
	{
		//ASC7581	=	ADCH;						//read 1 ADCH
			
		ASC_L = ADCL;
		ASC_H = ADCH;
		
		ASC7581	=	(ASC_H<<8)|ASC_L;
		
		
		ADMUX|= (1);							//and move on to 2 CH7
	}
	
	else										//Force starting condition
	{
		ADMUX |= (6u);
	}
	
	ADCSRA	|=	(1<<ADSC);						//Start next conversion

}
