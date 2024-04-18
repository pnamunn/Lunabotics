/*
 * HX711 DEV.c
 *
 * Created: 4/5/2024 6:55:09 PM
 * Author : Main
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


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
volatile uint16_t	known_grams			= 516;
volatile uint16_t	known_bit			= 270;
volatile uint16_t	grams				= 0;
volatile uint16_t	Tare_it_bruh		= 0;
volatile uint16_t	known_Delta_Ref		= 0;			//Associative bits to 516 Grams
volatile uint8_t	DepBinLoading		= 1;

void uart_init (void)						//initialize UART
{

	
	PRR0 &= ~(1<<PRUSART0);
	PRR1 &= ~(1<<PRUSART1);
	PRR1 &= ~(1<<PRUSART2);
	PRR1 &= ~(1<<PRUSART3);
	
	UBRR0H = 0;
	UBRR0L = 16;							//BAUD 115200
	
	UCSR0A &=  ~(1<<	DOR0)
		   &   ~(1<<	UPE0);
	
	UCSR0A	|=	(1<<	RXC0);
	UCSR0A	|=	(1<<	U2X0);					//p219 ref
	
	UCSR0B	|=	(1<<	RXEN0)
			|	(1<<	TXEN0)
			|	(1<<	RXCIE0);			//EN: Interrupt
	//EN: receiver/transmitter
	// Rx must be initialized first!
	
	UCSR0C	|=	(1<<	UCSZ00)
			|	(1<<	UCSZ01);			//8-bit char size	
}

void serial_transmit (unsigned char data)	//Tx serial
{

	while (!( UCSR0A & (1<<UDRE0)));		//w8 b4 read;
	//UDREn is read when 1
	UDR0 = data; 							//write the data in register

}

unsigned char uart_recieve (void)			//Rx serial
{
	
	while(!((UCSR0A) & (1<<RXC0)));			//w8t while data being received
	return UDR0;							//return 8-bit data read

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


void gpio_init()							//in's and out's
{
	
	PORTL |= (1<<6);	
	SCK_SIG_OUT
	HX711_DAT_IN
	
}

void use_HX711()
{
	
	cli	();
	//static uint8_t sensor_IP =1;

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

void tare_scale()
{
	tare_flag	=	1;
}

ISR(USART0_RX_vect, ISR_BLOCK)				//Serial In
{
	cli();									//disable interrupts
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

int main(void)
{
	uart_init();
	gpio_init();

	//uint16_t grams = 0;
    /* Replace with your application code */
    while (1) 
    {
		
		
		if(HX711_DATA_RDY)
		{
			use_HX711();
			hx711_data|=0xFF000000;
			hx711_data = ~hx711_data;
			hx711_data +=1;				//undo 2's comp
			
			hx711_data>>=8;
			sense_read|=hx711_data;
			
			if (tare_flag)
			{
				Tare_it_bruh = sense_read;
			}
			
			sense_read= sense_read-(Tare_it_bruh-3);
			
			//grams=(sense_read<<1);
			
			grams = ((uint32_t)sense_read*(uint32_t)known_grams)/275;
			term_Send_16_as_Digits(grams);
			serial_transmit('\n');				//Print nxt on
			serial_transmit('\r');				//new line
			
			tare_flag=0;
			sense_read		= 0;
			hx711_data =0;
			/*
			if (grams>50000)
			{
				grams = 0;
				term_Send_16_as_Digits(grams);
				
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
			    term_Send_16_as_Digits(grams);
				
				serial_transmit('G');				//notify Terminal
				serial_transmit('R');
				serial_transmit('A');
				serial_transmit('M');
				serial_transmit('S');
					serial_transmit('\n');				//Print nxt on
					serial_transmit('\r');				//new line
			sense_read		= 0;
			}
			
			*/
		
		}
	}
}

