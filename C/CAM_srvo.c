/*
Servo_Stuff *
 * Created: 4/17/2024 3:39:03 PM
 * Author : nickg
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#define T1A_COM_HI		(TCCR1A	|=  (1<<COM1A1)	| (1<<COM1B1)); //| (1<<COM1C1));
#define T1A_COM_LO		(TCCR1A	&= ~(1<<COM1A0)	&~(1<<COM1B0)); //&~(1<<COM1C0));
#define T1A_WGM_HI		(TCCR1A	|=	(1<<WGM11));
#define T1A_WGM_LO		(TCCR1A	&= ~(1<<WGM10));
#define T1B_WGM_HI		(TCCR1B	|=	(1<<WGM12) |  (1<<WGM13));
#define T1A_CS__LO		(TCCR1B	&= ~(1<< CS12)	&~(1<<CS10));
#define T1B_CS__HI		(TCCR1B	|=	(1<< CS11));

#define FPWM_1A_OUT		(DDRB	|=	(1<<PINB5));
#define FPWM_1B_OUT		(DDRB	|=	(1<<PINB6));
#define FPWM_1C_OUT		(DDRB	|=	(1<<PINB7));

#define FWD	 3999;
#define CAM_STOP_DUTY	 2999;
#define RVS  1999;

#define TOP_40HZ	50000

#define Y_AXIS		OCR1A
#define X_AXIS		OCR1B

//-----UART FUNCTIONS-------------------------------------//

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

void serial_transmit (uint8_t data)	//Tx serial
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
	//from ascii: '0'
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
	DDRB |= (1<<1); //Pin B1 is set to output
	DDRB |= (1<<2); //Pin B2 is set to output
}

void init_DIR()		// initialize all motor direction to be stopped
{
	X_AXIS		= CAM_STOP_DUTY;
	Y_AXIS		= CAM_STOP_DUTY;
	
}

void setDIR_serial(char input)                    //And speed atm
{
	char tosend ='A';                            //Place holder

	if(input=='w')
	{                                            //Crck Scrw L

		Y_AXIS = FWD

	}

	else if (input=='s')
	{                                            //Crck Scrw R

		Y_AXIS = RVS

	}

	else if (input == 'a')
	{
		X_AXIS = FWD
	}

	else if (input == 'd')
	{
		X_AXIS = RVS
	}



	else
	{

		X_AXIS = CAM_STOP_DUTY
		Y_AXIS = CAM_STOP_DUTY
	}

	serial_transmit(tosend);                //Send State to serial
	serial_transmit('\n');
	serial_transmit('\r');

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
	
	OCR1A	=	CAM_STOP_DUTY;				//DriveTrain Left
	OCR1B	=	CAM_STOP_DUTY;				//DriveTrain Right
//	OCR1C	=	CAM_STOP_DUTY;				//Deposition Tilt
	
}


ISR(USART_RX_vect)
{
	uint8_t datz;

	while ( !(UCSR0A & (1<<RXC0)) );

	datz=UDR0;

	char key = datz;

	setDIR_serial(key);

	//zyn4Scoobz = zynOverDose;
}

int main(void)
{
	gpio_init();
	//Prox_ISR_EN();
	init_DIR();
	
	uart_init();
	serial_transmit('i');
	serial_transmit('n');
	serial_transmit('i');
	serial_transmit('t');

	serial_transmit('\n');
	
	timer1_init();

	
	sei();
	
	while (1)
	{
		
	}
	
}





