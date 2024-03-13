/*
 * Fuck this project.c
 *
 * Created: 3/6/2024 9:59:39 PM
 * Author : Main
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_MSG_LENGTH					5		// max # of words in a msg
#define CMD_BYTE						0


volatile struct message heard_msg;		// creates a message struct instance
volatile uint8_t TEMP=0;


typedef struct message						// struct holding variables related to messages (a data Tx)
{
	volatile uint8_t werd_count;
	volatile uint8_t data[MAX_MSG_LENGTH];						//array to store all the words in a message
} message;




//-----UART FUNCTIONS-------------------------------------//

void uart_init (void)						//initialize UART
{
	cli();
	
	PRR0 &= ~(1<<PRUSART0);
	PRR1 &= ~(1<<PRUSART1);
	PRR1 &= ~(1<<PRUSART2);
	PRR1 &= ~(1<<PRUSART3);
	
	UBRR0H = 0;
	UBRR0L = 16;								//BAUD 500000
	
	UCSR0A &=  ~(1<<	DOR0)
				& ~(1<<UPE0);
	
	UCSR0A	|=	(1<<	RXC0);
	
	UCSR0B	|=	(1<<	RXEN0)
			|	(1<<	TXEN0)
			|	(1<<	RXCIE0);			//EN: Interrupt
	//EN: receiver/transmitter
	// Rx must be initialized first!
	
	UCSR0C	|=	(1<<	UCSZ00)
			|	(1<<	UCSZ01);			//8-bit char size
	
	//UCSR0A	&=	~(1<<U2X0);
	UCSR0A	|=	(1<<U2X0);					//p219 ref
	
	
	sei();

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


void MSG_handler ()		// points to the addr of a message struct
{
	//check_sum()&data[CHK_SUM]); here					//best be a good reason
	
	serial_transmit('\n');
	serial_transmit('\r');
	
	serial_transmit('m');
	serial_transmit('s');
	serial_transmit('g');
	serial_transmit(':');
	serial_transmit(' ');
	
	uint8_t msg_type = heard_msg.data[0];		// gets msg_type from the message's index 0
	//msg_type = '2';

	switch(msg_type)				// decodes the message, based on what type of message it is
	{
		
		case '0':											// message was a kill command
		//TODO handle_kill();									//Destroy the Child
		break;
		
		
		case '1':										// message was for buttons
		//TODO handle_butts();									//handle them hammy's
		// send them where they need to go
		
		// heard_msg.werd_count = 3;	// addr werd + 2 data werds
		
		serial_transmit('\t');
		serial_transmit('\t');
		
		serial_transmit('b');
		serial_transmit('u');
		serial_transmit('t');
		serial_transmit('t');

		serial_transmit('\n');
		serial_transmit('\r');
		

		//idk- def enough room for both
		//jack's on that door.
		// throw data[4] out the door

		// TODO signal_linear_actuators()		now that we have the values, make them control the linear actuators
		break;
		
		
		case '2':								// message was for the left joystick
		// heard_msg.werd_count = 5;	// addr werd + 4 data werds
		
		serial_transmit('j');
		serial_transmit('o');
		serial_transmit('y');

		serial_transmit('\t');
		
		// sets duty cycles of the left & right motors
		//	DRIVE_L = (heard_msg.data[1] << 8) | heard_msg.data[2];
		//DRIVE_R = (heard_msg.data[3] << 8) | heard_msg.data[4];

		for (uint8_t i = 0; i < 5; i++)
		{
			printBin8(heard_msg.data[i]);
			serial_transmit(' ');
			
		}
		
		serial_transmit('\n');
		serial_transmit('\r');
		
		//if (DRIVE_L == 3000 && DRIVE_R == 3000)
		////if (heard_msg.data[1] == '0' && heard_msg.data[2] == '0' && heard_msg.data[3] == '0' && heard_msg.data[4] == '0')
		//{
		//serial_transmit('d');
		//serial_transmit('e');
		//serial_transmit('a');
		//serial_transmit('d');
		//
		//}
		//else
		//{
		//printBin8(heard_msg.data[0]);
		//serial_transmit(' ');
		//printBin8(heard_msg.data[1]);
		//serial_transmit(' ');
		//printBin8(heard_msg.data[2]);
		//serial_transmit(' ');
		//printBin8(heard_msg.data[3]);
		//serial_transmit(' ');
		//printBin8(heard_msg.data[4]);
		//
		//serial_transmit(' ');
		//
		//serial_transmit(heard_msg.data[0]);
		//serial_transmit(heard_msg.data[1]);
		//serial_transmit(heard_msg.data[2]);
		//serial_transmit(heard_msg.data[3]);
		//serial_transmit(heard_msg.data[4]);
		//serial_transmit('\n');
		//serial_transmit('\r');
		//
		//
		//}
		
		break;
		
		
		default:											// if the msg_type is not a recognizable value
		//didnt_hear(boo_hoo);								// message is bunk, dump it, we're doing connectionless Tx
		serial_transmit('\t');
		serial_transmit('\t');
		
		serial_transmit('d');
		serial_transmit('f');
		serial_transmit('l');
		serial_transmit('t');
		
		serial_transmit('\n');
		serial_transmit('\r');

		break;
	}

	heard_msg.data[0] = 0;		// reset vals to 0
	heard_msg.data[1] = 0;
	heard_msg.data[2] = 0;
	heard_msg.data[3] = 0;
	heard_msg.data[4] = 0;

}



ISR (USART0_RX_vect)
{
	cli();									//disable interrupts
	
	serial_transmit('W');
	serial_transmit('H');
	serial_transmit('Y');
	serial_transmit('\n');
	
	if (heard_msg.werd_count < (MAX_MSG_LENGTH-1))
	{
		serial_transmit('\n');
		serial_transmit('\r');
		
		//while ( !(UCSR0A & (1<<RXC0)) );
		heard_msg.data[heard_msg.werd_count] = UDR0;
		
		//printBin8(UDR0);
		serial_transmit(heard_msg.werd_count + '0');
		//serial_transmit(' ');
		//term_Send_Val_as_Digits(heard_msg.werd_count);
		serial_transmit(' ');
		
		printBin8(heard_msg.data[heard_msg.werd_count] - '0');
		serial_transmit(' ');
		printBin8(UDR0);
		serial_transmit(' ');

		//printBin8(UDR0);
		serial_transmit(heard_msg.data[heard_msg.werd_count]);
		//printBin8(UDR0);
		
		serial_transmit(' ');
		printBin8(UCSR0A);
		serial_transmit(' ');
		printBin8(UCSR0B);
		serial_transmit(' ');
		printBin8(UCSR0C);
		
		//while(UCSR0A&(1<<RXC0)){ TEMP = UDR0;}
				
		serial_transmit(' ');
		//printBin8(UCSR0A);
		serial_transmit(' ');
		
		heard_msg.werd_count++;
	}
	
	else      // getting last data byte & jumping to msg_handler
	{
		serial_transmit('\n');
		serial_transmit('\r');
		
		//while ( !(UCSR0A & (1<<RXC0)) );
		heard_msg.data[heard_msg.werd_count] = UDR0;
		//delay();		// delay so Jetson has time to start ser.read() listening on serial port
		serial_transmit(heard_msg.werd_count + '0');
		serial_transmit(' ');
		printBin8(heard_msg.data[heard_msg.werd_count]);
		serial_transmit(' ');
		serial_transmit(heard_msg.data[heard_msg.werd_count]);
		
		serial_transmit('\n');
		serial_transmit('\r');
		
		MSG_handler(&heard_msg);
		heard_msg.werd_count = 0;		// reset count

	}
	
	sei();
}




int main(void)
{
	
	heard_msg.werd_count = 0;
	cli();
	
	uart_init();
		
	sei();
    /* Replace with your application code */
    while (1) 
    {
    }
}

