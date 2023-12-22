

/*

//-----LOGITECH CONTROLLER ADDY-----//

#define JOY_L_X				0		//Axis start
#define	JOY_L_Y				1
#define	LEFT_TRIG			2	
#define	JOY_R_X				3
#define	JOY_R_Y				4
#define	RIGHT_TRIG			5
#define	D-PAD_X				6
#define	D-PAD_y				7		//Axis End
#define BUTTON_A			8		//Button Start
#define	BUTTON_B			9
#define	BUTTON_X			10
#define BUTTON_Y			11
#define BUMPER_L			12
#define BUMPER_	R			13
#define	BUTTON_BACK			14
#define	BUTTON_START		15
#define	BUTTON_LOGI			16
#define	BUTTON_LT3			17
#define	BUTTON_RT3			18		//Button End
#define	ACKN_RQST			19

volatile int16t Log_F710[20];
									
									
#define DACCEL_STATE			0	//MPU-6050 Read
#define	DIRECT_STATE			1	//Concept of Direction
#define OBSTCL_STATE			2	//IR Sense Trig Loc
#define	I_LOAD_STATE			3	//ACS758 read
#define D_LOAD_STATE			4	//HX711	read
#define	E_DPLY_STATE			5	//Contact Sensor rdy
#define	D_DPLY_STATE			6	//Contact Sensor rdy
#define SYSPWR_STATE			7	//Report Power Level

volatile int16t Log_F710[8];



010 : 0000 1010 : hx0A : ox12
020 : 0001 0100 : hx14 : ox24
030 : 0001 1110 : hx1E : ox36
040 : 0010 1000 : hx28 : ox50
050 : 0011 0010 : hx32 : ox62
060 : 0011 1100 : hx3c : ox74
070 : 0100 0110 : hx46 : ox106
080 : 0101 0000 : hx50 : ox120
090 : 0101 1010 : hx5A : ox132
100 : 0110 0100 : hx64 : ox144
		 
i_state =("percentile">>3);  
					  operand=state>>2;
									v_state = i_state-operand;
									
i_state	  Precentile  operand		v_state
		 
1		  00001			0			1-0 = 1
2		  00010			0			2-0 = 2
3		  00011			0			3-0 = 3
5		  00101         1			5-1 = 4
6		  00110			1			6-1 = 5
7		  00111			1			7-1 = 6
8		  01000			2			8-2 = 6 //breaks here
10		  01010
11		  01011
12		  01100



void printPercentileState(uint8_t percentile) {
	const char* state;

	switch (percentile / 10) { // Dividing by 10 to map the value to a range 0-10
		case 0:  
		state = "Empty"; 
		BIN_CAP_HIST =0;
		break;
		
		case 1: 
		state = "10%"; 
		BIN_CAP_HIST =1;
		break;
		
		case 2:
		state = "20%";
		BIN_CAP_HIST =2;
		break;
		
		case 3:  
		state = "30%";
		BIN_CAP_HIST =3;
		break;
		
		case 4:  
		state = "40%";
		BIN_CAP_HIST =4;
		break;
		
		case 5:  
		state = "50%";
		BIN_CAP_HIST =0;   
		break;
		
		case 6:
		state = "60%";
		   break;
		case 7:  state = "70%";   break;
		case 8:  state = "80%";   break;
		case 9:  state = "90%";   break;
		default: state = "Full";  break; // Assuming 100% is Full
	}

	// Now print the state using your UART functions
	// Assuming serial_transmit can handle strings.
	// If not, you'd send the string character by character.
	for (const char* p = state; *p != '\0'; p++) {
		serial_transmit(*p);
	}
	serial_transmit('\n');
	serial_transmit('\r');
}

// Usage example:
printPercentileState(25); // This would print "20%"



*/

#include <avr/io.h>
#include <avr/interrupt.h>

//#define DEP_BIN_CAPACITY	50000

#define HX711_DATA_LINE		   PINB4
#define DOUT_PULL_UP		( PORTB|=	  (1<<PINB4) )
#define HX711_DATA_RDY	    (!(PINB  &	  (1<<PINB4) )     )
#define HX711_DAT_PENDING 	  (PINB  &	  (1<<PINB4) ) 
#define HX711_DATA_VALUE    ( (PINB  &	  (1<<PINB4) ) >> 4) //load, comp, shift: 3
													   
#define SCK_LINE			   PINC0					   
#define SCK_HIGH			 (PORTC |=    (1<<PINC0) ) 
#define SCK_LOW				 (PORTC &=   ~(1<<PINC0) ) 

#define GAIN128					1
#define GAIN32					2
#define GAIN64					3

#define TIMER0_NUM_OF_OVF	(61)
#define T1B_CS_HI			( TCCR0B |= (1 <<  CS01) | (1 << CS00));
#define T1B_CS_HI_1024		( TCCR0B |= (1 <<  CS02) | (1 << CS00));
#define T1_OVF_EN			(TIMSK0 |= (1 << TOIE0));
#define Norm_Init			( TCNT0 = 0);

/*									
#define DACCEL_STATE			0	//MPU-6050 Read
#define	DIRECT_STATE			1	//Concept of Direction
#define OBSTCL_STATE			2	//IR Sense Trig Loc
#define	I_LOAD_STATE			3	//ACS758 read
#define D_LOAD_STATE			4	//HX711	read
#define	E_DPLY_STATE			5	//Contact Sensor rdy
#define	D_DPLY_STATE			6	//Contact Sensor rdy
#define SYSPWR_STATE			7	//Report Power Level
*/


volatile uint32_t	hx711_data32	= 0;

//volatile uint8_t	powerUpInitFlag = 0;
//volatile uint8_t	ovf_count		= 0;
//volatile uint16_t	hx711_data16	= 0;
//volatile uint16_t	mass_grams		= 0;
//volatile uint8_t	hx711_data16_H	= 0;
//volatile uint8_t	hx711_data16_L	= 0;
//volatile uint8_t	New_data_flag	= 1;
//volatile uint8_t	bin_precentile	= 0;
/*
volatile uint16_t DataLog[8] = { powerUpInitFlag, 
								  ovf_count,
								  mass_grams,
								  New_data_flag,
								  hx711_data16_t,
								  bin_precentile,
								  0,
								  0				  };
								  
								  */

/*
uint16_t LoadGrams(uint16_t *mass_binary)
{
	return	uint16_t mass_grams = ((*mass_binary)*DEP_BIN_CAPACITY)>>16;
}

void DepositionLevel (uint8_t *mass_observed)
{
	const char* state;
	uint8_t process = (10*hx711_data16_t)>>16;

	switch (process / 10) 
	{ 
		case 0:
		state = "Empty";
		DataLog[D_LOAD_STATE] =0;
		break;
		
		case 1:
		state = "10%";
		DataLog[D_LOAD_STATE] =1;
		break;
		
		case 2:
		state = "20%";
		DataLog[D_LOAD_STATE] =2;
		break;
		
		case 3:
		state = "30%";
		DataLog[D_LOAD_STATE] =3;
		break;
		
		case 4:
		state = "40%";
		DataLog[D_LOAD_STATE] =4;
		break;
		
		case 5:
		state = "50%";
		DataLog[D_LOAD_STATE] =5;
		break;
		
		case 6:
		state = "60%";
		DataLog[D_LOAD_STATE] =6;
		break;
		
		case 7:  
		state = "70%"; 
		DataLog[D_LOAD_STATE] =7;  
		break;
		
		case 8:  
		state = "80%";
		DataLog[D_LOAD_STATE] =8;
		break;
		
		case 9:  
		state = "90%";
		DataLog[D_LOAD_STATE] =9;   
		break;
		
		default: 
		state = "Full";
		DataLog[D_LOAD_STATE] =10;
		  break;
	
	}
}
*/
/*
void timer0_start_one_shot()
{
	T1B_CS_HI_1024		//norm
	T1_OVF_EN			//ISR on ~1.024ms
	Norm_Init			//normz
}
*/

void gpio_init()
{
	DDRC |= (1<<0);
	DDRB &= ~(1<<4);
	DOUT_PULL_UP;
	SCK_LOW;
}

//** UART FUNCTIONS **//
void uart_init (void)							// function to initialize UART
{

	//UBRR0H = (BAUDRATE>>8);					//shift the register right by 8 bits
	UBRR0L = 8;									
	
	UCSR0B	|=	(1<<	TXEN0)
			|	(1<<	RXEN0);					// enable receiver and transmitter
//			|	(1<<	RXCIE0);				
	
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


void use_HX711() 
{
	//static uint8_t sensor_IP =1;		

			for (uint8_t i = 0; i < 24; i++)		//ACQUISITION
			{
			
			SCK_HIGH;
				
			
				asm volatile(" nop");				//62.5 nano seconds
				asm volatile(" nop");
				asm volatile(" nop");
				asm volatile(" nop");				//250th nano second
	
				
			hx711_data32   |= HX711_DATA_VALUE;		//3 ops to read
			hx711_data32 <<= 1;						//4 Scooch for nxt rd
			
			SCK_LOW;
			
				asm volatile(" nop");				//62.5 nano seconds
				asm volatile(" nop");
				asm volatile(" nop");
				asm volatile(" nop");				//250th nano second	
			}
			
			SCK_HIGH;								//for Gain 128
				asm volatile(" nop");				//62.5 nano seconds
				asm volatile(" nop");
				asm volatile(" nop");
				asm volatile(" nop");				//250th nano second
			
			hx711_data32 >>= 1;					//scooch back to compensate
			
			SCK_LOW;
			
				asm volatile(" nop");				//62.5 nano seconds
				asm volatile(" nop");
				asm volatile(" nop");
				asm volatile(" nop");				//250th nano second



			//sensor_IP = 0;

			
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
//**end of joseph uart functions**//

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

void term_Send_16_as_Digits(uint16_t val){
	uint8_t digit = '0';
	
	if(val&(1<<15)){
		serial_transmit('-');
		val=~val;
		val+=1;
	}
	
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



int main()
{
	sei();
	gpio_init();
	uart_init();
	//timer0_start_one_shot();
		
	//while (!powerUpInitFlag) {	SCK_LOW;}	//peripheral device passive Power
	
	while(1)
	{
		if(HX711_DATA_RDY)
		{
	
		use_HX711();
		/*
			uint8_t *peetee = (uint8_t *)&hx711_data32;
			//term_Send_Val_as_Digits((uint8_t)(hx711_data>>24));
			//term_Send_Val_as_Digits((uint8_t)(hx711_data>>16));
			//term_Send_Val_as_Digits((uint8_t)(hx711_data>> 8));
			//term_Send_Val_as_Digits((uint8_t)(hx711_data>> 0));
			printBin8(peetee[3]);				//little-endian: LSB 1st
			printBin8(peetee[2]);
			printBin8(peetee[1]);
			printBin8(peetee[0]);
			
*/			
			//hx711_data32 -= 168;
			hx711_data32 |= 0xFF000000;		//sign extnd?
			hx711_data32 = ~hx711_data32;	//un-do complement
			hx711_data32 +=1;				//add 1
			
			
			uint8_t *weee = (((uint8_t *)(&hx711_data32))+ 1u);	//window trickery
			term_Send_16_as_Digits(*((uint16_t *)weee));			//window extension
			
			
			serial_transmit('\n');
			serial_transmit('\r');
			hx711_data32 = 0;
		}
	}
	
	
 }
 


/*
ISR(TIMER0_OVF_vect) {
	ovf_count++;
	if (ovf_count >= TIMER0_NUM_OF_OVF) {
		powerUpInitFlag = 1; // Set power-up init flag
		TIMSK0 &= ~(1 << TOIE0); // Disable Timer0 overflow interrupt
		TCCR0B = 0; // Stop Timer0
		ovf_count = 0; // Reset overflow count
	}
}

*/

/*


ISR(TIMER0_OVF_vect)			//Delay for HX711 Power Up
{
	powerUpInitFlag = 1;		//Set PWR Init flag
	TIMSK0 &= ~(1 << TOIE0);	//Disable Timer0 overflow interrupt
	TCCR0B = 0;					//Stop Timer0
}



void use_HX711()
{
	static uint8_t sensor_IP =1;
	
	
	static enum HX711_State {STANDBY, ACTIVE, SEND} hx711_state = STANDBY;


	while(sensor_IP)
	{
		
		switch(hx711_state)
		{
			
			case STANDBY:
			
			SCK_LOW;
			hx711_state = ACTIVE;
			break;
			
			
			case ACTIVE:
			for (uint8_t i = 0; i < 24; i++)		//ACQUISITION
			{
				
				SCK_HIGH;
				
				
				asm volatile(" nop");				//62.5 nano seconds
				asm volatile(" nop");
				asm volatile(" nop");
				asm volatile(" nop");				//250th nano second
				
				
				hx711_data   = HX711_DATA_VALUE;		//3 ops to read
				hx711_data <<= 1;						//4 Scooch for nxt rd
				
				SCK_LOW;
				
				asm volatile(" nop");				//62.5 nano seconds
				asm volatile(" nop");
				asm volatile(" nop");
				asm volatile(" nop");				//250th nano second
			}
			
			for (uint8_t i = 0; i < GAIN128; i++)	//Gain Train ~7tick
			{
				
				SCK_HIGH;
				asm volatile(" nop");				//62.5 nano seconds
				asm volatile(" nop");
				asm volatile(" nop");
				asm volatile(" nop");				//250th nano second
				
				SCK_LOW;
				
				asm volatile(" nop");				//62.5 nano seconds
				asm volatile(" nop");
				asm volatile(" nop");
				asm volatile(" nop");				//250th nano second
				
			}
			hx711_state = SEND;
			break;
			
			
			case SEND:

			SCK_LOW;
			hx711_state=STANDBY;
			sensor_IP = 0;
			break;
		}
	}

}
*/