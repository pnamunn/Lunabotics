/*
 * Serial transmit an ASCII char, an integer up to the hundred's place,
 * 8 bits in binary, of a float of format XXXX.XXX
 */ 

#ifndef	SERIALTXFUNCTIONS_h
#define SERIALTXFUNCTIONS_h


void serial_transmit (uint8_t data)		// serial Tx a byte
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


void printFloat(float num)		// format is XXXX.XXX
{
	uint16_t integer_part = num;
	uint16_t decimal_part = (num - integer_part)*1000;
	
	uint8_t first_digit = integer_part / 1000;
	if (first_digit < 0 || first_digit > 9) {		serial_transmit('0');		}
	else {											serial_transmit(first_digit + '0');		}
	
	uint8_t second_digit = (integer_part - (first_digit * 1000)) / 100;
	if (second_digit < 0 || second_digit > 9) {		serial_transmit('0');		}
	else {											serial_transmit(second_digit + '0');	}
	
	uint8_t third_digit = (integer_part - (first_digit*1000 + second_digit*100)) / 10;
	if (third_digit < 0 || third_digit > 9) {		serial_transmit('0');		}
	else {											serial_transmit(third_digit + '0');		}
	
	uint8_t fourth_digit = (integer_part - (first_digit*1000 + second_digit*100 + third_digit*10));
	if (fourth_digit < 0 || fourth_digit > 9) {		serial_transmit('0');		}
	else {											serial_transmit(fourth_digit + '0');	}
	
	serial_transmit('.');
	uint8_t tenths = decimal_part / 100;
	if (tenths < 0 || tenths > 9) {			serial_transmit('0');		}
	else {									serial_transmit(tenths + '0');	}
	
	uint8_t hundreths = (decimal_part - tenths*100) / 10;
	if (hundreths < 0 || hundreths > 9) {		serial_transmit('0');	}
	else {										serial_transmit(hundreths + '0');	}
	
	uint8_t thousandths = (decimal_part - tenths*100 - hundreths*10);
	if (thousandths < 0 || thousandths > 9) {		serial_transmit('0');	}
	else {											serial_transmit(thousandths + '0');		}
	
}


#endif