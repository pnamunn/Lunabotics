/*
 * current_sensor.c
 *
 * Created: 2/12/2024 2:10:36 PM
 * Author : 
 */ 
#define F_CPU 16000000UL   	 // 16MHz
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//#define BAUD 115200					ignore
//#define BAUDRATE ((F_CPU)/(BAUD*16UL))-1		ignore
#define Vcc 5.0
float voltage = 0.0;
float current = 0.0;
float sum = 0.0;
volatile int i = 0;
volatile float array[5] = {0.0,0.0,0.0,0.0,0.0};

void ADC_init() {
	ADCSRA |= (1<<ADEN);	// enable ADC
	ADCSRA |= (1<<ADATE);	// enable ADC auto trigger interrupt
	ADCSRA |= (1<<ADIE);	// enable ADC conversion complete interrupt
	ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); // prescale 128 
	ADMUX |= (1<<REFS0);	// AVcc is our reference voltage
	ADMUX |= (1<<MUX1);		// ADC2, PC2
	ADCSRA |= (1<<ADSC);	// start first conversion
}

void UART_init() {
	UBRR0L = 8;   				// for baud rate of 115200
	UCSR0B |= (1<<TXEN0);		// enable transmitter, 8 bit size is default
	UCSR0B |= (1<<RXEN0);		// enable receiver
}

void serial_transmit(uint8_t data) {
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}
//void other_serial_transmit(unsigned char data[]) {
	//int i = 0;
    //while (data[i] != '\0') {
	    //while (!(UCSR0A & (1<<UDRE0)));   				// used for debugging, ignore
		//UDR0 = data[i];
		//i++; }
//}

// format is XXXX.XXX 
void printFloat(float num) {
	uint16_t integer_part = num;
	uint16_t decimal_part = (num - integer_part)*1000;
	uint8_t first_digit = integer_part / 1000;
	if (first_digit < 0 || first_digit > 9) {
		serial_transmit('0');
	}
	else {
		serial_transmit(first_digit + '0');
	}
	uint8_t second_digit = (integer_part - (first_digit * 1000)) / 100;
	if (second_digit < 0 || second_digit > 9) {
		serial_transmit('0');
	}
	else {
		serial_transmit(second_digit + '0');
	}
	uint8_t third_digit = (integer_part - (first_digit*1000 + second_digit*100)) / 10;
	if (third_digit < 0 || third_digit > 9) {
		serial_transmit('0');
	}
	else {
		serial_transmit(third_digit + '0');
	}
	uint8_t fourth_digit = (integer_part - (first_digit*1000 + second_digit*100 + third_digit*10));
	if (fourth_digit < 0 || fourth_digit > 9) {
		serial_transmit('0');
	}
	else {
		serial_transmit(fourth_digit + '0');
	}
	serial_transmit('.');
	uint8_t tenths = decimal_part / 100;
	if (tenths < 0 || tenths > 9) {
		serial_transmit('0');
	}
	else {
		serial_transmit(tenths + '0');
	}
	uint8_t hundreths = (decimal_part - tenths*100) / 10;
	if (hundreths < 0 || hundreths > 9) {
		serial_transmit('0');
	}
	else {
		serial_transmit(hundreths + '0');
	}
	uint8_t thousandths = (decimal_part - tenths*100 - hundreths*10);
	if (thousandths < 0 || thousandths > 9) {
		serial_transmit('0');
	}
	else {
		serial_transmit(thousandths + '0');
	}
}

ISR (ADC_vect) {	
	static uint16_t ADC_low = 0;
	static uint16_t ADC_high = 0;
	static uint16_t ADC_full = 0;
	
	ADC_low = ADCL;
	ADC_high = ADCH;
	ADC_full = (ADC_high << 8) | ADC_low;
	
	// Average past 5 ADC values to limit noise, although it updates
	// at a slightly slower rate now 
	sum = sum - array[i] + ADC_full;
	array[i] = ADC_full;
	i = (i+1) % 5;
	ADC_full = sum / 5;
	
	voltage = (Vcc / 1024.0)*ADC_full;	
	voltage = voltage - (Vcc / 2.0);
	current = voltage*25000.0;
	printFloat(current);
	serial_transmit(' ');
	serial_transmit('m');
	serial_transmit('A');
	_delay_ms(250);
	serial_transmit('\n');
}

int main(void)
{
	sei();				// enable interrupts
	
	UART_init();
	ADC_init();
	DDRC &= ~(1<<2);	// Input PC2, ADC2 (for uno)
	
	ADCSRA |= (1<<ADSC);	// start first conversion
    while (1) 
    {	
    }
}
