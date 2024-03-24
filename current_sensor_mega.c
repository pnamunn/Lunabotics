/*
 * current_sensor_mega.c
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
#define Vcc 4.872
#define arr_size 10
float voltage = 0.0;
float current = 0.0;
float sum1 = 0.0;
float sum2 = 0.0;
volatile int i = 0;
volatile int j = 0;
volatile uint16_t array1[arr_size] = {0};
volatile uint16_t array2[arr_size] = {0};

void ADC_init() {
	ADCSRA |= (1<<ADEN);	// enable ADC
	ADCSRA |= (1<<ADATE);	// enable ADC auto trigger interrupt
	ADCSRA |= (1<<ADIE);	// enable ADC conversion complete interrupt
	ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); // prescale 128 
	ADMUX |= (1<<REFS0);	// AVcc is our reference voltage
	ADMUX |= (1<<MUX0);		// ADC1, PF1
	ADCSRA |= (1<<ADSC);	// start first conversion
}

void UART1_init() {
	UBRR1L = 8;   				// for baud rate of 115200
	UCSR1B |= (1<<TXEN1);		// enable transmitter, 8 bit size is default
	UCSR1B |= (1<<RXEN1);		// enable receiver
}

void serial_transmit(uint8_t data) {
	while (!(UCSR1A & (1<<UDRE1)));
	UDR1 = data;
}

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
	
	if ((ADMUX & 0x0F) == 1) {
		sum1 = sum1 - array1[i] + ADC_full;
		array1[i] = ADC_full;
		i = (i+1) % arr_size;
		ADC_full = sum1 / arr_size;
		
		voltage = (Vcc / 1024.0)*ADC_full;
		voltage = fabs(voltage - (Vcc / 2.0));
		current = voltage*9.752;	// 1/0.11 * 1000 --> new sensitivity with amplifier
		
		serial_transmit('M');
		serial_transmit('1');
		serial_transmit(':');
		
		if (ADC_full < 512) {
			serial_transmit('-');
		}
		
		printFloat(current);
		serial_transmit(' ');
		serial_transmit('A');
		_delay_ms(150);
		serial_transmit('\n');
		
		
		ADMUX = 0b01000010;		// switch to ADC2
	}
	else if ((ADMUX & 0x0F) == 2) {
		sum2 = sum2 - array2[i] + ADC_full;
		array2[i] = ADC_full;
		j = (j+1) % arr_size;
		ADC_full = sum2 / arr_size;
		
		voltage = (Vcc / 1024.0)*ADC_full;
		voltage = fabs(voltage - (Vcc / 2.0));
		current = voltage*9.752;	
		
		serial_transmit('M');
		serial_transmit('2');
		serial_transmit(':');
		
		if (ADC_full < 512) {
			serial_transmit('-');
		}
		
		printFloat(current);
		serial_transmit(' ');
		serial_transmit('A');
		_delay_ms(150);
		serial_transmit('\n');
		
		ADMUX = 0b01000001;		// switch back to ADC1
	}
	else {
		ADMUX = 0b01000001;		// else set to ADC1
	}	
}
	

int main(void)
{
	sei();				// enable interrupts
	
	UART1_init();
	ADC_init();
	DDRF &= ~(1<<1); //  Input PF1, ADC1 (for mega)
	
	ADCSRA |= (1<<ADSC);	// start first conversion
    while (1) 
    {	
    }
}
