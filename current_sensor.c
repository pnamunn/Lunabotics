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
//#define BAUD 115200							ignore
//#define BAUDRATE ((F_CPU)/(BAUD*16UL))-1		ignore
#define Vcc 5.0
volatile uint16_t voltage = 0;
volatile uint16_t current = 0;
//unsigned char hello[] = "hello";
unsigned char newline[] = "\n";
unsigned char carriage[] = "\r";
unsigned char mA[] = " mA";

void ADC_init() {
	ADCSRA |= (1<<ADEN);	// enable ADC
	ADCSRA |= (1<<ADATE);	// enable ADC auto trigger interrupt
	ADCSRA |= (1<<ADIE);	// enable ADC conversion complete interrupt
	ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); // prescale 128 
	ADMUX |= (1<<REFS0);	// AVcc is our reference voltage
	ADMUX |= (1<<MUX1);		// ADC2, PC2
	ADCSRA |= (1<<ADSC);	// start first conversion
}

void serial_transmit(unsigned char data[]) {
	int i = 0;
	while (data[i] != '\0') {
		while (!(UCSR0A & (1<<UDRE0)));
		UDR0 = data[i];
		i++;
	}

}


ISR (ADC_vect) {	
static uint8_t ADC_low = ADCL;
static uint8_t ADC_high = ADCH;
static uint16_t ADC_full = 0;
ADC_full = (ADC_high << 8) | ADC_low;

voltage = (Vcc*(ADC_full/1024.0) - Vcc/1.99)*100000;	// multiply by big number to get reading
current = (voltage / 4.0);								// supposed to be 0.04 but voltage is already big
static unsigned char msg[] = "";; 
itoa(ADC_full, msg, 10);	// convert ADC_full to char[], decimal
serial_transmit(msg);		// transmit ADC_full
serial_transmit(newline);
_delay_ms(500);
}


int main(void)
{
    UBRR0L = 8;   					// for baud rate of 115200		
	UCSR0B |= (1<<TXEN0);			// enable transmitter, 8 bit size is default
	UCSR0B |= (1<<RXEN0);			// enable receiver
	DDRC &= ~(1<<2);	// Input PC2, ADC2 (for uno)
	//DDRF &= ~(1<<0);	// Input PF0, ADC0 (for mega)
    sei();				// enable interrupts
	ADC_init();
    while (1) 
    {

    }
}




