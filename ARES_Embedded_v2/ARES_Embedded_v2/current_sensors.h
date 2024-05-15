/*
 * Current sensors' code
 * Author:  Anabel Sanchez
 */ 

#ifndef	CURRENTSENSORS_h
#define CURRENTSENSORS_h


#define F_CPU 16000000UL   	 // 16MHz
#include <util/delay.h>
#include <stdbool.h>

#include "serialTxfunctions.h"


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


typedef struct current_alert
{
	volatile bool flag;		// 1 if a threshold was reached
	volatile uint8_t motor;		// which motor
	volatile uint8_t sign;		// + or - sign
	volatile float data;
} current_alert;


volatile struct current_alert current_alert_L = {0, 'L'};		// Left drivetrain current sensor
//current_alert_L.motor = 'L';

volatile struct current_alert current_alert_R = {0, 'R'};		// Right drivetrain current sensor
//current_alert_R.motor = 2;




void sensor_ADC_init() {
	ADCSRA |= (1<<ADEN);	// enable ADC
							// ADATE bit is initialized to 0 for manual conversion
							// manually trigger using ADSC
	ADCSRA |= (1<<ADIE);	// enable ADC conversion complete interrupt
	ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); // prescale 128
	ADMUX |= (1<<REFS0);	// AVcc is our reference voltage
	ADMUX |= (1<<MUX0);		// ADC1, PF1
	//ADCSRA |= (1<<ADSC);	// start first conversion; as long as this is 1, conversions will keep happening
}


//grabs ADC value from registers, finds the rolling average ADC value, normalizes the value,
//then checks that value against the set thresholds, if within thresholds, flags & stores data

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
		ADC_full = sum1 / arr_size;		// rolling average found
		
		voltage = (Vcc / 1024.0)*ADC_full;		// normalize
		voltage = fabs(voltage - (Vcc / 2.0));
		current = voltage*9.752;	// 1/0.11 * 1000 --> new sensitivity with amplifier


		if (current > 22.3) {		// Thresholds
			current_alert_L.flag = 1;
			
			if (ADC_full < 512) {
				current_alert_L.sign = '-';
			}
			else {
				current_alert_L.sign = '+';
			}
			
			current_alert_L.data = current;
			
			
			//serial_transmit('M');
			//serial_transmit('1');
			//serial_transmit(':');
			//
			//if (ADC_full < 512) {
				//serial_transmit('-');
			//}
			//else {
				//serial_transmit('+');
			//}
			//
			//printFloat(current);
			//_delay_ms(150);
			//serial_transmit('?');
		}
		
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


		if (current > 22.3) {		// Thresholds
			current_alert_R.flag = 1;
			
			if (ADC_full < 512) {
				current_alert_R.sign = '-';
			}
			else {
				current_alert_R.sign = '+';
			}
			
			current_alert_R.data = current;
			
			
			//serial_transmit('M');
			//serial_transmit('2');
			//serial_transmit(':');
			//
			//if (ADC_full < 512) {
				//serial_transmit('-');
			//}
			//else {
				//serial_transmit('+');
			//}
			//
			//printFloat(current);
			//_delay_ms(150);
			//serial_transmit('?');
		}
		
		ADMUX = 0b01000001;		// switch back to ADC1
	}
	else {
		ADMUX = 0b01000001;		// else set to ADC1
	}
}


#endif