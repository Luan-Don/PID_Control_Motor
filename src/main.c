/*
 * Servo_N5.c
 *
 * Created: 5/9/2022 9:53:19 PM
 * Author : luand
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/iom128.h>
#define F_CPU 7372800UL
#include <util/delay.h>
#include "myLCD.h"

#define MOTOR_CTRL_DDR  	DDRC
#define MOTOR_CTRL_PORT 	PORTC
#define MOTOR_DIR_CLKWS		0 		// Set motor direction at pin A8
#define MOTOR_DIR_ACLKWS	1		// Enable motor at pin A9
#define MOTOR_PWM_DDR 		DDRE
#define MOTOR_PWM_PORT 		PORTE
#define PWM 				3		// PWM pulse on OC3A
#define ENCODER_CHECK_DDR 	DDRD
#define ENCODER_CHECK_PORT 	PORTD
#define ENCODER_CHECK_PIN 	PIND
#define A					0		// Interrupt 0 on PD0 to check direction of motor
#define B					1		// Pin B is to compare with A, if B = 1 motor rotates clockwise, otherwise
#define samplingTime 		500	 	// 25 milliseconds for 1 sample
#define maxPwmPeriod		7200 	// 7372 cycles = 1s, F_CPU = 7372800UL

volatile int pulse = 0;
volatile int prePulse = 0;
volatile float currentSpeed = 0;
volatile float controlSpeed = 1.5 ; 	 	// control speed is 0.5rps or 30rpm
volatile float error = 0;
volatile float preError = 0;
//volatile int diff = 0;
int Kd = 8;
int Kp = 10;
int Ki = 1;
int pPart = 0;
int dPart = 0;
int iPart = 0;
int PID = 0;
int exactPID = 0;
char direction = 0;

void MotorSpeedPID(float desiredSpeed);
float absolute(float number);
void init_TIMER();
void init_IO();

int main(void) {
	init_LCD();
	init_IO();
	init_TIMER();
	sei(); // Global interrupt
	while (1) {
		clr_LCD();
		move_LCD(1, 1);
		//printf_LCD("Pulse %d", prePulse);
		printf_LCD("Speed: %.2f RPS", currentSpeed);
		move_LCD(2, 1);
		//printf_LCD("Error: %d", error);
		//printf_LCD("Pulse %d", pulse);
		//printf_LCD("Diff %d", diff);
		printf_LCD("PWM: %d", PID);
//		if(direction){
//			printf_LCD("Clocwise rotation");
//		}else{
//			printf_LCD("Reverse rotation");
//		}
		_delay_ms(300);
	}
}

ISR(TIMER1_OVF_vect) { //update sampling time
	TCNT1 = 61936;
	MotorSpeedPID(controlSpeed);
	pulse = 0;
}

ISR(INT0_vect) {
	pulse++;
	if (bit_is_set(ENCODER_CHECK_PIN, B)) {
		direction = 0;
	} else {
		direction = 1;
	}
}

void MotorSpeedPID(float desiredSpeed) {
	currentSpeed = pulse * 1000 / samplingTime / 135.0; // calculate recent speed
	error = (desiredSpeed - currentSpeed) * 60; // calculate error
	//diff = error - preError;
	// PID part
	pPart = Kp * error;
	dPart = Kd * (error - preError) * 1000 / samplingTime;
	iPart += Ki * error * samplingTime / 1000;
	PID += pPart + dPart + iPart;
	if (PID > maxPwmPeriod) {
		PID = maxPwmPeriod;
	}
	if (PID < 0) {
		PID = 1;
	}
//	if(currentSpeed == desiredSpeed){
//		exactPID = PID;
//		OCR3A = exactPID;
//	}
//	else{
		OCR3A = PID;
	//}

	//prePulse = pulse;
	preError = error;
}
float absolute(float number) {
	if (number < 0) {
		number = -number;
	}
	return number;
}
void init_TIMER() {
//	// Timer 0 delay
//	TCCR0 |= (1 << CS02) | (1 << CS01) | (1 << CS00); // F_Timer0 = F_CPU/1024 = 7200UL
//	TCNT0 = 76; // TCNT0 value = 256 - 7200*25/1000 = 76 for 25ms sampling time
//	TIMSK |= (1 << TOIE0);
	// Timer 1 delay
	TCCR1B |= (1<<CS12)|(1<<CS10);
	TCNT1 = 100; // TCNT1 value = 65536 - 7200*samplingTime/1000 = 61936
	TIMSK |= (1<<TOIE1);

	// Timer 3 PWM pulse
	TCCR3A |= (1 << COM3A1) | (1 << WGM31); 	// Mode Fast PWM 14
	TCCR3B |= (1 << WGM33) | (1 << WGM32) | (1 << CS30); // F_Timer3 = F_CPU
	TCNT3 = 0; 			// Reset counter
	ICR3 = maxPwmPeriod; 	// Set top value for 1 duty cycle
	OCR3A = 0; 		 	// Set duty cycle to 0%
}
void init_IO() {
	MOTOR_CTRL_DDR |= (1 << MOTOR_DIR_ACLKWS) | (1 << MOTOR_DIR_CLKWS);
	MOTOR_CTRL_PORT |= (0 << MOTOR_DIR_ACLKWS) | (1 << MOTOR_DIR_CLKWS);
	MOTOR_PWM_DDR |= (1 << PWM);
	ENCODER_CHECK_DDR &= ~((1 << A) | (1 << B));
	ENCODER_CHECK_PORT |= (1 << A) | (1 << B);
	EICRA |= (1 << ISC01); 				// Falling edge interrupt
	EIMSK |= (1 << INT0); 				// Enable interrupt 0 at A
}
