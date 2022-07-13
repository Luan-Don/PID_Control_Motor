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
#define INC_SPEED			4
#define DEC_SPEED			5
#define ENCODER_CHECK_DDR 	DDRD
#define ENCODER_CHECK_PORT 	PORTD
#define ENCODER_CHECK_PIN 	PIND
#define A					0		// Interrupt 0 on PD0 to check direction of motor, SCL pin
#define B					1		// Pin B is to compare with A, if B = 1 motor rotates clockwise, otherwise, SDA pin
#define samplingTime 		500	 	// 500 milliseconds for 1 sample
#define maxPwmPeriod		7200 	// 7372 cycles = 1s, F_CPU = 7372800UL

//USART
// Define baud rate
#define USART0_BAUD         115200ul
#define USART0_UBBR_VALUE   ((F_CPU/(USART0_BAUD<<4))-1)

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
void init_TIMER();
void init_IO();
void USART0_vInit(void);
void USART0_vSendByte(uint8_t u8Data);
void USART0_vSendStr(uint8_t *txBuffer, uint8_t bufferLength);
void USART0_Flush(void);

int intSpeed = 0;
uint8_t charSpeed[2] = {0};
int main(void) {
	init_LCD();
	init_IO();
	init_TIMER();
	sei(); // Global interrupt
	while (1) {
		clr_LCD();
		move_LCD(1, 1);
		printf_LCD("Speed: %.2f RPS", currentSpeed);
		move_LCD(2, 1);
		printf_LCD("Control: %0.2f", controlSpeed);
		intSpeed = currentSpeed * 60;
//		if(direction){
//			printf_LCD("Clocwise rotation");
//		}else{
//			printf_LCD("Reverse rotation");
//		}
		itoa(intSpeed, charSpeed, 10);
		USART0_vSendByte('S');
		USART0_vSendByte('p');
		USART0_vSendByte('e');
		USART0_vSendByte('e');
		USART0_vSendByte('d');
		USART0_vSendByte(':');
		USART0_vSendStr(&charSpeed, 3);
		USART0_vSendByte('R');
		USART0_vSendByte('P');
		USART0_vSendByte('M');
		USART0_vSendByte('\n');
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
ISR(INT4_vect){
	controlSpeed = controlSpeed + 0.1;
}
ISR(INT5_vect){
	controlSpeed = controlSpeed - 0.1;
	if(controlSpeed < 0){
			controlSpeed = 0;
		}
}
void MotorSpeedPID(float desiredSpeed) {
	currentSpeed = pulse * 1000 / samplingTime / 135.0; // calculate recent speed
	error = (desiredSpeed - currentSpeed) * 60; // calculate error
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
	OCR3A = PID;	// PWM pulse
	preError = error;
}
void init_TIMER() {
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
	MOTOR_PWM_DDR |= (0 << INC_SPEED) | (0 << DEC_SPEED);
	MOTOR_PWM_PORT |= (1 << INC_SPEED) | (1 << DEC_SPEED);

	ENCODER_CHECK_DDR &= ~((1 << A) | (1 << B));
	ENCODER_CHECK_PORT |= (1 << A) | (1 << B);
	EICRA |= (1 << ISC01); 							// Falling edge interrupt
	EICRB |= (1 << ISC41) | ( 1 <<ISC51);
	EIMSK |= (1 << INT0)|(1 << INT4)|(1 << INT5); 	// Enable interrupt 0 at A, PE4, PE5
}
void USART0_vInit(void)
{
    // Set baud rate
    UBRR0H = (uint8_t)(USART0_UBBR_VALUE>>8);
    UBRR0L = (uint8_t)USART0_UBBR_VALUE;

    // Set frame format to 8 data bits, no parity, 1 stop bit
    UCSR0C = (0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);

    // Enable receiver and transmitter
    UCSR0B = (1<<RXEN)|(1<<TXEN);
}
void USART0_vSendByte(uint8_t u8Data)
{
    // Wait if a byte is being transmitted
    while((UCSR0A&(1<<UDRE0)) == 0) //
    {
        ;
    }

    // Transmit data
    UDR0 = u8Data;
}

void USART0_vSendStr(uint8_t *txBuffer, uint8_t bufferLength)
{
	for (int i = 0; i < bufferLength; i++) {
		USART0_vSendByte(txBuffer[i]);
	}
	USART0_Flush();
}


void USART0_Flush(void)
{
	unsigned char dummy;
	while (UCSR0A & (1 << RXC)) {
		dummy = UDR0;
	}
}
