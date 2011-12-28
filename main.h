#ifndef Main_h
#define Main_h

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>

#ifndef F_CPU
#define F_CPU 8000000L
#endif

#define sbi(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define cbi(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))

#define interrupts() sei()
#define noInterrupts() cli()

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
//#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

// set default values for registers
void init();
// set bit on PORTB
int set_bit(int, int);
// get milliseconds
unsigned long millis();
// delay milliseconds
void delay(unsigned long);
// read ADC value
int analogRead(uint8_t);
// set PWM
void analogWrite(uint8_t, int);

// fresh ADC value
int16_t u;
int16_t x_TP;

#endif