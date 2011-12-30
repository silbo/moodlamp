#ifndef Main_h
#define Main_h

// define mcu freuquency
#ifndef F_CPU
#define F_CPU 12000000L
#endif

// add libs
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>

// define interrupts
/*
#define ADC_ISR 1
#define TIMER0_OVR_ISR 1
#define TIMER0_COMP_ISR 1
*/

#define TRUE 1
#define FALSE 0

#define sbi(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define cbi(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))

// set default values for registers
void init();
// SPI send byte
unsigned char spi_transfer( unsigned char );
// discrete hartley transform, to transform signal into frequency spectrum
int discreteHartley( int, float*, float* );
// the main function
int main( void );

// counter
long counter;

#endif
