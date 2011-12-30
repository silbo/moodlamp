
#include "main.h"

// ATMEL ATTINY85
//
//                  +-\/-+
// Ain0 (D 5) PB5  1|    |8  Vcc
// Ain3 (D 3) PB3  2|    |7  PB2 (D 2)  Ain1
// Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1
//            GND  4|    |5  PB0 (D 0) pwm0
//                  +----+

#ifdef ADC_ISR
ISR(ADC_vect)
{
	// handle ADC conversion ready interrupt
}//end ISR ADC_vect
#endif

#ifdef TIM0_OVF_ISR
ISR(TIM0_OVF_vect)
{
	// handle timer0 overflow interrupt
}// end ISR TIM0_OVF_vect
#endif

#ifdef TIM0_COMPA_ISR
ISR( TIM0_COMPA_vect )
{
        // handle timer0 compare match interrupt
}//end ISR TIM0_COMPA_vect
#endif

unsigned char spi_transfer(unsigned char data)
{
	USIDR = data;
  	USISR = (1<<USIOIF);
   	do {
      		USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC);
   	} while ((USISR & (1<<USIOIF)) == 0);
   	return USIDR;
}//end spi_transfer

void init()
{
	// this needs to be called before setup() or some functions won't
	// work there
	sei();

	// dumpt everything, and only added the 2 timers the attiny has 
	// on the ATmega168, timer 0 is also used for fast hardware pwm
	// (using phase-correct PWM would mean that timer 0 overflowed half as often
	// resulting in different millis() behavior on the ATmega8 and ATmega168)
	sbi(TCCR0A, WGM01);
	sbi(TCCR0A, WGM00);
	// set timer 0 prescale factor to 64
	// sbi(TCCR0B, CS02);
	sbi(TCCR0B, CS01);
	sbi(TCCR0B, CS00);
	// enable timer 0 overflow interrupt
	sbi(TIMSK, TOIE0);

	// timers 1 are used for phase-correct hardware pwm
	// this is better for motors as it ensures an even waveform
	// note, however, that fast pwm mode can achieve a frequency of up
	// 8 MHz (with a 16 MHz clock) at 50% duty cycle
	// set timer 1 prescale factor to 512
	sbi(TCCR1, CS13);
	cbi(TCCR1, CS12);
	sbi(TCCR1, CS11);
	cbi(TCCR1, CS10);
	sbi(TCCR1, PWM1A);
	// put timer 1 in 8-bit phase correct pwm mode
	// sbi(TCCR1, WGM10); non c'è nell attiny 45

	// set a2d prescale factor to 128
	// 16 MHz / 128 = 125 KHz, inside the desired 50-200 KHz range.
	// XXX: this will not work properly for other clock speeds, and
	// this code should use F_CPU to determine the prescale factor.
	// added F_CPU prescaler
#if F_CPU >= 16000000L //128
	sbi(ADCSRA, ADPS2);
	sbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS0);
#elif F_CPU >= 8000000L //64
	// ausgerechnet in docu
	sbi(ADCSRA, ADPS2);
	sbi(ADCSRA, ADPS1);
#else				//8
	sbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS0);
#endif
	// enable a2d conversions
	sbi(ADCSRA, ADEN);
	// select ADC1
	sbi(ADMUX, MUX0);
	// auto trigger enable
	//sbi(ADCSRA, ADATE);
	// enable interrupt
	//sbi(ADCSRA, ADIE);
	// set prescaler 64
	cbi(ADCSRA, ADPS0);
	sbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS2);
	// auto trigger source free running
	cbi(ADCSRB, ADTS0);
	cbi(ADCSRB, ADTS1);
	cbi(ADCSRB, ADTS2);
	// digital input disable for energy saveing
	sbi(DIDR0, ADC1D);
	
	
	// set output ports
	//sbi(DDRB, DDB0);
	//sbi(DDRB, DDB1);
	// set input ports
	//cbi(DDRB, DDB2);
	
	// write digital values
	//sbi(PORTB, PORTB0);
	//cbi(PORTB, PORTB1);
	// set all ports to OUTPUT
	//DDRB = 0xFF;
	// default value to LOWsbi(PORTB, DDB1);
	//PORTB = 0;

	// PWM fuseing
	// 1) prescaler for the frequency
	//TCCR0A |= (1<<WGM01);
	//TCCR0A |= (1<<WGM00);
	//TCCR0B |= (1<<CS01);
	//TCCR0B |= (1<<CS00);

	// set PWM value (0 to 255)
	//OCR0A = 255;
	//OCR1A = 255;
	
	// start the first conversion of ADC
	//sbi(ADCSRA, ADSC);
}//end init

int discreteHartley( int Width, float* sample, float* outputSample ) {
	int n = 0;
	int N = Width; //SAMPLE WIDTH
	int k = 0;
	int result = 0;
	if ( N > 0 && sample && outputSample ) {
		for ( k = 0; N - 1 >= k; k++ ) {
			float outputValue = 0.0;
			for ( n = 0; N - 1 >= n; n++ ) {
				float coeff = ((2.0f * 3.141592654f) / N ) * k * n;
				float im = (sin(coeff) + cos(coeff));
				outputValue += sample[n] * im;
			}
			// make the hartley backward compatible when multiplying
			outputSample[k] = outputValue * ( 1.0f / sqrt(N));
		}
		result = 0;
	} else {
		result = -1;
	}
	
	return result;
} //end discreteHartley

int main(void)
{
	/*
	// initialize counter
	lugeja = 100;
	//reset the Timer Counter Control Register to its reset value
	TCCR0B = 0;

        #if F_CPU == 8000000L
            	//set counter0 prescaler to 64
		// 125kHz
           	//our FCLK is 8mhz so this makes each timer tick be 8 microseconds long
            	TCCR0B &= ~(1<< CS02); //clear
            	TCCR0B |=  (1<< CS01); //set
            	TCCR0B |=  (1<< CS00); //set
        #elif F_CPU == 1000000L
            	//set counter0 prescaler to 8
		// 125 kHz
            	//our F_CPU is 1mhz so this makes each timer tick be 8 microseconds long
            	TCCR0B &= ~(1<< CS02); //clear
            	TCCR0B |=  (1<< CS01); //set
            	TCCR0B &= ~(1<< CS00); //clear
        #else
            	//unsupported clock speed
           	//TODO: find a way to have the compiler stop compiling and bark at the user
		#error "Please use 8 or 1 MHz"
        #endif

	// Enable Output Compare Match Interrupt
    	TIMSK |= (1 << OCIE0A);
    	//reset the counter to 0
    	TCNT0  = 0;
    	//set the compare value to any number larger than 0
    	OCR0A = 255;
	// Enable global interrupts
    	sei();
	*/
/*
	DDRB = 0xFF;
	sbi(PORTB, PORTB0);
	sbi(PORTB, PORTB1);
	sbi(PORTB, PORTB2);
*/
	//( PORTB & 1 ) ? cbi( PORTB, PORTB0 ) : sbi( PORTB, PORTB0 );

	while ( FALSE );
	while ( FALSE ) {
		sbi( PORTB, PORTB0);
		cbi( PORTB, PORTB0);
	}

	//float* outputSample = (float*)malloc( sizeof(float) * 4 ); sin(1);
	//init();
	//sbi(DDRB, DDB0);
	//sbi(DDRB, DDB1);
	//sbi(DDRB, DDB2);
	/*
	DDRB  = 1<<DDB0;
	DDRB = 1<<DDB1;
	PORTB = (1<<PB0);
	PORTB = (1<<PB1);
	TCCR0A=(1<<COM0A1) | (1<<WGM00) | (1<<WGM01);
	TCCR0B=(1<<WGM02) | (1<<CS00) | (0<<CS01) | (1<<CS02);
	OCR0A=128;
	OCR1A=255;
	for (;;);*/
	// write digital values
	
	//analogWrite(PORTB0, 20);
	/*for (;;) {
		_delay_ms(1000);
		cbi(PORTB, PORTB0);
		_delay_ms(1000);
		sbi(PORTB, PORTB0);
	}*/
	/*
	for (;;) {
		analogWrite(PORTB0, 255);
		_delay_ms(100);
		analogWrite(PORTB0, 200);
		_delay_ms(100);
		analogWrite(PORTB0, 150);
		_delay_ms(100);
		analogWrite(PORTB0, 100);
		_delay_ms(100);
		analogWrite(PORTB0, 50);
		_delay_ms(100);
		analogWrite(PORTB0, 0);
		_delay_ms(100);
		analogWrite(PORTB0, 50);
		_delay_ms(100);
		analogWrite(PORTB0, 100);
		_delay_ms(100);
		analogWrite(PORTB0, 150);
		_delay_ms(100);
		analogWrite(PORTB0, 200);
		_delay_ms(100);
	}*/

	sbi( DDRB, DDB0 );
	sbi( DDRB, DDB1 );
	sbi( DDRB, DDB2 );

	cbi( PORTB, PORTB0 );
	cbi( PORTB, PORTB1 );
	cbi( PORTB, PORTB2 );

	//SPCR |= _BV(MSTR);
  	//SPCR |= _BV(SPE);

	for (;;) {
		//spi_transfer( 'A' );
		//spi_transfer( 'B' );
		//spi_transfer( '0' );
		//_delay_ms(1);
	}

	DDRB |= (1 << 0) | (0 << 4);   //0 = input, 1 = output, PB0 is output, PB4 is input 
   	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);   //ADC Prescalar set to 64 - 125kHz@8MHz 
    	ADMUX |= (1 << REFS2) | (1 << REFS1) | (0 << REFS0);    //Sets ref. voltage to 2.56v internal reference 
    	ADMUX |= (1 << ADLAR);	// Left adjust ADC result to allow easy 8 bit reading 
   	ADMUX |= (0 << MUX3) | (0 << MUX2) | (1 << MUX1) | (0 << MUX0);   //Selects channel ADC2 (PB4) 
      	ADCSRA |= (1 << ADEN);  // Enable ADC 
   	ADCSRA |= (1 << ADATE);   // Enable ADC Auto Trigger Mode 

  	for (;;) {
		// start the conversion
		sbi(ADCSRA, ADSC);
		// ADSC is cleared when the conversion finishes
		while (bit_is_set(ADCSRA, ADSC));

		if ( ADCH < 100 ) {
         		PORTB |= (0 << 0); // Turn off PB0
      		} else {
         		PORTB &= ~(1 << 0); // Turn on PB0
     		}
  	}

	for (;;) {
		// blink led
		cbi(PORTB, PORTB0);
		sbi(PORTB, PORTB1);
		sbi(PORTB, PORTB2);
		_delay_ms(100);
		sbi(PORTB, PORTB0);
		cbi(PORTB, PORTB1);
		sbi(PORTB, PORTB2);
		_delay_ms(100);
		sbi(PORTB, PORTB0);
		sbi(PORTB, PORTB1);
		cbi(PORTB, PORTB2);
		_delay_ms(100);
		cbi(PORTB, PORTB0);
		cbi(PORTB, PORTB1);
		sbi(PORTB, PORTB2);
		_delay_ms(100);
		cbi(PORTB, PORTB0);
		sbi(PORTB, PORTB1);
		cbi(PORTB, PORTB2);
		_delay_ms(100);
		sbi(PORTB, PORTB0);
		cbi(PORTB, PORTB1);
		cbi(PORTB, PORTB2);
		_delay_ms(100);
		cbi(PORTB, PORTB0);
		cbi(PORTB, PORTB1);
		cbi(PORTB, PORTB2);
		_delay_ms(100);
	}
	return 1;
}
