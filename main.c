
#include "main.h"

// ATMEL ATTINY85
//
//                  +-\/-+
// Ain0 (D 5) PB5  1|    |8  Vcc
// Ain3 (D 3) PB3  2|    |7  PB2 (D 2)  Ain1
// Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1
//      	  GND  4|    |5  PB0 (D 0) pwm0
//                  +----+

ISR(ADC_vect)
{
	// handle interrupt
	
}

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

//different name, TIMER0_OVF_vect to this
/*ISR(TIM0_OVF_vect)
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}*/

unsigned long millis()
{
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;
}

void delay(unsigned long ms)
{
	unsigned long start = millis();

	while (millis() - start <= ms);
}

int set_bit(int pos, int val)
{
	//val == 0 ? PORTB &= ~(1 << pos) : PORTB |= (1 << pos);

	if (val == 0) {
		// LOW
		PORTB &= ~(1 << pos);
	} else {
		// HIGH
		PORTB |= (1 << pos);
	}
	return 1;
}

int analogRead(uint8_t pin)
{
	uint8_t low, high;

	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
	// ADMUX = (analog_reference << 6) | (pin & 0x3f); // more MUX
	// sapo per tiny45
	ADMUX = pin & 0x3f;

	// without a delay, we seem to read from the wrong channel
	//delay(1);

	// start the conversion
	sbi(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	low = ADCL;
	high = ADCH;

	// combine the two bytes
	return (high << 8) | low;
}

// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite(uint8_t pin, int val)
{
  // We need to make sure the PWM output is enabled for those pins
  // that support it, as we turn it off when digitally reading or
  // writing with them.  Also, make sure the pin is in output mode
  // for consistenty with Wiring, which doesn't require a pinMode
  // call for the analog output pins.
  //pinMode(pin, OUTPUT);

  //Yep, only 2 PMW, Saposoft
  	
	if (pin == PORTB0) {
		if (val == 0) {
			cbi(PORTB, pin);
		} else {
			// connect pwm to pin on timer 0, channel A
			sbi(TCCR0A, COM0A1);
			// set pwm duty
			OCR0A = val;      
		}
	} else if (pin == PORTB1) {
		if (val == 0) {
			cbi(PORTB, pin);
		} else {
			// connect pwm to pin on timer 0, channel B
			sbi(TCCR1, COM1A1);
			// set pwm duty
			OCR1A = val;
		}
	}
}

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
}

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
}

int main(void)
{
	//float* outputSample = (float*)malloc( sizeof(float) * 4 ); sin(1);
	init();
	sbi(DDRB, DDB0);
	sbi(DDRB, DDB1);
	sbi(DDRB, DDB2);
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
	for (;;) {
		// blink led
		sbi(PORTB, PORTB0);
		sbi(PORTB, PORTB1);
		cbi(PORTB, PORTB2);
		_delay_ms(100);
		sbi(PORTB, PORTB0);
		cbi(PORTB, PORTB1);
		sbi(PORTB, PORTB2);
		_delay_ms(100);
		cbi(PORTB, PORTB0);
		sbi(PORTB, PORTB1);
		sbi(PORTB, PORTB2);
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
