/*==============================================================================
 *
 * Title: wiring.c
 * Description: These are the processor specific functions  
 *      
 *
  =============================================================================
 *
 * 
 *============================================================================*/ 

/*==============================================================================
 * PREPROCESSOR DIRECTIVES
 *============================================================================*/
#include "wiring.h"

#define MILLISECOND_CNT_MAX	7 	//8*128uS = 1.024 mS

/*==============================================================================
 * CONSTANTS
 *============================================================================*/

/*
 * These following macros and arrays map the physical header pins to their
 * respective chip pins.
 */
#define portOffsetToField(off,what) ((volatile uint8_t *)(pgm_read_word(&(_port[off].what))))
#define PinToPORT(x) portOffsetToField(pgm_read_byte(&(_pin[x].port)),portx)
#define PinToPIN(x) portOffsetToField(pgm_read_byte(&(_pin[x].port)),pinx)
#define PinToDDR(x) portOffsetToField(pgm_read_byte(&(_pin[x].port)),ddrx)
#define PinToTimer(x) (volatile uint8_t *)(pgm_read_word(&(_pin[x].timer)))
#define PinToPinBit(x) pgm_read_word(&(_pin[x].pinbit))

/* tuples of port registers */
static
const
struct _port_lookup
{
	volatile uint8_t *portx;
	volatile uint8_t *pinx;
	volatile uint8_t *ddrx;
} PROGMEM _port[] =
{
	{ &PORTA, &PINA, &DDRA },
	{ &PORTB, &PINB, &DDRB },
	{ &PORTC, &PINC, &DDRC },
	{ &PORTD, &PIND, &DDRD },
	{ &PORTE, &PINE, &DDRE },
	{ &PORTF, &PINF, &DDRF },
	{ &PORTG, &PING, &DDRG },
};
enum _port_offsets
{
	PORT_A,
	PORT_B,
	PORT_C,
	PORT_D,
	PORT_E,
	PORT_F,
	PORT_G,
} __attribute__((__packed__)); /* fit it into a byte */

static
const
struct _pin_lookup
{
	enum _port_offsets port;
	uint8_t pinbit;
	volatile uint8_t *timer;
} PROGMEM _pin[] =
{
	{ PORT_E, PE0, NULL    }, /*  0 */
	{ PORT_E, PE1, NULL    }, /*  1 */
	{ PORT_E, PE2, NULL    }, /*  2 */
	{ PORT_E, PE3, NULL    }, /*  3 */
	{ PORT_E, PE4, NULL    }, /*  4 */
	{ PORT_E, PE5, NULL    }, /*  5 */
	{ PORT_E, PE6, NULL    }, /*  6 */
	{ PORT_E, PE7, NULL    }, /*  7 */

	{ PORT_D, PD2, NULL    }, /*  8 */
	{ PORT_D, PD3, NULL    }, /*  9 */
	{ PORT_D, PD4, NULL    }, /* 10 */
	{ PORT_D, PD6, NULL    }, /* 11 */
	{ PORT_D, PD7, NULL    }, /* 12 */

	{ PORT_G, PG0, NULL    }, /* 13 */

	{ PORT_B, PB0, NULL    }, /* 14 */
	{ PORT_B, PB4, &OCR0A  }, /* 15 */
	{ PORT_B, PB5, &OCR1AL }, /* 16 */
	{ PORT_B, PB6, &OCR1BL }, /* 17 */

	{ PORT_G, PG3, NULL    }, /* 18 */
	{ PORT_G, PG4, NULL    }, /* 19 */

	{ PORT_D, PD0, NULL    }, /* 20 */
	{ PORT_D, PD1, NULL    }, /* 21 */

	{ PORT_G, PG1, NULL    }, /* 22 */

	{ PORT_C, PC0, NULL    }, /* 23 */
	{ PORT_C, PC1, NULL    }, /* 24 */
	{ PORT_C, PC2, NULL    }, /* 25 */
	{ PORT_C, PC3, NULL    }, /* 26 */
	{ PORT_C, PC4, NULL    }, /* 27 */
	{ PORT_C, PC5, NULL    }, /* 28 */
	{ PORT_C, PC6, NULL    }, /* 29 */
	{ PORT_C, PC7, NULL    }, /* 30 */
	{ PORT_G, PG2, NULL    }, /* 31 */
	{ PORT_A, PA7, NULL    }, /* 32 */
	{ PORT_A, PA6, NULL    }, /* 33 */
	{ PORT_A, PA5, NULL    }, /* 34 */
	{ PORT_A, PA4, NULL    }, /* 35 */

	{ PORT_F, PF5, NULL    }, /* 36 */
	{ PORT_F, PF4, NULL    }, /* 37 */
	{ PORT_F, PF3, NULL    }, /* 38 */
	{ PORT_F, PF2, NULL    }, /* 39 */
	{ PORT_F, PF1, NULL    }, /* 40 */
	{ PORT_F, PF0, NULL    }, /* 41 */

	{ PORT_B, PB7, &OCR2A  }, /* 42 - 'bling' */
};

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

volatile unsigned long millis_var=0;
volatile unsigned char millis_var_counter=0;

/*==============================================================================
 * FUNCTIONS
 *============================================================================*/

/* ===========================================================================
*  FUNCTION: pinMode
*
*  DESIGN DESCRIPTION:
*     Changes the pin direction
*
*  PARAMETER LIST:
*     pin - The pin number to be changed.
*     val - INPUT or OUTPUT value to be written to the pin
*
*  RETURNED:
*     none
*
*  DESIGN NOTES/CONSTRAINTS:
*     
*
*===========================================================================*/
void pinMode(uint8_t pin, uint8_t mode)
{
	volatile uint8_t *timer_port = PinToTimer(pin);

    if (mode == INPUT)
    {
        CLRBIT(*PinToDDR(pin), PinToPinBit(pin));
    }
    else
    {
        SETBIT(*PinToDDR(pin), PinToPinBit(pin));
    }

	/* default to digital */	
	if (timer_port == &OCR0A)
	{
		CLRBIT(TCCR0A, COM0A1);
	}
	else if (timer_port == &OCR1AL)
	{
		CLRBIT(TCCR1A, COM1A1);
	}
	else if (timer_port == &OCR1BL)
	{
		CLRBIT(TCCR1A, COM1B1);
	}
	else if (timer_port == &OCR2A)
	{
		CLRBIT(TCCR2A, COM2A1);
		CLRBIT(TCCR2A, COM2A0);
	}
}

/* ===========================================================================
*  FUNCTION: digitalWrite
*
*  DESIGN DESCRIPTION:
*     Write a value to a pin
*
*  PARAMETER LIST:
*     pin - The pin number to be changed.
*     val - HIGH or LOW value to write to the pin
*
*  RETURNED:
*     none
*
*  DESIGN NOTES/CONSTRAINTS:
*     
*
*===========================================================================*/
void digitalWrite(uint8_t pin, uint8_t val)
{
    if (val == HIGH)
    {
        SETBIT(*PinToPORT(pin), PinToPinBit(pin)); 
    }
    else
    {
        CLRBIT(*PinToPORT(pin), PinToPinBit(pin));
    }
}

/* ===========================================================================
*  FUNCTION: digitalRead
*
*  DESIGN DESCRIPTION:
*     Reads a digital pin
*
*  PARAMETER LIST:
*     pin - The pin to be read
*
*  RETURNED:
*     returns 0 or the masked pin register value
*
*  DESIGN NOTES/CONSTRAINTS:
*     
*
*===========================================================================*/
uint8_t digitalRead(uint8_t pin)
{
    return CHECKBIT(*PinToPORT(pin), PinToPinBit(pin));
}

/* ===========================================================================
*  FUNCTION: analogRead
*
*  DESIGN DESCRIPTION:
*     Reads an analog pin
*
*  PARAMETER LIST:
*     pin - The pin to be read
*
*  RETURNED:
*     returns the 10-bit analog value from the pin  
*
*  DESIGN NOTES/CONSTRAINTS:
*     
*
*===========================================================================*/
int analogRead(uint8_t pin)
{
	uint8_t low, high;

	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits)
	ADMUX = (1 << REFS0) | ((5-pin) & 0x0f);
    
	// start the conversion
	SETBIT(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (CHECKBIT(ADCSRA, ADSC)) { ; }

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	low = ADCL;
	high = ADCH;

	// combine the two bytes
	return (high << 8) | low;
}

/* ===========================================================================
 * analogRead
 * sets the PWM value for the given pin if supported, otherwise sets
 * closest digital value
 * =========================================================================*/
void analogWrite(uint8_t pin, int val)
{
	volatile uint8_t *timer_port = PinToTimer(pin);

	/* set pin to output mode */
	SETBIT(*PinToDDR(pin), PinToPinBit(pin));

	if (timer_port == NULL)
	{
		/* defalt to digital if pin is not PWM capable */
		digitalWrite(pin, (val < 128) ? LOW : HIGH);
		return;
	}
	
	if (timer_port == &OCR0A)
	{
		SETBIT(TCCR0A, COM0A1);
	}
	else if (timer_port == &OCR1AL)
	{
		SETBIT(TCCR1A, COM1A1);
	}
	else if (timer_port == &OCR1BL)
	{
		SETBIT(TCCR1A, COM1B1);
	}
	else if (timer_port == &OCR2A)
	{
		SETBIT(TCCR2A, COM2A1);
		SETBIT(TCCR2A, COM2A0); /* bling needs to be inverted */
	}
	*timer_port = val;
}

/* ===========================================================================
*  FUNCTION: SIG_OVERFLOW0
*
*  DESIGN DESCRIPTION:
*     128usec timer 0 interrupt
*
*  PARAMETER LIST:
*     none
*
*  RETURNED:
*     none
*
*  DESIGN NOTES/CONSTRAINTS:
*     
*
*===========================================================================*/
SIGNAL(SIG_OVERFLOW0)
{
    /* The millisecond counter */
    if (millis_var_counter == MILLISECOND_CNT_MAX)
    {
        millis_var++;           // incremement the free running counter 
        millis_var_counter=0;   // reset the 128usec counter
    }
    else
    {
        millis_var_counter++;   // incremement the 128usec counter
    }

}

/* ===========================================================================
*  FUNCTION: millis
*
*  DESIGN DESCRIPTION:
*     Returns the free-running millisecond counter 
*
*  PARAMETER LIST:
*     none
*
*  RETURNED:
*     unsigned long - the milliseconds since startup
*
*  DESIGN NOTES/CONSTRAINTS:
*     
*
*===========================================================================*/
unsigned long millis()
{
	unsigned long m;

	cli();
	m = millis_var;
	sei();
	return m;
}

/* ===========================================================================
*  FUNCTION: delay
*
*  DESIGN DESCRIPTION:
*     A delay function. 
*
*  PARAMETER LIST:
*     ms - the number of milliseconds to delay
*
*  RETURNED:
*     none
*
*  DESIGN NOTES/CONSTRAINTS:
*     
*
*===========================================================================*/
void delay(unsigned long ms)
{
    long pTime = millis(); //save the current time
    while( (millis() - pTime) < ms ) //check the ellapsed time
    {
        ; //burn cycles
    }
}


void init()
{
    //Init TIMER 0 
	TCCR0A = (0<<CS02) | (1<<CS01) | (0<<CS00); //timer 0 setup to overflow every 128us
	TIMSK0 = (1<<TOIE0); //enable timer 0 overflow interrupts

	//Init TIMER 1
	TCCR1A = (1<<WGM10); /* PWM, Phase Correct */
	TCCR1B = (1<<CS10); /* CLK/1 */

	//Init TIMER 2
	TCCR2A = (1<<WGM20) | (1<<CS20); /* PWM, Phase Correct, CLK/1 */

    //Init ADC      
    ADCSRA = (1 << ADEN) |                                 // Turn on the ADC converter
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);   // Clock = F_CPU / 128

    sei();          //enable global interupts
    //delay(2500);    //wait for the DTR to stop messing with us
}

void blinga(uint8_t val)
{
	analogWrite(42, val);
}

void bling(uint8_t percent)
{
    pinMode(42, OUTPUT);

    if (percent)
    {   
        digitalWrite(42, LOW);
    }
    else
    {
        digitalWrite(42, HIGH);
    }
}
