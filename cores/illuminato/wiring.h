#ifndef WIRING_H
#define WIRING_H

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include "wiring_serial.h"

#ifdef __cplusplus
extern "C"{
#endif

#include "binary.h"

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT)) //!<Set a bit within a byte
#define CLRBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT)) //!< Clear a bit within a byte
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT)) //!< Flip a bit within a byte
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT)) //!< Check a bit within a byte

#define TRUE	1	//!< A Global true
#define FALSE	0	//!< A global false
#define ON		1	//!< A boolean
#define OFF		0	//!< A boolean
#define SET		1	//!< A boolean
#define CLEAR	0	//!< A boolean
#define INPUT   0   
#define OUTPUT  1
#define HIGH    1
#define LOW     0

void init(void);
void setup(void);
void loop(void);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
uint8_t digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);
unsigned long millis();
void delay(unsigned long ms);
void bling(uint8_t percent);
void blinga(uint8_t percent);


#ifdef __cplusplus
} // extern "C"
#endif

#endif
