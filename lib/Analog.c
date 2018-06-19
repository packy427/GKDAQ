/*! \file Analog.c \brief Simple library for AVR analog inputs. */
//-----------------------------------------------------------------------------
//  Filename   : Analog.c
//  Title      : Simple library for AVR analog inputs
//  Author     : Patrick Kennedy
//  Created    : 08/02/2016
//  Modified   : 02/26/2018
//  Version    : 0.1
//  Description:
//     Basic functions for simple analog inputs on AVR
//     Referenced from ATMega Datasheet
//-----------------------------------------------------------------------------

#include <avr/io.h>
#include "Analog.h"

#define MUX_ALL = (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)

void Analog_Init(){
  ADMUX = (1 << REFS0);   // Set reference voltage to Vcc, A0 as input
  ADCSRA = (1 << ADEN);   // Enable ADC
}

uint16_t GetAnalogInput(uint8_t AnalogInputPin){
  ADMUX &= 0xf0;              // Clear all mux bits
  ADMUX |= AnalogInputPin;    // Set proper mux bits

  ADCSRA |= (1 << ADSC);  // Start conversion
  while((ADCSRA & _BV(ADSC))){    // Wait until conversion completes
    // Do nothing
  }

  return ADC;
}
