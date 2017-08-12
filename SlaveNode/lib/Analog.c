/******************************************************************************
*  Analog.c
*  Patrick Kennedy
*
*  Controls reading analog inputs
*
******************************************************************************/

#include <avr/io.h>
#include Analog.h

#define MUX_ALL = (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)

void Analog_Init(){
  ADMUX = (1 << REFS0);   // Set reference voltage to Vcc, A0 as input
  ADCSRA = (1 << ADEN) | (1 << ADSP2);  // Enable ADC, CLK prescale 1/16
}

uint16_t GetAnalogInput(uint8_t AnalogInputPin){
  ADMUX &= ~MUX_ALL;  // Clear all mux bits, resets to A0 as ADC input

  if(AnalogInputPin == 1){
    ADMUX |= (1 << MUX0); // Set A1 as ADC input
  }
  else if(AnalogInputPin == 2){
    ADMUX |= (1 << MUX1); // Set A2 as ADC input
  }
  else if(AnalogInputPin == 3){
    ADMUX |= (1 << MUX1) | (1 << MUX0);   // Set A3 as ADC input
  }

  return GetADC();  // After setting correct pin, read input
}

uint16_t GetADC(){
  ADCSRA |= (1 << ADSC);  // Start conversion
  while((ADCSRA & (1 << ADIF) ~= 1){    // Wait until conversion completes
    // Do nothing
  }
  ADCSRA |= (1 << ADIF);  // Write logic 1 to clear flag
  return (ADCH << 8) | (ADCL);
}
