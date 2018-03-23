/*! \file Analog.h \brief Simple library for AVR analog inputs. */
//-----------------------------------------------------------------------------
//  Filename   : Analog.h
//  Title      : Simple library for AVR analog inputs
//  Author     : Patrick Kennedy
//  Created    : 08/02/2016
//  Modified   : 02/26/2018
//  Version    : 0.1
//  Description:
//     Basic functions for simple analog inputs on AVR
//     Referenced from ATMega Datasheet
//-----------------------------------------------------------------------------
#ifndef GKDAQ_ANALOG_H
#define GKDAQ_ANALOG_H
void Analog_Init();

uint16_t GetAnalogInput(uint8_t AnalogInputPin);

uint16_t GetADC();
#endif  //GKDAQ_ANALOG_H