/*! \file SPI.h \brief Simple SPI library for AVR. */
//-----------------------------------------------------------------------------
//  Filename   : SPI.h
//  Title      : Simple SPI library for AVR
//  Author     : Patrick Kennedy (PK3)
//  Created    : 08/12/2016
//  Modified   : 02/26/2018
//  Version    : 0.1
//  Description:
//    Fosc is assumed to be 16MHz, resulting baud rate is 1MHz
//-----------------------------------------------------------------------------

#ifndef GKDAQ_SPI_H
#define GKDAQ_SPI_H

#include <avr/io.h>

void SPI_Init();  // Initialize SPI
uint8_t SPI_ExchangeByte(uint8_t txData);   // Send and read byte
void SPI_TransmitByte(uint8_t txData);  // Send byte and ignore incoming byte
uint8_t SPI_ReceiveByte();  // Send 0 and read incoming byte


#endif  // GKDAQ_SPI_H
