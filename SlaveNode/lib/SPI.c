/*! \file SPI.c \brief Simple SPI library for AVR. */
//-----------------------------------------------------------------------------
//  Filename   : SPI.c
//  Title      : Simple SPI library for AVR
//  Author     : Patrick Kennedy (PK3)
//  Created    : 08/12/2016
//  Modified   : 02/26/2018
//  Version    : 0.1
//  Description:
//    Fosc is assumed to be 16MHz, resulting baud rate is 1MHz
//-----------------------------------------------------------------------------

#include <avr/io.h>
#include "SPI.h"
#include "PinDefinitions.h"

void SPI_Init(){
  SPI_DDR |= (1 << SPI_CAN_SS); // Set CAN chip select to output
  SPI_DDR |= (1 << SPI_SCK);    // Set SPI clock to output
  SPI_DDR |= (1 << SPI_MOSI);   // Set MOSI line to output

  SPI_PORT |= (1 << SPI_CAN_SS);  // Set CAN select to false
  SPI_PORT |= (1 << SPI_MISO);    // Activate pullup resistor

  SPCR |= (1 << SPR1);  // 1 MHz bit rate
  SPCR |= (1 << MSTR);  // Configure as master
  SPCR |= (1 << SPE);   // Enable
}

uint8_t SPI_ExchangeByte(uint8_t txData){
  SPDR = txData;    // Load SPI shift register, transmission starts immediately
  loop_until_bit_is_set(SPSR, SPIF); // Check for completion
  return SPDR;    // Return value of SPI shift register
}

void SPI_TransmitByte(uint8_t txData){
  SPDR = txData;    // Load SPI shift register, transmission starts immediately
  loop_until_bit_is_set(SPSR, SPIF); // Check for completion
}

uint8_t SPI_RecieveByte(){
  SPDR = 0;   // Send 0 byte
  loop_until_bit_is_set(SPSR, SPIF); // Check for completion
  return SPDR;    // Return value of SPI shift register
}
