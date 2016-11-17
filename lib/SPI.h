//--------------------------------------------------------
//  SPI.h
//  Patrick Kennedy
//
//  Fosc is assumed to 16MHz. Resultant bit rate is 1MHz
//--------------------------------------------------------

#ifndef SPI_H
#define SPI_H

#include <avr/io.h>

void SPI_Init();  // Initialize SPI
uint8_t SPI_ExchangeByte(uint8_t txData);   // Send and read byte
void SPI_TransmitByte(uint8_t txData);  // Send byte and ignore incoming byte
uint8_t SPI_RecieveByte();  // Send 0 and read incoming byte


#endif // SPI_H
