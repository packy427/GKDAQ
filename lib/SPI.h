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
void SPI_ExchangeByte(uint8_t txData, uint8_t* rxData);   // Send and read byte
void SPI_TransmitByte(uint8_t txData);  // Send byte and ignore incoming byte
void SPI_RecieveByte(uint8_t* rxData);  // Send 0 and read incoming byte

#endif // SPI_H
