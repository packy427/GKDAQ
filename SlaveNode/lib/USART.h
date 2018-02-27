/*! \file USART.h \brief Simple USART library for AVR. */
//-----------------------------------------------------------------------------
//  Filename   : USART.h
//  Title      : Simple USART library for AVR
//  Author     : Patrick Kennedy (PK3)
//  Created    : 08/20/2016
//  Modified   : 02/26/2018
//  Version    : 0.1
//  Description:
//     Basic functions for simple USART communication
//     Referenced from ATMega Datasheet
//-----------------------------------------------------------------------------

#ifndef GKDAQ_USART_H
#define GKDAQ_USART_H

#ifndef BAUD
#define BAUD  9600
#endif

#define   USART_HAS_DATA   bit_is_set(UCSR0A, RXC0)
#define   USART_READY      bit_is_set(UCSR0A, UDRE0)

void USART_Init(void);

void TransmitByte(uint8_t data);

uint8_t ReceiveByte(void);

void PrintString(const char myString[]);

void ReadString(char myString[], uint8_t maxLength);

void PrintDecimalByte(uint8_t byte);

void PrintDecimalWord(uint16_t word);

void PrintBinaryByte(uint8_t byte);

void PrintHexByte(uint8_t byte);

char NibbleToHex(uint8_t nibble);

uint8_t ReadByte(void);
#endif  // GKDAQ_USART_H
