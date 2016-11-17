// -------------------------------------------------------------
// USART.h
// Patrick Kennedy
//
// Basic functions for simple USART communication
// Referenced from ATMega Datasheet
//
// -------------------------------------------------------------
#ifndef USART_H
#define USART_H

#ifndef BAUD                          /* if not defined in Makefile... */
#define BAUD  9600                     /* set a safe default baud rate */
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
#endif
