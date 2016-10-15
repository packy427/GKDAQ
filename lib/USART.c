// -------------------------------------------------------------
// USART.c
// Patrick Kennedy
// 
// Basic functions for simple USART communication
// Referenced from ATMega Datasheet
// 
// -------------------------------------------------------------

#include <avr/io.h>
#include "USART.h"
#include <util/setbaud.h>

#define EN_DEBUG 1

// Set registers for USART communication
void InitUSART(void) {
  UBRR0H = UBRRH_VALUE;     // Values defined in setbaud.h
  UBRR0L = UBRRL_VALUE;
#if USE_2X                  // Set 2X mode if requested
  UCSR0A |= (1 << U2X0);
#else
  UCSR0A &= ~(1 << U2X0);
#endif

  UCSR0B = (1 << TXEN0) | (1 << RXEN0);     // Enable transmitter and reciever
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // Set frame to 8 data, 1 stop bits
}

// Load data into the transmit buffer for sending
void TransmitByte(uint8_t data) {
  loop_until_bit_is_set(UCSR0A, UDRE0);     // Wait for empty buffer
  UDR0 = data;      // Add data to transmit buffer
}

// Read data from the recieve buffer
uint8_t ReceiveByte(void) {
  loop_until_bit_is_set(UCSR0A, RXC0);      // Wait until buffer has data
  return UDR0;      // Read data from recieve buffer
}

// -------------------------- //
//      Output functions      //
// -------------------------- //

// Print string to serial
void PrintString(const char myString[]) {
  uint8_t i = 0;
  // Loop through string until NULL character is found (0)
  while (myString[i]) {
    TransmitByte(myString[i]);
    i++;
  }
}

// Print decimal equivalent of byte to serial
void PrintDecimalByte(uint8_t byte) {
  TransmitByte('0' + (byte / 100));         // Hundreds place
  TransmitByte('0' + ((byte / 10) % 10));   // Tens place
  TransmitByte('0' + (byte % 10));          // Ones place
}

// Print decimal equivalent of 16bit int to serial
void PrintDecimalWord(uint16_t word) {
  TransmitByte('0' + (word / 10000));       // Ten-thou place
  TransmitByte('0' + ((word / 1000) % 10)); // Thousands place
  TransmitByte('0' + ((word / 100) % 10));  // Hundreds place
  TransmitByte('0' + ((word / 10) % 10));   // Tens place
  TransmitByte('0' + (word % 10));          // Ones place
}

// Print byte as sequence of 8 bits to serial
void PrintBinaryByte(uint8_t byte) {
  uint8_t bit;
  for (bit = 0; bit < 8; bit++) {
    if (bit_is_set(byte, bit))
      TransmitByte('1');
    else
      TransmitByte('0');
  }
}

// Prints value of a byte as its hexadecimal equivalent to serial
void PrintHexByte(uint8_t byte) {
  uint8_t nibble;
  nibble = (byte & 0b11110000) >> 4;
  TransmitByte(NibbleToHex(nibble));
  nibble = byte & 0b00001111;
  TransmitByte(NibbleToHex(nibble));
}

// ------------------------- //
//      Input Functions      //
// ------------------------- //

// Read in sequential characters as a byte
uint8_t ReadByte(void) {
  // Initialize digit placeholders
  char hundreds = '0';
  char tens = '0';
  char ones = '0';
  char thisChar = '0';

  // As each new digit is recieved shift characters left into next placeholder
  do {
    hundreds = tens;
    tens = ones;
    ones = thisChar;
    thisChar = ReceiveByte();    // Get next character
#if EN_DEBUG
    TransmitByte(thisChar);     // Echo value to verify correct operation
#endif
  } while (thisChar != '\r');    // Loop until return character is read
  
  // Convert from characters to integer
  return (100 * (hundreds - '0') + 10 * (tens - '0') + ones - '0'); 
}

// Read in sequential characters as a string
void ReadString(char myString[], uint8_t maxLength) {
  char response;
  uint8_t i;
  i = 0;
  while (i < (maxLength - 1)) {     // Prevent overruns
    response = ReceiveByte();
#if EN_DEBUG
    TransmitByte(response);         // Echo to verify correct operation
#endif
    if (response == '\r') {         // End of transmisson
      break;
    }
    else {
      myString[i] = response;       // Add to string buffer
      i++;
    }
  }
  myString[i] = 0;      // Add NULL Character to signify end of array
}

// --------------------------- //
//      Support Functions      //
// --------------------------- //

// Converts a 4bit number to a char in its hexadecimal equivalent
char NibbleToHex(uint8_t nib) {
  if (nib < 10) {
    return ('0' + nib);
  }
  else {
    return ('A' + nib - 10);
  }
}