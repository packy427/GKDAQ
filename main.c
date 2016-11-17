//-----------------------------------------------
//  main.c
//  Patrick Kennedy
//
//  Main file for AVR program
//-----------------------------------------------

#include <avr/io.h>
#include "lib/MCP2515.c"
#include "lib/USART.c"
#include "lib/SPI.c"
#include "lib/PinDefinitions.h"
#include <util/delay.h>

// Used for
#define CAN_SELECT    SPI_PORT &= ~(1 << SPI_CAN_SS)
#define CAN_DESELECT  SPI_PORT |= (1 << SPI_CAN_SS)

//DO SHIT Plz
//and eat a fucking dick
//for the love of god
//suck a clown's ballsack
//cut off your cock
//throw it in a woodchipper
// i g tot nothing lol
// jewz
int main(void){
  uint8_t msgType;
  uint32_t msgID;
  uint64_t data;
  uint8_t c;
  uint64_t sendData;
  uint8_t ctrlCAN;
  uint8_t ctrlTx0;
  uint8_t ctrlTx1;
  uint8_t timeout;
  uint8_t att;
  uint8_t x;

  x = 0;
  timeout = 0xFF;
  _delay_ms(500);
  USART_Init();
  PrintString("USART Initialization Complete\r\n");
  SPI_Init();
  PrintString("SPI Initialization Complete\r\n");
  MCP2515_Init();
  PrintString("MCP2515 Initialization Complete\r\n");

  if(MCP2515_SetCANMode(MODE_LOOPBACK)){
    PrintString("Loopback Mode Fail");
  }
  else{
    PrintString("Loopback Mode OK\r\n");
  }
  sendData = 0;
  MCP2515_Write(0x6D, 0);
  MCP2515_Write(0x6C, 0);
  MCP2515_Write(0x6B, 0);
  MCP2515_Write(0x6A, 0);
  MCP2515_Write(0x69, 0);
  MCP2515_Write(0x68, 0);
  MCP2515_Write(0x67, 0);
  MCP2515_Write(0x66, 0);
  while(1){
  att = 0;
  PrintString("=== LOOP START === \r\n");

  MCP2515_LoadID(0, MSG_STD, 0xBAE, 8);
  MCP2515_LoadData(0, 0xABC);
  PrintString("TX0DLC: ");
  PrintBinaryByte(MCP2515_Read(0x35));
  PrintString("\r\n");
  MCP2515_Write(0x35, 0b00001000);

  _delay_ms(100);

  MCP2515_RTS(0);
  ctrlTx0 = MCP2515_Read(TXB0CTRL) & (1 << TXREQ);
  while(ctrlTx0 == 1 && att < timeout){
    ctrlTx0 = MCP2515_Read(TXB0CTRL) & (1 << TXREQ);
    TransmitByte(ctrlTx0);
    att = att + 1;
  }

  PrintString("Rx Status : ");
  PrintBinaryByte(MCP2515_ReadRxStatus());
  PrintString(" \r\n");
/*
  MCP2515_Write(0x6D, 0);
  MCP2515_Write(0x6C, 0);
  MCP2515_Write(0x6B, 0);
  MCP2515_Write(0x6A, 0);
  MCP2515_Write(0x69, 0);
  MCP2515_Write(0x68, 0);
  MCP2515_Write(0x67, 0);
  MCP2515_Write(0x66, 0);
  */
  data = MCP2515_ReadData(0);

  PrintString("Read Data: ");
  PrintHexByte((data >> 56) & 0xFF);
  PrintHexByte((data >> 48) & 0xFF);
  PrintHexByte((data >> 40) & 0xFF);
  PrintHexByte((data >> 32) & 0xFF);
  PrintHexByte((data >> 24) & 0xFF);
  PrintHexByte((data >> 16) & 0xFF);
  PrintHexByte((data >> 8) & 0xFF);
  PrintHexByte((data >> 0) & 0xFF);
  PrintString("\r\n");

  PrintString("Receive ID Buffer");
  PrintHexByte(MCP2515_Read(0x31));
  PrintHexByte(MCP2515_Read(0x32));
  PrintString("\r\n");

  msgID = MCP2515_ReadID(0);
  PrintString("Read ID: ");
  PrintHexByte((msgID >> 24) & 0xFF);
  PrintHexByte((msgID >> 16) & 0xFF);
  PrintHexByte((msgID >> 8) & 0xFF);
  PrintHexByte(msgID & 0xFF);
  PrintString(" \r\n");

  _delay_ms(10000);
  x = 1;
  }
  return 0;
}
