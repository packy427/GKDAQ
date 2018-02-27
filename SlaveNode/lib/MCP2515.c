/*! \file MCP2515.c \brief Simple library for MCP2515 operation with AVR. */
//-----------------------------------------------------------------------------
//  Filename   : MCP2515.c
//  Title      : Simple library for MCP2515 operation with AVR
//  Author     : Patrick Kennedy
//  Created    : 08/20/2016
//  Modified   : 02/26/2018
//  Version    : 0.1
//  Description:
//     Basic functions for simple MCP2515 operation
//     Referenced from MCP2515 Datasheet
//-----------------------------------------------------------------------------

#include <avr/io.h>
#include "SPI.h"
#include "MCP2515.h"
#include "USART.h"

// Used for selecting/deselecting CAN chip
#define CAN_SELECT    SPI_PORT &= ~(1 << SPI_CAN_SS)
#define CAN_DESELECT  SPI_PORT |= (1 << SPI_CAN_SS)

void MCP2515_Init(){
  uint8_t i;
  MCP2515_Reset(MODE_CONFIG);

  // Control registers set for 125kBaud with 16MHz ceramic resonator
  // Eventually will convert this to a set baud rate function
  MCP2515_Write(CNF1,0xC3);
  MCP2515_Write(CNF2,0xB1);
  MCP2515_Write(CNF3,0x05);

  // Set filters 0-2 to zero
  for(i = 0; i<12; i++){
    CAN_SELECT;
    MCP2515_Write(0x00 + i, 0x00);
  }
  CAN_DESELECT;

  // Set filters 3-5 to zero
  for(i=0; i<12; i++){
    CAN_SELECT;
    MCP2515_Write(0x10 + i, 0x00);
  }
  CAN_DESELECT;

  // Set all masks to 0
  for(i=0; i<8; i++){
    CAN_SELECT;
    MCP2515_Write(0x20 + i, 0x00);
  }
  CAN_DESELECT;

}

void MCP2515_SendCANMessage(uint8_t type, uint32_t ID, uint64_t data, uint8_t length){
  uint8_t txBuff;

  txBuff = MCP2515_GetEmptyTxBuffer();        // Get available buffer
  MCP2515_LoadID(txBuff, type, ID, length);   // Load ID information
  MCP2515_LoadData(txBuff, data);             // Load data into registers
  MCP2515_RTS(txBuff);                        // Request to send message
}

void MCP2515_ReceiveCANMessage(uint8_t *type, uint32_t *ID, uint64_t *data){
  uint8_t rxStatus;
  uint8_t rx0;
  uint8_t rx1;
  uint8_t filter;

  CAN_SELECT;
  SPI_TransmitByte(MCP_RXSTATUS);
  rxStatus = SPI_RecieveByte();
  SPI_RecieveByte();
  CAN_DESELECT;
  rx0 = rxStatus & 0x40;  // Get flag for receive buffer 0
  rx1 = rxStatus & 0x80;  // Get flag for receive buffer 1

  PrintString("Rx Status: ");
  PrintBinaryByte(rxStatus);
  PrintString(" \r\n");

  if (rx0){
    *type = rxStatus & 0x18;
    filter = rxStatus & 0x07;
    if ((*type == MSG_STD) | (*type == MSG_RTR)){
      CAN_SELECT;
      SPI_TransmitByte(MCP_READRXB0ID);
      *ID = SPI_RecieveByte() << 0x05;
      *ID |= SPI_RecieveByte();
      CAN_DESELECT;
    }
    if ((*type == MSG_EXT) | (*type == MSG_EXTRTR)){
      // Need to implement eventually
    }
    if ((*type == MSG_EXT) | (*type == MSG_STD)){
      CAN_SELECT;
      SPI_TransmitByte(MCP_READRXB0DATA);
      *data = (uint64_t)(SPI_RecieveByte() << 56);
      *data = (uint64_t)(SPI_RecieveByte() << 48);
      *data = (uint64_t)(SPI_RecieveByte() << 40);
      *data = (uint64_t)(SPI_RecieveByte() << 32);
      *data = (uint64_t)(SPI_RecieveByte() << 24);
      *data = (uint64_t)(SPI_RecieveByte() << 16);
      *data = (uint64_t)(SPI_RecieveByte() << 8);
      *data = (uint64_t)(SPI_RecieveByte() << 0);
      CAN_DESELECT;
    }
  }
  if (rx1){
    *type = rxStatus & 0x18;
    filter = rxStatus & 0x07;
  }

}
uint32_t MCP2515_ReadID(uint8_t rxBuff){
  uint32_t ID;
  uint8_t cmd;
  if (rxBuff == 0){
    cmd = MCP_READRXB0ID;
  }
  CAN_SELECT;
  SPI_TransmitByte(cmd);
  ID = SPI_RecieveByte() << 0x03;
  ID |= (SPI_RecieveByte() >> 0x05) & 0b111;
  CAN_DESELECT;
  return ID;
}

uint8_t MCP2515_ReadRxStatus(){
  uint8_t rxStatus;
  uint8_t rx0;
  uint8_t rx1;
  CAN_SELECT;
  SPI_TransmitByte(MCP_RXSTATUS);
  rxStatus = SPI_RecieveByte();
  SPI_RecieveByte();
  CAN_DESELECT;
  rx0 = rxStatus & 0x40;  // Get flag for receive buffer 0
  rx1 = rxStatus & 0x80;  // Get flag for receive buffer 1
  return (rx1 << 1) + rx0;
}

uint64_t MCP2515_ReadData(uint8_t rxBuff){
  uint64_t data;
  uint64_t buff;

  CAN_SELECT;
  //data = (uint64_t) MCP2515_Read(0x3D);
  //data |= (uint64_t) MCP2515_Read(0x3C);
  //data |= (uint64_t) MCP2515_Read(0x3B);
  //data |= (uint64_t) MCP2515_Read(0x3A);
  //data |= (uint64_t) MCP2515_Read(0x39);
  //data |= (uint64_t) MCP2515_Read(0x38);
  //data |= (uint64_t) MCP2515_Read(0x37);
  //data |= (uint64_t) MCP2515_Read(0x36);
  SPI_TransmitByte(MCP_READRXB0DATA);
  data = (uint64_t) SPI_RecieveByte() << 0;
  data |= (uint64_t) SPI_RecieveByte() << 8;
  data |= (uint64_t) SPI_RecieveByte() << 16;
  data |= (uint64_t) SPI_RecieveByte() << 24;
  data |= (uint64_t) SPI_RecieveByte() << 32;
  data |= (uint64_t) SPI_RecieveByte() << 40;
  data |= (uint64_t) SPI_RecieveByte() << 48;
  data |= (uint64_t) SPI_RecieveByte() << 56;
  CAN_DESELECT;
  return data;
}

void MCP2515_LoadID(uint8_t txBuff, uint8_t type, uint32_t ID, uint8_t length){
  uint8_t SIDL; // Second control register to write to
  uint8_t SIDH;
  uint8_t EID8;
  uint8_t EID0;
  uint8_t DLC;
  uint8_t cmd;

  SIDH = (ID >> 3) & 0xFF;   // Extract SID10-SID3
  SIDL = ((ID & 0x07) << 5); // Extract SID2-SID0

  if((type == MSG_EXT) | (type == MSG_EXTRTR)){
    SIDL |= (1 << 3);           // Set extended ID flag
    SIDL |= (ID >> 27) & 0x03; // Extract EID17-EID16
    EID8 = (ID >> 19) & 0xFF;   // Extract EID15-EID8
    EID0 = (ID >> 11) & 0xFF;   // Extract EID7-EID0
  }
  else{
    EID8 = 0;
    EID0 = 0;
  }

  DLC = length & 0b1111;
  if((type == MSG_RTR) || (type == MSG_EXTRTR)){
    DLC |= (1 << RTR);
  }

  switch(txBuff){
  case 0: cmd = MCP_LOADTXB0ID;
    break;
  case 1: cmd = MCP_LOADTXB1ID;
    break;
  default: cmd = MCP_LOADTXB2ID;
    break;
  }
  CAN_SELECT;
  SPI_TransmitByte(cmd);
  SPI_TransmitByte(SIDH);
  SPI_TransmitByte(SIDL);
  SPI_TransmitByte(EID8);
  SPI_TransmitByte(EID0);
  SPI_TransmitByte(DLC);
  CAN_DESELECT;
}

void MCP2515_LoadData(uint8_t txBuff, uint64_t data){
  // Always load all 8 bytes, let DLC determine how many to send
  uint8_t cmd;
  uint8_t d0;
  uint8_t d1;
  uint8_t d2;
  uint8_t d3;
  uint8_t d4;
  uint8_t d5;
  uint8_t d6;
  uint8_t d7;

  switch(txBuff){
  case 0: cmd = MCP_LOADTXB0DATA;
    break;
  case 1: cmd = MCP_LOADTXB1DATA;
    break;
  default: cmd = MCP_LOADTXB2DATA;
    break;
  }

  d0 = data & 0xFF;
  d1 = (data >> 8) & 0xFF;
  d2 = (data >> 16) & 0xFF;
  d3 = (data >> 24) & 0xFF;
  d4 = (data >> 32) & 0xFF;
  d5 = (data >> 40) & 0xFF;
  d6 = (data >> 48) & 0xFF;
  d7 = (data >> 56) & 0xFF;

  CAN_SELECT;
  SPI_TransmitByte(cmd);
  SPI_TransmitByte(d0);  // Data byte 0
  SPI_TransmitByte(d1);  // Data byte 1
  SPI_TransmitByte(d2);  // Data byte 2
  SPI_TransmitByte(d3);  // Data byte 3
  SPI_TransmitByte(d4);  // Data byte 4
  SPI_TransmitByte(d5);  // Data byte 5
  SPI_TransmitByte(d6);  // Data byte 6
  SPI_TransmitByte(d7);  // Data byte 7
  CAN_DESELECT;
}

uint8_t MCP2515_SetCANMode(uint8_t reqMode){
  uint8_t mode;
  MCP2515_BitModify(CANCTRL, 0b11100000, reqMode << 5);   // Write mode to register
  mode = (MCP2515_Read(CANSTAT));       // Read current mode
  PrintBinaryByte(mode);  // Add 0x30 to convert from single digit number to representative char
  mode = (mode >> 5) & 0b111;
  if (mode == reqMode) {
    return MCP2515_OK;  // Mode change successful
  }
  else{
    return MCP2515_ERR; // Mode change unsuccessful
  }
}

uint8_t MCP2515_GetEmptyTxBuffer(){
  // Gotta figure this out, just dump everything into TXB0 for now
  return 0;
}

uint8_t MCP2515_GetFullRxBuffer(){
  SPI_TransmitByte(MCP_READSTATUS);
  return SPI_RecieveByte() && 0x03;
}

// Mode: what to set mode to after reset
void MCP2515_Reset(uint8_t mode){
  CAN_SELECT;
  SPI_TransmitByte(MCP_RESET);
  CAN_DESELECT;
}

void MCP2515_BitModify(uint8_t addr, uint8_t mask, uint8_t data){
  CAN_SELECT;
  SPI_TransmitByte(MCP_BITMODIFY);
  SPI_TransmitByte(addr);
  SPI_TransmitByte(mask);
  SPI_TransmitByte(data);
  CAN_DESELECT;
}

// See Figure 12.2
uint8_t MCP2515_Read(uint8_t addr){
  uint8_t data;
  CAN_SELECT;   // Begin transmission
  SPI_TransmitByte(MCP_READ);
  SPI_TransmitByte(addr);
  data = SPI_RecieveByte();
  CAN_DESELECT; // End transmission
  return data;
}

void MCP2515_Write(uint8_t addr, uint8_t data){
  CAN_SELECT;
  SPI_TransmitByte(MCP_WRITE);
  SPI_TransmitByte(addr);
  SPI_TransmitByte(data);
  CAN_DESELECT;
}

void MCP2515_RTS(uint8_t txBuff){
  uint8_t cmd;

  if (txBuff == 0){
    cmd = MCP_RTSTXB0;
  }
  else if (txBuff == 1){
    cmd = MCP_RTSTXB1;
  }
  else{
    cmd = MCP_RTSTXB2;
  }
  CAN_SELECT;
  SPI_TransmitByte(cmd);
  CAN_DESELECT;
}
