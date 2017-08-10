//-----------------------------------------------------------------
//  MCP2515.h
//  Patrick Kennedy
//
//  Support file for MCP2515 operation
//------------------------------------------------------------------

#ifndef MCP2515_H
#define MCP2515_H

#include "MCP2515_def.h"
#include "PinDefinitions.h"

// Receive buffer status
#define DATA_NONE 0
#define DATA_RX0  1
#define DATA_RX1  2
#define DATA_RX0RX1 3

// MCP2515 OPERATION MODES
#define MODE_NORMAL 0
#define MODE_SLEEP  1
#define MODE_LOOPBACK 2
#define MODE_LISTEN 3
#define MODE_CONFIG 4

// CAN MESSAGE TYPES
#define MSG_STD 0   // CAN message with standard 11-bit ID
#define MSG_RTR 1   // CAN remote transfer request with std ID
#define MSG_EXT 2   // CAN message with extended 29-bit ID
#define MSG_EXTRTR 3  // CAN remote transfer request with ext ID

#define MCP2515_OK  0   // No MCP2515 error
#define MCP2515_ERR 1   // MCP2515 error

void MCP2515_Init();

void MCP2515_SendCANMessage(uint8_t type, uint32_t ID, uint64_t data, uint8_t length);

void MCP2515_ReceiveCANMessage(uint8_t *type, uint32_t *ID, uint64_t *data);

void MCP2515_LoadID(uint8_t txBuff, uint8_t type, uint32_t ID, uint8_t length);

void MCP2515_LoadData(uint8_t txBuff, uint64_t data);

uint8_t MCP2515_SetCANMode(uint8_t reqMode);

uint8_t MCP2515_GetEmptyTxBuffer();

uint8_t MCP2515_GetFullRxBuffer();

void MCP2515_Reset(uint8_t mode);

void MCP2515_BitModify(uint8_t addr, uint8_t data, uint8_t mask);

uint8_t MCP2515_Read(uint8_t addr);

void MCP2515_Write(uint8_t addr, uint8_t data);

void MCP2515_RTS(uint8_t txBuff);



#endif // MCP2515_H
