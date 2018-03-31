/*
  MCP_can.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
  2017 Copyright (c) Cory J. Fowler  All Rights Reserved.

  Author:Loovee
  Contributor: Cory J. Fowler
  2017-09-25
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "MCP2515_def.h"
#define MAX_CHAR_IN_MESSAGE 8


/*********************************************************************************************************
 *  MCP2515 driver function 
 *********************************************************************************************************/
void MCP2515_Reset(void);                                           // Soft Reset MCP2515
uint8_t MCP2515_ReadRegister(const uint8_t address);                    // Read MCP2515 register
void MCP2515_ReadRegisterS(const uint8_t address, uint8_t values[], const uint8_t n);
void MCP2515_SetRegister(const uint8_t address, const uint8_t value);
void MCP2515_SetRegisterS(const uint8_t address, const uint8_t values[], const uint8_t n);
void MCP2515_InitCANBuffers(void);
void MCP2515_ModifyRegister(const uint8_t address, const uint8_t mask, const uint8_t data);
uint8_t MCP2515_ReadStatus(void);                                     // Read MCP2515 Status
uint8_t MCP2515_SetCANControlMode(const uint8_t newmode);                 // Set mode
uint8_t MCP2515_ConfigRate(const uint8_t canSpeed, const uint8_t canClock);
uint8_t MCP2515_Init(const uint8_t canIDMode, const uint8_t canSpeed, const uint8_t canClock);
void MCP2515_WriteMF(const uint8_t MCP_addr, const uint8_t ext, const uint32_t id);
void MCP2515_WriteID(const uint8_t MCP_addr, const uint8_t ext, const uint32_t id);
void MCP2515_ReadID(const uint8_t MCP_addr, uint8_t* ext, uint32_t* id);
void MCP2515_WriteCANMsg(const uint8_t buffer_sidh_addr, uint8_t data[], uint32_t id, uint8_t dlc, uint8_t extFlag, uint8_t rtrFlag);
void MCP2515_ReadCANMsg(const uint8_t buffer_sidh_addr, uint8_t data[], uint32_t *id, uint8_t *dlc, uint8_t *extFlag, uint8_t *rtrFlag);
uint8_t MCP2515_GetNextFreeTxBuf(uint8_t *txbuf_n);                     // Find empty transmit buffer

/*********************************************************************************************************
*  CAN operator function
*********************************************************************************************************/

uint8_t MCP2515_ReadMsg(uint8_t data[], uint32_t *id, uint8_t *dlc, uint8_t *extFlag, uint8_t *rtrFlag);
uint8_t MCP2515_SendMsg(uint8_t data[], uint32_t id, uint8_t dlc, uint8_t extFlag, uint8_t rtrFlag);

uint8_t MCP2515_Begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset);   // Initialize controller parameters
uint8_t MCP2515_InitMask(uint8_t num, uint8_t ext, uint32_t ulData);            // Initialize Mask(s)
uint8_t MCP2515_InitFilter(uint8_t num, uint8_t ext, uint32_t ulData);          // Initialize Filter(s)
uint8_t MCP2515_SetMode(uint8_t opMode);                                        // Set operational mode
uint8_t MCP2515_CheckReceive(void);                                           // Check for received data
uint8_t MCP2515_CheckError(void);                                             // Check for errors
uint8_t MCP2515_GetError(void);                                               // Check for errors
uint8_t MCP2515_ErrorCountRX(void);                                           // Get error count
uint8_t MCP2515_ErrorCountTX(void);                                           // Get error count
uint8_t MCP2515_OneShotTX(uint8_t enable);                                    // Enable one-shot transmission
uint8_t MCP2515_AbortTX(void);                                                // Abort queued transmission(s)
uint8_t MCP2515_SetGPO(uint8_t data);                                         // Sets GPO
uint8_t MCP2515_GetGPI(void);                                                 // Reads GPI

#endif
/*********************************************************************************************************
 *  END FILE
 *********************************************************************************************************/
