/*
  MCP_can.cpp
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
  2017 Copyright (c) Cory J. Fowler  All Rights Reserved.
  2018 Copyright (c) Patrick J. Kennedy   All rights reserved

  Author: Loovee
  Contributor: Cory J. Fowler
  Contributor: Patrick J. Kennedy
  25/03/2018

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS forA PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License formore details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA

  Description of changes:
    PJK: Removed Arduino libraries and functionality forgeneric AVR operation
 */
#include <avr/io.h>
#include "MCP2515.h"
#include "USART.h"
#include "SPI.h"

/*********************************************************************************************************
** Function name:           MCP2515_reset
** Descriptions:            Performs a software reset
*********************************************************************************************************/
void MCP2515_Reset(void)
{
    MCP2515_SELECT();
    SPI_ExchangeByte(MCP_RESET);
    MCP2515_UNSELECT();
    _delay_ms(1);
}

/*********************************************************************************************************
** Function name:           MCP2515_ReadRegister
** Descriptions:            Read data register
*********************************************************************************************************/
uint8_t MCP2515_ReadRegister(const uint8_t address)
{
    uint8_t ret;

    MCP2515_SELECT();
    SPI_ExchangeByte(MCP_READ);
    SPI_ExchangeByte(address);
    ret = SPI_ReceiveByte();
    MCP2515_UNSELECT();

    return ret;
}

/*********************************************************************************************************
** Function name:           MCP2515_ReadRegisterS
** Descriptions:            Reads sucessive data registers
*********************************************************************************************************/
void MCP2515_ReadRegisterS(const uint8_t address, uint8_t values[], const uint8_t n)
{
    uint8_t i;
    MCP2515_SELECT();
    SPI_ExchangeByte(MCP_READ);
    SPI_ExchangeByte(address);

    // MCP2515 has auto-increment of address-pointer
    for(i=0; i<n; i++) {
      values[i] = SPI_ReceiveByte();
    }

    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           MCP2515_setRegister
** Descriptions:            Sets data register
*********************************************************************************************************/
void MCP2515_SetRegister(const uint8_t address, const uint8_t value)
{
    MCP2515_SELECT();
    SPI_ExchangeByte(MCP_WRITE);
    SPI_ExchangeByte(address);
    SPI_ExchangeByte(value);
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           MCP2515_setRegisterS
** Descriptions:            Sets sucessive data registers
*********************************************************************************************************/
void MCP2515_SetRegisterS(const uint8_t address, const uint8_t values[], const uint8_t n)
{
    uint8_t i;
    MCP2515_SELECT();
    SPI_ExchangeByte(MCP_WRITE);
    SPI_ExchangeByte(address);
       
    for(i=0; i<n; i++){
      SPI_ExchangeByte(values[i]);
    }

    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           MCP2515_ModifyRegister
** Descriptions:            Sets specific bits of a register
*********************************************************************************************************/
void MCP2515_ModifyRegister(const uint8_t address, const uint8_t mask, const uint8_t data)
{
    MCP2515_SELECT();
    SPI_ExchangeByte(MCP_BITMOD);
    SPI_ExchangeByte(address);
    SPI_ExchangeByte(mask);
    SPI_ExchangeByte(data);
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           MCP2515_readStatus
** Descriptions:            Reads status register
*********************************************************************************************************/
uint8_t MCP2515_ReadStatus(void)
{
    uint8_t i;
    MCP2515_SELECT();
    SPI_ExchangeByte(MCP_READ_STATUS);
    i = SPI_ReceiveByte();
    MCP2515_UNSELECT();
    return i;
}

/*********************************************************************************************************
** Function name:           MCP2515_SetCANControlMode
** Descriptions:            Set control mode
*********************************************************************************************************/
uint8_t MCP2515_SetCANControlMode(const uint8_t newmode)
{
    uint8_t i;

    MCP2515_ModifyRegister(MCP_CANCTRL, MODE_MASK, newmode);

    i = MCP2515_ReadRegister(MCP_CANCTRL);
    i &= MODE_MASK;

    if(i == newmode)
        return MCP2515_OK;

    return MCP2515_FAIL;
}

/*********************************************************************************************************
** Function name:           MCP2515_configRate
** Descriptions:            Set baudrate
*********************************************************************************************************/
uint8_t MCP2515_ConfigRate(const uint8_t canSpeed, const uint8_t canClock)
{
    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock)
    {
        case (MCP_8MHZ):
        switch (canSpeed) 
        {
            case (CAN_5KBPS):                                               //   5KBPS                  
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10KBPS                  
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20KBPS                  
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

            case (CAN_31K25BPS):                                            //  31.25KBPS                  
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

            case (CAN_33K3BPS):                                             //  33.33KBPS                  
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;
        
            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;  

            default:
            set = 0;
	    return MCP2515_FAIL;
            break;
        }
        break;

        case (MCP_16MHZ):
        switch (canSpeed) 
        {
            case (CAN_5KBPS):                                               //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

            case (CAN_33K3BPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;
        
            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;  

            default:
            set = 0;
	    return MCP2515_FAIL;
            break;
        }
        break;
        
        case (MCP_20MHZ):
        switch (canSpeed) 
        {
            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;
        
            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;  

            default:
            set = 0;
            return MCP2515_FAIL;
            break;
        }
        break;
        
        default:
        set = 0;
	return MCP2515_FAIL;
        break;
    }

    if(set) {
        MCP2515_SetRegister(MCP_CNF1, cfg1);
        MCP2515_SetRegister(MCP_CNF2, cfg2);
        MCP2515_SetRegister(MCP_CNF3, cfg3);
        return MCP2515_OK;
    }
     
    return MCP2515_FAIL;
}

/*********************************************************************************************************
** Function name:           MCP2515_InitCANBuffers
** Descriptions:            Initialize Buffers, Masks, and Filters
*********************************************************************************************************/
void MCP2515_InitCANBuffers(void)
{
    uint8_t i, a1, a2, a3;
    
    uint8_t std = 0;               
    uint8_t ext = 1;
    uint32_t ulMask = 0x00, ulFilt = 0x00;


    MCP2515_WriteMF(MCP_RXM0SIDH, ext, ulMask);			/*Set both masks to 0           */
    MCP2515_WriteMF(MCP_RXM1SIDH, ext, ulMask);			/*Mask register ignores ext bit */
    
                                                                        /* Set all filters to 0         */
    MCP2515_WriteMF(MCP_RXF0SIDH, ext, ulFilt);			/* RXB0: extended               */
    MCP2515_WriteMF(MCP_RXF1SIDH, std, ulFilt);			/* RXB1: standard               */
    MCP2515_WriteMF(MCP_RXF2SIDH, ext, ulFilt);			/* RXB2: extended               */
    MCP2515_WriteMF(MCP_RXF3SIDH, std, ulFilt);			/* RXB3: standard               */
    MCP2515_WriteMF(MCP_RXF4SIDH, ext, ulFilt);
    MCP2515_WriteMF(MCP_RXF5SIDH, std, ulFilt);

                                                                        /* Clear, deactivate the three  */
                                                                        /* transmit buffers             */
                                                                        /* TXBnCTRL -> TXBnD7           */
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for(i = 0; i < 14; i++) {                                          /* in-buffer loop               */
        MCP2515_SetRegister(a1, 0);
        MCP2515_SetRegister(a2, 0);
        MCP2515_SetRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    MCP2515_SetRegister(MCP_RXB0CTRL, 0);
    MCP2515_SetRegister(MCP_RXB1CTRL, 0);
}

/*********************************************************************************************************
** Function name:           MCP2515_Init
** Descriptions:            Initialize the controller
*********************************************************************************************************/
uint8_t MCP2515_Init(const uint8_t canIDMode, const uint8_t canSpeed, const uint8_t canClock)
{
  uint8_t res;

  MCP2515_Reset();

  res = MCP2515_SetCANControlMode(MODE_CONFIG);
  if(res > 0)
  {
#if DEBUG_MODE
    PrintString("Entering Configuration Mode Failure...\r\n");
#endif
    return res;
  }
#if DEBUG_MODE
  PrintString("Entering Configuration Mode Successful!\r\n");
#endif

  // Set Baudrate
  if(MCP2515_ConfigRate(canSpeed, canClock))
  {
#if DEBUG_MODE
    PrintString("Setting Baudrate Failure...\r\n");
#endif
    return res;
  }
#if DEBUG_MODE
  PrintString("Setting Baudrate Successful!\r\n");
#endif

  if(res == MCP2515_OK) {

                                                                        /* init canbuffers              */
      MCP2515_InitCANBuffers();
                                                                      /* interrupt mode               */
      MCP2515_SetRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);

	//Sets BF pins as GPO
	MCP2515_SetRegister(MCP_BFPCTRL,MCP_BxBFS_MASK | MCP_BxBFE_MASK);
	//Sets RTS pins as GPI
	MCP2515_SetRegister(MCP_TXRTSCTRL,0x00);

        switch(canIDMode)
        {
            case (MCP_ANY):
            MCP2515_ModifyRegister(MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
            MCP2515_ModifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_ANY);
            break;
/*          The followingn two functions of the MCP2515 do not work, there is a bug in the silicon.
            case (MCP_STD): 
            MCP2515_ModifyRegister(MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_STD | MCP_RXB_BUKT_MASK);
            MCP2515_ModifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_STD);
            break;

            case (MCP_EXT): 
            MCP2515_ModifyRegister(MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_EXT | MCP_RXB_BUKT_MASK);
            MCP2515_ModifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_EXT);
            break;
*/
            case (MCP_STDEXT): 
            MCP2515_ModifyRegister(MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK);
            MCP2515_ModifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_STDEXT);
            break;
    
            default:
#if DEBUG_MODE        
            PrintString("Setting ID Mode Failure...\r\n");
#endif           
            return MCP2515_FAIL;
            break;
}    


        res = MCP2515_SetCANControlMode(MCP_LOOPBACK);
        if(res)
        {
#if DEBUG_MODE        
          PrintString("Returning to Previous Mode Failure...\r\n");
#endif
          return res;
        }

    }
    return res;

}

/*********************************************************************************************************
** Function name:           MCP2515_write_id
** Descriptions:            Write CAN ID
*********************************************************************************************************/
void MCP2515_WriteID(const uint8_t MCP_addr, const uint8_t ext, const uint32_t id)
{
    uint16_t canid;
    uint8_t tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if(ext == 1)
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5);
    }
    else 
    {
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 3);
        tbufdata[MCP_SIDL] = (uint8_t) ((canid & 0x07) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    
    MCP2515_SetRegisterS(MCP_addr, tbufdata, 4);
}

/*********************************************************************************************************
** Function name:           MCP2515_WriteMF
** Descriptions:            Write Masks and Filters
*********************************************************************************************************/
void MCP2515_WriteMF(const uint8_t MCP_addr, const uint8_t ext, const uint32_t id)
{
    uint16_t canid;
    uint8_t tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if(ext == 1)
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5);
    }
    else 
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) ((canid & 0x07) << 5);
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 3);
    }
    
    MCP2515_SetRegisterS(MCP_addr, tbufdata, 4);
}

/*********************************************************************************************************
** Function name:           MCP2515_read_id
** Descriptions:            Read CAN ID
*********************************************************************************************************/
void MCP2515_ReadID(const uint8_t MCP_addr, uint8_t* ext, uint32_t* id)
{
    uint8_t tbufdata[4];

    *ext = 0;
    *id = 0;

    MCP2515_ReadRegisterS(MCP_addr, tbufdata, 4);

    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if((tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M)
    {
                                                                        /* extended id                  */
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id<<8) + tbufdata[MCP_EID8];
        *id = (*id<<8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

/*********************************************************************************************************
** Function name:           MCP2515_WriteCANMsg
** Descriptions:            Write message
*********************************************************************************************************/
void MCP2515_WriteCANMsg(const uint8_t buffer_sidh_addr, uint8_t data[], uint32_t id, uint8_t dlc, uint8_t extFlag, uint8_t rtrFlag)
{
    uint8_t MCP_addr;
    MCP_addr = buffer_sidh_addr;
    MCP2515_SetRegisterS(MCP_addr+5, data, dlc);                /* write data bytes             */
	
    if(rtrFlag == 1)                                          /* if RTR set bit in byte       */
        dlc |= MCP_RTR_MASK;

    MCP2515_SetRegister((MCP_addr+4), dlc);                      /* write the RTR and DLC        */
    MCP2515_WriteID(MCP_addr, extFlag, id);                      /* write CAN id                 */
}

/*********************************************************************************************************
** Function name:           MCP2515_read_canMsg
** Descriptions:            Read message
*********************************************************************************************************/
void MCP2515_ReadCANMsg(const uint8_t buffer_sidh_addr, uint8_t data[], uint32_t *id, uint8_t *dlc, uint8_t *extFlag, uint8_t *rtrFlag)
{
    uint8_t MCP_addr, ctrl;

    MCP_addr = buffer_sidh_addr;

    MCP2515_ReadID(MCP_addr, extFlag, id);

    ctrl = MCP2515_ReadRegister(MCP_addr-1);
    *dlc = MCP2515_ReadRegister(MCP_addr+4);

    if(ctrl & 0x08)
        *rtrFlag = 1;
    else
        *rtrFlag = 0;

    *dlc &= MCP_DLC_MASK;
    MCP2515_ReadRegisterS(MCP_addr+5, data, *dlc);
}

/*********************************************************************************************************
** Function name:           MCP2515_getNextFreeTXBuf
** Descriptions:            Send message
*********************************************************************************************************/
uint8_t MCP2515_GetNextFreeTxBuf(uint8_t *txbuf_n)                 /* get Next free txbuf          */
{
    uint8_t res, i, ctrlval;
    uint8_t ctrlregs[MCP_N_TXBUFFERS] = {MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL};

    res = MCP_ALLTXBUSY;
    *txbuf_n = 0x00;

                                                                        /* check all 3 TX-Buffers       */
    for(i=0; i<MCP_N_TXBUFFERS; i++) {
        ctrlval = MCP2515_ReadRegister(ctrlregs[i]);
        if((ctrlval & MCP_TXB_TXREQ_M) == 0) {
            *txbuf_n = ctrlregs[i]+1;                                   /* return SIDH-address of Buffer*/
            
            res = MCP2515_OK;
            return res;                                                 /* ! function exit              */
        }
    }
    return res;
}

/*********************************************************************************************************
** Function name:           begin
** Descriptions:            Public function to declare controller initialization parameters.
*********************************************************************************************************/
uint8_t MCP2515_Begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset)
{
    uint8_t res;

    SPI_Init();
    res = MCP2515_Init(idmodeset, speedset, clockset);
    if(res == MCP2515_OK)
        return CAN_OK;
    
    return CAN_FAILINIT;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            Public function to set mask(s).
*********************************************************************************************************/
uint8_t MCP2515_InitMask(uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
#if DEBUG_MODE
    PrintString("Starting to Set Mask!\r\n");
#endif
    res = MCP2515_SetCANControlMode(MODE_CONFIG);
    if(res > 0){
#if DEBUG_MODE
	PrintString("Entering Configuration Mode Failure...\r\n"); 
#endif
	return res;
     }
    
    if(num == 0){
        MCP2515_WriteMF(MCP_RXM0SIDH, ext, ulData);

    }
    else if(num == 1){
        MCP2515_WriteMF(MCP_RXM1SIDH, ext, ulData);
    }
    else res =  MCP2515_FAIL;
    
    res = MCP2515_SetCANControlMode(MCP_LOOPBACK);
    if(res > 0){
#if DEBUG_MODE
	PrintString("Entering Previous Mode Failure...\r\nSetting Mask Failure...\r\n"); 
#endif
	return res;
    }
#if DEBUG_MODE
    PrintString("Setting Mask Successful!\r\n");
#endif
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            Public function to set filter(s).
*********************************************************************************************************/
uint8_t MCP2515_InitFilter(uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
#if DEBUG_MODE
    PrintString("Starting to Set Filter!\r\n");
#endif
    res = MCP2515_SetCANControlMode(MODE_CONFIG);
    if(res > 0)
    {
#if DEBUG_MODE
    PrintString("Enter Configuration Mode Failure...\r\n"); 
#endif
      return res;
    }
    
    switch(num)
    {
        case 0:
        MCP2515_WriteMF(MCP_RXF0SIDH, ext, ulData);
        break;

        case 1:
        MCP2515_WriteMF(MCP_RXF1SIDH, ext, ulData);
        break;

        case 2:
        MCP2515_WriteMF(MCP_RXF2SIDH, ext, ulData);
        break;

        case 3:
        MCP2515_WriteMF(MCP_RXF3SIDH, ext, ulData);
        break;

        case 4:
        MCP2515_WriteMF(MCP_RXF4SIDH, ext, ulData);
        break;

        case 5:
        MCP2515_WriteMF(MCP_RXF5SIDH, ext, ulData);
        break;

        default:
        res = MCP2515_FAIL;
    }
    
    res = MCP2515_SetCANControlMode(MCP_LOOPBACK);
    if(res > 0)
    {
#if DEBUG_MODE
    PrintString("Entering Previous Mode Failure...\r\nSetting Filter Failure...\r\n"); 
#endif
      return res;
    }
#if DEBUG_MODE
    PrintString("Setting Filter Successfull!\r\n");
#endif
    
    return res;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            Send message
*********************************************************************************************************/
uint8_t MCP2515_SendMsg(uint8_t *pData, uint32_t id, uint8_t dlc, uint8_t extFlag, uint8_t rtrFlag)
{
    uint8_t res, res1, txbuf_n, i;
    uint8_t data[MAX_CHAR_IN_MESSAGE];
    uint16_t uiTimeOut = 0;

    for(i = 0; i<MAX_CHAR_IN_MESSAGE; i++)
      data[i] = *(pData+i);

    do {
        res = MCP2515_GetNextFreeTXBuf(&txbuf_n);                       /* info = addr.                 */
        uiTimeOut++;
    } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    if(uiTimeOut == TIMEOUTVALUE) 
    {   
        return CAN_GETTXBFTIMEOUT;                                      /* get tx buff time out         */
    }
    uiTimeOut = 0;
    MCP2515_WriteCANMsg(txbuf_n, data, id, dlc, extFlag, rtrFlag);
    MCP2515_ModifyRegister(txbuf_n-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M);
    
    do
    {
        uiTimeOut++;        
        res1 = MCP2515_ReadRegister(txbuf_n-1);                         /* read send buff ctrl reg 	*/
        res1 = res1 & 0x08;                               		
    } while (res1 && (uiTimeOut < TIMEOUTVALUE));   
    
    if(uiTimeOut == TIMEOUTVALUE)                                       /* send msg timeout             */	
        return CAN_SENDMSGTIMEOUT;
    
    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            Read message
*********************************************************************************************************/
uint8_t MCP2515_ReadMsg(uint8_t *pData, uint32_t *id, uint8_t *dlc, uint8_t *extFlag, uint8_t *rtrFlag)
{
    uint8_t stat, res;

    stat = MCP2515_ReadStatus();

    if(stat & MCP_STAT_RX0IF)                                        /* Msg in Buffer 0              */
    {
        MCP2515_ReadCANMsg(MCP_RXBUF_0, pData, id, dlc, extFlag, rtrFlag);
        MCP2515_ModifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if(stat & MCP_STAT_RX1IF)                                   /* Msg in Buffer 1              */
    {
        MCP2515_ReadCANMsg(MCP_RXBUF_1, pData, id, dlc, extFlag, rtrFlag);
        MCP2515_ModifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else 
        res = CAN_NOMSG;
    
    return res;
}



/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            Public function, Checks forreceived data.  (Used if not using the interrupt output)
*********************************************************************************************************/
uint8_t MCP2515_CheckReceive(void)
{
    uint8_t res;
    res = MCP2515_ReadStatus();                                         /* RXnifin Bit 1 and 0         */
    if(res & MCP_STAT_RXIF_MASK)
        return CAN_MSGAVAIL;
    else 
        return CAN_NOMSG;
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            Public function, Returns error register data.
*********************************************************************************************************/
uint8_t MCP2515_CheckError(void)
{
    uint8_t eflg = MCP2515_ReadRegister(MCP_EFLG);

    if(eflg & MCP_EFLG_ERRORMASK)
        return CAN_CTRLERROR;
    else
        return CAN_OK;
}

/*********************************************************************************************************
** Function name:           getError
** Descriptions:            Returns error register value.
*********************************************************************************************************/
uint8_t MCP2515_GetError(void)
{
    return MCP2515_ReadRegister(MCP_EFLG);
}

/*********************************************************************************************************
** Function name:           MCP2515_errorCountRX
** Descriptions:            Returns REC register value
*********************************************************************************************************/
uint8_t MCP2515_ErrorCountRX(void)
{
    return MCP2515_ReadRegister(MCP_REC);
}

/*********************************************************************************************************
** Function name:           MCP2515_errorCountTX
** Descriptions:            Returns TEC register value
*********************************************************************************************************/
uint8_t MCP2515_ErrorCountTx(void)
{
    return MCP2515_ReadRegister(MCP_TEC);
}

/*********************************************************************************************************
** Function name:           MCP2515_EnableOneShotTX
** Descriptions:            Enables one shot transmission mode
*********************************************************************************************************/
uint8_t MCP2515_EnableOneShotTx(uint8_t enable)
{
  if(enable != 0) {
    MCP2515_ModifyRegister(MCP_CANCTRL, MODE_ONESHOT, MODE_ONESHOT);
    if((MCP2515_ReadRegister(MCP_CANCTRL) & MODE_ONESHOT) != MODE_ONESHOT)
      return CAN_FAIL;
    else
      return CAN_OK;
  }
  else {
    MCP2515_ModifyRegister(MCP_CANCTRL, MODE_ONESHOT, 0);
    if((MCP2515_ReadRegister(MCP_CANCTRL) & MODE_ONESHOT) != 0)
      return CAN_FAIL;
    else
      return CAN_OK;
  }
}

/*********************************************************************************************************
** Function name:           MCP2515_abortTX
** Descriptions:            Aborts any queued transmissions
*********************************************************************************************************/
uint8_t MCP2515_AbortTx(void)
{
    MCP2515_ModifyRegister(MCP_CANCTRL, ABORT_TX, ABORT_TX);
	
    // Maybe check to see if the TX buffer transmission request bits are cleared instead?
    if((MCP2515_ReadRegister(MCP_CANCTRL) & ABORT_TX) != ABORT_TX)
	    return CAN_FAIL;
    else
	    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           setGPO
** Descriptions:            Public function, Checks forr
*********************************************************************************************************/
uint8_t MCP2515_SetGPO(uint8_t data)
{
    MCP2515_ModifyRegister(MCP_BFPCTRL, MCP_BxBFS_MASK, (data<<4));
	    
    return 0;
}

/*********************************************************************************************************
** Function name:           getGPI
** Descriptions:            Public function, Checks forr
*********************************************************************************************************/
uint8_t MCP2515_GetGPI(void)
{
    uint8_t res;
    res = MCP2515_ReadRegister(MCP_TXRTSCTRL) & MCP_BxRTS_MASK;
    return (res >> 3);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
