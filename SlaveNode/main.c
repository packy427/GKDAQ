/******************************************************************************
*  main.c
*  Patrick Kennedy
*
*  Main file for Slave Node
*
******************************************************************************/

/*------------------------
| PREPROCESSOR STATEMENTS |
 ------------------------*/
// Import AVR Libraries
#include <avr/io.h>       // Used to allow I/O
#include <util/delay.h>   // Used to set call wait() function
#include <avr/eeprom.h>   // Used to read/write to AVR EEPROM

// Import Core Functionality Code
#include "lib/MCP2515.c"  // MCP2515 SPI commands
#include "lib/USART.c"    // UART register controls
#include "lib/SPI.c"      // SPI register controls
#include "lib/Analog.c"   // Analog register controls
#include "lib/PinDefinitions.h"   // AVR pin definitions

// Import Sensor Libraries
// #include "the_ol'_razzle_dazzle.c"

// Set macros for MCP2515 SPI chip select
#define CAN_SELECT    SPI_PORT &= ~(1 << SPI_CAN_SS)
#define CAN_DESELECT  SPI_PORT |= (1 << SPI_CAN_SS)


/*-----------------
| GLOBAL VARIABLES |
 -----------------*/
uint8_t NodeAddress;

int main(void){
  /*----------------------
  | VARIABLE DECLARATIONS |
   ----------------------*/
  uint8_t sensors[8];

  /*-----------------------
  | INITIALIZATION ROUTINE |
   -----------------------*/

  // Wait
  _delay_ms(500); // Delay for 500ms, b/c it makes me feel better

  // Read EEPROM Values
  NodeAddress = eeprom_read_byte(0x00);   // Node address
  sensors[0] = eeprom_read_byte(0x01);    // Analog 0 Sensor  (IO Port 0)
  sensors[1] = eeprom_read_byte(0x02);    // Analog 1 Sensor  (IO Port 1)
  sensors[2] = eeprom_read_byte(0x03);    // Analog 2 Sensor  (IO Port 2)
  sensors[3] = eeprom_read_byte(0x04);    // Analog 3 Sensor  (IO Port 3)
  sensors[4] = eeprom_read_byte(0x05);    // Digital 0 Sensor (IO Port 4)
  sensors[5] = eeprom_read_byte(0x06);    // I2C (TWI) Sensor (IO Port 5)
  sensors[6] = eeprom_read_byte(0x07);    // UART Sensor      (IO Port 6)
  sensors[7] = eeprom_read_byte(0x08);    // SPI Sensor       (IO Port 7)

  // Make Decisions Based On EEPROM Values
  // or maybe not? See


  // Configure Hardware
  USART_Init();     // Configure serial
  SPI_Init();       // Configure SPI
  MCP2515_Init();   // Configure MCP2515    TODO: MAKE SURE INTERRUPT IS GENERATED ON MSG RCV
  Analog_Init();    // Configure analog inputs

  /*-------------
  | LOOP ROUTINE |
   -------------*/
  while(1){
    // Loop through each input and update value and transmit over CAN network
    for(i = 0; i < 8; i++){
      if(sensors[i] != 0){  // Check if input has no sensor attached
        UpdateAndSend(sensors[i], i);
      }
    }

    // Check for received messages, or maybe interrupt
    // Interrupt from MCP2515 located at INT1
  }
  return 0;
}

void UpdateAndSend(uint8_t sensor, uint8_t ioPort){
  uint64_t data;
  switch(sensor){

    // Analog Engine Head Temperature Sensor
    case 0x10:
      data = GetAnalogInput(ioPort);
      MCP2515_SendCANMessage(MSG_STD, 0x10, data, 2);
      break;

    // Analog Exhaust Gas Temperature Sensor
    case 0x11:
      data = GetAnalogInput(ioPort);
      MCP2515_SendCANMessage(MSG_STD, 0x11, data, 2);
      break;

    // Analog Tachometer (Engine Speed) Sensor
    case 0x12:
      data = GetAnalogInput(ioPort);
      MCP2515_SendCANMessage(MSG_STD, 0x12, data, 2);
      break;

    // L3GD20H + LSM303 9-Axis Acc/Gyr/Mag
    case 0x20:
      // TODO: Need code to support this
      // data = L_GetAccel()
      // MCP2515_SendCANMessage(MSG_STD, 0x20, data, 6);
      // data = L_GetGyro()
      // MCP2515_SendCANMessage(MSG_STD, 0x21, data, 6);
      // data = L_GetHeading()
      // MCP2515_SendCANMessage(MSG_STD, 0x22, data, 6);
      break;

    // MPU6050 Acc/Gyro (I2C)
    case 0x21:
      // TODO: Need code to support this, maybe ignore?
      // data = MPU_GetAccel()
      // MCP2515_SendCANMessage(MSG_STD, 0x20, data, 6);
      // data = MPU_GetGyro()
      // MCP2515_SendCANMessage(MSG_STD, 0x21, data, 6);
      break;

    // MPU6050 Acc/Gyro (SPI)
    case 0x22:
      // TODO: Need code, ignore?
      break;

    // MTK3339 GPS Module
    case 0x23:
      // TODO: Need code w/ interrupts, ignore? Handled w/ cgps on RPi
      break;

    // Analog Tachometer (Axle Speed) Sensor
    case 0x24:
      data = GetAnalogInput(ioPort);
      MCP2515_SendCANMessage(MSG_STD, 0x24, data, 2);
      break;

    // Analog Throttle Position Sensor
    case 0x30:
      data = GetAnalogInput(ioPort);
      MCP2515_SendCANMessage(MSG_STD, 0x30, data, 2);
      break;

    // Analog Brake Position Sensor
    case 0x31:
      data = GetAnalogInput(ioPort);
      MCP2515_SendCANMessage(MSG_STD, 0x31, data, 2);
      break;

    // Analog Steering Angle Sensor
    case 0x32:
      data = GetAnalogInput(ioPort);
      MCP2515_SendCANMessage(MSG_STD, 0x32, data, 2);
      break;

    // Analog Ambient Temperature Sensor
    case 0x40:
      data = GetAnalogInput(ioPort);
      MCP2515_SendCANMessage(MSG_STD, 0x40, data, 2);
      break;

    // Si7021 Temperature/Humidity Sensor
    case 0x41:
      // TODO: Need code, low priority
      // data = Si7021_GetTemperature()
      // MCP2515_SendCANMessage(MSG_STD, 0x40, data, 2);
      // data = Si7021_GetHumidity()
      // MCP2515_SendCANMessage(MSG_STD, 0x41, data, 2);
      break;
  }
}

void LoopbackTest(){
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
