/*! \file Main.c \brief GKDAQ Main File. */
//-----------------------------------------------------------------------------
//  Filename   : Main.c
//  Title      : Main function for GKDAQ slave nodes
//  Author     : Patrick Kennedy (PK3)
//  Created    : 08/02/2016
//  Modified   : 02/26/2018
//  Version    : 0.2
//  Description:
//     Contains all functionalty for slave node in GKDAQ system
//-----------------------------------------------------------------------------

/*== PREPROCESSOR STATEMENTS ==*/

// Import AVR Libraries
#include <avr/io.h>       // Used to allow I/O
#include <util/delay.h>   // Used to set call wait() function
#include <avr/eeprom.h>   // Used to read/write to AVR EEPROM
#include <avr/interrupt.h>  // Used to process hardware interrupts

// Import Core Functionality Code
#include "lib/PinDefinitions.h"   // AVR pin definitions
#include "lib/MCP2515.c"  // MCP2515 SPI commands
#include "lib/USART.c"    // USART register controls
#include "lib/SPI.c"      // SPI register controls
#include "lib/Analog.c"   // Analog register controls
#include "lib/Sensors.c"  // Supported sensors

// Program options
#define WRITE_VALUES_TO_EEPROM 1
#define ENABLE_EEPROM 0

/*== GLOBAL VARIABLES ==*/
uint8_t NodeAddress;
volatile int CALIBRATION_MODE; // Calib mode flag
volatile int CAN_RCV0_FLAG;    // Set on CAN msg rcv
volatile int CAN_RCV1_FLAG;    // Set on can msg rcv

// Enumerations
enum IOPORTS{
    IO_A0 = 0,
    IO_A1 = 1,
    IO_A2 = 2,
    IO_A3 = 3,
    IO_D0 = 4,
    IO_I2C = 5,
    IO_UART = 6,
    IO_SPI = 7
};
enum MEASUREMENTS{
    DATA_NONE     = 0,  // No measurement
    DATA_ENGINETEMP        = 1,  // AD8495
    DATA_EXHAUSTTEMP       = 2,  // AD8495
    DATA_ENGINESPEED       = 3,  // PJK0010
    DATA_AXLESPEED         = 4,  // PJK0020
    DATA_THROTTLEPOSITION  = 5,  // PJK0030
    DATA_BRAKEPOSITION     = 6,  // PJK0040
    DATA_STEERINGANGLE     = 7,  // PJK0050
    DATA_AMBIENTTEMP       = 8,  // TMP36
    DATA_ACCGYRO           = 9,  // MPU6050
    DATA_ACCGYROMAG        = 10, // MPU9250
    DATA_GPS               = 11, // MTK3339
    DATA_TEMPHUMIDITY      = 12, // Si7021
    DATA_TESTPOT           = 255 // Test potentiometer
};
enum CANIDs{
    CANID_CAL_START        = 0x01,
    CANID_CAL_CHANGEADDR   = 0x02,
    CANID_CAL_CHANGEIO     = 0x03,
    CANID_CAL_RESET        = 0x04,
    CANID_CAL_EXIT         = 0x0F,
    CANID_ENGINETEMP       = 0x10,
    CANID_EXHAUSTTEMP      = 0x11,
    CANID_ENGINESPEED      = 0x12,
    CANID_GPS              = 0x20,
    CANID_ACCELERATION     = 0x21,
    CANID_HEADING          = 0x22,
    CANID_AXLESPEED        = 0x23,
    CANID_GYRATION         = 0x24,
    CANID_THROTTLEPOSITION = 0x30,
    CANID_BRAKEPOSITION    = 0x31,
    CANID_STEERINGANGLE    = 0x32,
    CANID_AMBIENTTEMP      = 0x40,
    CANID_HUMIDITY         = 0x41,
    CANID_TESTPOT          = 0x7F
};
enum EEPROM{
  EE_NODEADDR = 0x00,
  EE_IO_A0 = 0x01,
  EE_IO_A1 = 0x02,
  EE_IO_A2 = 0x03,
  EE_IO_A3 = 0x04,
  EE_IO_D0 = 0x05,
  EE_IO_I2C = 0x06,
  EE_IO_UART = 0x07,
  EE_IO_SPI = 0x08
};

// Function Declarations
void InitializeSensors(uint8_t*);
void CalibrationRoutine(void);
void MeasurementRoutine(uint8_t, uint8_t);
void ToByteArray(uint64_t, uint8_t*);

// Global CAN
uint8_t canData[MAX_CHAR_IN_MESSAGE];
uint32_t canID;
uint8_t canDLC;
uint8_t canExtFlag;
uint8_t canRTRFlag;
uint8_t mcpMode;
uint64_t data;      // Testing variable, delete in production code

int main(void) {
  /*== VARIABLE DECLARATIONS ==*/
  uint8_t ioData[8];

  /*== INITIALIZATION ROUTINE ==*/
  _delay_ms(100); // Delay for 100ms, allow all devices to wake up

  // Configure Hardware
  USART_Init();     // Configure serial
  SPI_Init();       // Configure SPI
  MCP2515_Init(MCP_STD, CAN_250KBPS, MCP_16MHZ);   // Only std IDs, 250kbaud bus, 16MHz crystal
  Analog_Init();    // Configure analog inputs

#if ENABLE_EEPROM
#if WRITE_VALUES_TO_EEPROM
  // Write default values to the EEPROM on power up
  eeprom_write_byte((uint8_t*) EE_NODEADDR, (uint8_t) 0);
  eeprom_write_byte((uint8_t*) EE_IO_A0, (uint8_t) 0);
  eeprom_write_byte((uint8_t*) EE_IO_A1, (uint8_t) 0);
  eeprom_write_byte((uint8_t*) EE_IO_A2, (uint8_t) 0);
  eeprom_write_byte((uint8_t*) EE_IO_A3, (uint8_t) 0);
  eeprom_write_byte((uint8_t*) EE_IO_D0, (uint8_t) 0);
  eeprom_write_byte((uint8_t*) EE_IO_I2C, (uint8_t) 0);
  eeprom_write_byte((uint8_t*) EE_IO_UART, (uint8_t) 0);
  eeprom_write_byte((uint8_t*) EE_IO_SPI, (uint8_t) 0);

  _delay_ms(250);   // Wait after EEPROM write
#endif
  // Read EEPROM Values
  NodeAddress = eeprom_read_byte((uint8_t*) EE_NODEADDR);   // Node address
  ioData[IO_A0] = eeprom_read_byte((uint8_t*) EE_IO_A0);      // Analog 0  (IO Port 0)
  ioData[IO_A1] = eeprom_read_byte((uint8_t*) EE_IO_A1);      // Analog 1  (IO Port 1)
  ioData[IO_A2] = eeprom_read_byte((uint8_t*) EE_IO_A2);      // Analog 2  (IO Port 2)
  ioData[IO_A3] = eeprom_read_byte((uint8_t*) EE_IO_A3);      // Analog 3  (IO Port 3)
  ioData[IO_D0] = eeprom_read_byte((uint8_t*) EE_IO_D0);      // Digital 0 (IO Port 4)
  ioData[IO_I2C] = eeprom_read_byte((uint8_t*) EE_IO_I2C);    // I2C (TWI) (IO Port 5)
  ioData[IO_UART] = eeprom_read_byte((uint8_t*) EE_IO_UART);  // UART      (IO Port 6)
  ioData[IO_SPI] = eeprom_read_byte((uint8_t*) EE_IO_SPI);    // SPI       (IO Port 7)
#else
  // Skip EEPROM operations (for debugging)
  ioData[IO_A0] = DATA_NONE;
  ioData[IO_A1] = DATA_TESTPOT;
  ioData[IO_A2] = DATA_NONE;
  ioData[IO_A3] = DATA_NONE;
  ioData[IO_D0] = DATA_NONE;
  ioData[IO_I2C] = DATA_NONE;
  ioData[IO_UART] = DATA_NONE;
  ioData[IO_SPI] = DATA_NONE;
#endif

  // Test ToByteArray
  data = 0x1122334455667788;
  ToByteArray(data, canData);
  PrintHexByte(canData[0]);
  PrintHexByte(canData[1]);
  PrintHexByte(canData[2]);
  PrintHexByte(canData[3]);
  PrintHexByte(canData[4]);
  PrintHexByte(canData[5]);
  PrintHexByte(canData[6]);
  PrintHexByte(canData[7]);

  // Loop
  while (1) {
    /*
    data = TestPot_GetValue(IO_A1);
    PrintDecimalWord(data);
    PrintString("\r\n");
    */
    //MeasurementRoutine(DATA_TESTPOT, IO_A1);
    _delay_ms(1000);
    /*
    ioPort++;

    // If looped through all io, repeat
    if(ioPort > IO_SPI){
      ioPort = IO_A0;
    }
    _delay_ms(250);
  }
  */
    // Never should reach here
    return 0;
  }
}

// Run specific initialization code for certain sensors
void InitializeSensors(uint8_t* pIOData){
  uint8_t ioData, i;

  for (i = 0; i <= 7; i++) {
    ioData = *(pIOData+i);   // Get next measurement

    if (ioData == DATA_NONE) {
      return;   // Break out early if no sensor connected
    } else if (ioData == DATA_ACCGYRO) {
      // TODO: ADD INIT CODE FOR MPU6050
    } else if (ioData == DATA_ACCGYROMAG) {
      // TODO: ADD INIT CODE FOR MPU9250
    } else if (ioData == DATA_GPS) {
      // Todo: Add init code for MTK3339
    }
  }
}

void CalibrationRoutine(){
  uint8_t rxBuffer;
  uint32_t msgID;
  uint64_t msgData;

  while(CALIBRATION_MODE != 0){
    if(CAN_RCV0_FLAG != 0 || CAN_RCV1_FLAG != 0){
      if(CAN_RCV0_FLAG != 0){
        rxBuffer = 0;
      }
      else{
        rxBuffer = 1;
      }

      // Read CAN ID
      // Read CAN data

      if(msgID == CANID_CAL_CHANGEADDR){
        // Change node addr based on msgdata
        msgData = msgData;
      }
      else if(msgID == CANID_CAL_CHANGEIO){
        // change io port measurement based on msgdata
      }
      else if(msgID == CANID_CAL_EXIT){
        CALIBRATION_MODE = 0;
      }
    }
  }
  return;
}

void MeasurementRoutine(uint8_t IOData, uint8_t IOPort){
  uint64_t rawData;
  uint8_t arrData[MAX_CHAR_IN_MESSAGE];
  if(IOData == DATA_NONE) {
    return;
  }
  switch(IOData){
    // Analog Engine Head Temperature Sensor
    case DATA_ENGINETEMP:

      break;

    // Analog Exhaust Gas Temperature Sensor
    case DATA_EXHAUSTTEMP:

      break;

    // Analog Tachometer (Engine Speed) Sensor
    case DATA_ENGINESPEED:

      break;

    // Analog Tachometer (Axle Speed) Sensor
    case DATA_AXLESPEED:

      break;

    // Analog Throttle Position Sensor
    case DATA_THROTTLEPOSITION:

      break;

    // Analog Brake Position Sensor
    case DATA_BRAKEPOSITION:

      break;

    // Analog Steering Angle Sensor
    case DATA_STEERINGANGLE:

      break;

    // Analog Ambient Temperature Sensor
    case DATA_AMBIENTTEMP:

      break;

    // MPU6050 (I2C)
    case DATA_ACCGYRO:

      break;

    // MPU9250
    case DATA_ACCGYROMAG:

      break;

    // MTK3339 GPS Module
    case DATA_GPS:

      break;

    // Si7021 Temperature/Humidity Sensor
    case DATA_TEMPHUMIDITY:

      break;

    // Test potentiometer
    case DATA_TESTPOT:
      rawData = TestPot_GetValue(IOPort);
      ToByteArray(rawData, arrData);
      PrintDecimalWord(data);
      PrintString("\r\n");
      MCP2515_SendMsg(arrData, CANID_TESTPOT, DLC_1, CAN_NO_EXT, CAN_NO_RTR);
      break;
  }
}

void ToByteArray(uint64_t value, uint8_t *array){
  uint8_t i;
  for(i = 0; i<MAX_CHAR_IN_MESSAGE; i++)
    *(array+i) = (uint8_t) (value >> (8*i));
}
// EOF