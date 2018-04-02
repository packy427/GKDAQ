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
#include "lib/Sensors.c"  // Various supported sensors
#include "lib/MPU6050.c"
#include "lib/I2C.c"

// Program options
#define WRITE_VALUES_TO_EEPROM 1
#define ENABLE_EEPROM 0

/*== GLOBAL VARIABLES ==*/
uint8_t nodeAddress;        // Node address
uint8_t calibrationFlag;    // Calib mode flag
volatile uint8_t checkCANRcv;    // Set on 10ms timer overflow
volatile uint8_t isButtonPressed;    // Set pushbutton  timer overflow

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
void ChangeNodeAddress(uint8_t);
void ChangeNodeIO(uint8_t, uint8_t);

// Global CAN
uint64_t canData;
uint32_t canID;
uint8_t canDLC;
uint8_t canExtFlag;
uint8_t canRTRFlag;
uint8_t mcpMode;


// Interrupt Service Routines
ISR (TIMER0_OVF_vect){
    checkCANRcv = 1;
    TCNT0 = 177;   // Reset timer to ~10ms interrupt
}

ISR (TIMER1_OVF_vect){
    // Check D0 pin state
    if(D0_PIN & (1<<D0)){ // Pin high
      TCNT1 = 65520;   // Reset to 2 ms
    }
    else{   // Pin low
      isButtonPressed = 1;
      TCNT1 = 61630;   // Reset to 0.5s if pressed
    }
}


int main(void) {
  /*== VARIABLE DECLARATIONS ==*/
  uint8_t ioData[8];

  /*== INITIALIZATION ROUTINE ==*/
  // Pin modes
  D0_DDR &= ~(1 << D0);     // D0 as input

  _delay_ms(250); // Delay for 100ms, allow all devices to wake up

  // TIMER0 For CAN Polling
  TCNT0 = 177;     // For 10ms at 8MHz w/ 1024 prescale
  TCCR0A = 0x00;
  TCCR0B = (1<<CS10) | (1<<CS12);  // Timer mode with 1024 prescler
  TIMSK0 = (1 << TOIE0);   // Enable timer 0

  // TIMER1 For Button Polling
  TCNT1 = 65520;   // For 2ms at 8MHz w/ 1024 prescale
  TCCR1A = 0x00;
  TCCR1B = (1<<CS10) | (1<<CS12);  // Timer mode with 1024 prescler
  TIMSK1 |= (1 << TOIE1) ;   // Enable timer1 overflow interrupt(TOIE1)

  // Enable interrupts
  sei(); // Enable global interrupts by setting global interrupt enable bit in SREG

#if ENABLE_EEPROM
#if WRITE_DEFAULT_EEPROM_VALUES
  // Write default values to the EEPROM on power up
  eeprom_write_byte((uint8_t*) EE_NODEADDR, (uint8_t) 0);
  eeprom_write_byte((uint8_t*) EE_IO_A0, (uint8_t) DATA_NONE);
  eeprom_write_byte((uint8_t*) EE_IO_A1, (uint8_t) DATA_NONE);
  eeprom_write_byte((uint8_t*) EE_IO_A2, (uint8_t) DATA_NONE);
  eeprom_write_byte((uint8_t*) EE_IO_A3, (uint8_t) DATA_NONE);
  eeprom_write_byte((uint8_t*) EE_IO_D0, (uint8_t) DATA_NONE);
  eeprom_write_byte((uint8_t*) EE_IO_I2C, (uint8_t) DATA_NONE);
  eeprom_write_byte((uint8_t*) EE_IO_UART, (uint8_t) DATA_NONE);
  eeprom_write_byte((uint8_t*) EE_IO_SPI, (uint8_t) DATA_NONE);

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

  // Manually set measurements and skip EEPROM operations (for debugging)
  ioData[IO_A0] = DATA_NONE;
  ioData[IO_A1] = DATA_NONE;
  ioData[IO_A2] = DATA_TESTPOT;
  ioData[IO_A3] = DATA_NONE;
  ioData[IO_D0] = DATA_NONE;
  ioData[IO_I2C] = DATA_NONE;
  ioData[IO_UART] = DATA_NONE;
  ioData[IO_SPI] = DATA_NONE;
#endif

  // Configure Hardware
  USART_Init();     // Configure serial
  SPI_Init();       // Configure SPI
  MCP2515_Init(MCP_ANY, CAN_100KBPS, MCP_16MHZ);   // Only std IDs, 250kbaud bus, 16MHz crystal
  Analog_Init();    // Configure analog inputs
  InitializeSensors(ioData);  // Run any additional initialization routines required
  MCP2515_SetCANControlMode(MCP_NORMAL);  // Activate node on CAN bus
  MCP2515_EnableOneShotTx(1);
#if EN_DEBUG
  PrintString("<i> MCP2515 Normal Mode Set");
#endif

  // == MAIN LOOP == //
  while (1) {
    for(uint8_t io = IO_A0; io <= IO_SPI; io++){
      MeasurementRoutine(ioData[io], io);
      _delay_ms(5);
    }
    _delay_ms(200);
  }
  // Never should reach here
  return 0;
}


// Run specific initialization code for certain sensors
void InitializeSensors(uint8_t* pIOData){
  uint8_t ioData, i;

  for (i = 0; i <= 7; i++) {
    ioData = *(pIOData+i);   // Get next measurement

    if (ioData == DATA_NONE) {
      return;   // Break out early if no sensor connected
    } else if (ioData == DATA_ACCGYRO) {
        MPU6050_Init();
#if EN_DEBUG
        PrintString("<i> MPU6050 Init");
#endif
    } else if (ioData == DATA_ACCGYROMAG) {
      // TODO: ADD INIT CODE FOR MPU9250
    } else if (ioData == DATA_GPS) {
      // Todo: Add init code for MTK3339
    }
  }
}

void CalibrationRoutine(){
  uint8_t rxNodeAddr;
  uint64_t prevCANData = 0;
#if EN_DEBUG
  PrintString("<i> Entered Calibration Routine");
#endif
  rxNodeAddr = (uint8_t) canData & 0xFF;
  if(rxNodeAddr != nodeAddress){
    calibrationFlag = 0;    // If calibration mode isn't meant for this node, clear flag
  }
  while(calibrationFlag != 0) {
    MCP2515_ReadMsg(&canData, &canID, &canDLC, &canExtFlag, &canRTRFlag);
    if (prevCANData != canData) {  // Check that rx buffer changed
      prevCANData = canData;
      rxNodeAddr = (uint8_t) canData & 0xFF;    // Pull out node addr (first byte)
      if (rxNodeAddr == nodeAddress) {    // Command meant for this node
        if (canID == CANID_CAL_CHANGEADDR) {
          uint8_t newAddr = (uint8_t) (canData >> 8);
          ChangeNodeAddress(newAddr);
        }
        else if (canID == CANID_CAL_CHANGEIO) {
          uint8_t ioPort = (uint8_t) (canData >> 8);
          uint8_t ioData = (uint8_t) (canData >> 16);
          ChangeNodeIO(ioPort, ioData);
        }
        else if (canID == CANID_CAL_EXIT) {
          calibrationFlag = 0;
#if EN_DEBUG
          PrintString("<i> Left Calibration Routine");
#endif
        }
      }
    }
  }
  return;
}

void MeasurementRoutine(uint8_t IOData, uint8_t IOPort){
  // If no sensor is connected, do nothing
  if(IOData == DATA_NONE) {
    return;
  }

  uint64_t canData = 0;   // Holds sensor reading
  uint16_t x = 0;
  uint16_t y = 0;
  uint16_t z = 0;    // Vars to hold x,y,z components
  // If there is a sensor then read it and send the value over the bus
  switch(IOData){
    // Analog Engine Head Temperature Sensor
    case DATA_ENGINETEMP:
      canData |= AD8495_GetTemperature(IOPort, 0);    // Get farenheit temp, 16 bit
      MCP2515_SendMsg(canData, CANID_ENGINETEMP, DLC_2, CAN_NO_EXT, CAN_NO_RTR);
      break;

    // Analog Exhaust Gas Temperature Sensor
    case DATA_EXHAUSTTEMP:
      canData |= AD8495_GetTemperature(IOPort, 0);    // Get farineheit temp, 16 bit
      MCP2515_SendMsg(canData, CANID_EXHAUSTTEMP, DLC_2, CAN_NO_EXT, CAN_NO_RTR);
      break;

    // Analog Tachometer (Engine Speed) Sensor
    case DATA_ENGINESPEED:
      canData |= PJK0010_GetEngineSpeed(IOPort);      // Get engine speed, 16 bit
      MCP2515_SendMsg(canData, CANID_ENGINESPEED, DLC_2, CAN_NO_EXT, CAN_NO_RTR);
      break;

    // Analog Tachometer (Axle Speed) Sensor
    case DATA_AXLESPEED:
      canData |= PJK0020_GetKartSpeed(IOPort);        // Get kart speed, 16 bit
      MCP2515_SendMsg(canData, CANID_AXLESPEED, DLC_2, CAN_NO_EXT, CAN_NO_RTR);
      break;

    // Analog Throttle Position Sensor
    case DATA_THROTTLEPOSITION:
      canData |= GetThrottlePosition(IOPort);
      MCP2515_SendMsg(canData, CANID_THROTTLEPOSITION, DLC_2, CAN_NO_EXT, CAN_NO_RTR);
      break;

    // Analog Brake Position Sensor
    case DATA_BRAKEPOSITION:
      canData |= GetBrakePosition(IOPort);
      MCP2515_SendMsg(canData, CANID_BRAKEPOSITION, DLC_2, CAN_NO_EXT, CAN_NO_RTR);
      break;

    // Analog Steering Angle Sensor
    case DATA_STEERINGANGLE:
      canData |= GetSteeringAngle(IOPort);
      MCP2515_SendMsg(canData, CANID_STEERINGANGLE, DLC_2, CAN_NO_EXT, CAN_NO_RTR);
      break;

    // Analog Ambient Temperature Sensor
    case DATA_AMBIENTTEMP:

      break;

    // MPU6050 (I2C)
    case DATA_ACCGYRO:
      // Read accelerometer first
      MPU6050_GetAcceleration(&x, &y, &z);
      canData |= x | ((uint64_t) y << 16) | ((uint64_t) z << 32);
      MCP2515_SendMsg(canData, CANID_ACCELERATION, DLC_6, CAN_NO_EXT, CAN_NO_RTR);
      canData = 0;    // Clear raw data var

      // Read accelerometer first
      MPU6050_GetGyration(&x, &y, &z);
      canData |= x | ((uint64_t) y << 16) | ((uint64_t) z << 32);
      MCP2515_SendMsg(canData, CANID_GYRATION, DLC_6, CAN_NO_EXT, CAN_NO_RTR);
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
      canData |= TestPot_GetValue(IOPort);
      PrintDecimalWord(canData);
      PrintString("\r\n");
      MCP2515_SendMsg(canData, CANID_TESTPOT, DLC_1, CAN_NO_EXT, CAN_NO_RTR);
      break;
  }
}

void ChangeNodeAddress(uint8_t newAddress){
  eeprom_write_byte((uint8_t*) EE_NODEADDR, (uint8_t) newAddress);
}

void ChangeNodeIO(uint8_t ioPort, uint8_t ioData){
  switch(ioPort){
    case IO_A0:
      eeprom_write_byte((uint8_t*) EE_IO_A0, (uint8_t) ioData);
      break;
    case IO_A1:
      eeprom_write_byte((uint8_t*) EE_IO_A1, (uint8_t) ioData);
      break;
    case IO_A2:
      eeprom_write_byte((uint8_t*) EE_IO_A2, (uint8_t) ioData);
      break;
    case IO_A3:
      eeprom_write_byte((uint8_t*) EE_IO_A3, (uint8_t) ioData);
      break;
    case IO_D0:
      eeprom_write_byte((uint8_t*) EE_IO_D0, (uint8_t) ioData);
      break;
    case IO_I2C:
      eeprom_write_byte((uint8_t*) EE_IO_I2C, (uint8_t) ioData);
      break;
    case IO_UART:
      eeprom_write_byte((uint8_t*) EE_IO_UART, (uint8_t) ioData);
      break;
    case IO_SPI:
      eeprom_write_byte((uint8_t*) EE_IO_SPI, (uint8_t) ioData);
      break;
  }








}
// EOF