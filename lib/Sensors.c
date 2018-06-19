/*! \file Sensors.c \brief Library for all supported GKDAQ sensors. */
//-----------------------------------------------------------------------------
//  Filename   : Sensors.c
//  Title      : Library for all supported GKDAQ sensors
//  Author     : Patrick Kennedy (PK3)
//  Created    : 02/26/2018
//  Modified   : 02/26/2018
//  Version    : 0.1
//  Description:
//    Library that contains helper functions for all supported sensors in the
//    GKDAQ system
//-----------------------------------------------------------------------------

#include "Sensors.h"
#include "SPI.h"
#include "USART.h"
#include "Analog.h"
#include "I2C.h"
#include "MPU6050.h"

// Test Potentiometer
uint16_t TestPot_GetValue(uint8_t IOPort){
  return GetAnalogInput(IOPort);
}

// AD8495 Thermocouple Amplifier
uint16_t AD8495_GetTemperature(uint8_t IOPort, uint8_t celsius){
  uint16_t analogValue = GetAnalogInput(IOPort);
  float tempC = ((analogValue*5.0/1023.0) - 1.25)/0.005;
  if(celsius != 0){
    return (uint16_t)tempC;
  }
  else{
    return (uint16_t)(tempC*1.8+32);
  }
}

// MAX6675 Thermocouple Amplifier
uint16_t MAX6675_GetTemperature(uint8_t IOPort){
  // TODO MAX6675
  return 0;
}

// PJK0010 Engine Tachometer
uint16_t PJK0010_GetEngineSpeed(uint8_t IOPort){
  uint16_t analogValue = GetAnalogInput(IOPort);
  float rpm = analogValue*PJK0010_MAX_RPM/1023.0;
  return (uint16_t)rpm;
}

// PJK0020 Axle Tachometer
uint16_t PJK0020_GetKartSpeed(uint8_t IOPort){
  uint16_t analogValue = GetAnalogInput(IOPort);
  float rpm = analogValue*PJK0020_MAX_RPM/1023.0;
  float speed = rpm*PJK0020_TIRE_CIRCUM*60;
  return (uint16_t)speed;
}

// Throttle Position Sensor, returns (throttle pct * 10)
uint16_t GetThrottlePosition(uint8_t IOPort){
  uint16_t analogValue = GetAnalogInput(IOPort);
  float pct = 100.0*(analogValue/1023.0)*(360.0/MAX_THROTTLE_ANGLE);

  // Clamp value to 100 pct
  if(pct > 100){
    pct = 100.0;
  }
  return (uint16_t)(pct*10);
}

// Brake Position Sensor, returns (brake pct * 10)
uint16_t GetBrakePosition(uint8_t IOPort){
  uint16_t analogValue = GetAnalogInput(IOPort);
  float pct = 100.0*(analogValue/1023.0)*(360.0/MAX_BRAKE_ANGLE);

  // Clamp value to 100 pct
  if(pct > 100){
    pct = 100.0;
  }
  return (uint16_t)(pct*10);
}

// Steering Angle Sensor
uint16_t GetSteeringAngle(uint8_t IOPort){
  uint16_t analogValue = GetAnalogInput(IOPort);
  float angle = 360.0*((analogValue/1023.0)-0.5);   // Mid point is zero to get pos/neg values

  // Clamp value to +/- max steering angle
  if(angle > MAX_STEERING_ANGLE){
    angle = MAX_STEERING_ANGLE;
  }
  else if(angle < -MAX_STEERING_ANGLE){
    angle = -MAX_STEERING_ANGLE;
  }

  angle *= 10;  // Allow for 0.1 deg precision in uint

  if (angle < 0){
    return ((uint16_t)((-1)*(angle))) | STEERING_ANGLE_NEG_FLAG;   // Set neg flag since uint
  }
  else {
    return (uint16_t) angle;
  }
}

// TMP36 Ambient Temperature Sensor
uint16_t TMP36_GetTemperature(uint8_t IOPort){
  // TODO TMP36 (Low priority)
  return 0;
}



// MPU9250 Accelerometer/Gyroscope/Magnetometer
void MPU9250_Init(uint8_t calibrate){

}

void MPU9250_GetAcceleration(uint8_t IOPort, uint16_t* x, uint16_t* y, uint16_t* z){
  // TODO MPU9250
}

void MPU9250_GetGyration(uint8_t IOPort, uint16_t* x, uint16_t* y, uint16_t* z){

}

void MPU9250_GetHeading(uint8_t IOPort, uint16_t* heading){

}

void MPU9250_Calibrate(uint8_t IOPort){

}

// MTK3339 GPS
uint64_t MTK3339_GetPosition(uint8_t IOPort){
  return 0;
}

void MTK3339_Initialize(uint8_t IOPort){
  return;
}

// Si7021 Temperature/Humidity Sensor
uint16_t SI7021_GetTemperature(uint8_t IOPort){
  // TODO Si7021 (Low priority)
  return 0;
}

uint16_t SI7021_GetHumidity(uint8_t IOPort){
  return 0;
}
