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

// AD8495 Temperature Sensor
uint16_t AD8495_GetTemperature(uint8_t IOPort){
  return 0;
}

// PK0010 Engine Tachometer
uint16_t PK0010_GetEngineSpeed(uint8_t IOPort){
  return 0;
}

// PK0020 Axle Tachometer
uint16_t PK0020_GetAxleSpeed(uint8_t IOPort){
  return 0;
}

// PK0030 Throttle Position Sensor
uint16_t PK0030_GetThrottlePosition(uint_t IOPort){
  return 0;
}

// PK0040 Brake Position Sensor
uint16_t PK0040_GetBrakePosition(uint8_t IOPort){
  return 0;
}

// PK0050 Steering Angle Sensor
uint16_t PK0050_GetSteeringAngle(uint8_t IOPort){
  return 0;
}

// TMP36 Ambient Temperature Sensor
uint16_t TMP36_GetTemperature(uint8_t IOPort){
  return 0;
}

// MPU6050 Accelerometer/Gyroscope
uint64_t MPU6050_GetAcceleration(uint8_t IOPort){
  return 0;
}

uint64_t MPU6050_GetGyration(uint8_t IOPort){
  return 0;
}

// MPU9250 Accelerometer/Gyroscope/Magnetometer
uint64_t MPU9250_GetAcceleration(uint8_t IOPort){
  return 0;
}

uint64_t MPU9250_GetGyration(uint8_t IOPort){
  return 0;
}

uint64_t MPU9250_GetHeading(uint8_t IOPort){
  return 0;
}

// MTK3339 GPS
uint64_t MTK3339_GetPosition(uint8_t IOPort){
  return 0;
}

// Si7021 Temperature/Humidity Sensor
uint16_t TMP36_GetTemperature(uint8_t IOPort){
  return 0;
}

uint16_t TMP36_GetHumidity(uint8_t IOPort){
  return 0;
}
