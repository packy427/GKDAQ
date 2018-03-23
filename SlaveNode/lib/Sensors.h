/*! \file Sensors.h \brief Library for all supported GKDAQ sensors. */
//-----------------------------------------------------------------------------
//  Filename   : Sensors.h
//  Title      : Library for all supported GKDAQ sensors
//  Author     : Patrick Kennedy (PK3)
//  Created    : 02/26/2018
//  Modified   : 02/26/2018
//  Version    : 0.1
//  Description:
//    Library that contains helper functions for all supported sensors in the
//    GKDAQ system
//-----------------------------------------------------------------------------

#ifndef GKDAQ_SENSORS_H
#define GKDAQ_SENSORS_H

// Test pot
uint16_t TestPot_GetValue(uint8_t IOPort);

// AD8495 Temperature Sensor
uint16_t AD8495_GetTemperature(uint8_t IOPort);

// MAX6675 Temperature Sensor
uint16_t MAX6675_GetTemperature(uint8_t IOPort);

// PJK0010 Engine Tachometer
uint16_t PJK0010_GetEngineSpeed(uint8_t IOPort);

// PJK0020 Axle Tachometer
uint16_t PJK0020_GetAxleSpeed(uint8_t IOPort);

// PJK0030 Throttle Position Sensor
uint16_t PJK0030_GetThrottlePosition(uint8_t IOPort);

// PJK0040 Brake Position Sensor
uint16_t PJK0040_GetBrakePosition(uint8_t IOPort);

// PJK0050 Steering Angle Sensor
uint16_t PJK0050_GetSteeringAngle(uint8_t IOPort);

// TMP36 Ambient Temperature Sensor
uint16_t TMP36_GetTemperature(uint8_t IOPort);

// MPU6050 Accelerometer/Gyroscope
uint64_t MPU6050_GetAcceleration(uint8_t IOPort);

uint64_t MPU6050_GetGyration(uint8_t IOPort);

void MPU6050_Calibrate(uint8_t IOPort);

// MPU9250 Accelerometer/Gyroscope/Magnetometer
uint64_t MPU9250_GetAcceleration(uint8_t IOPort);

uint64_t MPU9250_GetGyration(uint8_t IOPort);

uint64_t MPU9250_GetHeading(uint8_t IOPort);

void MPU9250_Calibrate(uint8_t IOPort);

// MTK3339 GPS
uint64_t MTK3339_GetPosition(uint8_t IOPort);

void MTK3339_Initialize(uint8_t IOPort);

// Si7021 Temperature/Humidity Sensor
uint16_t TMP36_GetTemperature(uint8_t IOPort);

uint16_t TMP36_GetHumidity(uint8_t IOPort);

#endif  // GKDAQ_SENSORS_H
