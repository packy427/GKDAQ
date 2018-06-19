#include "MPU6050.h"
#include "I2C.h"

// MPU6050 Accelerometer/Gyroscope
void MPU6050_Init(){
  uint8_t regValue = 0; // variable to hold register values
  uint8_t gyroScale = MPU6050_SCALE_500DPS;
  uint8_t accelRange = MPU6050_RANGE_4G;
  uint8_t clockSource = MPU6050_CLOCK_INTERNAL_8MHZ;

  // Make sure I2C is enabled
  i2c_init();

  // Set gyro scale
  regValue = MPU6050_Read8(MPU6050_REG_GYRO_CONFIG);
  regValue &= 0b11100111;
  regValue |= (gyroScale << 3);
  MPU6050_Write8(MPU6050_REG_GYRO_CONFIG, regValue);

  // Set accel range
  regValue = MPU6050_Read8(MPU6050_REG_ACCEL_CONFIG);
  regValue &= 0b11100111;
  regValue |= (accelRange << 3);
  MPU6050_Write8(MPU6050_REG_ACCEL_CONFIG, regValue);

  // Set clock source
  regValue = MPU6050_Read8(MPU6050_REG_PWR_MGMT_1);
  regValue &= 0b11111000;
  regValue |= clockSource;
  MPU6050_Write8(MPU6050_REG_PWR_MGMT_1, regValue);
}

void MPU6050_Calibrate(uint8_t accelerometer, uint8_t gyroscope){
  // TODO
  if(accelerometer != 0){

  }
  if(gyroscope != 0){

  }
}

void MPU6050_GetAcceleration(uint16_t* x, uint16_t* y, uint16_t* z){
  i2c_start(MPU6050_ADDRESS+I2C_WRITE);     // set device address and write mode
  i2c_write(MPU6050_REG_ACCEL_XOUT_H);                        // write address = 5
  i2c_start(MPU6050_ADDRESS+I2C_READ);       // set device address and read mode

  uint8_t xh = i2c_read_ack();
  uint8_t xl = i2c_read_ack();
  uint8_t yh = i2c_read_ack();
  uint8_t yl = i2c_read_ack();
  uint8_t zh = i2c_read_ack();
  uint8_t zl = i2c_read_nack();
  i2c_stop();

  *x = (uint16_t) xh << 8 | xl;
  *y = (uint16_t) yh << 8 | yl;
  *z = (uint16_t) zh << 8 | zl;
}

void MPU6050_GetGyration(uint16_t* x, uint16_t* y, uint16_t* z){
  i2c_start(MPU6050_ADDRESS+I2C_WRITE);     // set device address and write mode
  i2c_write(MPU6050_REG_GYRO_XOUT_H);                        // write address = 5
  i2c_start(MPU6050_ADDRESS+I2C_READ);       // set device address and read mode

  uint8_t xh = i2c_read_ack();
  uint8_t xl = i2c_read_ack();
  uint8_t yh = i2c_read_ack();
  uint8_t yl = i2c_read_ack();
  uint8_t zh = i2c_read_ack();
  uint8_t zl = i2c_read_nack();
  i2c_stop();

  *x = (uint16_t) xh << 8 | xl;
  *y = (uint16_t) yh << 8 | yl;
  *z = (uint16_t) zh << 8 | zl;
}

void MPU6050_Write8(uint8_t reg, uint8_t value){
  i2c_start(MPU6050_ADDRESS+I2C_WRITE);     // set device address and write mode
  i2c_write(reg);
  i2c_write(value);                        // write value
  i2c_stop();                             // set stop condition = release bus
}

uint8_t MPU6050_Read8(uint8_t reg){
  uint8_t ret;
  i2c_start(MPU6050_ADDRESS+I2C_WRITE);     // set device address and write mode
  i2c_write(reg);                        // write address = 5
  i2c_start(MPU6050_ADDRESS+I2C_READ);       // set device address and read mode
  ret = i2c_read_nack();                    // read one byte, stop
  i2c_stop();

  return ret;
}