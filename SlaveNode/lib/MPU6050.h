//
// Created by PK3 on 4/1/2018.
//

#ifndef GKDAQ_MPU6050_H
#define GKDAQ_MPU6050_H
#define MPU6050_ADDRESS             (0x68) // 0x69 when AD0 pin to Vcc

#define MPU6050_REG_ACCEL_XOFFS_H     (0x06)
#define MPU6050_REG_ACCEL_XOFFS_L     (0x07)
#define MPU6050_REG_ACCEL_YOFFS_H     (0x08)
#define MPU6050_REG_ACCEL_YOFFS_L     (0x09)
#define MPU6050_REG_ACCEL_ZOFFS_H     (0x0A)
#define MPU6050_REG_ACCEL_ZOFFS_L     (0x0B)
#define MPU6050_REG_GYRO_XOFFS_H      (0x13)
#define MPU6050_REG_GYRO_XOFFS_L      (0x14)
#define MPU6050_REG_GYRO_YOFFS_H      (0x15)
#define MPU6050_REG_GYRO_YOFFS_L      (0x16)
#define MPU6050_REG_GYRO_ZOFFS_H      (0x17)
#define MPU6050_REG_GYRO_ZOFFS_L      (0x18)
#define MPU6050_REG_CONFIG            (0x1A)
#define MPU6050_REG_GYRO_CONFIG       (0x1B) // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG      (0x1C) // Accelerometer Configuration
#define MPU6050_REG_FF_THRESHOLD      (0x1D)
#define MPU6050_REG_FF_DURATION       (0x1E)
#define MPU6050_REG_MOT_THRESHOLD     (0x1F)
#define MPU6050_REG_MOT_DURATION      (0x20)
#define MPU6050_REG_ZMOT_THRESHOLD    (0x21)
#define MPU6050_REG_ZMOT_DURATION     (0x22)
#define MPU6050_REG_INT_PIN_CFG       (0x37) // INT Pin. Bypass Enable Configuration
#define MPU6050_REG_INT_ENABLE        (0x38) // INT Enable
#define MPU6050_REG_INT_STATUS        (0x3A)
#define MPU6050_REG_ACCEL_XOUT_H      (0x3B)
#define MPU6050_REG_ACCEL_XOUT_L      (0x3C)
#define MPU6050_REG_ACCEL_YOUT_H      (0x3D)
#define MPU6050_REG_ACCEL_YOUT_L      (0x3E)
#define MPU6050_REG_ACCEL_ZOUT_H      (0x3F)
#define MPU6050_REG_ACCEL_ZOUT_L      (0x40)
#define MPU6050_REG_TEMP_OUT_H        (0x41)
#define MPU6050_REG_TEMP_OUT_L        (0x42)
#define MPU6050_REG_GYRO_XOUT_H       (0x43)
#define MPU6050_REG_GYRO_XOUT_L       (0x44)
#define MPU6050_REG_GYRO_YOUT_H       (0x45)
#define MPU6050_REG_GYRO_YOUT_L       (0x46)
#define MPU6050_REG_GYRO_ZOUT_H       (0x47)
#define MPU6050_REG_GYRO_ZOUT_L       (0x48)
#define MPU6050_REG_MOT_DETECT_STATUS (0x61)
#define MPU6050_REG_MOT_DETECT_CTRL   (0x69)
#define MPU6050_REG_USER_CTRL         (0x6A) // User Control
#define MPU6050_REG_PWR_MGMT_1        (0x6B) // Power Management 1
#define MPU6050_REG_WHO_AM_I          (0x75) // Who Am I


#define MPU6050_CLOCK_KEEP_RESET       0b111
#define MPU6050_CLOCK_EXTERNAL_19MHZ   0b101
#define MPU6050_CLOCK_EXTERNAL_32KHZ   0b100
#define MPU6050_CLOCK_PLL_ZGYRO        0b011
#define MPU6050_CLOCK_PLL_YGYRO        0b010
#define MPU6050_CLOCK_PLL_XGYRO        0b001
#define MPU6050_CLOCK_INTERNAL_8MHZ    0b000
#define MPU6050_SCALE_2000DPS          0b11
#define MPU6050_SCALE_1000DPS          0b10
#define MPU6050_SCALE_500DPS           0b01
#define MPU6050_SCALE_250DPS           0b00
#define MPU6050_RANGE_16G              0b11
#define MPU6050_RANGE_8G               0b10
#define MPU6050_RANGE_4G               0b01
#define MPU6050_RANGE_2G               0b00

void MPU6050_Init();
void MPU6050_Calibrate(uint8_t,uint8_t);
void MPU6050_GetAcceleration(uint16_t*,uint16_t*,uint16_t*);
void MPU6050_GetGyration(uint16_t*,uint16_t*,uint16_t*);
uint8_t MPU6050_Read8(uint8_t);
void MPU6050_Write8(uint8_t, uint8_t);
#endif //GKDAQ_MPU6050_H
