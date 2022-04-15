#ifndef __MPU_6050
#define __MPU_6050

#include <stdint.h>

#include "i2c_interface.h"
#include "mpu_6050_enum.h"

static constexpr uint16_t ADC_RES = 1 << 14;

typedef struct __attribute__ ((packed)) MPU_6050_Sensor_Data {
  union {
    uint8_t all[6];
    struct {
      int16_t x;
      int16_t y;
      int16_t z;
    };
  } raw;
  union {
    uint8_t all[12];
    struct {
      float x;
      float y;
      float z;
    };
  } calculated;
} MPU_6050_Sensor_Data;

typedef struct MPU_6050_Measurement {
  MPU_6050_Sensor_Data acc_data;
  MPU_6050_Sensor_Data gyro_data;

  uint16_t temperature;
} MPU_6050_Measurement;

template <MPU_6050_ADDR ADDR, class I2C>
class MPU_6050 {
  MPU_6050_Measurement last_measurement;

 public:
  void init() {
    uint8_t data_write_pwr[] = {PWR_MGMT_1, 0x00};
    I2C::write(ADDR, &data_write_pwr[0], 2);
  }

  void measure() {
    uint8_t data_write_acc[] = {
        ACCEL_XOUT_L, ACCEL_XOUT_H, 
        ACCEL_YOUT_L, ACCEL_YOUT_H, 
        ACCEL_ZOUT_L, ACCEL_ZOUT_H,
    };

    uint8_t data_write_gyro[] = {
        GYRO_XOUT_L, GYRO_XOUT_H, 
        GYRO_YOUT_L, GYRO_YOUT_H, 
        GYRO_ZOUT_L, GYRO_ZOUT_H,
    };

    volatile uint8_t size_bla = sizeof(MPU_6050_Sensor_Data);

    for (int8_t i = 5; i >= 0; --i) {
      I2C::write(ADDR, &data_write_acc[i], 1);
      I2C::read(ADDR, &(this->last_measurement.acc_data.raw.all[i]), 1, true);

      // I2C::write(ADDR, &data_write_gyro[i], 1, true);
      // I2C::read(ADDR, &last_measurement.gyro_data.raw[i], 1, true);
    }

    this->last_measurement.acc_data.calculated.x = static_cast<float>(this->last_measurement.acc_data.raw.x) / ADC_RES;
    this->last_measurement.acc_data.calculated.y = static_cast<float>(this->last_measurement.acc_data.raw.y) / ADC_RES;
    this->last_measurement.acc_data.calculated.z = static_cast<float>(this->last_measurement.acc_data.raw.z) / ADC_RES;
  }

  MPU_6050_Measurement& getMeasurement() {
    return this->last_measurement;
  }
};

#endif /* __MPU_6050 */
