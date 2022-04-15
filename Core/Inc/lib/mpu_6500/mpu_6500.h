#ifndef __MPU_6500
#define __MPU_6500

#include <stdint.h>

#include "i2c_interface.h"
#include "mpu_6500_enum.h"

typedef struct MPU_6500_Sensor_Data {
  union {
    uint8_t raw[6];
    struct {
      uint16_t x;
      uint16_t y;
      uint16_t z;
    };
  };
} MPU_6500_Sensor_Data;

typedef struct MPU_6500_Measurement {
  union {
    uint8_t data_arr[12];
    MPU_6500_Sensor_Data acc_data;
    MPU_6500_Sensor_Data gyro_data;
  };

  uint16_t temperature;
} MPU_6500_Measurement;

template <MPU_6500_ADDR ADDR, class I2C>
class MPU_6500 {
  MPU_6500_Measurement last_measurement;

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

    for (int8_t i = 5; i >= 0; --i) {
      I2C::write(ADDR, &data_write_acc[i], 1);
      I2C::read(ADDR, &last_measurement.acc_data.raw[i], 1, true);

      // I2C::write(ADDR, &data_write_gyro[i], 1, true);
      // I2C::read(ADDR, &last_measurement.gyro_data.raw[i], 1, true);
    }
  }

  MPU_6500_Measurement& getMeasurement() {
    return this->last_measurement;
  }
};

#endif /* __MPU_6500 */
