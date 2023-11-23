#include "salsa/api.hpp"
#include "api.h"

pros::Imu *default_imu = nullptr;
CustomImu *imu = nullptr;

void initialise_sensors() {
  default_imu = new pros::Imu(PORT_IMU);
  default_imu->reset();
  while (default_imu->is_calibrating()) {
    pros::delay(40);
  }

  imu = new CustomImu(default_imu);
  imu->set_multiplier(0.997128639);
}
