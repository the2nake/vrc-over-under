#include "salsa/api.hpp"
#include "api.h"

pros::Imu *imu = nullptr;

void initialise_sensors() {
  imu = new pros::Imu(PORT_IMU);
  imu->reset();
  while (imu->is_calibrating()) {
    pros::delay(40);
  }
}