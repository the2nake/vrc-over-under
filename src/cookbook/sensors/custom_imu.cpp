#include "cookbook/sensors/custom_imu.hpp"
#include "cookbook/util.hpp"

CustomImu::CustomImu(pros::Imu *imu) {
  if (imu != nullptr) {
    base_imu = imu;
  }
}

void CustomImu::update_heading() {
  heading += shorter_turn(heading, base_imu->get_heading(), 360.0) * multiplier;
}
