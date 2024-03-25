#include "cookbook/sensors/custom_imu.hpp"
#include "cookbook/util.hpp"

CustomImu::CustomImu(pros::Imu *imu) {
  if (imu != nullptr) {
    base_imu = imu;
  }
}

void CustomImu::update_heading() {
  auto curr_base_heading = base_imu->get_heading();
  heading = mod(heading + shorter_turn(prev_base_heading, curr_base_heading, 360.0) * multiplier, 360.0);
  prev_base_heading = curr_base_heading;
}
