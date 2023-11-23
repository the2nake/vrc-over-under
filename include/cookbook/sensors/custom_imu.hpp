#pragma once
#include "pros/imu.hpp"

#include <cmath>

class CustomImu {
public:
  CustomImu(pros::Imu *imu);

  void set_heading(double heading = 0.0) {
    this->heading = heading;
    base_imu->set_heading(0);
  }
  void set_multiplier(double multiplier = 1.0) {
    this->multiplier = std::abs(multiplier);
  }

  void update_heading();

  double get_heading() {
    update_heading();
    return heading;
  }

private:
  double multiplier = 1.0;
  double heading = 0.0;

  pros::Imu *base_imu;
};