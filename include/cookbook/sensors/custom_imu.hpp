#pragma once
#include "pros/imu.hpp"

#include <cmath>

#include "cookbook/util.hpp"

class CustomImu {
public:
  CustomImu(pros::Imu *imu);

  void set_heading(double theta = 0.0) {
    theta = mod(theta, 360.0);
    this->heading = theta;
    this->prev_base_heading = theta;
    base_imu->set_heading((int)(std::floor(theta)));
    update_heading();
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
  double prev_base_heading = 0.0;

  pros::Imu *base_imu;
};