#pragma once

#include "pros/motors.hpp"

#include <vector>

class StarDrive {
public:
  class Builder {
  public:
    /**
     * @brief adds preconfigured motors to the drive. motors should be
     * configured such that positive voltage will move the robot forward.
     * @param motors a c-style array of the motors, going from front to back,
     * then left to right
     * @return the builder class
     */
    Builder &with_motors(std::vector<pros::Motor *> motors);
    StarDrive *build();

  private:
    bool failed = false;
  };
  static Builder *builder() { return new Builder(); }

private:
  StarDrive() {}
};