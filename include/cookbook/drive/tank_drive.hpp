#pragma once

#include "pros/motors.hpp"

#include <vector>

class TankDrive {
public:
  /**
   * @brief moves the drive using tank control, relative to its current
   * orientation
   * @param l the left wheel velocity, from 1 (fwd) to -1 (rev)
   * @param r the right wheel velocity, from 1 (fwd) to -1 (rev)
   */
  void drive_tank(float l, float r);

  void set_brake_mode(pros::motor_brake_mode_e_t mode);
  void brake();

  class TankDriveBuilder {
  public:
    /**
     * @brief adds preconfigured motors to the drive. motors should be
     * configured such that positive voltage will move the robot forward.
     * @param motors a list of the motors, going from front to back,
     * then left to right. lf -> lm -> lb -> rf -> rm -> rb
     * @return the builder class
     */
    TankDriveBuilder &with_motors(std::vector<pros::Motor *> motors);

    /**
     * @brief specify the geometry of the drive. required for imu-less turning
     * @param track_width the distance between the center of the opposite sides'
     * wheels
     * @return the builder class
     */
    TankDriveBuilder &with_geometry(float track_width);

    /**
     * @brief creates the drive object
     * @return a pointer to the created object
     */
    TankDrive *build();

  private:
    bool failed = false;
    float track_width = 1.0;
    std::vector<pros::Motor *> motors = {};
  };
  static TankDriveBuilder *builder() { return new TankDriveBuilder(); }

private:
  TankDrive() {}

  float track_width = 1.0;
  std::vector<pros::Motor *> motors = {}; // front to back, then left to right
};