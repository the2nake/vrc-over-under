#pragma once

#include "pros/motors.hpp"

#include <vector>

struct StarDriveVelocities {
  float v_lf = 0.0;
  float v_lm = 0.0;
  float v_lb = 0.0;

  float v_rf = 0.0;
  float v_rm = 0.0;
  float v_rb = 0.0;
};

class StarDrive {
public:
  /**
   * @brief moves the drive, relative to its current orientation
   * @param x the x velocity, from 1 (right) to -1 (left)
   * @param y the y velocity, from 1 (forward) to -1 (back)
   * @param r the rotation velocity, from 1 (clockwise) to -1 (anticlockwise)
   * @param boost whether to use the boost motors
   * @return the velocities given to the motors. useful for ptos
   */
  StarDriveVelocities drive_relative(float x, float y, float r,
                                     bool boost = true);

  /**
   * @brief moves the drive, with field oriented control
   * @param x the x velocity, from 1 (right) to -1 (left)
   * @param y the y velocity, from 1 (up) to -1 (down)
   * @param r the rotation velocity, from 1 (clockwise) to -1 (anticlockwise)
   * @param heading the current heading of the robot in degrees
   * @param boost whether to use the boost motors
   * @return the velocities given to the motors. useful for ptos
   */
  StarDriveVelocities drive_field_based(float x, float y, float r,
                                        float heading, bool boost = true);

  void set_brake_mode(pros::motor_brake_mode_e_t mode);
  void brake();

  class StarDriveBuilder {
  public:
    /**
     * @brief adds preconfigured motors to the drive. motors should be
     * configured such that positive voltage will move the robot forward.
     * @param motors a c-style array of the motors, going from front to back,
     * then left to right. lf -> lm -> lb -> rf -> rm -> rb
     * @return the builder class
     */
    StarDriveBuilder &with_motors(std::vector<pros::Motor *> motors);

    /**
     * @brief specify the geometry of the drive. required for smooth rotation
     * @param boost_width the distance between the center of the two boost
     * wheels
     * @param diagonal the distance between the centers of opposing corner
     * wheels
     * @return the builder class
     */
    StarDriveBuilder &with_geometry(float boost_width, float diagonal);

    /**
     * @brief creates the drive object
     * @return a pointer to the created object
     */
    StarDrive *build();

  private:
    bool failed = false;
    float boost_width = 0.0;
    float diagonal = 0.0;
    std::vector<pros::Motor *> motors = {};
  };
  static StarDriveBuilder *builder() { return new StarDriveBuilder(); }

private:
  StarDrive() {}
  float boost_width = 0.0;
  float diagonal = 0.0;
  // lf -> lm -> lb -> rf -> rm -> rb
  std::vector<pros::Motor *> motors = {};
};