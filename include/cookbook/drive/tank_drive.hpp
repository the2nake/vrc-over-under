#pragma once

#include "cookbook/control/pid.hpp"

#include "cookbook/control/models.hpp"
#include "pros/motors.hpp"

#include <vector>

// takes a target velocity proportion and outputs millivolts
typedef int (*vel_ff_model_t)(double);

typedef std::vector<double> (pros::MotorGroup::*mg_func_t)(void);

class TankDrive {
public:
  /**
   * @brief moves the drive using tank control
   * @param volt_l the left wheel voltage percentage, from 1 (fwd) to -1 (rev)
   * @param volt_r the right wheel voltage percentage, from 1 (fwd) to -1 (rev)
   */
  void drive_tank_raw(float volt_l, float volt_r);

  /**
   * @brief moves the drive using pid control to regulate wheel velocities. this
   * runs a single iteration of the loop, need to repeat this for proper pid
   * control
   * @param vl the left wheel velocity target (in real units)
   * @param vr the right wheel velocity target (in real units)
   * @returns true if the pid controllers have settled
   */
  bool drive_tank_vel(float vl, float vr);

  void set_brake_mode(pros::motor_brake_mode_e_t mode);
  void brake();

  double get_left_wheel_lin_vel();
  double get_right_wheel_lin_vel();
  double get_max_wheel_vel() { return this->max_wheel_vel; }
  double get_avg(mg_func_t func, bool right_side = true);

  class TankDriveBuilder {
  public:
    /**
     * @brief adds preconfigured motors to the left side. motors should be
     * configured such that positive voltage will move the robot forward.
     * @param motors a list of the motors, going from front to back
     * @return the builder object
     */
    TankDriveBuilder &with_left_motors(std::vector<pros::Motor> &motors);

    /**
     * @brief adds preconfigured motors to the right side. motors should be
     * configured such that positive voltage will move the robot forward.
     * @param motors a list of the motors, going from front to back
     * @return the builder object
     */
    TankDriveBuilder &with_right_motors(std::vector<pros::Motor> &motors);

    /**
     * @brief specify the geometry of the drive. required for imu-less turning
     * @param track_width the distance between the center of the opposite sides'
     * wheels
     * @param travel the distance travelled after spinning the motor 1 full
     * revolution
     * @return the builder object
     */
    TankDriveBuilder &with_geometry(float track_width, float travel);

    /**
     * @brief specify the kinematics of the drive. required for motion
     * algorithms.
     * @param max_wheel_vel the maximum linear velocity of the wheels
     * @return the builder object
     */
    TankDriveBuilder &with_kinematics(float max_wheel_vel);

    /**
     * @brief specify the PID constants for the drive. tune them so that the
     * output can reach 12000 millivolts magnitude
     * @param kp the proportional gain
     * @param ki the integral gain
     * @param kd the derivative gain
     * @param settle_threshold at what controller output to settle at
     * @return the builder object
     */
    TankDriveBuilder &with_pid_constants(float kp, float ki, float kd,
                                         float settle_threshold = 120.0);

    /**
     * @brief specifies a feedforward model for wheel velocities
     * @param model the feedforward model
     * @return the builder object
     */
    TankDriveBuilder &with_vel_feedfoward_model(vel_ff_model_t model);

    /**
     * @brief creates the drive object
     * @return a pointer to the created object
     */
    TankDrive *build();

  private:
    bool failed = false;
    float track_width = 1.0;
    float travel = 1.0;
    float max_wheel_vel = 1.0;

    float kp = 1.0, ki = 0.0, kd = 0.0;
    float settle_threshold = 120.0;
    vel_ff_model_t ff_model = models::default_motor_ff_model;

    pros::Motor_Group *left_motors = nullptr;
    pros::Motor_Group *right_motors = nullptr;
  };
  static TankDriveBuilder *builder() { return new TankDriveBuilder(); }

private:
  TankDrive() {}

  float track_width = 1.0;
  float travel = 1.0;
  float max_wheel_vel = 1.0;

  // front to back
  pros::Motor_Group *left_motors = nullptr;
  pros::Motor_Group *right_motors = nullptr;

  PIDFController *left_wheel_pid = nullptr;
  PIDFController *right_wheel_pid = nullptr;
  float settle_threshold = 120.0;
  vel_ff_model_t ff_model = models::default_motor_ff_model;
};