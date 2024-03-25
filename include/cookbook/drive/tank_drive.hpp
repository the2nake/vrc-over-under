#pragma once

#include "cookbook/control/pid.hpp"
#include "pros/motors.hpp"

#include <vector>

// takes a target velocity proportion and outputs millivolts
typedef int (*vel_ff_model_t)(double);

typedef double (pros::Motor::*motor_func_t)(void) const;

class TankDrive {
public:
  /**
   * @brief moves the drive using tank control, relative to its current
   * orientation
   * @param l the left wheel velocity, from 1 (fwd) to -1 (rev)
   * @param r the right wheel velocity, from 1 (fwd) to -1 (rev)
   */
  void drive_tank_raw(float l, float r);

  /**
   * @brief moves the drive using pid control to regulate wheel velocities. this
   * runs a single iteration of the loop, need to repeat this for proper pid
   * control
   * @param vl the left wheel velocity target (in real units)
   * @param vr the right wheel velocity target (in real units)
   * @returns true if the pid controllers have settled
   */
  bool drive_tank_pid(float vl, float vr);

  // TODO: implement async PID motion
  // NOTE: remember timeout and force stop implementation
  void drive_tank_pid_async(float vl, float vr, int ms_timeout = 0);

  void set_brake_mode(pros::motor_brake_mode_e_t mode);
  void brake();

  double get_left_wheel_lin_vel();
  double get_right_wheel_lin_vel();
  double get_max_wheel_vel() { return this->max_wheel_vel; }

  class TankDriveBuilder {
  public:
    /**
     * @brief adds preconfigured motors to the drive. motors should be
     * configured such that positive voltage will move the robot forward.
     * @param motors a list of the motors, going from front to back,
     * then left to right. lf -> lm -> lb -> rf -> rm -> rb
     * @return the builder object
     */
    TankDriveBuilder &with_motors(std::vector<pros::Motor *> motors);

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
    vel_ff_model_t model = nullptr;

    std::vector<pros::Motor *> motors = {};
  };
  static TankDriveBuilder *builder() { return new TankDriveBuilder(); }

private:
  TankDrive() {}
  double get_avg(motor_func_t func, bool right_side = true);

  float track_width = 1.0;
  float travel = 1.0;
  float max_wheel_vel = 1.0;
  std::vector<pros::Motor *> motors = {}; // front to back, then left to right

  PIDFController *left_wheel_pid = nullptr;
  PIDFController *right_wheel_pid = nullptr;
  float settle_threshold = 120.0;
  vel_ff_model_t model = nullptr;
};