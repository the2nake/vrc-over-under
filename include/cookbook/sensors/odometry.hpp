#pragma once
#include "cookbook/sensors/custom_imu.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include <atomic>

struct Pose {
  double x = 0;
  double y = 0;
  double heading = 0;
};

class Odometry {
public:
  struct TrackerConfig {
    double travel_per_unit;
    double tracker_coord;
  };

  class OdometryBuilder {
  public:
    /**
     * @brief adds an x-axis tracking wheel using an IME (internal motor
     * encoder)
     * @param motor a pointer to the motor
     * @param travel_per_encoder_unit wheel travel per unit of the motor
     * encoder. use any units perferred for length, but make sure all inputs to
     * the odometry class are in that same length unit
     * @param tracker_y_coord the y coordinate of the tracking wheel. positive
     * is up, negative is down. use any units perferred for length, but make
     * sure all inputs to the odometry class are in that same length unit
     * @returns the builder object
     */
    OdometryBuilder &with_x_tracker(pros::Motor *motor,
                                    double travel_per_encoder_unit,
                                    double tracker_y_coord);

    /**
     * @brief adds an y-axis tracking wheel using an IME (internal motor
     * encoder)
     * @param motor a pointer to the motor
     * @param travel_per_encoder_unit wheel travel per unit of the motor
     * encoder. use any units perferred for length, but make sure all inputs to
     * the odometry class are in that same length unit
     * @param tracker_x_coord the x coordinate of the tracking wheel. positive
     * is right, negative is left. use any units perferred for length, but make
     * sure all inputs to the odometry class are in that same length unit
     * @returns the builder object
     */
    OdometryBuilder &with_y_tracker(pros::Motor *motor,
                                    double travel_per_encoder_unit,
                                    double tracker_x_coord);

    OdometryBuilder &with_heading_imu(pros::Imu *imu);
    OdometryBuilder &with_heading_imu(CustomImu *imu);
    OdometryBuilder &with_heading_ime(pros::Motor *motor_left,
                                      double travel_per_encoder_unit_left,
                                      pros::Motor *motor_right,
                                      double travel_per_encoder_unit_right,
                                      double track_width);
    /**
     * @brief adds a rotation to the trackers, specifying how much they are
     * rotated clockwise
     * @param deg the rotation in degrees
     * @returns the builder object
     */
    OdometryBuilder &with_tracker_rotation(double deg = 0.0);

    /**
     * @brief creates the pid controller
     * @return a pointer to the created object
     */
    Odometry *build();

  private:
    bool failed = false;

    bool heading_uses_imu = false;
    bool heading_uses_motors = false;

    std::vector<std::pair<pros::Motor *, TrackerConfig>> motor_x_trackers = {};
    std::vector<std::pair<pros::Motor *, TrackerConfig>> motor_y_trackers = {};

    double tracker_rotation = 0.0;

    CustomImu *imu = nullptr;
  };
  static OdometryBuilder *builder() { return new OdometryBuilder(); }

  Pose get_pose() { return {x.load(), y.load(), heading.load()}; }
  void update();
  void auto_update(double ms_interval); // TODO: implementation

private:
  Odometry() {}
  bool heading_uses_imu = false;
  bool heading_uses_motors = false;

  std::vector<std::pair<pros::Motor *, TrackerConfig>> motor_x_trackers = {};
  std::vector<std::pair<pros::Motor *, TrackerConfig>> motor_y_trackers = {};

  std::vector<double> prev_motor_x_enc_vals = {};
  std::vector<double> prev_motor_y_enc_vals = {};
  double prev_heading = 0.0;

  // to implement this just do odometry as usual and then rotate the local
  // vector by an additional [tracker_rotation] degrees before adding to the
  // global position
  double tracker_rotation = 0.0;

  CustomImu *imu = nullptr;

  std::atomic<double> x = 0;
  std::atomic<double> y = 0;
  std::atomic<double> heading = 0;

  pros::Mutex mutex;
  pros::Task *auto_update_task;
};

// TODO: implement odometry functions for tracking wheels as well as heading
// based on imes
