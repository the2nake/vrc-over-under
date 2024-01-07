#pragma once

#include "cookbook/control/pid.hpp"
#include "cookbook/drive/star_drive.hpp"
#include "cookbook/sensors/odometry.hpp"

class StarDriveController {
public:
  void configure_pidf_x(double kp, double ki, double kd,
                        double feedforward = 0);
  void configure_pidf_y(double kp, double ki, double kd,
                        double feedforward = 0);
  void configure_pidf_r(double kp, double ki, double kd,
                        double feedforward = 0);

  void configure_stop_threshold(double threshold) {
    stop_threshold = std::abs(threshold);
  }

  bool is_motion_complete() { return motion_complete.load(); }

  // TODO: implement timeout

  void move_to_pose_pid_async(Pose goal);

  class StarDriveControllerBuilder {
  public:
    /**
     * @brief adds the chassis to be controlled by the controller
     * @param chassis a pointer to the chassis
     * @return the builder class
     */
    StarDriveControllerBuilder &with_drive(StarDrive *chassis);

    /**
     * @brief adds the odometry setup to be used by the controller
     * @param odom a pointer to the odometry object
     * @return the builder class
     */
    StarDriveControllerBuilder &with_odometry(Odometry *odom);

    /**
     * @brief creates the controller object
     * @return a pointer to the created object
     */
    StarDriveController *build();

  private:
    Odometry *odom = nullptr;
    StarDrive *chassis = nullptr;

    bool failed = false;
  };

  static StarDriveControllerBuilder *builder() {
    return new StarDriveControllerBuilder();
  }

private:
  StarDriveController() {}

  PIDFController *x_pidf = nullptr;
  PIDFController *y_pidf = nullptr;
  PIDFController *r_pidf = nullptr;

  Odometry *odom = nullptr;
  StarDrive *chassis = nullptr;

  std::atomic<bool> motion_complete = true;

  double stop_threshold = 0.01;
};
