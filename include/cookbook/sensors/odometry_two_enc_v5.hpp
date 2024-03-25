#pragma once

#include "odometry.hpp"

#include "cookbook/sensors/custom_imu.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <atomic>

class TwoEncoderV5Odometry : public Odometry {
public:
  void set_heading(double heading) override;
  void set_position(double x, double y) override;

  double get_heading() override;
  odom_pose_t get_pose() override;

  void update() override;

  class TwoEncoderV5OdometryBuilder {
  public:
    TwoEncoderV5OdometryBuilder &with_imu(pros::Imu *imu);
    TwoEncoderV5OdometryBuilder &with_imu(CustomImu *imu);

    TwoEncoderV5OdometryBuilder &with_x_encoder(pros::Rotation *rot,
                                                odom_enc_config_t config);
    TwoEncoderV5OdometryBuilder &with_y_encoder(pros::Rotation *rot,
                                                odom_enc_config_t config);

    /**
     * @brief creates the odometry object
     * @returns the odometry object
     */
    TwoEncoderV5Odometry *build();

  private:
    bool failed = false;

    CustomImu *imu = nullptr;
    pros::Rotation *x_enc;
    odom_enc_config_t x_conf;
    pros::Rotation *y_enc;
    odom_enc_config_t y_conf;
  };

  static TwoEncoderV5OdometryBuilder *builder() {
    return new TwoEncoderV5OdometryBuilder();
  }

private:
  TwoEncoderV5Odometry() {}
  void lock() {
    while (!this->state_mutex.take(5)) {
      pros::delay(1);
    }
  }

  CustomImu *imu = nullptr;
  pros::Rotation *x_enc = nullptr;
  odom_enc_config_t x_conf;
  pros::Rotation *y_enc = nullptr;
  odom_enc_config_t y_conf;

  uint32_t prev_update = 0; // in milliseconds
  double prev_heading = 0.0;
  double prev_x_enc_val = 0.0;
  double prev_y_enc_val = 0.0;

  odom_pose_t pose;

  pros::Mutex state_mutex;
};