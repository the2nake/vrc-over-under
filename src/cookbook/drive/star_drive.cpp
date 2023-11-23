#include "cookbook/drive/star_drive.hpp"
#include "api.h"
#include "cookbook/util.hpp"

StarDrive::Builder &
StarDrive::Builder::with_motors(std::vector<pros::Motor *> motors) {
  for (auto motor : motors) {
    if (motor == nullptr || errno == ENXIO || errno == ENODEV) {
      failed = true;
      return *this;
    }
  }

  this->motors = motors;

  return *this;
}

StarDrive::Builder &StarDrive::Builder::with_geometry(float boost_width,
                                                      float diagonal) {
  if (boost_width > diagonal) {
    // only slowing down boost will be implemented
    failed = true;
  }

  this->boost_width = boost_width;
  this->diagonal = diagonal;

  return *this;
}

StarDrive *StarDrive::Builder::build() {
  if (failed) {
    return nullptr;
  }

  StarDrive *drive = new StarDrive();

  drive->boost_width = this->boost_width;
  drive->diagonal = this->diagonal;
  drive->motors = this->motors;

  return drive;
}

StarDriveVelocities StarDrive::drive_relative(float x, float y, float r,
                                              bool boost) {
  // scale down x and y to have a combined max magnitude of one
  float v = polar_radius<float>(x, y);
  if (v > 1.0) {
    x /= v;
    y /= v;
  }

  // calculate wheel velocities for translation
  float v_lf = x + y;
  float v_lb = y - x;

  float v_rf = y - x;
  float v_rb = x + y;

  // 100% power is equivalent translation-wise between corner and boost motors
  float v_lm = y;
  float v_rm = y;

  // modify wheel velocities to account for rotational component
  v_lf += r;
  v_lb += r;

  v_rf -= r;
  v_rb -= r;

  // scale down boost by sqrt 2 times to account for faster middle wheel
  // then scale down again to account for smaller turn radius
  auto boost_scale = 0.707106781 * boost_width / diagonal;
  v_lm += boost_scale * r;
  v_rm -= boost_scale * r;

  // create output, skipping over boost motors if boost = false
  std::vector<float> velocities = {v_lf, 0, v_lb, v_rf, 0, v_rb};
  if (boost) {
    velocities = {v_lf, v_lm, v_lb, v_rf, v_rm, v_rb};
  }

  // limit everything down to 1 if above 1
  float fastest = std::abs(velocities[0]);
  for (int i = 1; i < velocities.size(); i++) {
    if (std::abs(velocities[i]) > fastest) {
      fastest = std::abs(velocities[i]);
    }
  }

  if (std::abs(fastest) > 1.0) {
    for (int i = 0; i < velocities.size(); i++) {
      velocities[i] /= std::abs(fastest);
    }
  }

  // move the motors
  for (int i = 0; i < velocities.size(); i++) {
    auto rpm = rpm_from_gearset(motors[i]->get_gearing());
    if (std::abs(velocities[i]) > 0.001) {
      motors[i]->move_voltage(12000 * velocities[i]);
    } else {
      motors[i]->brake();
    }
  }

  // return the velocities
  StarDriveVelocities output = {
      velocities[0], // lf
      velocities[1], // lm
      velocities[2], // lb
      velocities[3], // rf
      velocities[4], // rm
      velocities[5]  // rb
  };

  return output;
}

StarDriveVelocities StarDrive::drive_field_based(float x, float y, float r,
                                                 float heading, bool boost) {
  heading = mod(heading, 360.0);
  // anticlockwise rotation matrix
  auto x_r = x * cos_deg(heading) - y * sin_deg(heading);
  auto y_r = x * sin_deg(heading) + y * cos_deg(heading);
  return drive_relative(x_r, y_r, r, boost);
}

void StarDrive::set_brake_mode(pros::motor_brake_mode_e_t mode) {
  for (auto motor : motors) {
    motor->set_brake_mode(mode);
  }
}

void StarDrive::brake() {
  for (auto motor : motors) {
    motor->brake();
  }
}
