#include "cookbook/drive/tank_drive.hpp"
#include "api.h"
#include "cookbook/util.hpp"

TankDrive::TankDriveBuilder &
TankDrive::TankDriveBuilder::with_motors(std::vector<pros::Motor *> motors) {
  for (auto motor : motors) {
    if (motor == nullptr || errno == ENXIO || errno == ENODEV) {
      failed = true;
      return *this;
    }
  }

  this->motors = motors;

  return *this;
}

TankDrive::TankDriveBuilder &TankDrive::TankDriveBuilder::with_geometry(float track_width) {
  this->track_width = track_width;

  return *this;
}

TankDrive *TankDrive::TankDriveBuilder::build() {
  if (failed) {
    return nullptr;
  }

  TankDrive *drive = new TankDrive();

  drive->track_width = track_width;
  drive->motors = motors;

  return drive;
}

void TankDrive::drive_tank(float l, float r) {
  double vl = l;
  double vr = r;

  limit_magnitude(vl, 1.0);
  limit_magnitude(vr, 1.0);

  auto motors_per_side = std::floor(motors.size() / 2.0);
  for (int i = 0; i < motors_per_side; i++) {
    auto rpm = rpm_from_gearset(motors[i]->get_gearing());
    if (std::abs(vl) > 0.001) {
      motors[i]->move_voltage(12000 * vl);
    } else {
      motors[i]->brake();
    }
  }

  for (int i = motors_per_side; i < motors.size(); i++) {
    auto rpm = rpm_from_gearset(motors[i]->get_gearing());
    if (std::abs(vr) > 0.001) {
      motors[i]->move_voltage(12000 * vr);
    } else {
      motors[i]->brake();
    }
  }
}

void TankDrive::set_brake_mode(pros::motor_brake_mode_e_t mode) {
  for (auto motor : motors) {
    motor->set_brake_mode(mode);
  }
}

void TankDrive::brake() {
  for (auto motor : motors) {
    motor->brake();
  }
}
