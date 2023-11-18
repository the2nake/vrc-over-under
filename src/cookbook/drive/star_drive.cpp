#include "cookbook/drive/star_drive.hpp"

StarDrive::Builder &
StarDrive::Builder::with_motors(std::vector<pros::Motor *> motors) {
  return *this;
}

StarDrive *StarDrive::Builder::build() {
  StarDrive *drive = new StarDrive();
  return drive;
}