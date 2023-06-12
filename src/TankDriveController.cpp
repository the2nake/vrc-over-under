/* home.vn2007@gmail.com - 2023 */

#include "TankDriveController.hpp"

TankDriveController::TankDriveController(TankDrive *drive, Aps *aps)
{
    this->drive = drive;
    this->aps = aps;

    if (drive == nullptr || aps == nullptr) this->disabled = true;
}

TankDriveController::~TankDriveController()
{ // NOTE: will need to destroy created PIDController and RamseteController objects
}