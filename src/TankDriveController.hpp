/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"
#include "TankDrive.hpp"
#include "APS.hpp"

class TankDriveController {
public:
    TankDriveController(TankDrive *drive, APS *aps);
    ~TankDriveController();

private:
    TankDrive *drive = nullptr;
    APS *aps = nullptr;
};
