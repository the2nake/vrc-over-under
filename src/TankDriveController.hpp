/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"
#include "TankDrive.hpp"
#include "APS.hpp"

class TankDriveController
{
public:
    TankDriveController(TankDrive *drive, APS *aps);
    ~TankDriveController();

    bool is_disabled() { return this->disabled; }

private:
    TankDrive *drive = nullptr;
    APS *aps = nullptr;

    bool disabled = false;
};
