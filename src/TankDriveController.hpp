/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"
#include "TankDrive.hpp"
#include "Aps.hpp"

class TankDriveController
{
public:
    TankDriveController(TankDrive *drive, Aps *aps);
    ~TankDriveController();

    bool is_disabled() { return this->disabled; }

private:
    TankDrive *drive = nullptr;
    Aps *aps = nullptr;

    bool disabled = false;
};
