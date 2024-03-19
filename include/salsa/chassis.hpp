#pragma once

#include "cookbook/drive/tank_drive.hpp"

#include <atomic>

extern TankDrive *chassis;
extern std::atomic<bool> pto;

void initialise_chassis();
