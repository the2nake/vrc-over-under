#pragma once

#include "cookbook/drive/star_drive.hpp"
#include <atomic>

extern StarDrive *chassis;
extern std::atomic<bool> pto;

void initialise_chassis();
