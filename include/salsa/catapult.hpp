#pragma once

#include "pros/rtos.hpp"
#include <atomic>

extern pros::Mutex cata_state_mutex;
extern std::atomic<bool> firing;
extern std::atomic<bool> loading;

void initialise_catapult();

bool catapult_is_loaded();

void load_catapult(void *params = nullptr);

void load_catapult_async();

void fire_catapult(void *params = nullptr);

void fire_catapult_async();
