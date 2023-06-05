#pragma once

#include "main.h"

double rpm_from_gearset(pros::motor_gearset_e_t gearing);

void scale_down_magnitude(double &a, double &b, double max);
void limit_magnitude(double &a, double max);
