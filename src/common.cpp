#include "common.hpp"

double rpm_from_gearset(pros::motor_gearset_e_t gearing)
{
    switch (gearing)
    {
    case MOTOR_GEAR_100:
        return 100.0;
        break;
    case MOTOR_GEAR_200:
        return 200.0;
        break;
    case MOTOR_GEAR_600:
        return 600.0;
        break;
    default:
        return 0.0;
        break;
    }
}
