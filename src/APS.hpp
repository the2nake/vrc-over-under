/* home.vn2007@gmail.com - 2023 */

/* NOTE: This is a pure abstract class, the APS.cpp file shouldn't exist */

#pragma once

#include "main.h"

#include <atomic>

#define NO_CHANGE 1000000

struct Pose
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
};

class APS
{
public:
    virtual ~APS();

    virtual void set_pose(double x = NO_CHANGE, double y = NO_CHANGE, double heading = NO_CHANGE);
    virtual void update();
    virtual Pose get_pose();
    
};