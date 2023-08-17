/* home.vn2007@gmail.com - 2023 */

#pragma once

struct PathPoint {
    double x;
    double y;
    double dx;
    double dy;

    // ignore acceleration for now until motion profiles are complete
    double d2x;
    double d2y;
};

class Spline {
public:
    virtual PathPoint get_point(double u = 0.0) = 0;
};