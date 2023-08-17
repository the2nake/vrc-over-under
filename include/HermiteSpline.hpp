/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "Spline.hpp"

#define EIGEN_DONT_VECTORIZE
#include "eigen/Dense"

#include <vector>

class HermiteSpline : public Spline
{
public:
    void clear_control_points() { control_points.clear(); }
    void add_control_point(Eigen::Matrix<double, 2, 2> control_point)
    {
        this->control_points.push_back(control_point);
    }

    PathPoint get_point(double u = 0.0) override;

private:
    std::vector<Eigen::Matrix<double, 2, 2>> control_points;
};