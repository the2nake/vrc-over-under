/* home.vn2007@gmail.com - 2023 */

#pragma once

#include <vector>

#define EIGEN_DONT_VECTORIZE
#include "eigen/Dense"

class Filter
{
public:
    virtual void predict_state(Eigen::VectorXd control_inputs) = 0;
    virtual void correct_prediction(Eigen::VectorXd measurements) = 0;
    virtual Eigen::VectorXd get_state() = 0;
};