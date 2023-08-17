/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "Filter.hpp"

#define EIGEN_DONT_VECTORIZE
#include "eigen/Dense"

/**
 * State and measurements are (column) vectors
*/

class KalmanFilter : public Filter
{
public:
    KalmanFilter(Eigen::VectorXd initial_state, Eigen::MatrixXd initial_covariance);

    void set_state_transition_matrix(Eigen::MatrixXd matrix);
    void set_control_transition_matrix(Eigen::MatrixXd matrix);
    void set_observation_transition_matrix(Eigen::MatrixXd matrix);

    void set_process_noise_matrix(Eigen::MatrixXd matrix);
    void set_measurement_noise_matrix(Eigen::MatrixXd matrix);

    void predict_state(Eigen::VectorXd control_inputs) override;
    void correct_prediction(Eigen::VectorXd measurements) override;
    Eigen::VectorXd get_state() override { return this->state; }

private:
    int state_vector_size = 0;
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;

    Eigen::VectorXd predicted_state;
    Eigen::MatrixXd predicted_covariance;

    Eigen::MatrixXd f_matrix; // transforms previous state to predicted state
    Eigen::MatrixXd g_matrix; // transforms control space to state space
    Eigen::MatrixXd h_matrix; // transforms state space to measurement space

    Eigen::MatrixXd q_matrix; // process noise
    Eigen::MatrixXd r_matrix; // measurement noise
};
