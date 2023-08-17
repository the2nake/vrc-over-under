/* home.vn2007@gmail.com - 2023 */

#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(Eigen::VectorXd state, Eigen::MatrixXd covariance)
{
    this->state = state;
    this->state_vector_size = state.innerSize();
    if (covariance.rows() == covariance.cols() && covariance.cols() == this->state_vector_size)
    {
        this->covariance = covariance;
    }
    else
    {
        this->covariance = Eigen::MatrixXd::Zero(this->state_vector_size, this->state_vector_size);
    }
}

void KalmanFilter::set_state_transition_matrix(Eigen::MatrixXd matrix)
{
    if (matrix.rows() == matrix.cols() && matrix.rows() == this->state_vector_size)
    {
        this->f_matrix = matrix;
    }
}

void KalmanFilter::set_control_transition_matrix(Eigen::MatrixXd matrix)
{
    if (matrix.rows() == this->state_vector_size)
    {
        this->g_matrix = matrix;
    }
}

void KalmanFilter::set_observation_transition_matrix(Eigen::MatrixXd matrix)
{
    if (matrix.cols() == this->state_vector_size)
    {
        this->h_matrix = matrix;
    }
}

void KalmanFilter::set_process_noise_matrix(Eigen::MatrixXd matrix)
{
    if (matrix.rows() == matrix.cols() && matrix.rows() == this->state_vector_size)
    {
        this->q_matrix = matrix;
    }
}

void KalmanFilter::set_measurement_noise_matrix(Eigen::MatrixXd matrix)
{
    // commmented out section because it would require setting h_matrix first. may add again
    if (matrix.rows() == matrix.cols() /* && h_matrix.rows() == matrix.rows() */)
    {
        this->r_matrix = matrix;
    }
}

void KalmanFilter::predict_state(Eigen::VectorXd control_inputs)
{
    if (this->g_matrix.cols() != control_inputs.rows())
    {
        // NOTE: this is an error, attempts to correct by removing control inputs
        control_inputs = Eigen::MatrixXd::Zero(g_matrix.cols(), 1);
    }

    this->predicted_state = this->f_matrix * this->state + this->g_matrix * control_inputs;
    this->predicted_covariance = this->f_matrix * this->covariance * this->f_matrix.transpose() + this->q_matrix;
}

void KalmanFilter::correct_prediction(Eigen::VectorXd measurements)
{
    if (this->h_matrix.rows() != measurements.rows())
    {
        // NOTE: this is an error, skip the measurements
        return;
    }
    
    // h_matrix_backup = h_matrix;
    // h_matrix = state_dependent_modifier(state) * h_matrix

    Eigen::MatrixXd innovation = measurements - h_matrix * this->predicted_state;
    Eigen::MatrixXd innovation_covariance = h_matrix * this->predicted_covariance * h_matrix.transpose() + r_matrix;

    // gain has the dimensions of state.rows() by measurements.cols()
    // essentially it tells you how measurement differences should affect the state
    Eigen::MatrixXd gain = this->predicted_covariance * h_matrix.transpose() * innovation_covariance.inverse();
    this->state = this->predicted_state + gain * innovation;

    Eigen::MatrixXd i_kh = Eigen::MatrixXd::Identity(this->state_vector_size, this->state_vector_size) - gain * h_matrix;
    this->covariance = i_kh * this->predicted_covariance * i_kh.transpose() + gain * r_matrix * gain.transpose();

    // h_matrix = h_matrix_backup
}