/* home.vn2007@gmail.com - 2023 */

#include "TwoWheelAps.hpp"

#include "common.hpp"

TwoWheelAps::~TwoWheelAps()
{
    delete this->x_encoder;
    delete this->y_encoder;
}

void TwoWheelAps::set_pose(Pose pose)
{
    while (!this->pose_data_mutex.take(5))
        ;

    this->x_encoder->reset();
    this->y_encoder->reset();
    if (pose.x != NO_CHANGE)
        this->x = pose.x;
    if (pose.y != NO_CHANGE)
        this->y = pose.y;

    if (pose.heading != NO_CHANGE)
    {
        this->heading = pose.heading;
        this->imu->set_heading(this->heading);
    }

    pose_data_mutex.give();
}

void TwoWheelAps::update()
{
    double prev_x_enc_val = this->x_enc_val;
    double prev_y_enc_val = this->y_enc_val;
    double prev_imu = this->imu_heading;

    this->x_enc_val = this->x_encoder->get_value();
    this->y_enc_val = this->y_encoder->get_value();
    this->imu_heading = this->imu->get_heading();

    double d_x_enc = (this->x_enc_val - prev_x_enc_val) * this->x_wheel_travel / 360.0;
    double d_y_enc = (this->y_enc_val - prev_y_enc_val) * this->x_wheel_travel / 360.0;
    double d_heading = shorter_turn(prev_imu, this->imu_heading, 360.0) * imu_muliplier;

    double d_y = 0, d_x = 0;
    if (std::abs(d_heading) < 0.01)
    {
        d_x = d_x_enc;
        d_y = d_y_enc;
    }
    else
    {
        d_y = 2 * sin_deg(d_heading / 2.0) * (d_y_enc / in_radians(d_heading) + y_wheel_placement);
        d_x = 2 * sin_deg(d_heading / 2.0) * (d_x_enc / in_radians(d_heading) + x_wheel_placement);
    }

    auto now = std::chrono::high_resolution_clock::now();
    double time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_update_time).count() / 1000.0; // in seconds

    // use a rotation matrix
    double new_heading = this->heading + d_heading;
    auto d_g_x = d_x * cos_deg(new_heading) + d_y * sin_deg(new_heading);
    auto d_g_y = -d_x * sin_deg(new_heading) + d_y * cos_deg(new_heading);

    if (this->filter != nullptr)
    {
        // NOTE: the Aps does not actually need to know how to set up the filter (i.e. it doesn't need knowledge
        // of filter-specific parameters; all it does is tell the filter what was measured: the three values above)

        // push values to filter
        Eigen::Matrix<double, 3, 1> measurements;
        if (this->pass_local_coordinates)
        {
            measurements = {d_x, d_y, d_heading};
        } else {
            measurements = {d_g_x, d_g_y, d_heading};
        }
        this->filter->correct_prediction(measurements);
        // get values from filter
        Eigen::VectorXd state = this->filter->get_state();
        // store values
        while (!this->pose_data_mutex.take(5))
            ;

        // the state matrix is {x, y, theta, dx, dy, dtheta, d2x, d2y, d2theta}, in the field frame of reference
        this->x = this->x + state(0);
        this->y = this->y + state(1);
        this->heading = mod(state(2), 360.0);

        this->dx = state(3);
        this->dy = state(4);

        this->pose_data_mutex.give();
    }
    else
    {
        // mutex for thread safety
        while (!this->pose_data_mutex.take(5))
            ;

        this->x = this->x + d_g_x;
        this->y = this->y + d_g_y;
        this->heading = mod(new_heading, 360.0);

        this->dx = d_g_x / time_elapsed;
        this->dy = d_g_y / time_elapsed;

        this->pose_data_mutex.give();
    }

    this->last_update_time = now;
}
