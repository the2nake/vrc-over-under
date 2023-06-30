#include "TwoWheelApsBuilder.hpp"

TwoWheelApsBuilder &TwoWheelApsBuilder::with_encoders(EncoderSetup x_setup, EncoderSetup y_setup)
{
    this->x_encoder = new pros::ADIEncoder(x_setup.top, x_setup.bottom, x_setup.reversed);
    this->y_encoder = new pros::ADIEncoder(y_setup.top, y_setup.bottom, y_setup.reversed);

    if (errno == ENXIO || errno == ENODEV)
        this->disabled = true;

    return *this;
}

TwoWheelApsBuilder &TwoWheelApsBuilder::with_config(ApsSetup aps_setup)
{
    this->x_wheel_placement = aps_setup.strafe_wheel_distance;
    this->y_wheel_placement = -aps_setup.left_wheel_distance;

    this->x_wheel_travel = aps_setup.strafe_wheel_travel;
    this->y_wheel_travel = aps_setup.left_wheel_travel;
    this->aps_setup_given = true;

    return *this;
}

TwoWheelApsBuilder &TwoWheelApsBuilder::with_imu(ImuSetup imu_setup)
{
    // if invalid imu address or imu is too unreliable
    if (imu_setup.imu == nullptr || std::abs(imu_setup.multiplier) < 0.1)
        this->disabled = true;

    this->imu = imu_setup.imu;
    this->imu_drift = imu_setup.drift;
    this->imu_muliplier = imu_setup.multiplier;

    return *this;
}

TwoWheelApsBuilder &TwoWheelApsBuilder::with_filter(Filter *filter)
{
    this->filter = filter;
    return *this;
}

TwoWheelAps *TwoWheelApsBuilder::build()
{
    if (this->disabled || this->imu == nullptr || this->x_encoder == nullptr || this->y_encoder == nullptr || !aps_setup_given)
    {
        return nullptr;
    }

    TwoWheelAps *aps = new TwoWheelAps();
    aps->x_encoder = this->x_encoder;
    aps->y_encoder = this->y_encoder;

    aps->x_wheel_placement = this->x_wheel_placement;
    aps->y_wheel_placement = this->y_wheel_placement;
    aps->x_wheel_travel = this->x_wheel_travel;
    aps->y_wheel_travel = this->y_wheel_travel;

    aps->imu = this->imu;
    aps->imu_drift = this->imu_drift;
    aps->imu_muliplier = this->imu_muliplier;

    aps->filter = this->filter;
    aps->last_update_time = std::chrono::high_resolution_clock::now();

    return aps;
}
