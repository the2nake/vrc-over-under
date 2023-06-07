/* home.vn2007@gmail.com - 2023 */

#include "main.h"
#include "portDefinitions.h"
#include "TankDrive.hpp"
#include "APS.hpp"
#include "TwoWheelAPS.hpp"

#include <chrono>

namespace shared
{
    bool program_running;
    int program_update_hz;
    double program_delay_per_cycle;

    pros::Motor *left_motor_1;
    pros::Motor *left_motor_2;
    pros::Motor *left_motor_top;
    pros::Motor *right_motor_1;
    pros::Motor *right_motor_2;
    pros::Motor *right_motor_top;

    TankDrive *drivetrain;

    pros::Imu *imu;
    APS *aps;
};

using namespace shared;

void initialize()
{
    // ===== CONFIGURATION =====

    program_update_hz = 48;

    ApsSetup aps_config = {
        220.0,
        0.0, // not used
        220.0,
        100.0, // TODO: remeasure in mm
        0.0, // not used
        100.0 // TODO: remeasure in mm
    };

    double imu_multiplier = 1.0;
    double imu_drift = 0.0;

    //  ===== END CONFIG =====

    program_delay_per_cycle = std::max(1000.0 / program_update_hz, 5.0); // wait no lower than 5 ms

    left_motor_1 = new pros::Motor(LEFT_DRIVE_PORT_1, MOTOR_GEAR_600, true, MOTOR_ENCODER_DEGREES);
    left_motor_2 = new pros::Motor(LEFT_DRIVE_PORT_2, MOTOR_GEAR_600, true, MOTOR_ENCODER_DEGREES);
    left_motor_top = new pros::Motor(LEFT_DRIVE_PORT_TOP, MOTOR_GEAR_600, false, MOTOR_ENCODER_DEGREES);
    right_motor_1 = new pros::Motor(RIGHT_DRIVE_PORT_1, MOTOR_GEAR_600, false, MOTOR_ENCODER_DEGREES);
    right_motor_2 = new pros::Motor(RIGHT_DRIVE_PORT_2, MOTOR_GEAR_600, false, MOTOR_ENCODER_DEGREES);
    right_motor_top = new pros::Motor(RIGHT_DRIVE_PORT_TOP, MOTOR_GEAR_600, true, MOTOR_ENCODER_DEGREES);

    drivetrain = new TankDrive({left_motor_1, left_motor_2, left_motor_top}, {right_motor_1, right_motor_2, right_motor_top}, 0.8, 220.0);
    drivetrain->set_brake_mode(MOTOR_BRAKE_COAST);

    imu = new pros::Imu(IMU_PORT);
    aps = new TwoWheelAPS({X_ENCODER_PORT_TOP, X_ENCODER_PORT_BOTTOM, X_ENCODER_REVERSED}, {Y_ENCODER_PORT_TOP, Y_ENCODER_PORT_BOTTOM, Y_ENCODER_REVERSED},
                          aps_config, {imu, imu_multiplier, imu_drift});

    program_running = true;
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol()
{
    pros::Controller *controller = new pros::Controller(CONTROLLER_MASTER);

    while (program_running)
    {
        auto cycle_start = std::chrono::high_resolution_clock::now();

        auto left_stick_x = controller->get_analog(ANALOG_LEFT_X) / 127.0;
        auto right_stick_y = controller->get_analog(ANALOG_RIGHT_Y) / 127.0;

        if (std::abs(left_stick_x) < 0.02 && std::abs(right_stick_y) < 0.02)
        {
            drivetrain->brake();
        }
        else
        {
            drivetrain->drive(right_stick_y, left_stick_x, false, 0.0, 0.65);
        }

        aps->update();

        double cycle_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - cycle_start).count();
        pros::delay(std::max(0.0, program_delay_per_cycle - cycle_time));
    }
}
