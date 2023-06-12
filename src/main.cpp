/* home.vn2007@gmail.com - 2023 */

#include "main.h"

#include "portDefinitions.h"
#include "TankDrive.hpp"
#include "APS.hpp"
#include "TwoWheelAPS.hpp"
#include "GUI.hpp"

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

    int aps_update_hz;
};

using namespace shared;

void aps_update_handler(void *param)
{
    int updateDelay = (int)(1000 / aps_update_hz); // in ms
    while (aps != nullptr)
    {
        aps->update();

        pros::delay(updateDelay);
    }
}

void initialize()
{
    // ===== CONFIGURATION =====

    program_update_hz = 50;
    aps_update_hz = 100;

    ApsSetup aps_config = {
        220.0,
        0.0, // not used
        220.0,
        -85.773987718,
        0.0, // not used
        117.355549871};

    double imu_multiplier = 0.998673983;
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
    imu->reset();
    while (imu->is_calibrating())
    {
        pros::delay(100);
    }

    aps = new TwoWheelAPS({X_ENCODER_PORT_TOP, X_ENCODER_PORT_BOTTOM, X_ENCODER_REVERSED}, {Y_ENCODER_PORT_TOP, Y_ENCODER_PORT_BOTTOM, Y_ENCODER_REVERSED},
                          aps_config, {imu, imu_multiplier, imu_drift});

    pros::Task aps_update{aps_update_handler};

    program_running = true;
    pros::delay(250);
    aps->set_pose({0.0, 0.0, 0.0});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol()
{
    pros::Controller *controller = new pros::Controller(CONTROLLER_MASTER);
    gui::Graph *graph = new gui::Graph();
    graph->set_display_region({244, 4, 232, 232});
    graph->set_window(-1000.0, -1000.0, 2000.0, 2000.0);
    graph->point_width = 3;
    std::vector<Point<double>> points = {{2.0, 2.0}};

    /* test-drive-path
    drivetrain->set_brake_mode(MOTOR_BRAKE_BRAKE);
    for (int i = 0; i < 4; i++)
    {
        drivetrain->drive(0.25, 0.0, false);
        pros::delay(1000);
        drivetrain->brake();
        auto heading = imu->get_heading();
        drivetrain->drive(0.0, -0.1, false);
        while (std::abs(-90 - (imu->get_heading() - heading)) > 5)
        {
            pros::delay(10);
        }
        drivetrain->brake();
    }
    */

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
            drivetrain->drive(right_stick_y, left_stick_x * 0.65, false);
        }

        auto pose = aps->get_pose();
        auto encoder = aps->get_encoder_readings();
        pros::screen::print(TEXT_MEDIUM, 0, "(%.2f %.2f), theta: %.2f", pose.x, pose.y, pose.heading);
        pros::screen::print(TEXT_MEDIUM, 1, "encoders: %d %d", (int)(std::floor(encoder.strafe_enc)), (int)(std::floor(encoder.left_enc)));

        if (points.size() > 100)
        {
            points.erase(points.begin());
        }
        points.push_back({pose.x, pose.y});

        graph->draw();
        graph->plot(points);

        double cycle_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - cycle_start).count();
        pros::delay(std::max(0.0, program_delay_per_cycle - cycle_time));
    }
}
