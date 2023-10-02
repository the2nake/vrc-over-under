/* home.vn2007@gmail.com - 2023 */

#include "main.h"

#include "common.hpp"
#include "portDefinitions.h"
#include "Aps.hpp"
#include "TwoWheelAps.hpp"
#include "TwoWheelApsBuilder.hpp"
#include "StarDrive.hpp"
#include "StarDriveBuilder.hpp"
#include "Gui.hpp"

#include <chrono>

namespace shared
{
    bool training_mode;
    bool tank_drive_mode;

    bool program_running;
    int program_update_hz;
    int program_delay_per_cycle;

    pros::Motor *drive_front_left;
    pros::Motor *drive_middle_left;
    pros::Motor *drive_back_left;
    pros::Motor *drive_front_right;
    pros::Motor *drive_middle_right;
    pros::Motor *drive_back_right;

    StarDrive *drivetrain;

    double mult_stick_x;
    double mult_stick_y;
    double mult_stick_r;

    pros::Imu *imu;
    Aps *aps;

    int aps_update_hz;

    pros::controller_digital_e_t imu_reset_key;
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

    training_mode = false;
    tank_drive_mode = false;

    program_update_hz = 50;
    aps_update_hz = 100;

    ApsSetup aps_config = {
        173.333333333,
        0.0, // not used
        220.0,
        -85.773987718,  // TODO: Rerecord
        0.0,            // not used
        117.355549871}; // TODO: Rerecord

    double imu_multiplier = 0.998673983;
    double imu_drift = 0.0;

    // limits are per cycle
    // at 50 hz, divide limit per second by 20
    double drive_accel_limit_lin = 0.275;
    double drive_accel_limit_rot = 0.25;

    // ===== CONTROLS =====

    mult_stick_x = 1.0;
    mult_stick_y = 1.0;
    mult_stick_r = 0.8;

    imu_reset_key = DIGITAL_A;

    //  ===== END CONFIG =====

    program_delay_per_cycle = (int)std::floor(std::max(1000.0 / program_update_hz, 5.0));

    drive_front_left = new pros::Motor(DRIVE_FRONT_LEFT_PORT, MOTOR_GEAR_BLUE, 1);
    drive_middle_left = new pros::Motor(DRIVE_MIDDLE_LEFT_PORT, MOTOR_GEAR_BLUE, 1);
    drive_back_left = new pros::Motor(DRIVE_BACK_LEFT_PORT, MOTOR_GEAR_BLUE, 1);

    drive_front_right = new pros::Motor(DRIVE_FRONT_RIGHT_PORT, MOTOR_GEAR_BLUE, 0);
    drive_middle_right = new pros::Motor(DRIVE_MIDDLE_RIGHT_PORT, MOTOR_GEAR_BLUE, 0);
    drive_back_right = new pros::Motor(DRIVE_BACK_RIGHT_PORT, MOTOR_GEAR_BLUE, 0);

    pros::Motor *left_motors[] = {drive_front_left, drive_middle_left, drive_back_left};
    pros::Motor *right_motors[] = {drive_front_right, drive_middle_right, drive_back_right};
    
    imu = new pros::Imu(IMU_PORT);
    imu->reset();
    while (imu->is_calibrating())
    {
        pros::delay(100);
    }
    
    drivetrain = StarDriveBuilder()
                     .with_left_motors(left_motors)
                     .with_right_motors(right_motors)
                     .with_geometry(252.0, 254.0)
                     .with_imu(imu)
                     .build();
    // you do not want a holonomic to coast
    drivetrain->set_brake_mode(MOTOR_BRAKE_BRAKE);

    // AbstractEncoder y_enc(drive_left_1);
    // AbstractEncoder x_enc(ODOMETRY_X_PORT); // FIXME: should this be reversed?
    // aps = TwoWheelApsBuilder()
    //           .with_y_encoder(y_enc)
    //           .with_x_encoder(x_enc)
    //           .with_imu({imu, imu_multiplier, imu_drift})
    //           .with_config(aps_config)
    //           .build();

    // pros::Task aps_update{aps_update_handler};
    // pros::delay(250);
    // aps->set_pose({0.0, 0.0, 0.0});

    program_running = true;
}

void disabled() {}

void competition_initialize() {}

void autonomous()
{
    /*PurePursuitController *ppc = new PurePursuitController(shared::drivetrain, shared::aps);
    gui::Graph *graph = new gui::Graph();
    graph->set_display_region({244, 4, 232, 232});
    graph->set_window(-1000.0, -1000.0, 2000.0, 2000.0);
    graph->point_width = 3;
    std::vector<Point<double>> points = {{2.0, 2.0}};

    ppc->set_path({{0.0, 0.0}, {0.0, 600.0}, {600.0, 600.0}}, 450.0);
    ppc->set_motion_limits(shared::drivetrain->get_max_lin_vel() * 2, 20.0);
    ppc->set_gains(1.0, 10.0);
    ppc->follow_path_async();

    while (!ppc->is_motion_completed())
    {
        auto pose = aps->get_pose();
        auto readings = aps->get_encoder_readings();
        pros::screen::print(TEXT_MEDIUM, 0, "(%.2f %.2f), theta: %.2f", pose.x, pose.y, pose.heading);
        pros::screen::print(TEXT_MEDIUM, 1, "encoders: %d %d", (int)(std::floor(readings.strafe_enc)),
                                                               (int)(std::floor(readings.left_enc)));

        points.push_back({pose.x, pose.y});

        graph->draw();
        graph->plot(points);

        pros::delay(20);
    }

    pros::screen::print(TEXT_MEDIUM, 2, "motion completed");
    */
}

void imu_reset_control(pros::Controller *controller)
{
    if (controller->get_digital_new_press(imu_reset_key))
    {
        imu->set_heading(0.0);
    }
}

void opcontrol()
{
    pros::Controller *controller = new pros::Controller(CONTROLLER_MASTER);
    /*
    gui::Graph *graph = new gui::Graph();
    graph->set_display_region({244, 4, 232, 232});
    graph->set_window(-1000.0, -1000.0, 2000.0, 2000.0);
    graph->point_width = 3;
    std::vector<Point<double>> points = {{2.0, 2.0}};
    */

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

        auto right_stick_x = mult_stick_x * controller->get_analog(ANALOG_RIGHT_X) / 127.0;
        auto right_stick_y = mult_stick_y * controller->get_analog(ANALOG_RIGHT_Y) / 127.0;

        auto left_stick_x = mult_stick_r * controller->get_analog(ANALOG_LEFT_X) / 127.0;

        double threshold = 0.01;
        bool sticks_centered = std::abs(right_stick_x) < threshold && std::abs(right_stick_y) < threshold && std::abs(left_stick_x) < threshold;
        if (sticks_centered)
        {
            drivetrain->brake();
        }
        else
        {
            drivetrain->move(distance_to(right_stick_x, right_stick_y), heading_to(right_stick_x, right_stick_y), left_stick_x);
        }
        imu_reset_control(controller);
        /*
        auto pose = aps->get_pose();
        auto readings = aps->get_encoder_readings();
        pros::screen::print(TEXT_MEDIUM, 0, "(%.2f %.2f), theta: %.2f", pose.x, pose.y, pose.heading);
        pros::screen::print(TEXT_MEDIUM, 1, "encoders: %d %d", (int)(std::floor(readings.strafe_enc)), (int)(std::floor(readings.left_enc)));

        if (points.size() > 100)
        {
            points.erase(points.begin());
        }
        points.push_back({pose.x, pose.y});

        graph->draw();
        graph->plot(points);
        */

        int cycle_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - cycle_start).count();
        pros::delay(std::max(0, program_delay_per_cycle - cycle_time));
    }
}
