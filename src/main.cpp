/* home.vn2007@gmail.com - 2023 */

#include "main.h"

#include "common.hpp"
#include "portDefinitions.h"
#include "TankDrive.hpp"
#include "Aps.hpp"
#include "TwoWheelAps.hpp"
#include "TwoWheelApsBuilder.hpp"
#include "Gui.hpp"
#include "PurePursuitController.hpp"

#include <chrono>

namespace shared
{
    bool training_mode;

    bool program_running;
    int program_update_hz;
    double program_delay_per_cycle;

    pros::Motor *drive_left_1;
    pros::Motor *drive_left_2;
    pros::Motor *drive_left_top;
    pros::Motor *drive_right_1;
    pros::Motor *drive_right_2;
    pros::Motor *drive_right_top;

    TankDrive *drivetrain;

    double mult_stick_x;
    double mult_stick_y;

    pros::Imu *imu;
    Aps *aps;

    int aps_update_hz;

    pros::controller_digital_e_t intake_keybind;
    pros::controller_digital_e_t outtake_keybind;
    pros::controller_digital_e_t catapult_fire_keybind;

    namespace intake
    {
        pros::Motor *intake;
        double velocity;
    };

    namespace catapult
    {
        pros::Motor *catapult;
        pros::Distance *catapult_distance;
        double velocity;
        int loaded_threshold;
        bool ready;
        bool firing;
    };
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

    intake::velocity = 1.0;
    catapult::velocity = 0.6;
    catapult::loaded_threshold = 60;

    // ===== CONTROLS =====

    mult_stick_x = 0.75;
    mult_stick_y = 1.0;

    intake_keybind = DIGITAL_R2;
    outtake_keybind = DIGITAL_R1;
    catapult_fire_keybind = DIGITAL_L1;

    //  ===== END CONFIG =====

    program_delay_per_cycle = std::max(1000.0 / program_update_hz, 5.0); // wait no lower than 5 ms

    drive_left_1 = new pros::Motor(LEFT_DRIVE_PORT_1, MOTOR_GEAR_600, true, MOTOR_ENCODER_DEGREES);
    drive_left_2 = new pros::Motor(LEFT_DRIVE_PORT_2, MOTOR_GEAR_600, true, MOTOR_ENCODER_DEGREES);
    drive_left_top = new pros::Motor(LEFT_DRIVE_PORT_TOP, MOTOR_GEAR_600, false, MOTOR_ENCODER_DEGREES);
    drive_right_1 = new pros::Motor(RIGHT_DRIVE_PORT_1, MOTOR_GEAR_600, false, MOTOR_ENCODER_DEGREES);
    drive_right_2 = new pros::Motor(RIGHT_DRIVE_PORT_2, MOTOR_GEAR_600, false, MOTOR_ENCODER_DEGREES);
    drive_right_top = new pros::Motor(RIGHT_DRIVE_PORT_TOP, MOTOR_GEAR_600, true, MOTOR_ENCODER_DEGREES);

    std::vector<pros::Motor *> left_motors = {drive_left_1, drive_left_2, drive_left_top};
    std::vector<pros::Motor *> right_motors = {drive_right_1, drive_right_2, drive_right_top};
    drivetrain = new TankDrive(left_motors, right_motors, 0.667, 220.0, 252.0, 254.0);
    drivetrain->set_brake_mode(MOTOR_BRAKE_COAST);

    intake::intake = new pros::Motor(INTAKE_PORT, MOTOR_GEAR_600, true, MOTOR_ENCODER_DEGREES);
    intake::intake->set_brake_mode(MOTOR_BRAKE_COAST);

    // configure catapult so that the forward direction is pulling the catapult back
    catapult::catapult = new pros::Motor(CATAPULT_PORT, MOTOR_GEAR_200, false, MOTOR_ENCODER_DEGREES);
    catapult::catapult->set_brake_mode(MOTOR_BRAKE_HOLD); // BUG: do not use hold, use a ratchet
    catapult::ready = false;
    catapult::firing = false;
    catapult::catapult_distance = new pros::Distance(CATAPULT_DISTANCE_PORT);

    imu = new pros::Imu(IMU_PORT);
    imu->reset();
    while (imu->is_calibrating())
    {
        pros::delay(100);
    }

    // AbstractEncoder y_enc(drive_left_1);
    // AbstractEncoder x_enc(ODOMETRY_X_PORT); // FIXME: should this be reversed?
    // aps = TwoWheelApsBuilder()
    //           .with_y_encoder(y_enc)
    //           .with_x_encoder(x_enc)
    //           .with_imu({imu, imu_multiplier, imu_drift})
    //           .with_config(aps_config)
    //           .build();

    // pros::Task aps_update{aps_update_handler};

    program_running = true;
    pros::delay(250);
    // aps->set_pose({0.0, 0.0, 0.0});
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
        pros::screen::print(TEXT_MEDIUM, 1, "encoders: %d %d", (int)(std::floor(readings.strafe_enc)), (int)(std::floor(readings.left_enc)));

        points.push_back({pose.x, pose.y});

        graph->draw();
        graph->plot(points);

        pros::delay(20);
    }

    pros::screen::print(TEXT_MEDIUM, 2, "motion completed");
    */
}

void intake_control(pros::Controller *controller)
{
    using namespace shared;
    auto max_rpm = rpm_from_gearset(intake::intake->get_gearing());
    if (catapult::ready)
    {
        if (controller->get_digital(intake_keybind))
        {
            intake::intake->move_velocity(max_rpm * intake::velocity);
        }
        else if (controller->get_digital(outtake_keybind))
        {
            intake::intake->move_velocity(max_rpm * -intake::velocity);
        }
        else
        {
            intake::intake->brake();
        }
    } else {
        intake::intake->brake();
    }
}

void catapult_control(pros::Controller *controller)
{
    using namespace shared;
    double max_rpm = rpm_from_gearset(catapult::catapult->get_gearing());

    if (controller->get_digital(catapult_fire_keybind) && catapult::ready)
    {
        catapult::firing = true;
        catapult::ready = false;
    }

    if (catapult::firing)
    {
        catapult::catapult->move_velocity(catapult::velocity * max_rpm);
        if (catapult::catapult_distance->get() > catapult::loaded_threshold)
        {
            catapult::firing = false;
            catapult::ready = false;
        }
    }
    else if (!catapult::ready)
    {
        catapult::catapult->move_velocity(catapult::velocity * max_rpm);
        if (catapult::catapult_distance->get() < catapult::loaded_threshold)
        {
            catapult::ready = true;
            catapult::firing = false;
        }
    }

    if (catapult::ready)
    {
        catapult::catapult->brake();
    }
}

double joystick_transform(double x)
{
    double function_output = 2.3 * std::pow(std::abs(x), 3) - 2.8 * x * x + 1.4 * std::abs(x);
    if (function_output < 0)
        function_output = 0;
    if (function_output > 1)
        function_output = 1;
    if (x > 0)
    {
        return function_output;
    }
    else
    {
        return function_output * -1;
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

        auto left_stick_x = controller->get_analog(ANALOG_LEFT_X) / 127.0;
        auto right_stick_y = controller->get_analog(ANALOG_RIGHT_Y) / 127.0;

        if (std::abs(left_stick_x) < 0.02 && std::abs(right_stick_y) < 0.02)
        {
            drivetrain->brake();
        }
        else
        {
            if (training_mode && (std::abs(right_stick_y) > 0.75 || std::abs(left_stick_x) > 0.75))
            {
                drivetrain->brake();
            }
            else
            {
                // drivetrain->drive(joystick_transform(right_stick_y), joystick_transform(left_stick_x), false);
                drivetrain->drive(mult_stick_y * right_stick_y, mult_stick_x * left_stick_x, false);
            }
        }

        intake_control(controller);
        catapult_control(controller);

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

        double cycle_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - cycle_start).count();
        pros::delay(std::max(0.0, program_delay_per_cycle - cycle_time));
    }
}
