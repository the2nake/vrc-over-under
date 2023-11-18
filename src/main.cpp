/* home.vn2007@gmail.com - 2023 */

#include "main.h"

#include "common.hpp"
#include "portDefinitions.h"
#include "TankDrive.hpp"
#include "TankDriveBuilder.hpp"
#include "Aps.hpp"
#include "TwoWheelAps.hpp"
#include "TwoWheelApsBuilder.hpp"
#include "Gui.hpp"
#include "PurePursuitController.hpp"

#include <chrono>

namespace shared
{
    bool training_mode;
    bool tank_drive_mode;

    bool program_running;
    int program_update_hz;
    int program_delay_per_cycle;

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
    pros::controller_digital_e_t intake_actuate_keybind;

    namespace intake
    {
        pros::Motor *intake;
        pros::ADIDigitalOut *piston;
        double velocity;
        bool extended;
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
    tank_drive_mode = false;

    program_update_hz = 40;
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
    double drive_accel_limit_rot = 1000; // 0.35;

    intake::velocity = 0.9;
    catapult::velocity = 0.55;
    catapult::loaded_threshold = 77;

    // ===== CONTROLS =====

    mult_stick_x = 0.6;
    mult_stick_y = 1.0;

    intake_keybind = DIGITAL_R2;
    outtake_keybind = DIGITAL_R1;
    catapult_fire_keybind = DIGITAL_L1;

    intake_actuate_keybind = DIGITAL_A;

    //  ===== END CONFIG =====

    program_delay_per_cycle = (int)std::floor(std::max(1000.0 / program_update_hz, 5.0));

    imu = new pros::Imu(IMU_PORT);
    imu->reset();
    while (imu->is_calibrating())
    {
        pros::delay(100);
    }

    drive_left_1 = new pros::Motor(LEFT_DRIVE_PORT_1, MOTOR_GEAR_600, true, MOTOR_ENCODER_DEGREES);
    drive_left_2 = new pros::Motor(LEFT_DRIVE_PORT_2, MOTOR_GEAR_600, true, MOTOR_ENCODER_DEGREES);
    drive_left_top = new pros::Motor(LEFT_DRIVE_PORT_TOP, MOTOR_GEAR_600, false, MOTOR_ENCODER_DEGREES);

    drive_right_1 = new pros::Motor(RIGHT_DRIVE_PORT_1, MOTOR_GEAR_600, false, MOTOR_ENCODER_DEGREES);
    drive_right_2 = new pros::Motor(RIGHT_DRIVE_PORT_2, MOTOR_GEAR_600, false, MOTOR_ENCODER_DEGREES);
    drive_right_top = new pros::Motor(RIGHT_DRIVE_PORT_TOP, MOTOR_GEAR_600, true, MOTOR_ENCODER_DEGREES);

    std::vector<pros::Motor *> left_motors = {drive_left_1, drive_left_2, drive_left_top};
    std::vector<pros::Motor *> right_motors = {drive_right_1, drive_right_2, drive_right_top};
    drivetrain = TankDriveBuilder()
                     .with_left_motors(left_motors)
                     .with_right_motors(right_motors)
                     .with_gear_ratio(0.66667)
                     .with_wheel_travel(260.0)
                     .with_geometry(252.0, 254.0)
                     .with_imu(imu)
                     .build();
    drivetrain->set_brake_mode(MOTOR_BRAKE_COAST);
    drivetrain->set_accel_limit(drive_accel_limit_lin, drive_accel_limit_rot);

    intake::intake = new pros::Motor(INTAKE_PORT, MOTOR_GEAR_600, true, MOTOR_ENCODER_DEGREES);
    intake::intake->set_brake_mode(MOTOR_BRAKE_COAST);
    intake::extended = false;
    intake::piston = new pros::ADIDigitalOut(INTAKE_PISTON_PORT, intake::extended);

    // configure catapult so that the forward direction is pulling the catapult back
    catapult::catapult = new pros::Motor(CATAPULT_PORT, MOTOR_GEAR_200, false, MOTOR_ENCODER_DEGREES);
    catapult::catapult->set_brake_mode(MOTOR_BRAKE_COAST); // for safety
    catapult::ready = false;
    catapult::firing = false;
    catapult::catapult_distance = new pros::Distance(CATAPULT_DISTANCE_PORT);

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
    // drivetrain->drive(1, 0, false, false);
    // pros::delay(1000);
    // drivetrain->brake();

    auto max_rpm = rpm_from_gearset(intake::intake->get_gearing());

    while (catapult::catapult_distance->get() > catapult::loaded_threshold)
    {
        catapult::catapult->move_velocity(catapult::velocity * max_rpm);
        pros::delay(20);
    }
    catapult::catapult->brake();

    for (int i = 0; i < 40; i++)
    {
        drivetrain->drive(-1.0, 0.0, false, false);
        pros::delay(25);
    }
    drivetrain->brake();

    drivetrain->drive_proportional_pos(200, 200, 0.0020); // in mm

    for (int i = 0; i < 30; i++)
    {
        drivetrain->drive_tank(1.0, 0.1, false);
        pros::delay(25);
    }
    drivetrain->brake();

    imu->set_heading(315);

    drivetrain->swing_pid_heading(0.0, -1.0, 340.0, 0.005, 0.0, 0.0);

    intake::piston->set_value(1);

    for (int i = 0; i < 44; i++)
    {
        while (catapult::catapult_distance->get() > catapult::loaded_threshold)
        {
            catapult::catapult->move_voltage(12000);
            pros::delay(20);
        } // load
        catapult::catapult->brake();
        pros::delay(400);
        while (catapult::catapult_distance->get() < catapult::loaded_threshold)
        {
            catapult::catapult->move_voltage(12000);
            pros::delay(20);
        } // fire
    }     // count 44 shots

    while (catapult::catapult_distance->get() > catapult::loaded_threshold)
    {
        catapult::catapult->move_velocity(catapult::velocity * max_rpm);
        pros::delay(20);
    } // load

    catapult::catapult->brake();

    // DO DRIVING HERE

    // drivetrain->swing_pid_heading(-0.75, -1.0, 225.0, 0.005, 0.0, 0.0);

    // drivetrain->swing_pid_heading(1.0, -0.5, 135.0, 0.005, 0.0, 0.0);
    // drivetrain->swing_pid_heading(1.0, -0.5, 315.0, 0.005, 0.0, 0.0);
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
    }
    else
    {
        intake::intake->brake();
    }

    if (controller->get_digital_new_press(intake_actuate_keybind))
    {
        intake::extended = !intake::extended;
    }

    intake::piston->set_value(intake::extended);
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
        auto left_stick_y = controller->get_analog(ANALOG_LEFT_Y) / 127.0;

        if (tank_drive_mode)
        {
            if (std::abs(left_stick_y) < 0.02 && std::abs(right_stick_y) < 0.02)
            {
                drivetrain->brake();
            }
            else
            {
                if (training_mode)
                {

                    if (std::abs(right_stick_y) > 0.75 || std::abs(left_stick_y) > 0.75)
                    {
                        drivetrain->brake();
                    }
                    else
                    {
                        drivetrain->drive_tank(left_stick_y, right_stick_y, false);
                    }
                }
                else
                {
                    drivetrain->drive_tank(left_stick_y, right_stick_y, false);
                }
            }
        }
        else
        {
            if (std::abs(left_stick_x) < 0.02 && std::abs(right_stick_y) < 0.02)
            {
                drivetrain->brake();
            }
            else
            {
                if (training_mode)
                {
                    if (std::abs(right_stick_y) > 0.75 || std::abs(left_stick_x) > 0.75)
                    {
                        drivetrain->brake();
                    }
                    else
                    {
                        drivetrain->drive(right_stick_y, left_stick_x, false, false);
                    }
                }
                else
                {
                    drivetrain->drive(mult_stick_y * right_stick_y, mult_stick_x * left_stick_x, false, false);
                }
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

        int cycle_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - cycle_start).count();
        pros::delay(std::max(0, program_delay_per_cycle - cycle_time));
    }
}
