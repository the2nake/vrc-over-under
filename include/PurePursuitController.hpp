/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "TankDrive.hpp"
#include "Aps.hpp"
#include "Spline.hpp"
#include "common.hpp"

#include <vector>
#include <atomic>
#include <chrono>

struct PurePursuitWaypoint
{
    double x;
    double y;
};

class PurePursuitController
{
    friend void path_following_loop(void *);

public:
    PurePursuitController(TankDrive *drivetrain, Aps *aps);

    void set_path(std::vector<PurePursuitWaypoint> points, double look_ahead);
    // resolution is number of points to sample aside from start and finish. 0 resolution will sample (8 * waypoints) points
    void set_path(Spline *path, double look_ahead, int resolution = 0);
    // velocity here refers to the velocity along the arc that pure pursuit drives the drivetrain on
    void set_motion_limits(double max_acceleration, double accuracy);
    void set_gains(double lin_kP, double rot_kP);
    void follow_path_async();

    bool is_motion_completed() { return this->motion_completed.load(); }
    bool is_moving() { return this->moving.load(); }

private:
    void follow_path_step();

    TankDrive *drivetrain = nullptr;
    Aps *aps = nullptr;

    double track_width = 0.0;

    // segments that have already been passed are removed
    std::vector<Segment<double>> remaining_segments = {};
    double look_ahead = 0.0;

    double lin_kP = 0.0;
    double rot_kP = 0.0;

    double curr_vel = 0.0;
    double max_vel = 0.0;
    double max_acceleration = 0.0;

    double accuracy = 10.0;
    double lowest_distance_from_end = 0.0;

    std::atomic<bool> motion_completed = true;
    std::atomic<bool> moving = false;

    pros::Task *update_task = nullptr;

    std::chrono::high_resolution_clock::time_point last_update_time;
};

void path_following_loop(void *param);
