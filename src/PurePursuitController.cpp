/* home.vn2007@gmail.com - 2023 */

#include "PurePursuitController.hpp"

PurePursuitController::PurePursuitController(TankDrive *drivetrain, Aps *aps)
{
    this->drivetrain = drivetrain;
    this->aps = aps;
    this->track_width = drivetrain->get_track_width();
    this->max_vel = drivetrain->get_max_lin_vel();
}

void PurePursuitController::set_path(std::vector<PurePursuitWaypoint> points, double look_ahead)
{
    while (this->is_moving())
    {
        pros::delay(10);
    }

    for (int i = 0; i < points.size() - 1; i++)
    {
        Segment<double> segment{points[i].x, points[i].y, points[i + 1].x, points[i + 1].y};
        this->remaining_segments.push_back(segment);
    }
    this->look_ahead = look_ahead;
    auto pose = this->aps->get_pose();
    this->lowest_distance_from_end = distance_between_points<double>({pose.x, pose.y}, {points.back().x, points.back().y});
    this->motion_completed = false;
}

void PurePursuitController::set_gains(double lin_kP, double rot_kP)
{
    this->lin_kP = std::abs(lin_kP);
    this->rot_kP = std::abs(rot_kP);
}

void PurePursuitController::set_motion_limits(double max_acceleration, double accuracy)
{
    this->max_acceleration = std::abs(max_acceleration);
    this->accuracy = std::max(std::abs(accuracy), 10.0);
}

void path_following_loop(void *param)
{
    // fatal error will occur if param is not of PurePursuitController* type
    PurePursuitController *controller = (PurePursuitController *)(param);
    while (!controller->is_motion_completed())
    {
        controller->follow_path_step();
        pros::delay(10);
    }
}

void PurePursuitController::follow_path_async()
{
    if (this->is_motion_completed() || this->remaining_segments.size() == 0)
        return;

    if (this->max_acceleration <= 0.0)
        return;

    this->moving = true;
    this->last_update_time = std::chrono::high_resolution_clock::now();
    this->update_task = new pros::Task(path_following_loop, (void *)this, "Pure Pursuit Task");
}

void PurePursuitController::follow_path_step()
{
    auto pose = this->aps->get_pose();
    Point<double> current_pos{pose.x, pose.y};

    Point<double> goal;
    Point<double> next_endpoint;

    if (this->remaining_segments.size() == 0)
    {
        this->motion_completed = true;
        this->moving = false;
        return;
    }

    if (this->remaining_segments.size() == 1)
    {
        // this is the final segment
        if (distance_between_points(current_pos, this->remaining_segments[0].end()) < this->look_ahead)
        {
            // if end is within the look ahead zone
            goal = this->remaining_segments[0].end();
        }
        else
        {
            try
            {
                auto intersection = find_last_intersection(current_pos, this->look_ahead, this->remaining_segments[0]);
                goal = intersection;
            }
            catch (const char *msg)
            {
                goal = this->remaining_segments[0].start();
            }
        }
    }
    else
    {
        if (distance_between_points(current_pos, this->remaining_segments[0].end()) < this->look_ahead)
        {
            this->remaining_segments.erase(this->remaining_segments.begin());
        }

        // there are no intersections with the second segment, continue as normal
        try
        {
            auto intersection = find_last_intersection(current_pos, this->look_ahead, this->remaining_segments[0]);
            goal = intersection;
        }
        catch (const char *msg)
        {
            goal = this->remaining_segments[0].start();
        }
    }

    // requires a goal point to be set

    double lin_err = distance_between_points(goal, {pose.x, pose.y});
    double goal_heading = mod(90.0 - in_deg(std::atan2(goal.y - pose.y, goal.x - pose.x)), 360.0);
    double turn_err = shorter_turn(pose.heading, goal_heading, 360.0);

    auto now = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_update_time).count() / 1000.0; // in seconds
    this->last_update_time = now;

    double inno = this->lin_kP * lin_err - this->curr_vel;
    limit_magnitude(inno, this->max_acceleration * dt);
    this->curr_vel += inno;

    double turn_vel = this->rot_kP * turn_err;

    // should only happen if you're at the end
    Point<double> path_end = {this->remaining_segments.back().x1, this->remaining_segments.back().y1};
    double distance_from_end = distance_between_points({pose.x, pose.y}, path_end);
    if ((distance_from_end < this->accuracy || (distance_from_end > 1.1 * this->lowest_distance_from_end && distance_from_end < this->accuracy * 3)) &&
        this->remaining_segments.size() == 1)
    {
        this->moving = false;
        this->motion_completed = true;
        this->drivetrain->brake();
    }
    else
    {
        this->moving = true;
        this->motion_completed = false;
        this->drivetrain->drive_tank(this->drivetrain->to_pct(this->curr_vel + turn_vel), this->drivetrain->to_pct(this->curr_vel - turn_vel));
    }

    this->lowest_distance_from_end = std::min(this->lowest_distance_from_end, distance_from_end);

    pros::screen::print(TEXT_MEDIUM, 3, "goal: (%.2f, %.2f)", goal.x, goal.y);
    pros::screen::print(TEXT_MEDIUM, 4, "endpoint: (%.2f, %.2f)", next_endpoint.x, next_endpoint.y);
    pros::screen::print(TEXT_MEDIUM, 5, "goal_h: %.2f err: %.2f", goal_heading, turn_err);
    pros::screen::print(TEXT_MEDIUM, 6, "curr: %.2f turn: %.2f", this->curr_vel, turn_vel);
}