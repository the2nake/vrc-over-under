/* home.vn2007@gmail.com - 2023 */

#include "HermiteSpline.hpp"

PathPoint HermiteSpline::get_point(double u)
{
    PathPoint point;

    if (u > this->control_points.size() - 1 || u < 0)
    {
        return point;
    }

    if (std::floor(u) == u)
    {
        point.x = this->control_points[(int)(u)](0, 0);
        point.y = this->control_points[(int)(u)](0, 1);
        point.dx = this->control_points[(int)(u)](1, 0);
        point.dy = this->control_points[(int)(u)](1, 1);
        return point;
    }

    Eigen::Matrix<double, 2, 2> starting_point = this->control_points[(int)(std::floor(u))];
    Eigen::Matrix<double, 2, 2> ending_point = this->control_points[(int)(std::floor(u) + 1)];

    double t = u - std::floor(u);

    Eigen::RowVector4d t_matrix = {1, t, t * t, t * t * t}; // matrix of the powers of u - std::floor(u)

    Eigen::Matrix<double, 4, 4> characteristic_matrix{
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {-3, -2, 3, -1},
        {2, 1, -2, 1}};

    Eigen::Matrix<double, 1, 2> p_0 = starting_point.row(0);
    Eigen::Matrix<double, 1, 2> v_0 = starting_point.row(1);
    Eigen::Matrix<double, 1, 2> p_1 = ending_point.row(0);
    Eigen::Matrix<double, 1, 2> v_1 = ending_point.row(1);
    Eigen::Matrix<double, 4, 2> p_matrix{
        {p_0(0, 0), p_0(0, 1)},
        {v_0(0, 0), v_0(0, 1)},
        {p_1(0, 0), p_1(0, 1)},
        {v_1(0, 0), v_1(0, 1)}};

    // use matrix equation to find position
    Eigen::RowVector2d point_position = t_matrix * characteristic_matrix * p_matrix;
    point.x = point_position(0, 0);
    point.y = point_position(0, 1);
    // estimate velocity with derivative? or find the exact value using another equation
    return point;
}
