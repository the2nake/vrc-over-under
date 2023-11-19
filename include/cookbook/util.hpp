/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"

#include <cmath>
#include <vector>

template <typename T> struct Point {
  T x = 0;
  T y = 0;
};

template <typename T> struct Segment {
  T x0 = 0;
  T y0 = 0;
  T x1 = 0;
  T y1 = 0;

  /**
   * @returns The starting point
   */
  Point<T> start() { return {x0, y0}; }

  /**
   * @returns The end point
   */
  Point<T> end() { return {x1, y1}; }
};

/**
 * Get the rpm of a gearset
 *
 * @param gearing The gearset
 * @returns The rpm
 */
double rpm_from_gearset(pros::motor_gearset_e_t gearing);

/**
 * Scales down the magnitudes of a and b proportionally to go under max
 *
 * @param a A variable
 * @param b Another variable
 * @param max The maximum magnitude
 */
void scale_down_magnitude(double &a, double &b, double max);

/**
 * A function to limit the magnitude of a variable
 *
 * @param a A variable
 * @param max The maximum magnitude
 */
void limit_magnitude(double &a, double max);

double mod(double x, double modulo);

/**
 * Returns the shortest turn to get from h_0 to h_f
 *
 * @param h_0 The starting heading
 * @param h_f The final heading
 * @param circum The number of units in the circle
 * @returns The shortest left (negative) or right (positive) turn to get from
 * h_0 to h_f
 */
double shorter_turn(double h_0, double h_f, double circum);

/**
 * Returns a value denoting the sign of a number
 *
 * @param val The number to check
 * @returns One of {-1, 0, 1} corresponding to the sign of the number
 */
template <typename T> int sgn(T val) {
  if (val = 0) {
    return 0;
  }

  if (val > 0) {
    return 1;
  }

  return -1;
}

/**
 * Converts an angle from radians to degrees
 *
 * @param rad The angle in radians
 * @returns The angle in degrees
 */
template <typename T> T in_deg(T rad) { return rad * 57.2957795131; }

/**
 * Converts an angle from degrees to radians
 *
 * @param deg The angle in degrees
 * @returns The angle in radians
 */
template <typename T> T in_radians(T deg) { return 0.01745329251 * deg; }

/**
 * Calculates the degrees sine
 *
 * @param deg The angle in degrees
 * @returns The sine of the angle
 */
template <typename T> T sin_deg(T deg) { return std::sin(in_radians(deg)); }

/**
 * Calculates the degrees cosine
 *
 * @param deg The angle in degrees
 * @returns The cosine of the angle
 */
template <typename T> T cos_deg(T deg) { return std::cos(in_radians(deg)); }

/**
 * @brief calculates the radius value of the polar representation of a cartesian
 * coordinate
 */
template <typename T> T polar_radius(T x, T y) {
  return std::sqrt(x * x + y * y);
}

/**
 * A function that returns the circle intersection closest to the end of the
 * segment
 *
 * @param centre The center of the circle
 * @param radius The radius of the circle
 * @param segment The line segment to check intersection with
 * @returns The circle intersection closest to the end of the segment
 *
 * @exception Throws "No intersection" if there is no intersection
 */
template <typename T>
Point<T> find_last_intersection(Point<T> centre, double radius,
                                Segment<T> segment) {
  double x1 = segment.start().x;
  double x2 = segment.end().x;
  double y1 = segment.start().y;
  double y2 = segment.end().y;

  double x0 = centre.x;
  double y0 = centre.y;

  x1 -= x0;
  y1 -= y0;
  x2 -= x0;
  y2 -= y0;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr = std::sqrt(dx * dx + dy * dy);
  double D = x1 * y2 - x2 * y1;

  if (radius * radius * dr * dr - D * D < 0) {
    throw "No intersection";
  }

  double x_swing =
      (dy < 0 ? -1 : 1) * dx * std::sqrt(radius * radius * dr * dr - D * D);
  double y_swing = std::abs(dy) * std::sqrt(radius * radius * dr * dr - D * D);

  double sol1x = x0 + (D * dy + x_swing) / (dr * dr);
  double sol1y = y0 + (-D * dx + y_swing) / (dr * dr);
  double sol2x = x0 + (D * dy - x_swing) / (dr * dr);
  double sol2y = y0 + (-D * dx - y_swing) / (dr * dr);

  x1 += x0;
  y1 += y0;
  x2 += x0;
  y2 += y0;

  bool sol1_valid = false;
  bool sol2_valid = false;

  if (std::min(x1, x2) - 0.0001 <= sol1x &&
      sol1x <= std::max(x1, x2) + 0.0001) {
    sol1_valid = true;
  }
  if (std::min(x1, x2) - 0.0001 <= sol2x &&
      sol2x <= std::max(x1, x2) + 0.0001) {
    sol2_valid = true;
  }
  if ((sol1_valid && sol2_valid) &&
      (std::abs(sol1x - sol2x) < 0.001 && std::abs(sol1y - sol2y) < 0.001)) {
    sol2_valid = false;
  }

  if ((!sol1_valid) && (!sol2_valid)) {
    return {static_cast<T>(nan("")), static_cast<T>(nan(""))};
  }

  if (sol1_valid && !sol2_valid) {
    return {sol1x, sol1y};
  }

  if (!sol1_valid && sol2_valid) {
    return {sol2x, sol2y};
  }

  if (std::abs(x2 - sol1x) < std::abs(x2 - sol2x)) {
    return {sol1x, sol1y};
  } else if (std::abs(y2 - sol1y) < std::abs(y2 - sol2y)) {
    return {sol1x, sol1y};
  } else {
    return {sol2x, sol2y};
  }
}

/**
 * A function that appends all of the relevant intersections to an output vector
 *
 * @param centre The center of the circle
 * @param radius The radius of the circle
 * @param segment The line segment to check intersection with
 * @param output The output vector
 */
void find_intersections(Point<double> centre, double radius,
                        Segment<double> segment,
                        std::vector<Point<double>> &output);

/**
 * Calculates the distance between two points
 *
 * @param a The first point
 * @param b The second point
 * @returns The distance between the points
 */
template <typename T> T distance_between_points(Point<T> a, Point<T> b) {
  double dx = (a.x - b.x);
  double dy = (a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

/**
 * Picks the closest point to a goal
 *
 * @param a The first point
 * @param b The second point
 * @param goal The goal point
 * @returns The closest point to the goal
 */
template <typename T>
Point<T> pick_closest_point_to(Point<T> a, Point<T> b, Point<T> goal) {
  if (distance_between_points(a, goal) < distance_between_points(b, goal)) {
    return a;
  } else {
    return b;
  }
}

/**
 * Picks the closer point to a goal
 *
 * @param points A vector containing the points
 * @param goal The goal point
 * @returns The closest point to the goal
 */
template <typename T>
Point<T> pick_closest_point_to(std::vector<Point<T>> points, Point<T> goal) {
  if (points.size() == 0) {
    return {0, 0};
  }

  Point<T> closest_point = points[0];
  T closest_distance = distance_between_points<T>(points[0], goal);

  for (Point<T> point : points) {
    T current_distance = distance_between_points<T>(point, goal);
    if (current_distance < closest_distance) {
      closest_distance = current_distance;
      closest_point = point;
    }
  }

  return closest_point;
}
