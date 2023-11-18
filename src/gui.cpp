/* home.vn2007@gmail.com - 2023 */

#include "gui.hpp"

#include <cmath>

namespace gui {
Graph::Graph() {
  axes_colour = 0x00707070;
  point_colour = 0x00ffffff;
  bg_colour = 0x00000000;
}

void Graph::set_window(double x, double y, double w, double h) {
  win_x = x;
  win_y = y;
  win_w = w;
  win_h = h;
}

void Graph::set_display_region(ScreenArea region) { display_region = region; }

void Graph::plot(std::vector<Point<double>> &points) {
  for (auto point : points) {
    int pixel_x = display_region.x +
                  std::floor((point.x - win_x) * (display_region.w / win_w));
    int pixel_y = display_region.y + display_region.h -
                  std::floor((point.y - win_y) * (display_region.h / win_h));
    pros::screen::set_pen(point_colour);
    pros::screen::fill_circle(pixel_x, pixel_y, std::ceil(point_width / 2));
  }
}

void Graph::draw() {
  clear();

  short x_axis_pixel_y =
      display_region.y + std::floor(-(win_y) * (display_region.h / win_h));
  short y_axis_pixel_x =
      display_region.x + std::floor(-(win_x) * (display_region.w / win_w));
  pros::screen::set_pen(axes_colour);
  pros::screen::draw_rect(display_region.x, display_region.y,
                          display_region.x + display_region.w,
                          display_region.y + display_region.h);
  pros::screen::draw_line(display_region.x, x_axis_pixel_y,
                          display_region.x + display_region.w, x_axis_pixel_y);
  pros::screen::draw_line(y_axis_pixel_x, display_region.y, y_axis_pixel_x,
                          display_region.y + display_region.h);
}

void Graph::clear() {
  pros::screen::set_pen(bg_colour);
  pros::screen::fill_rect(display_region.x, display_region.y,
                          display_region.x + display_region.w,
                          display_region.y + display_region.h);
  pros::screen::set_pen(axes_colour);
  pros::screen::draw_rect(display_region.x, display_region.y,
                          display_region.x + display_region.w,
                          display_region.y + display_region.h);
}
}; // namespace gui