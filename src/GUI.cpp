#include "GUI.hpp"

#include <cmath>

namespace gui
{
    Graph::Graph()
    {
        this->axes_colour = 0x00707070;
        this->point_colour = 0x00ffffff;
        this->bg_colour = 0x00000000;
    }

    void Graph::set_window(double x, double y, double w, double h)
    {
        this->win_x = x;
        this->win_y = y;
        this->win_w = w;
        this->win_h = h;
    }

    void Graph::set_display_region(ScreenArea region)
    {
        this->display_region = region;
    }

    void Graph::plot(std::vector<Point<double>> &points)
    {
        for (auto point : points)
        {
            int pixel_x = this->display_region.x + std::floor((point.x - win_x) * (this->display_region.w / win_w));
            int pixel_y = this->display_region.y + this->display_region.h - std::floor((point.y - win_y) * (this->display_region.h / win_h));
            pros::screen::set_pen(this->point_colour);
            pros::screen::fill_circle(pixel_x, pixel_y, std::ceil(this->point_width / 2));
        }
    }

    void Graph::draw()
    {
        this->clear();

        short x_axis_pixel_y = this->display_region.y + std::floor(-(win_y) * (this->display_region.h / win_h));
        short y_axis_pixel_x = this->display_region.x + std::floor(-(win_x) * (this->display_region.w / win_w));
        pros::screen::set_pen(this->axes_colour);
        pros::screen::draw_rect(this->display_region.x, this->display_region.y, this->display_region.x + this->display_region.w, this->display_region.y + this->display_region.h);
        pros::screen::draw_line(this->display_region.x, x_axis_pixel_y, this->display_region.x + this->display_region.w, x_axis_pixel_y);
        pros::screen::draw_line(y_axis_pixel_x, this->display_region.y, y_axis_pixel_x, this->display_region.y + this->display_region.h);
    }

    void Graph::clear()
    {
        pros::screen::set_pen(this->bg_colour);
        pros::screen::fill_rect(this->display_region.x, this->display_region.y, this->display_region.x + this->display_region.w, this->display_region.y + this->display_region.h);
        pros::screen::set_pen(this->axes_colour);
        pros::screen::draw_rect(this->display_region.x, this->display_region.y, this->display_region.x + this->display_region.w, this->display_region.y + this->display_region.h);
    }
};