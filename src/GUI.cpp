#include "GUI.hpp"

#include <cmath>

namespace gui
{
    Graph::Graph(lv_obj_t *parent)
    {
        this->axes_colour = lv_color_hex(0x00707070);
        this->point_colour = lv_color_hex(0x00ffffff);
        this->bg_colour = lv_color_hex(0x00000000);
        this->canvas = lv_canvas_create(parent, nullptr);
    }

    Graph::~Graph()
    {
        lv_obj_del(this->canvas);
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

        lv_point_t points[4] = {{0, 0},
                                {this->display_region.w, 0},
                                {this->display_region.w, this->display_region.h},
                                {0, this->display_region.h}};

        this->corner_points.assign(points, points + 4);

        lv_obj_set_x(this->canvas, this->display_region.x);
        lv_obj_set_y(this->canvas, this->display_region.y);
        lv_obj_set_width(this->canvas, this->display_region.w);
        lv_obj_set_height(this->canvas, this->display_region.h);
    }

    void Graph::plot(std::vector<Point<double>> &points)
    {
        for (auto point : points)
        {
            int pixel_x = std::floor((point.x - win_x) * (this->display_region.w / win_w));
            int pixel_y = std::floor((win_y - point.y) * (this->display_region.h / win_h));

            lv_canvas_draw_circle(this->canvas, pixel_x, pixel_y, this->point_width / 2.0, this->point_colour);
        }
    }

    void Graph::draw()
    {
        this->clear();

        short x_axis_pixel_y = std::floor((win_y) * (this->display_region.h / win_h));
        short y_axis_pixel_x = std::floor(-(win_x) * (this->display_region.w / win_w));
        lv_canvas_draw_line(this->canvas, {0, x_axis_pixel_y}, {this->display_region.w, x_axis_pixel_y}, this->axes_colour);
        lv_canvas_draw_line(this->canvas, {0, y_axis_pixel_x}, {this->display_region.h, y_axis_pixel_x}, this->axes_colour);

        lv_canvas_draw_rect(this->canvas, this->corner_points.data(), axes_colour);
    }

    void Graph::clear()
    {
        lv_canvas_fill_polygon(this->canvas, this->corner_points.data(), 4, this->axes_colour, this->bg_colour);
    }
};