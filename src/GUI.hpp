#pragma once

#include "pros/apix.h"

#include "common.hpp"

#include <vector>

namespace gui
{
    struct ScreenArea
    {
        short x = 0, y = 0;
        short w = 10, h = 10;
    };

    class Graph
    {
    public:
        Graph(lv_obj_t *parent);
        ~Graph();

        // bottom-left, w is to the right, h is upwards
        void set_window(double x, double y, double w, double h);
        void set_display_region(ScreenArea area);

        void draw();
        void plot(std::vector<Point<double>> &points);
        void clear();

        lv_color_t axes_colour;
        lv_color_t point_colour;
        lv_color_t bg_colour;
        int point_width = 1;

    private:
        lv_obj_t *canvas = nullptr;

        ScreenArea display_region{0, 0, 10, 10};
        double win_x = 0.0, win_y = 0.0;   // bottom-left point of the graphed area
        double win_w = 10.0, win_h = 10.0; // the height and width of the graphed area

        std::vector<lv_point_t> corner_points = {};
    };
};