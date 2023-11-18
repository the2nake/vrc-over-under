/* home.vn2007@gmail.com - 2023 */

#pragma once

#include "main.h"

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
        Graph();

        // bottom-left, w is to the right, h is upwards
        void set_window(double x, double y, double w, double h);
        void set_display_region(ScreenArea area);

        void draw();
        void plot(std::vector<Point<double>> &points);
        void clear();

        uint32_t axes_colour;
        uint32_t point_colour;
        uint32_t bg_colour;
        int point_width = 1;

    private:
        ScreenArea display_region{0, 0, 10, 10};
        double win_x = 0.0, win_y = 0.0;   // bottom-left point of the graphed area
        double win_w = 10.0, win_h = 10.0; // the height and width of the graphed area
    };
};