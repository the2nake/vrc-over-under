# 3818C Over Under Codebase

## Description

This contains code for the VRC 2023-24 Over Under. The code will be updated over time to match the capabilities of the physical robot, with branches made to archive old code versions when the robot undergoes significant changes.

## Features

- A set of various utility functions
- Differential drive control schemes
  - voltage based
  - configurable feedforward + feedback to target wheel velocities
- Star (*) drive control schemes
  - voltage based
    - local or global reference frames
  - Async PID controller for position and heading in global reference frame
- `CustomImu` class to improve IMU sensor accuracy
- Asynchronous position tracking with 2 tracking wheels and IMU
- Touchscreen configurable autonomous selector
- Onscreen visualisation tools

## Upcoming

- Differential drive trajectory generation with quintic polynomials
- Asynchronous trajectory control with RAMSETE controller

## License

GNU GPLv3

## Credits

Charts made using Lucidchart
Jonathan Bayless for `baylessj/robotsquiggles`, which the trajectory generation code is based upon
BLRS Wiki for Ramsete controller algorithm description
