# FRC Team 120

This repository contains the C++ robot source code for FRC Team 120 from the 2021 season onward.  It contains the sources for both the roboRIO program and a peripheral RIOduino board.

## Key Features

- Robot control derived from the TimedRobot class.
- An autonomous framework for configurable routines and sensor control.
- Configurable support for various driver station input controller types.
- A motor group class for managing multiple motor controllers at once.
- A camera class dedicated to handling vision related operations.
- An I2C class for handling communication with the RIOduino.

## Branching Strategy

The `main` branch contains code and features that are largely compatible with most robots.  Customization for each year's particular robot is handled by delivering content to a `yta/main/<year>` branch.  Reusable features developed during a particular season will be merged back to the mainline periodically.
