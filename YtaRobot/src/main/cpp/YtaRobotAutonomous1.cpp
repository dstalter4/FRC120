////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous1.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 1 for YtaRobot.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotUtils.hpp"               // for DisplayMessage()
#include "YtaRobot.hpp"                 // for robot class declaration
#include "YtaRobotAutonomous.hpp"       // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousRoutine1
///
/// Autonomous routine 1.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine1()
{
    // Parameters:
    // 1. Direction (ROBOT_FORWARD)
    // 2. Forward speed for both motors 0.0 -> 1.0
    // 3. Time in seconds
    // 4-6. Only present if doing an uneven distribution of power (turn)
    // 4. Always 'true'
    // 5. How much faster the left side is (0.0 -> 1.0)
    // 6. How much faster the right side is (0.0 -> 1.0)
    
    // First 'S' turn
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 1.30, true, 0.0, (0.15 * RIGHT_DRIVE_FORWARD_SCALAR));
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 0.90, true, (0.15 * LEFT_DRIVE_FORWARD_SCALAR), 0.0);

    // Straight away (Normal/low batter: 1.1, Fresh battery: 1.0)
    AutonomousDriveSequence(ROBOT_FORWARD, 0.4, 1.25);

    // Second 'S' turn into loop
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 0.70, true, (0.20 * LEFT_DRIVE_FORWARD_SCALAR), 0.0);
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 0.60, true, 0.0, (0.20 * RIGHT_DRIVE_FORWARD_SCALAR));

    // Loop
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 2.2, true, 0.0, (0.20 * RIGHT_DRIVE_FORWARD_SCALAR));
    return;

    // First 'S' turn back
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 0.7, true, 0.0, (0.15 * RIGHT_DRIVE_FORWARD_SCALAR));
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 0.9, true, (0.15 * LEFT_DRIVE_FORWARD_SCALAR), 0.0);

    // Straight away
    AutonomousDriveSequence(ROBOT_FORWARD, 0.4, 1.1);

    // Last 'S' turn
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 0.7, true, (0.20 * LEFT_DRIVE_FORWARD_SCALAR), 0.0);
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 1.2, true, 0.0, (0.15 * RIGHT_DRIVE_FORWARD_SCALAR));

    // Park
    AutonomousDriveSequence(ROBOT_FORWARD, 0.15, 1.25);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}

/*
void Routine1NoBumpers()
{
    // First 'S' turn
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 1.35, true, 0.0, (0.15 * RIGHT_DRIVE_FORWARD_SCALAR));
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 0.90, true, (0.15 * LEFT_DRIVE_FORWARD_SCALAR), 0.0);

    // Straight away (Normal/low batter: 1.1, Fresh battery: 1.0)
    AutonomousDriveSequence(ROBOT_FORWARD, 0.4, 1.1);

    // Second 'S' turn into loop
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 0.65, true, (0.15 * LEFT_DRIVE_FORWARD_SCALAR), 0.0);
    AutonomousDriveSequence(ROBOT_FORWARD, 0.3, 0.65);

    // Loop
    AutonomousDriveSequence(ROBOT_FORWARD, 0.15, 2.2, true, 0.0, (0.2 * RIGHT_DRIVE_FORWARD_SCALAR));

    // First 'S' turn back
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 0.7, true, 0.0, (0.15 * RIGHT_DRIVE_FORWARD_SCALAR));
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 0.9, true, (0.15 * LEFT_DRIVE_FORWARD_SCALAR), 0.0);

    // Straight away
    AutonomousDriveSequence(ROBOT_FORWARD, 0.4, 1.1);

    // Last 'S' turn
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 0.7, true, (0.20 * LEFT_DRIVE_FORWARD_SCALAR), 0.0);
    AutonomousDriveSequence(ROBOT_FORWARD, 0.20, 1.2, true, 0.0, (0.15 * RIGHT_DRIVE_FORWARD_SCALAR));

    // Park
    AutonomousDriveSequence(ROBOT_FORWARD, 0.15, 1.25);
}
*/
