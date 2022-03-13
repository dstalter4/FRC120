////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomousTest.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 3 for YtaRobot.
///
/// Copyright (c) 2022 Youth Technology Academy
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
/// @method YtaRobot::AutonomousTestRoutine
///
/// Autonomous test routine.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousTestRoutine()
{
    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 3 done.");
}
