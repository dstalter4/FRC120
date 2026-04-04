////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous.hpp
/// @author David Stalter
///
/// @details
/// Contains the declarations for the autonomous portions of code ran in an FRC
/// robot.
///
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTAROBOTAUTONOMOUS_HPP
#define YTAROBOTAUTONOMOUS_HPP

// SYSTEM INCLUDES
#include <frc2/command/CommandPtr.h>    // for CommandPtr type

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                 // for inline autonomous function declarations

using namespace frc;

////////////////////////////////////////////////////////////////
/// @namespace YtaRobotAutonomous
///
/// Namespace that contains robot autonomous variable and
/// function declarations.
///
////////////////////////////////////////////////////////////////
namespace YtaRobotAutonomous
{
    // TYPEDEFS
    // (none)
    
    // ENUMS
    // (none)    
    
    // STRUCTS
    // (none)
    
    // VARIABLES
    extern bool bAutonomousExecutionComplete;
    extern std::optional<CommandPtr> AutonomousCommand;
    
    // CONSTS
    
    // Autonomous Mode Constants
    // @todo: Convert to class and make a friend in YtaRobot
    
    // Note: Only enable one autonomous routine!
    // Note: Autonomous routines are currently controlled by
    // the SendableChooser.
    //static const bool       ROUTINE_1                           = true;
    //static const bool       ROUTINE_2                           = false;
    //static const bool       ROUTINE_3                           = false;
    //static const bool       TEST_ENABLED                        = false;
    static const bool       USE_COMMAND_BASED_AUTONOMOUS        = false;
} // End namespace

#endif // YTAROBOTAUTONOMOUS_HPP
