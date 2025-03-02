////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomousCommand.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routines for YtaRobot.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include "frc/smartdashboard/SmartDashboard.h"  // for interacting with the smart dashboard

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotUtils.hpp"
#include "YtaRobotAutonomousCommand.hpp"       // for autonomous command declarations


////////////////////////////////////////////////////////////////
/// @method AutonomousSubsystem::AutonomousSubsystem
///
/// Constructor for the autonomous subsystem.
///
////////////////////////////////////////////////////////////////
void AutonomousSubsystem::KeyMethod()
{
}


////////////////////////////////////////////////////////////////
/// @method AutonomousHelperCommand::AutonomousHelperCommand
///
/// Constructor for a custom autonomous command.
///
////////////////////////////////////////////////////////////////
AutonomousHelperCommand::AutonomousHelperCommand(AutonomousSubsystem * pAutonomousSubsystem) :
    m_pAutonomousSubsystem{pAutonomousSubsystem}
{
    RobotUtils::DisplayMessage("Autonomous helper command constructor.");
    // Register that this command requires the subsystem.
    AddRequirements(m_pAutonomousSubsystem);
}


////////////////////////////////////////////////////////////////
/// @method AutonomousHelperCommand::Execute
///
/// Primary execution function for the custom autonomous
/// command.
///
////////////////////////////////////////////////////////////////
void AutonomousHelperCommand::Execute()
{
    static unsigned a = 0U;
    frc::SmartDashboard::PutNumber("Autonomous helper execute", a++);
}
