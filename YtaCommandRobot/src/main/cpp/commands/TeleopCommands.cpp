////////////////////////////////////////////////////////////////////////////////
/// @file   TeleopCommands.cpp
/// @author David Stalter
///
/// @details
/// Factory and custom teleop commands.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// WPILIB INCLUDES
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

// C++ INCLUDES
#include "commands/TeleopCommands.hpp"
#include "subsystems/TeleopSubsystem.hpp"


////////////////////////////////////////////////////////////////
/// @method GlobalRobotHook
///
/// A global function hook used just for demonstration/testing
/// purposes.
///
////////////////////////////////////////////////////////////////
unsigned g = 0U;
void GlobalRobotHook()
{
    frc::SmartDashboard::PutNumber("Global robot hook", g++);
}


////////////////////////////////////////////////////////////////
/// @method Yta::Teleop::StaticFactoryCommand
///
/// An example command factory function.  It creates a command
/// that continuously executes from the passed in function (or
/// lambda).
///
////////////////////////////////////////////////////////////////
frc2::CommandPtr Yta::Teleop::StaticFactoryCommand(TeleopSubsystem * pTeleopSubsystem)
{
    std::function<void()> funcPtr = GlobalRobotHook;
    return frc2::cmd::Run(funcPtr);
}


////////////////////////////////////////////////////////////////
/// @method TeleopHelperCommand::TeleopHelperCommand
///
/// Constructor for a custom teleop command.
///
////////////////////////////////////////////////////////////////
TeleopHelperCommand::TeleopHelperCommand(TeleopSubsystem * pTeleopSubsystem) :
    m_pTeleopSubsystem{pTeleopSubsystem}
{
    // Register that this command requires the subsystem.
    AddRequirements(m_pTeleopSubsystem);
}


////////////////////////////////////////////////////////////////
/// @method TeleopHelperCommand::Execute
///
/// Primary execution function for the custom teleop command.
///
////////////////////////////////////////////////////////////////
void TeleopHelperCommand::Execute()
{
    static unsigned e = 0U;
    frc::SmartDashboard::PutNumber("Teleop helper execute", e++);
}
