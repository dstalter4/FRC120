////////////////////////////////////////////////////////////////////////////////
/// @file   AutonomousCommands.hpp
/// @author David Stalter
///
/// @details
/// Factory and custom autonomous commands.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// WPILIB INCLUDES
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

// C++ INCLUDES
#include "commands/AutonomousCommands.hpp"
#include "subsystems/AutonomousSubsystem.hpp"


////////////////////////////////////////////////////////////////
/// @method Yta::Autonomous::ExampleCommand
///
/// An example command function.  It creates a sequence of
/// commands from the list specified.  In this example, it calls
/// the subsystem static factory command followed by the custom
/// autonomous command.
///
////////////////////////////////////////////////////////////////
frc2::CommandPtr Yta::Autonomous::ExampleCommand(AutonomousSubsystem * pAutonomousSubsystem)
{
    return frc2::cmd::Sequence(pAutonomousSubsystem->StaticFactoryCommand(),
                               AutonomousHelperCommand(pAutonomousSubsystem).ToPtr());
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
