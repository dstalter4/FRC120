////////////////////////////////////////////////////////////////////////////////
/// @file   AutonomousCommands.hpp
/// @author David Stalter
///
/// @details
/// Factory and custom autonomous commands.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#pragma once

// WPILIB INCLUDES
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>

// C++ INCLUDES
// (none)

// FORWARD DECLARATIONS
class AutonomousSubsystem;


////////////////////////////////////////////////////////////////
/// @namespace Yta::Autonomous
///
////////////////////////////////////////////////////////////////
namespace Yta::Autonomous
{
    frc2::CommandPtr ExampleCommand(AutonomousSubsystem * pAutonomousSubsystem);
}


////////////////////////////////////////////////////////////////
/// @class AutonomousHelperCommand
///
/// Declarations for a custom autonomous command.
///
////////////////////////////////////////////////////////////////
class AutonomousHelperCommand : public frc2::CommandHelper<frc2::Command, AutonomousHelperCommand>
{
public:
    // Creates a new AutonomousHelperCommand.
    explicit AutonomousHelperCommand(AutonomousSubsystem * pAutonomousSubsystem);

    // Primary execution function for the command
    virtual void Execute();

private:
    AutonomousSubsystem * m_pAutonomousSubsystem;
};
