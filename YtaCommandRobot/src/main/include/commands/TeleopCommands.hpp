////////////////////////////////////////////////////////////////////////////////
/// @file   TeleopCommands.hpp
/// @author David Stalter
///
/// @details
/// Factory and custom teleop commands.
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
class TeleopSubsystem;


////////////////////////////////////////////////////////////////
/// @namespace Yta::Teleop
///
////////////////////////////////////////////////////////////////
namespace Yta::Teleop
{
    // Example static factory for a teleop command
    frc2::CommandPtr StaticFactoryCommand(TeleopSubsystem * pTeleopSubsystem);
}


////////////////////////////////////////////////////////////////
/// @class TeleopHelperCommand
///
/// Declarations for a custom teleop command.
///
////////////////////////////////////////////////////////////////
class TeleopHelperCommand : public frc2::CommandHelper<frc2::Command, TeleopHelperCommand>
{
public:
    // Creates a new TeleopHelperCommand.
    explicit TeleopHelperCommand(TeleopSubsystem * pTeleopSubsystem);

    // Primary execution function for the command
    virtual void Execute();

private:
    TeleopSubsystem * m_pTeleopSubsystem;
};
