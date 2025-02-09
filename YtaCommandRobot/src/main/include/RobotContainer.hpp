////////////////////////////////////////////////////////////////////////////////
/// @file   RobotContainer.hpp
/// @author David Stalter
///
/// @details
/// Robot container declarations.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#pragma once

// WPILIB INCLUDES
#include <frc2/command/CommandPtr.h>

// C++ INCLUDES
#include "subsystems/AutonomousSubsystem.hpp"
#include "subsystems/TeleopSubsystem.hpp"
#include "commands/TeleopCommands.hpp"


////////////////////////////////////////////////////////////////
/// @class RobotContainer
///
/// This class is where the bulk of the robot should be
/// declared.  Since Command-based is a "declarative" paradigm,
/// very little robot logic should actually be handled in the
/// {@link Robot} periodic methods (other than the scheduler
/// calls).  Instead, the structure of the robot (including
/// subsystems, commands, and trigger mappings) should be
/// declared here.
///
////////////////////////////////////////////////////////////////
class RobotContainer
{
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();
    frc2::CommandPtr GetTeleopCommand();

private:
    // The robot's subsystems are defined here.
    // Subsystems automatically get Periodic() invoked by the command scheduler.
    AutonomousSubsystem m_AutonomousSubsystem;
    TeleopSubsystem m_TeleopSubsystem;

    // In the example project, but currently unimplemented
    void ConfigureBindings();
};
