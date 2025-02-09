////////////////////////////////////////////////////////////////////////////////
/// @file   RobotContainer.cpp
/// @author David Stalter
///
/// @details
/// Robot container implementation.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// WPILIB INCLUDES
#include <frc2/command/Commands.h>

// C++ INCLUDES
#include "RobotContainer.hpp"
#include "commands/AutonomousCommands.hpp"
#include "commands/TeleopCommands.hpp"


////////////////////////////////////////////////////////////////
/// @method RobotContainer::RobotContainer
///
/// Constructor for a robot container object.
///
////////////////////////////////////////////////////////////////
RobotContainer::RobotContainer() :
    m_AutonomousSubsystem(),
    m_TeleopSubsystem()
{
    // Options:
    //
    // 1. Directly set the default command by constructing one with a lambda.
    // 2. Create a command object for a custom command.
    // 3. Register the subsystem and let its periodic function run.

    // Option 1
    m_TeleopSubsystem.SetDefaultCommand(frc2::cmd::Run(
        // Lambda
        [this]
        {
            m_TeleopSubsystem.MainRoutine();
        },

        // Requirements
        {&m_TeleopSubsystem}
    ));

    // Option 2
    // Cannot bind an rvalue reference to a pointer (or lvalue)
    //m_TeleopSubsystem.SetDefaultCommand(TeleopHelperCommand(&m_TeleopSubsystem));
    TeleopHelperCommand(&m_TeleopSubsystem).Schedule();

    // Option 3 happened when the subsystem was registered

    // Configure the button bindings
    ConfigureBindings();
}


////////////////////////////////////////////////////////////////
/// @method RobotContainer::ConfigureBindings
///
/// Function to bind commands to controller inputs.
///
////////////////////////////////////////////////////////////////
void RobotContainer::ConfigureBindings()
{
    // Configure your trigger bindings here

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //frc2::Trigger([this] {
    //  return m_subsystem.ExampleCondition();
    //}).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

    // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.
    //m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}


////////////////////////////////////////////////////////////////
/// @method RobotContainer::GetAutonomousCommand
///
/// Retrieves the command to use during autonomous.
///
////////////////////////////////////////////////////////////////
frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    // An example command will be run in autonomous
    return Yta::Autonomous::ExampleCommand(&m_AutonomousSubsystem);
}


////////////////////////////////////////////////////////////////
/// @method RobotContainer::GetTeleopCommand
///
/// Retrieves the command to use during teleop.
///
////////////////////////////////////////////////////////////////
frc2::CommandPtr RobotContainer::GetTeleopCommand()
{
    // An example command will be run in teleop
    return Yta::Teleop::StaticFactoryCommand(&m_TeleopSubsystem);
}
