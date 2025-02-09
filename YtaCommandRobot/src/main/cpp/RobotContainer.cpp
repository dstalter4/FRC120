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
    // Initialize all of your commands and subsystems here
    //m_TeleopSubsystem.SetDefaultCommand(frc2::cmd::Run(
    //    [this] {
    //      m_TeleopSubsystem.TeleopRoutine(
    //          );
    //    },
    //    {&m_TeleopSubsystem}));

    // Cannot bind an rvalue reference to a pointer (or lvalue)
    m_TeleopSubsystem.SetDefaultCommand(TeleopHelperCommand(&m_TeleopSubsystem));

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
