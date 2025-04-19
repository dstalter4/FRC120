////////////////////////////////////////////////////////////////////////////////
/// @file   AutonomousSubsystem.cpp
/// @author David Stalter
///
/// @details
/// Autonomous subsystem for a command based robot.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// WPILIB INCLUDES
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

// C++ INCLUDES
#include "subsystems/AutonomousSubsystem.hpp"


////////////////////////////////////////////////////////////////
/// @method AutonomousSubsystem::AutonomousSubsystem
///
/// Constructor for the autonomous subsystem.
///
////////////////////////////////////////////////////////////////
AutonomousSubsystem::AutonomousSubsystem()
{
}


////////////////////////////////////////////////////////////////
/// @method AutonomousSubsystem::Periodic
///
/// Periodic function for the autonomous subsystem.  This will
/// be invoked by the command scheduler if the subsystem is
/// present in the robot program.
///
////////////////////////////////////////////////////////////////
void AutonomousSubsystem::Periodic()
{
}


////////////////////////////////////////////////////////////////
/// @method AutonomousSubsystem::SimulationPeriodic
///
/// Simulation periodic function for the autonomous subsystem.
///
////////////////////////////////////////////////////////////////
void AutonomousSubsystem::SimulationPeriodic()
{
}


////////////////////////////////////////////////////////////////
/// @method AutonomousSubsystem::StaticFactoryCommand
///
/// An example command factory function.  It creates a command
/// that runs once from a passed in lambda.
///
////////////////////////////////////////////////////////////////
frc2::CommandPtr AutonomousSubsystem::StaticFactoryCommand()
{
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    //  return RunOnce([/* this */] { /* one-time action goes here */ });
    auto emptyLambda = [this]{};
    return RunOnce(emptyLambda);
}


////////////////////////////////////////////////////////////////
/// @method AutonomousSubsystem::ExampleConditionFunction
///
/// Example condition function that would query a Boolean state
/// such as a digital sensor.
///
////////////////////////////////////////////////////////////////
bool AutonomousSubsystem::ExampleConditionFunction()
{
    return false;
}
