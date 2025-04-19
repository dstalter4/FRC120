////////////////////////////////////////////////////////////////////////////////
/// @file   TeleopSubsystem.cpp
/// @author David Stalter
///
/// @details
/// Teleop subsystem for a command based robot.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// WPILIB INCLUDES
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

// C++ INCLUDES
#include "subsystems/TeleopSubsystem.hpp"


////////////////////////////////////////////////////////////////
/// @method TeleopSubsystem::TeleopSubsystem
///
/// Constructor for the teleop subsystem.
///
////////////////////////////////////////////////////////////////
TeleopSubsystem::TeleopSubsystem()
{
}


////////////////////////////////////////////////////////////////
/// @method TeleopSubsystem::Periodic
///
/// Periodic function for the teleop subsystem.  This will be
/// invoked by the command scheduler if the subsystem is
/// present in the robot program.
///
////////////////////////////////////////////////////////////////
void TeleopSubsystem::Periodic()
{
    static unsigned t;
    frc::SmartDashboard::PutNumber("Teleop subsystem periodic", t++);
}


////////////////////////////////////////////////////////////////
/// @method TeleopSubsystem::SimulationPeriodic
///
/// Simulation periodic function for the teleop subsystem.
///
////////////////////////////////////////////////////////////////
void TeleopSubsystem::SimulationPeriodic()
{
}


////////////////////////////////////////////////////////////////
/// @method TeleopSubsystem::StaticFactoryCommand
///
/// An example command factory function.  It creates a command
/// that runs once from a passed in lambda.
///
////////////////////////////////////////////////////////////////
frc2::CommandPtr TeleopSubsystem::StaticFactoryCommand()
{
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    //  return RunOnce([/* this */] { /* one-time action goes here */ });
    auto emptyLambda = [this]{};
    return RunOnce(emptyLambda);
}


////////////////////////////////////////////////////////////////
/// @method TeleopSubsystem::ExampleConditionFunction
///
/// Example condition function that would query a Boolean state
/// such as a digital sensor.
///
////////////////////////////////////////////////////////////////
bool TeleopSubsystem::ExampleConditionFunction()
{
    return false;
}


////////////////////////////////////////////////////////////////
/// @method TeleopSubsystem::MainRoutine
///
/// The main routine for the teleop subsystem.  This is the glue
/// function that the subsystem will invoke to use a traditional
/// time based implementation instead of commands.
///
////////////////////////////////////////////////////////////////
void TeleopSubsystem::MainRoutine()
{
    static unsigned m;
    frc::SmartDashboard::PutNumber("Teleop subsystem main", m++);
}
