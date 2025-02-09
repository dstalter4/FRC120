// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/AutonomousSubsystem.hpp"


AutonomousSubsystem::AutonomousSubsystem()
{
}

void AutonomousSubsystem::Periodic()
{
    // This will be invoked by the command scheduler if the subsystem is present in the robot program
}

void AutonomousSubsystem::SimulationPeriodic()
{
}

frc2::CommandPtr AutonomousSubsystem::StaticFactoryCommand()
{
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    //  return RunOnce([/* this */] { /* one-time action goes here */ });
    auto emptyLambda = [this]{};
    return RunOnce(emptyLambda);
}

bool AutonomousSubsystem::ExampleConditionFunction()
{
    // Query some boolean state, such as a digital sensor.
    return false;
}
