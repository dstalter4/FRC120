// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>      // for interacting with the smart dashboard

#include "subsystems/TeleopSubsystem.hpp"


TeleopSubsystem::TeleopSubsystem()
{
}

void TeleopSubsystem::Periodic()
{
    // This will be invoked by the command scheduler if the subsystem is present in the robot program
    
    //static unsigned z;
    //std::printf("Loop iteration (z): %d\n", z);
    //frc::SmartDashboard::PutNumber("Heartbeat", z++);
}

void TeleopSubsystem::SimulationPeriodic()
{
}

frc2::CommandPtr TeleopSubsystem::StaticFactoryCommand()
{
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    //  return RunOnce([/* this */] { /* one-time action goes here */ });
    auto emptyLambda = [this]{};
    return RunOnce(emptyLambda);
}

bool TeleopSubsystem::ExampleConditionFunction()
{
    // Query some boolean state, such as a digital sensor.
    return false;
}

void TeleopSubsystem::MainRoutine()
{
    static unsigned y;
    std::printf("Loop iteration (y): %d\n", y);
    frc::SmartDashboard::PutNumber("Heartbeat", y++);
}
