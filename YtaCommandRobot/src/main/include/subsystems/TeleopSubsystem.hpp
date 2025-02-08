// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class TeleopSubsystem : public frc2::SubsystemBase
{
public:
    TeleopSubsystem();

    // Lifecycle function
    // Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    // Lifecycle function
    // Will be called periodically whenever the CommandScheduler runs during simulation.
    void SimulationPeriodic() override;



    // Example static factory for a teleop command
    // This is just like what is in TeleopCommands.cpp, just scoped to a subsystem class
    frc2::CommandPtr StaticFactoryCommand();

    // An example method querying a boolean state of the subsystem (for example, a digital sensor).
    //
    // @return value of some boolean subsystem state, such as a digital sensor.
    bool ExampleConditionFunction();

private:
    // Function to handle the primary responsibilities of teleop.
    // This is the glue function to bind a TimedRobot into a CommandRobot
    void MainRoutine();
};
