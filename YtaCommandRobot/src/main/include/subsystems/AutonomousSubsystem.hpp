////////////////////////////////////////////////////////////////////////////////
/// @file   AutonomousSubsystem.hpp
/// @author David Stalter
///
/// @details
/// Autonomous subsystem for a command based robot.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#pragma once

// WPILIB INCLUDES
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

// C++ INCLUDES
// (none)


////////////////////////////////////////////////////////////////
/// @class AutonomousSubsystem
///
/// Declarations for the teleop subsystem.
///
////////////////////////////////////////////////////////////////
class AutonomousSubsystem : public frc2::SubsystemBase
{
public:
    AutonomousSubsystem();

    // Lifecycle function
    // Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    // Lifecycle function
    // Will be called periodically whenever the CommandScheduler runs during simulation.
    void SimulationPeriodic() override;



    // Example static factory for a autonomous command.
    frc2::CommandPtr StaticFactoryCommand();

    // An example method querying a boolean state of the subsystem (for example, a digital sensor).
    bool ExampleConditionFunction();

private:

};
