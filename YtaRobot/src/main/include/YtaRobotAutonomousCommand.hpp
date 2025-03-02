////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomousCommand.hpp
/// @author David Stalter
///
/// @details
/// Contains the declarations for the autonomous portions of code ran in an FRC
/// robot.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTAROBOTAUTONOMOUSCOMMAND_HPP
#define YTAROBOTAUTONOMOUSCOMMAND_HPP

// SYSTEM INCLUDES
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

// C INCLUDES
// (none)

// C++ INCLUDES
// (none)

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class AutonomousSubsystem
///
/// Declarations for the teleop subsystem.
///
////////////////////////////////////////////////////////////////
class AutonomousSubsystem : public frc2::SubsystemBase
{
public:
    virtual void KeyMethod();

    AutonomousSubsystem() {}

    // Lifecycle function
    // Will be called periodically whenever the CommandScheduler runs.
    //void Periodic() override;

    // Lifecycle function
    // Will be called periodically whenever the CommandScheduler runs during simulation.
    //void SimulationPeriodic() override;

private:

};


////////////////////////////////////////////////////////////////
/// @class AutonomousHelperCommand
///
/// Declarations for a custom autonomous command.
///
////////////////////////////////////////////////////////////////
class AutonomousHelperCommand : public frc2::CommandHelper<frc2::Command, AutonomousHelperCommand>
{
public:
    // Creates a new AutonomousHelperCommand.
    explicit AutonomousHelperCommand(AutonomousSubsystem * pAutonomousSubsystem);

    // Primary execution function for the command
    virtual void Execute();

private:
    AutonomousSubsystem * m_pAutonomousSubsystem;
};

#endif // YTAROBOTAUTONOMOUSCOMMAND_HPP
