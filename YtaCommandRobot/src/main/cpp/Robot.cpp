////////////////////////////////////////////////////////////////////////////////
/// @file   Robot.cpp
/// @author David Stalter
///
/// @details
/// Robot class definitions.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// WPILIB INCLUDES
#include <frc2/command/CommandScheduler.h>

// C++ INCLUDES
#include "Robot.hpp"


////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////
Robot::Robot()
{
}

////////////////////////////////////////////////////////////////
/// This function is called every 20 ms, no matter the mode. Use
/// this for items like diagnostics that you want to run during disabled,
/// autonomous, teleoperated and test.
///
/// <p> This runs after the mode specific periodic functions, but before
/// LiveWindow and SmartDashboard integrated updating.
////////////////////////////////////////////////////////////////
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
}

////////////////////////////////////////////////////////////////
/// This function is called once each time the robot enters Disabled mode. You
/// can use it to reset any subsystem information you want to clear when the
/// robot is disabled.
////////////////////////////////////////////////////////////////
void Robot::DisabledInit()
{
}

////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////
void Robot::DisabledPeriodic()
{
}

////////////////////////////////////////////////////////////////
/// This autonomous runs the autonomous command selected by your {@link
/// RobotContainer} class.
////////////////////////////////////////////////////////////////
void Robot::AutonomousInit()
{
    // This makes sure that teleop stops running when autonomous
    // starts running. This is relevant for at home testing.
    if (m_TeleopCommand)
    {
        m_TeleopCommand->Cancel();
    }

    m_AutonomousCommand = m_RobotContainer.GetAutonomousCommand();
    if (m_AutonomousCommand)
    {
        m_AutonomousCommand->Schedule();
    }
}

////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////
void Robot::AutonomousPeriodic()
{
}

////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////
void Robot::TeleopInit()
{
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_AutonomousCommand)
    {
        m_AutonomousCommand->Cancel();
    }

    m_TeleopCommand = m_RobotContainer.GetTeleopCommand();
    if (m_TeleopCommand)
    {
        //m_TeleopCommand->Schedule();
    }
}

////////////////////////////////////////////////////////////////
/// This function is called periodically during operator control.
////////////////////////////////////////////////////////////////
#include "commands/TeleopCommands.hpp"
#include "frc/smartdashboard/SmartDashboard.h"
void Robot::TeleopPeriodic()
{
    static frc2::CommandPtr teleopCmd = TeleopHelperCommand(m_RobotContainer.GetTeleopSubsystem()).ToPtr();
    static uint32_t iteration = 0UL;
    if ((iteration % 4) == 0U)
    {
        // This probably can't be cancelled if it's continuously scheduled
        //m_RobotContainer.ScheduleSingleTeleopCommand();

        //m_TeleopCommand->Cancel();
        teleopCmd.Schedule();
    }
    else if ((iteration % 5) == 0U)
    {
        //teleopCmd.Cancel();
        m_TeleopCommand->Schedule();
    }
    else
    {
        m_TeleopCommand->Cancel();
        teleopCmd.Cancel();
    }
    iteration++;
    frc::SmartDashboard::PutNumber("Iteration", iteration);
}

////////////////////////////////////////////////////////////////
/// This function is called periodically during test mode.
////////////////////////////////////////////////////////////////
void Robot::TestPeriodic()
{
}

////////////////////////////////////////////////////////////////
/// This function is called once when the robot is first started up.
////////////////////////////////////////////////////////////////
void Robot::SimulationInit()
{
}

////////////////////////////////////////////////////////////////
/// This function is called periodically whilst in simulation.
////////////////////////////////////////////////////////////////
void Robot::SimulationPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
