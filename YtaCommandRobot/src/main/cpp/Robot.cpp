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
#include <frc2/command/Commands.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

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
        m_TeleopCommand->Schedule();
    }
}

////////////////////////////////////////////////////////////////
/// This function is called periodically during operator control.
////////////////////////////////////////////////////////////////
void Robot::TeleopPeriodic()
{
    // Demonstration of some command scheduling

    static uint32_t instantCount = 0UL;
    static frc2::CommandPtr instantCommand = frc2::InstantCommand([this](){ frc::SmartDashboard::PutNumber("Instant command", instantCount++); }).ToPtr();
    static bool bTeleopCommandScheduled = true;
    static bool bCommandsPaused = false;

    // Scheduler runs every 20ms, so there are 50 iterations per second
    static uint32_t iteration = 0UL;

    // Every five seconds, cancel all commands for a bit
    if ((iteration % 250) == 0U)
    {
        bCommandsPaused = !bCommandsPaused;
        if (bCommandsPaused)
        {
            m_TeleopCommand->Cancel();
            instantCommand.Cancel();
        }
    }

    if (!bCommandsPaused)
    {
        if ((iteration % 50) == 0U)
        {
            // Once a second, switch the teleop command on/off
            if (bTeleopCommandScheduled)
            {
                m_TeleopCommand->Cancel();
            }
            else
            {
                m_TeleopCommand->Schedule();
            }
            bTeleopCommandScheduled = !bTeleopCommandScheduled;
        }

        // Every three seconds, schedule the instant command
        if ((iteration % 150) == 0U)
        {
            instantCommand.Schedule();
        }
    }

    iteration++;
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
