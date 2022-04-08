////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous1.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 1 for YtaRobot.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotUtils.hpp"               // for DisplayMessage()
#include "YtaRobot.hpp"                 // for robot class declaration
#include "YtaRobotAutonomous.hpp"       // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousRoutine1
///
/// Autonomous routine 1.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine1()
{
    // Drop the intake, spin the motors
    m_pIntakeSolenoid->Set(INTAKE_DOWN_SOLENOID_VALUE);
    m_pIntakeMotors->GetMotorObject(INTAKE_MOTORS_CAN_START_ID + 1)->Set(ControlMode::PercentOutput, -INTAKE_MOTOR_SPEED);
    Wait(0.5_s);

    // Start driving, this should pick up a cargo
    AutonomousDriveSequence(YtaRobot::RobotDirection::ROBOT_FORWARD, 0.30, 1.0_s);
    Wait(0.5_s);

    // Get closer for an accurate shot
    AutonomousDriveSequence(YtaRobot::RobotDirection::ROBOT_REVERSE, 0.30, 0.75_s);

    // Give some time to let other robots/human player cargo clear
    Wait(1.5_s);

    // Start spinning up the shooter motors
    m_pShooterMotors->Set(0.75);
    Wait(1.0_s);

    // Start shooting
    m_pFeederMotors->Set(FEEDER_MOTOR_SPEED);

    // Clear the first cargo
    Wait(1.0_s);

    // Now start the low intake motor (to try and space the shots)
    m_pIntakeMotors->Set(INTAKE_MOTOR_SPEED);

    // Bring the intake up in case the cargo was already grabbed
    m_pIntakeSolenoid->Set(INTAKE_UP_SOLENOID_VALUE);
    Wait(3.0_s);

    // Done, everything off
    m_pIntakeMotors->Set(OFF);
    m_pFeederMotors->Set(OFF);
    m_pShooterMotors->Set(OFF);

    // Back out of the tarmac
    AutonomousDriveSequence(YtaRobot::RobotDirection::ROBOT_FORWARD, 0.30, 1.0_s);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
