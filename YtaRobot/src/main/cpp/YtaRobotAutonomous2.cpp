////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous2.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 2 for YtaRobot.
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
/// @method YtaRobot::AutonomousRoutine2
///
/// Starts from the hub, moves to the depot, picks up fuel,
/// returns towards the hub, and shoots.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine2()
{
    // The robot faces the driver station, so it is off by 180 degrees
    m_pPigeon->SetYaw(units::angle::degree_t(ANGLE_180_DEGREES));

    // Intake down, motor on
    m_pIntakeAngleMotor->SetPositionVoltage(INTAKE_DOWN_ANGLE_DEGREES.value());
    m_pIntakeRollersMotor->SetDutyCycle(-INTAKE_ROLLERS_MOTOR_SPEED);

    // Drive towards the depot
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_STRAFE_LEFT, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.25, 0.20, 0.0, 2.0_s, true);

    // Slowly finish backing up to collect the fuel
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 2.0_s, true);

    AutonomousDelay(1.0_s);

    // Intake up, motor off
    m_pIntakeAngleMotor->SetPositionVoltage(INTAKE_UP_ANGLE_DEGREES.value());
    m_pIntakeRollersMotor->SetDutyCycle(0.0);

    // Ramp up the shooter while we move back toward the hub to shoot
    m_pShooterMotors->Set(SHOOTER_MOTOR_SPEED);

    // On a real field, moving back from the depot needs higher rotate
    // to compensate for driving over the depot perimeter bump.
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_STRAFE_RIGHT, RobotRotation::ROBOT_COUNTER_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.25, 0.10, 0.35, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.30, 0.0, 0.0, 1.0_s, false);

    // Injector and feeder on
    m_pInjectorMotor->SetDutyCycle(-INJECTOR_MOTOR_SPEED);
    m_pFeederMotor->SetDutyCycle(-FEEDER_MOTOR_SPEED);
    AutonomousDelay(7.0_s);

    // Everything off
    m_pShooterMotors->Set(0.0);
    m_pInjectorMotor->SetDutyCycle(0.0);
    m_pFeederMotor->SetDutyCycle(0.0);
    m_pIntakeRollersMotor->SetDutyCycle(0.0);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 2 done.");
}
