////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous3.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 3 for YtaRobot.
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
/// @method YtaRobot::AutonomousRoutine3
///
/// Starts at the hub, shoots, moves to the depot, and rotates
/// so the intake faces the fuel.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine3()
{
    // The robot does not face the driver station, so no need to set the pigeon yaw.
    static SwerveDrive::SwerveDirections swerveDirections;

    // Back up just a bit to be in range for shooting
    swerveDirections.SetSwerveDirections(SwerveDrive::RobotTranslation::ROBOT_TRANSLATION_REVERSE, SwerveDrive::RobotStrafe::ROBOT_STRAFE_NONE, SwerveDrive::RobotRotation::ROBOT_ROTATION_NONE);
    m_pSwerveDrive->AutonomousDrive(swerveDirections, 0.20, 0.20, 0.0, 0.5_s, true);

    // Ramp up the shooter
    m_pShooterMotors->SetDutyCycle(SHOOTER_MOTOR_SPEED);
    Wait(1.0_s);

    // Injector and feeder on
    m_pInjectorMotor->SetDutyCycle(-INJECTOR_MOTOR_SPEED);
    m_pFeederMotor->SetDutyCycle(-FEEDER_MOTOR_SPEED);
    Wait(5.0_s);

    // Everything off
    m_pShooterMotors->SetDutyCycle(0.0);
    m_pInjectorMotor->SetDutyCycle(0.0);
    m_pFeederMotor->SetDutyCycle(0.0);
    m_pIntakeRollersMotor->SetDutyCycle(0.0);

    // Anything after this is part of the move to depot
    std::string selectedAutoPositionString = m_AutonomousPositionChooser.GetSelected();
    if (selectedAutoPositionString == "Just shoot")
    {
        return;
    }

    // Move to the depot
    swerveDirections.SetSwerveDirections(SwerveDrive::RobotTranslation::ROBOT_TRANSLATION_REVERSE, SwerveDrive::RobotStrafe::ROBOT_STRAFE_LEFT, SwerveDrive::RobotRotation::ROBOT_ROTATION_NONE);
    m_pSwerveDrive->AutonomousDrive(swerveDirections, 0.25, 0.27, 0.0, 1.0_s, true);

    // Rotate to face the depot with the intake
    m_pSwerveDrive->AutonomousRotateByGyro(SwerveDrive::RobotRotation::ROBOT_ROTATION_CLOCKWISE, 175.0, 0.15, true);

    // Intake down, motor on
    m_pIntakeAngleMotor->SetPositionVoltage(INTAKE_DOWN_ANGLE_DEGREES);
    m_pIntakeRollersMotor->SetDutyCycle(-INTAKE_ROLLERS_MOTOR_SPEED);

    // Slowly finish backing up to collect the fuel
    swerveDirections.SetSwerveDirections(SwerveDrive::RobotTranslation::ROBOT_TRANSLATION_REVERSE, SwerveDrive::RobotStrafe::ROBOT_STRAFE_NONE, SwerveDrive::RobotRotation::ROBOT_ROTATION_NONE);
    m_pSwerveDrive->AutonomousDrive(swerveDirections, 0.10, 0.0, 0.0, 2.0_s, true);

    Wait(1.0_s);

    // Intake up, motor off
    m_pIntakeAngleMotor->SetPositionVoltage(INTAKE_UP_ANGLE_DEGREES);
    m_pIntakeRollersMotor->SetDutyCycle(0.0);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 3 done.");
}
