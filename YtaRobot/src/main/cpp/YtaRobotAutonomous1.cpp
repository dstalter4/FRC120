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
/// Autonomous routine 1.  Start from the left bump or center,
/// go to the depot, pickup fuel, move toward the hub, shoot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine1()
{
    /*
    struct Auto1Controls
    {
        RobotTranslation m_Translate1;
        RobotRotation m_Rotation1;
    };
    constexpr const Auto1Controls AUTO1_CONTROLS[] =
    {
        {RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotRotation::ROBOT_CLOCKWISE},          // Hub Left
        {RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotRotation::ROBOT_COUNTER_CLOCKWISE}   // Hub right
    };
    */

    // The robot faces the driver station, so it is off by 180 degrees
    m_pPigeon->SetYaw(units::angle::degree_t(ANGLE_180_DEGREES));

    std::string selectedAutoPositionString = m_AutonomousPositionChooser.GetSelected();
    if (selectedAutoPositionString == "Left bump")
    {
        // Backing up towards the depot from the left bump requires just translation
        m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
        AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.25, 0.0, 0.00, 2.0_s, true);
    }
    else if (selectedAutoPositionString == "Hub")
    {
        // Backing up towards the depot from the hub requires translation and strafe
        m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_STRAFE_LEFT, RobotRotation::ROBOT_NO_ROTATION);
        AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.25, 0.20, 0.0, 2.0_s, true);
    }
    else
    {
        // Other configurations not supported
        return;
    }

    // Intake down, motor on
    m_pIntakeAngleMotor->SetPositionVoltage(INTAKE_DOWN_ANGLE_DEGREES.value());
    m_pIntakeRollersMotor->SetDutyCycle(-INTAKE_ROLLERS_MOTOR_SPEED);

    // Slowly finish backing up to collect the fuel
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);

    AutonomousDelay(1.0_s);

    // Intake up, motor off
    m_pIntakeAngleMotor->SetPositionVoltage(INTAKE_UP_ANGLE_DEGREES.value());
    m_pIntakeRollersMotor->SetDutyCycle(0.0);

    // Ramp up the shooter while we move back toward the hub to shoot
    m_pShooterMotors->Set(SHOOTER_MOTOR_SPEED);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_STRAFE_RIGHT, RobotRotation::ROBOT_COUNTER_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.25, 0.10, 0.30, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.25, 0.0, 0.0, 1.0_s, false);

    // Injector and feeder on
    m_pInjectorMotor->SetDutyCycle(-INJECTOR_MOTOR_SPEED);
    m_pFeederMotor->SetDutyCycle(-FEEDER_MOTOR_SPEED);
    AutonomousDelay(7.0_s);

    // Everybody off
    m_pShooterMotors->Set(0.0);
    m_pInjectorMotor->SetDutyCycle(0.0);
    m_pFeederMotor->SetDutyCycle(0.0);
    m_pIntakeRollersMotor->SetDutyCycle(0.0);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
