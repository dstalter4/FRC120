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
/// Autonomous routine 2.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine2()
{
    // The robot faces the driver station, so it is off by 180 degrees
    m_pPigeon->SetYaw(units::angle::degree_t(ANGLE_180_DEGREES));
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.15, 0.0, 0.00, 2.0_s, true);

    // Ramp up the shooter
    m_pShooterMotors->Set(-0.65);
    AutonomousDelay(0.5_s);

    // Injector on
    m_pInjectorMotor->SetDutyCycle(-INJECTOR_MOTOR_SPEED);
 
    // Feeder oscillating loop
    Timer feederTimer;   
    for (uint32_t i = 0U; i < 10U; i++)
    {
        m_pFeederMotor->SetDutyCycle(-FEEDER_MOTOR_SPEED);
        AutonomousDelay(0.35_s);
        m_pFeederMotor->SetDutyCycle(0.0);
        AutonomousDelay(0.15_s);
        m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
        AutonomousDelay(0.35_s);
        m_pFeederMotor->SetDutyCycle(0.0);
        AutonomousDelay(0.15_s);
        if (i == 2U)
        {
            m_pIntakeRollersMotor->SetDutyCycle(-INTAKE_ROLLERS_MOTOR_SPEED);
        }
    }

    m_pShooterMotors->Set(0.0);
    m_pInjectorMotor->SetDutyCycle(0.0);
    m_pFeederMotor->SetDutyCycle(0.0);
    m_pIntakeRollersMotor->SetDutyCycle(0.0);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 2 done.");
}
