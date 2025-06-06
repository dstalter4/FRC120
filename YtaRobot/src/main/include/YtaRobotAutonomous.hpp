////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous.hpp
/// @author David Stalter
///
/// @details
/// Contains the declarations for the autonomous portions of code ran in an FRC
/// robot.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTAROBOTAUTONOMOUS_HPP
#define YTAROBOTAUTONOMOUS_HPP

// SYSTEM INCLUDES
#include <frc2/command/CommandPtr.h>    // for CommandPtr type

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                 // for inline autonomous function declarations

using namespace frc;

////////////////////////////////////////////////////////////////
/// @namespace YtaRobotAutonomous
///
/// Namespace that contains robot autonomous variable and
/// function declarations.
///
////////////////////////////////////////////////////////////////
namespace YtaRobotAutonomous
{
    // TYPEDEFS
    // (none)
    
    // ENUMS
    // (none)    
    
    // STRUCTS
    // (none)
    
    // VARIABLES
    extern bool bAutonomousExecutionComplete;
    extern std::optional<CommandPtr> AutonomousCommand;
    
    // CONSTS
    
    // Autonomous Mode Constants
    // @todo: Convert to class and make a friend in YtaRobot
    
    // Note: Only enable one autonomous routine!
    // Note: Autonomous routines are currently controlled by
    // the SendableChooser.
    //static const bool       ROUTINE_1                           = true;
    //static const bool       ROUTINE_2                           = false;
    //static const bool       ROUTINE_3                           = false;
    //static const bool       TEST_ENABLED                        = false;
    static const bool       USE_COMMAND_BASED_AUTONOMOUS        = false;

    // Autonomous drive speed constants
    static constexpr double DRIVE_SPEED_SLOW                    =  0.30;
    static constexpr double DRIVE_SPEED_FAST                    =  0.50;
    static constexpr double TURN_SPEED                          =  0.25;
    static constexpr double COUNTERACT_COAST_MOTOR_SPEED        =  0.20;
    
    // Autonomous angle constants
    static const int        FORTY_FIVE_DEGREES                  = 45;
    static const int        NINETY_DEGREES                      = 90;
    static const int        ONE_HUNDRED_EIGHTY_DEGREES          = 180;
    static const int        THREE_HUNDRED_SIXTY_DEGREES         = 360;
    
    // Autonomous delay constants
    static constexpr units::second_t SWERVE_OP_STEP_TIME_S      =  0.10_s;
    static constexpr units::second_t COUNTERACT_COAST_TIME_S    =  0.25_s;
    static constexpr units::second_t DELAY_SHORT_S              =  0.50_s;
    static constexpr units::second_t DELAY_MEDIUM_S             =  1.00_s;
    static constexpr units::second_t DELAY_LONG_S               =  2.00_s;
    
    // Autonomous misc constants
    static const unsigned   I2C_THREAD_UPDATE_RATE_MS           = 20U;
    
} // End namespace



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousDelay
///
/// Waits for a specified amount of time in autonomous.  Used
/// while an operation is ongoing but not yet complete, and
/// nothing else needs to occur.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::AutonomousDelay(units::second_t time)
{
    Wait(time);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousDriveSequence
///
/// Drives during autonomous for a specified amount of time
/// using traditional differential drive.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::AutonomousDriveSequence(RobotDirection direction, double speed, units::second_t time)
{
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    switch (direction)
    {
        case RobotDirection::ROBOT_FORWARD:
        {
            leftSpeed = speed * LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        case RobotDirection::ROBOT_REVERSE:
        {
            leftSpeed = speed * LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        case RobotDirection::ROBOT_LEFT:
        {
            leftSpeed = speed * LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        case RobotDirection::ROBOT_RIGHT:
        {
            leftSpeed = speed * LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        default:
        {
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            break;
        }
    }

    // First turn the motors on
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);

    // Time it
    AutonomousDelay(time);

    // Motors back off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousSwerveDriveSequence
///
/// Drives during autonomous for a specified amount of time
/// using swerve drive modules.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::AutonomousSwerveDriveSequence(RobotSwerveDirections & rSwerveDirections, double translationSpeed, double strafeSpeed, double rotateSpeed, units::second_t time, bool bFieldRelative)
{
    units::meter_t translation = 0.0_m;
    units::meter_t strafe = 0.0_m;

    switch (rSwerveDirections.GetTranslation())
    {
        case RobotTranslation::ROBOT_TRANSLATION_FORWARD:
        {
            translation = units::meter_t(translationSpeed);
            break;
        }
        case RobotTranslation::ROBOT_TRANSLATION_REVERSE:
        {
            translation = units::meter_t(-translationSpeed);
            break;
        }
        default:
        {
            break;
        }
    }

    switch (rSwerveDirections.GetStrafe())
    {
        case RobotStrafe::ROBOT_STRAFE_LEFT:
        {
            strafe = units::meter_t(strafeSpeed);
            break;
        }
        case RobotStrafe::ROBOT_STRAFE_RIGHT:
        {
            strafe = units::meter_t(-strafeSpeed);
            break;
        }
        default:
        {
            break;
        }
    }

    switch (rSwerveDirections.GetRotation())
    {
        case RobotRotation::ROBOT_NO_ROTATION:
        {
            // Just in case the user decided to pass a speed anyway
            rotateSpeed = 0.0;
            break;
        }
        case RobotRotation::ROBOT_CLOCKWISE:
        {
            rotateSpeed *= -1.0;
            break;
        }
        case RobotRotation::ROBOT_COUNTER_CLOCKWISE:
        default:
        {
            break;
        }
    }

    Translation2d translation2d = {translation, strafe};
    units::second_t duration = 0.0_s;
    while (duration < time)
    {
        m_pSwerveDrive->SetModuleStates(translation2d, rotateSpeed, bFieldRelative, true);
        AutonomousDelay(YtaRobotAutonomous::SWERVE_OP_STEP_TIME_S);
        duration += YtaRobotAutonomous::SWERVE_OP_STEP_TIME_S;
    }

    // Stop motion
    m_pSwerveDrive->SetModuleStates({0_m, 0_m}, 0.0, true, true);

    // Clear the swerve directions to prevent the caller from
    // accidentally reusing them without explicitly setting them again.
    rSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousRotateByGyroSequence
///
/// Turns the robot by the gyro.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::AutonomousRotateByGyroSequence(RobotRotation robotRotation, double rotateDegrees, double rotateSpeed, bool bFieldRelative)
{
    double startingGyroAngle = m_pPigeon->GetYaw().GetValueAsDouble();

    while (std::abs(m_pPigeon->GetYaw().GetValueAsDouble() - startingGyroAngle) <= rotateDegrees)
    {
        if (robotRotation == RobotRotation::ROBOT_CLOCKWISE)
        {
            m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, rotateSpeed, bFieldRelative, true);
        }
        else if (robotRotation == RobotRotation::ROBOT_COUNTER_CLOCKWISE)
        {
            m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, -rotateSpeed, bFieldRelative, true);
        }
        else
        {
        }
    }

    // Stop motion
    m_pSwerveDrive->SetModuleStates({0_m, 0_m}, 0.0, true, true);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousBackDrive
///
/// Back drives the motors to abruptly stop the robot.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::AutonomousBackDrive(RobotDirection currentDirection)
{
    double leftSpeed = YtaRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    double rightSpeed = YtaRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;

    switch (currentDirection)
    {
        // If we are currently going forward, back drive is reverse
        case RobotDirection::ROBOT_FORWARD:
        {
            leftSpeed *= LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed *= RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        // If we are currently going reverse, back drive is forward
        case RobotDirection::ROBOT_REVERSE:
        {
            leftSpeed *= LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed *= RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        default:
        {
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            break;
        }
    }
    
    // Counteract coast
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);
    
    // Delay
    AutonomousDelay(YtaRobotAutonomous::COUNTERACT_COAST_TIME_S);
    
    // Motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Reset();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousBackDriveTurn
///
/// Back drives the motors to abruptly stop the robot during
/// a turn.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::AutonomousBackDriveTurn(RobotDirection currentDirection)
{
    double leftSpeed = YtaRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    double rightSpeed = YtaRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;

    switch (currentDirection)
    {
        // If the turn is left, counteract is right
        case RobotDirection::ROBOT_LEFT:
        {
            leftSpeed *= LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed *= RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        // If the turn is right, counteract is left
        case RobotDirection::ROBOT_RIGHT:
        {
            leftSpeed *= LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed *= RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        default:
        {
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            break;
        }
    }
    
    // Counteract coast
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);
    
    // Delay
    AutonomousDelay(YtaRobotAutonomous::COUNTERACT_COAST_TIME_S);
    
    // Motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Reset();
}

#endif // YTAROBOTAUTONOMOUS_HPP
