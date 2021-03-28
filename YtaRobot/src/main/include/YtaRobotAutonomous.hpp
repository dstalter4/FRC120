////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous.hpp
/// @author David Stalter
///
/// @details
/// Contains the declarations for the autonomous portions of code ran in an FRC
/// robot.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTAROBOTAUTONOMOUS_HPP
#define YTAROBOTAUTONOMOUS_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"             // for inline autonomous function declarations

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
    static constexpr double COUNTERACT_COAST_TIME_S             =  0.25;
    static constexpr double ENCODER_DRIVE_MAX_DELAY_S           =  5.00;
    static constexpr double DELAY_SHORT_S                       =  0.50;
    static constexpr double DELAY_MEDIUM_S                      =  1.00;
    static constexpr double DELAY_LONG_S                        =  2.00;
    
    // Autonomous encoder drive constants
    static const int        ENCODER_DRIVE_STRAIGHT_IN           =  12*8;                    
    static constexpr double ENCODER_COMPENSATE_SPEED            =  0.02;
    
    // Autonomous sonar drive constants
    static const uint32_t   SONAR_DRIVE_STATE_SIDE_MASK         = 0x0F;
    static const uint32_t   SONAR_DRIVE_STATE_LATERAL_MASK      = 0xF0;
    static const int        SONAR_LATERAL_DRIVE_DIST_INCHES     =  7*12;
    static const int        SONAR_SIDE_DRIVE_DIST_INCHES        =     6;
    static const int        SONAR_MIN_DRIVE_ENABLE_INCHES       = 10*12;
    static const int        SONAR_INIT_TURN_DIST_INCHES         =     5;
    static const int        SONAR_MAX_ALLOWED_READING_DIFF      =     2;
    static const unsigned   SONAR_BUMPER_CLEARANCE_DIST_INCHES  =     4;
    static constexpr double SONAR_ROUTINE_TIME_S                =  5.00;
    static constexpr double SONAR_DRIVE_LEFT_SPEED              = -0.10;
    static constexpr double SONAR_DRIVE_RIGHT_SPEED             =  0.10;
    static constexpr double SONAR_COMPENSATE_LEFT_SPEED         = -0.05;
    static constexpr double SONAR_COMPENSATE_RIGHT_SPEED        =  0.05;
    
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
inline void YtaRobot::AutonomousDelay(double time)
{
    m_pAutonomousTimer->Start();
    while (m_pAutonomousTimer->Get() < time) {}
    m_pAutonomousTimer->Stop();
    m_pAutonomousTimer->Reset();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousDriveSequence
///
/// Drives during autonomous for a specified amount of time.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::AutonomousDriveSequence(RobotDirection direction, double speed, double time, bool bSwingTurn, double leftDiff, double rightDiff)
{
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    switch (direction)
    {
        case ROBOT_FORWARD:
        {
            leftSpeed = speed * LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        case ROBOT_REVERSE:
        {
            leftSpeed = speed * LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        case ROBOT_LEFT:
        {
            leftSpeed = speed * LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        case ROBOT_RIGHT:
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

    if (bSwingTurn)
    {
        leftSpeed += leftDiff;
        rightSpeed += rightDiff;
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
        case ROBOT_FORWARD:
        {
            leftSpeed *= LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed *= RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        // If we are currently going backward, back drive is forward
        case ROBOT_REVERSE:
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
        case ROBOT_LEFT:
        {
            leftSpeed *= LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed *= RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        // If the turn is right, counteract is left
        case ROBOT_RIGHT:
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
