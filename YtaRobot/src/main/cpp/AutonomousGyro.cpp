////////////////////////////////////////////////////////////////////////////////
/// @file   AutonomousGyro.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous gyroscope routines.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                 // for robot class declaration
#include "YtaRobotAutonomous.hpp"       // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousGyroLeftTurn
///
/// Turns the robot left based on gyro readings.
///
////////////////////////////////////////////////////////////////
bool YtaRobot::AutonomousGyroLeftTurn(double destAngle, double turnSpeed)
{
    // 20xx LEFT TURNS DECREASE GYRO ANGLE
    // Left turn is left motors back, right motors forward
    
    // Left turns are right motors forward, left motors reverse
    m_pLeftDriveMotors->Set(turnSpeed * LEFT_DRIVE_REVERSE_SCALAR);
    m_pRightDriveMotors->Set(turnSpeed * RIGHT_DRIVE_FORWARD_SCALAR);
    
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    
    // Angle will be decreasing.  Assumption: Robot orientation is 0 -> 90 -> 180 -> 270 -> 360.
    // @todo: This needs improvements for figuring out fastest way to turn and crossing the 0/360 boundary.
    while ((GetGyroValue(BNO055) > destAngle) && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE))
    {
        if (!m_pDriverStation->IsAutonomous())
        {
            break;
        }
        
        SmartDashboard::PutNumber("Gyro angle", GetGyroValue(BNO055));
    }
    
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Stop();
    if (m_pSafetyTimer->Get() > SAFETY_TIMER_MAX_VALUE)
    {
        m_pSafetyTimer->Reset();
        return false;
    }
    m_pSafetyTimer->Reset();
    
    // Counteract coast
    AutonomousBackDriveTurn(ROBOT_LEFT);
    
    return true;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousGyroRightTurn
///
/// Turns the robot right based on gyro readings.
///
////////////////////////////////////////////////////////////////
bool YtaRobot::AutonomousGyroRightTurn(double destAngle, double turnSpeed)
{
    // 20xx RIGHT TURNS INCREASE GYRO ANGLE
    // Right turn is left motors forward, right motors back
    
    // Right turns are left motors forward, right motors reverse
    m_pLeftDriveMotors->Set(turnSpeed * LEFT_DRIVE_FORWARD_SCALAR);
    m_pRightDriveMotors->Set(turnSpeed * RIGHT_DRIVE_REVERSE_SCALAR);
    
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    
    // Angle will be increasing.  Assumption: Robot orientation is 0 -> 90 -> 180 -> 270 -> 360.
    // @todo: This needs improvements for figuring out fastest way to turn and crossing the 0/360 boundary.
    while ((GetGyroValue(BNO055) < destAngle) && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE))
    {
        if (!m_pDriverStation->IsAutonomous())
        {
            break;
        }
        
        SmartDashboard::PutNumber("Gyro angle", GetGyroValue(BNO055));
    }
    
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Stop();
    if (m_pSafetyTimer->Get() > SAFETY_TIMER_MAX_VALUE)
    {
        m_pSafetyTimer->Reset();
        return false;
    }
    m_pSafetyTimer->Reset();
    
    // Counteract coast
    AutonomousBackDriveTurn(ROBOT_RIGHT);
    
    return true;
}
