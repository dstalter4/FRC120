////////////////////////////////////////////////////////////////////////////////
/// @file   AutonomousSonar.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous sonar routines.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotI2c.hpp"                 // for GetSonarData()
#include "YtaRobot.hpp"                 // for robot class declaration
#include "YtaRobotAutonomous.hpp"       // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousSonarDrive
///
/// Autonomous method to drive the robot controlled by the
/// sonar sensors.
///
////////////////////////////////////////////////////////////////
bool YtaRobot::AutonomousSonarDrive(RobotDirection direction, SonarDriveState driveState, uint32_t destLateralDist, uint32_t destSideDist)
{    
    // Set directions based on drive state
    uint32_t sideDirection = driveState & YtaRobotAutonomous::SONAR_DRIVE_STATE_SIDE_MASK;
    uint32_t lateralDirection = driveState & YtaRobotAutonomous::SONAR_DRIVE_STATE_LATERAL_MASK;
    
    uint32_t frontGuideSensor = 0U;
    uint32_t backGuideSensor = 0U;
    uint32_t destGuideSensorA = 0U;
    uint32_t destGuideSensorB = 0U;
    
    // Set values based on which side is guiding drive        
    switch (lateralDirection)
    {
        case FORWARD_GUIDE:
        {
            destGuideSensorA = RobotI2c::GetSonarData()->m_FrontDistances.m_SonarA;
            destGuideSensorB = RobotI2c::GetSonarData()->m_FrontDistances.m_SonarB;
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    frontGuideSensor = RobotI2c::GetSonarData()->m_LeftDistances.m_SonarB;
                    backGuideSensor = RobotI2c::GetSonarData()->m_LeftDistances.m_SonarA;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    frontGuideSensor = RobotI2c::GetSonarData()->m_RightDistances.m_SonarA;
                    backGuideSensor = RobotI2c::GetSonarData()->m_RightDistances.m_SonarB;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            break;
        }
        case REVERSE_GUIDE:
        {
            destGuideSensorA = RobotI2c::GetSonarData()->m_BackDistances.m_SonarA;
            destGuideSensorB = RobotI2c::GetSonarData()->m_BackDistances.m_SonarB;
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    frontGuideSensor = RobotI2c::GetSonarData()->m_LeftDistances.m_SonarA;
                    backGuideSensor = RobotI2c::GetSonarData()->m_LeftDistances.m_SonarB;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    frontGuideSensor = RobotI2c::GetSonarData()->m_RightDistances.m_SonarB;
                    backGuideSensor = RobotI2c::GetSonarData()->m_RightDistances.m_SonarA;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            break;
        }
        default:
        {
            break;
        }
    }
    
    // Start with defaults of off and no turning
    double leftDriveSpeed = OFF;
    double rightDriveSpeed = OFF;    
    bool bLeftTurn = false;
    bool bRightTurn = false;
    bool bCanOverrideTurn = true;
    
    // Make sure we're close enough to a guiding structure
    if (    (frontGuideSensor < YtaRobotAutonomous::SONAR_MIN_DRIVE_ENABLE_INCHES)
         && (backGuideSensor < YtaRobotAutonomous::SONAR_MIN_DRIVE_ENABLE_INCHES) )
    {
        // Start assuming a straight drive
        leftDriveSpeed = YtaRobotAutonomous::SONAR_DRIVE_LEFT_SPEED;
        rightDriveSpeed = YtaRobotAutonomous::SONAR_DRIVE_RIGHT_SPEED;
        
        // Check for turning need.  The first checks here determine
        // if we need to turn the robot left or right, and are to
        // align the robot at a (mostly) right angle.
        if (frontGuideSensor > backGuideSensor)
        {
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    bRightTurn = true;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    bLeftTurn = true;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            // If the robot is offset too sharply, don't allow
            // the guiding below to override what we want to do.
            if ((frontGuideSensor - backGuideSensor) > YtaRobotAutonomous::SONAR_MAX_ALLOWED_READING_DIFF)
            {
                bCanOverrideTurn = false;
            }
        }
        else if (backGuideSensor > frontGuideSensor)
        {
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    bLeftTurn = true;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    bRightTurn = true;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            // If the robot is offset too sharply, don't allow
            // the guiding below to override what we want to do.
            if ((backGuideSensor - frontGuideSensor) > YtaRobotAutonomous::SONAR_MAX_ALLOWED_READING_DIFF)
            {
                bCanOverrideTurn = false;
            }
        }
        else
        {
        }
        
        // Align with the destination distance.  These checks, unlike the ones
        // above, are to move towards the target distance from the wall.
        if (bCanOverrideTurn && (frontGuideSensor > destSideDist))
        {
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    bLeftTurn = true;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    bRightTurn = true;
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        
        // Set the motor speed values
        if (bLeftTurn)
        {
            leftDriveSpeed -= YtaRobotAutonomous::SONAR_COMPENSATE_LEFT_SPEED;
            rightDriveSpeed += YtaRobotAutonomous::SONAR_COMPENSATE_RIGHT_SPEED;
        }
        else if (bRightTurn)
        {
            leftDriveSpeed += YtaRobotAutonomous::SONAR_COMPENSATE_LEFT_SPEED;
            rightDriveSpeed -= YtaRobotAutonomous::SONAR_COMPENSATE_RIGHT_SPEED;
        }
        else
        {
        }
        
        // Speeds are now set based on need to turn.  Enable motors
        // only if we have not reached the maximum distance.
        if ((destGuideSensorA < destLateralDist) && (destGuideSensorB < destLateralDist))
        {
            if (direction == ROBOT_FORWARD)
            {
                m_pLeftDriveMotors->Set(leftDriveSpeed * LEFT_DRIVE_FORWARD_SCALAR);
                m_pRightDriveMotors->Set(rightDriveSpeed * RIGHT_DRIVE_FORWARD_SCALAR);
            }
            else if (direction == ROBOT_REVERSE)
            {
                m_pLeftDriveMotors->Set(leftDriveSpeed * LEFT_DRIVE_REVERSE_SCALAR);
                m_pRightDriveMotors->Set(rightDriveSpeed * RIGHT_DRIVE_REVERSE_SCALAR);
            }
            else
            {
            }
            
            return false;
        }
    }
    
    return true;
}
