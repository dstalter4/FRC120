////////////////////////////////////////////////////////////////////////////////
/// @file   AutonomousEncoder.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous encoder based routines.
///
/// Copyright (c) 2022 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotUtils.hpp"               // for DEBUG_PRINTS
#include "YtaRobot.hpp"                 // for robot class declaration
#include "YtaRobotAutonomous.hpp"       // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method YtaRobot::GetEncoderRotationsFromInches
///
/// Returns a number of encoder turns based on input distance in
/// inches and a diameter of the object turning.  It is based on
/// the 4x (4096) quadrature encoders.
///
////////////////////////////////////////////////////////////////
int YtaRobot::GetEncoderRotationsFromInches(int inches, double diameter, bool bUseQuadEncoding)
{
    // c = PI*d
    // (PI*d)/4096 is ratio of one encoder turn to a distance of
    // travel of one diameter rotations.  To scale up, use cross
    // multiply and divide.  Therefore:
    //   PI * d     x(in.)
    //  -------- = --------
    //    4096      y(rot)
    // x and d are inputs, so solve for y.
    // y = (4096x)/(PI*d)
    // This is for quadrature encoding, so if analog (single)
    // encoding is desired, the result needs to be divided by four.
    // If 4" wheels are in use, 3911.39188 turns = 12"
    volatile int numerator = QUADRATURE_ENCODING_ROTATIONS * inches;
    volatile double denominator = M_PI * diameter;
    volatile int result = numerator / denominator;
    //int result = (QUADRATURE_ENCODING_ROTATIONS * inches) / (M_PI * diameter);
    
    if (!bUseQuadEncoding)
    {
        result /= 4;
    }
    
    return result;
}



////////////////////////////////////////////////////////////////
// @method YtaRobot::AutonomousEncoderDrive
///
/// Autonomous method to drive the robot controlled by the
/// encoders.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousEncoderDrive(double speed, double distance, RobotDirection direction)
{
    // 20xx LEFT ENCODER VALUE DECREASES GOING FORWARD
    // 20xx RIGHT ENCODER VALUE INCREASES GOING FORWARD
    
    // 1024 is the encoder output after one rotation
    // 20xx Wheels are 4", therefore circumference = 4*PI
    // Ratio is (4*PI)/1024 = PI/256
    //
    //   x  |  y  
    // -----------
    //  12" | (3072 / PI) = 977.84797
    //  24" | (6144 / PI) = 1955.6959
    //  ... | ...
    // 120" | (30720 / PI) = 9778.4797
    // 132" | (33792 / PI) = 10756.32767
    // 144" | (36864 / PI) = 11734.17564
    
    // New drive operation, tare encoders
    m_pLeftDriveMotors->TareEncoder();
    m_pRightDriveMotors->TareEncoder();
    
    // Start the safety timer        
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    
    // Get initial encoder values
    int leftEncVal = 0;
    int rightEncVal = 0;
    
    do
    {
        if (!DriverStation::IsAutonomous())
        {
            break;
        }
        
        // Set speeds, adjust below if needed
        double leftDriveSpeed = speed;
        double rightDriveSpeed = speed;
        double leftDriveScale = 0.0;
        double rightDriveScale = 0.0;
        
        // Get encoder values to always be positive, based on direction.
        // Also scale the drive motors for direction.
        switch (direction)
        {
            case ROBOT_FORWARD:
            {
                leftEncVal = -(m_pLeftDriveMotors->GetEncoderValue());
                rightEncVal = m_pRightDriveMotors->GetEncoderValue();
                leftDriveScale = LEFT_DRIVE_FORWARD_SCALAR;
                rightDriveScale = RIGHT_DRIVE_FORWARD_SCALAR;
                break;
            }
            case ROBOT_REVERSE:
            {
                leftEncVal = m_pLeftDriveMotors->GetEncoderValue();
                rightEncVal = -(m_pRightDriveMotors->GetEncoderValue());
                leftDriveScale = LEFT_DRIVE_REVERSE_SCALAR;
                rightDriveScale = RIGHT_DRIVE_REVERSE_SCALAR;
                break;
            }
            default:
            {
                leftDriveSpeed = 0.0;
                rightDriveSpeed = 0.0;
                break;
            }
        }
    
        // If left is ahead of right, slow down left, increase right
        if (leftEncVal > rightEncVal)
        {
            leftDriveSpeed -= YtaRobotAutonomous::ENCODER_COMPENSATE_SPEED;
            rightDriveSpeed += YtaRobotAutonomous::ENCODER_COMPENSATE_SPEED;
        }
        // If right is head of left, slow down right, increase left
        else if (leftEncVal < rightEncVal)
        {
            leftDriveSpeed += YtaRobotAutonomous::ENCODER_COMPENSATE_SPEED;
            rightDriveSpeed -= YtaRobotAutonomous::ENCODER_COMPENSATE_SPEED;
        }
        else
        {
        }
        
        // Motors on
        m_pLeftDriveMotors->Set(leftDriveSpeed * leftDriveScale);
        m_pRightDriveMotors->Set(rightDriveSpeed * rightDriveScale);
        
        // Send stats back to the smart dashboard
        if (RobotUtils::DEBUG_PRINTS)
        {
            SmartDashboard::PutNumber("Enc. L: ", leftEncVal);
            SmartDashboard::PutNumber("Enc. R: ", rightEncVal);
            SmartDashboard::PutNumber("Enc Diff: ", std::abs(leftEncVal - rightEncVal));
        }
    
    } while ( (leftEncVal < GetEncoderRotationsFromInches(distance, DRIVE_WHEEL_DIAMETER_INCHES)) 
           && (rightEncVal < GetEncoderRotationsFromInches(distance, DRIVE_WHEEL_DIAMETER_INCHES))
           && (m_pSafetyTimer->Get() <= YtaRobotAutonomous::ENCODER_DRIVE_MAX_DELAY_S) );
    
    // Motors back off    
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    AutonomousBackDrive(direction);
    
    // Stop and reset the safety timer
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
}
