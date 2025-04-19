////////////////////////////////////////////////////////////////////////////////
/// @file TestCode.cpp
///
/// A class to separate and test code for the robot.  This keeps official
/// stable robot code isolated.
///
/// CMSD FRC 2016
/// Author: David Stalter
///
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "CmsdRobot.hpp"        // Robot class declaration



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousTestCode
///
/// Test code to try out for autonomous mode.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousTestCode()
{
    // Motors off
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    
    // Done, just loop
    while ( m_pDriverStation->IsAutonomous() )
    {
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::OperatorTestCode
///
/// Test code to try out for operator control mode.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::OperatorTestCode()
{
    // Test code for reading the built in accelerometer
    double x = m_pAccelerometer->GetX();
    double y = m_pAccelerometer->GetY();
    double z = m_pAccelerometer->GetZ();
    printf("x: %f, y: %f, z: %f\n", x, y, z);

    // Sample code for testing the detect trigger change code
    TriggerChangeValues testValues;
    testValues.bCurrentValue = m_pControlJoystick->GetRawButton(10);
    if ( DetectTriggerChange(&testValues) )
    {
        printf("Trigger change detected!\n");
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::MotorTest
///
/// Motor test code to make sure they aren't driving against
/// each other.
///
////////////////////////////////////////////////////////////////
Joystick * pDriveJoystick;
Joystick * pControlJoystick;
CANTalon * pLeft1;
CANTalon * pLeft2;
CANTalon * pRight1;
CANTalon * pRight2;
void CmsdRobot::MotorTest()
{
    static bool bInitialized = false;
    if (!bInitialized)
    {
        pDriveJoystick = new Joystick(DRIVE_JOYSTICK);
        pControlJoystick = new Joystick(CONTROL_JOYSTICK);
        
        pLeft1 = new CANTalon(LEFT_MOTORS_CAN_START_ID);
        pLeft2 = new CANTalon(LEFT_MOTORS_CAN_START_ID + 1);
        pRight1 = new CANTalon(RIGHT_MOTORS_CAN_START_ID);
        pRight2 = new CANTalon(RIGHT_MOTORS_CAN_START_ID + 1);
        
        pLeft1->SetControlMode(CANSpeedController::kPercentVbus);
        pLeft2->SetControlMode(CANSpeedController::kPercentVbus);
        pRight1->SetControlMode(CANSpeedController::kPercentVbus);
        pRight2->SetControlMode(CANSpeedController::kPercentVbus);
        
        pLeft1->ConfigNeutralMode(CANSpeedController::kNeutralMode_Coast);
        pLeft2->ConfigNeutralMode(CANSpeedController::kNeutralMode_Coast);
        pRight1->ConfigNeutralMode(CANSpeedController::kNeutralMode_Coast);
        pRight2->ConfigNeutralMode(CANSpeedController::kNeutralMode_Coast);
        
        bInitialized = true;
    }
    
    while (pDriveJoystick->GetRawButton(6))
    {
        pLeft1->Set(1);
    }
    while (pDriveJoystick->GetRawButton(7))
    {
        pLeft1->Set(-1);
    }
    while (pDriveJoystick->GetRawButton(8))
    {
        pLeft2->Set(1);
    }
    while (pDriveJoystick->GetRawButton(9))
    {
        pLeft2->Set(-1);
    }
    while (pControlJoystick->GetRawButton(6))
    {
        pRight1->Set(1);
    }
    while (pControlJoystick->GetRawButton(7))
    {
        pRight1->Set(-1);
    }
    while (pControlJoystick->GetRawButton(8))
    {
        pRight2->Set(1);
    }
    while (pControlJoystick->GetRawButton(9))
    {
        pRight2->Set(-1);
    }
    
    pLeft1->Set(0);
    pLeft2->Set(0);
    pRight1->Set(0);
    pRight2->Set(0);
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::TankDrive
///
/// Test code for tank drive of the robot.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::TankDrive()
{
    m_pLeftDriveMotor->Set(-m_pDriveJoystick->GetY());
    m_pRightDriveMotor->Set(m_pControlJoystick->GetY());
}
