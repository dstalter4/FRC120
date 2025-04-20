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
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::QuickTestCode
///
/// Test code to try out for operator control mode.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::QuickTestCode()
{
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
void CmsdRobot::MotorTest()
{
    static XboxController * pDriveJoystick;
    static XboxController * pControlJoystick;
    static TalonSRX * pLeft1;
    static TalonSRX * pLeft2;
    static TalonSRX * pRight1;
    static TalonSRX * pRight2;
    static bool bInitialized = false;
    if (!bInitialized)
    {
        pDriveJoystick = new XboxController(DRIVE_JOYSTICK);
        pControlJoystick = new XboxController(CONTROL_JOYSTICK);
        
        pLeft1 = new TalonSRX(LEFT_MOTORS_CAN_START_ID);
        pLeft2 = new TalonSRX(LEFT_MOTORS_CAN_START_ID + 1);
        pRight1 = new TalonSRX(RIGHT_MOTORS_CAN_START_ID);
        pRight2 = new TalonSRX(RIGHT_MOTORS_CAN_START_ID + 1);
        
        pLeft1->SetNeutralMode(NeutralMode::Coast);
        pLeft2->SetNeutralMode(NeutralMode::Coast);
        pRight1->SetNeutralMode(NeutralMode::Coast);
        pRight2->SetNeutralMode(NeutralMode::Coast);
        
        bInitialized = true;
    }
    
    while (pDriveJoystick->GetRawButton(6))
    {
        pLeft1->Set(ControlMode::PercentOutput, 1);
        DriverStation::RefreshData();
    }
    while (pDriveJoystick->GetRawButton(7))
    {
        pLeft1->Set(ControlMode::PercentOutput, -1);
        DriverStation::RefreshData();
    }
    while (pDriveJoystick->GetRawButton(8))
    {
        pLeft2->Set(ControlMode::PercentOutput, 1);
        DriverStation::RefreshData();
    }
    while (pDriveJoystick->GetRawButton(9))
    {
        pLeft2->Set(ControlMode::PercentOutput, -1);
        DriverStation::RefreshData();
    }
    while (pControlJoystick->GetRawButton(6))
    {
        pRight1->Set(ControlMode::PercentOutput, 1);
        DriverStation::RefreshData();
    }
    while (pControlJoystick->GetRawButton(7))
    {
        pRight1->Set(ControlMode::PercentOutput, -1);
        DriverStation::RefreshData();
    }
    while (pControlJoystick->GetRawButton(8))
    {
        pRight2->Set(ControlMode::PercentOutput, 1);
        DriverStation::RefreshData();
    }
    while (pControlJoystick->GetRawButton(9))
    {
        pRight2->Set(ControlMode::PercentOutput, -1);
        DriverStation::RefreshData();
    }
    
    pLeft1->Set(ControlMode::PercentOutput, 0);
    pLeft2->Set(ControlMode::PercentOutput, 0);
    pRight1->Set(ControlMode::PercentOutput, 0);
    pRight2->Set(ControlMode::PercentOutput, 0);
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::TankDrive
///
/// Test code for tank drive of the robot.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::TankDrive()
{
    m_pLeftDriveMotor->Set(-m_pDriveJoystick->GetLeftY());
    m_pRightDriveMotor->Set(m_pControlJoystick->GetRightY());
}
