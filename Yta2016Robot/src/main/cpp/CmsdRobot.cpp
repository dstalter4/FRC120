////////////////////////////////////////////////////////////////////////////////
/// @file CmsdRobot.cpp
///
/// Implementation of the CmsdRobot class.  This file contains the functions
/// for full robot operation in FRC.  It contains the autonomous and operator
/// control routines as well as all necessary support for interacting with all
/// motors, sensors and input/outputs on the robot.
///
/// CMSD FRC 2016
/// Author(s): David Stalter
/// @Edit History
/// - dts   09-JAN-2016 Created from 2015.
///
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "CmsdRobot.hpp"    // For class declaration (and other headers)

// Do not use static initialization!  There is a bug in the
// WPI libraries that will cause an exception during object
// instantiation for the robot.


////////////////////////////////////////////////////////////////
// @method CmsdRobot::CmsdRobot
///
/// Constructor.  Instantiates all robot control objects.
///
////////////////////////////////////////////////////////////////
CmsdRobot::CmsdRobot()
: m_pDriverStation              (&DriverStation::GetInstance())
, m_pDriveJoystick              (new Joystick(DRIVE_JOYSTICK))
, m_pControlJoystick            (new Joystick(CONTROL_JOYSTICK))
, m_pLeftDriveMotor             (new TalonMotorGroup(NUMBER_OF_LEFT_DRIVE_MOTORS, LEFT_MOTORS_CAN_START_ID, NeutralMode::kNeutralMode_Coast, ControlMode::FOLLOW))
, m_pRightDriveMotor            (new TalonMotorGroup(NUMBER_OF_RIGHT_DRIVE_MOTORS, RIGHT_MOTORS_CAN_START_ID, NeutralMode::kNeutralMode_Coast, ControlMode::FOLLOW))
, m_pBallLiftMotor              (new TalonMotorGroup(NUMBER_OF_BALL_LIFT_MOTORS, BALL_LIFT_CAN_START_ID, NeutralMode::kNeutralMode_Brake, ControlMode::INVERSE))
, m_pRobotClimbMotor            (new TalonMotorGroup(NUMBER_OF_ROBOT_CLIMB_MOTORS, ROBOT_CLIMB_CAN_START_ID, NeutralMode::kNeutralMode_Brake, ControlMode::INVERSE))
, m_pBallIntakeMotor            (new CANTalon(BALL_INTAKE_CAN_ID))
, m_pLedRelay                   (new Relay(LED_RELAY_ID))
, m_pAutonomousRoutine1Switch   (new DigitalInput(AUTONOMOUS_ROUTINE_1_SWITCH))
, m_pAutonomousRoutine2Switch   (new DigitalInput(AUTONOMOUS_ROUTINE_2_SWITCH))
, m_pAutonomousRoutine3Switch   (new DigitalInput(AUTONOMOUS_ROUTINE_3_SWITCH))
, m_pLiftLowerLimitSwitch       (new DigitalInput(BALL_LIFT_LOWER_LIMIT_SWITCH))
, m_pLiftUpperLimitSwitch       (new DigitalInput(BALL_LIFT_UPPER_LIMIT_SWITCH))
, m_pBallLaunchSolenoid         (new DoubleSolenoid(BALL_LAUNCH_FORWARD_SOLENOID, BALL_LAUNCH_REVERSE_SOLENOID))
, m_pExtraBallLaunchSolenoid    (new DoubleSolenoid(EXTRA_BALL_LAUNCH_FORWARD_SOLENOID,EXTRA_BALL_LAUNCH_REVERSE_SOLENOID))
, m_pClimbPoleRaiseSolenoid     (new DoubleSolenoid(CLIMB_POLE_RAISE_FORWARD_SOLENOID, CLIMB_POLE_RAISE_REVERSE_SOLENOID))
, m_pClimbPoleHookSolenoid      (new DoubleSolenoid(CLIMB_POLE_HOOK_FORWARD_SOLENOID, CLIMB_POLE_HOOK_REVERSE_SOLENOID))
, m_pAutonomousTimer            (new Timer())
, m_pSafetyTimer                (new Timer())
, m_pCameraRunTimer             (new Timer())
, m_pSolenoidRetractTimer       (new Timer())
, m_pAccelerometer              (new BuiltInAccelerometer())
, m_pCameras                    (new RobotCamera(CameraType::USB))
, m_serialPortBuffer            ()
, m_pSerialPort                 (new SerialPort(SERIAL_PORT_BAUD_RATE, SerialPort::kMXP, SERIAL_PORT_NUM_DATA_BITS, SerialPort::kParity_None, SerialPort::kStopBits_One))
, m_bDriveSwap                  (true)  // 2016 Change Only!
, m_bShotInProgress             (false)
, m_bPoleRaised                 (false)
, m_bHookLatched                (false)
, m_bTargetInRange              (false)
{
    // Reset all timers
    m_pAutonomousTimer->Reset();
    m_pSafetyTimer->Reset();
    m_pCameraRunTimer->Reset();
    m_pSolenoidRetractTimer->Reset();

    // Reset the serial port
    m_pSerialPort->Reset();
    
    // Reset the position on the ball lift encoders
    m_pBallLiftMotor->CreateEncoderFeedbackDevice(FeedbackDevice::CtreMagEncoder_Absolute);
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::InitialStateSetup
///
/// This method contains the work flow for putting motors,
/// solenoids, etc. into a known state.  It is intended to be
/// used by both autonomous and user control.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::InitialStateSetup()
{
    // Start with motors off
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    m_pBallLiftMotor->Set(OFF);
    m_pBallIntakeMotor->Set(OFF);
    m_pRobotClimbMotor->Set(OFF);

    // Put solenoids in a known state
    m_pBallLaunchSolenoid->Set(SolenoidState::kReverse);
    m_pExtraBallLaunchSolenoid->Set(SolenoidState::kReverse);
    m_pClimbPoleRaiseSolenoid->Set(SolenoidState::kForward);
    m_pClimbPoleHookSolenoid->Set(SolenoidState::kReverse);
    
    // Start the camera timer and orient the camera
    m_pCameraRunTimer->Start();
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::Autonomous
///
/// The autonomous control method.  This method is called once
/// each time the robot enters autonomous control.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::Autonomous()
{
    // Put everything in a stable state
    InitialStateSetup();
    
    // Change values in the header to control having an
    // autonomous routine and which is selected

    // Auto routines 1/2
    if ( m_pAutonomousRoutine1Switch->Get() || m_pAutonomousRoutine2Switch->Get() )
    {       
        // Bar is all the way up, tare it out
        m_pBallLiftMotor->TareEncoder();
        
        float driveSpeed = 0.0F;
        float driveTime = 0.0F;
        
        // Routine 1 only, lower the rear bar
        if (m_pAutonomousRoutine1Switch->Get())
        {
            // Climbing arm moves up
        	m_pClimbPoleRaiseSolenoid->Set(SolenoidState::kForward);
            
        	AutonomousDelay(AUTO_DRIVE_TO_SHOOT_DELAY);
            
            // Lower the ball intake so we clear the bar
        	// The sign has been changed to (-)!!in Cincy
            m_pBallLiftMotor->Set(-AUTO_BALL_LIFT_SPEED);
            
            AutonomousDelay(AUTO_CLIMB_ARM_LOWER_DELAY);
            
            // Climbing arm moves down
            m_pClimbPoleRaiseSolenoid->Set(SolenoidState::kReverse);
            
            AutonomousDelay(AUTO_DRIVE_TO_SHOOT_DELAY);
            
            // Wait until ball intake is down so the low bar is cleared
            while (m_pLiftLowerLimitSwitch->Get()) {}
            m_pBallLiftMotor->Set(OFF);
            
            driveSpeed = AUTO_DRIVE_SPEED_SLOW;
            driveTime = AUTO_DRIVE_DELAY_TIME_SLOW;
        }
        else
        {
            AutonomousDelay(AUTO_2_START_DELAY);
            driveSpeed = AUTO_DRIVE_SPEED_FAST;
            driveTime = AUTO_DRIVE_DELAY_TIME_FAST;
        }
        
        // Drive forward to approach the defenses
        AutonomousDriveSequence(driveSpeed, driveTime);
        
        // Routine 1 also tries to line up and shoot
        if (m_pAutonomousRoutine1Switch->Get())
        {
            // Motors are named backward this year, this will veer right
            m_pRightDriveMotor->Set(0.4F);
            AutonomousDelay(1.25F);
            m_pRightDriveMotor->Set(OFF);
            
            // Stabalize
            AutonomousDelay(AUTO_DRIVE_TO_SHOOT_DELAY);
            
            // Right turn, Clyde
            m_pRightDriveMotor->Set(.5F);
            m_pLeftDriveMotor->Set(.5F);
            AutonomousDelay(1.35F);
            m_pRightDriveMotor->Set(OFF);
            m_pLeftDriveMotor->Set(OFF);
            
            // Inch forward
            AutonomousDriveSequence(-.3F, 1.25F);
            
            // Stabalize
            AutonomousDelay(AUTO_DRIVE_TO_SHOOT_DELAY);
            
            // Take the shot
            //m_pBallLaunchSolenoid->Set(SolenoidState::kForward);
            //AutonomousDelay(AUTO_DRIVE_TO_SHOOT_DELAY);
            //m_pBallLaunchSolenoid->Set(SolenoidState::kReverse);
        }

        // Done, just loop
        while ( m_pDriverStation->IsAutonomous() ) {}
    }   // Autonomous routine 1
    
    // Auto routine 3
    else if ( m_pAutonomousRoutine3Switch->Get() )
    {
        // Done, do nothing until autonomous ends
        while (m_pDriverStation->IsAutonomous()) {}
    }

    /* !!! ONLY ENABLE TEST AUTONOMOUS CODE WHEN TESTING
           SELECT A FUNCTIONING ROUTINE FOR ACTUAL MATCHES !!! */
    else if ( AUTONOMOUS_TEST_ENABLED )
    {
        // This code will never return
        AutonomousTestCode();
    }

    else
    {
        // No option was selected; ensure known behavior to avoid issues
        while ( m_pDriverStation->IsAutonomous() ) {}
    }

}   // End Autonomous



////////////////////////////////////////////////////////////////
// @method CmsdRobot::OperatorControl
///
/// The user control method.  This method is called once each
/// time the robot enters operator control.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::OperatorControl()
{
    // Autonomous should have left things in a known state, but
    // just in case clear everything.  Timers were reset in the
    // constructor, no need to do it again
    InitialStateSetup();
    
    // This is to counter the state logic for arm control
    m_pClimbPoleRaiseSolenoid->Set(SolenoidState::kReverse);

    // Set camera quality
    //CameraServer::GetInstance()->SetQuality(50);
    //CameraServer::GetInstance()->StartAutomaticCapture("cam0");

    // Main tele op loop
    while ( m_pDriverStation->IsOperatorControl() )
    {
        CheckForDriveSwap();

        DriveControlSequence();
        
        BallIntakeSequence();
        
        RobotClimbSequence();

        //LedSequence();

        SolenoidSequence();

        //SonarSensorSequence();

        //SerialPortSequence();
        
        CameraSequence();

        // TEST CODE
        // Recommended to only enable this in test scenarios
        // to not impact matches
        //OperatorTestCode();
        //MotorTest();
        // END TEST CODE
        
    } // End main OperatorControl loop
} // End OperatorControl



////////////////////////////////////////////////////////////////
// @method CmsdRobot::BallIntakeSequence
///
/// This method contains the main workflow for controlling
/// ball pickup.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::BallIntakeSequence()
{
    float throttleControl = GetThrottleControl(m_pControlJoystick);
    
    // First check for ball intake
    if (m_pControlJoystick->GetRawButton(BALL_INTAKE_FORWARD_BUTTON))
    {
        m_pBallIntakeMotor->Set(ON * throttleControl);
    }
    else if (m_pControlJoystick->GetRawButton(BALL_INTAKE_REVERSE_BUTTON))
    {
        m_pBallIntakeMotor->Set(-ON * throttleControl);
    }
    else
    {
        m_pBallIntakeMotor->Set(OFF);
    }
    
    // Now check the ball lift mechanism
    bool bLowerLimitSwitchState = m_pLiftLowerLimitSwitch->Get();
    bool bUpperLimitSwitchState = m_pLiftUpperLimitSwitch->Get();
    
    // If the upper limit switch is tripped, tare out
    if (!bUpperLimitSwitchState)
    {
        m_pBallLiftMotor->TareEncoder();
    }
    //For Cincinattin this has been changed (Reveresed!both limit Switches and Ball_InTAKE!!)
    if (m_pControlJoystick->GetRawButton(BALL_INTAKE_LIFT_UP_BUTTON) && bUpperLimitSwitchState)
    {
        m_pBallLiftMotor->Set(BALL_LIFT_MAX_OUTPUT);
    }
    else if (m_pControlJoystick->GetRawButton(BALL_INTAKE_LIFT_DOWN_BUTTON) && bLowerLimitSwitchState)
    {
        m_pBallLiftMotor->Set(-BALL_LIFT_MAX_OUTPUT);
    }
    /*
    else if (m_pDriveJoystick->GetRawButton(BALL_INTAKE_LIFT_AUTO_DOWN_BUTTON) && bLowerLimitSwitchState)
    {
        m_pBallLiftMotor->StartEncoderMove(BALL_LIFT_DOWN_POSITION);
    }
    else if (m_pDriveJoystick->GetRawButton(BALL_INTAKE_LIFT_AUTO_DRIVE_BUTTON))
    {
        m_pBallLiftMotor->StartEncoderMove(BALL_LIFT_DRIVE_POSITION);
    }
    else if (m_pDriveJoystick->GetRawButton(BALL_INTAKE_LIFT_AUTO_UP_BUTTON) && bUpperLimitSwitchState)
    {
        m_pBallLiftMotor->StartEncoderMove(BALL_LIFT_UP_POSITION);
    }
    else if (m_pDriveJoystick->GetRawButton(9))
    {
        m_pBallLiftMotor->TareEncoder();
    }
    */
    else
    {
        m_pBallLiftMotor->Set(OFF);
    }
    
    //m_pBallLiftMotor->EncoderSequence(!bLowerLimitSwitchState || !bUpperLimitSwitchState);
    
    /*
    // Now for raising or lowering it
    float yAxisControl = Trim((m_pControlJoystick->GetY() * throttleControl), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
        
    // Negate the input since the controller y-axis input is in the range [-1:1]
    m_pBallLiftMotor->Set(-yAxisControl);   
    */
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::RobotClimbSequence
///
/// This method contains the main workflow for controlling
/// the robot lift mechanisms.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::RobotClimbSequence()
{
    // Peng: m_pControlJoystick
    // Devon: m_pDriveJoystick
    
    // Climbing mechanism state machine
    // Only raise/lower the pole if the hook piston is not extended
    if (m_pControlJoystick->GetRawButton(ROBOT_CLIMB_POLE_RAISE_BUTTON) && !m_bHookLatched) //comment out last and close bracket RG RG RG RG
    {
        m_pClimbPoleRaiseSolenoid->Set(SolenoidState::kForward);
        m_bPoleRaised = true;
    }
    else if (m_pControlJoystick->GetRawButton(ROBOT_CLIMB_POLE_LOWER_BUTTON) && !m_bHookLatched) //comment out last and close bracket
    {
        m_pClimbPoleRaiseSolenoid->Set(SolenoidState::kReverse);
        m_bPoleRaised = false;
    }
    // Only move the hook in/out if the pole raise piston is extended
    else if (m_pControlJoystick->GetRawButton(ROBOT_CLIMB_HOOK_OUT_BUTTON) && m_bPoleRaised) //comment out last and close bracket
    {
        m_pClimbPoleHookSolenoid->Set(SolenoidState::kForward);
        m_bHookLatched = true;
    }
    else if (m_pControlJoystick->GetRawButton(ROBOT_CLIMB_HOOK_IN_BUTTON) && m_bPoleRaised) //comment out last and close bracket
    {
        m_pClimbPoleHookSolenoid->Set(SolenoidState::kReverse);
        m_bHookLatched = false;
    }
    else
    {
    }
    
    // Last check the motor climb inputs
    if (m_pControlJoystick->GetRawButton(ROBOT_CLIMB_UP_BUTTON))
    {
        m_pRobotClimbMotor->Set(ON);
    }
    else if (m_pControlJoystick->GetRawButton(ROBOT_CLIMB_DOWN_BUTTON))
    {
        m_pRobotClimbMotor->Set(-ON);
    }
    else
    {
        m_pRobotClimbMotor->Set(OFF);
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::LedSequence
///
/// This method contains the main workflow for controlling
/// any LEDs on the robot.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::LedSequence()
{
    // If the target's in range, give a visual indication
    if (m_bTargetInRange)
    {
        m_pLedRelay->Set(RelayValue::kOn);
    }
    else
    {
        // Otherwise set them off
        m_pLedRelay->Set(RelayValue::kOff);
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::SolenoidSequence
///
/// This method contains the main workflow for updating the
/// state of the solenoids on the robot.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::SolenoidSequence()
{
    if (m_pDriveJoystick->GetRawButton(BALL_SHOOT_BUTTON) && !m_bShotInProgress)
    {
        // false is wired as launch
        m_pBallLaunchSolenoid->Set(SolenoidState::kForward);
        m_pExtraBallLaunchSolenoid->Set(SolenoidState::kForward);
        m_pSolenoidRetractTimer->Start();
        m_bShotInProgress = true;
    }
    else if (m_bShotInProgress)
    {
        if (m_pSolenoidRetractTimer->Get() >= SOLENOID_RETRACT_TIME)
        {
            m_pBallLaunchSolenoid->Set(SolenoidState::kReverse);
            m_pExtraBallLaunchSolenoid->Set(SolenoidState::kReverse);
            m_pSolenoidRetractTimer->Stop();
            m_pSolenoidRetractTimer->Reset();
            m_bShotInProgress = false;
        }
    }
    else
    {
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::SonarSensorSequence
///
/// This method contains the main workflow for getting updates
/// from the sonar sensors.  In order to not interfere with
/// each other, each sensor is enabled/disabled and checked
/// individually.
///
////////////////////////////////////////////////////////////////
/*void CmsdRobot::SonarSensorSequence()
{
}
*/


////////////////////////////////////////////////////////////////
// @method CmsdRobot::SerialPortSequence
///
/// This method contains the main workflow for interaction with
/// the serial port.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::SerialPortSequence()
{
    // Check for any incoming transmissions, limit it to our read buffer size
    int32_t bytesReceived = m_pSerialPort->GetBytesReceived();
    bytesReceived = (bytesReceived > SERIAL_PORT_BUFFER_SIZE_BYTES) ? SERIAL_PORT_BUFFER_SIZE_BYTES : bytesReceived;

    // If we got data, read it
    if (bytesReceived > 0)
    {
        static_cast<void>(m_pSerialPort->Read(m_serialPortBuffer, bytesReceived));

        // See if its a packet intended for us
        if (memcmp(m_serialPortBuffer, SERIAL_PORT_PACKET_HEADER, SERIAL_PORT_PACKET_HEADER_SIZE_BYTES) == 0)
        {
            // Next character is the command.  Array indexing starts at zero, thus no +1 on the size bytes constant
            int32_t command = static_cast<int32_t>(m_serialPortBuffer[SERIAL_PORT_PACKET_HEADER_SIZE_BYTES]) - ASCII_0_OFFSET;

            // Sanity check it
            if (command >= 0 && command <= 9)
            {
                printf("Received a valid packet, command: %d\n", command);
            }
            else
            {
                printf("Invalid command received: %d\n", command);
            }
        }

        printf(m_serialPortBuffer);
    }
    m_serialPortBuffer[0] = NULL_CHARACTER;
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::CameraSequence
///
/// This method handles camera related behavior.  See the
/// RobotCamera class for full details.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::CameraSequence()
{    
    // Now do the camera processing
    bool bDoFullVisionProcessing = false;
    
    // To not kill the CPU/hog this thread, only do full
    // vision processing (particle analysis) periodically.
    if (m_pCameraRunTimer->Get() >= CAMERA_RUN_INTERVAL_S)
    {
        //bDoFullVisionProcessing = true;
        m_pCameraRunTimer->Reset();
    }
    
    m_bTargetInRange = m_pCameras->ProcessTarget(bDoFullVisionProcessing);
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::DriveControlSequence
///
/// This method contains the main workflow for drive control.
/// It will gather input from the drive joystick and then filter
/// those values to ensure they are past a certain threshold and
/// within range to send to the speed controllers.  Lastly it
/// will actually set the speed values.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::DriveControlSequence()
{
    // Computes what the maximum drive speed could be
    float throttleControl = GetThrottleControl(m_pDriveJoystick);

    // Get joystick inputs and make sure they clear a certain threshold.
    // This will help to drive straight.
    float xAxisDrive = Trim((m_pDriveJoystick->GetX() * throttleControl), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    float yAxisDrive = Trim((m_pDriveJoystick->GetY() * throttleControl), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);

    // If the swap direction button was pressed, negate y value
    if ( m_bDriveSwap )
    {
        yAxisDrive *= -1;
    }

    // Filter motor speeds
    float leftSpeed = Limit((xAxisDrive - yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    float rightSpeed = Limit((xAxisDrive + yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);

    // Set motor speed
    m_pLeftDriveMotor->Set(leftSpeed);
    m_pRightDriveMotor->Set(rightSpeed);
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::Test
///
/// This method is run when entering test mode.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::Test()
{
    while (true)
    {
    }
}



// EXECUTION START
START_ROBOT_CLASS(CmsdRobot);   // Macro to instantiate and run the class
