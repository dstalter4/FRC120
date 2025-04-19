////////////////////////////////////////////////////////////////////////////////
/// @file CmsdRobot.hpp
///
/// This is the class declaration for a FRC robot derived from the WPI library
/// base classes.  The SampleRobot class is the base of a robot application that
/// will automatically call the Autonomous and OperatorControl methods at the
/// right time as controlled by the switches on the driver station or the field
/// controls.
///
/// CMSD FRC 2016
/// Author(s): David Stalter
/// @Edit History
/// - dts   09-JAN-2016 Created from 2015.
///
////////////////////////////////////////////////////////////////////////////////

#ifndef CMSDROBOT_HPP
#define CMSDROBOT_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "WPILib.h"                 // For FRC library

// C++ INCLUDES
#include "TalonMotorGroup.hpp"      // For Talon group motor control
#include "RobotCamera.hpp"          // For interaction with the cameras

////////////////////////////////////////////////////////////////
// @class CmsdRobot
///
/// Derived class from SampleRobot.  The object that will
/// control all robot functionality.
///
////////////////////////////////////////////////////////////////
class CmsdRobot : public SampleRobot
{
public:

    // MEMBER FUNCTIONS
    
    // Autonomous routine
    void Autonomous();
    
    // Main operator drive control routine
    void OperatorControl();
    
    // Test mode routine
    void Test();
    
    // Constructor, destructor
    CmsdRobot();
    virtual ~CmsdRobot() {}
    
    // Do not declare or implement copy constructor or assignment
    // operator!  The base classes won't like it and most likely
    // it will crash the thread when running!
      
private:

    // TYPEDEFS
    typedef Relay::Value RelayValue;
    typedef Relay::Direction RelayDirection;
    typedef DoubleSolenoid::Value SolenoidState;
    typedef CANTalon::NeutralMode NeutralMode;
    typedef CANTalon::FeedbackDevice FeedbackDevice;
    typedef CANTalon::FeedbackDeviceStatus FeedbackDeviceStatus;
    typedef TalonMotorGroup::ControlMode ControlMode;
    typedef RobotCamera::Camera CameraType;
    
    // ENUMS
    // (none)
    
    // STRUCTS
    struct TriggerChangeValues
    {
        bool bCurrentValue;
        bool bOldValue;
    };
    
    // Ensures a number is between the upper and lower bounds
    inline float Limit( float num, float upper, float lower );
    
    // Trims a number to be in between the upper and lower bounds
    inline float Trim( float num, float upper, float lower );

    // Detect that a button has been pressed (rather than released)
    inline bool DetectTriggerChange(TriggerChangeValues * pTriggerVals);

    // Function to check for drive control direction swap
    inline void CheckForDriveSwap();
    
    // Get a throttle control value from a joystick
    inline float GetThrottleControl(Joystick * pJoystick);

    // Grabs a value from a sonar sensor individually
   // inline float UpdateSonarSensor(Ultrasonic * pSensor);

    // Autonomous wait for something to complete delay routine
    inline void AutonomousDelay(float time);

    // Autonomous drive for a specified time
    inline void AutonomousDriveSequence(float speed, float time);

    // Routine to put things in a known state
    void InitialStateSetup();

    // Main sequence for drive motor control
    void DriveControlSequence();

    // Main sequence for picking up a ball
    void BallIntakeSequence();

    // Main sequence for lifting the robot on the tower
    void RobotClimbSequence();

    // Main sequence for LED control
    void LedSequence();

    // Main sequence for updating solenoid states
    void SolenoidSequence();

    // Main sequence for grabbing values from the sonars
    //void SonarSensorSequence();

    // Main sequence for interaction with the serial port
    void SerialPortSequence();

    // Main sequence for vision processing
    void CameraSequence();

    // Test routines for trying out experimental code
    void AutonomousTestCode();
    void OperatorTestCode();
    void MotorTest();
    void TankDrive();
    
    // MEMBER VARIABLES
    
    // User Controls
    DriverStation       *m_pDriverStation;              // Driver station object for getting selections
    Joystick            *m_pDriveJoystick;              // Drive control
    Joystick            *m_pControlJoystick;            // Robot functional control
    
    // Motors
    TalonMotorGroup     *m_pLeftDriveMotor;             // Left drive motor control
    TalonMotorGroup     *m_pRightDriveMotor;            // Right drive motor control
    TalonMotorGroup     *m_pBallLiftMotor;              // Motor to move the ball into the shooter
    TalonMotorGroup     *m_pRobotClimbMotor;            // Motor to scale the tower with
    CANTalon            *m_pBallIntakeMotor;            // Ball pickup motor
    
    // Spike Relays
    Relay               *m_pLedRelay;                   // Controls whether or not the LEDs are lit up
    
    // Digital I/O
    DigitalInput        *m_pAutonomousRoutine1Switch;   // Switch to select autonomous routine 1
    DigitalInput        *m_pAutonomousRoutine2Switch;   // Switch to select autonomous routine 2
    DigitalInput        *m_pAutonomousRoutine3Switch;   // Switch to select autonomous routine 3
    DigitalInput        *m_pLiftLowerLimitSwitch;       // Limit switch to make sure ball lift doesn't go down too far
    DigitalInput        *m_pLiftUpperLimitSwitch;       // Limit switch to make sure ball lift doesn't come up too much
    
    // Solenoids
    // Note: no compressor related objects required,
    // instantiating a solenoid gets that for us.
    DoubleSolenoid      *m_pBallLaunchSolenoid;         // Solenoid for controlling ball launching
    DoubleSolenoid      *m_pExtraBallLaunchSolenoid;    // Solenoid for giving a little extra power to the shooter
    DoubleSolenoid      *m_pClimbPoleRaiseSolenoid;     // Solenoid for raising/lowering the climbing hook pole
    DoubleSolenoid      *m_pClimbPoleHookSolenoid;      // Solenoid for moving the climbing pole hook towards/from the bar
    
    // Servos
    // (none)
    
    // Encoders
    // (none)
    
    // Timers
    Timer               *m_pAutonomousTimer;            // Time things during autonomous
    Timer               *m_pSafetyTimer;                // Fail safe in case critical operations don't complete
    Timer               *m_pCameraRunTimer;             // Keep track of how often to do camera intense code runs
    Timer               *m_pSolenoidRetractTimer;       // Timer to automatically retract the ball shooter
    
    // Accelerometer
    BuiltInAccelerometer*m_pAccelerometer;              // Built in roborio accelerometer

    // Camera
    RobotCamera         *m_pCameras;                    // Camera object

    // Serial port configuration
    static const int     SERIAL_PORT_BUFFER_SIZE_BYTES          = 1024;
    static const int     SERIAL_PORT_NUM_DATA_BITS              = 8;
    static const int     SERIAL_PORT_BAUD_RATE                  = 115200;
    static const int     ASCII_0_OFFSET                         = 48;
    const char *         SERIAL_PORT_PACKET_HEADER              = "Frc120";
    const int            SERIAL_PORT_PACKET_HEADER_SIZE_BYTES   = 6;
    char                 m_serialPortBuffer[SERIAL_PORT_BUFFER_SIZE_BYTES];

    // On board serial port
    SerialPort          *m_pSerialPort;

    // Misc
    bool                 m_bDriveSwap;                  // Allow the user to push a button to change forward/reverse
    bool                 m_bShotInProgress;             // Keep track of if we are shooting
    bool                 m_bPoleRaised;                 // Keep track of if we are starting to climbing
    bool                 m_bHookLatched;                // Keep track of if we are ending the climb
    bool                 m_bTargetInRange;              // Indicates if we are in the prime area for trying a shot
    
    // CONSTS
    
    // Autonomous Mode Constants
    // Note: Only enable one autonomous routine!
    // Note: Autonomous routines currently controlled by
    // physical switches on the robot.
    static const bool       AUTONOMOUS_ROUTINE_1                = false;
    static const bool       AUTONOMOUS_ROUTINE_2                = false;
    static const bool       AUTONOMOUS_ROUTINE_3                = false;
    static const bool       AUTONOMOUS_TEST_ENABLED             = false;

    // Autonomous routine 1/2 constants
    static constexpr float  AUTO_DRIVE_SPEED_SLOW               = -.5F;
    static constexpr float  AUTO_DRIVE_SPEED_FAST               = -.75F;
    static constexpr float  AUTO_DRIVE_TO_SHOOT_SPEED           = .5F;
    static constexpr float  AUTO_DRIVE_DELAY_TIME_SLOW          = 3.5F;
    static constexpr float  AUTO_DRIVE_DELAY_TIME_FAST          = 5.0F;
    static constexpr float  AUTO_DRIVE_TO_SHOOT_DELAY           = 1.0F;
    static constexpr float  AUTO_INCH_FORWARD_SPEED             = -.3F;
    static constexpr float  AUTO_INCH_FORWARD_DELAY             = 1.0F;
    static constexpr float  AUTO_BALL_LIFT_SPEED                = .5F;
    static constexpr float  AUTO_CLIMB_ARM_LOWER_DELAY          = .5F;
    static constexpr float  AUTO_1_RIGHT_TURN_SPEED             = .65F;
    static constexpr float  AUTO_1_RIGHT_TURN_DELAY             = .25F;
    static constexpr float  AUTO_2_START_DELAY                  = 5.0F;
    
    // Joysticks/Buttons
    static const int        DRIVE_JOYSTICK                      = 0;
    static const int        CONTROL_JOYSTICK                    = 1;

    // Driver buttons
    static const int        BALL_SHOOT_BUTTON                   = 1;
    static const int        CAMERA_UP_BUTTON                    = 2;
    static const int        CAMERA_DOWN_BUTTON                  = 4;
    static const int        DRIVE_CONTROLS_FORWARD_BUTTON       = 6;
    static const int        DRIVE_CONTROLS_REVERSE_BUTTON       = 7;
    //static const int        BALL_INTAKE_LIFT_AUTO_DOWN_BUTTON   = 8;
    //static const int        BALL_INTAKE_LIFT_AUTO_DRIVE_BUTTON  = 9;
    //static const int        BALL_INTAKE_LIFT_AUTO_UP_BUTTON     = 10;
    
    // Control buttons
    static const int        BALL_INTAKE_FORWARD_BUTTON          = 1;
    static const int        BALL_INTAKE_REVERSE_BUTTON          = 2;
    static const int        BALL_INTAKE_LIFT_DOWN_BUTTON        = 3;
    static const int        BALL_INTAKE_LIFT_UP_BUTTON          = 5;
    static const int        ROBOT_CLIMB_POLE_RAISE_BUTTON       = 6;
    static const int        ROBOT_CLIMB_POLE_LOWER_BUTTON       = 7;
    static const int        ROBOT_CLIMB_HOOK_OUT_BUTTON         = 8;
    static const int        ROBOT_CLIMB_HOOK_IN_BUTTON          = 9;
    static const int        ROBOT_CLIMB_UP_BUTTON               = 12;
    static const int        ROBOT_CLIMB_DOWN_BUTTON             = 11;
    static const int        ESTOP_BUTTON                        = 14;

    // CAN Signals
    static const int        LEFT_MOTORS_CAN_START_ID            = 1;
    static const int        RIGHT_MOTORS_CAN_START_ID           = 3;
    static const int        BALL_LIFT_CAN_START_ID              = 5;
    static const int        BALL_INTAKE_CAN_ID                  = 7;
    static const int        ROBOT_CLIMB_CAN_START_ID            = 9;

    // PWM Signals
    // (none)
    
    // Relays
    static const int        LED_RELAY_ID                        = 0;
    
    // Digital I/O Signals
    static const int        BALL_LIFT_LOWER_LIMIT_SWITCH        = 0;
    static const int        BALL_LIFT_UPPER_LIMIT_SWITCH        = 5;
    static const int        AUTONOMOUS_ROUTINE_1_SWITCH         = 7;
    static const int        AUTONOMOUS_ROUTINE_2_SWITCH         = 8;
    static const int        AUTONOMOUS_ROUTINE_3_SWITCH         = 9;
    
    // Solenoid Signals
    static const int        BALL_LAUNCH_FORWARD_SOLENOID        = 0;
    static const int        BALL_LAUNCH_REVERSE_SOLENOID        = 1;
    static const int        EXTRA_BALL_LAUNCH_FORWARD_SOLENOID  = 2;
    static const int        EXTRA_BALL_LAUNCH_REVERSE_SOLENOID  = 3;
    static const int        CLIMB_POLE_RAISE_FORWARD_SOLENOID   = 4;
    static const int        CLIMB_POLE_RAISE_REVERSE_SOLENOID   = 5;
    static const int        CLIMB_POLE_HOOK_FORWARD_SOLENOID    = 6;
    static const int        CLIMB_POLE_HOOK_REVERSE_SOLENOID    = 7;
    
    // Misc
    static const int        OFF                                 = 0;
    static const int        ON                                  = 1;
    static const int        NUMBER_OF_LEFT_DRIVE_MOTORS         = 2;
    static const int        NUMBER_OF_RIGHT_DRIVE_MOTORS        = 2;
    static const int        NUMBER_OF_BALL_LIFT_MOTORS          = 2;
    static const int        NUMBER_OF_ROBOT_CLIMB_MOTORS        = 2;
    static const int        BALL_LIFT_UP_POSITION               = 0;
    static const int        BALL_LIFT_DRIVE_POSITION            = 1500;
    static const int        BALL_LIFT_DOWN_POSITION             = 3000;
    static const char       NULL_CHARACTER                      = '\0';
    static constexpr float  DRIVE_MOTOR_UPPER_LIMIT             = 1.0F;
    static constexpr float  DRIVE_MOTOR_LOWER_LIMIT             = -1.0F;
    static constexpr float  JOYSTICK_TRIM_UPPER_LIMIT           = .1F;
    static constexpr float  JOYSTICK_TRIM_LOWER_LIMIT           = -.1F;
    static constexpr float  THROTTLE_VALUE_RANGE                = .65F;
    static constexpr float  THROTTLE_VALUE_BASE                 = .35F;
    static constexpr float  SAFETY_TIMER_MAX_VALUE              = 5.0F;
    static constexpr float  CAMERA_RUN_INTERVAL_S               = 1.0F;
    static constexpr float  SOLENOID_RETRACT_TIME               = 1.0F;
    static constexpr float  CLIMB_SOLENOID_DELAY_TIME           = .5F;
    static constexpr float  BALL_LIFT_MAX_OUTPUT                = 0.55F;//0.70F;
    
};  // End class



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousDelay
///
/// Waits for a specified amount of time in autonomous.  Used
/// while an operation is ongoing but not yet complete, and
/// nothing else needs to occur.
///
////////////////////////////////////////////////////////////////
inline void CmsdRobot::AutonomousDelay(float time)
{
    m_pAutonomousTimer->Start();
    while (m_pAutonomousTimer->Get() < time) {}
    m_pAutonomousTimer->Stop();
    m_pAutonomousTimer->Reset();
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousDriveSequence
///
/// Drives during autonomous for a specified amount of time.
///
////////////////////////////////////////////////////////////////
inline void CmsdRobot::AutonomousDriveSequence(float speed, float time)
{
    // First turn the motors on
    m_pLeftDriveMotor->Set(speed);
    m_pRightDriveMotor->Set(-speed);

    // Time it
    AutonomousDelay(time);

    // Motors back off
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::CheckForDriveSwap
///
/// Updates the drive control direction.
///
////////////////////////////////////////////////////////////////
inline void CmsdRobot::CheckForDriveSwap()
{
    // Check if the driver pushed the button to have
    // forward be reverse and vice versa
    if ( m_pDriveJoystick->GetRawButton(DRIVE_CONTROLS_FORWARD_BUTTON) )
    {
        m_bDriveSwap = false;
        
        // Switch to the forward cam
        //m_pCameras->SetCamera(CameraType::AXIS);
    }
    else if ( m_pDriveJoystick->GetRawButton(DRIVE_CONTROLS_REVERSE_BUTTON) )
    {
        m_bDriveSwap = true;
        
        // Switch to the reverse cam
        //m_pCameras->SetCamera(CameraType::USB);
    }
    else
    {
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::GetThrottleControl
///
/// Returns a throttle value based on input from the joystick.
///
////////////////////////////////////////////////////////////////
inline float CmsdRobot::GetThrottleControl(Joystick * pJoystick)
{
    // Get throttle control
    // The z axis goes from -1 to 1, so it needs to be normalized.
    // Subtract one and negate to make it zero based to give a value
    // between zero and two.  This will be used to scale the voltage
    // to the motors.  It essentially computes the max speed value
    // that can be reached.    
    return ((pJoystick->GetThrottle() - 1.0F) / -2.0F) * THROTTLE_VALUE_RANGE + THROTTLE_VALUE_BASE;
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::UpdateSonarSensor
///
/// This method is used to get a value from the sonar sensor.
/// It is intended to be used to turn a sensor briefly on and
/// get a reading from it so as to not interfere with other
/// sensors that may need to get readings.
///
////////////////////////////////////////////////////////////////
/*inline float CmsdRobot::UpdateSonarSensor(Ultrasonic * pSensor)
{
    pSensor->SetEnabled(true);
    float sensorValue = pSensor->GetRangeInches();
    pSensor->SetEnabled(false);
    return sensorValue;
}
*/


////////////////////////////////////////////////////////////////
// @method CmsdRobot::Limit
///
/// This method is used to prevent a value outside the range
/// specified by upper and lower from being sent to physical
/// devices.
///
////////////////////////////////////////////////////////////////
inline float CmsdRobot::Limit( float num, float upper, float lower )
{
    if ( num > upper )
    {
        return upper;
    }
    else if ( num < lower )
    {
        return lower;
    }

    return num;
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::Trim
///
/// This method is used to ensure a signal value is above a
/// certain threshold to ensure there is actual input to a
/// device and not just noise/jitter.
///
////////////////////////////////////////////////////////////////
inline float CmsdRobot::Trim( float num, float upper, float lower )
{
    if ( (num < upper) && (num > lower) )
    {
        return 0;
    }
    
    return num;
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::DetectTriggerChange
///
/// This method is used to check if a button has undergone a
/// state change.  The same button can be used to reverse state
/// of a particular part of the robot (such as a motor or
/// solenoid).  If the state is reversed inside an 'if'
/// statement that is part of a loop, the final state will be
/// whatever transition just occurred, which could be back to
/// the same state started in.  The intended use case is to
/// have TriggerChangeValues variables in the code and update
/// their 'current' value each time through the loop by reading
/// the joystick input.  This input will then be checked against
/// the old input and return 'true' if it detects the button has
/// been pressed.  This method is intended to be called inside
/// 'if' statements for logic control.
///
////////////////////////////////////////////////////////////////
inline bool CmsdRobot::DetectTriggerChange(TriggerChangeValues * pTriggerVals)
{
    // Only report a change if the current value is different than the old value
    // Also make sure the transition is to being pressed since we are detecting
    // presses and not releases
    if ( (pTriggerVals->bCurrentValue != pTriggerVals->bOldValue) && pTriggerVals->bCurrentValue )
    {
        // Update the old value, return the button was pressed
        pTriggerVals->bOldValue = pTriggerVals->bCurrentValue;
        return true;
    }
    
    // Otherwise update the old value
    pTriggerVals->bOldValue = pTriggerVals->bCurrentValue;
    return false;
}

#endif // CMSDROBOT_HPP
