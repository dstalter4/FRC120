////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobot.hpp
/// @author David Stalter
///
/// @details
/// This is the class declaration for a FRC robot derived from the WPI library
/// base classes.  The TimedRobot class is the base of a robot application that
/// will automatically call appropriate Autonomous and Teleop methods at the
/// right time as controlled by the switches on the driver station or the field
/// controls.
///
/// Copyright (c) 2023 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTAROBOT_HPP
#define YTAROBOT_HPP

// SYSTEM INCLUDES
#include <cmath>                                // for M_PI
#include <thread>                               // for std::thread

// C INCLUDES
#include "ctre/phoenix/led/CANdle.h"            // for interacting with the CANdle
#include "ctre/phoenix/led/RainbowAnimation.h"  // for interacting with the CANdle
#include "ctre/phoenix/sensors/Pigeon2.h"       // for PigeonIMU
#include "frc/ADXRS450_Gyro.h"                  // for using the SPI port FRC gyro
#include "frc/AnalogGyro.h"                     // for using analog gyros
#include "frc/BuiltInAccelerometer.h"           // for using the built-in accelerometer
#include "frc/Compressor.h"                     // for retrieving info on the compressor
#include "frc/DigitalInput.h"                   // for DigitalInput type
#include "frc/DigitalOutput.h"                  // for DigitalOutput type
#include "frc/DoubleSolenoid.h"                 // for DoubleSolenoid type
#include "frc/DriverStation.h"                  // for interacting with the driver station
#include "frc/Relay.h"                          // for Relay type
#include "frc/SerialPort.h"                     // for interacting with a serial port
#include "frc/Solenoid.h"                       // for Solenoid type
#include "frc/TimedRobot.h"                     // for base class decalartion
#include "frc/Ultrasonic.h"                     // for Ultrasonic type
#include "frc/livewindow/LiveWindow.h"          // for controlling the LiveWindow
#include "frc/smartdashboard/SendableChooser.h" // for using the smart dashboard sendable chooser functionality
#include "frc/smartdashboard/SmartDashboard.h"  // for interacting with the smart dashboard

// C++ INCLUDES
#include "DriveConfiguration.hpp"               // for information on the drive config
#include "RobotI2c.hpp"                         // for GetGyroData()
#include "RobotUtils.hpp"                       // for ASSERT, DEBUG_PRINTS
#include "SwerveDrive.hpp"                      // for using swerve drive
#include "TalonMotorGroup.hpp"                  // for Talon group motor control
#include "YtaController.hpp"                    // for controller interaction

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class YtaRobot
///
/// Derived class from TimedRobot.  The object that will
/// control all robot functionality.
///
////////////////////////////////////////////////////////////////
class YtaRobot : public TimedRobot
{
public:
    friend class RobotCamera;
    friend class YtaRobotTest;

    // MEMBER FUNCTIONS
    
    // Base robot routines
    virtual void RobotInit() override;
    virtual void RobotPeriodic() override;
    
    // Autonomous routines
    virtual void AutonomousInit() override;
    virtual void AutonomousPeriodic() override;
    
    // Teleop routines
    virtual void TeleopInit() override;
    virtual void TeleopPeriodic() override;
    
    // Test mode routines
    virtual void TestInit() override;
    virtual void TestPeriodic() override;
    
    // Robot disabled routines
    virtual void DisabledInit() override;
    virtual void DisabledPeriodic() override;
    
    // Constructor, destructor, copy, assignment
    YtaRobot();
    virtual ~YtaRobot() = default;
    YtaRobot(YtaRobot&& rhs) = default;
    YtaRobot& operator=(YtaRobot&& rhs) = default;
      
private:

    // TYPEDEFS
    typedef YtaTalon::MotorGroupControlMode MotorGroupControlMode;
    typedef Yta::Controller::Config::Models ControllerModels;
    typedef Yta::Controller::Config::Mappings ControllerMappings;
    typedef YtaDriveController<YtaCustomController> DriveControllerType;
    typedef YtaController<YtaCustomController> AuxControllerType;
    
    // ENUMS
    enum RobotMode
    {
        ROBOT_MODE_AUTONOMOUS,
        ROBOT_MODE_TELEOP,
        ROBOT_MODE_TEST,
        ROBOT_MODE_DISABLED,
        ROBOT_MODE_NOT_SET
    };
    
    enum DriveState
    {
        MANUAL_CONTROL,
        DIRECTIONAL_INCH,
        DIRECTIONAL_ALIGN
    };
    
    enum RobotDirection
    {
        ROBOT_NO_DIRECTION,
        ROBOT_FORWARD,
        ROBOT_REVERSE,
        ROBOT_LEFT,
        ROBOT_RIGHT
    };
    
    enum RobotRotate
    {
        ROBOT_NO_ROTATE,
        ROBOT_CLOCKWISE,
        ROBOT_COUNTER_CLOCKWISE
    };

    enum GyroType
    {
        ADXRS450,
        ANALOG,
        BNO055
    };
    
    // STRUCTS
    struct LedColors
    {
        int m_Red;
        int m_Green;
        int m_Blue;
        int m_White;
    };
    
    // This is a hacky way of retrieving a pointer to the robot object
    // outside of the robot class.  The robot object itself is a static
    // variable inside the function StartRobot() in the RobotBase class.
    // This makes retrieving the address difficult.  To work around this,
    // we'll allocate some static storage for a pointer to a robot object.
    // When RobotInit() is called, m_pThis will be filled out.  This works
    // because only one YtaRobot object is ever constructed.
    static YtaRobot * m_pThis;
    inline void SetStaticThisInstance() { m_pThis = this; }
    inline static YtaRobot * GetRobotInstance() { return m_pThis; }

    // Increments a variable to indicate the robot code is successfully running
    inline void HeartBeat();
    
    // Checks for a robot state change and logs a message if so
    inline void CheckAndUpdateRobotMode(RobotMode robotMode);

    // Updates information on the smart dashboard for the drive team
    void UpdateSmartDashboard();

    // Grabs a value from a sonar sensor individually
    inline double GetSonarValue(Ultrasonic * pSensor);
   
    // Get a reading from the gyro sensor
    inline double GetGyroValue(GyroType gyroType, AnalogGyro * pSensor = nullptr);

    // Autonomous wait for something to complete delay routine
    inline void AutonomousDelay(units::second_t time);

    // Autonomous drive for a specified time
    inline void AutonomousDriveSequence(RobotDirection direction, double speed, units::second_t time);
    inline void AutonomousSwerveDriveSequence(RobotDirection direction, RobotRotate rotate, double speed, double rotateSpeed, units::second_t time, bool bFieldRelative);
    
    // Autonomous routines to back drive the motors to abruptly stop
    inline void AutonomousBackDrive(RobotDirection currentDirection);
    inline void AutonomousBackDriveTurn(RobotDirection currentDirection);
    
    // Autonomous routines
    // @todo: Make YtaRobotAutonomous a friend and move these out (requires accessor to *this)!
    void AutonomousRoutine1();
    void AutonomousRoutine2();
    void AutonomousRoutine3();
    void AutonomousTestRoutine();
    void AutonomousTestSwerveRoutine();
    void AutonomousTestTrajectoryRoutine();
    void AutonomousCommon();
    void AutonomousCommonRed();
    void AutonomousCommonBlue();

    // Resets member variables
    void ResetMemberData();

    // Routine to put things in a known state
    void InitialStateSetup();

    // Configure motor controller parameters
    void ConfigureMotorControllers();

    // Main sequence for drive motor control
    void SwerveDriveSequence();
    void DriveControlSequence();
    void SideDriveSequence();

    // Function to check for drive control direction swap
    inline void CheckForDriveSwap();
    
    // Function to automate slightly moving the robot
    bool DirectionalInch();
    
    // Function to automatically align the robot to a certain point
    void DirectionalAlign();

    // Function to periodically cool the drive talons
    void DriveMotorsCool();

    // Main sequence for LED control
    void LedSequence();
    inline void SetLedsToAllianceColor();
    void MarioKartLights(double translation, double strafe, double rotate);

    // Main sequence for controlling pneumatics
    void PneumaticSequence();

    // Main sequence for interaction with the serial port
    void SerialPortSequence();
    
    // Main sequence for I2C interaction
    void I2cSequence();
    
    // Main sequence for vision processing
    void CameraSequence();

    // Check for a request to reset encoder counts
    void CheckAndResetEncoderCounts();

    // Superstructure control testing sequence
    void SuperStructureTestSequence();
    
    // MEMBER VARIABLES
    
    // Autonomous
    SendableChooser<std::string>    m_AutonomousChooser;                    // Selects from the dashboard which auto routine to run
    
    // User Controls
    DriveControllerType *           m_pDriveController;                     // Drive controller
    AuxControllerType *             m_pAuxController;                       // Auxillary input controller
    
    // Swerve Drive
    Pigeon2 *                       m_pPigeon;                              // CTRE Pigeon2 IMU
    SwerveDrive *                   m_pSwerveDrive;                         // Swerve drive control
    
    // Motors
    TalonMotorGroup<TalonFX> *      m_pLeftDriveMotors;                     // Left drive motor control
    TalonMotorGroup<TalonFX> *      m_pRightDriveMotors;                    // Right drive motor control
    TalonMotorGroup<TalonFX> *      m_pCarriageMotors;                      // Carriage motor control
    TalonFX *                       m_pIntakeMotor;                         // Intake motor control
    
    // LEDs
    CANdle *                        m_pCandle;                              // Controls an RGB LED strip
    RainbowAnimation                m_RainbowAnimation;                     // Rainbow animation configuration (brightness, speed, # LEDs)

    // Interrupts
    // (none)

    // Digital I/O
    DigitalOutput *                 m_pDebugOutput;                         // Debug assist output
    
    // Analog I/O
    // (none)
    
    // Pneumatics
    DoubleSolenoid *                m_pTalonCoolingSolenoid;                // Controls the solenoid for cooling the drive talons
    Compressor *                    m_pCompressor;                          // Object to get info about the compressor
    
    // Servos
    // (none)
    
    // Encoders
    // (none)
    
    // Timers
    Timer *                         m_pMatchModeTimer;                      // Times how long a particular mode (autonomous, teleop) is running
    Timer *                         m_pSafetyTimer;                         // Fail safe in case critical operations don't complete
    
    // Accelerometer
    BuiltInAccelerometer *          m_pAccelerometer;                       // Built in roborio accelerometer
    
    // Gyro
    ADXRS450_Gyro *                 m_pAdxrs450Gyro;                        // SPI port FRC gyro
    int                             m_Bno055Angle;                          // Angle from the BNO055 sensor on the RIOduino

    // Camera
    // Note: Only need to have a thread here and tie it to
    // the RobotCamera class, which handles everything else.
    std::thread                     m_CameraThread;

    // Serial port configuration
    static const int                SERIAL_PORT_BUFFER_SIZE_BYTES           = 64;
    static const int                SERIAL_PORT_NUM_DATA_BITS               = 8;
    static const int                SERIAL_PORT_BAUD_RATE                   = 115200;
    static const int                ASCII_0_OFFSET                          = 48;
    const char *                    SERIAL_PORT_PACKET_HEADER               = "Frc120Serial";
    const int                       SERIAL_PORT_PACKET_HEADER_SIZE_BYTES    = sizeof(SERIAL_PORT_PACKET_HEADER);
    char                            m_SerialPortBuffer[SERIAL_PORT_BUFFER_SIZE_BYTES];

    // On board serial port
    SerialPort *                    m_pSerialPort;
    
    // I2C configuration
    std::thread                     m_I2cThread;
    
    // Misc
    RobotMode                       m_RobotMode;                            // Keep track of the current robot state
    DriveState                      m_RobotDriveState;                      // Keep track of how the drive sequence flows
    DriverStation::Alliance         m_AllianceColor;                        // Color reported by driver station during a match
    bool                            m_bDriveSwap;                           // Allow the user to push a button to change forward/reverse
    uint32_t                        m_HeartBeat;                            // Incremental counter to indicate the robot code is executing
    
    // CONSTS
    
    // Joysticks/Buttons
    // Note: Don't forget to update the controller object typedefs if
    //       necessary when changing these types!
    static const ControllerModels DRIVE_CONTROLLER_MODEL                        = ControllerModels::CUSTOM_XBOX;
    static const ControllerModels AUX_CONTROLLER_MODEL                          = ControllerModels::CUSTOM_XBOX;
    static constexpr const ControllerMappings * const DRIVE_CONTROLLER_MAPPINGS = Yta::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL);
    static constexpr const ControllerMappings * const AUX_CONTROLLER_MAPPINGS   = Yta::Controller::Config::GetControllerMapping(AUX_CONTROLLER_MODEL);
    
    static const int                DRIVE_JOYSTICK_PORT                     = 0;
    static const int                AUX_JOYSTICK_PORT                       = 1;

    // Driver inputs
    static const int                DRIVE_SLOW_X_AXIS                       = DRIVE_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.RIGHT_X_AXIS;
    static const int                DRIVE_SLOW_Y_AXIS                       = DRIVE_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.RIGHT_Y_AXIS;

    static const int                FIELD_RELATIVE_TOGGLE_BUTTON            = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUMPER;
    static const int                ZERO_GYRO_YAW_BUTTON                    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUMPER;
    static const int                CAMERA_TOGGLE_FULL_PROCESSING_BUTTON    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                CAMERA_TOGGLE_PROCESSED_IMAGE_BUTTON    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                SELECT_FRONT_CAMERA_BUTTON              = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                SELECT_BACK_CAMERA_BUTTON               = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                DRIVE_SWAP_BUTTON                       = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                DRIVE_CONTROLS_INCH_FORWARD_BUTTON      = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                DRIVE_CONTROLS_INCH_REVERSE_BUTTON      = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                DRIVE_CONTROLS_INCH_LEFT_BUTTON         = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                DRIVE_CONTROLS_INCH_RIGHT_BUTTON        = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    
    // Aux inputs
    static const int                AUX_TOGGLE_LEDS_BUTTON                  = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.DOWN_BUTTON;
    static const int                ESTOP_BUTTON                            = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;

    // CAN Signals
    // Note: The use of high CAN values if swerve drive is in use is
    //       to prevent instantiating multiple motor controllers with
    //       the same IDs, but still allow code for both drive base
    //       types to be present.  When using swerve drive, IDs 1-8
    //       are used by the swerve modules (see the SwerveModuleConfigs
    //       in SwerveDrive.hpp).
    static const unsigned           LEFT_DRIVE_MOTORS_CAN_START_ID          = Yta::Drive::Config::USE_SWERVE_DRIVE ? 64 : 1;
    static const unsigned           RIGHT_DRIVE_MOTORS_CAN_START_ID         = Yta::Drive::Config::USE_SWERVE_DRIVE ? 66 : 3;

    // CANivore Signals
    // Note: IDs 1-4 are used by the CANcoders (see the
    //       SwerveModuleConfigs in SwerveDrive.hpp).
    static const int                PIGEON_CAN_ID                           = 5;
    static const int                CANDLE_CAN_ID                           = 6;

    // PWM Signals
    // (none)
    
    // Relays
    // (none)
    
    // Digital I/O Signals
    static const int                DEBUG_OUTPUT_DIO_CHANNEL                = 7;
    
    // Analog I/O Signals
    // (none)
    
    // Solenoid Signals
    static const int                TALON_COOLING_SOLENOID_FWD_CHANNEL      = 6;
    static const int                TALON_COOLING_SOLENOID_REV_CHANNEL      = 7;

    // Solenoids
    static const DoubleSolenoid::Value  TALON_COOLING_ON_SOLENOID_VALUE     = DoubleSolenoid::kReverse;
    static const DoubleSolenoid::Value  TALON_COOLING_OFF_SOLENOID_VALUE    = DoubleSolenoid::kForward;
    
    // Misc
    const std::string               AUTO_ROUTINE_1_STRING                   = "Autonomous Routine 1";
    const std::string               AUTO_ROUTINE_2_STRING                   = "Autonomous Routine 2";
    const std::string               AUTO_ROUTINE_3_STRING                   = "Autonomous Routine 3";
    const std::string               AUTO_TEST_ROUTINE_STRING                = "Autonomous Test Routine";

    static const int                OFF                                     = 0;
    static const int                ON                                      = 1;
    static const int                ANGLE_90_DEGREES                        = 90;
    static const int                ANGLE_180_DEGREES                       = 180;
    static const int                ANGLE_360_DEGREES                       = 360;
    static const int                POV_INPUT_TOLERANCE_VALUE               = 30;
    static const int                SCALE_TO_PERCENT                        = 100;
    static const int                QUADRATURE_ENCODING_ROTATIONS           = 4096;
    static const unsigned           SINGLE_MOTOR                            = 1;
    static const unsigned           TWO_MOTORS                              = 2;
    static const unsigned           NUMBER_OF_LEFT_DRIVE_MOTORS             = 2;
    static const unsigned           NUMBER_OF_RIGHT_DRIVE_MOTORS            = 2;
    static const unsigned           NUMBER_OF_LEDS                          = 8;
    static const char               NULL_CHARACTER                          = '\0';
    static const bool               ADXRS450_GYRO_PRESENT                   = false;

    static const unsigned           CAMERA_RUN_INTERVAL_MS                  = 1000U;
    static const unsigned           I2C_RUN_INTERVAL_MS                     = 240U;
    
    static constexpr double         JOYSTICK_TRIM_UPPER_LIMIT               =  0.05;
    static constexpr double         JOYSTICK_TRIM_LOWER_LIMIT               = -0.05;
    static constexpr double         DRIVE_THROTTLE_VALUE_RANGE              =  1.00;
    static constexpr double         DRIVE_THROTTLE_VALUE_BASE               =  0.00;
    static constexpr double         DRIVE_SLOW_THROTTLE_VALUE               =  0.35;
    static constexpr double         SWERVE_ROTATE_SLOW_JOYSTICK_THRESHOLD   =  0.10;
    static constexpr double         SWERVE_ROTATE_SLOW_SPEED                =  0.10;
    static constexpr double         DRIVE_MOTOR_UPPER_LIMIT                 =  1.00;
    static constexpr double         DRIVE_MOTOR_LOWER_LIMIT                 = -1.00;
    static constexpr double         FALCON_ENCODER_COUNTS_PER_ROTATION      =  2048.0;

    static constexpr units::second_t    SAFETY_TIMER_MAX_VALUE_S            =  5.00_s;



    // These indicate which motor value (+1/-1) represent
    // forward/reverse in the robot.  They are used to keep
    // autonomous movement code common without yearly updates.

    static constexpr double         LEFT_DRIVE_FORWARD_SCALAR               = -1.00;
    static constexpr double         LEFT_DRIVE_REVERSE_SCALAR               = +1.00;
    static constexpr double         RIGHT_DRIVE_FORWARD_SCALAR              = +1.00;
    static constexpr double         RIGHT_DRIVE_REVERSE_SCALAR              = -1.00;

    ////////////////////////////////////////////////////////////////
    // Inputs from joystick:
    //
    // Forward:     (0, -1)
    // Reverse:     (0, +1)
    // Left:        (-1, 0)
    // Right:       (+1, 0)
    //
    // Equations:
    //
    //     x+y   x-y   -x+y   -x-y
    // F:   -1    +1     -1     +1
    // B:   +1    -1     +1     -1
    // L:   -1    -1     +1     +1
    // R:   +1    +1     -1     -1
    //
    // Output to motors:
    //
    // Left forward/right = +1, Right forward/left  = +1:
    // Left reverse/left  = -1, Right reverse/right = -1:
    // x-y, -x-y
    //
    // Left forward/right = -1, Right forward/left  = -1:
    // Left reverse/left  = +1, Right reverse/right = +1:
    // -x+y, x+y
    //
    // Left forward/right = +1, Right forward/left  = -1:
    // Left reverse/left  = -1, Right reverse/right = +1:
    // x-y, x+y
    //
    // Left forward/right = -1, Right forward/left  = +1:
    // Left reverse/left  = +1, Right reverse/right = -1:
    // -x+y, -x-y
    ////////////////////////////////////////////////////////////////

    inline static constexpr double LeftDriveEquation(double xInput, double yInput)
    {
        double leftValue = 0.0;

        if (static_cast<int>(LEFT_DRIVE_FORWARD_SCALAR) == 1)
        {
            leftValue = xInput - yInput;
        }
        else
        {
            leftValue = -xInput + yInput;
        }
        
        return leftValue;
    }

    inline static constexpr double RightDriveEquation(double xInput, double yInput)
    {
        double rightValue = 0.0;

        if (static_cast<int>(RIGHT_DRIVE_FORWARD_SCALAR) == 1)
        {
            rightValue = -xInput - yInput;
        }
        else
        {
            rightValue = xInput + yInput;
        }
        
        return rightValue;
    }

};  // End class



////////////////////////////////////////////////////////////////
/// @method YtaRobot::HeartBeat
///
/// Increments the heartbeat counter.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::HeartBeat()
{
    m_HeartBeat++;
    SmartDashboard::PutNumber("Heartbeat", m_HeartBeat);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CheckForDriveSwap
///
/// Updates the drive control direction.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::CheckForDriveSwap()
{
    // Check if the driver pushed the button to have
    // forward be reverse and vice versa
    if (m_pDriveController->DetectButtonChange(DRIVE_SWAP_BUTTON))
    {
        m_bDriveSwap = !m_bDriveSwap;
        SmartDashboard::PutBoolean("Drive swap", m_bDriveSwap);
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::SetLedsToAllianceColor
///
/// Sets the LEDs to the alliance color.
///
////////////////////////////////////////////////////////////////
void YtaRobot::SetLedsToAllianceColor()
{
    switch (m_AllianceColor)
    {
        case DriverStation::Alliance::kRed:
        {
            m_pCandle->SetLEDs(255, 0, 0, 0, 0, NUMBER_OF_LEDS);
            break;
        }
        case DriverStation::Alliance::kBlue:
        {
            m_pCandle->SetLEDs(0, 0, 255, 0, 0, NUMBER_OF_LEDS);
            break;
        }
        default:
        {
            break;
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::GetGyroValue
///
/// This method is used to get a value from an analog gyro
/// sensor.  There are three possible places a gyro could be
/// connected: analog sensor, the on board SPI port (ADXRS450),
/// or externally (BNO055 on a RIOduino).  This method will
/// obtain a value from the sensor corresponding to the passed
/// in parameter.
///
////////////////////////////////////////////////////////////////
inline double YtaRobot::GetGyroValue(GyroType gyroType, AnalogGyro * pSensor)
{
    double value = 0.0;
    
    switch (gyroType)
    {
        case ADXRS450:
        {
            if (m_pAdxrs450Gyro != nullptr)
            {
                value = m_pAdxrs450Gyro->GetAngle();
            }
            break;
        }
        case ANALOG:
        {
            if (pSensor != nullptr)
            {
                value = pSensor->GetAngle();
            }
            break;
        }
        case BNO055:
        {
            // Read the angle
            GyroI2cData * pGyroData = RobotI2c::GetGyroData();
            
            // Only update the value if valid data came across the wire
            if (pGyroData != nullptr)
            {
                m_Bno055Angle = pGyroData->m_xAxisInfo.m_Angle;
                
                // Reapply negative sign if needed
                if (pGyroData->m_xAxisInfo.m_bIsNegative)
                {
                    m_Bno055Angle *= -1;
                }
            }
            
            value = static_cast<double>(m_Bno055Angle);
            
            break;
        }
        default:
        {
            // Should never happen
            ASSERT(false);
            break;
        }
    }
    
    if (RobotUtils::DEBUG_PRINTS)
    {
        SmartDashboard::PutNumber("Gyro angle", value);
    }
    
    return value;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::GetSonarValue
///
/// This method is used to get a value from the sonar sensor.
/// It is intended to be used to turn a sensor briefly on and
/// get a reading from it so as to not interfere with other
/// sensors that may need to get readings.
///
////////////////////////////////////////////////////////////////
inline double YtaRobot::GetSonarValue(Ultrasonic * pSensor)
{
    return 0.0;
    
    /*
    pSensor->SetEnabled(true);
    double sensorValue = pSensor->GetRangeInches();
    pSensor->SetEnabled(false);
    return sensorValue;
    */
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CheckAndUpdateRobotMode
///
/// Checks the current robot mode for a state change and updates
/// accordingly, including displaying a message.
///
////////////////////////////////////////////////////////////////
void YtaRobot::CheckAndUpdateRobotMode(RobotMode robotMode)
{
    // These array messages match the order of the RobotMode enum
    const char * MODE_CHANGE_ENTER_MESSAGES[] = 
                {
                    "Autonomous entered.",
                    "Teleop entered.",
                    "Test entered.",
                    "Disabled entered."
                };

    const char * MODE_CHANGE_EXIT_MESSAGES[] = 
                {
                    "Autonomous exited.",
                    "Teleop exited.",
                    "Test exited.",
                    "Disabled exited."
                };
    
    // Check for the mode to have changed
    if (m_RobotMode != robotMode)
    {
        // First display the exit message for the old mode
        RobotUtils::DisplayMessage(MODE_CHANGE_EXIT_MESSAGES[m_RobotMode]);

        // Enter the new mode and display an enter message
        m_RobotMode = robotMode;
        RobotUtils::DisplayMessage(MODE_CHANGE_ENTER_MESSAGES[m_RobotMode]);
    }
}

#endif // YTAROBOT_HPP
