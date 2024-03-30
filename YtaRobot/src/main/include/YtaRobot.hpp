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
/// Copyright (c) 2024 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTAROBOT_HPP
#define YTAROBOT_HPP

// SYSTEM INCLUDES
#include <cmath>                                // for M_PI
#include <thread>                               // for std::thread

// C INCLUDES
#include "frc/Compressor.h"                     // for retrieving info on the compressor
#include "frc/DigitalInput.h"                   // for DigitalInput type
#include "frc/DigitalOutput.h"                  // for DigitalOutput type
#include "frc/DoubleSolenoid.h"                 // for DoubleSolenoid type
#include "frc/DriverStation.h"                  // for interacting with the driver station
#include "frc/DutyCycleEncoder.h"               // for interacting with the rev through bore encoder
#include "frc/Relay.h"                          // for Relay type
#include "frc/Solenoid.h"                       // for Solenoid type
#include "frc/TimedRobot.h"                     // for base class decalartion
#include "frc/livewindow/LiveWindow.h"          // for controlling the LiveWindow
#include "frc/smartdashboard/SendableChooser.h" // for using the smart dashboard sendable chooser functionality
#include "frc/smartdashboard/SmartDashboard.h"  // for interacting with the smart dashboard

// C++ INCLUDES
#include "DriveConfiguration.hpp"               // for information on the drive config
#include "RobotUtils.hpp"                       // for ASSERT, DEBUG_PRINTS
#include "SwerveDrive.hpp"                      // for using swerve drive
#include "YtaController.hpp"                    // for controller interaction
#include "YtaTalon.hpp"                         // for custom Talon control
#include "ctre/phoenix/led/CANdle.h"            // for interacting with the CANdle
#include "ctre/phoenix/led/RainbowAnimation.h"  // for interacting with the CANdle
#include "ctre/phoenix6/Pigeon2.hpp"            // for PigeonIMU
#include "ctre/phoenix6/controls/MusicTone.hpp" // for creating music tones

using namespace frc;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix::led;


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
    typedef Yta::Talon::MotorGroupControlMode MotorGroupControlMode;
    typedef Yta::Talon::TalonFxMotorController TalonFxMotorController;
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

    enum RobotDriveState
    {
        MANUAL_CONTROL,
        DIRECTIONAL_INCH,
        DIRECTIONAL_ALIGN
    };
    
    enum class RobotDirection
    {
        ROBOT_NO_DIRECTION,
        ROBOT_FORWARD,
        ROBOT_REVERSE,
        ROBOT_LEFT,
        ROBOT_RIGHT
    };

    enum class RobotTranslation
    {
        ROBOT_NO_TRANSLATION,
        ROBOT_TRANSLATION_FORWARD,
        ROBOT_TRANSLATION_REVERSE
    };

    enum class RobotStrafe
    {
        ROBOT_NO_STRAFE,
        ROBOT_STRAFE_LEFT,
        ROBOT_STRAFE_RIGHT
    };

    enum class RobotRotation
    {
        ROBOT_NO_ROTATION,
        ROBOT_CLOCKWISE,
        ROBOT_COUNTER_CLOCKWISE
    };

    // STRUCTS
    struct RobotSwerveDirections
    {
      public:
        RobotSwerveDirections() : m_Translation(RobotTranslation::ROBOT_NO_TRANSLATION), m_Strafe(RobotStrafe::ROBOT_NO_STRAFE), m_Rotation(RobotRotation::ROBOT_NO_ROTATION) {}
        inline void SetSwerveDirections(RobotTranslation translationDirection, RobotStrafe strafeDirection, RobotRotation rotationDirection)
        {
            m_Translation = translationDirection;
            m_Strafe = strafeDirection;
            m_Rotation = rotationDirection;
        }
        inline RobotTranslation GetTranslation() { return m_Translation; }
        inline RobotStrafe GetStrafe() { return m_Strafe; }
        inline RobotRotation GetRotation() { return m_Rotation; }
      private:
        RobotTranslation m_Translation;
        RobotStrafe m_Strafe;
        RobotRotation m_Rotation;
    };

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

    // Autonomous wait for something to complete delay routine
    inline void AutonomousDelay(units::second_t time);

    // Autonomous drive for a specified time
    inline void AutonomousDriveSequence(RobotDirection direction, double speed, units::second_t time);
    inline void AutonomousSwerveDriveSequence(RobotSwerveDirections & rSwerveDirections, double translationSpeed, double strafeSpeed, double rotateSpeed, units::second_t time, bool bFieldRelative);
    
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

    // Function to check for drive control direction swap
    inline void CheckForDriveSwap();
    
    // Function to automate slightly moving the robot
    bool DirectionalInch();
    
    // Function to automatically align the robot to a certain point
    void DirectionalAlign();

    // Main sequence for LED control
    void LedSequence();
    inline void SetLedsToAllianceColor();
    void MarioKartLights(double translation, double strafe, double rotate);
    void BlinkMorseCodePattern();

    // Main sequence for music control
    void MusicSequence();

    // Main sequence for controlling pneumatics
    void PneumaticSequence();
    
    // Main sequence for vision processing
    void CameraSequence();

    // Check for a request to reset encoder counts
    void CheckAndResetEncoderCounts();

    // Main sequence for intake control
    void IntakeSequence();

    // Main sequence for superstructure mechanism pivot control
    void PivotSequence();
    void CheckAndUpdateShootValues();

    // Main sequence for shoot control
    void ShootSequence();

    // Main sequence for lifting the robot
    void LiftSequence();

    // Superstructure control testing sequence
    void SuperStructureTestSequence();
    
    // MEMBER VARIABLES
    
    // Autonomous
    SendableChooser<std::string>    m_AutonomousChooser;                    // Selects from the dashboard which auto routine to run
    RobotSwerveDirections           m_AutoSwerveDirections;                 // Used by autonomous routines to control swerve drive movements
    
    // User Controls
    DriveControllerType *           m_pDriveController;                     // Drive controller
    AuxControllerType *             m_pAuxController;                       // Auxillary input controller
    
    // Swerve Drive
    Pigeon2 *                       m_pPigeon;                              // CTRE Pigeon2 IMU
    SwerveDrive *                   m_pSwerveDrive;                         // Swerve drive control
    
    // Motors
    typedef Yta::Talon::EmptyTalonFx ArcadeDriveTalonFxType;                // Switch to TalonMotorGroup<TalonFX> for real implementation
    ArcadeDriveTalonFxType *        m_pLeftDriveMotors;                     // Left drive motor control
    ArcadeDriveTalonFxType *        m_pRightDriveMotors;                    // Right drive motor control
    TalonFxMotorController *        m_pIntakeMotor;                         // Intake motor control
    TalonFxMotorController *        m_pFeederMotor;                         // Feeder motor control
    TalonMotorGroup<TalonFX> *      m_pShooterMotors;                       // Shooter motors control
    TalonMotorGroup<TalonFX> *      m_pPivotMotors;                         // Pivot motors control
    Yta::Talon::EmptyTalonFx *      m_pLiftMotors;                          // Lift motors control
    
    // LEDs
    CANdle *                        m_pCandle;                              // Controls an RGB LED strip
    RainbowAnimation                m_RainbowAnimation;                     // Rainbow animation configuration (brightness, speed, # LEDs)

    // Interrupts
    // (none)

    // Digital I/O
    DigitalInput *                  m_pNoteBeamSensor;                      // AndyMark am-4255 beam sensor for monitoring ring position
    DigitalOutput *                 m_pDebugOutput;                         // Debug assist output
    
    // Analog I/O
    // (none)
    
    // Pneumatics
    Compressor *                    m_pCompressor;                          // Object to get info about the compressor
    
    // Servos
    // (none)
    
    // Encoders
    // (none)
    
    // Timers
    Timer *                         m_pMatchModeTimer;                      // Times how long a particular mode (autonomous, teleop) is running
    Timer *                         m_pSafetyTimer;                         // Fail safe in case critical operations don't complete
    
    // Accelerometer
    // (none)
    
    // Gyro
    // (none)

    // Camera
    // Note: Only need to have a thread here and tie it to
    // the RobotCamera class, which handles everything else.
    std::thread                     m_CameraThread;
    
    // Misc
    RobotMode                       m_RobotMode;                            // Keep track of the current robot state
    RobotDriveState                 m_RobotDriveState;                      // Keep track of how the drive sequence flows
    std::optional
    <DriverStation::Alliance>       m_AllianceColor;                        // Color reported by driver station during a match
    bool                            m_bDriveSwap;                           // Allow the user to push a button to change forward/reverse
    bool                            m_bCameraAlignInProgress;               // Indicates if an automatic camera align is in progress
    bool                            m_bShootSpeaker;                        // Differentiates between shooting at the speaker or the amp
    bool                            m_bShootSpeakerClose;                   // Indicates if shooting the speaker from close up or further away
    bool                            m_bShotInProgress;                      // Indicates whether a shot is in progress or not
    bool                            m_bPass;                                // Indicates whether or not a note is being passed
    bool                            m_bIntakeInProgress;                    // Indicates whether a note is being picked up
    bool                            m_bPivotTareInProgress;                 // Indicates whether or not a tare of the pivot mechanism is in progress
    units::angle::degree_t          m_PivotTargetDegrees;                   // Tracks the desired angle position of the superstructure mechanism
    units::angle::degree_t          m_SpeakerTargetDegrees;                 // The current target angle for the pivot mechanism when shooting at the speaker
    units::angle::degree_t          m_AmpTargetDegrees;                     // The current target angle for the pivot mechanism when shooting at the amp
    double                          m_AmpTargetSpeed;                       // The current target speed for the shooter motors when shooting at the amp
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

    static const int                DRIVE_LIFT_ROBOT_BUTTON                 = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUTTON;
    static const int                DRIVE_ALIGN_WITH_CAMERA_BUTTON          = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.DOWN_BUTTON;
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
    static const int                ESTOP_BUTTON                            = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                AUX_SHOOT_AXIS                          = AUX_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.RIGHT_TRIGGER;
    static const int                AUX_INTAKE_AXIS                         = AUX_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.LEFT_TRIGGER;
    static const int                AUX_INTAKE_OUT_BUTTON                   = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUMPER;
    static const int                AUX_TOGGLE_SPEAKER_SHOOT_CLOSE          = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUMPER;
    static const int                AUX_TOGGLE_SPEAKER_AMP_SHOOT_BUTTON     = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.UP_BUTTON;
    static const int                AUX_PASS_BUTTON                         = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUTTON;
    static const int                AUX_INTAKE_AT_SOURCE_BUTTON             = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.DOWN_BUTTON;
    static const int                AUX_TARE_PIVOT_ANGLE                    = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.START;
    static const int                AUX_MANUAL_PIVOT_AXIS                   = AUX_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.LEFT_Y_AXIS;

    // CAN Signals
    // Note: The use of high CAN values if swerve drive is in use is
    //       to prevent instantiating multiple motor controllers with
    //       the same IDs, but still allow code for both drive base
    //       types to be present.  When using swerve drive, IDs 1-8
    //       are used by the swerve modules (see the SwerveModuleConfigs
    //       in SwerveDrive.hpp).
    static const unsigned           LEFT_DRIVE_MOTORS_CAN_START_ID          = Yta::Drive::Config::USE_SWERVE_DRIVE ? 64 : 1;
    static const unsigned           RIGHT_DRIVE_MOTORS_CAN_START_ID         = Yta::Drive::Config::USE_SWERVE_DRIVE ? 66 : 3;
    static const unsigned           SHOOTER_MOTORS_CAN_START_ID             = 9;
    static const unsigned           INTAKE_MOTOR_CAN_ID                     = 11;
    static const unsigned           FEEDER_MOTOR_CAN_ID                     = 12;
    static const unsigned           PIVOT_MOTORS_CAN_START_ID               = 13;
    static const unsigned           LIFT_MOTORS_CAN_START_ID                = 15;

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
    static const int                BEAM_SENSOR_DIO_CHANNEL                 = 0;
    static const int                SENSOR_TEST_CODE_DIO_CHANNEL            = 6;
    static const int                DEBUG_OUTPUT_DIO_CHANNEL                = 7;
    
    // Analog I/O Signals
    // (none)
    
    // Solenoid Signals
    // (none)

    // Solenoids
    // (none)

    // Motor speeds
    static constexpr double         INTAKE_MOTOR_SPEED                      = -1.0;
    static constexpr double         FEEDER_MOTOR_SPEED                      =  0.5;
    static constexpr double         SHOOTER_MOTOR_SPEAKER_CLOSE_CW_SPEED    = -0.5;
    static constexpr double         SHOOTER_MOTOR_SPEAKER_FAR_CW_SPEED      = -0.7;
    static constexpr double         SHOOTER_MOTOR_SPEAKER_CW_OFFSET_SPEED   =  0.2;
    static constexpr double         SHOOTER_MOTOR_SPEAKER_CLOSE_CCW_SPEED   = -0.3;
    static constexpr double         SHOOTER_MOTOR_SPEAKER_FAR_CCW_SPEED     = -0.5;
    static constexpr double         SHOOTER_MOTOR_SPEAKER_CCW_OFFSET_SPEED  = -0.2;
    static constexpr double         SHOOTER_MOTOR_AMP_SPEED                 = -0.18;
    static constexpr double         SHOOTER_MOTOR_LOAD_AT_SOURCE_SPEED      =  0.25;
    static constexpr double         SHOOTER_STEP_SPEED                      =  0.02;
    static constexpr double         SHOOTER_AMP_SPEED_MIN                   = -1.00;
    static constexpr double         SHOOTER_AMP_SPEED_MAX                   =  0.00;
    static constexpr double         LIFT_MOTOR_SPEED                        =  0.70;
    static constexpr double         LIFT_MOTOR_OFFSET_SPEED                 =  0.15;

    // Misc
    const std::string               AUTO_ROUTINE_1_STRING                   = "Speaker center";
    const std::string               AUTO_ROUTINE_2_STRING                   = "Speaker source";
    const std::string               AUTO_ROUTINE_3_STRING                   = "Speaker amp";
    const std::string               AUTO_TEST_ROUTINE_STRING                = "Autonomous Test Routine";
    static constexpr units::angle::degree_t PIVOT_ANGLE_RUNTIME_BASE        =  3.0_deg;
    static constexpr units::angle::degree_t PIVOT_ANGLE_INTAKE_NOTE         = 30.0_deg;
    static constexpr units::angle::degree_t PIVOT_ANGLE_TOUCHING_SPEAKER    = 47.5_deg;
    static constexpr units::angle::degree_t PIVOT_ANGLE_FROM_PODIUM         = 30.0_deg;
    static constexpr units::angle::degree_t PIVOT_ANGLE_TOUCHING_AMP        = 100.0_deg;
    static constexpr units::angle::degree_t SHOOTER_STEP_ANGLE              =  2.0_deg;
    static constexpr units::angle::degree_t PIVOT_ANGLE_MIN                 =  4.0_deg;
    static constexpr units::angle::degree_t PIVOT_ANGLE_MAX                 = 106.0_deg;

    static const int                OFF                                     = 0;
    static const int                ON                                      = 1;
    static const int                ANGLE_90_DEGREES                        = 90;
    static const int                ANGLE_180_DEGREES                       = 180;
    static const int                ANGLE_360_DEGREES                       = 360;
    static const int                POV_INPUT_TOLERANCE_VALUE               = 30;
    static const int                SCALE_TO_PERCENT                        = 100;
    static const unsigned           SINGLE_MOTOR                            = 1;
    static const unsigned           TWO_MOTORS                              = 2;
    static const unsigned           NUMBER_OF_LEDS                          = 8 + 44;

    static const unsigned           CAMERA_RUN_INTERVAL_MS                  = 1000U;
    
    static constexpr double         JOYSTICK_TRIM_UPPER_LIMIT               =  0.05;
    static constexpr double         JOYSTICK_TRIM_LOWER_LIMIT               = -0.05;
    static constexpr double         DRIVE_THROTTLE_VALUE_RANGE              =  1.00;
    static constexpr double         DRIVE_THROTTLE_VALUE_BASE               =  0.00;
    static constexpr double         DRIVE_SLOW_THROTTLE_VALUE               =  0.35;
    static constexpr double         SWERVE_ROTATE_SLOW_JOYSTICK_THRESHOLD   =  0.10;
    static constexpr double         SWERVE_DRIVE_SLOW_SPEED                 =  0.10;
    static constexpr double         SWERVE_ROTATE_SLOW_SPEED                =  0.10;
    static constexpr double         DRIVE_MOTOR_UPPER_LIMIT                 =  1.00;
    static constexpr double         DRIVE_MOTOR_LOWER_LIMIT                 = -1.00;
    static constexpr double         AXIS_INPUT_DEAD_BAND                    =  0.10;
    static constexpr double         LIFT_MAX_ROLL_DEGREES                   =  5.00;
    static constexpr double         LIFT_OFFSET_STOP_POINT_DEGREES          =  0.50;

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
    switch (m_AllianceColor.value())
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
