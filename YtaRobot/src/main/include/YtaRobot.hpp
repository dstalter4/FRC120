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
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTAROBOT_HPP
#define YTAROBOT_HPP

// SYSTEM INCLUDES
#include <cmath>                                            // for M_PI
#include <thread>                                           // for std::thread

// C INCLUDES
#include "frc/Compressor.h"                                 // for retrieving info on the compressor
#include "frc/DigitalInput.h"                               // for DigitalInput type
#include "frc/DigitalOutput.h"                              // for DigitalOutput type
#include "frc/DoubleSolenoid.h"                             // for DoubleSolenoid type
#include "frc/DriverStation.h"                              // for interacting with the driver station
#include "frc/DutyCycleEncoder.h"                           // for interacting with PWM based encoders
#include "frc/PWM.h"                                        // for interacting with PWM based sensors (e.g. actuators)
#include "frc/Relay.h"                                      // for Relay type
#include "frc/Solenoid.h"                                   // for Solenoid type
#include "frc/TimedRobot.h"                                 // for base class decalartion
#include "frc/livewindow/LiveWindow.h"                      // for controlling the LiveWindow
#include "frc/smartdashboard/SendableChooser.h"             // for using the smart dashboard sendable chooser functionality
#include "frc/smartdashboard/SmartDashboard.h"              // for interacting with the smart dashboard
#include "frc2/command/CommandPtr.h"                        // for CommandPtr

// C++ INCLUDES
#include "DifferentialDrive.hpp"                            // for using differential drive
#include "DriveConfiguration.hpp"                           // for information on the drive config
#include "RobotUtils.hpp"                                   // for ASSERT, DEBUG_PRINTS
#include "SwerveDrive.hpp"                                  // for using swerve drive
#include "YtaController.hpp"                                // for controller interaction
#include "YtaTalon.hpp"                                     // for custom Talon control
#include "ctre/phoenix6/CANBus.hpp"                         // for creating CANBus objects
#include "ctre/phoenix6/CANdle.hpp"                         // for interacting with the CANdle
#include "ctre/phoenix6/Pigeon2.hpp"                        // for PigeonIMU
#include "ctre/phoenix6/SignalLogger.hpp"                   // for disabling automatic signal logging
#include "ctre/phoenix6/controls/RainbowAnimation.hpp"      // for creating animations on the CANdle


using namespace frc;
using namespace frc2;
using namespace ctre::phoenix6;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::signals;


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

    // STRUCTS
    // (none)

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
    
    // Autonomous routines
    // @todo: Make YtaRobotAutonomous a friend and move these out (requires accessor to *this)!
    void AutonomousPeriodicTimed();
    void AutonomousPeriodicCommand();
    void AutonomousCommon();
    void AutonomousCommonRed();
    void AutonomousCommonBlue();
    void AutonomousRoutine1();
    void AutonomousRoutine2();
    void AutonomousRoutine3();
    void AutonomousTestRoutine();
    void AutonomousTestSwerveRoutine();
    CommandPtr AutonomousTestCommandDashboardRoutine();
    CommandPtr AutonomousTestCommandMotionRoutine();
    CommandPtr AutonomousTestTrajectoryRoutine();

    // Resets member variables
    void ResetMemberData();

    // Routine to put things in a known state
    void InitialStateSetup();

    // Checks for the RIO pin readings to stabilize
    void CheckIfRioPinsAreStable();

    // Configure motor controller parameters
    void ConfigureMotorControllers();

    // Main sequence for drive motor control
    void SwerveDriveSequence();
    void DifferentialDriveControlSequence();

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

    // Superstructure sequences
    void IntakeSequence();
    void ShootSequence();
    void HoodSequence();
    void HangSequence();
    void CheckForManualAdjust();

    // MEMBER VARIABLES
    
    // Autonomous
    SendableChooser<std::string>    m_AutonomousChooser;                    // Selects from the dashboard which auto routine to run
    SendableChooser<std::string>    m_AutonomousPositionChooser;            // Selects from the dashboard where the robot is located
    
    // User Controls
    DriveControllerType *           m_pDriveController;                     // Drive controller
    AuxControllerType *             m_pAuxController;                       // Auxillary input controller

    // CAN Bus
    CANBus                          m_RioCanBus;                            // CAN bus object for the RIO
    CANBus                          m_CanivoreBus;                          // CAN bus object for the canivore

    static constexpr const std::string_view RIO_CAN_BUS_NAME = "rio";
    static constexpr const std::string_view CANIVORE_CAN_BUS_NAME = "canivore-120";

    // GetCanBusReferenceLambda
    // Lambda to retrieve a reference to the CANBus with the specified string name.
    std::function<const CANBus&(std::string_view)> m_GetCanBusReferenceLambda = [this](std::string_view canBusName) -> const CANBus&
    {
        if (canBusName.compare(CANIVORE_CAN_BUS_NAME) == 0)
        {
            return m_CanivoreBus;
        }
        else
        {
            return m_RioCanBus;
        }
    };
    
    // Swerve Drive
    Pigeon2 *                       m_pPigeon;                              // CTRE Pigeon2 IMU
    SwerveDrive *                   m_pSwerveDrive;                         // Swerve drive control

    // Differential Drive
    DifferentialDrive *             m_pDifferentialDrive;                   // Differential drive control

    // Motors
    TalonFxMotorController *        m_pIntakeRollersMotor;                  // Intake rollers motor control
    TalonFxMotorController *        m_pIntakeAngleMotor;                    // Intake angle motor control
    TalonFxMotorController *        m_pFeederMotor;                         // Feeder motor control
    TalonFxMotorController *        m_pInjectorMotor;                       // Injector motor control
    TalonMotorGroup<TalonFX, TalonFXConfiguration> * m_pShooterMotors;      // Shooter motor control
    TalonFxMotorController *        m_pHoodMotor;                           // Hood motor control
    TalonFxMotorController *        m_pHangMotor;                           // Hang motor control
    
    // LEDs
    CANdle *                        m_pCandle;                              // Controls an RGB LED strip
    SolidColor                      m_LedStripSolidColor;                   // Used when setting the LEDs to RGB values
    RainbowAnimation                m_RainbowAnimation;                     // Rainbow animation configuration (brightness, speed, # LEDs)
    static constexpr const RGBWColor RGBW_OFF{0, 0, 0, 0};                  // Common RGBWColor expression representing LEDs off

    // Interrupts
    // (none)

    // Digital I/O
    DigitalOutput *                 m_pDebugOutput;                         // Debug assist output
    
    // Analog I/O
    // (none)

    // PWM
    PWM *                           m_pHoodLeftServoActuator;              // Object for controlling the hood servo actuator on the left
    PWM *                           m_pHoodRightServoActuator;             // Object for controlling the hood servo actuator on the right
    
    // Pneumatics
    Compressor *                    m_pCompressor;                          // Object to get info about the compressor

    // Solenoids
    // (none)
    
    // Encoders
    CANcoder *                      m_pIntakeCanCoder;                      // Absolute encoder to monitor intake position
    CANcoder *                      m_pHoodCanCoder;                        // Absolute encoder to monitor hood position
    
    // Timers
    Timer *                         m_pMatchModeTimer;                      // Times how long a particular mode (autonomous, teleop) is running
    Timer *                         m_pRobotProgramTimer;                   // Starts at robot program entry, free runs for program life time
    
    // Accelerometer
    // (none)
    
    // Gyro
    // (none)

    // Camera
    // Note: Only need to have a thread here and tie it to
    // the RobotCamera class, which handles everything else.
    std::thread                     m_CameraThread;
    
    // Misc
    double                          m_ShooterMotorSpeed;                    // Keep track of the shooter motor speed
    double                          m_InjectorMotorSpeed;                   // Keep track of the injector motor speed
    units::angle::degree_t          m_IntakeAngleDegrees;                   // Keep track of the intake angle
    units::angle::degree_t          m_IntakeAngleOffsetDegrees;             // Keep track of the intake angle offset from manual adjustment
    RobotMode                       m_RobotMode;                            // Keep track of the current robot state
    std::optional
    <DriverStation::Alliance>       m_AllianceColor;                        // Color reported by driver station during a match
    bool                            m_bIntakeLowered;                       // Keep track if the intake is lowered or raised
    bool                            m_bIntakeSequenceActive;                // Keep track if the robot is actively intaking/ejecting
    bool                            m_bShootSequenceActive;                 // Keep track if the robot is actively shooting/unclogging
    bool                            m_bShotInProgress;                      // Keep track if the robot is shooting balls
    bool                            m_bRioPinsStable;                       // Indicates whether the RIO pin measurements (e.g. PWM) are stable
    bool                            m_bCameraAlignInProgress;               // Indicates if an automatic camera align is in progres
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
    static const int                FIELD_RELATIVE_TOGGLE_BUTTON            = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUMPER;
    static const int                REZERO_SWERVE_BUTTON                    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUMPER;
    static const int                LOCK_SWERVE_WHEELS_BUTTON               = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUTTON;
    static const int                JOG_SWERVE_BUTTON                       = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUTTON;
    static const int                PLAY_MUSIC_BUTTON                       = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_STICK_CLICK;
    static const int                DRIVE_ALIGN_WITH_CAMERA_BUTTON          = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_STICK_CLICK;
    static const int                CAMERA_TOGGLE_FULL_PROCESSING_BUTTON    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                CAMERA_TOGGLE_PROCESSED_IMAGE_BUTTON    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                SELECT_FRONT_CAMERA_BUTTON              = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                SELECT_BACK_CAMERA_BUTTON               = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;

    static const Yta::Controller::PovDirections  DRIVE_CONTROLS_SWERVE_FORWARD_SLOW_POV     = Yta::Controller::PovDirections::POV_UP;
    static const Yta::Controller::PovDirections  DRIVE_CONTROLS_SWERVE_REVERSE_SLOW_POV     = Yta::Controller::PovDirections::POV_DOWN;
    static const Yta::Controller::PovDirections  DRIVE_CONTROLS_SWERVE_LEFT_OR_CCW_SLOW_POV = Yta::Controller::PovDirections::POV_LEFT;
    static const Yta::Controller::PovDirections  DRIVE_CONTROLS_SWERVE_RIGHT_OR_CW_SLOW_POV = Yta::Controller::PovDirections::POV_RIGHT;


    // These driver inputs only apply to differential drive.
    // Note: The primary drive axes are in the controller headers.
    static const int                DRIVE_SLOW_X_AXIS                       = DRIVE_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.RIGHT_X_AXIS;
    static const int                DRIVE_SLOW_Y_AXIS                       = DRIVE_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.RIGHT_Y_AXIS;
    static const int                DRIVE_SWAP_BUTTON                       = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;

    static const Yta::Controller::PovDirections  DRIVE_CONTROLS_INCH_FORWARD_POV            = Yta::Controller::PovDirections::POV_INVALID_UP;
    static const Yta::Controller::PovDirections  DRIVE_CONTROLS_INCH_REVERSE_POV            = Yta::Controller::PovDirections::POV_INVALID_DOWN;
    static const Yta::Controller::PovDirections  DRIVE_CONTROLS_INCH_LEFT_POV               = Yta::Controller::PovDirections::POV_INVALID_LEFT;
    static const Yta::Controller::PovDirections  DRIVE_CONTROLS_INCH_RIGHT_POV              = Yta::Controller::PovDirections::POV_INVALID_RIGHT;


    // Aux inputs
    static const int                AUX_SHOOT_AXIS                          = AUX_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.RIGHT_TRIGGER;
    static const int                AUX_RAMP_UP_AXIS                        = AUX_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.LEFT_TRIGGER;
    static const int                AUX_INTAKE_UP_DOWN_BUTTON               = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUTTON;
    static const int                AUX_UNCLOG_BUTTON                       = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUTTON;
    static const int                AUX_HOOD_UP_BUTTON                      = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.UP_BUTTON;
    static const int                AUX_HOOD_DOWN_BUTTON                    = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.DOWN_BUTTON;
    static const int                AUX_INTAKE_BUTTON                       = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUMPER;
    static const int                AUX_EJECT_BUTTON                        = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUMPER;
    static const int                AUX_MANUAL_ADJUST_BUTTON                = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.START;
    static const int                AUX_MANUAL_ADJUST_TOGGLE_BUTTON         = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.SELECT;
    static const int                ESTOP_BUTTON                            = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;

    static const Yta::Controller::PovDirections  AUX_MANUAL_ADJUST_UP_POV_DIRECTION         = Yta::Controller::PovDirections::POV_UP;
    static const Yta::Controller::PovDirections  AUX_MANUAL_ADJUST_DOWN_POV_DIRECTION       = Yta::Controller::PovDirections::POV_DOWN;

    // CAN Signals
    // Note: The use of high CAN values if swerve drive is in use is
    //       to prevent instantiating multiple motor controllers with
    //       the same IDs, but still allow code for both drive base
    //       types to be present.  When using swerve drive, IDs 11-18
    //       are used by the swerve modules (see the SwerveModuleConfigs
    //       in SwerveConfig.hpp).  When using differential drive, check
    //       the IDs in DifferentialDrive.hpp.
    // Superstructure uses IDs starting at 31
    static const unsigned           INTAKE_ROLLERS_MOTOR_CAN_ID             = 31;   // PDH 6
    static const unsigned           INTAKE_ANGLE_MOTOR_CAN_ID               = 32;   // PDH 4
    static const unsigned           FEEDER_MOTOR_CAN_ID                     = 33;   // PDH 16
    static const unsigned           INJECTOR_MOTOR_CAN_ID                   = 34;   // PDH 5
    static const unsigned           SHOOTER_MOTORS_CAN_START_ID             = 35;   // PDH 14, PDH 12
    static const unsigned           HOOD_MOTOR_CAN_ID                       = 37;   // PDH 7
    static const unsigned           HANG_MOTOR_CAN_ID                       = 38;
    static const unsigned           INTAKE_CANCODER_CAN_ID                  = 41;
    static const unsigned           HOOD_CANCODER_CAN_ID                    = 42;

    // CANivore Signals
    // Note: IDs 21-24 are used by the CANcoders (see the
    //       SwerveModuleConfigs in SwerveConfig.hpp).
    static const int                PIGEON_CAN_ID                           = 25;
    static const int                CANDLE_CAN_ID                           = 26;

    // PWM Signals
    static const int                HOOD_SERVO_LEFT_ACTUATOR_PWM_CHANNEL    = 0;
    static const int                HOOD_SERVO_RIGHT_ACTUATOR_PWM_CHANNEL   = 1;
    
    // Relays
    // (none)
    
    // Digital I/O Signals
    static const int                SENSOR_TEST_CODE_DIO_CHANNEL            = 6;
    static const int                DEBUG_OUTPUT_DIO_CHANNEL                = 7;
    
    // Analog I/O Signals
    // (none)
    
    // Solenoid Signals
    // (none)

    // Motor speeds and angles
    static constexpr double         INTAKE_ROLLERS_MOTOR_SPEED              = 0.75;
    static constexpr double         FEEDER_MOTOR_SPEED                      = 0.20;
    static constexpr double         INJECTOR_MOTOR_SPEED                    = -0.80;
    static constexpr double         INJECTOR_MOTOR_SPEED_STEP               = -0.05;
    static constexpr double         SHOOTER_MOTOR_SPEED                     = 0.50;
    static constexpr double         SHOOTER_MOTOR_SPEED_STEP                = 0.05;

    static constexpr const units::angle::degree_t INTAKE_UP_ANGLE_DEGREES               = -20.0_deg;
    static constexpr const units::angle::degree_t INTAKE_DOWN_ANGLE_DEGREES             = -120.0_deg;
    static constexpr const units::angle::degree_t INTAKE_MANUAL_ADJUST_STEP_DEGREES     = 10.0_deg;

    static constexpr const units::time::second_t    SHOOTER_RAMP_UP_TIME_S  = 0.25_s;

    // Misc
    const std::string               AUTO_NO_ROUTINE_STRING                  = "No autonomous routine";
    const std::string               AUTO_ROUTINE_1_STRING                   = "Left bump to depot, then shoot";
    const std::string               AUTO_ROUTINE_2_STRING                   = "Hub to depot, then shoot";
    const std::string               AUTO_ROUTINE_3_STRING                   = "Shoot from hub, move to depot, rotate";
    const std::string               AUTO_TEST_ROUTINE_STRING                = "Autonomous Test Routine";

    static const int                OFF                                     = 0;
    static const int                ON                                      = 1;
    static const int                ANGLE_90_DEGREES                        = 90;
    static const int                ANGLE_180_DEGREES                       = 180;
    static const int                ANGLE_360_DEGREES                       = 360;
    static const int                POV_INPUT_TOLERANCE_VALUE               = 30;
    static const int                SCALE_TO_PERCENT                        = 100;
    static const unsigned           SINGLE_MOTOR                            = 1;
    static const unsigned           TWO_MOTORS                              = 2;
    static const unsigned           NUMBER_OF_LEDS                          = 8;

    static constexpr double         JOYSTICK_AXIS_INPUT_DEAD_BAND           =  0.10;
    static constexpr double         DRIVE_TRIM_UPPER_LIMIT                  =  0.05;
    static constexpr double         DRIVE_TRIM_LOWER_LIMIT                  = -0.05;
    static constexpr double         SWERVE_DRIVE_SLOW_SPEED                 =  0.10;
    static constexpr double         SWERVE_ROTATE_SLOW_SPEED                =  0.10;
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
            constexpr const RGBWColor RGBW_RED{255, 0, 0, 0};
            m_pCandle->SetControl(m_LedStripSolidColor.WithColor(RGBW_RED));
            break;
        }
        case DriverStation::Alliance::kBlue:
        {
            constexpr const RGBWColor RGBW_BLUE{0, 0, 255, 0};
            m_pCandle->SetControl(m_LedStripSolidColor.WithColor(RGBW_BLUE));
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
