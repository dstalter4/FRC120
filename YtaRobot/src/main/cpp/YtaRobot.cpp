////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobot.cpp
/// @author David Stalter
///
/// @details
/// Implementation of the YtaRobot class.  This file contains the functions for
/// full robot operation in FRC.  It contains the autonomous and operator
/// control routines as well as all necessary support for interacting with all
/// motors, sensors and input/outputs on the robot.
///
/// Copyright (c) 2023 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <cstddef>                      // for nullptr
#include <cstring>                      // for memset

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                 // for class declaration (and other headers)
#include "RobotCamera.hpp"              // for interacting with cameras
#include "RobotI2c.hpp"                 // for I2cThread()
#include "RobotUtils.hpp"               // for Trim(), Limit() and DisplayMessage()

// STATIC MEMBER VARIABLES
YtaRobot * YtaRobot::m_pThis;


////////////////////////////////////////////////////////////////
/// @method YtaRobot::YtaRobot
///
/// Constructor.  Instantiates all robot control objects.
///
////////////////////////////////////////////////////////////////
YtaRobot::YtaRobot() :
    m_AutonomousChooser                 (),
    m_AutonomousGamePieceChooser        (),
    m_pDriveController                  (new DriveControllerType(DRIVE_CONTROLLER_MODEL, DRIVE_JOYSTICK_PORT)),
    m_pAuxController                    (new AuxControllerType(AUX_CONTROLLER_MODEL, AUX_JOYSTICK_PORT)),
    m_pPigeon                           (new Pigeon2(PIGEON_CAN_ID, "canivore-120")),
    m_pSwerveDrive                      (new SwerveDrive(m_pPigeon)),
    m_pLeftDriveMotors                  (new TalonMotorGroup<TalonFX>("Left Drive", NUMBER_OF_LEFT_DRIVE_MOTORS, LEFT_DRIVE_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW, NeutralMode::Brake, FeedbackDevice::IntegratedSensor, true)),
    m_pRightDriveMotors                 (new TalonMotorGroup<TalonFX>("Right Drive", NUMBER_OF_RIGHT_DRIVE_MOTORS, RIGHT_DRIVE_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW, NeutralMode::Brake, FeedbackDevice::IntegratedSensor, true)),
    m_pCarriageMotors                   (new TalonMotorGroup<TalonFX>("Carriage", NUMBER_OF_CARRIAGE_MOTORS, CARRIAGE_MOTORS_START_CAN_ID, MotorGroupControlMode::FOLLOW_INVERSE, NeutralMode::Brake, FeedbackDevice::IntegratedSensor, true)),
    m_pIntakeMotor                      (new TalonFX(INTAKE_MOTOR_CAN_ID)),
    m_pCandle                           (new CANdle(CANDLE_CAN_ID, "canivore-120")),
    m_RainbowAnimation                  ({1, 0.5, 308}),
    m_pDebugOutput                      (new DigitalOutput(DEBUG_OUTPUT_DIO_CHANNEL)),
    m_pTalonCoolingSolenoid             (new DoubleSolenoid(PneumaticsModuleType::CTREPCM, TALON_COOLING_SOLENOID_FWD_CHANNEL, TALON_COOLING_SOLENOID_REV_CHANNEL)),
    m_pCompressor                       (new Compressor(PneumaticsModuleType::CTREPCM)),
    m_pDriveMotorCoolTimer              (new Timer()),
    m_pMatchModeTimer                   (new Timer()),
    m_pInchingDriveTimer                (new Timer()),
    m_pDirectionalAlignTimer            (new Timer()),
    m_pSafetyTimer                      (new Timer()),
    m_pAccelerometer                    (new BuiltInAccelerometer),
    m_pAdxrs450Gyro                     (nullptr),
    m_Bno055Angle                       (),
    m_CameraThread                      (RobotCamera::LimelightThread),
    m_SerialPortBuffer                  (),
    m_pSerialPort                       (new SerialPort(SERIAL_PORT_BAUD_RATE, SerialPort::kMXP, SERIAL_PORT_NUM_DATA_BITS, SerialPort::kParity_None, SerialPort::kStopBits_One)),
    m_I2cThread                         (RobotI2c::I2cThread),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_RobotDriveState                   (MANUAL_CONTROL),
    m_AllianceColor                     (DriverStation::GetAlliance()),
    m_bIntakeCube                       (true),
    m_bIntakeStalled                    (false),
    m_bDriveSwap                        (false),
    m_bCoolingDriveMotors               (true),
    m_LastDriveMotorCoolTime            (0_s),
    m_HeartBeat                         (0U)
{
    RobotUtils::DisplayMessage("Robot constructor.");
    
    // LiveWindow is not used
    LiveWindow::SetEnabled(false);
    
    // Set the autonomous options
    m_AutonomousChooser.SetDefaultOption(AUTO_ROUTINE_1_STRING, AUTO_ROUTINE_1_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_2_STRING, AUTO_ROUTINE_2_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_TEST_ROUTINE_STRING, AUTO_TEST_ROUTINE_STRING);
    SmartDashboard::PutData("Autonomous Modes", &m_AutonomousChooser);
    m_AutonomousGamePieceChooser.SetDefaultOption("Cone", "Cone");
    m_AutonomousGamePieceChooser.AddOption("Cube", "Cube");
    SmartDashboard::PutData("Auto Game Piece", &m_AutonomousGamePieceChooser);
    
    RobotUtils::DisplayFormattedMessage("The drive forward axis is: %d\n", Yta::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.RIGHT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive reverse axis is: %d\n", Yta::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive left/right axis is: %d\n", Yta::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_X_AXIS);

    ConfigureMotorControllers();

    CANdleConfiguration candleConfig;
    candleConfig.stripType = LEDStripType::RGB;
    m_pCandle->ConfigAllSettings(candleConfig);
    m_pCandle->Animate(m_RainbowAnimation);

    // Construct the ADXRS450 gyro if configured
    if (ADXRS450_GYRO_PRESENT)
    {
        m_pAdxrs450Gyro = new ADXRS450_Gyro();
    }

    // Reset the serial port and clear buffer
    m_pSerialPort->Reset();
    std::memset(&m_SerialPortBuffer, 0U, sizeof(m_SerialPortBuffer));
    
    // Spawn the vision and I2C threads
    // @todo: Use a control variable to prevent the threads from executing too soon.
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::ARRAY_OFF);
    m_CameraThread.detach();
    m_I2cThread.detach();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::ResetMemberData
///
/// This method resets relevant member data variables.  Since
/// the robot object is only constructed once, it may be
/// necessary/helpful to return to a state similar to when the
/// constructor first ran (e.g. when enabling/disabling robot
/// states).  Only variables that need to be reset are modified
/// here.  This also works around the issue where non-member
/// static data cannot be easily reinitialized (since clearing
/// the .bss and running static constructors will only happen
/// once on program start up).
///
////////////////////////////////////////////////////////////////
void YtaRobot::ResetMemberData()
{
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::RobotInit
///
/// This method is run when initializing the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::RobotInit()
{
    RobotUtils::DisplayMessage("RobotInit called.");
    SetStaticThisInstance();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::RobotPeriodic
///
/// This method is run in all robot states.  It is called each
/// time a new packet is received from the driver station.
///
////////////////////////////////////////////////////////////////
void YtaRobot::RobotPeriodic()
{
    static bool bRobotPeriodicStarted = false;
    if (!bRobotPeriodicStarted)
    {
        RobotUtils::DisplayMessage("RobotPeriodic called.");
        bRobotPeriodicStarted = true;
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::ConfigureMotorControllers
///
/// Sets motor controller specific configuration information.
///
////////////////////////////////////////////////////////////////
void YtaRobot::ConfigureMotorControllers()
{
    // These are the defaults for the configuration (see TalonFX.h)
    //ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
    //double integratedSensorOffsetDegrees = 0;
    //ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;

    // The default constructor for TalonFXConfiguration will call the parent
    // BaseTalonConfiguration constructor with FeedbackDevice::IntegratedSensor.
    // Currently no current limiting being done.
    // New lift: try kP = 0.05 and kD = 5.0 (orig 0.08/0.5)
    TalonFXConfiguration talonConfig;
    talonConfig.slot0.kP = 0.08;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.3;
    talonConfig.slot0.kF = 0.0;
    talonConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    talonConfig.integratedSensorOffsetDegrees = 0.0;
    talonConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    //talonConfig.peakOutputForward = 0.5;
    //talonConfig.peakOutputReverse = 0.5;

    TalonFX * pTalon = m_pCarriageMotors->GetMotorObject(CARRIAGE_MOTORS_START_CAN_ID);
    pTalon->ConfigFactoryDefault();
    pTalon->ConfigAllSettings(talonConfig);
    pTalon->SetSelectedSensorPosition(0);

    m_pIntakeMotor->ConfigFactoryDefault();
    //talonConfig.slot0.closedLoopPeakOutput = 0.10;
    //talonConfig.peakOutputForward = 1.0;
    //talonConfig.peakOutputReverse = 1.0;
    m_pIntakeMotor->ConfigAllSettings(talonConfig);
    m_pIntakeMotor->SetSelectedSensorPosition(0);
    //const StatorCurrentLimitConfiguration INTAKE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG = {true, 5.0, 50.0, 5.0};
    //m_pIntakeMotor->ConfigStatorCurrentLimit(INTAKE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::InitialStateSetup
///
/// This method contains the work flow for putting motors,
/// solenoids, etc. into a known state.  It is intended to be
/// used by both autonomous and user control.
///
////////////////////////////////////////////////////////////////
void YtaRobot::InitialStateSetup()
{
    // First reset any member data
    ResetMemberData();

    // Carriage motor neutral mode was configured in the constructor
    m_pIntakeMotor->SetNeutralMode(NeutralMode::Brake);
    m_pCarriageMotors->GetMotorObject()->Set(ControlMode::Position, CARRIAGE_CONE_FIXED_ENCODER_POSITION);

    // Solenoids to known state
    m_pTalonCoolingSolenoid->Set(TALON_COOLING_OFF_SOLENOID_VALUE);

    // Disable the rainbow animation
    m_pCandle->ClearAnimation(0);
    
    // Stop/clear any timers, just in case
    // @todo: Make this a dedicated function.
    m_pDriveMotorCoolTimer->Stop();
    m_pDriveMotorCoolTimer->Reset();
    m_pMatchModeTimer->Stop();
    m_pMatchModeTimer->Reset();
    m_pInchingDriveTimer->Stop();
    m_pInchingDriveTimer->Reset();
    m_pDirectionalAlignTimer->Stop();
    m_pDirectionalAlignTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Just in case constructor was called before these were set (likely the case)
    m_AllianceColor = DriverStation::GetAlliance();
    
    // Clear the debug output pin
    m_pDebugOutput->Set(false);

    // Reset the heartbeat
    m_HeartBeat = 0U;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::TeleopInit
///
/// The teleop init method.  This method is called once each
/// time the robot enters teleop control.
///
////////////////////////////////////////////////////////////////
void YtaRobot::TeleopInit()
{
    RobotUtils::DisplayMessage("TeleopInit called.");
    
    // Autonomous should have left things in a known state, but
    // just in case clear everything.
    InitialStateSetup();

    // Start showing a cube (purple)
    m_pCandle->SetLEDs(240, 73, 241, 0, 0, NUMBER_OF_LEDS);

    // Tele-op won't do detailed processing of the images unless instructed to
    RobotCamera::SetFullProcessing(false);
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::ARRAY_OFF);
    
    // Indicate to the I2C thread to get data less often
    RobotI2c::SetThreadUpdateRate(I2C_RUN_INTERVAL_MS);

    // Start the mode timer for teleop
    m_pMatchModeTimer->Start();

    // Start the drive motor cooling timer
    m_pDriveMotorCoolTimer->Reset();
    m_pDriveMotorCoolTimer->Start();
    m_LastDriveMotorCoolTime = 0_s;
    m_bCoolingDriveMotors = true;

    // Set the swerve modules to a known angle.  This (somehow) mitigates
    // the random spin when enabling teleop until it can be investigated.
    m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, 0.10, true, true);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::TeleopPeriodic
///
/// The teleop control method.  This method is called
/// periodically while the robot is in teleop control.
///
////////////////////////////////////////////////////////////////
void YtaRobot::TeleopPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_TELEOP);

    HeartBeat();

    if (Yta::Drive::Config::USE_SWERVE_DRIVE)
    {
        // Disabled until more testing is done
        if (false)//m_pDriveController->GetButtonState(DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUTTON))
        {
            RobotCamera::SetLimelightPipeline(1);
            RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::VISION_PROCESSOR);
            RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::ARRAY_ON);
            RobotCamera::AutonomousCamera::AlignToTargetSwerve();
        }
        else
        {
            RobotCamera::SetLimelightPipeline(0);
            RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
            RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::ARRAY_OFF);
            SwerveDriveSequence();
        }
    }
    else
    {
        DriveControlSequence();
    }

    //SuperStructureSequence();
    CheckAndResetEncoderCounts();
    CarriageControlSequence();
    IntakeControlSequence();

    PneumaticSequence();

    //SerialPortSequence();
    
    //I2cSequence();
    
    //CameraSequence();

    LedSequence();

    UpdateSmartDashboard();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::UpdateSmartDashboard
///
/// Updates values in the smart dashboard.
///
////////////////////////////////////////////////////////////////
void YtaRobot::UpdateSmartDashboard()
{
    // @todo: Check if RobotPeriodic() is called every 20ms and use static counter.
    // Give the drive team some state information
    SmartDashboard::PutNumber("Carriage encoder count", m_pCarriageMotors->GetMotorObject()->GetSelectedSensorPosition());
    SmartDashboard::PutNumber("Intake encoder count", m_pIntakeMotor->GetSelectedSensorPosition());
    SmartDashboard::PutBoolean("Intake stalled", m_bIntakeStalled);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::SuperStructureTestSequence
///
/// Quick super structure test.
///
////////////////////////////////////////////////////////////////
void YtaRobot::SuperStructureTestSequence()
{
    // Arm length = 25.5 inches
    // Arm gear ratio: 30:22 * 100:1
    // Wrist gear ratio: 100:1
    // position counts per degree = (gear ratio * 2048) / 360

    // Wrist measurements:  Absolute = 1192 @ vertical
    //                      F 3606 <-> N -60122 <-> B -131674
    //                      F<->N range = 63728
    //                      B<->N range = 71552
    //                      Full range = 135280
    //
    // Arm measurements:    Absolute = 1888 @ vertical
    //                      F -106632 <-> N 1319 <-> B 102704
    //                      F<->N range = 107951
    //                      B<->N range = 101385
    //                      Full range = 209336
    //
    // Carriage measurements:   Absolute = 1139 @ base, 1429 @ base
    //                          Bottom: ~10000
    //                          Top: ~1.141M
    //                          Range: ~1.13M
    //

    TalonFX * pTalon = m_pCarriageMotors->GetMotorObject(CARRIAGE_MOTORS_START_CAN_ID);
    double encoderCount = pTalon->GetSelectedSensorPosition();
    //const uint32_t STEP_VALUE = (FALCON_ENCODER_COUNTS_PER_ROTATION / 2.0);
    const uint32_t STEP_VALUE = 20'000;
    static int32_t setRequest = 0U;
    static double lastEncoderSetPoint = 0;
    if (m_pDriveController->DetectButtonChange(DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.START))
    {
        // SetSelectedSensorPosition() seems to actually set the position count register, not cause movement
        pTalon->Set(ControlMode::Position, (encoderCount + STEP_VALUE));
        lastEncoderSetPoint = encoderCount + STEP_VALUE;
        setRequest++;
    }
    if (m_pDriveController->DetectButtonChange(DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.SELECT))
    {
        // SetSelectedSensorPosition() seems to actually set the position count register, not cause movement
        pTalon->Set(ControlMode::Position, (encoderCount - STEP_VALUE));
        lastEncoderSetPoint = encoderCount - STEP_VALUE;
        setRequest--;
    }
    if (m_pDriveController->DetectButtonChange(DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.UP_BUTTON))
    {
        // open loop/closed loop?
        // ConfigClosedLoopPeakOutput()?
        pTalon->Set(ControlMode::PercentOutput, 0.0);
        pTalon->SetSelectedSensorPosition(0);
    }
    SmartDashboard::PutNumber("Debug A", pTalon->GetSensorCollection().GetIntegratedSensorAbsolutePosition());
    SmartDashboard::PutNumber("Debug B", encoderCount);
    SmartDashboard::PutNumber("Debug C", lastEncoderSetPoint);
    SmartDashboard::PutNumber("Debug D", lastEncoderSetPoint - encoderCount);

/*
    static double speed = 0.5;
    static bool bForward = true;
    if (m_pDriveController->DetectButtonChange(DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_STICK_CLICK))
    {
        speed *= -1.0;
        bForward = !bForward;
    }

    if (m_pDriveController->GetButtonState(DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.UP_BUTTON))
    {
        m_pArmMotor->Set(ControlMode::PercentOutput, speed);
    }
    else
    {
        m_pArmMotor->Set(ControlMode::PercentOutput, 0.0);
    }

    if (m_pDriveController->GetButtonState(DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUTTON))
    {
        m_pWristMotor->Set(ControlMode::PercentOutput, speed);
    }
    else
    {
        m_pWristMotor->Set(ControlMode::PercentOutput, 0.0);
    }

    if (m_pDriveController->GetButtonState(DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.DOWN_BUTTON))
    {
        m_pCarriageMotors->Set(speed);
    }
    else
    {
        m_pCarriageMotors->Set(0.0);
    }

    if (m_pDriveController->GetButtonState(DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUTTON))
    {
        m_pIntakeMotor->Set(ControlMode::PercentOutput, speed);
    }
    else
    {
        m_pIntakeMotor->Set(ControlMode::PercentOutput, 0.0);
    }
*/
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CheckAndResetEncoderCounts
///
/// Checks for driver input to rezero all encoder counts.
///
////////////////////////////////////////////////////////////////
void YtaRobot::CheckAndResetEncoderCounts()
{
    if (m_pDriveController->GetButtonState(DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.START) && m_pAuxController->GetButtonState(AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.START))
    {
        m_pCarriageMotors->GetMotorObject()->SetSelectedSensorPosition(0);
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CarriageControlSequence
///
/// Updates the state of the arm (carriage and swing position).
///
////////////////////////////////////////////////////////////////
void YtaRobot::CarriageControlSequence()
{
    enum CarriagePosition
    {
        CARRIAGE_LOW,
        CARRIAGE_MID,
        CARRIAGE_HIGH,
        CARRIAGE_HUMAN_PLAYER
    };

    // Static variables for fixed position control
    static bool bUsingFixedMode = true;
    static int32_t fixedPosition = 0U;
    static CarriagePosition carriagePosition = CARRIAGE_LOW;
    static bool bAtLoadHeight = false;

    //const int32_t FIXED_POSITION_STEP_VALUE = 25'000;
    if (m_pAuxController->DetectButtonChange(AUX_CARRIAGE_UP_BUTTON))
    {
        switch (carriagePosition)
        {
            case CARRIAGE_LOW:
            {
                fixedPosition = CARRIAGE_MID_FIXED_ENCODER_POSITION;
                carriagePosition = CARRIAGE_MID;
                bAtLoadHeight = true;
                break;
            }
            case CARRIAGE_MID:
            case CARRIAGE_HUMAN_PLAYER:
            {
                fixedPosition = CARRIAGE_MAX_FIXED_ENCODER_POSITION;
                carriagePosition = CARRIAGE_HIGH;
                break;
            }
            case CARRIAGE_HIGH:
            default:
            {
                // If already in the high position, extension can't go further
                break;
            }
        }
        /*
        fixedPosition += FIXED_POSITION_STEP_VALUE;
        if (fixedPosition > 150'000)
        {
            fixedPosition = 150'000;
        }
        */
        //fixedPosition = CARRIAGE_MID_FIXED_ENCODER_POSITION;
        m_pCarriageMotors->GetMotorObject()->Set(ControlMode::Position, fixedPosition);
        bUsingFixedMode = true;
        return;
    }
    else if (m_pAuxController->DetectButtonChange(AUX_CARRIAGE_DOWN_BUTTON))
    {
        switch (carriagePosition)
        {
            case CARRIAGE_HIGH:
            case CARRIAGE_HUMAN_PLAYER:
            {
                fixedPosition = CARRIAGE_MID_FIXED_ENCODER_POSITION;
                carriagePosition = CARRIAGE_MID;
                bAtLoadHeight = true;
                break;
            }
            case CARRIAGE_MID:
            {
                // We should set the position based on cone/cube, but the state change logic also needs an update
                //fixedPosition = m_bIntakeCube ? CARRIAGE_CONE_FIXED_ENCODER_POSITION : CARRIAGE_MIN_FIXED_ENCODER_POSITION;
                fixedPosition = CARRIAGE_CONE_FIXED_ENCODER_POSITION;
                carriagePosition = CARRIAGE_LOW;
                bAtLoadHeight = false;
                break;
            }
            case CARRIAGE_LOW:
            default:
            {
                // If already in the low position, retraction can't go further
                break;
            }
        }
        /*
        fixedPosition -= FIXED_POSITION_STEP_VALUE;
        if (fixedPosition < 0)
        {
            fixedPosition = 0;
        }
        */
        //fixedPosition = CARRIAGE_CONE_FIXED_ENCODER_POSITION;
        m_pCarriageMotors->GetMotorObject()->Set(ControlMode::Position, fixedPosition);
        bUsingFixedMode = true;
        return;
    }
    else if (m_pAuxController->DetectButtonChange(AUX_HUMAN_PLAYER_LOAD_BUTTON))
    {
        if (!bAtLoadHeight)
        {
            m_pCarriageMotors->GetMotorObject()->Set(ControlMode::Position, CARRIAGE_LOAD_FIXED_ENCODER_POSITION);
            carriagePosition = CARRIAGE_HUMAN_PLAYER;
        }
        else
        {
            m_pCarriageMotors->GetMotorObject()->Set(ControlMode::Position, CARRIAGE_CONE_FIXED_ENCODER_POSITION);
            carriagePosition = CARRIAGE_LOW;
        }
        bUsingFixedMode = true;
        bAtLoadHeight = !bAtLoadHeight;
        return;
    }
    else
    {
    }

    // Controls moving the carriage up/down
    double auxCarriageValue = RobotUtils::Trim(-m_pAuxController->GetAxisValue(AUX_MOVE_CARRIAGE_AXIS), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    double carriageSetValue = auxCarriageValue;

    // Inputs from the drive team negate using fixed mode
    if (carriageSetValue != 0.0)
    {
        bUsingFixedMode = false;
    }

    // Logic to set the manual control
    if (!bUsingFixedMode)
    {
        // Manual override
        if (carriageSetValue != 0.0 && m_pAuxController->GetButtonState(AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.START))
        {
            m_pCarriageMotors->Set(carriageSetValue * CARRIAGE_MOVEMENT_SCALING_FACTOR);
            return;
        }
        // Do not allow extending past the max range
        if ((carriageSetValue > 0.0) && (m_pCarriageMotors->GetMotorObject()->GetSelectedSensorPosition() > CARRIAGE_MAX_FIXED_ENCODER_POSITION))
        {
            carriageSetValue = 0.0;
        }
        // Do not allow retracting past the base
        if ((carriageSetValue < 0.0) && (m_pCarriageMotors->GetMotorObject()->GetSelectedSensorPosition() < CARRIAGE_MIN_FIXED_ENCODER_POSITION))
        {
            carriageSetValue = 0.0;
        }
        m_pCarriageMotors->Set(carriageSetValue * CARRIAGE_MOVEMENT_SCALING_FACTOR);
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::IntakeControlSequence
///
/// Updates the state of the wrist (position and intake).
///
////////////////////////////////////////////////////////////////
void YtaRobot::IntakeControlSequence()
{
    // @todo: Check for stall with (m_pIntakeMotor->GetSelectedSensorVelocity() == 0.0)
    // Nominal free spin current: 10-15A
    // Stall current: 60-75A
    // Sensor velocity is negative on intake, ~-5000

    // Static variables for stall control
    static double stalledEncoderCount = 0.0;
    static Timer * pJogTimer = new Timer();
    static units::second_t lastTimeStamp = 0.0_s;
    static bool bJogOn = false;

    // Check for a stall condition
    if (!m_bIntakeStalled)
    {
        // If the sensor is barely moving and current is high, assume a stall
        if ((std::abs(m_pIntakeMotor->GetSelectedSensorVelocity()) < 10.0) && (m_pIntakeMotor->GetStatorCurrent() > 15.0))
        {
            // Motor off, start jogging
            m_pIntakeMotor->Set(ControlMode::PercentOutput, OFF);
            m_bIntakeStalled = true;
            stalledEncoderCount = m_pIntakeMotor->GetSelectedSensorPosition();
            pJogTimer->Start();
            lastTimeStamp = pJogTimer->Get();
            return;
        }
    }

    // Allow the drive team to override the stall controls if needed
    if (m_pAuxController->DetectButtonChange(AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.START))
    {
        m_bIntakeStalled = false;
    }

    // Controls the intake (cube and cones spin in different directions)
    const double CUBE_CONE_MULTIPLIER = m_bIntakeCube ? -1.0 : 1.0;
    const double CUBE_CONE_IN_SPEED = m_bIntakeCube ? INTAKE_IN_CUBE_MOTOR_SPEED : INTAKE_IN_CONE_MOTOR_SPEED;
    double manualControlSpeed = 0.0;
    if (std::abs(m_pAuxController->GetAxisValue(AUX_INTAKE_FORWARD_AXIS)) > JOYSTICK_TRIM_UPPER_LIMIT)
    {
        manualControlSpeed = (INTAKE_OUT_MOTOR_SPEED * CUBE_CONE_MULTIPLIER);

        // Clear a stall when drive team tries to eject
        m_bIntakeStalled = false;
        pJogTimer->Stop();
        pJogTimer->Reset();
    }
    else if (std::abs(m_pAuxController->GetAxisValue(AUX_INTAKE_REVERSE_AXIS)) > JOYSTICK_TRIM_UPPER_LIMIT)
    {
        manualControlSpeed = (-CUBE_CONE_IN_SPEED * CUBE_CONE_MULTIPLIER);
    }
    else
    {
        manualControlSpeed = OFF;
    }



    // Hold position if stalled
    if (m_bIntakeStalled)
    {
        // Cube will just try to hold position
        if (m_bIntakeCube)
        {
            m_pIntakeMotor->Set(ControlMode::Position, stalledEncoderCount);
        }
        else
        {
            // Cones need to jog the motor
            if ((pJogTimer->Get() - lastTimeStamp) > 0.2_s)
            {
                if (bJogOn)
                {
                    m_pIntakeMotor->Set(ControlMode::PercentOutput, -0.07);
                }
                else
                {
                    m_pIntakeMotor->Set(ControlMode::PercentOutput, -0.07);
                }
                bJogOn = !bJogOn;
                lastTimeStamp = pJogTimer->Get();
            }
        }
    }
    // Not stalled, just use the manual input value
    else
    {
        m_pIntakeMotor->Set(ControlMode::PercentOutput, manualControlSpeed);
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::LedSequence
///
/// This method contains the main workflow for controlling
/// any LEDs on the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::LedSequence()
{
    enum LedStrandColor
    {
        CUBE_PURPLE,
        CONE_YELLOW,
        NUM_STRAND_COLORS
    };
    const LedColors CANDLE_LED_COLORS[NUM_STRAND_COLORS] =
    {
        // This order matches the enum order above (purple, yellow) for easy indexing
        {240,  73, 241,   0},
        {255, 240,   0,   0}
    };
    static LedStrandColor strandColor = CUBE_PURPLE;

    if (m_pDriveController->DetectButtonChange(DRV_TOGGLE_LEDS_BUTTON) || m_pAuxController->DetectButtonChange(AUX_TOGGLE_LEDS_BUTTON))
    {
        switch (strandColor)
        {
            case CUBE_PURPLE:
            {
                // Set all LEDs to yellow
                m_pCandle->SetLEDs(CANDLE_LED_COLORS[CONE_YELLOW].m_Red,
                                   CANDLE_LED_COLORS[CONE_YELLOW].m_Green,
                                   CANDLE_LED_COLORS[CONE_YELLOW].m_Blue,
                                   CANDLE_LED_COLORS[CONE_YELLOW].m_White,
                                   0, NUMBER_OF_LEDS);
                strandColor = CONE_YELLOW;
                m_bIntakeCube = false;
                break;
            }
            case CONE_YELLOW:
            {
                // Set all LEDs to purple
                m_pCandle->SetLEDs(CANDLE_LED_COLORS[CUBE_PURPLE].m_Red,
                                   CANDLE_LED_COLORS[CUBE_PURPLE].m_Green,
                                   CANDLE_LED_COLORS[CUBE_PURPLE].m_Blue,
                                   CANDLE_LED_COLORS[CUBE_PURPLE].m_White,
                                   0, NUMBER_OF_LEDS);
                strandColor = CUBE_PURPLE;
                m_bIntakeCube = true;
                break;
            }
            default:
            {
                break;
            }
        }
    }

    // Variables for blinking the LEDs
    static Timer * pBlinkTimer = new Timer();
    static units::second_t lastTimeStamp = 0.0_s;
    static bool bLastStallValue = m_bIntakeStalled;
    static bool bOn = false;
    static bool bBlink = false;

    // Check if there was a state change on intake stalled
    if (bLastStallValue != m_bIntakeStalled)
    {
        // Changed to stalled, start the blink timer
        if (m_bIntakeStalled)
        {
            pBlinkTimer->Start();
            lastTimeStamp = pBlinkTimer->Get();
            bBlink = true;
            bOn = false;
        }
        else
        {
            // Changed to not stalled, stop the timer, LEDs back on
            pBlinkTimer->Stop();
            pBlinkTimer->Reset();
            bBlink = false;
            m_pCandle->SetLEDs(CANDLE_LED_COLORS[strandColor].m_Red,
                               CANDLE_LED_COLORS[strandColor].m_Green,
                               CANDLE_LED_COLORS[strandColor].m_Blue,
                               CANDLE_LED_COLORS[strandColor].m_White,
                               0, NUMBER_OF_LEDS);
        }
        bLastStallValue = m_bIntakeStalled;
    }

    // If we are supposed to blink and if the timer elapsed enough for a change
    if ((bBlink) && ((pBlinkTimer->Get() - lastTimeStamp) > 0.25_s))
    {
        // LEDs on
        if (bOn)
        {
            m_pCandle->SetLEDs(CANDLE_LED_COLORS[strandColor].m_Red,
                               CANDLE_LED_COLORS[strandColor].m_Green,
                               CANDLE_LED_COLORS[strandColor].m_Blue,
                               CANDLE_LED_COLORS[strandColor].m_White,
                               0, NUMBER_OF_LEDS);
        }
        // LEDs off
        else
        {
            m_pCandle->SetLEDs(0, 0, 0, 0, 0, NUMBER_OF_LEDS);
        }

        bOn = !bOn;
        lastTimeStamp = pBlinkTimer->Get();
    }

}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::PneumaticSequence
///
/// This method contains the main workflow for updating the
/// state of the pnemuatics on the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::PneumaticSequence()
{
    // @todo: Monitor other compressor API data?
    SmartDashboard::PutBoolean("Compressor status", m_pCompressor->IsEnabled());
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::SerialPortSequence
///
/// This method contains the main workflow for interaction with
/// the serial port.
///
////////////////////////////////////////////////////////////////
void YtaRobot::SerialPortSequence()
{
    /*
    // Check for any incoming transmissions, limit it to our read buffer size
    int32_t bytesReceived = m_pSerialPort->GetBytesReceived();
    bytesReceived = (bytesReceived > SERIAL_PORT_BUFFER_SIZE_BYTES) ? SERIAL_PORT_BUFFER_SIZE_BYTES : bytesReceived;

    // If we got data, read it
    if (bytesReceived > 0)
    {
        static_cast<void>(m_pSerialPort->Read(m_SerialPortBuffer, bytesReceived));

        // See if its a packet intended for us
        if (memcmp(m_SerialPortBuffer, SERIAL_PORT_PACKET_HEADER, SERIAL_PORT_PACKET_HEADER_SIZE_BYTES) == 0)
        {
            // Next character is the command.  Array indexing starts at zero, thus no +1 on the size bytes constant
            int32_t command = static_cast<int32_t>(m_SerialPortBuffer[SERIAL_PORT_PACKET_HEADER_SIZE_BYTES]) - ASCII_0_OFFSET;

            // Sanity check it
            if (command >= 0 && command <= 9)
            {
                RobotUtils::DisplayFormattedMessage("Received a valid packet, command: %d\n", command);
            }
            else
            {
                RobotUtils::DisplayFormattedMessage("Invalid command received: %d\n", command);
            }
        }

        RobotUtils::DisplayFormattedMessage(m_SerialPortBuffer);
    }
    m_SerialPortBuffer[0] = NULL_CHARACTER;
    */
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::I2cSequence
///
/// This method contains the main workflow for interaction with
/// the I2C bus.
///
////////////////////////////////////////////////////////////////
void YtaRobot::I2cSequence()
{
    static std::chrono::time_point<std::chrono::high_resolution_clock> currentTime;
    static std::chrono::time_point<std::chrono::high_resolution_clock> oldTime;
    currentTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = currentTime - oldTime;
    if (elapsed.count() > I2C_RUN_INTERVAL_MS)
    {
        RobotI2c::ManualTrigger();
        
        oldTime = currentTime;
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CameraSequence
///
/// This method handles camera related behavior.  See the
/// RobotCamera class for full details.
///
////////////////////////////////////////////////////////////////
void YtaRobot::CameraSequence()
{
    static bool bFullProcessing = false;
    
    // @note: Use std::chrono if precise time control is needed.
    
    // Check for any change in camera
    if (m_pDriveController->GetButtonState(SELECT_FRONT_CAMERA_BUTTON))
    {
        RobotCamera::SetCamera(RobotCamera::FRONT_USB);
    }
    else if (m_pDriveController->GetButtonState(SELECT_BACK_CAMERA_BUTTON))
    {
        RobotCamera::SetCamera(RobotCamera::BACK_USB);
    }
    else
    {
    }
    
    // Look for full processing to be enabled/disabled
    if (m_pDriveController->DetectButtonChange(CAMERA_TOGGLE_FULL_PROCESSING_BUTTON))
    {
        // Change state first, because the default is set before this code runs
        bFullProcessing = !bFullProcessing;
        RobotCamera::SetFullProcessing(bFullProcessing);
    }
    
    // Look for the displayed processed image to be changed
    if (m_pDriveController->DetectButtonChange(CAMERA_TOGGLE_PROCESSED_IMAGE_BUTTON))
    {
        RobotCamera::ToggleCameraProcessedImage();
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DriveMotorsCool
///
/// This method controls active or passive cooling of the drive
/// motors.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DriveMotorsCool()
{
    SmartDashboard::PutBoolean("Drive motor cooling", m_bCoolingDriveMotors);

    // Get the current time
    units::second_t currentTime = m_pDriveMotorCoolTimer->Get();

    // Set some values for the common logic based on whether or not cooling is currently active or passive
    units::second_t timerLimit = m_bCoolingDriveMotors ? DRIVE_MOTOR_COOL_ON_TIME : DRIVE_MOTOR_COOL_OFF_TIME;
    DoubleSolenoid::Value solenoidValue = m_bCoolingDriveMotors ? TALON_COOLING_OFF_SOLENOID_VALUE : TALON_COOLING_ON_SOLENOID_VALUE;

    // If the time until the next state change has elapsed
    if ((currentTime - m_LastDriveMotorCoolTime) > timerLimit)
    {
        // Change solenoid state, update control variables
        m_pTalonCoolingSolenoid->Set(solenoidValue);
        m_bCoolingDriveMotors = !m_bCoolingDriveMotors;
        m_LastDriveMotorCoolTime = currentTime;
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::MarioKartLights
///
/// This method will generate LED behavior that mimicks
/// drifting in Mario Kart.  It watches for non-zero rotational
/// inputs while driving/strafing.
///
////////////////////////////////////////////////////////////////
void YtaRobot::MarioKartLights(double translation, double strafe, double rotate)
{
    enum DriftState
    {
        DRIFT_OFF,
        DRIFT_BLUE,
        DRIFT_YELLOW,
        DRIFT_PURPLE,
        DRIFT_DISABLED
    };

    static DriftState driftState = DRIFT_OFF;
    static Timer * pDriftTimer = new Timer();
    static units::second_t lastTimeStamp = 0.0_s;
    static bool bLastDriftValue = false;
    static const double MIN_TRANSLATION_OR_STRAFE_VALUE = 0.25;

    // First check if the robot is moving in a way that qualifies for "drift"
    bool bDrifting = false;
    if (((std::abs(translation) > MIN_TRANSLATION_OR_STRAFE_VALUE) || (std::abs(strafe) > MIN_TRANSLATION_OR_STRAFE_VALUE)) && (std::abs(rotate) > 0.0))
    {
        bDrifting = true;
    }

    // See if there was a state change in drift status
    if (bDrifting != bLastDriftValue)
    {
        // Now drifting, previously were not
        if (bDrifting)
        {
            // Start the timer, clear the LEDs
            pDriftTimer->Start();
            lastTimeStamp = pDriftTimer->Get();
            m_pCandle->SetLEDs(0, 0, 0, 0, 0, NUMBER_OF_LEDS);
        }
        // Not drifting, previously were
        else
        {
            // Stop the timer, turn the LEDs back on
            pDriftTimer->Stop();
            pDriftTimer->Reset();
            driftState = DRIFT_OFF;
            if (m_bIntakeCube)
            {
                // Purple
                m_pCandle->SetLEDs(240, 73, 241, 0, 0, NUMBER_OF_LEDS);
            }
            else
            {
                // Yellow
                m_pCandle->SetLEDs(255, 240, 0, 0, 0, NUMBER_OF_LEDS);
            }
        }
        bLastDriftValue = bDrifting;
    }

    // B: {132, 240, 109}
    // Y: {255, 240, 0}
    // P: {240, 73, 241}
    // Candle has 8 LEDs in it that need to be bypassed
    if (bDrifting)
    {
        units::second_t currentTimeStamp = pDriftTimer->Get();
        switch (driftState)
        {
            case DRIFT_OFF:
            {
                // Transition to blue (total time 0.5 seconds)
                if ((currentTimeStamp - lastTimeStamp) > 0.5_s)
                {
                    // 2023 LED layout/spacing
                    m_pCandle->SetLEDs(132, 132, 255, 0, 8, 7);
                    m_pCandle->SetLEDs(132, 132, 255, 0, 47, 7);
                    m_pCandle->SetLEDs(132, 132, 255, 0, 54, 7);
                    m_pCandle->SetLEDs(132, 132, 255, 0, 93, 7);
                    driftState = DRIFT_BLUE;
                    lastTimeStamp = currentTimeStamp;
                }
                break;
            }
            case DRIFT_BLUE:
            {
                // Transition to yellow (total time 1.5 seconds)
                if ((currentTimeStamp - lastTimeStamp) > 1.0_s)
                {
                    // 2023 LED layout/spacing
                    m_pCandle->SetLEDs(255, 240, 0, 0, 15, 8);
                    m_pCandle->SetLEDs(255, 240, 0, 0, 39, 8);
                    m_pCandle->SetLEDs(255, 240, 0, 0, 61, 8);
                    m_pCandle->SetLEDs(255, 240, 0, 0, 85, 8);
                    driftState = DRIFT_YELLOW;
                    lastTimeStamp = currentTimeStamp;
                }
                break;
            }
            case DRIFT_YELLOW:
            {
                // Transition to purple (total time 2.5 seconds)
                if ((currentTimeStamp - lastTimeStamp) > 1.0_s)
                {
                    // 2023 LED layout/spacing
                    m_pCandle->SetLEDs(240, 73, 241, 0, 23, 8);
                    m_pCandle->SetLEDs(240, 73, 241, 0, 31, 8);
                    m_pCandle->SetLEDs(240, 73, 241, 0, 69, 8);
                    m_pCandle->SetLEDs(240, 73, 241, 0, 77, 8);
                    driftState = DRIFT_PURPLE;
                    lastTimeStamp = currentTimeStamp;
                }
                break;
            }
            case DRIFT_PURPLE:
            {
                // @todo: Implement some kind of 'burst' pattern
                driftState = DRIFT_DISABLED;
                break;
            }
            case DRIFT_DISABLED:
            default:
            {
                break;
            }
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::SwerveDriveSequence
///
/// This method contains the main workflow for swerve drive
/// control.  It will gather input from the drive joystick and
/// then filter those values to ensure they are past a certain
/// threshold (deadband) and generate the information to pass
/// on to the swerve drive system.
///
////////////////////////////////////////////////////////////////
void YtaRobot::SwerveDriveSequence()
{
    // Check for a switch between field relative and robot centric
    static bool bFieldRelative = true;
    if (m_pDriveController->DetectButtonChange(FIELD_RELATIVE_TOGGLE_BUTTON))
    {
        bFieldRelative = !bFieldRelative;
    }

    if (m_pDriveController->DetectButtonChange(ZERO_GYRO_YAW_BUTTON))
    {
        m_pSwerveDrive->ZeroGyroYaw();
    }

    // The GetDriveX() and GetDriveYInput() functions refer to ***controller joystick***
    // x and y axes.  Multiply by -1.0 here to keep the joystick input retrieval code common.
    double translationAxis = RobotUtils::Trim(m_pDriveController->GetDriveYInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    double strafeAxis = RobotUtils::Trim(m_pDriveController->GetDriveXInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    double rotationAxis = RobotUtils::Trim(m_pDriveController->GetDriveRotateInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);

    // Override normal control if a fine positioning request is made
    if (m_pDriveController->GetPovAsDirection() == Yta::Controller::PovDirections::POV_LEFT)
    {
        translationAxis = 0.0;
        strafeAxis = 0.0;
        rotationAxis = SWERVE_ROTATE_SLOW_SPEED;
    }
    else if (m_pDriveController->GetPovAsDirection() == Yta::Controller::PovDirections::POV_RIGHT)
    {
        translationAxis = 0.0;
        strafeAxis = 0.0;
        rotationAxis = -SWERVE_ROTATE_SLOW_SPEED;
    }
    else
    {
    }

    SmartDashboard::PutNumber("Strafe", strafeAxis);
    SmartDashboard::PutNumber("Translation", translationAxis);
    SmartDashboard::PutNumber("Rotation", rotationAxis);
    SmartDashboard::PutBoolean("Field Relative", bFieldRelative);

    // Notice that this is sending translation to X and strafe to Y, despite
    // the inputs coming from the opposite of what may be intuitive (strafe as X,
    // translation as Y).  See the comment in Translation2d.h about the robot
    // placed at origin facing the X-axis.  Forward movement increases X and left
    // movement increases Y.
    Translation2d translation = {units::meter_t(translationAxis), units::meter_t(strafeAxis)};

    // Update the swerve module states
    m_pSwerveDrive->SetModuleStates(translation, rotationAxis, bFieldRelative, true);
    MarioKartLights(translationAxis, strafeAxis, rotationAxis);

    // Display some useful information
    m_pSwerveDrive->UpdateSmartDashboard();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DriveControlSequence
///
/// This method contains the main workflow for drive control.
/// It will gather input from the drive joystick and then filter
/// those values to ensure they are past a certain threshold and
/// within range to send to the speed controllers.  Lastly it
/// will actually set the speed values.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DriveControlSequence()
{
    if (Yta::Drive::Config::DRIVE_MOTOR_COOLING_ENABLED)
    {
        DriveMotorsCool();
    }

    if (Yta::Drive::Config::DIRECTIONAL_ALIGN_ENABLED)
    {
        // Check for a directional align first
        DirectionalAlign();
        
        // If an align is in progress, do not accept manual driver input
        if (m_RobotDriveState == DIRECTIONAL_ALIGN)
        {
            return;
        }
    }

    if (Yta::Drive::Config::DIRECTIONAL_INCH_ENABLED)
    {
        // If a directional inch occurred, just return
        if (DirectionalInch())
        {
            return;
        }
    }

    if (Yta::Drive::Config::DRIVE_SWAP_ENABLED)
    {
        CheckForDriveSwap();
    }
    
    // Computes what the maximum drive speed could be
    double throttleControl = (m_pDriveController->GetThrottleControl() * DRIVE_THROTTLE_VALUE_RANGE) + DRIVE_THROTTLE_VALUE_BASE;

    // All the controllers are normalized
    // to represent the x and y axes with
    // the following values:
    //   -1
    //    |
    // -1---+1
    //    |
    //   +1
    
    // Get driver X/Y inputs
    double xAxisDrive = m_pDriveController->GetDriveXInput();
    double yAxisDrive = m_pDriveController->GetDriveYInput();

    if (RobotUtils::DEBUG_PRINTS)
    {
        SmartDashboard::PutNumber("x-axis input", xAxisDrive);
        SmartDashboard::PutNumber("y-axis input", yAxisDrive);
    }
    
    // Make sure axes inputs clear a certain threshold.  This will help to drive straight.
    xAxisDrive = RobotUtils::Trim((xAxisDrive * throttleControl), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    yAxisDrive = RobotUtils::Trim((yAxisDrive * throttleControl), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);

    // If the swap direction button was pressed, negate y value
    if (m_bDriveSwap)
    {
        yAxisDrive *= -1.0;
    }

    // By default, the drive equations cause the x-axis input
    // to be flipped when going reverse.  Correct that here,
    // if configured.  Remember, y-axis full forward is negative.
    if ((!Yta::Drive::Config::USE_INVERTED_REVERSE_CONTROLS) && (yAxisDrive > 0.0))
    {
        xAxisDrive *= -1.0;
    }
    
    if (Yta::Drive::Config::SLOW_DRIVE_ENABLED)
    {
        // Get the slow drive control joystick input
        double xAxisSlowDrive = m_pDriveController->GetAxisValue(DRIVE_SLOW_X_AXIS);
        xAxisSlowDrive = RobotUtils::Trim((xAxisSlowDrive * DRIVE_SLOW_THROTTLE_VALUE), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
        
        // If the normal x-axis drive is non-zero, use it.  Otherwise use the slow drive input, which could also be zero.
        xAxisDrive = (xAxisDrive != 0.0) ? xAxisDrive : xAxisSlowDrive;
    }
    
    // Filter motor speeds
    double leftSpeed = RobotUtils::Limit((LeftDriveEquation(xAxisDrive, yAxisDrive)), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    double rightSpeed = RobotUtils::Limit(RightDriveEquation(xAxisDrive, yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    
    // Set motor speed
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);

    if (RobotUtils::DEBUG_PRINTS)
    {
        SmartDashboard::PutNumber("Left drive speed", leftSpeed);
        SmartDashboard::PutNumber("Right drive speed", rightSpeed);
    }

    m_pLeftDriveMotors->DisplayStatusInformation();
    m_pRightDriveMotors->DisplayStatusInformation();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DirectionalInch
///
/// This method contains the main workflow for drive directional
/// inching.  Based on input direction, it will briefly move the
/// robot a slight amount in that direction.
///
////////////////////////////////////////////////////////////////
bool YtaRobot::DirectionalInch()
{
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    if (m_pDriveController->GetPovAsDirection() == Yta::Controller::PovDirections::POV_UP)
    {
        leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_FORWARD_SCALAR;
        rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_FORWARD_SCALAR;
    }
    else if (m_pDriveController->GetPovAsDirection() == Yta::Controller::PovDirections::POV_DOWN)
    {
        leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_REVERSE_SCALAR;
        rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_REVERSE_SCALAR;
    }
    else if (m_pDriveController->GetPovAsDirection() == Yta::Controller::PovDirections::POV_LEFT)
    {
        leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_REVERSE_SCALAR;
        rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_FORWARD_SCALAR;
    }
    else if (m_pDriveController->GetPovAsDirection() == Yta::Controller::PovDirections::POV_RIGHT)
    {
        leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_FORWARD_SCALAR;
        rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_REVERSE_SCALAR;
    }
    else
    {
    }
    
    if ((leftSpeed == 0.0) && (rightSpeed == 0.0))
    {
        // No directional inch input, just return
        return false;
    }
    
    // Start the timer
    m_pInchingDriveTimer->Reset();
    m_pInchingDriveTimer->Start();
    
    // Motors on
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);
    
    while (m_pInchingDriveTimer->Get() < INCHING_DRIVE_DELAY_S)
    {
    }
    
    // Motors back off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    // Stop the timer
    m_pInchingDriveTimer->Stop();
    m_pInchingDriveTimer->Reset();

    return true;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DirectionalAlign
///
/// This method contains the main workflow for automatically
/// aligning the robot to an angle based on input from the
/// driver.  The angles are relative to the robot at the start
/// of the match (when power is applied to the gyro and zero
/// is set).  The robot angle is reported as follows:
///
///     0
///     |
/// 270---90
///     |
///    180
///
/// The POV input is used to pick the angle to align to.  The
/// corresponding input on the d-pad maps 1:1 to the drawing.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DirectionalAlign()
{
    // Retain the last POV value between function invocations
    static int lastPovValue = -1;
    
    // Indicate whether or not a change between align/no align is allowed
    static bool bStateChangeAllowed = false;
    
    // Get the current POV value
    int povValue = m_pDriveController->GetPovValue();
    
    // Check if it changed since last function call
    if (povValue != lastPovValue)
    {
        // Something changed, figure out what
        
        // POV button was released
        if (povValue == -1)
        {
            // State change not allowed until next button press
            bStateChangeAllowed = false;
        }
        // POV button was pressed
        else if (lastPovValue == -1)
        {
            // State change allowed since button is now pressed
            bStateChangeAllowed = true;
        }
        // There was some change in the already pressed POV value, which doesn't matter
        else
        {
        }
    }
    
    const int POV_NORMALIZATION_ANGLE = 45;
    
    // Save off a new last POV value
    lastPovValue = povValue;
    
    // This alignment uses the following from the POV input:
    //
    // ///////////////////////
    // //   315      45     //
    // //     \  up  /      //
    // // left |    | right //
    // //     / down \      //
    // //   225      135    //
    // ///////////////////////
    //
    // The input value (0 -> 360) will be normalized such that
    // angle 315 is interpreted as zero.
    static int destinationAngle = -1;
    
    switch (m_RobotDriveState)
    {
        case MANUAL_CONTROL:
        {
            // Only start an align if a state change is allowed
            if (bStateChangeAllowed)
            {                
                // @todo: Switch this logic to use GetPovAsDirection()

                // This gives a value between 45 -> 405
                povValue += POV_NORMALIZATION_ANGLE;
                
                // Normalize between 0 -> 360 (maps 0:360 in to 45:360:0:45 out)
                if (povValue >= ANGLE_360_DEGREES)
                {
                    povValue -= ANGLE_360_DEGREES;
                }
                
                // Now at value between 0 -> 360, where:
                // 0 -> 89 = align up
                // 90 -> 179 = align right
                // 180 -> 269 = align down
                // 270 -> 359 = align left
                // Get a scalar multiplier to find the destination angle.
                // Making this volatile to prevent the compiler from trying
                // to optimize the division followed by multliplication of
                // the same constant.  Integer division is deliberate.
                // This gives a scalar multiplier of 0 -> 3
                volatile int degreeMultiplier = (povValue / ANGLE_90_DEGREES);
                
                // Find the destination angle.
                // This gives a value of 0, 90, 180 or 270
                destinationAngle = ANGLE_90_DEGREES * degreeMultiplier;
                
                // Read the starting angle
                RobotI2c::ManualTrigger();
                int startingAngle = static_cast<int>(GetGyroValue(BNO055));
                
                // Do some angle math to figure out which direction is faster to turn.
                // Examples:
                // Starting: 45, 180    Destination: 0, 90, 180, 270
                // 45 - 0 = 45          180 - 0 = 180
                // 45 - 90 = -45        180 - 90 = 90
                // 45 - 180 = -135      180 - 180 = 0
                // 45 - 270 = -225      180 - 270 = -90
                int angleDistance = startingAngle - destinationAngle;
                int absValueAngleDistance = std::abs(angleDistance);
                
                // Variables to indicate which way to turn
                bool bTurnLeft = false;
                bool bTurnRight = false;
                
                // Figure out which way to turn
                if (angleDistance > 0)
                {
                    // Target is to the left of where we are
                    bTurnLeft = true;
                }
                else
                {
                    // Target is to the right of where we are
                    bTurnRight = true;
                }

                // If the target distance is more than halfway around, it's actually faster to turn the other way 
                if (absValueAngleDistance > ANGLE_180_DEGREES)
                {
                    bTurnLeft = !bTurnLeft;
                    bTurnRight = !bTurnRight;
                }
                
                // The destination angle and direction is now known, time to do the move
                if (bTurnLeft)
                {
                    m_pLeftDriveMotors->Set(DIRECTIONAL_ALIGN_DRIVE_SPEED * LEFT_DRIVE_REVERSE_SCALAR);
                    m_pRightDriveMotors->Set(DIRECTIONAL_ALIGN_DRIVE_SPEED * RIGHT_DRIVE_FORWARD_SCALAR);
                }
                if (bTurnRight)
                {
                    m_pLeftDriveMotors->Set(DIRECTIONAL_ALIGN_DRIVE_SPEED * LEFT_DRIVE_FORWARD_SCALAR);
                    m_pRightDriveMotors->Set(DIRECTIONAL_ALIGN_DRIVE_SPEED * RIGHT_DRIVE_REVERSE_SCALAR);
                }
                
                // Start the safety timer
                m_pDirectionalAlignTimer->Start();

                // Indicate a state change is not allowed until POV release
                bStateChangeAllowed = false;
                
                // Indicate a directional align is in process
                m_RobotDriveState = DIRECTIONAL_ALIGN;
            }
            
            break;
        }
        case DIRECTIONAL_ALIGN:
        {   
            // Force update gyro value
            RobotI2c::ManualTrigger();
            
            // Three conditions for stopping the align:
            // 1. Destination angle is reached
            // 2. Safety timer expires
            // 3. User cancels the operation
            // @todo: Is it a problem that (destinationAngle - 1) can be negative when angle == zero?
            int currentAngle = static_cast<int>(GetGyroValue(BNO055));
            if (((currentAngle >= (destinationAngle - 1)) && (currentAngle <= (destinationAngle + 1))) ||
                (m_pDirectionalAlignTimer->Get() > DIRECTIONAL_ALIGN_MAX_TIME_S) ||
                (bStateChangeAllowed))
            {
                // Motors off
                m_pLeftDriveMotors->Set(OFF);
                m_pRightDriveMotors->Set(OFF);
                
                // Reset the safety timer
                m_pDirectionalAlignTimer->Stop();
                m_pDirectionalAlignTimer->Reset();
                
                // Clear this just to be safe
                destinationAngle = -1;
                
                // Indicate a state change is not allowed until POV release
                bStateChangeAllowed = false;
                
                // Align done, back to manual control
                m_RobotDriveState = MANUAL_CONTROL;
            }
            
            break;
        }
        default:
        {
            break;
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DisabledInit
///
/// The disabled init method.  This method is called once each
/// time the robot enters disabled mode.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DisabledInit()
{
    RobotUtils::DisplayMessage("DisabledInit called.");

    // @todo: Shut off the limelight LEDs?
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::ARRAY_OFF);
    
    // All motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);

    // Motor cooling off
    m_pTalonCoolingSolenoid->Set(TALON_COOLING_OFF_SOLENOID_VALUE);

    // Turn the rainbow animation back on    
    m_pCandle->Animate(m_RainbowAnimation);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DisabledPeriodic
///
/// The disabled control method.  This method is called
/// periodically while the robot is disabled.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DisabledPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_DISABLED);
}



////////////////////////////////////////////////////////////////
/// @method main
///
/// Execution start for the robt.
///
////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<YtaRobot>();
}
#endif
