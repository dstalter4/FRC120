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
/// Copyright (c) 2022 Youth Technology Academy
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
    m_pDriveController                  (new DriveControllerType(DRIVE_CONTROLLER_MODEL, DRIVE_JOYSTICK_PORT)),
    m_pAuxController                    (new AuxControllerType(AUX_CONTROLLER_MODEL, AUX_JOYSTICK_PORT)),
    m_pLeftDriveMotors                  (new TalonMotorGroup<TalonFX>(NUMBER_OF_LEFT_DRIVE_MOTORS, LEFT_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW, FeedbackDevice::CTRE_MagEncoder_Relative)),
    m_pRightDriveMotors                 (new TalonMotorGroup<TalonFX>(NUMBER_OF_RIGHT_DRIVE_MOTORS, RIGHT_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW, FeedbackDevice::CTRE_MagEncoder_Relative)),
    m_pIntakeMotors                     (new TalonMotorGroup<TalonFX>(TWO_MOTORS, INTAKE_MOTORS_CAN_START_ID, MotorGroupControlMode::INVERSE, FeedbackDevice::None)),
    m_pFeederMotors                     (new TalonMotorGroup<TalonFX>(TWO_MOTORS, FEEDER_MOTORS_CAN_START_ID, MotorGroupControlMode::INVERSE, FeedbackDevice::None)),
    m_pShooterMotors                    (new TalonMotorGroup<TalonFX>(TWO_MOTORS, SHOOTER_MOTORS_CAN_START_ID, MotorGroupControlMode::INVERSE, FeedbackDevice::None)),
    m_pWinchMotor                       (new TalonFX(WINCH_MOTOR_CAN_ID)),
    m_pLedsEnableRelay                  (new Relay(LEDS_ENABLE_RELAY_ID)),
    m_pRedLedRelay                      (new Relay(RED_LED_RELAY_ID)),
    m_pGreenLedRelay                    (new Relay(GREEN_LED_RELAY_ID)),
    m_pBlueLedRelay                     (new Relay(BLUE_LED_RELAY_ID)),
    m_pDebugOutput                      (new DigitalOutput(DEBUG_OUTPUT_DIO_CHANNEL)),
    m_pIntakeSolenoid                   (new DoubleSolenoid(PneumaticsModuleType::CTREPCM, INTAKE_SOLENOID_FWD_CHANNEL, INTAKE_SOLENOID_REV_CHANNEL)),
    m_pHangerSolenoid                   (new DoubleSolenoid(PneumaticsModuleType::CTREPCM, HANGER_SOLENOID_FWD_CHANNEL, HANGER_SOLENOID_REV_CHANNEL)),
    m_pTalonCoolingSolenoid             (new DoubleSolenoid(PneumaticsModuleType::CTREPCM, TALON_COOLING_SOLENOID_FWD_CHANNEL, TALON_COOLING_SOLENOID_REV_CHANNEL)),
    m_pCompressor                       (new Compressor(PneumaticsModuleType::CTREPCM)),
    m_pShootMotorSpinUpTimer            (new Timer()),
    m_pIntakePulseTimer                 (new Timer()),
    m_pDriveMotorCoolTimer              (new Timer()),
    m_pMatchModeTimer                   (new Timer()),
    m_pAutonomousTimer                  (new Timer()),
    m_pInchingDriveTimer                (new Timer()),
    m_pDirectionalAlignTimer            (new Timer()),
    m_pSafetyTimer                      (new Timer()),
    m_pAccelerometer                    (new BuiltInAccelerometer),
    m_pAdxrs450Gyro                     (nullptr),
    m_Bno055Angle                       (),
    m_CameraThread                      (RobotCamera::LimelightThread),
    m_SerialPortBuffer                  (),
    m_pSerialPort                       (new SerialPort(SERIAL_PORT_BAUD_RATE, SerialPort::kMXP, SERIAL_PORT_NUM_DATA_BITS, SerialPort::kParity_None, SerialPort::kStopBits_One)),
    //m_I2cThread                         (RobotI2c::I2cThread),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_RobotDriveState                   (MANUAL_CONTROL),
    m_AllianceColor                     (DriverStation::GetAlliance()),
    m_bDriveSwap                        (false),
    m_bUnjamming                        (false),
    m_bShotInProgress                   (false),
    m_bIntakePulsingEnabled             (true),
    m_bIntakePulsing                    (false),
    m_bCoolingDriveMotors               (true),
    m_ShootingSpeed                     (SHOOTER_75_MOTOR_SPEED),
    m_FeederSpeed                       (FEEDER_MOTOR_SPEED),
    m_LastDriveMotorCoolTime            (0_s),
    m_LastIntakePulseTime               (0_s),
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
    
    RobotUtils::DisplayFormattedMessage("The drive forward axis is: %d\n", Yta::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.RIGHT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive reverse axis is: %d\n", Yta::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive left/right axis is: %d\n", Yta::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_X_AXIS);
    
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
    m_CameraThread.detach();
    //m_I2cThread.detach();
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

    // Start with motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    m_pIntakeMotors->Set(OFF);
    m_pFeederMotors->Set(OFF);
    m_pShooterMotors->Set(OFF);
    m_pWinchMotor->Set(ControlMode::PercentOutput, OFF);
    
    // Configure brake or coast for the motors
    m_pLeftDriveMotors->SetBrakeMode();
    m_pRightDriveMotors->SetBrakeMode();
    m_pIntakeMotors->SetBrakeMode();
    m_pIntakeMotors->GetMotorObject()->SetNeutralMode(NeutralMode::Coast);
    m_pFeederMotors->SetBrakeMode();
    m_pShooterMotors->SetCoastMode();
    m_pWinchMotor->SetNeutralMode(NeutralMode::Brake);

    // Solenoids to known state
    m_pIntakeSolenoid->Set(INTAKE_UP_SOLENOID_VALUE);
    m_pTalonCoolingSolenoid->Set(TALON_COOLING_OFF_SOLENOID_VALUE);
    m_pHangerSolenoid->Set(DoubleSolenoid::kOff);
    
    // Tare encoders
    m_pLeftDriveMotors->TareEncoder();
    m_pRightDriveMotors->TareEncoder();
    
    // Enable LEDs, but keep them off for now
    m_pLedsEnableRelay->Set(LEDS_ENABLED);
    m_pRedLedRelay->Set(LEDS_OFF);
    m_pGreenLedRelay->Set(LEDS_OFF);
    m_pBlueLedRelay->Set(LEDS_OFF);
    
    // Stop/clear any timers, just in case
    m_pShootMotorSpinUpTimer->Stop();
    m_pShootMotorSpinUpTimer->Reset();
    m_pIntakePulseTimer->Stop();
    m_pIntakePulseTimer->Reset();
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

    // 2022: Maybe help with overheating?  Doing it here to leave autonomous on brake.
    m_pLeftDriveMotors->SetCoastMode();
    m_pRightDriveMotors->SetCoastMode();
    
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

    m_ShootingSpeed = SHOOTER_75_MOTOR_SPEED;
    m_FeederSpeed = FEEDER_MOTOR_SPEED;

    // Start the intake pulse timer, which will just free run
    m_pIntakePulseTimer->Reset();
    m_pIntakePulseTimer->Start();
    m_LastIntakePulseTime = 0_s;
    m_bIntakePulsingEnabled = true;
    m_bIntakePulsing = true;
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

    DriveControlSequence();

    IntakeSequence();

    FeedAndShootSequence();

    UnjamSequence();

    HangSequence();

    PneumaticSequence();

    //SerialPortSequence();
    
    //I2cSequence();
    
    //CameraSequence();

    //LedSequence();

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
    // Give the drive team some state information
    SmartDashboard::PutNumber("Shooter speed", m_ShootingSpeed);
    SmartDashboard::PutNumber("Feeder speed", m_FeederSpeed);
    SmartDashboard::PutBoolean("Unjamming", m_bUnjamming);
    SmartDashboard::PutBoolean("Shot in progress", m_bShotInProgress);
    SmartDashboard::PutBoolean("Intake pulse enabled", m_bIntakePulsingEnabled);
    SmartDashboard::PutBoolean("Intake pulsing", m_bIntakePulsing);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::IntakeSequence
///
/// Controls the intake.
///
////////////////////////////////////////////////////////////////
void YtaRobot::IntakeSequence()
{
    // Only spin the intake if we are not unjamming
    if (!m_bUnjamming)
    {
        // Check intake buttons
        if (m_pDriveController->GetButtonState(DRIVE_INTAKE_BUTTON) || m_pAuxController->GetButtonState(AUX_INTAKE_BUTTON))
        {
            // Normal request, intake down, motor on
            m_pIntakeSolenoid->Set(INTAKE_DOWN_SOLENOID_VALUE);
            m_pIntakeMotors->Set(INTAKE_MOTOR_SPEED);
        }
        else
        {
            // If a normal intake request is not in process, the intake should always be up
            m_pIntakeSolenoid->Set(INTAKE_UP_SOLENOID_VALUE);

            // No intake request, not shooting, intake motor off
            if (!m_bShotInProgress)
            {
                m_pIntakeMotors->Set(OFF);
            }
            // No intake request, but shooting
            else
            {
                // Check if this shooting speed should pulse
                if (m_bIntakePulsingEnabled)
                {
                    units::second_t currentTime = m_pIntakePulseTimer->Get();
                    if ((currentTime - m_LastIntakePulseTime) > INTAKE_PULSE_TIME_S)
                    {
                        if (m_bIntakePulsing)
                        {
                            // When pulsing, first state is off
                            m_pIntakeMotors->GetMotorObject()->Set(ControlMode::PercentOutput, OFF);
                        }
                        else
                        {
                            m_pIntakeMotors->GetMotorObject()->Set(ControlMode::PercentOutput, INTAKE_MOTOR_SPEED);
                        }
                        m_bIntakePulsing = !m_bIntakePulsing;
                        m_LastIntakePulseTime = currentTime;
                    }
                }
                // This shooting speed is continuous
                else
                {
                    // Spin only the intake motor in the feeder chassis
                    m_pIntakeMotors->GetMotorObject()->Set(ControlMode::PercentOutput, INTAKE_MOTOR_SPEED);
                }
            }
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::FeedAndShootSequence
///
/// Moves cargo through the feeder and shoots it.  This method
/// has two primary responsibilities: It first determines the
/// speed at which to shoot and then utilizes a basic state
/// machine to shoot.  Shooting requires a delay initially to
/// allow the shooter motors to get up to speed.
///
////////////////////////////////////////////////////////////////
void YtaRobot::FeedAndShootSequence()
{
    // First determine and set the shooter speed
    if (m_pDriveController->GetButtonState(DRIVE_SET_SHOOT_75_BUTTON) || m_pAuxController->GetButtonState(AUX_SET_SHOOT_75_BUTTON))
    {
        m_ShootingSpeed = SHOOTER_75_MOTOR_SPEED;
        m_FeederSpeed = FEEDER_MOTOR_SPEED;
        m_bIntakePulsingEnabled = true;
    }
    else if (m_pDriveController->GetButtonState(DRIVE_SET_SHOOT_65_BUTTON) || m_pAuxController->GetButtonState(AUX_SET_SHOOT_65_BUTTON))
    {
        m_ShootingSpeed = SHOOTER_65_MOTOR_SPEED;
        m_FeederSpeed = FEEDER_MOTOR_SPEED;
        m_bIntakePulsingEnabled = true;
    }
    else if (m_pDriveController->GetButtonState(DRIVE_SET_SHOOT_60_BUTTON) || m_pAuxController->GetButtonState(AUX_SET_SHOOT_60_BUTTON))
    {
        m_ShootingSpeed = SHOOTER_60_MOTOR_SPEED;
        m_FeederSpeed = FEEDER_MOTOR_SPEED;
        m_bIntakePulsingEnabled = true;
    }
    else if (m_pAuxController->GetButtonState(AUX_SET_SHOOT_35_BUTTON))
    {
        m_ShootingSpeed = SHOOTER_35_MOTOR_SPEED;
        m_FeederSpeed = FEEDER_QUICK_MOTOR_SPEED;
        m_bIntakePulsingEnabled = false;
    }
    else
    {
    }

    enum ShootState
    {
        NO_SHOT,
        SPIN_UP,
        SHOOTING
    };
    static ShootState shootState = NO_SHOT;

    // Only feed and shoot if we are not unjamming
    if (!m_bUnjamming)
    {
        // Look for a request to shoot
        if (m_pDriveController->GetButtonState(DRIVE_SHOOT_BUTTON) || (m_pAuxController->GetAxisValue(AUX_SHOOT_AXIS) != 0))
        {
            // Simple state machine to control shooting
            switch (shootState)
            {
                // State which spins up the shooter motors before enabling the feeder
                case NO_SHOT:
                {
                    // Nothing fancy, just start a timer
                    m_pShooterMotors->Set(m_ShootingSpeed);
                    m_pShootMotorSpinUpTimer->Start();
                    shootState = SPIN_UP;
                    break;
                }
                // State that checks if the shooter motors are at speed
                case SPIN_UP:
                {
                    // Is the time delay up?
                    if (m_pShootMotorSpinUpTimer->Get() > SHOOTING_SPIN_UP_DELAY_S)
                    {
                        // Start the feeders too
                        m_pFeederMotors->Set(m_FeederSpeed);
                        m_pShootMotorSpinUpTimer->Stop();
                        m_pShootMotorSpinUpTimer->Reset();
                        m_bIntakePulsing = true;
                        m_bShotInProgress = true;
                        shootState = SHOOTING;
                    }
                    break;
                }
                case SHOOTING:
                default:
                {
                    // The shooting state requires no changes
                    break;
                }
            }
        }
        // No request, motors off
        else
        {
            m_pFeederMotors->Set(OFF);
            m_pShooterMotors->Set(OFF);
            m_bShotInProgress = false;
            shootState = NO_SHOT;
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::UnjamSequence
///
/// Provides a way to move cargo out of the feeder chassis.
///
////////////////////////////////////////////////////////////////
void YtaRobot::UnjamSequence()
{
    if (m_pAuxController->GetAxisValue(AUX_UNJAM_AXIS) > 0.0)
    {
        m_bUnjamming = true;
        m_bShotInProgress = false;
        m_pIntakeSolenoid->Set(INTAKE_DOWN_SOLENOID_VALUE);
        m_pIntakeMotors->Set(-INTAKE_MOTOR_SPEED);
        m_pFeederMotors->Set(-FEEDER_QUICK_MOTOR_SPEED);
        m_pShooterMotors->Set(OFF);
    }
    else
    {
        m_bUnjamming = false;
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::HangSequence
///
/// Controls the hanging mechanisms.
///
////////////////////////////////////////////////////////////////
void YtaRobot::HangSequence()
{
    Yta::Controller::PovDirections povDirection = m_pAuxController->GetPovAsDirection();

    if (povDirection == Yta::Controller::PovDirections::POV_UP)
    {
        //SmartDashboard::PutNumber("Aux POV input", 1);
        m_pWinchMotor->Set(ControlMode::PercentOutput, ON);
    }
    else if (povDirection == Yta::Controller::PovDirections::POV_DOWN)
    {
        //SmartDashboard::PutNumber("Aux POV input", -1);
        m_pWinchMotor->Set(ControlMode::PercentOutput, -ON);
    }
    else
    {
        //SmartDashboard::PutNumber("Aux POV input", 0);
        m_pWinchMotor->Set(ControlMode::PercentOutput, OFF);
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
    SmartDashboard::PutBoolean("Compressor status", m_pCompressor->Enabled());
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

    // 2022: The talons seem to overheat at the end of the match.
    //       Constantly cool them for the last 40s.
    units::second_t matchModeTime = m_pMatchModeTimer->Get();
    if ((matchModeTime > DRIVE_MOTOR_COOL_ALWAYS_ON_TIME) && (matchModeTime < (DRIVE_MOTOR_COOL_ALWAYS_ON_TIME + 40_s)))
    {
        m_pTalonCoolingSolenoid->Set(TALON_COOLING_ON_SOLENOID_VALUE);
        m_bCoolingDriveMotors = true;
        return;
    }

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
    if (DRIVE_MOTOR_COOLING_ENABLED)
    {
        DriveMotorsCool();
    }

    if (DIRECTIONAL_ALIGN_ENABLED)
    {
        // Check for a directional align first
        DirectionalAlign();
        
        // If an align is in progress, do not accept manual driver input
        if (m_RobotDriveState == DIRECTIONAL_ALIGN)
        {
            return;
        }
    }

    if (DIRECTIONAL_INCH_ENABLED)
    {
        // If a directional inch occurred, just return
        if (DirectionalInch())
        {
            return;
        }
    }

    if (DRIVE_SWAP_ENABLED)
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
    if ((!USE_INVERTED_REVERSE_CONTROLS) && (yAxisDrive > 0.0))
    {
        xAxisDrive *= -1.0;
    }
    
    if (SLOW_DRIVE_ENABLED)
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

    // Retrieve motor temperatures
    double leftTemp = ConvertCelsiusToFahrenheit(m_pLeftDriveMotors->GetMotorObject()->GetTemperature());
    double rightTemp = ConvertCelsiusToFahrenheit(m_pRightDriveMotors->GetMotorObject()->GetTemperature());

    if (RobotUtils::DEBUG_PRINTS)
    {
        SmartDashboard::PutNumber("Left drive speed", leftSpeed);
        SmartDashboard::PutNumber("Right drive speed", rightSpeed);
        SmartDashboard::PutNumber("Left temperature (F)", leftTemp);
        SmartDashboard::PutNumber("Right temperature (F)", rightTemp);
    }
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

    // 2022 Note: These are deliberately inverted for inching while facing the hub!

    if (m_pDriveController->GetPovAsDirection() == Yta::Controller::PovDirections::POV_DOWN)
    {
        leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_FORWARD_SCALAR;
        rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_FORWARD_SCALAR;
    }
    else if (m_pDriveController->GetPovAsDirection() == Yta::Controller::PovDirections::POV_UP)
    {
        leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_REVERSE_SCALAR;
        rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_REVERSE_SCALAR;
    }
    else if (m_pDriveController->GetPovAsDirection() == Yta::Controller::PovDirections::POV_RIGHT)
    {
        leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_REVERSE_SCALAR;
        rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_FORWARD_SCALAR;
    }
    else if (m_pDriveController->GetPovAsDirection() == Yta::Controller::PovDirections::POV_LEFT)
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
    
    // All motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);

    // Motor cooling off
    m_pTalonCoolingSolenoid->Set(TALON_COOLING_OFF_SOLENOID_VALUE);
    
    // Even though 'Disable' shuts off the relay signals, explitily turn the LEDs off
    m_pLedsEnableRelay->Set(LEDS_DISABLED);
    m_pRedLedRelay->Set(LEDS_OFF);
    m_pGreenLedRelay->Set(LEDS_OFF);
    m_pBlueLedRelay->Set(LEDS_OFF);
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
