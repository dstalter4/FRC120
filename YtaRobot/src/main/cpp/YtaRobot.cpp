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
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <cctype>                       // for alphanumeric character checking
#include <cstddef>                      // for nullptr
#include <cstring>                      // for memset

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                 // for class declaration (and other headers)
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
    m_AutonomousPositionChooser         (),
    m_pDriveController                  (new DriveControllerType(DRIVE_CONTROLLER_MODEL, DRIVE_JOYSTICK_PORT)),
    m_pAuxController                    (new AuxControllerType(AUX_CONTROLLER_MODEL, AUX_JOYSTICK_PORT)),
    m_RioCanBus                         (RIO_CAN_BUS_NAME),
    m_CanivoreBus                       (CANIVORE_CAN_BUS_NAME),
    m_pPigeon                           (new Pigeon2(PIGEON_CAN_ID, m_CanivoreBus)),
    m_pSwerveDrive                      (new SwerveDrive(m_pPigeon, m_GetCanBusReferenceLambda)),
    m_pDifferentialDrive                (new DifferentialDrive(m_GetCanBusReferenceLambda)),
    m_pIntakeRollersMotor               (new TalonFxMotorController(INTAKE_ROLLERS_MOTOR_CAN_ID, m_RioCanBus)),
    m_pIntakeAngleMotor                 (new TalonFxMotorController(INTAKE_ANGLE_MOTOR_CAN_ID, m_RioCanBus)),
    m_pFeederMotor                      (new TalonFxMotorController(FEEDER_MOTOR_CAN_ID, m_RioCanBus)),
    m_pInjectorMotor                    (new TalonFxMotorController(INJECTOR_MOTOR_CAN_ID, m_RioCanBus)),
    m_pShooterMotors                    (new TalonMotorGroup<TalonFX, TalonFXConfiguration>("Shooter motors", TWO_MOTORS, SHOOTER_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW_INVERSE, NeutralModeValue::Coast, m_RioCanBus)),
    m_pHoodMotor                        (new TalonFxMotorController(HOOD_MOTOR_CAN_ID, m_RioCanBus)),
    m_pHangMotor                        (new TalonFxMotorController(HANG_MOTOR_CAN_ID, m_RioCanBus)),
    m_pLedController                    (new YtaLedController(CANDLE_CAN_ID, NUMBER_OF_LEDS, m_RioCanBus)),
    m_pDebugOutput                      (new DigitalOutput(DEBUG_OUTPUT_DIO_CHANNEL)),
    m_pHoodLeftServoActuator            (new PWM(HOOD_SERVO_LEFT_ACTUATOR_PWM_CHANNEL)),
    m_pHoodRightServoActuator           (new PWM(HOOD_SERVO_RIGHT_ACTUATOR_PWM_CHANNEL)),
    m_pCompressor                       (new Compressor(PneumaticsModuleType::CTREPCM)),
    m_pIntakeCanCoder                   (new CANcoder(INTAKE_CANCODER_CAN_ID, m_RioCanBus)),
    m_pHoodCanCoder                     (new CANcoder(HOOD_CANCODER_CAN_ID, m_RioCanBus)),
    m_pMatchModeTimer                   (new Timer()),
    m_pRobotProgramTimer                (new Timer()),
    m_pLimelightCamera                  (new LimelightCamera("limelight")),
    m_pLimelightFound                   (false),
    m_ShooterMotorSpeed                 (SHOOTER_MOTOR_SPEED),
    m_InjectorMotorSpeed                (INJECTOR_MOTOR_SPEED),
    m_IntakeAngleDegrees                (INTAKE_UP_ANGLE_DEGREES),
    m_IntakeAngleOffsetDegrees          (0.0_deg),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_AllianceColor                     (DriverStation::GetAlliance()),
    m_bIntakeLowered                    (false),
    m_bIntakeSequenceActive             (false),
    m_bShootSequenceActive              (false),
    m_bShotInProgress                   (false),
    m_bRioPinsStable                    (false),
    m_bCameraAlignInProgress            (false),
    m_HeartBeat                         (0U)
{
    RobotUtils::DisplayMessage("Robot constructor.");
    
    // LiveWindow is not used
    LiveWindow::SetEnabled(false);

    // Signal logger is not used
    SignalLogger::EnableAutoLogging(false);
    
    // Set the autonomous options
    // @todo: Update these outside the constructor?
    m_AutonomousChooser.SetDefaultOption(AUTO_ROUTINE_1_STRING, AUTO_ROUTINE_1_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_2_STRING, AUTO_ROUTINE_2_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_NO_ROUTINE_STRING, AUTO_NO_ROUTINE_STRING);
    m_AutonomousChooser.AddOption(AUTO_TEST_ROUTINE_STRING, AUTO_TEST_ROUTINE_STRING);
    SmartDashboard::PutData("Autonomous Modes", &m_AutonomousChooser);

    m_AutonomousPositionChooser.SetDefaultOption("Just shoot", "Just shoot");
    m_AutonomousPositionChooser.AddOption("Shoot and move", "Shoot and move");
    SmartDashboard::PutData("Center Action", &m_AutonomousPositionChooser);

    // WCP Parameters for L16-R Actuonix Linear Actuators
    // max, deadbandMax, center, deadbandMin, min (units are microseconds)
    m_pHoodLeftServoActuator->SetBounds(2000.0_us, 1800.0_us, 1500.0_us, 1200.0_us, 1000.0_us);
    m_pHoodRightServoActuator->SetBounds(2000.0_us, 1800.0_us, 1500.0_us, 1200.0_us, 1000.0_us);

    // Start the free running timer
    m_pRobotProgramTimer->Reset();
    m_pRobotProgramTimer->Start();
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

    // Attempt to locate the limelight.  The called function has a
    // search timeout.  If it isn't found, this will have to be
    // called again later.
    m_pLimelightFound = m_pLimelightCamera->FindAndSetNetworkTable();
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

    // @todo: Read and display sensor values for calibration when not enabled
    // @note: From testing, smart dashboard prints of sensor values do give real time data.
    CheckIfRioPinsAreStable();
    UpdateSmartDashboard();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CheckIfRioPinsAreStable
///
/// Waits for any sensors on the robot that route to the RIO
/// to stabilize for accurate readings.
///
////////////////////////////////////////////////////////////////
void YtaRobot::CheckIfRioPinsAreStable()
{
    // This is the logic to wait to take PWM based sensor readings until the RIO is ready.
    // The behavior of the RIO is that it measures how many microseconds the signal is high
    // every second.  This requires waiting to get stable readings.
    // See https://github.com/wpilibsuite/allwpilib/issues/5284 for some related info.
    static units::time::second_t enabledTimeStamp = 0.0_s;
    units::time::second_t currentTimeStamp = m_pRobotProgramTimer->Get();
    if (DriverStation::IsEnabled() && (!m_bRioPinsStable))
    {
        // If the robot was just enabled (in any mode)
        if (enabledTimeStamp == 0.0_s)
        {
            // Set the start time stamp
            enabledTimeStamp = currentTimeStamp;
        }

        // Now check if enough time has passed for the RIO pins to have stabilized
        static constexpr const units::time::second_t RIO_DUTY_CYCLE_ENCODER_STARTUP_DELAY = 2.0_s;
        if ((currentTimeStamp - enabledTimeStamp) > RIO_DUTY_CYCLE_ENCODER_STARTUP_DELAY)
        {
            // Example encoder configuration algorithm

            //double encoderValue = m_pEncoder->Get();
            //units::angle::degree_t encoderValueDegrees(encoderValue * ANGLE_360_DEGREES);

            // This is the delta between the current mechanism position and the desired starting position (or zero point)
            //units::angle::degree_t startingOffsetDegrees = encoderValueDegrees - STARTING_POSITION_ENCODER_VALUE;
            //std::printf("startingOffsetDegrees (start): %f\n", startingOffsetDegrees.value());

            // If the starting offset is negative, we crossed over the absolute encoder boundary
            // We give a tolerance of five degrees in case the mechanism is near where we want to start
            // @todo: Does this need to check for very small readings below zero?
            // @todo: Boundary conditions here will be difficult
            //if (startingOffsetDegrees < ENCODER_BOUNDARY_TOLERANCE_DEGREES)
            //{
                // the 0/1 boundary is 360, so subtract the starting position to see how many degrees were up to that point
                // Add in the absolute value of the overage, which was negative
                //startingOffsetDegrees = (units::angle::degree_t(ANGLE_360_DEGREES) - STARTING_POSITION_ENCODER_VALUE) + encoderValueDegrees;
            //}

            // At this point we have the angle we want relative to zero
            //(void)m_pMotor->GetMotorObject()->GetConfigurator().SetPosition(startingOffsetDegrees);
            //std::printf("encoderValue: %f\n", encoderValue);
            //std::printf("encoderValueDegrees: %f\n", encoderValueDegrees.value());
            //std::printf("startingOffsetDegrees (final): %f\n", startingOffsetDegrees.value());

            m_bRioPinsStable = true;
        }
    }
    else
    {
        // Set the enabled time stamp back to zero until the robot is enabled again
        enabledTimeStamp = 0.0_s;

        // m_bRioPinsStable exists for the life of the program.  Once we have a stable
        // reading acquired, we don't need to do it again until the robot program restarts.
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

    /*
    // @todo_phoenix6: Update the example for the new API.
    // Example configuration
    TalonFXConfiguration talonConfig;
    talonConfig.slot0.kP = 0.08;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.3;
    talonConfig.slot0.kF = 0.0;
    talonConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    talonConfig.integratedSensorOffsetDegrees = 0.0;
    talonConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    talonConfig.peakOutputForward = 1.0;
    talonConfig.peakOutputReverse = 1.0;
    talonConfig.slot0.closedLoopPeakOutput = 0.10;

    TalonFX * pTalon = new TalonFX(0xFF);
    pTalon->ConfigFactoryDefault();
    pTalon->ConfigAllSettings(talonConfig);
    pTalon->SetSelectedSensorPosition(0);
    const StatorCurrentLimitConfiguration INTAKE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG = {true, 5.0, 50.0, 5.0};
    pTalon->ConfigStatorCurrentLimit(INTAKE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG);
    */

    // Some notes about applying motor configurations:
    // - The classes/structs in YtaTalon.hpp have motor configuration objects in them.
    // - Declaring stack local or class scope configuration objects are *separate and
    //   distinct* from the configuration objects in the YtaTalon.hpp classes/structs.
    // - If a stack local or class scope configuration is applied, it will overwrite
    //   the configuration stored in the device.
    // - Calling the methods provided by YtaTalon.hpp *never* update the configuration
    //   objects in the classes/structs.  To update those objects, retrieve the objects
    //   via things like GetMotorConfiguration().
    // - The classes/structs in YtaTalon.hpp provide ApplyConfiguration() routines.
    //   These can be used to directly apply a stack local or class scope configuration,
    //   or to apply an updated configuration when the configuration objects were directly
    //   modified.  Keep the notes above in mind when calling them.
    // - The ApplyConfiguration() method for motor groups will default to applying the
    //   configuration to all motors unless a specific CAN ID is given.  The configuration
    //   applied to each motor in the group is the *saved configuration* for that specific
    //   motor.  It may be different for each motor, depending on the robot code.
    // - Configurations can be applied to the whole configuration object type, or to
    //   sub-types only (e.g. TalonFXConfiguration vs. CurrentLimitsConfigs), as
    //   ApplyConfiguration() is overloaded.  The template version only applies stack
    //   local or class scope configs, so remember the notes above.  Right now the
    //   template version is disabled, so don't call it.
    // - Thank CTRE for all this.  Instead of letting the config be a member of the motor
    //   object class with simple getter/setters, it's separate and overly complex.

    // Example configurations

    // Configure a motor group (only needs to be applied to the lead motor of the group)
    // Brake mode was set when the motor group was constructed
    //(void)m_pMotors->GetMotorConfiguration(MOTORS_CAN_START_ID)->Feedback.WithSensorToMechanismRatio(12.0 / 1.0);
    //(void)m_pMotors->GetMotorConfiguration(MOTORS_CAN_START_ID)->Slot0.WithKP(18.0).WithKI(0.0).WithKD(0.1);
    //m_pMotors->ApplyConfiguration(MOTORS_CAN_START_ID);
    //(void)m_pMotors->GetMotorObject(MOTORS_CAN_START_ID)->GetConfigurator().SetPosition(0.0_tr);

    // Configure a single motor
    //(void)m_pMotor->GetMotorConfiguration()->MotorOutput.WithNeutralMode(NeutralModeValue::Brake);
    //(void)m_pMotor->GetMotorConfiguration()->Feedback.WithSensorToMechanismRatio(135.0 / 1.0);
    //(void)m_pMotor->GetMotorConfiguration()->Slot0.WithKP(18.0).WithKI(0.0).WithKD(0.1);
    //(void)m_pMotor->GetMotorObject()->GetConfigurator().SetPosition(0.0_tr);
    //m_pMotor->ApplyConfiguration();

    // Configure shooter motor in case WithVelocity() is called
    m_pShooterMotors->GetMotorConfiguration()->Slot0.WithKP(10.0).WithKI(0.0).WithKD(0.0);
    m_pShooterMotors->ApplyConfiguration();





    // Configure CANCoder
    // CANCoder: 0.835449 (300.76164_deg) is full up, 0.0.501221 (180.43956_deg) is full down, currently moving as CW+
    // Starting position = 0.831299 (299.26764_deg)
    // 120.32208_deg range of motion, FX is only showing ~110_deg range of motion?
    constexpr const units::angle::degree_t INTAKE_STARTING_ANGLE_CANCODER_REF = 298.0_deg;
    CANcoderConfiguration canCoderConfig;
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0_tr;
    //canCoderConfig.MagnetSensor.SensorDirection = InvertedValue::CounterClockwise_Positive;
    (void)m_pIntakeCanCoder->GetConfigurator().Apply(canCoderConfig);
    
    // Configure the intake angle motor
    // Ratio is 50:1
    (void)m_pIntakeAngleMotor->GetMotorConfiguration()->MotorOutput.WithNeutralMode(NeutralModeValue::Brake);
    (void)m_pIntakeAngleMotor->GetMotorConfiguration()->Feedback.WithSensorToMechanismRatio(50.0 / 1.0);
    (void)m_pIntakeAngleMotor->GetMotorConfiguration()->Slot0.WithKP(18.0).WithKI(0.0).WithKD(0.1);
    //(void)m_pIntakeAngleMotor->GetMotorConfiguration()->SoftwareLimitSwitch.WithForwardSoftLimitThreshold(10.0_deg).WithReverseSoftLimitThreshold(-120.0_deg);
    //(void)m_pIntakeAngleMotor->GetMotorConfiguration()->SoftwareLimitSwitch.WithForwardSoftLimitEnable(true).WithReverseSoftLimitEnable(true);
    m_pIntakeAngleMotor->ApplyConfiguration();

    units::angle::degree_t intakeCanCoderDegrees = m_pIntakeCanCoder->GetAbsolutePosition().GetValue();
    units::angle::degree_t intakeAngleDelta = intakeCanCoderDegrees - INTAKE_STARTING_ANGLE_CANCODER_REF;
    SmartDashboard::PutNumber("Intake delta", intakeAngleDelta.value());

    // If the delta is negative, the intake is below where we want it (down further).
    //    Down further means a negative angle position for the FX.
    // If the delta is positive, the intake is above where we want it (up higher).
    //    Up higher means a positive angle position for the FX.
    units::angle::turn_t intakeSetPositionTurns = intakeAngleDelta;
    (void)m_pIntakeAngleMotor->GetMotorObject()->GetConfigurator().SetPosition(intakeSetPositionTurns);
    // Expected starting position is up, but in local testing, may sometimes be down
    if (intakeAngleDelta < -50.0_deg)
    {
        m_bIntakeLowered = true;
        m_IntakeAngleDegrees = intakeSetPositionTurns;
    }





    // Hood CANcoder: No measurements yet for full up/down.
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0_tr;
    //canCoderConfig.MagnetSensor.SensorDirection = InvertedValue::CounterClockwise_Positive;
    (void)m_pHoodCanCoder->GetConfigurator().Apply(canCoderConfig);

    (void)m_pHoodMotor->GetMotorConfiguration()->MotorOutput.WithNeutralMode(NeutralModeValue::Brake);
    m_pHoodMotor->ApplyConfiguration();
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

    // Configure the motor controllers
    ConfigureMotorControllers();

    // Stop/clear any timers, just in case
    // @todo: Make this a dedicated function.
    m_pMatchModeTimer->Stop();
    m_pMatchModeTimer->Reset();
    
    // Just in case constructor was called before these were set (likely the case)
    m_AllianceColor = DriverStation::GetAlliance();
    m_pLedController->SetAllianceColor(m_AllianceColor.value());

    // Set the LEDs to the alliance color
    m_pLedController->SetLedsToAllianceColor();

    // Set the limelight priority ID
    m_pLimelightCamera->SetPriorityId(LimelightCamera::TaggedFieldElement::ELEMENT_HUB_CENTER, m_AllianceColor.value());

    // Clear the debug output pin
    m_pDebugOutput->Set(false);

    // Reset the heartbeat
    m_HeartBeat = 0U;

    // Point the swerve modules straight
    m_pSwerveDrive->HomeModules();
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
    
    // Autonomous should have left things in a known state, but just in case, clear everything.
    CommandScheduler::GetInstance().CancelAll();
    InitialStateSetup();

    // Start the mode timer for teleop
    m_pMatchModeTimer->Start();
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
        if (!m_bCameraAlignInProgress)
        {
            SwerveDriveSequence();
        }
    }
    else
    {
        DifferentialDriveControlSequence();
    }

    IntakeSequence();
    ShootSequence();
    //HoodSequence();
    //HangSequence();
    CheckForManualAdjust();

    //PneumaticSequence();
    
    CameraSequence();

    // These only do things if their configs are enabled.
    // See YtaLed.hpp and YtaMusic.hpp for the controls.
    LedSequence();
    MusicSequence();
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
    units::time::second_t matchTime = 0.0_s;
    double batteryVoltage = DriverStation::GetBatteryVoltage();
    std::string gameData = DriverStation::GetGameSpecificMessage();

    if (DriverStation::IsFMSAttached())
    {
        matchTime = DriverStation::GetMatchTime();
    }
    else
    {
        matchTime = m_pMatchModeTimer->Get();
    }

    struct HubShift
    {
        bool m_Transition;
        bool m_bShift1;
        bool m_bShift2;
        bool m_bShift3;
        bool m_bShift4;
        bool m_EndGame;
    };
    constexpr const HubShift ACTIVE_FIRST = {true, true, false, true, false, true};
    constexpr const HubShift INACTIVE_FIRST = {true, false, true, false, true, true};

    static bool bGotGameData = false;
    static HubShift allianceHubShift;

    // Look for the game data to be ready
    if (!bGotGameData)
    {
        bool bInactiveFirst = false;
        if (!gameData.empty())
        {
            // For some reason the game data is who is *inactive* first (instead of active)
            bInactiveFirst = (((gameData.at(0U) == 'R') && (m_AllianceColor == DriverStation::kRed)) ||
                              ((gameData.at(0U) == 'B') && (m_AllianceColor == DriverStation::kBlue)));
        }

        allianceHubShift = bInactiveFirst ? INACTIVE_FIRST : ACTIVE_FIRST;
        bGotGameData = true;
    }

    // Auto: 20_s, Teleop: 110_s, End Game: 30_s (Driver Control Total: 140_s or 2m20s)
    constexpr const units::time::second_t TRANSITION_END_TIME_S = 130_s;
    constexpr const units::time::second_t SHIFT_1_END_TIME_S = 105_s;
    constexpr const units::time::second_t SHIFT_2_END_TIME_S = 80_s;
    constexpr const units::time::second_t SHIFT_3_END_TIME_S = 55_s;
    constexpr const units::time::second_t SHIFT_4_END_TIME_S = 30_s;

    bool bHubActive = false;
    units::time::second_t shiftTime = 0.0_s;
    if (matchTime > TRANSITION_END_TIME_S)
    {
        bHubActive = allianceHubShift.m_Transition;
        shiftTime = matchTime - TRANSITION_END_TIME_S;
    }
    else if (matchTime > SHIFT_1_END_TIME_S)
    {
        bHubActive = allianceHubShift.m_bShift1;
        shiftTime = matchTime - SHIFT_1_END_TIME_S;
    }
    else if (matchTime > SHIFT_2_END_TIME_S)
    {
        bHubActive = allianceHubShift.m_bShift2;
        shiftTime = matchTime - SHIFT_2_END_TIME_S;
    }
    else if (matchTime > SHIFT_3_END_TIME_S)
    {
        bHubActive = allianceHubShift.m_bShift3;
        shiftTime = matchTime - SHIFT_3_END_TIME_S;
    }
    else if (matchTime > SHIFT_4_END_TIME_S)
    {
        bHubActive = allianceHubShift.m_bShift4;
        shiftTime = matchTime - SHIFT_4_END_TIME_S;
    }
    else
    {
        bHubActive = allianceHubShift.m_EndGame;
        shiftTime = matchTime;
    }

    // Give the drive team some state information
    SmartDashboard::PutBoolean("RIO pins stable", m_bRioPinsStable);
    SmartDashboard::PutBoolean("Limelight found", m_pLimelightFound);
    SmartDashboard::PutNumber("Battery voltage", batteryVoltage);
    SmartDashboard::PutNumber("Match time", matchTime.value());
    SmartDashboard::PutNumber("Shift time", shiftTime.value());
    SmartDashboard::PutBoolean("Hub active", bHubActive);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CheckForManualAdjust
///
/// Checks for requests to manually adjust robot control values.
///
////////////////////////////////////////////////////////////////
void YtaRobot::CheckForManualAdjust()
{
    constexpr const char * MANUAL_ADJUST_STATE_STRINGS[] = {"Shooter speed", "Injector speed", "Intake angle"};
    enum ManualAdjustState : uint32_t
    {
        SHOOTER_SPEED,
        INJECTOR_SPEED,
        INTAKE_ANGLE,
        INVALID_CHECK
    };
    static ManualAdjustState manualCheckState = INJECTOR_SPEED;
    uint32_t stateAsUint = static_cast<uint32_t>(manualCheckState);

    // Update the manual check state, if needed
    if (m_pAuxController->DetectButtonChange(AUX_MANUAL_ADJUST_TOGGLE_BUTTON))
    {
        stateAsUint++;
        if (stateAsUint == static_cast<uint32_t>(INVALID_CHECK))
        {
            stateAsUint = 0UL;
        }
        manualCheckState = static_cast<ManualAdjustState>(stateAsUint);
    }

    // Check for manual adjustment
    if (m_pAuxController->GetButtonState(AUX_MANUAL_ADJUST_BUTTON))
    {
        switch (manualCheckState)
        {
            case SHOOTER_SPEED:
            {
                if (m_pAuxController->DetectPovChange(AUX_MANUAL_ADJUST_UP_POV_DIRECTION))
                {
                    m_ShooterMotorSpeed += SHOOTER_MOTOR_SPEED_STEP;
                }
                else if (m_pAuxController->DetectPovChange(AUX_MANUAL_ADJUST_DOWN_POV_DIRECTION))
                {
                    m_ShooterMotorSpeed -= SHOOTER_MOTOR_SPEED_STEP;
                }
                else
                {
                }
                break;
            }
            case INJECTOR_SPEED:
            {
                if (m_pAuxController->DetectPovChange(AUX_MANUAL_ADJUST_UP_POV_DIRECTION))
                {
                    m_InjectorMotorSpeed += INJECTOR_MOTOR_SPEED_STEP;
                }
                else if (m_pAuxController->DetectPovChange(AUX_MANUAL_ADJUST_DOWN_POV_DIRECTION))
                {
                    m_InjectorMotorSpeed -= INJECTOR_MOTOR_SPEED_STEP;
                }
                else
                {
                }
                break;
            }
            case INTAKE_ANGLE:
            {
                if (m_pAuxController->DetectPovChange(AUX_MANUAL_ADJUST_UP_POV_DIRECTION))
                {
                    m_IntakeAngleOffsetDegrees += INTAKE_MANUAL_ADJUST_STEP_DEGREES;
                }
                else if (m_pAuxController->DetectPovChange(AUX_MANUAL_ADJUST_DOWN_POV_DIRECTION))
                {
                    m_IntakeAngleOffsetDegrees -= INTAKE_MANUAL_ADJUST_STEP_DEGREES;
                }
                else
                {
                }
                break;
            }
            default:
            {
                break;
            }
        }
    }

    // Display what we are manually adjusting
    SmartDashboard::PutString("Manual adjust state", MANUAL_ADJUST_STATE_STRINGS[stateAsUint]);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::IntakeSequence
///
/// Main sequence for ball intake logic.
///
////////////////////////////////////////////////////////////////
void YtaRobot::IntakeSequence()
{
    /*
    static VelocityVoltage vv(0.0_tps);
    static bool bInit = false;
    if (!bInit)
    {
        (void)vv.WithSlot(0);
        (void)m_pIntakeRollersMotor->m_MotorConfiguration.Slot0.WithKS(0.1).WithKV(0.12).WithKP(0.11).WithKI(0.0).WithKD(0.1);
        (void)m_pIntakeRollersMotor->ApplyConfiguration();
        bInit = true;
    }
    static bool bOff = true;
    units::angular_velocity::turns_per_second_t targetTps = 0.0_tps;
    if (m_pAuxController->DetectButtonChange(AUX_INTAKE_BUTTON))
    {
        bOff = !bOff;
        targetTps = bOff ? 0.0_tps : 50.0_tps;
    }
    m_pIntakeRollersMotor->m_pTalonFx->SetControl(vv.WithVelocity(targetTps));
    return;
    */

    if (m_pAuxController->DetectButtonChange(AUX_INTAKE_UP_DOWN_BUTTON))
    {
        m_bIntakeLowered = !m_bIntakeLowered;
        if (m_bIntakeLowered)
        {
            m_IntakeAngleDegrees = INTAKE_DOWN_ANGLE_DEGREES;
        }
        else
        {
            m_IntakeAngleDegrees = INTAKE_UP_ANGLE_DEGREES;
        }
    }

    // Check for intake in/out control
    if (m_pAuxController->GetButtonState(AUX_INTAKE_BUTTON))
    {
        m_pIntakeRollersMotor->SetDutyCycle(-INTAKE_ROLLERS_MOTOR_SPEED);
        m_bIntakeSequenceActive = false;
    }
    else if (m_pAuxController->GetButtonState(AUX_EJECT_BUTTON))
    {
        // Ejecting also moves the feeder and injector
        m_pIntakeRollersMotor->SetDutyCycle(INTAKE_ROLLERS_MOTOR_SPEED);
        m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
        m_pInjectorMotor->SetDutyCycle(m_InjectorMotorSpeed);
        m_bIntakeSequenceActive = true;
    }
    else
    {
        m_pIntakeRollersMotor->SetDutyCycle(0.0);
        m_bIntakeSequenceActive = false;

        // Only shut off the feeder/injector if not active
        if (!m_bShootSequenceActive)
        {
            m_pFeederMotor->SetDutyCycle(0.0);
            m_pInjectorMotor->SetDutyCycle(0.0);
        }
    }

    m_pIntakeAngleMotor->SetPositionVoltage(m_IntakeAngleDegrees + m_IntakeAngleOffsetDegrees);

    // Display some information on the intake position
    SmartDashboard::PutBoolean("Intake lowered", m_bIntakeLowered);
    SmartDashboard::PutNumber("Intake angle", m_IntakeAngleDegrees.value());
    SmartDashboard::PutNumber("Intake angle offset", m_IntakeAngleOffsetDegrees.value());
    SmartDashboard::PutNumber("Intake FX", units::angle::degree_t(m_pIntakeAngleMotor->GetMotorObject()->GetPosition().GetValue()).value());
    SmartDashboard::PutNumber("Intake CANcoder", units::angle::degree_t(m_pIntakeCanCoder->GetAbsolutePosition().GetValue()).value());
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::ShootSequence
///
/// Main sequence for ball shooting logic.
///
////////////////////////////////////////////////////////////////
void YtaRobot::ShootSequence()
{
    static Timer shootTimer;
    static units::time::second_t shootTimeStamp = 0.0_s;
    static bool bManualRamp = false;
    //static VelocityVoltage shooterMotorVv(0.0_tps);
    //constexpr units::angular_velocity::turns_per_second_t SHOOTER_MOTOR_TARGET_TPS = 3500.0_tps / 60.0;

    // First check for a manual ramp up request
    if (m_pAuxController->GetAxisValue(AUX_RAMP_UP_AXIS) > JOYSTICK_AXIS_INPUT_DEAD_BAND)
    {
        // Not shooting, but communicate to the intake logic that the injector is in use.
        m_bShootSequenceActive = true;
        m_bShotInProgress = true;
        bManualRamp = true;
        m_pShooterMotors->SetDutyCycle(m_ShooterMotorSpeed);
        //m_pShooterMotors->GetMotorObject()->SetControl(shooterMotorVv.WithVelocity(SHOOTER_MOTOR_TARGET_TPS));
    }
    else
    {
        bManualRamp = false;
    }

    if (m_pAuxController->GetAxisValue(AUX_SHOOT_AXIS) > JOYSTICK_AXIS_INPUT_DEAD_BAND)
    {
        if (!m_bShotInProgress)
        {
            shootTimer.Reset();
            shootTimer.Start();
            m_pShooterMotors->SetDutyCycle(m_ShooterMotorSpeed);
            //m_pShooterMotors->GetMotorObject()->SetControl(shooterMotorVv.WithVelocity(SHOOTER_MOTOR_TARGET_TPS));
            shootTimeStamp = shootTimer.Get();
            m_bShootSequenceActive = true;
            m_bShotInProgress = true;
        }
        else if (bManualRamp || ((shootTimer.Get() - shootTimeStamp) > SHOOTER_RAMP_UP_TIME_S))
        {
            m_pFeederMotor->SetDutyCycle(-FEEDER_MOTOR_SPEED);
            m_pInjectorMotor->SetDutyCycle(-m_InjectorMotorSpeed);
        }
        else
        {
        }
    }
    else if (m_pAuxController->GetButtonState(AUX_UNCLOG_BUTTON))
    {
        // Not shooting, but communicate to the intake logic that the injector is in use.
        m_bShootSequenceActive = true;
        m_bShotInProgress = false;
        m_pShooterMotors->SetDutyCycle(0.0);
        //m_pShooterMotors->GetMotorObject()->SetControl(shooterMotorVv.WithVelocity(0.0_tps));
        m_pFeederMotor->SetDutyCycle(0.0);
        m_pInjectorMotor->SetDutyCycle(m_InjectorMotorSpeed);
    }
    else
    {
        if (!bManualRamp)
        {
            m_bShootSequenceActive = false;
            m_bShotInProgress = false;
            m_pShooterMotors->SetDutyCycle(0.0);
            //m_pShooterMotors->GetMotorObject()->SetControl(shooterMotorVv.WithVelocity(0.0_tps));
        }

        // Make sure the intake isn't active before shutting these off
        if (!m_bIntakeSequenceActive)
        {
            m_pFeederMotor->SetDutyCycle(0.0);
            m_pInjectorMotor->SetDutyCycle(0.0);
        }
    }

    SmartDashboard::PutNumber("Injector speed", m_InjectorMotorSpeed);
    SmartDashboard::PutBoolean("Shooting", m_bShotInProgress);
    SmartDashboard::PutNumber("Shooter speed", m_ShooterMotorSpeed);
    units::angular_velocity::turns_per_second_t shooterMotorTps = m_pShooterMotors->GetMotorObject()->GetVelocity().GetValue();
    double shooterMotorRpm = shooterMotorTps.value() * 60.0;
    SmartDashboard::PutNumber("Shooter RPM", shooterMotorRpm);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::HoodSequence
///
/// Main sequence for hood control logic.
///
////////////////////////////////////////////////////////////////
void YtaRobot::HoodSequence()
{
    static double hoodServoValue = 0.0;
    constexpr const double HOOD_SERVO_STEP_VALUE = 0.1;
    constexpr const double HOOD_SERVO_UPPER_LIMIT = 1.0;
    constexpr const double HOOD_SERVO_LOWER_LIMIT = 0.0;

    if (m_pAuxController->DetectButtonChange(AUX_HOOD_UP_BUTTON))
    {
        hoodServoValue += HOOD_SERVO_STEP_VALUE;
    }
    else if (m_pAuxController->DetectButtonChange(AUX_HOOD_DOWN_BUTTON))
    {
        hoodServoValue -= HOOD_SERVO_STEP_VALUE;
    }
    else
    {
    }

    hoodServoValue = RobotUtils::Limit(hoodServoValue, HOOD_SERVO_UPPER_LIMIT, HOOD_SERVO_LOWER_LIMIT);
    m_pHoodLeftServoActuator->SetPosition(hoodServoValue);
    m_pHoodRightServoActuator->SetPosition(hoodServoValue);

    SmartDashboard::PutNumber("Hood servo", hoodServoValue);
    SmartDashboard::PutNumber("Hood CANcoder", units::angle::degree_t(m_pHoodCanCoder->GetAbsolutePosition().GetValue()).value());
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::HangSequence
///
/// Main sequence for hanging logic.
///
////////////////////////////////////////////////////////////////
void YtaRobot::HangSequence()
{
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
    if (Yta::Led::Config::MORSE_CODE_ENABLED)
    {
        m_pLedController->BlinkMorseCodePattern();
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::MusicSequence
///
/// This method contains the main workflow for controlling
/// any hardware capable of playing music (e.g. TalonFX).
///
////////////////////////////////////////////////////////////////
void YtaRobot::MusicSequence()
{
    if (Yta::Music::Config::PLAYING_TONES_ENABLED)
    {
        // Note: The control mode for the motors can only be one
        //       thing at a time.  Using a motor for acutal motion
        //       will not work at the same time as playing tones.
        static bool bPlayMusic = false;
        if (m_pDriveController->DetectButtonChange(PLAY_MUSIC_BUTTON))
        {
            bPlayMusic = true;
        }

        if (bPlayMusic)
        {
            // Note: Change the nullptr to the TalonFX object for the
            //       motor to play tones on!  This code will crash otherwise.

            // When PlayTones() returns false, the music is over
            if (!YtaMusicController::PlayTones(nullptr))
            {
                bPlayMusic = false;
            }
        }
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
/// @method YtaRobot::CameraSequence
///
/// This method handles camera related behavior.  See the
/// camera classes for full details.
///
////////////////////////////////////////////////////////////////
void YtaRobot::CameraSequence()
{
    if (m_pDriveController->GetButtonState(DRIVE_ALIGN_WITH_CAMERA_BUTTON))
    {
        m_bCameraAlignInProgress = true;
        m_pLimelightCamera->AlignToTargetSwerve(m_LimelightDriveLambda, m_pPigeon->GetYaw().GetValue());
    }
    else
    {
        m_bCameraAlignInProgress = false;
    }

    m_pLimelightCamera->UpdateSmartDashboard();
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

    if (m_pDriveController->DetectButtonChange(REZERO_SWERVE_BUTTON))
    {
        m_pSwerveDrive->ZeroGyroYaw();
        m_pSwerveDrive->RecalibrateModules();
        m_pSwerveDrive->HomeModules();
    }

    if (m_pDriveController->DetectButtonChange(LOCK_SWERVE_WHEELS_BUTTON))
    {
        m_pSwerveDrive->LockWheels();
    }

    static Timer swerveJogTimer;
    static units::time::second_t lastJogTimeStamp = 0.0_s;
    static bool bJogInit = false;
    static bool bJogFirstDirection = false;

    if (!bJogInit)
    {
        swerveJogTimer.Reset();
        swerveJogTimer.Start();
        bJogInit = true;
    }

    if (m_pDriveController->GetButtonState(JOG_SWERVE_BUTTON))
    {
        constexpr const double JOG_SWERVE_ROTATE_SPEED = 0.10;
        constexpr const units::time::second_t JOG_CHANGE_DIRECTION_TIME_S = 0.25_s;

        units::time::second_t currentTimeStamp = swerveJogTimer.Get();
        if ((currentTimeStamp - lastJogTimeStamp) > JOG_CHANGE_DIRECTION_TIME_S)
        {
            bJogFirstDirection = !bJogFirstDirection;
            lastJogTimeStamp = currentTimeStamp;
        }

        if (bJogFirstDirection)
        {
            Translation2d translation = {units::meter_t(0.0), units::meter_t(0.0)};
            m_pSwerveDrive->SetModuleStates(translation, JOG_SWERVE_ROTATE_SPEED, bFieldRelative, true);
        }
        else
        {
            Translation2d translation = {units::meter_t(0.0), units::meter_t(0.0)};
            m_pSwerveDrive->SetModuleStates(translation, -JOG_SWERVE_ROTATE_SPEED, bFieldRelative, true);
        }

        return;
    }

    // The GetDriveX() and GetDriveY() functions refer to ***controller joystick***
    // x and y axes.  Multiply by -1.0 here to keep the joystick input retrieval code common.
    double translationAxis = RobotUtils::Trim(m_pDriveController->GetDriveYInput() * -1.0, DRIVE_TRIM_UPPER_LIMIT, DRIVE_TRIM_LOWER_LIMIT);
    double strafeAxis = RobotUtils::Trim(m_pDriveController->GetDriveXInput() * -1.0, DRIVE_TRIM_UPPER_LIMIT, DRIVE_TRIM_LOWER_LIMIT);
    double rotationAxis = RobotUtils::Trim(m_pDriveController->GetDriveRotateInput() * -1.0, DRIVE_TRIM_UPPER_LIMIT, DRIVE_TRIM_LOWER_LIMIT);

    // Override normal control if a fine positioning request is made
    switch (m_pDriveController->GetPovAsDirection())
    {
        case DRIVE_CONTROLS_SWERVE_FORWARD_SLOW_POV:
        {
            translationAxis = SWERVE_DRIVE_SLOW_SPEED;
            strafeAxis = 0.0;
            rotationAxis = 0.0;
            break;
        }
        case DRIVE_CONTROLS_SWERVE_REVERSE_SLOW_POV:
        {
            translationAxis = -SWERVE_DRIVE_SLOW_SPEED;
            strafeAxis = 0.0;
            rotationAxis = 0.0;
            break;
        }
        case DRIVE_CONTROLS_SWERVE_LEFT_OR_CCW_SLOW_POV:
        {
            // Left/right POV control can either toggle strafe or rotation
            translationAxis = 0.0;
            strafeAxis = (Yta::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (0.0) : (SWERVE_DRIVE_SLOW_SPEED);
            rotationAxis = (Yta::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (SWERVE_ROTATE_SLOW_SPEED) : (0.0);
            break;
        }
        case DRIVE_CONTROLS_SWERVE_RIGHT_OR_CW_SLOW_POV:
        {
            // Left/right POV control can either toggle strafe or rotation
            translationAxis = 0.0;
            strafeAxis = (Yta::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (0.0) : (-SWERVE_DRIVE_SLOW_SPEED);
            rotationAxis = (Yta::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (-SWERVE_ROTATE_SLOW_SPEED) : (0.0);
            break;
        }
        default:
        {
            break;
        }
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
    constexpr const double SWERVE_ROTATE_2026_LIMIT_FACTOR = 0.80;
    m_pSwerveDrive->SetModuleStates(translation, rotationAxis * SWERVE_ROTATE_2026_LIMIT_FACTOR, bFieldRelative, true);

    // Update the odometry
    m_pSwerveDrive->UpdateOdometry();

    // Pretend to Mario Kart drift
    if (Yta::Led::Config::MARIO_KART_DRIFT_ENABLED)
    {
        m_pLedController->MarioKartLights(translationAxis, strafeAxis, rotationAxis);
    }

    // Display some useful information
    m_pSwerveDrive->UpdateSmartDashboard();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DifferentialDriveControlSequence
///
/// This method contains the main workflow for drive control.
/// It will gather input from the drive joystick and then filter
/// those values to ensure they are past a certain threshold and
/// within range to send to the speed controllers.  Lastly it
/// will actually set the speed values.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DifferentialDriveControlSequence()
{
    static DifferentialDrive::DriveControlInputs driveControlInputs;
    std::function<const DifferentialDrive::DriveControlInputs & ()> getDriveControlInputsLambda = [this]() -> const DifferentialDrive::DriveControlInputs &
    {
        driveControlInputs.m_xAxis = RobotUtils::Trim(m_pDriveController->GetDriveXInput(), DRIVE_TRIM_UPPER_LIMIT, DRIVE_TRIM_LOWER_LIMIT);
        driveControlInputs.m_yAxis = RobotUtils::Trim(m_pDriveController->GetDriveYInput(), DRIVE_TRIM_UPPER_LIMIT, DRIVE_TRIM_LOWER_LIMIT);
        driveControlInputs.m_xAxisSlow = RobotUtils::Trim(m_pDriveController->GetAxisValue(DRIVE_SLOW_X_AXIS), DRIVE_TRIM_UPPER_LIMIT, DRIVE_TRIM_LOWER_LIMIT);
        driveControlInputs.m_yAxisSlow = RobotUtils::Trim(m_pDriveController->GetAxisValue(DRIVE_SLOW_Y_AXIS), DRIVE_TRIM_UPPER_LIMIT, DRIVE_TRIM_LOWER_LIMIT);
        driveControlInputs.m_PovValue = m_pDriveController->GetPovValue();
        driveControlInputs.m_Throttle = m_pDriveController->GetThrottleControl();

        if (Yta::Drive::Config::DRIVE_SWAP_ENABLED)
        {
            // Check if the driver pushed the button to have
            // forward be reverse and vice versa
            if (m_pDriveController->DetectButtonChange(DRIVE_SWAP_BUTTON))
            {
                driveControlInputs.m_bDriveSwap = !driveControlInputs.m_bDriveSwap;
            }
        }

        Yta::Controller::PovDirections povDirection = m_pDriveController->GetPovAsDirection();
        switch (povDirection)
        {
            case DRIVE_CONTROLS_INCH_FORWARD_POV:
            {
                driveControlInputs.m_InchingDirection = DifferentialDrive::RobotDirection::ROBOT_FORWARD;
                break;
            }
            case DRIVE_CONTROLS_INCH_REVERSE_POV:
            {
                driveControlInputs.m_InchingDirection = DifferentialDrive::RobotDirection::ROBOT_REVERSE;
                break;
            }
            case DRIVE_CONTROLS_INCH_LEFT_POV:
            {
                driveControlInputs.m_InchingDirection = DifferentialDrive::RobotDirection::ROBOT_LEFT;
                break;
            }
            case DRIVE_CONTROLS_INCH_RIGHT_POV:
            {
                driveControlInputs.m_InchingDirection = DifferentialDrive::RobotDirection::ROBOT_RIGHT;
                break;
            }
            default:
            {
                driveControlInputs.m_InchingDirection = DifferentialDrive::RobotDirection::ROBOT_NO_DIRECTION;
                break;
            }
        }

        return driveControlInputs;
    };

    m_pDifferentialDrive->DriveSequence(getDriveControlInputsLambda);
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

    // Turn the rainbow animation back on
    m_pLedController->SetAnimation(YtaLedController::LedAnimation::LED_RAINBOW_ANIMATION);
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
