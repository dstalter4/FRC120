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
/// Copyright (c) 2024 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <cctype>                       // for alphanumeric character checking
#include <cstddef>                      // for nullptr
#include <cstring>                      // for memset

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                 // for class declaration (and other headers)
#include "RobotCamera.hpp"              // for interacting with cameras
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
    m_AutoSwerveDirections              (),
    m_pDriveController                  (new DriveControllerType(DRIVE_CONTROLLER_MODEL, DRIVE_JOYSTICK_PORT)),
    m_pAuxController                    (new AuxControllerType(AUX_CONTROLLER_MODEL, AUX_JOYSTICK_PORT)),
    m_pPigeon                           (new Pigeon2(PIGEON_CAN_ID, "canivore-120")),
    m_pSwerveDrive                      (new SwerveDrive(m_pPigeon)),
    m_pLeftDriveMotors                  (new ArcadeDriveTalonFxType("Left Drive", TWO_MOTORS, LEFT_DRIVE_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW, NeutralModeValue::Brake, true)),
    m_pRightDriveMotors                 (new ArcadeDriveTalonFxType("Right Drive", TWO_MOTORS, RIGHT_DRIVE_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW, NeutralModeValue::Brake, true)),
    m_pCandle                           (new CANdle(CANDLE_CAN_ID, "canivore-120")),
    m_RainbowAnimation                  ({1, 0.5, 308}),
    m_pDebugOutput                      (new DigitalOutput(DEBUG_OUTPUT_DIO_CHANNEL)),
    m_pCompressor                       (new Compressor(PneumaticsModuleType::CTREPCM)),
    m_pMatchModeTimer                   (new Timer()),
    m_pSafetyTimer                      (new Timer()),
    m_CameraThread                      (RobotCamera::LimelightThread),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_RobotDriveState                   (MANUAL_CONTROL),
    m_AllianceColor                     (DriverStation::GetAlliance()),
    m_bDriveSwap                        (false),
    m_bCameraAlignInProgress            (false),
    m_HeartBeat                         (0U)
{
    RobotUtils::DisplayMessage("Robot constructor.");
    
    // LiveWindow is not used
    LiveWindow::SetEnabled(false);
    
    // Set the autonomous options
    // @todo: Update these outside the constructor?
    m_AutonomousChooser.SetDefaultOption(AUTO_ROUTINE_1_STRING, AUTO_ROUTINE_1_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_2_STRING, AUTO_ROUTINE_2_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_NO_ROUTINE_STRING, AUTO_NO_ROUTINE_STRING);
    m_AutonomousChooser.AddOption(AUTO_TEST_ROUTINE_STRING, AUTO_TEST_ROUTINE_STRING);
    SmartDashboard::PutData("Autonomous Modes", &m_AutonomousChooser);
    
    RobotUtils::DisplayFormattedMessage("The drive forward axis is: %d\n", Yta::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.RIGHT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive reverse axis is: %d\n", Yta::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive left/right axis is: %d\n", Yta::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_X_AXIS);

    ConfigureMotorControllers();

    CANdleConfiguration candleConfig;
    candleConfig.stripType = LEDStripType::RGB;
    m_pCandle->ConfigAllSettings(candleConfig);
    m_pCandle->Animate(m_RainbowAnimation);

    // Spawn the vision thread
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);
    m_CameraThread.detach();
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

    // Stop/clear any timers, just in case
    // @todo: Make this a dedicated function.
    m_pMatchModeTimer->Stop();
    m_pMatchModeTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Just in case constructor was called before these were set (likely the case)
    m_AllianceColor = DriverStation::GetAlliance();

    // Disable the rainbow animation
    m_pCandle->ClearAnimation(0);

    // Set the LEDs to the alliance color
    SetLedsToAllianceColor();

    // Indicate the camera thread can continue
    RobotCamera::ReleaseThread();

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
    CommandScheduler::GetInstance().CancelAll();
    InitialStateSetup();

    // Tele-op won't do detailed processing of the images unless instructed to
    RobotCamera::SetFullProcessing(false);
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);

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
        DriveControlSequence();
    }

    //PneumaticSequence();
    
    //CameraSequence();

    //LedSequence();
    //BlinkMorseCodePattern();
    //MusicSequence();

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
    // Nothing to send yet
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
        DRIFT_DISABLED,
        NUM_DRIFT_STATES
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
            SetLedsToAllianceColor();
        }
        bLastDriftValue = bDrifting;
    }

    // B: {132, 132, 255}
    // Y: {255, 240, 0}
    // P: {240, 73, 241}
    const LedColors MARIO_KART_LED_COLORS[NUM_DRIFT_STATES] =
    {
        {   0,   0,   0,   0},
        { 132, 132, 255,   0},
        { 255, 240,   0,   0},
        { 240,  73, 241,   0},
        {   0,   0,   0,   0}
    };

    // Light up the LEDs based on state
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
                    m_pCandle->SetLEDs(MARIO_KART_LED_COLORS[DRIFT_BLUE].m_Red,
                                       MARIO_KART_LED_COLORS[DRIFT_BLUE].m_Green,
                                       MARIO_KART_LED_COLORS[DRIFT_BLUE].m_Blue,
                                       MARIO_KART_LED_COLORS[DRIFT_BLUE].m_White,
                                       0, NUMBER_OF_LEDS);
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
                    m_pCandle->SetLEDs(MARIO_KART_LED_COLORS[DRIFT_YELLOW].m_Red,
                                       MARIO_KART_LED_COLORS[DRIFT_YELLOW].m_Green,
                                       MARIO_KART_LED_COLORS[DRIFT_YELLOW].m_Blue,
                                       MARIO_KART_LED_COLORS[DRIFT_YELLOW].m_White,
                                       0, NUMBER_OF_LEDS);
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
                    m_pCandle->SetLEDs(MARIO_KART_LED_COLORS[DRIFT_PURPLE].m_Red,
                                       MARIO_KART_LED_COLORS[DRIFT_PURPLE].m_Green,
                                       MARIO_KART_LED_COLORS[DRIFT_PURPLE].m_Blue,
                                       MARIO_KART_LED_COLORS[DRIFT_PURPLE].m_White,
                                       0, NUMBER_OF_LEDS);
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
/// @method YtaRobot::BlinkMorseCodePattern
///
/// This method contains the main workflow for blinking a Morse
/// code pattern through the robot LEDs.
///
////////////////////////////////////////////////////////////////
void YtaRobot::BlinkMorseCodePattern()
{
    enum MorseCodeSignal
    {
        END_MARKER,
        DOT,
        DASH,
        EMPTY,
        INVALID
    };

    // Characters include the terminating break.
    // The word break only contains four empty signals because
    // the characters always end with the first three empty signals.
    constexpr const MorseCodeSignal MORSE_A[] = {DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_B[] = {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_C[] = {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_D[] = {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_E[] = {DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_F[] = {DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_G[] = {DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_H[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_I[] = {DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_J[] = {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_K[] = {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_L[] = {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_M[] = {DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_N[] = {DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_O[] = {DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_P[] = {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_Q[] = {DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_R[] = {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_S[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_T[] = {DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_U[] = {DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_V[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_W[] = {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_X[] = {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_Y[] = {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_Z[] = {DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_0[] = {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_1[] = {DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_2[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_3[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_4[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_5[] = {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_6[] = {DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_7[] = {DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_8[] = {DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_9[] = {DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_PERIOD[] =            {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_COMMA[] =             {DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_QUESTION_MARK[] =     {DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_SINGLE_QUOTE[] =      {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_FORWARD_SLASH[] =     {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_OPEN_PARENTHESIS[] =  {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_CLOSE_PARENTHESIS[] = {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_COLON[] =             {DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_EQUAL[] =             {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_PLUS[] =              {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_HYPHEN[] =            {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_DOUBLE_QUOTE[] =      {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_AT[] =                {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_EXCLAMATION[] =       {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_AMPERSAND[] =         {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_SEMICOLON[] =         {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_UNDERSCORE[] =        {DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_DOLLAR_SIGN[] =       {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_WORD_BREAK[] =        {EMPTY, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_MESSAGE_END[] =       {END_MARKER};
    constexpr const MorseCodeSignal MORSE_INVALID[] =           {INVALID};

    // This table is kept in the same order as the ASCII table to facilitate easy conversion/indexing
    const MorseCodeSignal * MORSE_SIGNALS[] = {
                                                // 0 - 31
                                                MORSE_MESSAGE_END, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID,
                                                MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID,
                                                MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID,
                                                MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID,
                                                // 32 - 63
                                                MORSE_WORD_BREAK, MORSE_EXCLAMATION, MORSE_DOUBLE_QUOTE, MORSE_INVALID, MORSE_DOLLAR_SIGN, MORSE_INVALID, MORSE_AMPERSAND, MORSE_SINGLE_QUOTE,
                                                MORSE_OPEN_PARENTHESIS, MORSE_CLOSE_PARENTHESIS, MORSE_INVALID, MORSE_PLUS, MORSE_COMMA, MORSE_HYPHEN, MORSE_PERIOD, MORSE_FORWARD_SLASH,
                                                MORSE_0, MORSE_1, MORSE_2, MORSE_3, MORSE_4, MORSE_5, MORSE_6, MORSE_7,
                                                MORSE_8, MORSE_9, MORSE_COLON, MORSE_SEMICOLON, MORSE_INVALID, MORSE_EQUAL, MORSE_INVALID, MORSE_QUESTION_MARK,
                                                // 64 - 95
                                                MORSE_AT, MORSE_A, MORSE_B, MORSE_C, MORSE_D, MORSE_E, MORSE_F, MORSE_G,
                                                MORSE_H, MORSE_I, MORSE_J, MORSE_K, MORSE_L, MORSE_M, MORSE_N, MORSE_O,
                                                MORSE_P, MORSE_Q, MORSE_R, MORSE_S, MORSE_T, MORSE_U, MORSE_V, MORSE_W,
                                                MORSE_X, MORSE_Y, MORSE_Z, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_UNDERSCORE,
                                                // 96-127
                                                MORSE_INVALID, MORSE_A, MORSE_B, MORSE_C, MORSE_D, MORSE_E, MORSE_F, MORSE_G,
                                                MORSE_H, MORSE_I, MORSE_J, MORSE_K, MORSE_L, MORSE_M, MORSE_N, MORSE_O,
                                                MORSE_P, MORSE_Q, MORSE_R, MORSE_S, MORSE_T, MORSE_U, MORSE_V, MORSE_W,
                                                MORSE_X, MORSE_Y, MORSE_Z, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID
                                              };

    // Old approach commented out (uses a constant variable length array of pointers instead of fixed length array/string conversion).
    //static const MorseCodeSignal * const MORSE_MESSAGE[] = {MORSE_S, MORSE_O, MORSE_S, MORSE_WORD_BREAK, MORSE_MESSAGE_END};
    static const size_t MORSE_MSG_MAX_LENGTH = 128U;
    static const MorseCodeSignal * MORSE_MESSAGE[MORSE_MSG_MAX_LENGTH] = {};
    static const char MORSE_STRING[] = "S.O.S.";
    static const size_t MORSE_STRING_SIZE = (sizeof(MORSE_STRING) / sizeof(MORSE_STRING[0]));
    static_assert((MORSE_STRING_SIZE <= MORSE_MSG_MAX_LENGTH), "Morse message is too long!");

    static Timer * pMorseTimer = new Timer();
    static bool bInit = false;

    // Perform one time initialization logic
    if (!bInit)
    {
        // Build the morse message by converting it from the human readable string
        size_t messageOutputPosition = 0U;
        for (size_t i = 0U; i < MORSE_STRING_SIZE; i++)
        {
            // Convert the character to its integer representation
            uint8_t charVal = static_cast<uint8_t>(MORSE_STRING[i]);
            const uint8_t NUM_ASCII_TABLE_ENTRIES = 128U;

            // Make sure the index is in range
            if (charVal >= NUM_ASCII_TABLE_ENTRIES)
            {
                // A not basic ASCII value was found at the current character position, move on
                continue;
            }

            // Get the character's Morse code signal
            const MorseCodeSignal * pThisCharacterMorseSignal = MORSE_SIGNALS[charVal];

            // If the Morse character is valid, add it to the message
            if (pThisCharacterMorseSignal[0] != INVALID)
            {
                // The only character with end marker first is the end of message marker.
                // End of message also needs an end of word inserted, so we have to manually
                // handle that here.
                if (pThisCharacterMorseSignal[0] == END_MARKER)
                {
                    MORSE_MESSAGE[messageOutputPosition++] = MORSE_WORD_BREAK;
                }
                MORSE_MESSAGE[messageOutputPosition++] = pThisCharacterMorseSignal;
            }
        }

        // Start with the LEDs off
        m_pCandle->SetLEDs(0, 0, 0, 0, 0, NUMBER_OF_LEDS);

        // Start the timer
        pMorseTimer->Reset();
        pMorseTimer->Start();

        bInit = true;
    }

    // Deliberately start this index as all Fs so the initial state change rolls over.
    static uint32_t currentCharacterSignalIndex = 0xFFFFFFFFU;
    static uint32_t currentCharacterIndex = 0U;
    static const MorseCodeSignal * pCurrentMorseCharacter = MORSE_MESSAGE[0];

    // Signal display and time variables
    constexpr const units::time::second_t SIGNAL_DISPLAY_TIME_UNIT_S = 0.25_s;
    constexpr const units::time::second_t SIGNAL_DISPLAY_TIME_DOT = SIGNAL_DISPLAY_TIME_UNIT_S;
    constexpr const units::time::second_t SIGNAL_DISPLAY_TIME_DASH = SIGNAL_DISPLAY_TIME_UNIT_S * 3.0;
    static units::time::second_t currentSignalDisplayLengthSeconds = 1.0_s;

    // Check if a signal change is required
    if (pMorseTimer->Get() > currentSignalDisplayLengthSeconds)
    {
        // Move on to the next signal
        currentCharacterSignalIndex++;
        pMorseTimer->Reset();
    }
    else
    {
        // The timer has not reached a point for a change, do nothing different
        return;
    }

    // If we make it here, a state change is needed

    // Examine the next signal
    bool bLedsOn = false;
    switch (pCurrentMorseCharacter[currentCharacterSignalIndex])
    {
        case DOT:
        {
            currentSignalDisplayLengthSeconds = SIGNAL_DISPLAY_TIME_DOT;
            bLedsOn = true;
            break;
        }
        case DASH:
        {
            currentSignalDisplayLengthSeconds = SIGNAL_DISPLAY_TIME_DASH;
            bLedsOn = true;
            break;
        }
        case EMPTY:
        {
            currentSignalDisplayLengthSeconds = SIGNAL_DISPLAY_TIME_UNIT_S;
            bLedsOn = false;
            break;
        }
        // At the end of the current character signals
        case END_MARKER:
        default:
        {
            currentSignalDisplayLengthSeconds = 0.0_s;
            bLedsOn = false;

            // Move to the next character
            pCurrentMorseCharacter = MORSE_MESSAGE[++currentCharacterIndex];
            currentCharacterSignalIndex = 0xFFFFFFFFU;

            // Check if the next character is actually the end of message marker
            if (pCurrentMorseCharacter[0] == END_MARKER)
            {
                // Back to the start of the message
                currentCharacterIndex = 0U;
                pCurrentMorseCharacter = MORSE_MESSAGE[0];
            }

            break;
        }
    }

    // Update the state of the LEDs
    if (bLedsOn)
    {
        if (m_AllianceColor.value() == DriverStation::Alliance::kRed)
        {
            m_pCandle->SetLEDs(255, 0, 0, 0, 0, NUMBER_OF_LEDS);
        }
        else
        {
            m_pCandle->SetLEDs(0, 0, 255, 0, 0, NUMBER_OF_LEDS);
        }
    }
    else
    {
        m_pCandle->SetLEDs(0, 0, 0, 0, 0, NUMBER_OF_LEDS);
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
    // Note: The control mode for the motors can only be one
    //       thing at a time.  Using a motor for acutal motion
    //       will not work at the same time as playing tones.
    static bool bMusicPlaying = false;
    if (m_pDriveController->GetButtonState(PLAY_MUSIC_BUTTON))
    {
        bMusicPlaying = true;
    }

    // Not playing any music, just return
    if (!bMusicPlaying)
    {
        return;
    }

    // Add more of these as needed
    static const MusicTone noNote(units::frequency::hertz_t(0));
    static const MusicTone cNote(units::frequency::hertz_t(262));
    static const MusicTone dNote(units::frequency::hertz_t(294));
    static const MusicTone eNote(units::frequency::hertz_t(330));
    static const MusicTone fNote(units::frequency::hertz_t(349));
    static const MusicTone gNote(units::frequency::hertz_t(392));
    static const MusicTone aNote(units::frequency::hertz_t(440));
    static const MusicTone bNote(units::frequency::hertz_t(494));
    static const MusicTone CNote(units::frequency::hertz_t(523));

    static Timer * pMusicTimer = new Timer();
    static units::time::second_t lastNotePlayTime = 0.0_s;
    static bool bInit = false;
    static uint32_t noteIndex = 0U;

    if (!bInit)
    {
        pMusicTimer->Start();
        bInit = true;
    }

    struct NoteControl
    {
        MusicTone m_Tone;
        units::time::second_t m_LengthSeconds;
    };

    // Change the notes and the lengths in here to make a song.
    // You can add or remove or change as many you want.
    // If you need more tones (frequencies in hertz), add them up above.
    // If you want a pause, use noNote and a length.
    static NoteControl scaleNotes[] =
    {
        {noNote, 100_ms},
        {cNote, 100_ms},
        {dNote, 100_ms},
        {eNote, 100_ms},
        {fNote, 100_ms},
        {gNote, 100_ms},
        {aNote, 100_ms},
        {bNote, 100_ms},
        {CNote, 100_ms},
        {noNote, 100_ms},
    };
    static const size_t MAX_NOTE_INDEX = sizeof(scaleNotes) / sizeof (NoteControl);

    if ((pMusicTimer->Get() - lastNotePlayTime) > scaleNotes[noteIndex].m_LengthSeconds)
    {
        noteIndex++;
        if (noteIndex >= MAX_NOTE_INDEX)
        {
            noteIndex = 0U;
            bMusicPlaying = false;
        }
        // Pick a motor to play a sound
        //m_pMusicMotor->SetControl(scaleNotes[noteIndex].m_Tone);
        lastNotePlayTime = pMusicTimer->Get();
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
/// RobotCamera class for full details.
///
////////////////////////////////////////////////////////////////
void YtaRobot::CameraSequence()
{
    if (m_pDriveController->GetButtonState(DRIVE_ALIGN_WITH_CAMERA_BUTTON))
    {
        m_bCameraAlignInProgress = true;
        RobotCamera::SetLimelightPipeline(1);
        RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::VISION_PROCESSOR);
        RobotCamera::AutonomousCamera::AlignToTargetSwerve();
    }
    else
    {
        m_bCameraAlignInProgress = false;
        RobotCamera::SetLimelightPipeline(0);
        RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    }

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
        m_pSwerveDrive->HomeModules();
    }

    if (m_pDriveController->DetectButtonChange(LOCK_SWERVE_WHEELS_BUTTON))
    {
        m_pSwerveDrive->LockWheels();
    }

    // The GetDriveX() and GetDriveYInput() functions refer to ***controller joystick***
    // x and y axes.  Multiply by -1.0 here to keep the joystick input retrieval code common.
    double translationAxis = RobotUtils::Trim(m_pDriveController->GetDriveYInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    double strafeAxis = RobotUtils::Trim(m_pDriveController->GetDriveXInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    double rotationAxis = RobotUtils::Trim(m_pDriveController->GetDriveRotateInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);

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
        case DRIVE_CONTROLS_SWERVE_ROTATE_CCW_SLOW_POV:
        {
            translationAxis = 0.0;
            strafeAxis = 0.0;
            rotationAxis = SWERVE_ROTATE_SLOW_SPEED;
            break;
        }
        case DRIVE_CONTROLS_SWERVE_ROTATE_CW_SLOW_POV:
        {
            translationAxis = 0.0;
            strafeAxis = 0.0;
            rotationAxis = -SWERVE_ROTATE_SLOW_SPEED;
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
    m_pSwerveDrive->SetModuleStates(translation, rotationAxis, bFieldRelative, true);

    // Pretend to Mario Kart drift
    //MarioKartLights(translationAxis, strafeAxis, rotationAxis);

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
    static Timer * pInchingDriveTimer = new Timer();
    static constexpr units::second_t INCHING_DRIVE_DELAY_S = 0.10_s;
    static constexpr double INCHING_DRIVE_SPEED = 0.25;

    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    if (m_pDriveController->GetPovAsDirection() == DRIVE_CONTROLS_INCH_FORWARD_POV)
    {
        leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_FORWARD_SCALAR;
        rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_FORWARD_SCALAR;
    }
    else if (m_pDriveController->GetPovAsDirection() == DRIVE_CONTROLS_INCH_REVERSE_POV)
    {
        leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_REVERSE_SCALAR;
        rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_REVERSE_SCALAR;
    }
    else if (m_pDriveController->GetPovAsDirection() == DRIVE_CONTROLS_INCH_LEFT_POV)
    {
        leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_REVERSE_SCALAR;
        rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_FORWARD_SCALAR;
    }
    else if (m_pDriveController->GetPovAsDirection() == DRIVE_CONTROLS_INCH_RIGHT_POV)
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
    pInchingDriveTimer->Reset();
    pInchingDriveTimer->Start();
    
    // Motors on
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);
    
    while (pInchingDriveTimer->Get() < INCHING_DRIVE_DELAY_S)
    {
    }
    
    // Motors back off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    // Stop the timer
    pInchingDriveTimer->Stop();
    pInchingDriveTimer->Reset();

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
    static Timer * pDirectionalAlignTimer = new Timer();
    static constexpr units::second_t DIRECTIONAL_ALIGN_MAX_TIME_S = 3.00_s;
    static constexpr double DIRECTIONAL_ALIGN_DRIVE_SPEED = 0.55;

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
                // @todo: Use Pigeon2 to get angle.
                int startingAngle = 0;
                
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
                pDirectionalAlignTimer->Start();

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
            //RobotI2c::ManualTrigger();
            
            // Three conditions for stopping the align:
            // 1. Destination angle is reached
            // 2. Safety timer expires
            // 3. User cancels the operation
            // @todo: Is it a problem that (destinationAngle - 1) can be negative when angle == zero?
            // @todo: Use Pigeon2 to get angle.
            int currentAngle = 0;
            if (((currentAngle >= (destinationAngle - 1)) && (currentAngle <= (destinationAngle + 1))) ||
                (pDirectionalAlignTimer->Get() > DIRECTIONAL_ALIGN_MAX_TIME_S) ||
                (bStateChangeAllowed))
            {
                // Motors off
                m_pLeftDriveMotors->Set(OFF);
                m_pRightDriveMotors->Set(OFF);
                
                // Reset the safety timer
                pDirectionalAlignTimer->Stop();
                pDirectionalAlignTimer->Reset();
                
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

    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);

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
