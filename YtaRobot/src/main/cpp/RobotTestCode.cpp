////////////////////////////////////////////////////////////////////////////////
/// @file   RobotTestCode.cpp
/// @author David Stalter
///
/// @details
/// Implementation of the YtaRobot test functions.  This keeps official stable
/// robot code isolated.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/BuiltInAccelerometer.h"   // for using the built-in accelerometer
#include "frc/DutyCycleEncoder.h"       // for through bore encoder test
#include "rev/SparkMax.h"               // for interacting with spark max motor controllers

// C++ INCLUDES
#include "RobotUtils.hpp"               // for DisplayMessage(), DisplayFormattedMessage()
#include "YtaRobot.hpp"                 // for robot class declaration


// Helper macro to get the robot object, only for use in test class code
#define YTA_ROBOT_OBJ() YtaRobot::GetRobotInstance()


////////////////////////////////////////////////////////////////
/// @class YtaRobotTest
///
/// A class used to test robot functionality.  The intention of
/// this class is to enable quick tests or rapid prototypes.
/// It leverages the YtaRobot 'Test' mode functions to execute
/// routines.  Since it is separate from the 'product' robot
/// code (in YtaRobot), it cannot directly use the various
/// member objects from that code.  Instead they can be accessed
/// through the YTA_ROBOT_OBJ() macro.
///
/// A second, but currently unused, test approach is also
/// presented.  This approach attempts to mimic direct use of
/// the YtaRobot object members by binding references to them.
///
////////////////////////////////////////////////////////////////
class YtaRobotTest
{
public:
    static void InitializeCommonPointers();
    static void QuickTestCode();

    static void CtreSpeedControllerTest();
    static void RevSpeedControllerTest();
    static void TankDrive();
    static void SwerveDriveTest();
    static void SuperstructureTest();
    static void PneumaticsTest();

    static void TimeTest();
    static void ButtonChangeTest();
    static void SensorTest();
    static void AccelerometerTest();
    static void CandleLedsTest();
    static void RelayLedsTest();

private:
    // Objects for use in test routines
    static Joystick * m_pJoystick;

    // Alternate test approach (not currently used):
    // Singleton test object with members bound by reference to YtaRobot member objects.
    /*
    YtaRobotTest() :
        m_rpDebugOutput(YtaRobot::GetRobotInstance()->m_pDebugOutput)
    {
    }
    static YtaRobotTest * GetInstance() { return m_pRobotTestObj; }
    static void CreateInstance()
    {
        m_pRobotTestObj = new YtaRobotTest();
    }

    static YtaRobotTest * m_pRobotTestObj;
    DigitalOutput *& m_rpDebugOutput;
    */
};

// STATIC MEMBER DATA
Joystick * YtaRobotTest::m_pJoystick = nullptr;



////////////////////////////////////////////////////////////////
/// @method YtaRobot::TestInit
///
/// The test init method.  This method is called once each time
/// the robot enters test mode.
///
////////////////////////////////////////////////////////////////
void YtaRobot::TestInit()
{
    RobotUtils::DisplayMessage("TestInit called.");

    YtaRobotTest::InitializeCommonPointers();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::TestPeriodic
///
/// The test control method.  This method is called
/// periodically while the robot is in test mode.
///
////////////////////////////////////////////////////////////////
void YtaRobot::TestPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_TEST);

    // Enable or disable routines for testing
    YtaRobotTest::QuickTestCode();
    //YtaRobotTest::CtreSpeedControllerTest();
    //YtaRobotTest::RevSpeedControllerTest();
    //YtaRobotTest::TankDrive();
    //YtaRobotTest::SwerveDriveTest();
    //YtaRobotTest::PneumaticsTest();
    //YtaRobotTest::SuperstructureTest();
    //YtaRobotTest::TimeTest();
    //YtaRobotTest::ButtonChangeTest();
    //YtaRobotTest::SensorTest();
    //YtaRobotTest::AccelerometerTest();
    //YtaRobotTest::CandleLedsTest();
    //YtaRobotTest::RelayLedsTest();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::InitializeCommonPointers
///
/// Initializes any common test pointers by creating objects
/// for them to use.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::InitializeCommonPointers()
{
    static bool bPointersInitialized = false;
    if (!bPointersInitialized)
    {
        // Only support one joystick in test code
        m_pJoystick = new Joystick(0);
        bPointersInitialized = true;
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::QuickTestCode
///
/// Test code to try out for rapid prototyping.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::QuickTestCode()
{
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::SuperstructureTest
///
/// Test code to try out functionality on the superstructure.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::SuperstructureTest()
{
    static TalonFX * pTalonFx9 = new TalonFX(9);
    static TalonFX * pTalonFx10 = new TalonFX(10);
    static TalonFX * pTalonFx11 = new TalonFX(11);
    static TalonFX * pTalonFx12 = new TalonFX(12);
    static DutyCycleOut dutyCycleOut9(0.0);
    static DutyCycleOut dutyCycleOut10(0.0);
    static DutyCycleOut dutyCycleOut11(0.0);
    static DutyCycleOut dutyCycleOut12(0.0);

    dutyCycleOut9.Output = 0.0;
    dutyCycleOut10.Output = 0.0;
    dutyCycleOut11.Output = 0.0;
    dutyCycleOut12.Output = 0.0;

    if (m_pJoystick->GetRawButton(1))
    {
        dutyCycleOut9.Output = 0.1;
    }
    if (m_pJoystick->GetRawButton(2))
    {
        dutyCycleOut10.Output = 0.1;
    }
    if (m_pJoystick->GetRawButton(3))
    {
        dutyCycleOut11.Output = 0.1;
    }
    if (m_pJoystick->GetRawButton(4))
    {
        dutyCycleOut12.Output = 0.1;
    }
    pTalonFx9->SetControl(dutyCycleOut9);
    pTalonFx10->SetControl(dutyCycleOut10);
    pTalonFx11->SetControl(dutyCycleOut11);
    pTalonFx12->SetControl(dutyCycleOut12);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::CtreSpeedControllerTest
///
/// Test code for CTRE speed controllers.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::CtreSpeedControllerTest()
{
    TalonFX * pLeft1 = nullptr;
    TalonFX * pLeft2 = nullptr;
    TalonFX * pRight1 = nullptr;
    TalonFX * pRight2 = nullptr;
    // Alternative control objects for testing with Phoenix 6
    //static DutyCycleOut dutyCycleOut(0.0);
    //static PositionVoltage positionVoltage(0.0_tr);
    static bool bCreatedObjs = false;

    // This approach is used instead of static objects in case calling
    // constructors at program startup create the TalonFX objects instead
    // of first function invocation.  The TalonFX IDs may be invalid in
    // some scenarios.
    if (!bCreatedObjs)
    {
        pLeft1 = new TalonFX(YtaRobot::LEFT_DRIVE_MOTORS_CAN_START_ID);
        pLeft2 = new TalonFX(YtaRobot::LEFT_DRIVE_MOTORS_CAN_START_ID + 1);
        pRight1 = new TalonFX(YtaRobot::RIGHT_DRIVE_MOTORS_CAN_START_ID);
        pRight2 = new TalonFX(YtaRobot::RIGHT_DRIVE_MOTORS_CAN_START_ID + 1);
        bCreatedObjs = true;
    }
 
    // Starting in 2024, wpilib seemed to update how DriverStation::RefreshData() is
    // called (see IterativeRobotBase::LoopFunc()).  In order to use loops below
    // (instead of conditional statements), manual calls to RefreshData() are required.
    // Otherwise the loop never relinquishes thread control and the code never makes it
    // back to LoopFunc().  This admittedly is bad practice, but it's test code.

    while (m_pJoystick->GetRawButton(1))
    {
        pLeft1->Set(1.0);
        pLeft2->Set(1.0);
        DriverStation::RefreshData();
    }
    while (m_pJoystick->GetRawButton(2))
    {
        pLeft1->Set(-1.0);
        pLeft2->Set(-1.0);
        DriverStation::RefreshData();
    }
    while (m_pJoystick->GetRawButton(3))
    {
        pRight1->Set(1.0);
        pRight2->Set(1.0);
        DriverStation::RefreshData();
    }
    while (m_pJoystick->GetRawButton(4))
    {
        pRight1->Set(-1.0);
        pRight2->Set(-1.0);
        DriverStation::RefreshData();
    }
    
    pLeft1->Set(0.0);
    pLeft2->Set(0.0);
    pRight1->Set(0.0);
    pRight2->Set(0.0);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::RevSpeedControllerTest
///
/// Test code for REV speed controllers.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::RevSpeedControllerTest()
{
    static rev::spark::SparkMax * pLeftNeo = new rev::spark::SparkMax(1, rev::spark::SparkMax::MotorType::kBrushless);
    static rev::spark::SparkMax * pRightNeo = new rev::spark::SparkMax(2, rev::spark::SparkMax::MotorType::kBrushless);

    while (m_pJoystick->GetRawButton(1))
    {
        pLeftNeo->Set(1.0);
    }
    while (m_pJoystick->GetRawButton(2))
    {
        pLeftNeo->Set(-1.0);
    }
    while (m_pJoystick->GetRawButton(3))
    {
        pRightNeo->Set(1.0);
    }
    while (m_pJoystick->GetRawButton(4))
    {
        pRightNeo->Set(-1.0);
    }

    pLeftNeo->Set(0.0);
    pRightNeo->Set(0.0);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::TankDrive
///
/// Test code for tank drive of the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::TankDrive()
{
    YTA_ROBOT_OBJ()->m_pLeftDriveMotors->Set(YTA_ROBOT_OBJ()->m_pDriveController->GetAxisValue(1) * -1.0);
    YTA_ROBOT_OBJ()->m_pRightDriveMotors->Set(YTA_ROBOT_OBJ()->m_pDriveController->GetAxisValue(5) * -1.0);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::SwerveDriveTest
///
/// Test code for swerve drive of the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::SwerveDriveTest()
{
    static SwerveDrive * pSwerveDrive = YTA_ROBOT_OBJ()->m_pSwerveDrive;

    // Tests returning modules to absolute reference angles
    if (YTA_ROBOT_OBJ()->m_pDriveController->DetectButtonChange(4))
    {
        // Not available yet
        //YTA_ROBOT_OBJ()->m_pSwerveDrive->HomeModules();
    }

    // Dynamically switch between field relative and robot centric
    static bool bFieldRelative = true;
    if (YTA_ROBOT_OBJ()->m_pDriveController->DetectButtonChange(5))
    {
        bFieldRelative = !bFieldRelative;
    }

    // Zero the gryo
    if (YTA_ROBOT_OBJ()->m_pDriveController->DetectButtonChange(6))
    {
        pSwerveDrive->ZeroGyroYaw();
    }

    // Dynamically switch between arcade and GTA drive controls
    static bool bGtaControls = false;
    if (YTA_ROBOT_OBJ()->m_pDriveController->DetectButtonChange(10))
    {
        bGtaControls = !bGtaControls;
    }

    // Get joystick inputs (x = strafe, y = translation)
    // logitech and xbox controller: strafe = kLeftX (0), translation = kLeftY(1) or triggers (2/3), rotation = kRightX (4)
    double translationAxis = 0.0;
    if (bGtaControls)
    {
        double lAxis = YTA_ROBOT_OBJ()->m_pDriveController->GetAxisValue(2) * -1.0;
        double rAxis = YTA_ROBOT_OBJ()->m_pDriveController->GetAxisValue(3);
        translationAxis = lAxis + rAxis;
    }
    else
    {
        translationAxis = YTA_ROBOT_OBJ()->m_pDriveController->GetAxisValue(1) * -1.0;
    }
    double strafeAxis = YTA_ROBOT_OBJ()->m_pDriveController->GetAxisValue(0) * -1.0;
    double rotationAxis = YTA_ROBOT_OBJ()->m_pDriveController->GetAxisValue(4) * -1.0;

    strafeAxis = RobotUtils::Trim(strafeAxis, 0.10, -0.10);
    translationAxis = RobotUtils::Trim(translationAxis, 0.10, -0.10);
    rotationAxis = RobotUtils::Trim(rotationAxis, 0.10, -0.10);

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
    // Translation2d, double rotation, field relative, open loop
    pSwerveDrive->SetModuleStates(translation, rotationAxis, bFieldRelative, true);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::PneumaticsTest
///
/// Test code for validating pneumatics.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::PneumaticsTest()
{
    // The pneumatics library checks if channels are already in use
    // when creating the object.  The test code either has to pick
    // channels not in use (likely 6/7) or grab a reference to some
    // solenoid object from the actual robot code.
    //static DoubleSolenoid *& rpSolenoid = YTA_ROBOT_OBJ()->m_pTalonCoolingSolenoid;
    static DoubleSolenoid * pSolenoid = new DoubleSolenoid(PneumaticsModuleType::CTREPCM, 6, 7);
    
    if (m_pJoystick->GetRawButton(1))
    {
        pSolenoid->Set(DoubleSolenoid::kForward);
    }
    else if (m_pJoystick->GetRawButton(2))
    {
        pSolenoid->Set(DoubleSolenoid::kReverse);
    }
    else if (m_pJoystick->GetRawButton(3))
    {
        pSolenoid->Set(DoubleSolenoid::kOff);
    }
    else
    {
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::TimeTest
///
/// Test code for manually managing timing (including threads).
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::TimeTest()
{
    // Example code using standard library delays and time tracking
    static std::chrono::time_point<std::chrono::high_resolution_clock> currentTime;
    static std::chrono::time_point<std::chrono::high_resolution_clock> oldTime;
    
    currentTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = currentTime - oldTime;
    
    // Run for 100ms, sleep for 100ms
    const unsigned RUN_SLEEP_INTERVAL_MS = 100U;
    if (elapsed.count() > RUN_SLEEP_INTERVAL_MS)
    {
        auto start = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(RUN_SLEEP_INTERVAL_MS));
        auto end = std::chrono::high_resolution_clock::now();
        
        std::chrono::duration<double, std::milli> elapsed = end - start;
        std::cout << "Slept for " << elapsed.count() << " ms." << std::endl;
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::ButtonChangeTest
///
/// Test code to verify button state change detection works.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::ButtonChangeTest()
{
    // Sample code for testing the detect trigger change code
    if (YTA_ROBOT_OBJ()->m_pDriveController->DetectButtonChange(1, Yta::Controller::ButtonStateChanges::BUTTON_RELEASED))
    {
        RobotUtils::DisplayMessage("Trigger change detected!");
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::SensorTest
///
/// Test code to verify a sensor behavior.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::SensorTest()
{
    static DutyCycleEncoder * pRevThroughBoreEncoder = new DutyCycleEncoder(YtaRobot::SENSOR_TEST_CODE_DIO_CHANNEL);
    (void)pRevThroughBoreEncoder->Get();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::AccelerometerTest
///
/// Test code to verify the built in accelerometer.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::AccelerometerTest()
{
    // Test code for reading the built in accelerometer
    static BuiltInAccelerometer * pAccelerometer = new BuiltInAccelerometer();
    double x = pAccelerometer->GetX();
    double y = pAccelerometer->GetY();
    double z = pAccelerometer->GetZ();
    RobotUtils::DisplayFormattedMessage("x: %f, y: %f, z: %f\n", x, y, z);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::CandleLedsTest
///
/// Test code to verify functionality of CANdle controlled RGB
// LED strips.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::CandleLedsTest()
{
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::RelayLedsTest
///
/// Test code to verify functionality of relay controlled RGB
/// LED strips.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::RelayLedsTest()
{
    enum LedDisplayState
    {
        NONE,
        RED_ONLY,
        GREEN_ONLY,
        BLUE_ONLY,
        RED_GREEN,
        RED_BLUE,
        GREEN_BLUE,
        RED_GREEN_BLUE
    };
    static LedDisplayState displayState = NONE;

    // This may seem backward, but the LEDS work by creating
    // a voltage differential.  The LED strip has four lines,
    // 12V, red, green and blue.  The 12V line gets enabled by
    // one relay during initialization.  The RGB LEDs turn on
    // when there is a voltage differential, so 'on' is when
    // there is 0V on a RGB line (kOff) and 'off' is when there
    // is 12V on a RGB line (kForward).
    static const Relay::Value LEDS_ENABLED  = Relay::kForward;
    static const Relay::Value LEDS_DISABLED = Relay::kOff;
    static const Relay::Value LEDS_OFF      = Relay::kForward;
    static const Relay::Value LEDS_ON       = Relay::kOff;

    static Relay * pLedsEnableRelay = new Relay(0);     // Controls whether the LEDs will light up at all
    static Relay * pRedLedRelay     = new Relay(1);     // Controls whether or not the red LEDs are lit up
    static Relay * pGreenLedRelay   = new Relay(2);     // Controls whether or not the green LEDs are lit up
    static Relay * pBlueLedRelay    = new Relay(3);     // Controls whether or not the blue LEDs are lit up
    
    static std::chrono::time_point<std::chrono::high_resolution_clock> currentTime;
    static std::chrono::time_point<std::chrono::high_resolution_clock> oldTime;
    currentTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = currentTime - oldTime;
    if (elapsed.count() > 1000)
    {
        // kForward turns the LEDs off (voltage difference is zero)
        // kOff turns the LEDs on (voltage difference is +12V)
        switch (displayState)
        {
            case NONE:
            {
                pLedsEnableRelay->Set(LEDS_DISABLED);
                pRedLedRelay->Set(LEDS_OFF);
                pGreenLedRelay->Set(LEDS_OFF);
                pBlueLedRelay->Set(LEDS_OFF);
                displayState = RED_ONLY;
                break;
            }
            case RED_ONLY:
            {
                pLedsEnableRelay->Set(LEDS_ENABLED);
                pRedLedRelay->Set(LEDS_ON);
                pGreenLedRelay->Set(LEDS_OFF);
                pBlueLedRelay->Set(LEDS_OFF);
                displayState = GREEN_ONLY;
                break;
            }
            case GREEN_ONLY:
            {
                pRedLedRelay->Set(LEDS_OFF);
                pGreenLedRelay->Set(LEDS_ON);
                pBlueLedRelay->Set(LEDS_OFF);
                displayState = BLUE_ONLY;
                break;
            }
            case BLUE_ONLY:
            {
                pRedLedRelay->Set(LEDS_OFF);
                pGreenLedRelay->Set(LEDS_OFF);
                pBlueLedRelay->Set(LEDS_ON);
                displayState = RED_GREEN;
                break;
            }
            case RED_GREEN:
            {
                pRedLedRelay->Set(LEDS_ON);
                pGreenLedRelay->Set(LEDS_ON);
                pBlueLedRelay->Set(LEDS_OFF);
                displayState = RED_BLUE;
                break;
            }
            case RED_BLUE:
            {
                pRedLedRelay->Set(LEDS_ON);
                pGreenLedRelay->Set(LEDS_OFF);
                pBlueLedRelay->Set(LEDS_ON);
                displayState = GREEN_BLUE;
                break;
            }
            case GREEN_BLUE:
            {
                pRedLedRelay->Set(LEDS_OFF);
                pGreenLedRelay->Set(LEDS_ON);
                pBlueLedRelay->Set(LEDS_ON);
                displayState = RED_GREEN_BLUE;
                break;
            }
            case RED_GREEN_BLUE:
            {
                pRedLedRelay->Set(LEDS_ON);
                pGreenLedRelay->Set(LEDS_ON);
                pBlueLedRelay->Set(LEDS_ON);
                displayState = NONE;
                break;
            }
            default:
            {
                break;
            }
        }

        oldTime = currentTime;
    }
}
