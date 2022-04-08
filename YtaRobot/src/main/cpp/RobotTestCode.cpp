////////////////////////////////////////////////////////////////////////////////
/// @file   RobotTestCode.cpp
/// @author David Stalter
///
/// @details
/// Implementation of the YtaRobot test functions.  This keeps official stable
/// robot code isolated.
///
/// Copyright (c) 2022 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "rev/CANSparkMax.h"    // for interacting with spark max motor controllers

// C++ INCLUDES
#include "RobotUtils.hpp"       // for DisplayMessage(), DisplayFormattedMessage()
#include "YtaRobot.hpp"         // for robot class declaration


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
    static void SuperstructureTest();
    static void PneumaticsTest();

    static void TimeTest();
    static void ButtonChangeTest();
    static void AccelerometerTest();
    static void LedsTest();

private:
    // Objects for use in test routines
    static Joystick * m_pJoystick;

    // Alternate test approach (not currently used):
    // Singleton test object with members bound by reference to YtaRobot member objects.
    /*
    YtaRobotTest() :
        m_pAccelerometer(YtaRobot::GetRobotInstance()->m_pAccelerometer)
    {
    }
    static YtaRobotTest * GetInstance() { return m_pRobotTestObj; }
    static void CreateInstance()
    {
        m_pRobotTestObj = new YtaRobotTest();
    }

    static YtaRobotTest * m_pRobotTestObj;
    BuiltInAccelerometer *& m_pAccelerometer;
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
    //YtaRobotTest::PneumaticsTest();
    //YtaRobotTest::SuperstructureTest();
    //YtaRobotTest::TimeTest();
    //YtaRobotTest::ButtonChangeTest();
    //YtaRobotTest::AccelerometerTest();
    //YtaRobotTest::LedsTest();
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
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::CtreSpeedControllerTest
///
/// Test code for CTRE speed controllers.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::CtreSpeedControllerTest()
{
    static TalonFX * pLeft1 = new TalonFX(YtaRobot::LEFT_MOTORS_CAN_START_ID);
    static TalonFX * pLeft2 = new TalonFX(YtaRobot::LEFT_MOTORS_CAN_START_ID + 1);
    static TalonFX * pRight1 = new TalonFX(YtaRobot::RIGHT_MOTORS_CAN_START_ID);
    static TalonFX * pRight2 = new TalonFX(YtaRobot::RIGHT_MOTORS_CAN_START_ID + 1);
    
    while (m_pJoystick->GetRawButton(1))
    {
        pLeft1->Set(ControlMode::PercentOutput, 1.0);
        pLeft2->Set(ControlMode::PercentOutput, 1.0);
    }
    while (m_pJoystick->GetRawButton(2))
    {
        pLeft1->Set(ControlMode::PercentOutput, -1.0);
        pLeft2->Set(ControlMode::PercentOutput, -1.0);
    }
    while (m_pJoystick->GetRawButton(3))
    {
        pRight1->Set(ControlMode::PercentOutput, 1.0);
        pRight2->Set(ControlMode::PercentOutput, 1.0);
    }
    while (m_pJoystick->GetRawButton(4))
    {
        pRight1->Set(ControlMode::PercentOutput, -1.0);
        pRight2->Set(ControlMode::PercentOutput, -1.0);
    }
    
    pLeft1->Set(ControlMode::PercentOutput, 0.0);
    pLeft2->Set(ControlMode::PercentOutput, 0.0);
    pRight1->Set(ControlMode::PercentOutput, 0.0);
    pRight2->Set(ControlMode::PercentOutput, 0.0);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::RevSpeedControllerTest
///
/// Test code for REV speed controllers.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::RevSpeedControllerTest()
{
    static rev::CANSparkMax * pLeftNeo = new rev::CANSparkMax(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    static rev::CANSparkMax * pRightNeo = new rev::CANSparkMax(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

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
    //static DoubleSolenoid *& rpSolenoid = YTA_ROBOT_OBJ()->m_p<Name>;
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
/// @method YtaRobotTest::AccelerometerTest
///
/// Test code to verify the built in accelerometer.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::AccelerometerTest()
{
    // Test code for reading the built in accelerometer
    double x = YTA_ROBOT_OBJ()->m_pAccelerometer->GetX();
    double y = YTA_ROBOT_OBJ()->m_pAccelerometer->GetY();
    double z = YTA_ROBOT_OBJ()->m_pAccelerometer->GetZ();
    RobotUtils::DisplayFormattedMessage("x: %f, y: %f, z: %f\n", x, y, z);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::LedsTest
///
/// Test code to verify functionality of RGB LED strips.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::LedsTest()
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
                YTA_ROBOT_OBJ()->m_pRedLedRelay->Set(YtaRobot::LEDS_OFF);
                YTA_ROBOT_OBJ()->m_pGreenLedRelay->Set(YtaRobot::LEDS_OFF);
                YTA_ROBOT_OBJ()->m_pBlueLedRelay->Set(YtaRobot::LEDS_OFF);
                displayState = RED_ONLY;
                break;
            }
            case RED_ONLY:
            {
                YTA_ROBOT_OBJ()->m_pRedLedRelay->Set(YtaRobot::LEDS_ON);
                YTA_ROBOT_OBJ()->m_pGreenLedRelay->Set(YtaRobot::LEDS_OFF);
                YTA_ROBOT_OBJ()->m_pBlueLedRelay->Set(YtaRobot::LEDS_OFF);
                displayState = GREEN_ONLY;
                break;
            }
            case GREEN_ONLY:
            {
                YTA_ROBOT_OBJ()->m_pRedLedRelay->Set(YtaRobot::LEDS_OFF);
                YTA_ROBOT_OBJ()->m_pGreenLedRelay->Set(YtaRobot::LEDS_ON);
                YTA_ROBOT_OBJ()->m_pBlueLedRelay->Set(YtaRobot::LEDS_OFF);
                displayState = BLUE_ONLY;
                break;
            }
            case BLUE_ONLY:
            {
                YTA_ROBOT_OBJ()->m_pRedLedRelay->Set(YtaRobot::LEDS_OFF);
                YTA_ROBOT_OBJ()->m_pGreenLedRelay->Set(YtaRobot::LEDS_OFF);
                YTA_ROBOT_OBJ()->m_pBlueLedRelay->Set(YtaRobot::LEDS_ON);
                displayState = RED_GREEN;
                break;
            }
            case RED_GREEN:
            {
                YTA_ROBOT_OBJ()->m_pRedLedRelay->Set(YtaRobot::LEDS_ON);
                YTA_ROBOT_OBJ()->m_pGreenLedRelay->Set(YtaRobot::LEDS_ON);
                YTA_ROBOT_OBJ()->m_pBlueLedRelay->Set(YtaRobot::LEDS_OFF);
                displayState = RED_BLUE;
                break;
            }
            case RED_BLUE:
            {
                YTA_ROBOT_OBJ()->m_pRedLedRelay->Set(YtaRobot::LEDS_ON);
                YTA_ROBOT_OBJ()->m_pGreenLedRelay->Set(YtaRobot::LEDS_OFF);
                YTA_ROBOT_OBJ()->m_pBlueLedRelay->Set(YtaRobot::LEDS_ON);
                displayState = GREEN_BLUE;
                break;
            }
            case GREEN_BLUE:
            {
                YTA_ROBOT_OBJ()->m_pRedLedRelay->Set(YtaRobot::LEDS_OFF);
                YTA_ROBOT_OBJ()->m_pGreenLedRelay->Set(YtaRobot::LEDS_ON);
                YTA_ROBOT_OBJ()->m_pBlueLedRelay->Set(YtaRobot::LEDS_ON);
                displayState = RED_GREEN_BLUE;
                break;
            }
            case RED_GREEN_BLUE:
            {
                YTA_ROBOT_OBJ()->m_pRedLedRelay->Set(YtaRobot::LEDS_ON);
                YTA_ROBOT_OBJ()->m_pGreenLedRelay->Set(YtaRobot::LEDS_ON);
                YTA_ROBOT_OBJ()->m_pBlueLedRelay->Set(YtaRobot::LEDS_ON);
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
