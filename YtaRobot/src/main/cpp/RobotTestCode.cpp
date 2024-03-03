////////////////////////////////////////////////////////////////////////////////
/// @file   RobotTestCode.cpp
/// @author David Stalter
///
/// @details
/// Implementation of the YtaRobot test functions.  This keeps official stable
/// robot code isolated.
///
/// Copyright (c) 2024 Youth Technology Academy
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
    static void SwerveDriveTest();
    static void SuperstructureTest();
    static void PneumaticsTest();

    static void TimeTest();
    static void ButtonChangeTest();
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
    //YtaRobotTest::SwerveDriveTest();
    //YtaRobotTest::PneumaticsTest();
    //YtaRobotTest::SuperstructureTest();
    //YtaRobotTest::TimeTest();
    //YtaRobotTest::ButtonChangeTest();
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
    static TalonFX * pTalon1 = new TalonFX(2);
    static TalonFX * pTalon2 = new TalonFX(4);
    static TalonFX * pTalon3 = new TalonFX(6);
    static TalonFX * pTalon4 = new TalonFX(8);
    static PositionVoltage * pPositionVoltage1 = new PositionVoltage(0.0_tr);
    static PositionVoltage * pPositionVoltage2 = new PositionVoltage(0.0_tr);
    static PositionVoltage * pPositionVoltage3 = new PositionVoltage(0.0_tr);
    static PositionVoltage * pPositionVoltage4 = new PositionVoltage(0.0_tr);
    static CANcoder * pCanCoder1 = new CANcoder(1, "canivore-120");
    static CANcoder * pCanCoder2 = new CANcoder(2, "canivore-120");
    static CANcoder * pCanCoder3 = new CANcoder(3, "canivore-120");
    static CANcoder * pCanCoder4 = new CANcoder(4, "canivore-120");

    // Bevel R, Bevel L
    // FL: 0.440186, -0.065186
    // FR: 0.092773, -0.396484
    // BL: 0.337646, -0.167725
    // BR: -0.378418, 0.114990
    static const units::angle::turn_t TARGET_CANCODER_POS1 = 0.440186_tr;
    static const units::angle::turn_t TARGET_CANCODER_POS2 = 0.092773_tr;
    static const units::angle::turn_t TARGET_CANCODER_POS3 = 0.337646_tr;
    static const units::angle::turn_t TARGET_CANCODER_POS4 = -0.378418_tr;
    static units::angle::turn_t initialTurn1 = 0.0_tr;
    static units::angle::turn_t initialTurn2 = 0.0_tr;
    static units::angle::turn_t initialTurn3 = 0.0_tr;
    static units::angle::turn_t initialTurn4 = 0.0_tr;
    static units::angle::turn_t deltaTurns1 = 0.0_tr;
    static units::angle::turn_t deltaTurns2 = 0.0_tr;
    static units::angle::turn_t deltaTurns3 = 0.0_tr;
    static units::angle::turn_t deltaTurns4 = 0.0_tr;

    if (m_pJoystick->GetRawButtonPressed(8))
    {
        TalonFXConfiguration angleTalonConfig;
        angleTalonConfig.MotorOutput.Inverted = InvertedValue::Clockwise_Positive;
        angleTalonConfig.MotorOutput.NeutralMode = NeutralModeValue::Coast;
        angleTalonConfig.Feedback.SensorToMechanismRatio = SwerveConfig::ANGLE_GEAR_RATIO;
        angleTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;

        angleTalonConfig.CurrentLimits.SupplyCurrentLimit = 25.0;
        angleTalonConfig.CurrentLimits.SupplyCurrentThreshold = 40.0;
        angleTalonConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
        angleTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        angleTalonConfig.Slot0.kP = 50.0;
        angleTalonConfig.Slot0.kI = 0.0;
        angleTalonConfig.Slot0.kD = 0.5;

        (void)pTalon1->GetConfigurator().Apply(angleTalonConfig);
        (void)pTalon2->GetConfigurator().Apply(angleTalonConfig);
        (void)pTalon3->GetConfigurator().Apply(angleTalonConfig);
        (void)pTalon4->GetConfigurator().Apply(angleTalonConfig);

        CANcoderConfiguration canCoderConfig;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
        (void)pCanCoder1->GetConfigurator().Apply(canCoderConfig);
        (void)pCanCoder2->GetConfigurator().Apply(canCoderConfig);
        (void)pCanCoder3->GetConfigurator().Apply(canCoderConfig);
        (void)pCanCoder4->GetConfigurator().Apply(canCoderConfig);

        initialTurn1 = pCanCoder1->GetAbsolutePosition().GetValue();
        initialTurn2 = pCanCoder2->GetAbsolutePosition().GetValue();
        initialTurn3 = pCanCoder3->GetAbsolutePosition().GetValue();
        initialTurn4 = pCanCoder4->GetAbsolutePosition().GetValue();
        deltaTurns1 = initialTurn1 - TARGET_CANCODER_POS1;
        deltaTurns2 = initialTurn2 - TARGET_CANCODER_POS2;
        deltaTurns3 = initialTurn3 - TARGET_CANCODER_POS3;
        deltaTurns4 = initialTurn4 - TARGET_CANCODER_POS4;
        pTalon1->SetPosition(-deltaTurns1);
        pTalon2->SetPosition(-deltaTurns2);
        pTalon3->SetPosition(-deltaTurns3);
        pTalon4->SetPosition(-deltaTurns4);

        SmartDashboard::PutNumber("Debug A", pCanCoder1->GetAbsolutePosition().GetValueAsDouble());
        SmartDashboard::PutNumber("Debug B", pCanCoder2->GetAbsolutePosition().GetValueAsDouble());
        SmartDashboard::PutNumber("Debug C", pCanCoder3->GetAbsolutePosition().GetValueAsDouble());
        SmartDashboard::PutNumber("Debug D", pCanCoder4->GetAbsolutePosition().GetValueAsDouble());
        SmartDashboard::PutNumber("Debug E", -deltaTurns1.value());
        SmartDashboard::PutNumber("Debug F", -deltaTurns2.value());
        SmartDashboard::PutNumber("Debug G", -deltaTurns3.value());
        SmartDashboard::PutNumber("Debug H", -deltaTurns4.value());
    }

    units::angle::turn_t turns = pTalon1->GetPosition().GetValue();
    SmartDashboard::PutNumber("Debug I", turns.value());
    SmartDashboard::PutNumber("Debug J", pCanCoder1->GetAbsolutePosition().GetValueAsDouble());

    if (m_pJoystick->GetRawButtonPressed(5))
    {
        (void)pTalon1->SetControl(pPositionVoltage1->WithPosition(turns - 0.25_tr));
        //(void)pTalon1->SetControl(pPositionVoltage1->WithPosition(0_tr));
        (void)pTalon2->SetControl(pPositionVoltage2->WithPosition(0_tr));
        (void)pTalon3->SetControl(pPositionVoltage3->WithPosition(0_tr));
        (void)pTalon4->SetControl(pPositionVoltage4->WithPosition(0_tr));
    }

    if (m_pJoystick->GetRawButtonPressed(6))
    {
        (void)pTalon1->SetControl(pPositionVoltage1->WithPosition(turns + 0.25_tr));
        //(void)pTalon1->SetControl(pPositionVoltage1->WithPosition(0_tr + 0.25_tr));
        (void)pTalon2->SetControl(pPositionVoltage2->WithPosition(0_tr + 0.25_tr));
        (void)pTalon3->SetControl(pPositionVoltage3->WithPosition(0_tr + 0.25_tr));
        (void)pTalon4->SetControl(pPositionVoltage4->WithPosition(0_tr + 0.25_tr));
    }

    if (m_pJoystick->GetRawButtonPressed(1))
    {
        (void)pTalon1->SetControl(pPositionVoltage1->WithPosition(180_deg));
        (void)pTalon2->SetControl(pPositionVoltage2->WithPosition(180_deg));
        (void)pTalon3->SetControl(pPositionVoltage3->WithPosition(180_deg));
        (void)pTalon4->SetControl(pPositionVoltage4->WithPosition(180_deg));
    }
    if (m_pJoystick->GetRawButtonPressed(2))
    {
        (void)pTalon1->SetControl(pPositionVoltage1->WithPosition(90_deg));
        (void)pTalon2->SetControl(pPositionVoltage2->WithPosition(90_deg));
        (void)pTalon3->SetControl(pPositionVoltage3->WithPosition(90_deg));
        (void)pTalon4->SetControl(pPositionVoltage4->WithPosition(90_deg));
    }
    if (m_pJoystick->GetRawButtonPressed(3))
    {
        (void)pTalon1->SetControl(pPositionVoltage1->WithPosition(-90_deg));
        (void)pTalon2->SetControl(pPositionVoltage2->WithPosition(-90_deg));
        (void)pTalon3->SetControl(pPositionVoltage3->WithPosition(-90_deg));
        (void)pTalon4->SetControl(pPositionVoltage4->WithPosition(-90_deg));
    }
    if (m_pJoystick->GetRawButtonPressed(4))
    {
        (void)pTalon1->SetControl(pPositionVoltage1->WithPosition(0_deg));
        (void)pTalon2->SetControl(pPositionVoltage2->WithPosition(0_deg));
        (void)pTalon3->SetControl(pPositionVoltage3->WithPosition(0_deg));
        (void)pTalon4->SetControl(pPositionVoltage4->WithPosition(0_deg));
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::SuperstructureTest
///
/// Test code to try out functionality on the superstructure.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::SuperstructureTest()
{
    static TalonFX * pTalonFx5 = new TalonFX(5);
    static TalonFX * pTalonFx6 = new TalonFX(6);
    static TalonFX * pTalonFx7 = new TalonFX(7);
    static TalonFX * pTalonFx8 = new TalonFX(8);
    static TalonFX * pTalonFx9 = new TalonFX(9);
    static TalonFX * pTalonFx10 = new TalonFX(10);
    
    while (m_pJoystick->GetRawButton(1))
    {
        pTalonFx5->Set(0.3);
        pTalonFx6->Set(0.3);
    }
    pTalonFx5->Set(0.0);
    pTalonFx6->Set(0.0);
    while (m_pJoystick->GetRawButton(2))
    {
        pTalonFx7->Set(0.3);
        pTalonFx8->Set(0.3);
    }
    pTalonFx7->Set(0.0);
    pTalonFx8->Set(0.0);
    while (m_pJoystick->GetRawButton(3))
    {
        pTalonFx9->Set(0.3);
        pTalonFx10->Set(0.3);
    }
    pTalonFx9->Set(0.0);
    pTalonFx10->Set(0.0);
    while (m_pJoystick->GetRawButton(4))
    {
        pTalonFx5->Set(0.3);
        pTalonFx5->Set(0.3);
        pTalonFx7->Set(0.5);
        pTalonFx8->Set(0.5);
        pTalonFx9->Set(1.0);
        pTalonFx10->Set(1.0);
    }
    pTalonFx5->Set(0.0);
    pTalonFx6->Set(0.0);
    pTalonFx7->Set(0.0);
    pTalonFx8->Set(0.0);
    pTalonFx9->Set(0.0);
    pTalonFx10->Set(0.0);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobotTest::CtreSpeedControllerTest
///
/// Test code for CTRE speed controllers.
///
////////////////////////////////////////////////////////////////
void YtaRobotTest::CtreSpeedControllerTest()
{
    static TalonFX * pLeft1 = new TalonFX(YtaRobot::LEFT_DRIVE_MOTORS_CAN_START_ID);
    static TalonFX * pLeft2 = new TalonFX(YtaRobot::LEFT_DRIVE_MOTORS_CAN_START_ID + 1);
    static TalonFX * pRight1 = new TalonFX(YtaRobot::RIGHT_DRIVE_MOTORS_CAN_START_ID);
    static TalonFX * pRight2 = new TalonFX(YtaRobot::RIGHT_DRIVE_MOTORS_CAN_START_ID + 1);
    
    while (m_pJoystick->GetRawButton(1))
    {
        pLeft1->Set(1.0);
        pLeft2->Set(1.0);
    }
    while (m_pJoystick->GetRawButton(2))
    {
        pLeft1->Set(-1.0);
        pLeft2->Set(-1.0);
    }
    while (m_pJoystick->GetRawButton(3))
    {
        pRight1->Set(1.0);
        pRight2->Set(1.0);
    }
    while (m_pJoystick->GetRawButton(4))
    {
        pRight1->Set(-1.0);
        pRight2->Set(-1.0);
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
    static rev::CANSparkMax * pLeftNeo = new rev::CANSparkMax(1, rev::CANSparkLowLevel::MotorType::kBrushless);
    static rev::CANSparkMax * pRightNeo = new rev::CANSparkMax(2, rev::CANSparkLowLevel::MotorType::kBrushless);

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
        //pSwerveDrive->ZeroGyroYaw();
        pSwerveDrive->ZeroHeading();
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
