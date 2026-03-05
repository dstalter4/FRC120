////////////////////////////////////////////////////////////////////////////////
/// @file   RomiRobot.cpp
/// @author David Stalter
///
/// @details
/// Class implementation for a timed Romi robot.
///
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////


// NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the
// hardware "overlay"
// that is specified when launching the wpilib-ws server on the Romi raspberry
// pi. By default, the following are available (listed in order from inside of
// the board to outside):
// - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
// - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
// - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
// - PWM 2 (mapped to Arduino Pin 21)
// - PWM 3 (mapped to Arduino Pin 22)
//
// Your subsystem configuration should take the overlays into account

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/print.h>
#include "RomiRobot.hpp"


////////////////////////////////////////////////////////////////
// @method RomiRobot::RomiRobot
//
// Constructor
////////////////////////////////////////////////////////////////
RomiRobot::RomiRobot() :
    m_Chooser(),
    m_AutoSelected(),
    m_XboxController(0),
    m_Drivetrain(),
    m_OnboardIo(OnBoardIO::ChannelMode::OUTPUT, OnBoardIO::ChannelMode::OUTPUT)
{
    m_Chooser.SetDefaultOption(DEFAULT_AUTO_STRING, DEFAULT_AUTO_STRING);
    m_Chooser.AddOption(CUSTOM_AUTO_STRING, CUSTOM_AUTO_STRING);
    frc::SmartDashboard::PutData("Auto Modes", &m_Chooser);
}


////////////////////////////////////////////////////////////////
// @method RomiRobot::RobotPeriodic
//
// This function is called every 20 ms, no matter the mode. Use
// this for items like diagnostics that you want ran during disabled,
// autonomous, teleoperated and test.
//
// <p> This runs after the mode specific periodic functions, but before
// LiveWindow and SmartDashboard integrated updating.
////////////////////////////////////////////////////////////////
void RomiRobot::RobotPeriodic()
{
}


////////////////////////////////////////////////////////////////
// @method RomiRobot::AutonomousInit
//
// Functionality executed once before teleop.
//
// This autonomous (along with the chooser code above) shows how to select
// between different autonomous modes using the dashboard. The sendable chooser
// code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
// remove all of the chooser code and uncomment the GetString line to get the
// auto name from the text box below the Gyro.
//
// You can add additional auto modes by adding additional comparisons to the
// if-else structure below with additional strings. If using the SendableChooser
// make sure to add them to the chooser code above as well.
////////////////////////////////////////////////////////////////
void RomiRobot::AutonomousInit()
{
    m_AutoSelected = m_Chooser.GetSelected();
    // m_AutoSelected = SmartDashboard::GetString("Auto Selector", DEFAULT_AUTO_STRING);
    wpi::print("Auto selected: {}\n", m_AutoSelected);

    if (m_AutoSelected == CUSTOM_AUTO_STRING)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }
}


////////////////////////////////////////////////////////////////
// @method RomiRobot::AutonomousPeriodic
//
// Functionality to be executed periodically during autonomous.
////////////////////////////////////////////////////////////////
void RomiRobot::AutonomousPeriodic()
{
    if (m_AutoSelected == CUSTOM_AUTO_STRING)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }
}


////////////////////////////////////////////////////////////////
// @method RomiRobot::TeleopInit
//
// Functionality executed once before teleop.
////////////////////////////////////////////////////////////////
void RomiRobot::TeleopInit()
{
}


////////////////////////////////////////////////////////////////
// @method RomiRobot::TeleopPeriodic
//
// Functionality to be executed periodically during teleop.
////////////////////////////////////////////////////////////////
void RomiRobot::TeleopPeriodic()
{
    DriveSequence();
    UpdateOnBoardIoSequence();
}


////////////////////////////////////////////////////////////////
// @method RomiRobot::DriveSequence
//
// Main sequence for driving the Romi robot.  Uses GTA style
// controls by getting input from the left and right triggers
// and combining their values (if both are pressed, no motion
// should occur).  Turning is controlled by the left joystick.
////////////////////////////////////////////////////////////////
void RomiRobot::DriveSequence()
{
    double rightTriggerAxis = m_XboxController.GetRightTriggerAxis();
    double leftTriggerAxis = m_XboxController.GetLeftTriggerAxis();
    double translationValue = rightTriggerAxis - leftTriggerAxis;
    double zAxisRotate = -m_XboxController.GetLeftX();

    m_Drivetrain.ArcadeDrive(translationValue, zAxisRotate);
}


////////////////////////////////////////////////////////////////
// @method RomiRobot::UpdateOnBoardIoSequence
//
// Demonstrates use of the on-board I/O functionality.
////////////////////////////////////////////////////////////////
void RomiRobot::UpdateOnBoardIoSequence()
{
    // Romi on-board I/O:
    //   DIO 0 - Button A (input only)
    //   DIO 1 - Button B (input) or Green LED (output)
    //   DIO 2 - Button C (input) or Red LED (output)
    //   DIO 3 - Yellow LED (output only)
    //
    // When the constructor runs, it configures DIO 1/2 for
    // input or output (can't be both).

    static bool bInitialized = false;
    static unsigned timerCounter = 0U;
    static Timer ledTimer;
    static units::second_t lastTimeStamp = 0.0_s;

    if (!bInitialized)
    {
        ledTimer.Start();
        bInitialized = true;
    }

    units::second_t currentTimeStamp = ledTimer.Get();
    if ((currentTimeStamp - lastTimeStamp) > 1.0_s)
    {
        timerCounter++;
        if (timerCounter == 8U)
        {
            timerCounter = 0U;
        }

        m_OnboardIo.SetRedLed((timerCounter & 0x4) == 0x4);
        m_OnboardIo.SetYellowLed((timerCounter & 0x2) == 0x2);
        m_OnboardIo.SetGreenLed((timerCounter & 0x1) == 0x1);

        lastTimeStamp = currentTimeStamp;
    }

    //SmartDashboard::PutBoolean("Romi Button A:", m_OnboardIo.GetButtonAPressed());
    //SmartDashboard::PutBoolean("Romi Button B:", m_OnboardIo.GetButtonBPressed());
    //SmartDashboard::PutBoolean("Romi Button C:", m_OnboardIo.GetButtonCPressed());
}


void RomiRobot::DisabledInit() {}

void RomiRobot::DisabledPeriodic() {}

void RomiRobot::TestInit() {}

void RomiRobot::TestPeriodic() {}

void RomiRobot::SimulationInit() {}

void RomiRobot::SimulationPeriodic() {}


#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<RomiRobot>();
}
#endif
