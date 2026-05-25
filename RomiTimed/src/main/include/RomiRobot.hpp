////////////////////////////////////////////////////////////////////////////////
/// @file   RomiRobot.hpp
/// @author David Stalter
///
/// @details
/// Class declaration for a timed Romi robot.
///
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstring>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/romi/OnBoardIO.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Drivetrain.hpp"

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class RomiRobot
///
/// Derived class from TimedRobot.  The object that will
/// control all robot functionality.
///
////////////////////////////////////////////////////////////////
class RomiRobot : public frc::TimedRobot
{
public:
    RomiRobot();
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

private:
    void DriveSequence();
    void UpdateOnBoardIoSequence();

    frc::SendableChooser<std::string> m_Chooser;
    const std::string DEFAULT_AUTO_STRING = "Default";
    const std::string CUSTOM_AUTO_STRING = "My Auto";
    std::string m_AutoSelected;

    XboxController m_XboxController;
    Drivetrain m_Drivetrain;
    OnBoardIO m_OnboardIo;
};
