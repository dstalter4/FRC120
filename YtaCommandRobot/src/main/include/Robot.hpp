////////////////////////////////////////////////////////////////////////////////
/// @file   Robot.hpp
/// @author David Stalter
///
/// @details
/// Robot class declarations.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#pragma once

// SYSTEM INCLUDES
#include <optional>

// WPILIB INCLUDES
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

// C++ INCLUDES
#include "RobotContainer.hpp"


////////////////////////////////////////////////////////////////
/// @class Robot
///
/// Derived class from TimedRobot.  The object that will
/// control all robot functionality.
///
////////////////////////////////////////////////////////////////
class Robot : public frc::TimedRobot
{
public:
    Robot();
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

private:
    // Have it empty by default so that if testing teleop it
    // doesn't have undefined behavior and potentially crash.
    std::optional<frc2::CommandPtr> m_AutonomousCommand;
    std::optional<frc2::CommandPtr> m_TeleopCommand;

    RobotContainer m_RobotContainer;
};
