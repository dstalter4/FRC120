////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous1.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 1 for YtaRobot.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotUtils.hpp"               // for DisplayMessage()
#include "YtaRobot.hpp"                 // for robot class declaration
#include "YtaRobotAutonomous.hpp"       // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousRoutine1
///
/// Autonomous routine 1.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine1()
{
    enum StationState
    {
        ASCENDING_STATION,
        BALANCING_STATION,
        BALANCED
    };
    StationState stationState = ASCENDING_STATION;

    // Enable the intake motor to spit out a cube
    m_pIntakeMotor->Set(ControlMode::PercentOutput, 1.0);
    Wait(2.0_s);
    m_pIntakeMotor->Set(ControlMode::PercentOutput, 0.0);

    AutonomousSwerveDriveSequence(RobotDirection::ROBOT_FORWARD, ROBOT_NO_ROTATE, 0.30, 0.0, 3.5_s, true);
    return;

    uint32_t decreasingRollCount = 0U;
    double lastRoll = 0.0;

    m_pSafetyTimer->Start();
    units::second_t startTime = m_pSafetyTimer->Get();
    while ((stationState != BALANCED) && ((m_pSafetyTimer->Get() - startTime) < 10.0_s))
    //while ((stationState != BALANCED) && DriverStation::IsAutonomousEnabled())
    {
        double currentRoll = m_pPigeon->GetRoll();

        switch (stationState)
        {
            case ASCENDING_STATION:
            {
                m_pSwerveDrive->SetModuleStates({0.20_m, 0.0_m}, 0.0, true, true);

                // If the pitch is > 10 degrees, we can start to try and balance
                if (currentRoll > 10.0)
                {
                    lastRoll = currentRoll;
                    stationState = BALANCING_STATION;

                    // The delay is because the ramp angle bounces as the robot moves it.
                    // Give it some time to climb.
                    AutonomousDelay(0.5_s);
                }

                break;
            }
            case BALANCING_STATION:
            {
                if (currentRoll < lastRoll)
                {
                    decreasingRollCount++;
                }
                else
                {
                    decreasingRollCount = 0U;
                }

                // If we see three consecutive decreasing roll counts, lock wheels and balance
                if (decreasingRollCount == 3U)
                {
                    m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, 1.0, true, true);
                    AutonomousDelay(0.02_s);
                    m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, 0.0, true, true);
                    stationState = BALANCED;
                }

                lastRoll = currentRoll;
                AutonomousDelay(0.04_s);

                break;
            }
            default:
            {
                m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, 0.0, true, true);
                break;
            }
        }
    }
    m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, 0.0, true, true);
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
