////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous2.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 2 for YtaRobot.
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
/// @method YtaRobot::AutonomousRoutine2
///
/// Autonomous routine 2.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine2()
{
    // Extend the lift
    m_pCarriageMotors->GetMotorObject()->Set(ControlMode::Position, 152'000);
    Wait(1.0_s);

    // Enable the intake motor to spit out a cube
    m_pIntakeMotor->Set(ControlMode::PercentOutput, -0.35);
    Wait(1.0_s);
    m_pIntakeMotor->Set(ControlMode::PercentOutput, 0.0);

    // Retract the lift
    m_pCarriageMotors->GetMotorObject()->Set(ControlMode::Position, 1000);
    Wait(1.0_s);



    enum StationState
    {
        ASCENDING_STATION,
        BALANCING_STATION,
        BALANCED
    };
    StationState stationState = ASCENDING_STATION;

    uint32_t increasingRollCount = 0U;
    double lastRoll = 0.0;

    m_pSafetyTimer->Start();
    units::second_t startTime = m_pSafetyTimer->Get();
    while ((stationState != BALANCED) && ((m_pSafetyTimer->Get() - startTime) < 7.0_s))
    //while ((stationState != BALANCED) && DriverStation::IsAutonomousEnabled())
    {
        double currentRoll = m_pPigeon->GetRoll();

        switch (stationState)
        {
            case ASCENDING_STATION:
            {
                // Drive backward onto the ramp
                m_pSwerveDrive->SetModuleStates({-0.30_m, 0.0_m}, 0.0, true, true);

                // If the pitch is over abs(10 degrees), we can start to try and balance
                if (currentRoll < -10.0)
                {
                    lastRoll = currentRoll;
                    stationState = BALANCING_STATION;
                    m_pCandle->SetLEDs(255, 255, 255, 0, 0, NUMBER_OF_LEDS);

                    // The delay is because the ramp angle bounces as the robot moves it.
                    // Give it some time to climb.
                    AutonomousDelay(0.5_s);
                }

                break;
            }
            case BALANCING_STATION:
            {
                // This is a logic issue that wasn't fixed when the pitch direction changed, but it's working
                if (currentRoll < lastRoll)
                {
                    increasingRollCount++;
                }
                else
                {
                    increasingRollCount = 0U;
                }

                // If we see three consecutive decreasing roll counts, lock wheels and balance
                if (increasingRollCount == 3U)
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
    m_pCarriageMotors->Set(0.0);
    m_pPigeon->SetYaw(180);
    m_pCandle->SetLEDs(0, 255, 0, 0, 0, NUMBER_OF_LEDS);
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 2 done.");
}
