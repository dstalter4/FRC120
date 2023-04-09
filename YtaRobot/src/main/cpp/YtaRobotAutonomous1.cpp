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

    AutonomousSwerveDriveSequence(RobotDirection::ROBOT_REVERSE, ROBOT_NO_ROTATE, 0.30, 0.0, 3.5_s, true);

    // The extended version attempts to strafe and balance on the station.
    // It requires the robot to start positioned in the spot closest to the opposing human player area.
    static const bool EXTENDED_ROUTINE_1 = false;
    if (EXTENDED_ROUTINE_1)
    {
        switch (m_AllianceColor)
        {
            case DriverStation::Alliance::kRed:
            {
                AutonomousSwerveDriveSequence(RobotDirection::ROBOT_RIGHT, ROBOT_NO_ROTATE, 0.30, 0.0, 2.0_s, true);
                break;
            }
            case DriverStation::Alliance::kBlue:
            {
                AutonomousSwerveDriveSequence(RobotDirection::ROBOT_LEFT, ROBOT_NO_ROTATE, 0.30, 0.0, 2.0_s, true);
                break;
            }
            default:
            {
                break;
            }
        }

        // This section starts the balancing steps
        enum StationState
        {
            ASCENDING_STATION,
            BALANCING_STATION,
            BALANCED
        };
        StationState stationState = ASCENDING_STATION;

        uint32_t decreasingRollCount = 0U;
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
                    // Drive forward onto the ramp
                    m_pSwerveDrive->SetModuleStates({0.30_m, 0.0_m}, 0.0, true, true);

                    // If the pitch is over abs(10 degrees), we can start to try and balance
                    if (currentRoll > 10.0)
                    {
                        lastRoll = currentRoll;
                        stationState = BALANCING_STATION;
                        m_pCandle->SetLEDs(255, 255, 255, 0, 0, NUMBER_OF_LEDS);

                        // The delay is because the ramp angle bounces as the robot moves it.
                        // Give it some time to climb.
                        // If this fails, go back to 0.20_m and no currentRoll check.
                        m_pSwerveDrive->SetModuleStates({0.15_m, 0.0_m}, 0.0, true, true);
                        AutonomousDelay(0.5_s);
                    }

                    break;
                }
                case BALANCING_STATION:
                {
                    // Logic issued fixed, does it make a difference?
                    if ((currentRoll < lastRoll) && (currentRoll < 12.0))
                    {
                        decreasingRollCount++;
                        m_pCandle->SetLEDs(0, 0, 255, 0, 0, (NUMBER_OF_LEDS / 3 * decreasingRollCount));
                    }
                    else if (currentRoll < 2.0)
                    {
                        decreasingRollCount++;
                        m_pCandle->SetLEDs(255, 0, 0, 0, 0, (NUMBER_OF_LEDS / 3 * decreasingRollCount));
                    }
                    else
                    {
                        decreasingRollCount = 0U;
                        m_pCandle->SetLEDs(255, 255, 255, 0, 0, NUMBER_OF_LEDS);
                    }

                    // If we see three consecutive increasing roll counts, lock wheels and balance
                    if (decreasingRollCount == 3U)
                    {
                        m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, 1.0, true, true);
                        AutonomousDelay(0.02_s);
                        m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, 0.0, true, true);
                        stationState = BALANCED;
                        Wait(1.0_s);
                    }

                    lastRoll = currentRoll;
                    AutonomousDelay(0.1_s);

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
        m_pCandle->SetLEDs(0, 255, 0, 0, 0, NUMBER_OF_LEDS);
        m_pSafetyTimer->Stop();
        m_pSafetyTimer->Reset();
    }

    m_pPigeon->SetYaw(180);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
