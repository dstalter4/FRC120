////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routines for YtaRobot.
///
/// Copyright (c) 2022 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                 // for robot class declaration
#include "YtaRobotAutonomous.hpp"       // for autonomous declarations
#include "RobotCamera.hpp"              // for interacting with cameras

// NAMESPACE DATA
bool YtaRobotAutonomous::bAutonomousExecutionComplete;


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousInit
///
/// The autonomous init method.  This method is called once each
/// time the robot enters autonomous control.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousInit()
{
    RobotUtils::DisplayMessage("AutonomousInit called.");
    
    // Put everything in a stable state
    InitialStateSetup();

    switch (m_AllianceColor)
    {
        case DriverStation::Alliance::kRed:
        {
            m_pCandle->SetLEDs(255, 0, 0, 0, 0, NUMBER_OF_LEDS);
            break;
        }
        case DriverStation::Alliance::kBlue:
        {
            m_pCandle->SetLEDs(0, 0, 255, 0, 0, NUMBER_OF_LEDS);
            break;
        }
        default:
        {
            break;
        }
    }
    
    // Indicate the autonomous routine has not executed yet
    YtaRobotAutonomous::bAutonomousExecutionComplete = false;
    
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Autonomous needs full camera processing
    RobotCamera::SetFullProcessing(true);
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::VISION_PROCESSOR);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::ARRAY_ON);
    
    // Indicate to the I2C thread to get data more often
    RobotI2c::SetThreadUpdateRate(YtaRobotAutonomous::I2C_THREAD_UPDATE_RATE_MS);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousPeriodic
///
/// The autonomous control method.  This method is called
/// periodically while the robot is in autonomous control.
/// Even though this method wil be called periodically, it
/// deliberately checks the driver station state controls.
/// This is to give finer control over the autonomous state
/// machine flow.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_AUTONOMOUS);
    
    if (YtaRobotAutonomous::bAutonomousExecutionComplete)
    {
        return;
    }
    
    // @todo: Figure out how to kick the watchdog from here so it doesn't overrun.
    // @note: Since autonomous is configured as a one shot state machine, there will definitely be watchdog overrun.
    
    // Change values in the header to control having an
    // autonomous routine and which is selected
    
    // Get the selected autonomous routine from the smart dashboard
    std::string selectedAutoRoutineString = m_AutonomousChooser.GetSelected();
    
    // Auto routine 1
    //if ( YtaRobotAutonomous::ROUTINE_1 )
    if (selectedAutoRoutineString == AUTO_ROUTINE_1_STRING)
    {
        RobotUtils::DisplayMessage("Auto routine 1.");
        AutonomousRoutine1();
    }
    
    // Auto routine 2
    //else if ( YtaRobotAutonomous::ROUTINE_2 )
    else if (selectedAutoRoutineString == AUTO_ROUTINE_2_STRING)
    {
        RobotUtils::DisplayMessage("Auto routine 2.");
        AutonomousRoutine2();
    }
    
    // Auto routine 3
    //else if ( YtaRobotAutonomous::ROUTINE_3 )
    else if (selectedAutoRoutineString == AUTO_ROUTINE_3_STRING)
    {
        RobotUtils::DisplayMessage("Auto routine 3.");
        AutonomousRoutine3();
    }

    /* !!! ONLY ENABLE TEST AUTONOMOUS CODE WHEN TESTING
           SELECT A FUNCTIONING ROUTINE FOR ACTUAL MATCHES !!! */
    //else if ( YtaRobotAutonomous::TEST_ENABLED )
    else if (selectedAutoRoutineString == AUTO_TEST_ROUTINE_STRING)
    {
        RobotUtils::DisplayMessage("Auto test code.");
        AutonomousTestRoutine();
    }

    else
    {
        // No option was selected; ensure known behavior to avoid issues
        RobotUtils::DisplayMessage("No auto selection made, going idle.");
    }
    
    // One shot through autonomous is over, indicate as such.
    YtaRobotAutonomous::bAutonomousExecutionComplete = true;
    
    /*
    // Idle until auto is terminated
    RobotUtils::DisplayMessage("Auto idle loop.");
    while ( m_pDriverStation->IsAutonomous() && m_pDriverStation->IsEnabled() )
    {
    }
    */
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousPlaceGamePiece
///
/// Autonomous routine to place a game piece.  This code will
/// check the smart dashboard input for whether to place a cube
/// or a cone.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousPlaceGamePiece()
{
    // Get the selected autonomous routine from the smart dashboard
    std::string selectedGamePiece = m_AutonomousGamePieceChooser.GetSelected();
    bool bIsCube = false;
    if (selectedGamePiece == "Cube")
    {
        bIsCube = true;

        // LEDs purple to indicate a cube
        m_pCandle->SetLEDs(240, 73, 241, 0, 0, NUMBER_OF_LEDS);
    }
    else
    {
        // LEDs yellow to indicate a cone
        m_pCandle->SetLEDs(255, 240, 0, 0, 0, NUMBER_OF_LEDS);
    }
    const double INTAKE_OUT_MULTIPLIER = bIsCube ? (-1.0) : (+1.0);

    // If placing a cone, turn the intake motor on to hold the piece in place
    if (!bIsCube)
    {
        m_pIntakeMotor->Set(ControlMode::PercentOutput, -INTAKE_IN_CONE_MOTOR_SPEED);
    }

    // Extend the lift
    m_pCarriageMotors->GetMotorObject()->Set(ControlMode::Position, CARRIAGE_MAX_FIXED_ENCODER_POSITION);
    Wait(1.0_s);

    // Enable the intake motor to place a game piece
    m_pIntakeMotor->Set(ControlMode::PercentOutput, (INTAKE_OUT_MULTIPLIER * INTAKE_OUT_MOTOR_SPEED));
    Wait(1.0_s);
    m_pIntakeMotor->Set(ControlMode::PercentOutput, 0.0);

    // Retract the lift
    m_pCarriageMotors->GetMotorObject()->Set(ControlMode::Position, CARRIAGE_MIN_FIXED_ENCODER_POSITION);
    Wait(1.0_s);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousChargeStationSequence
///
/// Autonomous routine to balance on the charge station.  This
/// routine will first leave the community by driving over the
/// charge station, then return back and balance.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousChargeStationSequence()
{
    enum StationState
    {
        ASCENDING_STATION,
        CROSSING_STATION,
        DESCENDING_STATION,
        REASCEND_STATION,
        BALANCING_STATION,
        BALANCED
    };
    StationState stationState = ASCENDING_STATION;

    uint32_t decreasingRollCount = 0U;
    double lastRoll = 0.0;

    m_pSafetyTimer->Start();
    units::second_t startTime = m_pSafetyTimer->Get();
    while ((stationState != BALANCED) && ((m_pSafetyTimer->Get() - startTime) < 12.0_s))
    //while ((stationState != BALANCED) && DriverStation::IsAutonomousEnabled())
    {
        double currentRoll = m_pPigeon->GetRoll();

        switch (stationState)
        {
            case ASCENDING_STATION:
            {
                // Drive backward onto the ramp
                m_pSwerveDrive->SetModuleStates({-0.30_m, 0.0_m}, 0.0, true, true);

                // If the pitch is over abs(10 degrees), we can start the descent drive
                if (currentRoll < -10.0)
                {
                    stationState = CROSSING_STATION;
                    m_pCandle->SetLEDs(255, 255, 255, 0, 0, NUMBER_OF_LEDS);

                    // The delay is because the ramp angle bounces as the robot moves it.
                    // Give it some time to climb.
                    AutonomousDelay(0.5_s);
                }

                break;
            }
            case CROSSING_STATION:
            {
                // Drop off some speed, continue driving over the ramp
                m_pSwerveDrive->SetModuleStates({-0.25_m, 0.0_m}, 0.0, true, true);

                // When the pitch changes the other way, the robot is now going down the ramp
                if (currentRoll > 10.0)
                {
                    stationState = DESCENDING_STATION;
                }

                break;
            }
            case DESCENDING_STATION:
            {
                // At this point the robot should be clearly descending the back.
                // Just do a timed drive to exit the community.
                AutonomousSwerveDriveSequence(RobotDirection::ROBOT_REVERSE, ROBOT_NO_ROTATE, 0.20, 0.0, 1.0_s, true);
                AutonomousDelay(0.5_s);
                stationState = REASCEND_STATION;
                m_pCandle->SetLEDs(0, 0, 0, 0, 0, NUMBER_OF_LEDS);
                break;
            }
            case REASCEND_STATION:
            {
                // Drive forward back onto the ramp
                m_pSwerveDrive->SetModuleStates({0.30_m, 0.0_m}, 0.0, true, true);

                // If the pitch is over abs(10 degrees), we can start to try and balance
                if (currentRoll > 10.0)
                {
                    lastRoll = currentRoll;
                    stationState = BALANCING_STATION;
                    m_pCandle->SetLEDs(255, 255, 255, 0, 0, NUMBER_OF_LEDS);

                    // The delay is because the ramp angle bounces as the robot moves it.
                    // Slow down and give it some time to climb.
                    // GPR: Started with 0.25_s, too short.
                    // If this fails, go back to 0.20_m and no currentRoll check.
                    m_pSwerveDrive->SetModuleStates({0.12_m, 0.0_m}, 0.0, true, true);
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
    m_pPigeon->SetYaw(180);
    m_pCandle->SetLEDs(0, 255, 0, 0, 0, NUMBER_OF_LEDS);
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousCommon
///
/// Common autonomous behavior.  It moves away from the alliance
/// wall and to the fuel loading station.  The variance is
/// whether it shoots at the start or at the end.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousCommon()
{

    if (m_AllianceColor == DriverStation::Alliance::kRed)
    {
        AutonomousCommonRed();
    }
    else if (m_AllianceColor == DriverStation::Alliance::kBlue)
    {
        AutonomousCommonBlue();
    }
    else
    {
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousCommonRed
///
/// Common autonomous behavior when on the red alliance.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousCommonRed()
{
}





////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///                           Red/Blue Separation                            ///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////
// @method YtaRobot::AutonomousCommonBlue
///
/// Common autonomous behavior when on the blue alliance.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousCommonBlue()
{
}
