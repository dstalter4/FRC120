////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routines for YtaRobot.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <frc2/command/CommandScheduler.h>      // for scheduling commands

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                         // for robot class declaration
#include "YtaRobotAutonomous.hpp"               // for autonomous declarations
#include "RobotCamera.hpp"                      // for interacting with cameras
#include "RobotUtils.hpp"                       // for DisplayMessage()

// NAMESPACE DATA
bool YtaRobotAutonomous::bAutonomousExecutionComplete;
std::optional<CommandPtr> AutonomousCommand;


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
    
    // Indicate the autonomous routine has not executed yet
    YtaRobotAutonomous::bAutonomousExecutionComplete = false;
    
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Autonomous needs full camera processing
    RobotCamera::SetFullProcessing(true);
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::VISION_PROCESSOR);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);
    RobotCamera::SetLimelightPipeline(0);

    if (YtaRobotAutonomous::USE_COMMAND_BASED_AUTONOMOUS)
    {
        RobotUtils::DisplayMessage("Autonomous init - command based.");

        // Scheduled commands must have non-local scope!
        // Otherwise the scheduler does not continue to see them.
        AutonomousCommand = AutonomousTestCommandDashboardRoutine();
        //AutonomousCommand = AutonomousTestCommandMotionRoutine();
        //AutonomousCommand = AutonomousTestTrajectoryRoutine();

        if (AutonomousCommand.has_value())
        {
            RobotUtils::DisplayMessage("Autonomous init - command scheduled.");
            AutonomousCommand->Schedule();
        }
        else
        {
            RobotUtils::DisplayMessage("Autonomous init - command NOT scheduled.");
        }
    }
    else
    {
        RobotUtils::DisplayMessage("Autonomous init - time based.");
    }
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

    if (YtaRobotAutonomous::USE_COMMAND_BASED_AUTONOMOUS)
    {
        AutonomousPeriodicCommand();
    }
    else
    {
        AutonomousPeriodicTimed();
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousPeriodicCommand
///
/// Main workflow for command based autonomous routines.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousPeriodicCommand()
{
    // Update the swerve odometry and run the command scheduler
    // @todo: The odometry updates may be causing drift
    m_pSwerveDrive->UpdateOdometry();
    CommandScheduler::GetInstance().Run();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousPeriodicTimed
///
/// Main workflow for time based autonomous routines.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousPeriodicTimed()
{    
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
    
    // No autonomous routine
    else if (selectedAutoRoutineString == AUTO_NO_ROUTINE_STRING)
    {
        RobotUtils::DisplayMessage("No autonomous routine.");
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
