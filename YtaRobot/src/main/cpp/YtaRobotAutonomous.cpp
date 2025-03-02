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
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandScheduler.h>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                 // for robot class declaration
#include "YtaRobotAutonomous.hpp"       // for autonomous declarations
#include "YtaRobotAutonomousCommand.hpp"       // for autonomous command declarations
#include "RobotCamera.hpp"              // for interacting with cameras
#include "RobotUtils.hpp"

// NAMESPACE DATA
bool YtaRobotAutonomous::bAutonomousExecutionComplete;
AutonomousSubsystem YtaRobotAutonomous::m_AutonomousSubsystem;


std::optional<CommandPtr> f_AutonomousCommand;
CommandPtr GetCommand()
{
    static uint32_t l1;
    static uint32_t l2;
    static uint32_t l3;
    return cmd::Sequence
    (
        InstantCommand([](){SmartDashboard::PutNumber("Autonomous lambda 1", ++l1);}).ToPtr(),
        InstantCommand([](){SmartDashboard::PutNumber("Autonomous lambda 2", ++l2);}).ToPtr(),
        InstantCommand([](){SmartDashboard::PutNumber("Autonomous lambda 3", ++l3);}).ToPtr()
    );
}


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
        //m_AutonomousCommand = m_RobotContainer.GetAutonomousCommand();
        //if (m_AutonomousCommand)
        //{
        //    m_AutonomousCommand->Schedule();
        //}
        RobotUtils::DisplayMessage("Autonomous init - command based.");
        //static AutonomousHelperCommand autonomousCommand(&YtaRobotAutonomous::m_AutonomousSubsystem);
        //autonomousCommand.Schedule();
        
        //CommandPtr autonomousCommand = AutonomousTestTrajectoryRoutine();
        //std::optional<CommandPtr> autonomousCommand = GetCommand();
        // Scheduled commands must have non-local scope!  Otherwise the scheduler does not continue to see them.
        f_AutonomousCommand = GetCommand();
        if (f_AutonomousCommand)
        {
            RobotUtils::DisplayMessage("Autonomous init - command scheduled.");
            f_AutonomousCommand->Schedule();
        }
        else
        {
            RobotUtils::DisplayMessage("Autonomous init - command NOT scheduled.");
        }
        /*
        static uint32_t l1;
        static uint32_t l2;
        static uint32_t l3;
        cmd::Sequence
        (
            InstantCommand([this](){SmartDashboard::PutNumber("Autonomous lambda 1", ++l1);}).ToPtr(),
            InstantCommand([this](){SmartDashboard::PutNumber("Autonomous lambda 2", ++l2);}).ToPtr(),
            InstantCommand([this](){SmartDashboard::PutNumber("Autonomous lambda 3", ++l3);}).ToPtr()
        ).Schedule();
        */
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
        //RobotUtils::DisplayMessage("Autonomous periodic - command based.");
        // This updates the odometry so it is accurate for the command running
        static uint32_t ac;
        SmartDashboard::PutNumber("Autonomous periodic counter", ac++);
        m_pSwerveDrive->UpdateSmartDashboard();
        //static std::optional<CommandPtr> autonomousCommand = AutonomousTestTrajectoryRoutine();
        //autonomousCommand->Schedule();
        CommandScheduler::GetInstance().Run();
        return;
    }
    
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
    //YtaRobotAutonomous::bAutonomousExecutionComplete = true;
    
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
