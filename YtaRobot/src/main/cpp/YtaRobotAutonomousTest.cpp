////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomousTest.cpp
/// @author David Stalter
///
/// @details
/// Implementation of an autonomous test routines for YtaRobot.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/geometry/Pose2d.h"                    // for type declaration
#include "frc/geometry/Rotation2d.h"                // for type declaration
#include "frc/geometry/Translation2d.h"             // for type declaration
#include "frc/trajectory/Trajectory.h"              // for working with trajectories
#include "frc/trajectory/TrajectoryConfig.h"        // for creating a trajectory config
#include "frc/trajectory/TrajectoryGenerator.h"     // for generating a trajectory
#include "frc/trajectory/TrapezoidProfile.h"
#include "frc/controller/ProfiledPIDController.h"
#include "frc2/command/Commands.h"
#include "frc2/command/SwerveControllerCommand.h"

// C++ INCLUDES
#include "RobotUtils.hpp"                           // for DisplayMessage()
#include "YtaRobot.hpp"                             // for robot class declaration
#include "YtaRobotAutonomous.hpp"                   // for autonomous declarations
#include "SwerveConfig.hpp"
#include "SwerveDrive.hpp"


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousTestRoutine
///
/// Autonomous test routine.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousTestRoutine()
{
    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto test routine done.");
}


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousTestSwerveRoutine
///
/// Autonomous swerve test routine.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousTestSwerveRoutine()
{
    // Simple demonstration of directional movements
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, RobotStrafe::ROBOT_STRAFE_LEFT, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, RobotStrafe::ROBOT_STRAFE_RIGHT, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.0, 0.10, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_COUNTER_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.0, 0.10, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_STRAFE_RIGHT, RobotRotation::ROBOT_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.10, 0.10, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_STRAFE_LEFT, RobotRotation::ROBOT_COUNTER_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.10, 0.10, 1.0_s, true);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto test swerve routine done.");
}


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousTestCommandDashboardRoutine
///
/// Autonomous test demonstration routine that uses commands.
/// Runs a few instant commands sequenced together passing in
/// simple lambdas to display things on the smart dashboard.
///
////////////////////////////////////////////////////////////////
CommandPtr YtaRobot::AutonomousTestCommandDashboardRoutine()
{
    static uint32_t lambda1Var;
    static uint32_t lambda2Var;
    static uint32_t lambda3Var;
    return cmd::Sequence
    (
        InstantCommand([](){SmartDashboard::PutNumber("Autonomous lambda 1", ++lambda1Var);}).ToPtr(),
        InstantCommand([](){SmartDashboard::PutNumber("Autonomous lambda 2", ++lambda2Var);}).ToPtr(),
        InstantCommand([](){SmartDashboard::PutNumber("Autonomous lambda 3", ++lambda3Var);}).ToPtr()
    );

}


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousTestCommandMotionRoutine
///
/// Autonomous test demonstration routine that uses commands.
/// Runs a few instant commands that produce simple motion.
///
////////////////////////////////////////////////////////////////
CommandPtr YtaRobot::AutonomousTestCommandMotionRoutine()
{
    return cmd::Sequence
    (
        InstantCommand([this]()
        {
            m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
            AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
        }).ToPtr(),
        InstantCommand([this]()
        {
            m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
            AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
        }).ToPtr()
    );
}


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousTestTrajectoryRoutine
///
/// Autonomous swerve test trajectory routine.
///
////////////////////////////////////////////////////////////////
CommandPtr YtaRobot::AutonomousTestTrajectoryRoutine()
{
    // Swerve trajectory routine, but requires switching to command based robot.

    // Constructed with max velocity, max acceleration
    TrajectoryConfig trajectoryConfig = {1.5_mps, 3.0_mps_sq};
    trajectoryConfig.SetKinematics(SwerveConfig::Kinematics);

    // An example trajectory to follow.  All units in meters.
    // 1. Start at the origin facing the +X direction (INITIAL_POSE)
    // 2. Pass through these two interior waypoints, making an 's' curve path (WAY_POINTS)
    // 3. End three meters straight ahead of the start position, facing forward (FINAL_POSE)
    // Note: First pose parameter is field y-axis (positive is forward)
    //       Second pose parameter is field x-axis (positive is left)
    const Pose2d INITIAL_POSE = {0_m, 0_m, 0_deg};
    const Pose2d FINAL_POSE = {3_m, 0_m, 0_deg};
    const std::vector<Translation2d> WAY_POINTS = 
    {
        {1.0_m,  1.0_m},
        {2.0_m, -1.0_m}
    };
    Trajectory testTrajectory = TrajectoryGenerator::GenerateTrajectory(INITIAL_POSE, WAY_POINTS, FINAL_POSE, trajectoryConfig);

    // Translation PID controllers
    PIDController xPidController(1.0, 0.0, 0.0);
    PIDController yPidController(1.0, 0.0, 0.0);

    // Values are max angular velocity, max angular acceleration
    TrapezoidProfile<units::radians>::Constraints thetaPidControllerConstraints(3.1415_rad_per_s, 3.1415_rad_per_s_sq);
    
    // Parameters are P, I, D, TrapezoidProfile<>::Constraints
    ProfiledPIDController<units::radians> thetaPidController(0.5, 0.0, 0.0, thetaPidControllerConstraints);

    thetaPidController.EnableContinuousInput(units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi));

    // Construct the swerve controller command
    CommandPtr swerveControllerCommand = 
    SwerveControllerCommand<SwerveConfig::NUM_SWERVE_DRIVE_MODULES>
    (
        testTrajectory,
        [this](){return m_pSwerveDrive->GetPose();},
        SwerveConfig::Kinematics,
        xPidController,
        yPidController,
        thetaPidController,
        [this](auto moduleStates){ m_pSwerveDrive->SetModuleStates(moduleStates); }
        // Last parameter is requirements and (maybe?) can be defaulted
    ).ToPtr();

    // Build the command sequence and return it
    return cmd::Sequence
    (
        InstantCommand([this, initialPose = testTrajectory.InitialPose()]() { m_pSwerveDrive->SetPose(initialPose); }, {}).ToPtr(),
        std::move(swerveControllerCommand),
        InstantCommand([this] { m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, 0.0, false, false); }, {}).ToPtr()
    );
}
