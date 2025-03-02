////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomousTest.cpp
/// @author David Stalter
///
/// @details
/// Implementation of an autonomous test routines for YtaRobot.
///
/// Copyright (c) 2024 Youth Technology Academy
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
/// @method YtaRobot::AutonomousTestTrajectoryRoutine
///
/// Autonomous swerve test routine.
///
////////////////////////////////////////////////////////////////
CommandPtr YtaRobot::AutonomousTestTrajectoryRoutine()
{
RobotUtils::DisplayMessage("Autonomous trajectory - enter.");
    // Swerve trajectory routine, but requires switching to command based robot.

    //TrajectoryConfig config =
    //new TrajectoryConfig(
    //        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    //        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //    .setKinematics(Constants.Swerve.swerveKinematics);
    TrajectoryConfig trajectoryConfig = {0.5_mps, 1.0_mps_sq};
    trajectoryConfig.SetKinematics(SwerveConfig::Kinematics);

RobotUtils::DisplayMessage("Autonomous trajectory - point 1.");
    // An example trajectory to follow.  All units in meters.
    //Trajectory exampleTrajectory =
    //TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        //new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        //new Pose2d(3, 0, new Rotation2d(0)),
        //config);
    const Pose2d INITIAL_POSE = {0_m, 0_m, 0_deg};
    const Pose2d FINAL_POSE = {2_m, 0_m, 0_deg};
    const std::vector<Translation2d> WAY_POINTS = 
    {
        {1_m, 0_m}
    };
    Trajectory testTrajectory = TrajectoryGenerator::GenerateTrajectory(INITIAL_POSE, WAY_POINTS, FINAL_POSE, trajectoryConfig);
    //(void)testTrajectory;
RobotUtils::DisplayMessage("Autonomous trajectory - point 2.");

    //var thetaController =
    //new ProfiledPIDController(
    //    Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    //thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // Values are max velocity, max acceleration
    TrapezoidProfile<units::radians>::Constraints thetaPidControllerConstraints(3.1415_rad_per_s, 3.1415_rad_per_s_sq);
    // Parameters are P, I, D, TrapezoidProfile<>::Constraints
    ProfiledPIDController<units::radians> thetaPidController(0.5, 0.0, 0.0, thetaPidControllerConstraints);
    thetaPidController.EnableContinuousInput(units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi));

RobotUtils::DisplayMessage("Autonomous trajectory - point 3.");
    //SwerveControllerCommand swerveControllerCommand =
    //new SwerveControllerCommand(
    //    exampleTrajectory,
    //    s_Swerve::getPose,
    //    Constants.Swerve.swerveKinematics,
    //    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //    thetaController,
    //    s_Swerve::setModuleStates,
    //    s_Swerve);
    PIDController xPidController(0.5, 0.0, 0.0);
    PIDController yPidController(0.5, 0.0, 0.0);
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

    //addCommands(
    //    new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
    //    swerveControllerCommand
    //);
/*
    return cmd::Sequence
    (
        InstantCommand([this, initialPose = testTrajectory.InitialPose()]() { m_pSwerveDrive->SetPose(initialPose); }, {}).ToPtr(),
        std::move(swerveControllerCommand),
        InstantCommand([this] { m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, 0.0, false, false); }, {}).ToPtr()
    );
*/
    RobotUtils::DisplayMessage("Autonomous trajectory - before sequencing.");
    static uint32_t l1;
    static uint32_t l2;
    static uint32_t l3;
    return cmd::Sequence
    (
        InstantCommand([this](){SmartDashboard::PutNumber("Autonomous lambda 1", ++l1);}).ToPtr(),
        InstantCommand([this](){SmartDashboard::PutNumber("Autonomous lambda 2", ++l2);}).ToPtr(),
        InstantCommand([this](){SmartDashboard::PutNumber("Autonomous lambda 3", ++l3);}).ToPtr()
    );
    RobotUtils::DisplayMessage("Autonomous trajectory - after sequencing.");

    // Returning from here will enter the idle state until autonomous is over
    //RobotUtils::DisplayMessage("Auto test trajectory routine done.");
}
