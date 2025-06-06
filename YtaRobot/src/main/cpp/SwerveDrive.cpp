////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveDrive.cpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve drive robot base.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/geometry/Translation2d.h"                 // for class declaration
#include "frc/kinematics/ChassisSpeeds.h"               // for struct declaration
#include "frc/kinematics/SwerveDriveKinematics.h"       // for class declaration
#include "frc/kinematics/SwerveModulePosition.h"        // for struct declaration
#include "frc/kinematics/SwerveModuleState.h"           // for struct declaration
#include "frc/smartdashboard/SmartDashboard.h"          // for interacting with the smart dashboard

// C++ INCLUDES
#include "SwerveConfig.hpp"                             // for swerve configuration and constants
#include "SwerveDrive.hpp"                              // for class declaration

using namespace frc;


////////////////////////////////////////////////////////////////
/// @method SwerveDrive::SwerveDrive
///
/// Constructs a swerve drive object.  This is the primary
/// object the robot code will use for generating swerve drive
/// motion.  It constructs the swerve modules and supporting
/// members.  Note that the IMU is on the canivore bus.
///
////////////////////////////////////////////////////////////////
SwerveDrive::SwerveDrive(Pigeon2 * pPigeon) :
    m_pPigeon(pPigeon),
    m_SwerveModules{FRONT_LEFT_MODULE_INFO, FRONT_RIGHT_MODULE_INFO, BACK_LEFT_MODULE_INFO, BACK_RIGHT_MODULE_INFO},
    m_Odometry(SwerveConfig::Kinematics, Rotation2d(units::degree_t(0)), GetModulePositions())
{
    m_pPigeon->SetYaw(0.0_deg);
}


////////////////////////////////////////////////////////////////
/// @method SwerveDrive::GetPose
///
/// Gets the current 2D pose from the swerve module states.
///
////////////////////////////////////////////////////////////////
Pose2d SwerveDrive::GetPose()
{
    return m_Odometry.GetPose();
}


////////////////////////////////////////////////////////////////
/// @method SwerveDrive::SetPose
///
/// Sets the swerve module states to the passed in 2D pose.
///
////////////////////////////////////////////////////////////////
void SwerveDrive::SetPose(Pose2d pose)
{
    // There are several APIs provided by the Odometry class, which SwerveDriveOdometry
    // derives from.  Despite having a ResetPose() option, the example projects call
    // ResetPosition() instead.
    m_Odometry.ResetPosition(m_pPigeon->GetYaw().GetValue(), GetModulePositions(), pose);
}


////////////////////////////////////////////////////////////////
/// @method SwerveDrive::SetModuleStates
///
/// Updates each individual swerve module.  This is the method
/// the robot code will call to pass in predetermined inputs,
/// such as during autonomous
///
////////////////////////////////////////////////////////////////
void SwerveDrive::SetModuleStates(wpi::array<SwerveModuleState, SwerveConfig::NUM_SWERVE_DRIVE_MODULES> swerveModuleStates)
{
    // Desaturate the wheel speeds
    SwerveConfig::Kinematics.DesaturateWheelSpeeds(&swerveModuleStates, SwerveConfig::MAX_DRIVE_VELOCITY_MPS);

    // Set each individual swerve module state
    for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
    {
        m_SwerveModules[i].SetDesiredState(swerveModuleStates[i], false);
    }
}


////////////////////////////////////////////////////////////////
/// @method SwerveDrive::SetModuleStates
///
/// Updates each individual swerve module.  This is the method
/// the robot code will call to pass in e.g. driver joystick
/// inputs.  It supports both field centric and robot centric
/// control options.  It will compute the chassis speeds and use
/// these to compute swerve module states, which are then set.
///
////////////////////////////////////////////////////////////////
void SwerveDrive::SetModuleStates(Translation2d translation, double rotation, bool bFieldRelative, bool bIsOpenLoop)
{
    // The Translation2d passed in is just raw input from the joysticks.
    // It has to be scaled and converted for use with m/s units.
    units::meters_per_second_t xMps = (translation.X().value()) * SwerveConfig::MAX_DRIVE_VELOCITY_MPS;
    units::meters_per_second_t yMps = (translation.Y().value()) * SwerveConfig::MAX_DRIVE_VELOCITY_MPS;
    units::radians_per_second_t rotationRadPerSec = units::radians_per_second_t(rotation * SwerveConfig::MAX_ANGULAR_VELOCITY_RAD_PER_SEC);

    // This can be useful during debug
    SmartDashboard::PutNumber("x mps", xMps.value());
    SmartDashboard::PutNumber("y mps", yMps.value());
    SmartDashboard::PutNumber("rot rps", rotationRadPerSec.value());

    // Compute the chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (bFieldRelative)
    {
        chassisSpeeds = ChassisSpeeds::FromFieldRelativeSpeeds(xMps, yMps, rotationRadPerSec, units::angle::degree_t(m_pPigeon->GetYaw().GetValue()));
    }
    else
    {
        chassisSpeeds = {xMps, yMps, rotationRadPerSec};
    }

    // Capture the desired module states
    wpi::array<SwerveModuleState, SwerveConfig::NUM_SWERVE_DRIVE_MODULES> swerveModuleStates = SwerveConfig::Kinematics.ToSwerveModuleStates(chassisSpeeds);

    // Desaturate the wheel speeds
    SwerveConfig::Kinematics.DesaturateWheelSpeeds(&swerveModuleStates, SwerveConfig::MAX_DRIVE_VELOCITY_MPS);

    // Set each individual swerve module state
    for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
    {
        m_SwerveModules[i].SetDesiredState(swerveModuleStates[i], bIsOpenLoop);
    }
}


////////////////////////////////////////////////////////////////
/// @method SwerveDrive::GetModulePositions
///
/// Retrieves the current swerve module positions.
///
////////////////////////////////////////////////////////////////
wpi::array<SwerveModulePosition, SwerveConfig::NUM_SWERVE_DRIVE_MODULES> SwerveDrive::GetModulePositions()
{
    wpi::array<SwerveModulePosition, SwerveConfig::NUM_SWERVE_DRIVE_MODULES> swerveModulePositions =
    {
        SwerveModulePosition{0.0_m, 0.0_deg},
        SwerveModulePosition{0.0_m, 0.0_deg},
        SwerveModulePosition{0.0_m, 0.0_deg},
        SwerveModulePosition{0.0_m, 0.0_deg}
    };

    for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
    {
        swerveModulePositions[i] = m_SwerveModules[i].GetSwerveModulePosition();
    }

    return swerveModulePositions;
}


////////////////////////////////////////////////////////////////
/// @method SwerveDrive::UpdateSmartDashboard
///
/// Support routine to put useful information on the dashboard.
///
////////////////////////////////////////////////////////////////
void SwerveDrive::UpdateSmartDashboard()
{
    SmartDashboard::PutNumber("Pigeon yaw", m_pPigeon->GetYaw().GetValueAsDouble());
    SmartDashboard::PutNumber("Pigeon pitch", m_pPigeon->GetPitch().GetValueAsDouble());
    SmartDashboard::PutNumber("Pigeon roll", m_pPigeon->GetRoll().GetValueAsDouble());

    for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
    {
        m_SwerveModules[i].UpdateSmartDashboard();
    }
}
