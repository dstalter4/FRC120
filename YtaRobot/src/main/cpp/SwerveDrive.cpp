////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveDrive.cpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve drive robot base.
///
/// Copyright (c) 2022 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/kinematics/ChassisSpeeds.h"           // for struct declaration
#include "frc/kinematics/SwerveModuleState.h"       // for struct declaration

// C++ INCLUDES
#include "SwerveDrive.hpp"                          // for class declaration

using namespace frc;


// From https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
// 0 degrees / radians represents the robot angle when the robot is facing directly toward your opponent’s
// alliance station. As your robot turns to the left, your gyroscope angle should increase. By default, WPILib
// gyros exhibit the opposite behavior, so you should negate the gyro angle.

const Translation2d SwerveDrive::FRONT_LEFT_MODULE_T2D = {WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0};
const Translation2d SwerveDrive::FRONT_RIGHT_MODULE_T2D = {WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0};
const Translation2d SwerveDrive::BACK_LEFT_MODULE_T2D = {-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0};
const Translation2d SwerveDrive::BACK_RIGHT_MODULE_T2D = {-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0};
const SwerveDriveKinematics<SwerveDrive::NUM_SWERVE_DRIVE_MODULES> SwerveDrive::SWERVE_DRIVE_KINEMATICS
{
    FRONT_LEFT_MODULE_T2D,
    FRONT_RIGHT_MODULE_T2D,
    BACK_LEFT_MODULE_T2D,
    BACK_RIGHT_MODULE_T2D
};

SwerveDrive::SwerveDrive() :
    m_pPigeon(new Pigeon2(PIGEON_CAN_ID, "canivore")),
    m_Odometry(SWERVE_DRIVE_KINEMATICS, Rotation2d(units::degree_t(0))),
    m_SwerveModules{FRONT_LEFT_MODULE_CONFIG, FRONT_RIGHT_MODULE_CONFIG, BACK_LEFT_MODULE_CONFIG, BACK_RIGHT_MODULE_CONFIG}
{
    m_pPigeon->ConfigFactoryDefault();
    m_pPigeon->SetYaw(0.0);
}

void SwerveDrive::SetModuleStates(Translation2d translation, double rotation, bool bFieldRelative, bool bIsOpenLoop)
{
    /*
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getYaw()
                            )
                            : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation)
                            );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
    */

    // The Translation2d passed in is just raw input from the joysticks.
    // It has to be scaled and converted for use with m/s units.
    units::meters_per_second_t xMps = (translation.X().value()) * MAX_DRIVE_VELOCITY_MPS;
    units::meters_per_second_t yMps = (translation.Y().value()) * MAX_DRIVE_VELOCITY_MPS;
    units::radians_per_second_t rotationRadPerSec = units::radians_per_second_t(rotation * MAX_ANGULAR_VELOCITY_RAD_PER_SEC);

    ChassisSpeeds chassisSpeeds;
    if (bFieldRelative)
    {
        chassisSpeeds = ChassisSpeeds::FromFieldRelativeSpeeds(xMps, yMps, rotationRadPerSec, units::angle::degree_t(m_pPigeon->GetYaw()));
    }
    else
    {
        chassisSpeeds = {xMps, yMps, rotationRadPerSec};
    }

    //auto [fl, fr, bl, br] =
    wpi::array<SwerveModuleState, NUM_SWERVE_DRIVE_MODULES> swerveModuleStates = SWERVE_DRIVE_KINEMATICS.ToSwerveModuleStates(chassisSpeeds);

    //SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    SWERVE_DRIVE_KINEMATICS.DesaturateWheelSpeeds(&swerveModuleStates, MAX_DRIVE_VELOCITY_MPS);

    //for(SwerveModule mod : mSwerveMods){
    //mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    //}
    for (uint32_t i = 0U; i < NUM_SWERVE_DRIVE_MODULES; i++)
    {
        m_SwerveModules[i].SetDesiredState(swerveModuleStates[i], bIsOpenLoop);
    }
}
