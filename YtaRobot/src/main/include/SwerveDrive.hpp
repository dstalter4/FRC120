////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveDrive.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve drive robot base.
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef SWERVEDRIVE_HPP
#define SWERVEDRIVE_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/geometry/Translation2d.h"                 // for class declaration
#include "frc/kinematics/SwerveDriveOdometry.h"         // for class declaration
#include "frc/kinematics/SwerveModulePosition.h"        // for struct declaration
#include "frc/kinematics/SwerveModuleState.h"           // for struct declaration
#include "units/angle.h"                                // for angle user defined literals
#include "units/angular_velocity.h"                     // for angular velocity user defined literals
#include "units/length.h"                               // for distance user defined literals

// C++ INCLUDES
#include "SwerveConfig.hpp"                             // for swerve configuration and constants
#include "SwerveModule.hpp"                             // for interacting with a swerve module
#include "ctre/phoenix6/Pigeon2.hpp"                    // for PigeonIMU

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class SwerveDrive
///
/// Declarations for a swerve drive object.
///
////////////////////////////////////////////////////////////////
class SwerveDrive
{
    typedef SwerveModule::SwerveModuleConfig SwerveModuleConfig;

public:
    // Constructor
    SwerveDrive(Pigeon2 * pPigeon);

    // Gets the current 2D pose from the swerve module states
    Pose2d GetPose();

    // Sets the swerve module states to the passed in 2D pose
    void SetPose(Pose2d pose);

    // Updates each swerve module based on specified inputs (used by autonomous)
    void SetModuleStates(wpi::array<SwerveModuleState, SwerveConfig::NUM_SWERVE_DRIVE_MODULES> swerveModuleStates);

    // Updates each swerve module based on controller inputs (used by teleop)
    void SetModuleStates(Translation2d translation, double rotation, bool bFieldRelative, bool bIsOpenLoop);

    // Puts useful values on the dashboard
    void UpdateSmartDashboard();

    // Update the odometry
    inline void UpdateOdometry()
    {
        m_Odometry.Update(m_pPigeon->GetYaw().GetValue(), GetModulePositions());
    }

    // Sets the gyro yaw back to zero degrees
    inline void ZeroGyroYaw()
    {
        m_pPigeon->SetYaw(0.0_deg);
    }

    // Points all the modules to zero degrees, which should be straight forward
    inline void HomeModules()
    {
        for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
        {
            m_SwerveModules[i].HomeModule();
        }
    }

    // Lock the wheels in an X pattern to prevent movement
    inline void LockWheels()
    {
        for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
        {
            m_SwerveModules[i].LockWheel();
        }
    }

private:
    wpi::array<SwerveModulePosition, SwerveConfig::NUM_SWERVE_DRIVE_MODULES> GetModulePositions();

    Pigeon2 * m_pPigeon;
    SwerveModule m_SwerveModules[SwerveConfig::NUM_SWERVE_DRIVE_MODULES];

    // From https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
    // 0 degrees / radians represents the robot angle when the robot is facing directly toward your opponent’s
    // alliance station. As your robot turns to the left, your gyroscope angle should increase. By default, WPILib
    // gyros exhibit the opposite behavior, so you should negate the gyro angle.
    SwerveDriveOdometry<SwerveConfig::NUM_SWERVE_DRIVE_MODULES> m_Odometry;

    // Config information on each swerve module.
    // Fields are: Name, Position, Drive TalonFX CAN ID, Angle TalonFX CAN ID, CANCoder ID, Angle Offset
    // 2025 Raw Measurements (bevels right), recal measurements after base module swaps, recal measurements after encoder correction
    // FL: 0.816162_tr (293.81832_deg), 0.315430_tr (113.5548_deg), 0.979492 (352.61712_deg)
    // FR: 0.371582_tr (133.76952_deg), 0.856934_tr (308.49624_deg), 0.540039 (194.41404_deg)
    // BL: 0.037598_tr (13.53528_deg), 0.538330_tr (193.7988_deg), 0.871094 (313.59384_deg)
    // BR: 0.478027_tr (172.08972_deg), 0.972900_tr (350.244_deg), 0.315186 (113.46696_deg)
    static constexpr const SwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = {"Front left", SwerveModule::FRONT_LEFT, 11, 12, 1, 352.61712_deg};
    static constexpr const SwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = {"Front right", SwerveModule::FRONT_RIGHT, 13, 14, 2, 194.41404_deg};
    static constexpr const SwerveModuleConfig BACK_LEFT_MODULE_CONFIG = {"Back left", SwerveModule::BACK_LEFT, 15, 16, 3, 313.59384_deg};
    static constexpr const SwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = {"Back right", SwerveModule::BACK_RIGHT, 17, 18, 4, 113.46696_deg};

    SwerveDrive(const SwerveDrive &) = delete;
    SwerveDrive & operator=(const SwerveDrive &) = delete;
};

#endif // SWERVEDRIVE_HPP
