////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveDrive.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve drive robot base.
///
/// Copyright (c) 2023 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef SWERVEDRIVE_HPP
#define SWERVEDRIVE_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "ctre/phoenix/sensors/Pigeon2.h"               // for PigeonIMU
#include "frc/geometry/Translation2d.h"                 // for class declaration
#include "frc/kinematics/SwerveDriveKinematics.h"       // for class declaration
#include "frc/kinematics/SwerveDriveOdometry.h"         // for class declaration
#include "frc/kinematics/SwerveModulePosition.h"        // for struct declaration
#include "frc/kinematics/SwerveModuleState.h"           // for struct declaration
#include "units/angle.h"                                // for angle user defined literals
#include "units/angular_velocity.h"                     // for angular velocity user defined literals
#include "units/length.h"                               // for distance user defined literals

// C++ INCLUDES
#include "SwerveModule.hpp"                             // for interacting with a swerve module

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
    SwerveDrive();
    void SetModuleStates(Translation2d translation, double rotation, bool bFieldRelative, bool bIsOpenLoop);
    void UpdateSmartDashboard();

    inline void ZeroGyro()
    {
        m_pPigeon->SetYaw(0.0);
    }

private:
    /*
    inline static Rotation2d GetYaw()
    {
        // This probably isn't needed.  Use GetYaw() directly.  invertGyro = false, so no math needed.
        //double[] ypr = new double[3];
        //gyro.getYawPitchRoll(ypr);
        //return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
        return units::degree_t(0);
    }
    */

    static constexpr const size_t NUM_SWERVE_DRIVE_MODULES = 4U;
    Pigeon2 * m_pPigeon;
    SwerveModule m_SwerveModules[NUM_SWERVE_DRIVE_MODULES];
    SwerveDriveOdometry<NUM_SWERVE_DRIVE_MODULES> m_Odometry;

    static constexpr units::meters_per_second_t MAX_DRIVE_VELOCITY_MPS = 4.5_mps;
    static constexpr units::radians_per_second_t MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 11.5_rad_per_s;

    // Distance between front/back wheel centers 23.5 inches
    static constexpr const units::meter_t WHEEL_BASE = 23.5_m * 0.0254; // 1 inch = 0.0254 meters, 0.5969_m;
    // Distance between left/right wheel centers, 21.5 inches
    static constexpr const units::meter_t TRACK_WIDTH = 21.5_m * 0.0254; // 1 inch = 0.0254 meters, 0.5461_m;

    static const Translation2d FRONT_LEFT_MODULE_T2D;
    static const Translation2d FRONT_RIGHT_MODULE_T2D;
    static const Translation2d BACK_LEFT_MODULE_T2D;
    static const Translation2d BACK_RIGHT_MODULE_T2D;
    static const SwerveDriveKinematics<NUM_SWERVE_DRIVE_MODULES> SWERVE_DRIVE_KINEMATICS;
    static constexpr const SwerveModulePosition INITIAL_SWERVE_MODULE_POSITION = {0_m, 0_deg};

    // @todo_swerve: These angle offsets probably come from calibration.  Reference code is 37.35, 10.45, 38.75, 58.88.
    // position, drive talon CAN, angle talon CAN, CANCoder ID, angle offset
    static constexpr const SwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = {"Front left", SwerveModule::FRONT_LEFT, 1, 2, 1, 158.818_deg};
    static constexpr const SwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = {"Front right", SwerveModule::FRONT_RIGHT, 3, 4, 2, 38.057_deg};
    static constexpr const SwerveModuleConfig BACK_LEFT_MODULE_CONFIG = {"Back left", SwerveModule::BACK_LEFT, 5, 6, 3, 119.883_deg};
    static constexpr const SwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = {"Back right", SwerveModule::BACK_RIGHT, 7, 8, 4, 225.176_deg};
    static const int PIGEON_CAN_ID = 5;

    SwerveDrive(const SwerveDrive &) = delete;
    SwerveDrive & operator=(const SwerveDrive &) = delete;
};

#endif // SWERVEDRIVE_HPP
