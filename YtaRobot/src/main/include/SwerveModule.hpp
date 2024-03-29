////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveModule.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve module on a swerve drive robot.
///
/// Copyright (c) 2023 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef SWERVEMODULE_HPP
#define SWERVEMODULE_HPP

// SYSTEM INCLUDES
#include <cmath>                                        // for M_PI

// C INCLUDES
#include "ctre/Phoenix.h"                               // for CTRE library interfaces
#include "frc/controller/SimpleMotorFeedForward.h"      // for feedforward control
#include "frc/kinematics/SwerveModulePosition.h"        // for struct declaration
#include "frc/kinematics/SwerveModuleState.h"           // for struct declaration
#include "frc/geometry/Rotation2d.h"                    // for class declaration
#include "units/angle.h"                                // for degree user defined literal
#include "units/voltage.h"                              // for voltage unit user defined literals

// C++ INCLUDES
// (none)

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class SwerveModule
///
/// Declarations for a swerve module object.
///
////////////////////////////////////////////////////////////////
class SwerveModule
{
    friend class SwerveDrive;

private:
    enum ModulePosition
    {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    };

    struct SwerveModuleConfig
    {
        const char * m_pModuleName;
        ModulePosition m_Position;
        int m_DriveMotorCanId;
        int m_AngleMotorCanId;
        int m_CanCoderId;
        const Rotation2d m_AngleOffset;
    };

    // Constructor
    SwerveModule(SwerveModuleConfig config);

    // Update a swerve module to the desired state
    void SetDesiredState(SwerveModuleState desiredState, bool bIsOpenLoop);

    // Optimizes the desired swerve module state
    SwerveModuleState Optimize(SwerveModuleState desiredState, Rotation2d currentAngle);

    // Retrieves the swerve module state/position
    SwerveModuleState GetSwerveModuleState();
    SwerveModulePosition GetSwerveModulePosition();

    // Puts useful values on the dashboard
    void UpdateSmartDashboard();

    // Storage space for strings for the smart dashboard
    struct DisplayStrings
    {
        static const unsigned MAX_MODULE_DISPLAY_STRING_LENGTH = 64U;
        char m_CancoderAngleString[MAX_MODULE_DISPLAY_STRING_LENGTH];
        char m_FxEncoderAngleString[MAX_MODULE_DISPLAY_STRING_LENGTH];
        char m_DriveTalonTemp[MAX_MODULE_DISPLAY_STRING_LENGTH];
        char m_AngleTalonTemp[MAX_MODULE_DISPLAY_STRING_LENGTH];
    };
    DisplayStrings m_DisplayStrings;
    static uint32_t m_DetailedModuleDisplayIndex;

    ModulePosition m_MotorGroupPosition;
    TalonFX * m_pDriveTalon;
    TalonFX * m_pAngleTalon;
    CANCoder * m_pAngleCanCoder;
    Rotation2d m_AngleOffset;
    Rotation2d m_LastAngle;
    SimpleMotorFeedforward<units::meters> * m_pFeedForward;

    // Divide by 12 on these constants to convert from volts to percent output for CTRE
    using Distance = units::meters;
    using Velocity = units::compound_unit<Distance, units::inverse<units::seconds>>;
    using Acceleration = units::compound_unit<Velocity, units::inverse<units::seconds>>;
    using kv_unit = units::compound_unit<units::volts, units::inverse<Velocity>>;
    using ka_unit = units::compound_unit<units::volts, units::inverse<Acceleration>>;
    static constexpr units::volt_t KS = (0.32_V / 12.0);
    static constexpr units::unit_t<kv_unit> KV = units::unit_t<kv_unit>(1.51 / 12.0);
    static constexpr units::unit_t<ka_unit> KA = units::unit_t<ka_unit>(0.27 / 12.0);

    // Swerve Profiling Values
    static constexpr double OPEN_LOOP_RAMP = 0.25;
    static constexpr double CLOSED_LOOP_RAMP = 0.0;

    SwerveModule(const SwerveModule &) = delete;
    SwerveModule & operator=(const SwerveModule &) = delete;
};

#endif // SWERVEMODULE_HPP
