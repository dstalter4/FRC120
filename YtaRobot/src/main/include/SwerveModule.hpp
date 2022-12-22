////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveModule.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve module on a swerve drive robot.
///
/// Copyright (c) 2022 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef SWERVEMODULE_HPP
#define SWERVEMODULE_HPP

// SYSTEM INCLUDES
#include <cmath>                                        // for M_PI

// C INCLUDES
#include "ctre/Phoenix.h"                               // for CTRE library interfaces
#include "frc/controller/SimpleMotorFeedForward.h"      // for feedforward control
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
public:
    enum ModulePosition
    {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    };

    struct SwerveModuleConfig
    {
        ModulePosition m_Position;
        int m_DriveMotorCanId;
        int m_AngleMotorCanId;
        int m_CanCoderId;
        double m_AngleOffset;
    };

    SwerveModule(SwerveModuleConfig config);

    void SetDesiredState(SwerveModuleState desiredState, bool bIsOpenLoop);

    SwerveModuleState GetSwerveModuleState();

private:
    SwerveModuleState Optimize(SwerveModuleState desiredState, Rotation2d currentAngle);
    /*
    inline SwerveModuleState GetSwerveModuleState()
    {
        // https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution
        // Units per rotation: 2048 (FX integrated sensor)
        // From FX user guide: kMaxRPM = Free Speed RPM = 6380 RPM
        // Calculate the expect peak sensor velocity (sensor units per 100ms) as:
        // Vsensor_max = (kMaxRPM  / 600) * (kSensorUnitsPerRotation / kGearRatio)
        // Read sensor velocity and solve above equation for kMaxRPM term for any RPM.

        double motorRpm = m_pDriveTalon->GetSelectedSensorVelocity() * (600.0 / FX_INTEGRATED_SENSOR_UNITS_PER_ROTATION);        
        double wheelRpm = motorRpm / DRIVE_GEAR_RATIO;
        units::velocity::meters_per_second_t wheelMetersPerSec {(wheelRpm * WHEEL_CIRCUMFERENCE) / 60.0};
        m_SwerveModuleState.speed = wheelMetersPerSec;

        units::angle::degree_t moduleAngle {AbsolutePositionToDegrees(m_pAngleTalon->GetSelectedSensorPosition(), ANGLE_GEAR_RATIO)};
        m_SwerveModuleState.angle = moduleAngle;

        return m_SwerveModuleState;
    }
    */

    /*
    inline static double DegreesToAbsolutePosition(double degrees, const double gearRatio)
    {
        return degrees / (360.0 / (gearRatio * FX_INTEGRATED_SENSOR_UNITS_PER_ROTATION));
    }
    */

    /*
    inline static double AbsolutePositionToDegrees(double controllerUnits, const double gearRatio)
    {
        return controllerUnits * (360.0 / (gearRatio * FX_INTEGRATED_SENSOR_UNITS_PER_ROTATION));
    }
    */

    ModulePosition m_MotorGroupPosition;
    TalonFX * m_pDriveTalon;
    TalonFX * m_pAngleTalon;
    CANCoder * m_pAngleCanCoder;
    double m_AngleOffset;
    double m_LastAngle;

    SwerveModuleState m_SwerveModuleState;
    SimpleMotorFeedforward<units::meters> * m_pFeedForward;

    // Divide by 12 on these constants to convert from volts to percent output for CTRE
    using Distance = units::meters;
    using Velocity = units::compound_unit<Distance, units::inverse<units::seconds>>;
    using Acceleration = units::compound_unit<Velocity, units::inverse<units::seconds>>;
    using kv_unit = units::compound_unit<units::volts, units::inverse<Velocity>>;
    using ka_unit = units::compound_unit<units::volts, units::inverse<Acceleration>>;
    static constexpr units::volt_t KS = (0.667_V / 12.0);
    static constexpr units::unit_t<kv_unit> KV = units::unit_t<kv_unit>(2.44 / 12.0);
    static constexpr units::unit_t<ka_unit> KA = units::unit_t<ka_unit>(0.27 / 12.0);

    static constexpr double DRIVE_GEAR_RATIO = (6.12 / 1.0);                        // L3 Very Fast configuration is 6.12:1
    static constexpr double ANGLE_GEAR_RATIO = (12.8 / 1.0);
    static constexpr double FX_INTEGRATED_SENSOR_UNITS_PER_ROTATION = 2048.0;
    static constexpr double WHEEL_CIRCUMFERENCE = 4.0 * 0.0254 * M_PI;              // 1 inch = 0.0254 meters

    // Swerve Profiling Values
    static constexpr double MAX_SWERVE_VELOCITY = 4.5;
    static constexpr double MAX_ANGULAR_VELOCITY = 11.5;
    static constexpr double OPEN_LOOP_RAMP = 0.25;
    static constexpr double CLOSED_LOOP_RAMP = 0.0;

    SwerveModule(const SwerveModule &) = delete;
    SwerveModule & operator=(const SwerveModule &) = delete;
};

#endif // SWERVEMODULE_HPP
