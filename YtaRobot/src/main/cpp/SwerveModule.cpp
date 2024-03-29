////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveModule.cpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve module on a swerve drive robot.
///
/// Copyright (c) 2023 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/Timer.h"                              // for timers
#include "frc/smartdashboard/SmartDashboard.h"      // for interacting with the smart dashboard
#include "units/length.h"                           // for units::meters

// C++ INCLUDES
#include "RobotUtils.hpp"                           // for ConvertCelsiusToFahrenheit
#include "SwerveConfig.hpp"                         // for swerve configuration and constants
#include "SwerveConversions.hpp"                    // for conversion functions
#include "SwerveModule.hpp"                         // for class declaration

using namespace frc;

uint32_t SwerveModule::m_DetailedModuleDisplayIndex = 0U;


////////////////////////////////////////////////////////////////
/// @method SwerveModule::SwerveModule
///
/// Constructs a swerve module object.  This will configure the
/// settings for each TalonFX (PID values, current limiting,
/// etc.) and the CANCoder.  It also builds the display strings
/// sent to the dashboard.  Note that the CANCoders are placed
/// on the CANivore bus, which requires a 120 ohm terminating
/// resistor.
///
/// 2023: Bevels facing right is 1.0 forward on the Talons.
///
////////////////////////////////////////////////////////////////
SwerveModule::SwerveModule(SwerveModuleConfig config) :
    m_MotorGroupPosition(config.m_Position),
    m_pDriveTalon(new TalonFX(config.m_DriveMotorCanId)),
    m_pAngleTalon(new TalonFX(config.m_AngleMotorCanId)),
    m_pAngleCanCoder(new CANCoder(config.m_CanCoderId, "canivore-120")),
    m_AngleOffset(config.m_AngleOffset),
    m_LastAngle(),
    m_pFeedForward(new SimpleMotorFeedforward<units::meters>(KS, KV, KA))
{
    // Build the strings to use in the display method
    std::snprintf(&m_DisplayStrings.m_CancoderAngleString[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "cancoder");
    std::snprintf(&m_DisplayStrings.m_FxEncoderAngleString[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "FX encoder");
    std::snprintf(&m_DisplayStrings.m_DriveTalonTemp[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "drive temp (F)");
    std::snprintf(&m_DisplayStrings.m_AngleTalonTemp[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "angle temp (F)");

    // Configure drive motor controller
    // Current limiting values: enable, limit, threshold, duration
    SupplyCurrentLimitConfiguration driveTalonSupplyLimit = {true, 35, 60, 0.1};
    TalonFXConfiguration driveTalonConfig;
    driveTalonConfig.slot0.kP = 0.1;
    driveTalonConfig.slot0.kI = 0.0;
    driveTalonConfig.slot0.kD = 0.0;
    driveTalonConfig.slot0.kF = 0.0;        
    driveTalonConfig.supplyCurrLimit = driveTalonSupplyLimit;
    driveTalonConfig.openloopRamp = OPEN_LOOP_RAMP;
    driveTalonConfig.closedloopRamp = CLOSED_LOOP_RAMP;
    m_pDriveTalon->ConfigFactoryDefault();
    m_pDriveTalon->ConfigAllSettings(driveTalonConfig);
    m_pDriveTalon->SetInverted(false);
    m_pDriveTalon->SetNeutralMode(NeutralMode::Brake);
    m_pDriveTalon->SetSelectedSensorPosition(0);

    // @todo: Should the talons change default group status rates to preserve CAN bandwidth?
    //m_pDriveTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 100);
    //m_pDriveTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 100);

    // Configure angle motor controller
    // Current limiting values: enable, limit, threshold, duration
    SupplyCurrentLimitConfiguration angleTalonSupplyLimit = {true, 25, 40, 0.1};
    TalonFXConfiguration angleTalonConfig;
    angleTalonConfig.slot0.kP = 0.5;
    angleTalonConfig.slot0.kI = 0.0;
    angleTalonConfig.slot0.kD = 0.0;
    angleTalonConfig.slot0.kF = 0.0;
    angleTalonConfig.supplyCurrLimit = angleTalonSupplyLimit;
    m_pAngleTalon->ConfigFactoryDefault();
    m_pAngleTalon->ConfigAllSettings(angleTalonConfig);
    m_pAngleTalon->SetInverted(false);
    m_pAngleTalon->SetNeutralMode(NeutralMode::Coast);

    // Configure CANCoder
    CANCoderConfiguration canCoderConfig;
    canCoderConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    canCoderConfig.sensorDirection = false;
    canCoderConfig.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    canCoderConfig.sensorTimeBase = SensorTimeBase::PerSecond;
    m_pAngleCanCoder->ConfigFactoryDefault();
    m_pAngleCanCoder->ConfigAllSettings(canCoderConfig);

    // Reset the swerve module to the absolute angle starting position.
    // This reads the current angle from the CANCoder and figures out how
    // far the module is from the config passed in (the predetermined
    // position from manual measurement/calibration).
    // SetSelectedSensorPosition() is causing excessive spins when downloading new code without a power cycle
    double fxEncoderAbsPosition = m_pAngleTalon->GetSelectedSensorPosition();
    double fxEncoderTarget = SwerveConversions::DegreesToFalcon(m_pAngleCanCoder->GetAbsolutePosition() - m_AngleOffset.Degrees().value(), SwerveConfig::ANGLE_GEAR_RATIO);
    const double FX_ENCODER_UNITS_PER_360_DEGREES = SwerveConversions::DegreesToFalcon(360, SwerveConfig::ANGLE_GEAR_RATIO);

    // Adjust the TalonFX target position to something close to its current position
    while (fxEncoderTarget < fxEncoderAbsPosition)
    {
        fxEncoderTarget += FX_ENCODER_UNITS_PER_360_DEGREES;
    }
    while (fxEncoderTarget > fxEncoderAbsPosition)
    {
        fxEncoderTarget -= FX_ENCODER_UNITS_PER_360_DEGREES;
    }

    // Set the angle TalonFX built-in encoder position
    m_pAngleTalon->SetSelectedSensorPosition(fxEncoderTarget);

    // Save off the initial angle
    m_LastAngle = units::degree_t(SwerveConversions::FalconToDegrees(m_pAngleTalon->GetSelectedSensorPosition(), SwerveConfig::ANGLE_GEAR_RATIO));
}


////////////////////////////////////////////////////////////////
/// @method SwerveModule::Optimize
///
/// Optimizes a swerve module state for use with setting a
/// desired state.  This finds the shortest way to move to a
/// target angle to prevent motion over 180 degrees (reversing
/// the target speed, if necessary).
///
////////////////////////////////////////////////////////////////
SwerveModuleState SwerveModule::Optimize(SwerveModuleState desiredState, Rotation2d currentAngle)
{
    // This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not
    double targetAngle = SwerveConversions::AdjustAngleScope(currentAngle.Degrees().value(), desiredState.angle.Degrees().value());
    double targetSpeed = desiredState.speed.value();
    double delta = targetAngle - currentAngle.Degrees().value();

    if (std::abs(delta) > 90)
    {
        targetSpeed = -targetSpeed;
        if (delta > 90)
        {
            targetAngle -= 180;
        }
        else
        {
            targetAngle += 180;
        }
    }

    return {units::velocity::meters_per_second_t(targetSpeed), units::angle::degree_t(targetAngle)};
}


////////////////////////////////////////////////////////////////
/// @method SwerveModule::SetDesiredState
///
/// Sets a swerve module to the input state.  It computes the
/// target velocity and angle and updates the motor controllers
/// as appropriate.
///
////////////////////////////////////////////////////////////////
void SwerveModule::SetDesiredState(SwerveModuleState desiredState, bool bIsOpenLoop)
{
    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
    desiredState = Optimize(desiredState, GetSwerveModuleState().angle);

    // Update the drive motor controller
    if (bIsOpenLoop)
    {
        double percentOutput = desiredState.speed / SwerveConfig::MAX_DRIVE_VELOCITY_MPS;
        m_pDriveTalon->Set(ControlMode::PercentOutput, percentOutput);
    }
    else
    {
        double velocity = SwerveConversions::MpsToFalcon((desiredState.speed).value(), SwerveConfig::WHEEL_CIRCUMFERENCE, SwerveConfig::DRIVE_GEAR_RATIO);
        m_pDriveTalon->Set(ControlMode::Velocity, velocity, DemandType::DemandType_ArbitraryFeedForward, m_pFeedForward->Calculate(desiredState.speed).value());
    }

    // Update the angle motor controller
    // Prevent rotating module if speed is less then 1% (prevents jitter).
    // (If the wheels are moving too slow, don't turn them.)
    Rotation2d angle = 0.0_deg;
    if (std::abs(desiredState.speed.value()) <= (SwerveConfig::MAX_ANGULAR_VELOCITY_RAD_PER_SEC.value() * 0.01))
    {
        angle = m_LastAngle;
    }
    else
    {
        angle = desiredState.angle;
    }
    m_pAngleTalon->Set(ControlMode::Position, SwerveConversions::DegreesToFalcon(angle.Degrees().value(), SwerveConfig::ANGLE_GEAR_RATIO));

    // Save off the updated last angle
    m_LastAngle = angle;
}


////////////////////////////////////////////////////////////////
/// @method SwerveModule::GetSwerveModuleState
///
/// Returns a swerve module state based on information from the
/// motor controllers and sensors.
///
////////////////////////////////////////////////////////////////
SwerveModuleState SwerveModule::GetSwerveModuleState()
{
    // Get the current velocity
    units::velocity::meters_per_second_t velocity(SwerveConversions::FalconToMps(m_pDriveTalon->GetSelectedSensorVelocity(), SwerveConfig::WHEEL_CIRCUMFERENCE, SwerveConfig::DRIVE_GEAR_RATIO));

    // Get the current angle
    units::angle::degree_t angle(SwerveConversions::FalconToDegrees(m_pAngleTalon->GetSelectedSensorPosition(), SwerveConfig::ANGLE_GEAR_RATIO));

    return {velocity, angle};
}


////////////////////////////////////////////////////////////////
/// @method SwerveModule::GetSwerveModulePosition
///
/// Returns a swerve module position based on information from
/// the motor controllers and sensors.
///
////////////////////////////////////////////////////////////////
SwerveModulePosition SwerveModule::GetSwerveModulePosition()
{
    // Get the current distance
    units::meter_t distance(SwerveConversions::FalconToMeters(m_pDriveTalon->GetSelectedSensorPosition(), SwerveConfig::WHEEL_CIRCUMFERENCE, SwerveConfig::DRIVE_GEAR_RATIO));

    // Get the current angle
    units::angle::degree_t angle(SwerveConversions::FalconToDegrees(m_pAngleTalon->GetSelectedSensorPosition(), SwerveConfig::ANGLE_GEAR_RATIO));

    return {distance, angle};
}


////////////////////////////////////////////////////////////////
/// @method SwerveModule::UpdateSmartDashboard
///
/// Support routine to put useful information on the dashboard.
///
////////////////////////////////////////////////////////////////
void SwerveModule::UpdateSmartDashboard()
{
    // Print the encoder values every time
    SmartDashboard::PutNumber(m_DisplayStrings.m_CancoderAngleString, m_pAngleCanCoder->GetAbsolutePosition());
    SmartDashboard::PutNumber(m_DisplayStrings.m_FxEncoderAngleString, GetSwerveModulePosition().angle.Degrees().value());

    // Create and start a timer the first time through
    static Timer * pTimer = new Timer();
    static bool bTimerStarted = false;
    if (!bTimerStarted)
    {
        pTimer->Start();
        bTimerStarted = true;
    }
    static units::second_t lastUpdateTime = 0_s;
    units::second_t currentTime = pTimer->Get();

    // If it's time for a detailed update, print more info
    const units::second_t DETAILED_DISPLAY_TIME_S = 0.5_s;
    if ((currentTime - lastUpdateTime) > DETAILED_DISPLAY_TIME_S)
    {
        // Even at the slower update rate, only do one swerve module at a time
        if (m_DetailedModuleDisplayIndex == static_cast<uint32_t>(m_MotorGroupPosition))
        {
            SmartDashboard::PutNumber(m_DisplayStrings.m_DriveTalonTemp, RobotUtils::ConvertCelsiusToFahrenheit(m_pDriveTalon->GetTemperature()));
            SmartDashboard::PutNumber(m_DisplayStrings.m_AngleTalonTemp, RobotUtils::ConvertCelsiusToFahrenheit(m_pAngleTalon->GetTemperature()));

            m_DetailedModuleDisplayIndex++;
            if (m_DetailedModuleDisplayIndex == SwerveConfig::NUM_SWERVE_DRIVE_MODULES)
            {
                m_DetailedModuleDisplayIndex = 0U;
            }
        }
        lastUpdateTime = currentTime;
    }
}
