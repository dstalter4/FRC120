////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveModule.cpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve module on a swerve drive robot.
///
/// Copyright (c) 2024 Youth Technology Academy
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
    m_pAngleCanCoder(new CANcoder(config.m_CanCoderId, "canivore-120")),
    m_pFeedForward(new SimpleMotorFeedforward<units::meters>(KS, KV, KA)),
    m_DriveDutyCycleOut(0.0),
    m_DriveVelocityVoltage(0.0_tps),
    m_AnglePositionVoltage(0.0_tr),
    CANCODER_STARTING_ABSOLUTE_OFFSET(config.m_CancoderStartingAbsoluteOffset)
{
    // Build the strings to use in the display method
    std::snprintf(&m_DisplayStrings.m_CancoderAngleString[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "cancoder");
    std::snprintf(&m_DisplayStrings.m_FxEncoderAngleString[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "FX encoder");
    std::snprintf(&m_DisplayStrings.m_DriveTalonTemp[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "drive temp (F)");
    std::snprintf(&m_DisplayStrings.m_AngleTalonTemp[0], DisplayStrings::MAX_MODULE_DISPLAY_STRING_LENGTH, "%s %s", config.m_pModuleName, "angle temp (F)");

    // Configure drive motor controller
    // Current limiting values: enable, limit, threshold, duration
    /*
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
    */

    TalonFXConfiguration driveTalonConfig;
    driveTalonConfig.MotorOutput.Inverted = InvertedValue::CounterClockwise_Positive;
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;
    driveTalonConfig.Feedback.SensorToMechanismRatio = SwerveConfig::DRIVE_GEAR_RATIO;

    driveTalonConfig.CurrentLimits.SupplyCurrentLimit = 35.0;
    driveTalonConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
    driveTalonConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    driveTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveTalonConfig.Slot0.kP = 0.1;
    driveTalonConfig.Slot0.kI = 0.0;
    driveTalonConfig.Slot0.kD = 0.0;

    driveTalonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    driveTalonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

    driveTalonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
    driveTalonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    (void)m_pDriveTalon->GetConfigurator().Apply(driveTalonConfig);
    (void)m_pDriveTalon->GetConfigurator().SetPosition(0.0_tr);


    // @todo: Should the talons change default group status rates to preserve CAN bandwidth?
    //m_pDriveTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 100);
    //m_pDriveTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 100);

    // Configure angle motor controller
    // Current limiting values: enable, limit, threshold, duration
    /*
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
    */

    TalonFXConfiguration angleTalonConfig;
    angleTalonConfig.MotorOutput.Inverted = InvertedValue::Clockwise_Positive;
    angleTalonConfig.MotorOutput.NeutralMode = NeutralModeValue::Coast;
    angleTalonConfig.Feedback.SensorToMechanismRatio = SwerveConfig::ANGLE_GEAR_RATIO;
    angleTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;

    angleTalonConfig.CurrentLimits.SupplyCurrentLimit = 25.0;
    angleTalonConfig.CurrentLimits.SupplyCurrentThreshold = 40.0;
    angleTalonConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    angleTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // @todo_phoenix6: Tune these, they only crudely work.
    angleTalonConfig.Slot0.kP = 50.0;
    angleTalonConfig.Slot0.kI = 0.0;
    angleTalonConfig.Slot0.kD = 0.5;

    (void)m_pAngleTalon->GetConfigurator().Apply(angleTalonConfig);

    // Configure CANCoder
    /*
    CANCoderConfiguration canCoderConfig;
    canCoderConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    canCoderConfig.sensorDirection = false;
    canCoderConfig.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    canCoderConfig.sensorTimeBase = SensorTimeBase::PerSecond;
    m_pAngleCanCoder->ConfigFactoryDefault();
    m_pAngleCanCoder->ConfigAllSettings(canCoderConfig);
    */

    CANcoderConfiguration canCoderConfig;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
    (void)m_pAngleCanCoder->GetConfigurator().Apply(canCoderConfig);

    // Reset the swerve module to the absolute angle starting position.
    // This reads the current angle from the CANCoder and figures out how
    // far the module is from the config passed in (the predetermined
    // position from manual measurement/calibration).
    // SetSelectedSensorPosition() is causing excessive spins when downloading new code without a power cycle
/*
    double fxEncoderAbsPosition = m_pAngleTalon->GetPosition();
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
*/

/*
    units::angle::turn_t canCoderAbsPositionInRotations = m_pAngleCanCoder->GetAbsolutePosition().GetValue();
    units::angle::degree_t canCoderAbsPositionInDegrees = canCoderAbsPositionInRotations;
    units::angle::degree_t degreesToMove = canCoderAbsPositionInDegrees - m_AngleOffset.Degrees();
*/

    // Bevel R, Bevel L
    // FL: 0.440186, -0.065186
    // FR: 0.092773, -0.396484
    // BL: 0.337646, -0.167725
    // BR: -0.378418, 0.114990

    //const units::angle::turn_t TARGET_CANCODER_POS1 = 0.440186_tr;
    //const units::angle::turn_t TARGET_CANCODER_POS2 = 0.092773_tr;
    //const units::angle::turn_t TARGET_CANCODER_POS3 = 0.337646_tr;
    //const units::angle::turn_t TARGET_CANCODER_POS4 = -0.378418_tr;

    // @todo_phoenix6: This should use CANCODER_STARTING_ABSOLUTE_OFFSET.
    units::angle::turn_t TARGET_CANCODER_POS = 0.0_tr;
    switch (m_MotorGroupPosition)
    {
        case 0:
            TARGET_CANCODER_POS = 0.440186_tr;
            break;
        case 1:
            TARGET_CANCODER_POS = 0.092773_tr;
            break;
        case 2:
            TARGET_CANCODER_POS = 0.337646_tr;
            break;
        case 3:
            TARGET_CANCODER_POS = -0.378418_tr;
            break;
        default:
            break;
    }

    // @todo_phoenix6: Formalize this code.  Figure out the math.
    static units::angle::turn_t initialTurn = 0.0_tr;
    static units::angle::turn_t deltaTurns = 0.0_tr;
    initialTurn = m_pAngleCanCoder->GetAbsolutePosition().GetValue();
    // Delta might end up > abs(.5), but still aligns with the correct position
    deltaTurns = initialTurn - TARGET_CANCODER_POS;
    m_pAngleTalon->SetPosition(-deltaTurns);
    std::printf("mod %d delta: %f\n", m_MotorGroupPosition, (-deltaTurns).value());
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
/*
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
*/


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
    // 2024: No more custom command
    // @todo_phoenix6: This negative sign is critical for some reason.  Figure out why.
    desiredState = SwerveModuleState::Optimize(desiredState, -GetSwerveModuleState().angle);

    // Update the drive motor controller
    if (bIsOpenLoop)
    {
        //double percentOutput = desiredState.speed / SwerveConfig::MAX_DRIVE_VELOCITY_MPS;
        //m_pDriveTalon->Set(ControlMode::PercentOutput, percentOutput);
        m_DriveDutyCycleOut.Output = desiredState.speed / SwerveConfig::MAX_DRIVE_VELOCITY_MPS;
        (void)m_pDriveTalon->SetControl(m_DriveDutyCycleOut);
    }
    else
    {
        //double velocity = SwerveConversions::MpsToFalcon((desiredState.speed).value(), SwerveConfig::WHEEL_CIRCUMFERENCE, SwerveConfig::DRIVE_GEAR_RATIO);
        //m_pDriveTalon->Set(ControlMode::Velocity, velocity, DemandType::DemandType_ArbitraryFeedForward, m_pFeedForward->Calculate(desiredState.speed).value());
        units::angular_velocity::turns_per_second_t driveTalonDesiredVelocityTps = units::angular_velocity::turns_per_second_t(SwerveConversions::MpsToRps(desiredState.speed.value(), SwerveConfig::WHEEL_CIRCUMFERENCE));
        m_DriveVelocityVoltage.Velocity = driveTalonDesiredVelocityTps;
        m_DriveVelocityVoltage.FeedForward = m_pFeedForward->Calculate(desiredState.speed);
        (void)m_pDriveTalon->SetControl(m_DriveVelocityVoltage);
    }

    // Update the angle motor controller
    // Prevent rotating module if speed is less then 1% (prevents jitter).
    // (If the wheels are moving too slow, don't turn them.)
    /*
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
    */
    // @todo_phoenix6: This negative sign is critical for some reason.  Figure out why.
    units::angle::turn_t targetAngle = -desiredState.angle.Degrees();

    (void)m_pAngleTalon->SetControl(m_AnglePositionVoltage.WithPosition(targetAngle));
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
    //units::velocity::meters_per_second_t velocity(SwerveConversions::FalconToMps(m_pDriveTalon->GetSelectedSensorVelocity(), SwerveConfig::WHEEL_CIRCUMFERENCE, SwerveConfig::DRIVE_GEAR_RATIO));
    units::angular_velocity::turns_per_second_t driveTalonVelocityRps = m_pDriveTalon->GetVelocity().GetValue();
    units::velocity::meters_per_second_t velocity(SwerveConversions::RpsToMps(driveTalonVelocityRps.value(), SwerveConfig::WHEEL_CIRCUMFERENCE));

    // Get the current angle
    //units::angle::degree_t angle(SwerveConversions::FalconToDegrees(m_pAngleTalon->GetSelectedSensorPosition(), SwerveConfig::ANGLE_GEAR_RATIO));
    units::angle::turn_t angleTalonPositionInTurns = m_pAngleTalon->GetPosition().GetValue();
    units::angle::degree_t angle = angleTalonPositionInTurns;

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
    //units::meter_t distance(SwerveConversions::FalconToMeters(m_pDriveTalon->GetSelectedSensorPosition(), SwerveConfig::WHEEL_CIRCUMFERENCE, SwerveConfig::DRIVE_GEAR_RATIO));
    units::angle::turn_t driveTalonPositionInTurns = m_pDriveTalon->GetPosition().GetValue();
    units::meter_t distance(SwerveConversions::RotationsToMeters(driveTalonPositionInTurns.value(), SwerveConfig::WHEEL_CIRCUMFERENCE));

    // Get the current angle
    //units::angle::degree_t angle(SwerveConversions::FalconToDegrees(m_pAngleTalon->GetSelectedSensorPosition(), SwerveConfig::ANGLE_GEAR_RATIO));
    units::angle::turn_t angleTalonPositionInTurns = m_pAngleTalon->GetPosition().GetValue();
    units::angle::degree_t angle = angleTalonPositionInTurns;

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
    SmartDashboard::PutNumber(m_DisplayStrings.m_CancoderAngleString, m_pAngleCanCoder->GetAbsolutePosition().GetValueAsDouble());
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
            SmartDashboard::PutNumber(m_DisplayStrings.m_DriveTalonTemp, RobotUtils::ConvertCelsiusToFahrenheit(m_pDriveTalon->GetDeviceTemp().GetValueAsDouble()));
            SmartDashboard::PutNumber(m_DisplayStrings.m_AngleTalonTemp, RobotUtils::ConvertCelsiusToFahrenheit(m_pAngleTalon->GetDeviceTemp().GetValueAsDouble()));

            m_DetailedModuleDisplayIndex++;
            if (m_DetailedModuleDisplayIndex == SwerveConfig::NUM_SWERVE_DRIVE_MODULES)
            {
                m_DetailedModuleDisplayIndex = 0U;
            }
        }
        lastUpdateTime = currentTime;
    }
}
