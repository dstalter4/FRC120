////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveModule.cpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve module on a swerve drive robot.
///
/// Copyright (c) 2022 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "SwerveModule.hpp"                 // for class declaration
#include "SwerveConversions.hpp"            // for conversion functions

using namespace frc;


SwerveModule::SwerveModule(SwerveModuleConfig config) :
    m_MotorGroupPosition(config.m_Position),
    m_pDriveTalon(new TalonFX(config.m_DriveMotorCanId)),
    m_pAngleTalon(new TalonFX(config.m_AngleMotorCanId)),
    m_pAngleCanCoder(new CANCoder(config.m_CanCoderId, "canivore")),
    m_AngleOffset(config.m_AngleOffset),
    m_LastAngle(),
    m_SwerveModuleState(),
    m_pFeedForward(new SimpleMotorFeedforward<units::meters>(KS, KV, KA))
{
    // @todo_swerve: Tune these configurations.  Set drive falcon to FeedbackDevice.IntegratedSensor?

    // Configure drive motor controller (ref: configDriveMotor())
    // enable, limit, threshold, duration
    SupplyCurrentLimitConfiguration driveTalonSupplyLimit = {true, 35, 60, 0.1};
    TalonFXConfiguration driveTalonConfig;
    driveTalonConfig.slot0.kP = 0.10;
    driveTalonConfig.slot0.kI = 0.0;
    driveTalonConfig.slot0.kD = 0.0;
    driveTalonConfig.slot0.kF = 0.0;        
    driveTalonConfig.supplyCurrLimit = driveTalonSupplyLimit;
    driveTalonConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    driveTalonConfig.openloopRamp = OPEN_LOOP_RAMP;
    driveTalonConfig.closedloopRamp = CLOSED_LOOP_RAMP;
    m_pDriveTalon->ConfigFactoryDefault();
    m_pDriveTalon->ConfigAllSettings(driveTalonConfig);
    m_pDriveTalon->SetInverted(false);
    m_pDriveTalon->SetNeutralMode(NeutralMode::Brake);
    m_pDriveTalon->SetSelectedSensorPosition(0);

    // Configure angle motor controller (ref: configAngleMotor())
    // enable, limit, threshold, duration
    SupplyCurrentLimitConfiguration angleTalonSupplyLimit = {true, 25, 40, 0.1};
    TalonFXConfiguration angleTalonConfig;
    angleTalonConfig.slot0.kP = 0.6;
    angleTalonConfig.slot0.kI = 0.0;
    angleTalonConfig.slot0.kD = 12.0;
    angleTalonConfig.slot0.kF = 0.0;
    angleTalonConfig.supplyCurrLimit = angleTalonSupplyLimit;
    angleTalonConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    m_pAngleTalon->ConfigFactoryDefault();
    m_pAngleTalon->ConfigAllSettings(angleTalonConfig);
    m_pAngleTalon->SetInverted(false);
    m_pAngleTalon->SetNeutralMode(NeutralMode::Coast);

    // Configure CANCoder (ref: configAngleEncoder())
    CANCoderConfiguration canCoderConfig;
    canCoderConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    canCoderConfig.sensorDirection = false;
    canCoderConfig.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    canCoderConfig.sensorTimeBase = SensorTimeBase::PerSecond;
    m_pAngleCanCoder->ConfigFactoryDefault();
    m_pAngleCanCoder->ConfigAllSettings(canCoderConfig);

    // (ref: resetToAbsolute())
    // double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
    m_pAngleTalon->SetSelectedSensorPosition(SwerveConversions::degreesToFalcon(m_pAngleCanCoder->GetAbsolutePosition() - m_AngleOffset, ANGLE_GEAR_RATIO));
}

SwerveModuleState SwerveModule::Optimize(SwerveModuleState desiredState, Rotation2d currentAngle)
{
    double targetAngle = SwerveConversions::placeInAppropriate0To360Scope(currentAngle.Degrees().value(), desiredState.angle.Degrees().value());
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
    //return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    return {units::velocity::meters_per_second_t(targetSpeed), units::angle::degree_t(targetAngle)};
}

void SwerveModule::SetDesiredState(SwerveModuleState desiredState, bool bIsOpenLoop)
{
    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
    //desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    //desiredState = SwerveModuleState::Optimize(desiredState, GetSwerveModuleState().angle);
    desiredState = Optimize(desiredState, GetSwerveModuleState().angle);

    if (bIsOpenLoop)
    {
        //double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        double percentOutput = desiredState.speed.value() / MAX_SWERVE_VELOCITY;
        //mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        m_pDriveTalon->Set(ControlMode::PercentOutput, percentOutput);
    }
    else
    {
        //double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        double velocity = SwerveConversions::MPSToFalcon((desiredState.speed).value(), WHEEL_CIRCUMFERENCE, DRIVE_GEAR_RATIO);
        //mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        m_pDriveTalon->Set(ControlMode::Velocity, velocity, DemandType::DemandType_ArbitraryFeedForward, m_pFeedForward->Calculate(desiredState.speed).value());
    }

    // Prevent rotating module if speed is less then 1%.  Prevents jitter.
    //double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees();
    double angle = 0.0;
    if (std::abs(desiredState.speed.value()) <= (MAX_SWERVE_VELOCITY * 0.01))
    {
        angle = m_LastAngle;
    }
    else
    {
        angle = desiredState.angle.Degrees().value();
    }

    //mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio));
    m_pAngleTalon->Set(ControlMode::Position, SwerveConversions::degreesToFalcon(angle, ANGLE_GEAR_RATIO));
    //lastAngle = angle;
    m_LastAngle = angle;
}

SwerveModuleState SwerveModule::GetSwerveModuleState()
{
    //double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
    units::velocity::meters_per_second_t velocity(SwerveConversions::falconToMPS(m_pDriveTalon->GetSelectedSensorVelocity(), WHEEL_CIRCUMFERENCE, DRIVE_GEAR_RATIO));

    //Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    units::angle::degree_t angle(SwerveConversions::falconToDegrees(m_pAngleTalon->GetSelectedSensorPosition(), ANGLE_GEAR_RATIO));

    //return new SwerveModuleState(velocity, angle);
    //m_SwerveModuleState.speed = velocity;
    //m_SwerveModuleState.angle = angle;
    //return m_SwerveModuleState;
    return {velocity, angle};
}
