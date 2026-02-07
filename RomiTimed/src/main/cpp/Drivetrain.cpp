////////////////////////////////////////////////////////////////////////////////
/// @file   Drivetrain.hpp
/// @author David Stalter
///
/// @details
/// Class implementation for a drivetrain for a Romi robot.
///
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#include <numbers>
#include "Drivetrain.hpp"


////////////////////////////////////////////////////////////////
// @method Drivetrain::Drivetrain
//
// Constructor
////////////////////////////////////////////////////////////////
Drivetrain::Drivetrain()
{
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_RightMotor.SetInverted(true);

    m_LeftEncoder.SetDistancePerPulse(std::numbers::pi * WHEEL_DIAMETER.value() / COUNTS_PER_REVOLUTION);
    m_RightEncoder.SetDistancePerPulse(std::numbers::pi * WHEEL_DIAMETER.value() / COUNTS_PER_REVOLUTION);
    ResetEncoders();
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::Drivetrain
//
// Drives the robot via arcade controls.
////////////////////////////////////////////////////////////////
void Drivetrain::ArcadeDrive(double xAxisSpeed, double zAxisRotate)
{
    m_DifferentialDrive.ArcadeDrive(xAxisSpeed, zAxisRotate);
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::ResetEncoders
//
// Resets the encoders.
////////////////////////////////////////////////////////////////
void Drivetrain::ResetEncoders()
{
    m_LeftEncoder.Reset();
    m_RightEncoder.Reset();
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::GetLeftEncoderCount
//
// Returns the left wheel encoder count.
////////////////////////////////////////////////////////////////
int Drivetrain::GetLeftEncoderCount()
{
    return m_LeftEncoder.Get();
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::GetRightEncoderCount
//
// Returns the right wheel encoder count.
////////////////////////////////////////////////////////////////
int Drivetrain::GetRightEncoderCount()
{
    return m_RightEncoder.Get();
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::GetLeftDistance
//
// Returns the distance in meters traveled by the left wheel.
////////////////////////////////////////////////////////////////
units::meter_t Drivetrain::GetLeftDistance()
{
    return units::meter_t{m_LeftEncoder.GetDistance()};
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::GetRightDistance
//
// Returns the distance in meters traveled by the right wheel.
////////////////////////////////////////////////////////////////
units::meter_t Drivetrain::GetRightDistance()
{
    return units::meter_t{m_RightEncoder.GetDistance()};
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::GetAverageDistance
//
// Returns the average distance in meters traveled by both wheels.
////////////////////////////////////////////////////////////////
units::meter_t Drivetrain::GetAverageDistance()
{
    return (GetLeftDistance() + GetRightDistance()) / 2.0;
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::GetAccelX
//
// Returns the accelerometer x-axis value.
////////////////////////////////////////////////////////////////
units::meters_per_second_squared_t Drivetrain::GetAccelX()
{
    return units::meters_per_second_squared_t{m_Accelerometer.GetX()};
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::GetAccelY
//
// Returns the accelerometer y-axis value.
////////////////////////////////////////////////////////////////
units::meters_per_second_squared_t Drivetrain::GetAccelY()
{
    return units::meters_per_second_squared_t{m_Accelerometer.GetY()};
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::GetAccelZ
//
// Returns the accelerometer z-axis value.
////////////////////////////////////////////////////////////////
units::meters_per_second_squared_t Drivetrain::GetAccelZ()
{
    return units::meters_per_second_squared_t{m_Accelerometer.GetZ()};
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::GetGyroAngleX
//
// Returns the gyro x-axis angle.
////////////////////////////////////////////////////////////////
units::radian_t Drivetrain::GetGyroAngleX()
{
  return m_Gyro.GetAngleX();
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::GetGyroAngleY
//
// Returns the gyro y-axis angle.
////////////////////////////////////////////////////////////////
units::radian_t Drivetrain::GetGyroAngleY()
{
    return m_Gyro.GetAngleY();
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::GetGyroAngleZ
//
// Returns the gyro z-axis angle.
////////////////////////////////////////////////////////////////
units::radian_t Drivetrain::GetGyroAngleZ()
{
    return m_Gyro.GetAngleZ();
}


////////////////////////////////////////////////////////////////
// @method Drivetrain::ResetGyro
//
// Resets the gyro.
////////////////////////////////////////////////////////////////
void Drivetrain::ResetGyro()
{
    m_Gyro.Reset();
}
