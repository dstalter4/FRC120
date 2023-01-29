////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveConversions.hpp
/// @author David Stalter
///
/// @details
/// Utility routines for swerve drive conversions.
///
/// Copyright (c) 2023 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef SWERVECONVERSIONS_HPP
#define SWERVECONVERSIONS_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
// (none)

using namespace frc;


////////////////////////////////////////////////////////////////
/// @namespace SwerveConversions
///
/// Routines for swerve drive conversions.
///
////////////////////////////////////////////////////////////////
namespace SwerveConversions
{
    inline static constexpr double ConvertCelsiusToFahrenheit(double degreesC) { return ((degreesC * 9.0/5.0) + 32.0); }

  /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    inline static double placeInAppropriate0To360Scope(double scopeReference, double newAngle)
    {
      double lowerBound;
      double upperBound;
      double lowerOffset = static_cast<int>(scopeReference) % 360;
      if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
      } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
          newAngle += 360;
      }
      while (newAngle > upperBound) {
          newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
          newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
      }
      return newAngle;
    }

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    inline static double CANcoderToDegrees(double positionCounts, double gearRatio)
    {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    inline static double degreesToCANcoder(double degrees, double gearRatio)
    {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param counts Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    inline static double falconToDegrees(double positionCounts, double gearRatio)
    {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts
     */
    inline static double degreesToFalcon(double degrees, double gearRatio)
    {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    inline static double falconToRPM(double velocityCounts, double gearRatio)
    {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    inline static double RPMToFalcon(double RPM, double gearRatio)
    {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Falcon Velocity Counts
     */
    inline static double falconToMPS(double velocitycounts, double circumference, double gearRatio)
    {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Falcon Velocity Counts
     */
    inline static double MPSToFalcon(double velocity, double circumference, double gearRatio)
    {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts Falcon Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    inline static double falconToMeters(double positionCounts, double circumference, double gearRatio)
    {
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Falcon Position Counts
     */
    inline static double MetersToFalcon(double meters, double circumference, double gearRatio)
    {
        return meters / (circumference / (gearRatio * 2048.0));
    }
}

#endif // SWERVECONVERSIONS_HPP
