#ifndef LINEARSERVO_HPP
#define LINEARSERVO_HPP

#include <frc/Servo.h>
#include <frc/Timer.h>
#include <units/length.h>
#include <units/velocity.h>

using namespace frc;

/**
 * Class for controlling a WCP L16-R Miniature Linear Servo Actuator
 */
class LinearServo : public Servo
{
public:
    using millimeters_per_second_t = units::compound_unit<units::millimeters, units::inverse<units::seconds>>;

    enum StrokeOption
    {
        STROKE_OPTION_50_MM,
        STROKE_OPTION_100_MM,
        STROKE_OPTION_140_MM
    };

    enum GearOption
    {
        GEAR_OPTION_35_TO_1,
        GEAR_OPTION_63_TO_1,
        GEAR_OPTION_150_TO_1,
    };


    /**
    * Creates a Linear Servo Actuator object.  m_SetPositionMm is initialized to a negative
    * length so that the first time SetPosition() is called a change will occur.
    *
    * @param channel PWM channel used to control the servo
    * @param strokeOption the stroke length option (used to set the physical length in [mm])
    * @param gearOption the gear option (used to set the max speed of the servo [mm/second])
    */
    LinearServo(int channel, StrokeOption strokeOption, GearOption gearOption) : Servo(channel),
        m_ActuatorLengthMm(GetStrokeLength(strokeOption)),
        m_ServoMaxSpeedMmPerSecond(GetMaxSpeedMmPerSecond(gearOption)),
        m_SetPositionMm(-1.0_mm),
        m_CurrentPositionMm(0.0_mm),
        m_LastTimeSeconds(0.0_s)
    {
        // From https://docs.wcproducts.com/welcome/electronics/miniature-linear-servo-actuators/overview-and-features/general-specs.
        // Parameters are max, deadbandMax, center, deadbandMin, min (units to SetBounds() are microseconds).
        SetBounds(2000.0_us, 1800.0_us, 1500.0_us, 1200.0_us, 1000.0_us);
    }


    /**
    * Call this method to set the position of the servo in millimeters of its length.
    *
    * @param setPoint the target position of the servo [mm]
    */
    void SetPosition(units::length::millimeter_t setPoint)
    {
        // std::clamp restricts setPoint between 0 and max length in mm.
        setPoint = units::length::millimeter_t(std::clamp(setPoint.value(), 0.0, m_ActuatorLengthMm.value()));

        // Only update if there has been a change, to possibly prevent resetting m_LastTimeSeconds incorrectly
        if (setPoint != m_SetPositionMm)
        {
            m_SetPositionMm = setPoint;
            double setPosition = m_SetPositionMm / m_ActuatorLengthMm;
            PWM::SetPosition(setPosition);
            m_LastTimeSeconds = Timer::GetFPGATimestamp();

            // This is an alternate approach that uses SetSpeed() instead of SetPosition().
            // m_SetPositionMm / m_ActuatorLengthMm computes the percentage of the
            // length for the set point (which results in a value between 0.0 and 1.0).
            // The *2.0 results in a value between 0.0 and 2.0.
            // The -1.0 results in a value between -1.0 and +1.0, which is the range for SetSpeed().
            //double setSpeed = ((m_SetPositionMm / m_ActuatorLengthMm) * 2.0) - 1.0;
            //PWM::SetSpeed(((m_SetPositionMm / m_ActuatorLengthMm) * 2.0) - 1.0);
        }
    }


    /**
    * Run this method in any periodic function to update the position estimation of the servo.
    */
    void UpdateCurrentPosition()
    {
        // Compute the time delta
        units::time::second_t currentTimeS = Timer::GetFPGATimestamp();
        units::time::second_t timeDeltaS =  currentTimeS - m_LastTimeSeconds;
        m_LastTimeSeconds = currentTimeS;
        units::length::millimeter_t traveledMm = m_ServoMaxSpeedMmPerSecond * timeDeltaS;

        // d = r*t (distance = rate * time)
        if (m_CurrentPositionMm > (m_SetPositionMm + traveledMm))
        {
            // Position is too far, move backward
            m_CurrentPositionMm -= traveledMm;
        }
        else if (m_CurrentPositionMm < (m_SetPositionMm - traveledMm))
        {
            // Position is too short, move forward
            m_CurrentPositionMm += traveledMm;
        }
        else
        {
            m_CurrentPositionMm = m_SetPositionMm;
        }
    }


    /**
    * Gets the current position of the servo.
    * Must be calling UpdateCurrentPosition() periodically for accurate values.
    *
    * @return Servo Position [mm]
    */
    units::length::millimeter_t GetCurrentPosition()
    {
        return m_CurrentPositionMm;
    }


    /**
    * Checks if the servo is at its target position.
    * Must be calling UpdateCurrentPosition() periodically for accurate values.
    * Note: The max speed reported in the WCP documentation is not observed in
    *       empirical measurements.  As a result, this function returns 'true'
    *       before the position is actually reached.  This effect is more
    *       noticeable as the distance between the start and end points grows.
    * 
    * @return true when servo is at its target
    */
    bool IsAtTargetPosition()
    {
        return (m_CurrentPositionMm == m_SetPositionMm);
    }


private:
    constexpr const units::unit_t<millimeters_per_second_t> GetMaxSpeedMmPerSecond(GearOption gearOption)
    {
        units::unit_t<millimeters_per_second_t> speedMmPerSecond = units::unit_t<millimeters_per_second_t>(0.0);
        switch (gearOption)
        {
            case GEAR_OPTION_35_TO_1:
            {
                speedMmPerSecond = (32.0_mm / 1.0_s);
                break;
            }
            case GEAR_OPTION_63_TO_1:
            {
                speedMmPerSecond = (20.0_mm / 1.0_s);
                break;
            }
            case GEAR_OPTION_150_TO_1:
            {
                speedMmPerSecond = (8.0_mm / 1.0_s);
                break;
            }
            default:
            {
                break;
            }
        }
        return speedMmPerSecond;
    }

    constexpr const units::length::millimeter_t GetStrokeLength(StrokeOption strokeOption)
    {
        units::length::millimeter_t lengthMm = 0.0_mm;
        switch (strokeOption)
        {
            case STROKE_OPTION_50_MM:
            {
                lengthMm = 50.0_mm;
                break;
            }
            case STROKE_OPTION_100_MM:
            {
                lengthMm = 100.0_mm;
                break;
            }
            case STROKE_OPTION_140_MM:
            {
                lengthMm = 140.0_mm;
                break;
            }
            default:
            {
                break;
            }
        }
        return lengthMm;
    }

    const units::length::millimeter_t m_ActuatorLengthMm;
    const units::unit_t<millimeters_per_second_t> m_ServoMaxSpeedMmPerSecond;
    units::length::millimeter_t m_SetPositionMm;
    units::length::millimeter_t m_CurrentPositionMm;
    units::time::second_t m_LastTimeSeconds;
};

#endif  // LINEARSERVO_HPP