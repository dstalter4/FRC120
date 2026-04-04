////////////////////////////////////////////////////////////////////////////////
/// @file   DifferentialDrive.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a differential drive robot base.  This file is
/// largely legacy due to the prevalance of swerve drive robots.
///
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef DIFFERENTIALDRIVE_HPP
#define DIFFERENTIALDRIVE_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "DriveConfiguration.hpp"                   // for drive configuration constants/values
#include "YtaTalon.hpp"                             // for motor controller object declarations


////////////////////////////////////////////////////////////////
/// @class DifferentialDrive
///
/// Declarations for a differential drive object.
///
////////////////////////////////////////////////////////////////
class DifferentialDrive
{
public:
    friend class YtaRobotTest;

    enum class RobotDirection
    {
        ROBOT_NO_DIRECTION,
        ROBOT_FORWARD,
        ROBOT_REVERSE,
        ROBOT_LEFT,
        ROBOT_RIGHT
    };

    struct DriveControlInputs
    {
        double m_xAxis;
        double m_yAxis;
        double m_xAxisSlow;
        double m_yAxisSlow;
        double m_Throttle;
        int m_PovValue;
        RobotDirection m_InchingDirection;
        bool m_bDriveSwap;
    };

    // Constructor
    DifferentialDrive(const std::function<const CANBus&(std::string_view)>& rGetCanBusReferenceLambda);

    // Main drive control
    void DriveSequence(std::function<const DriveControlInputs & ()> & rGetDriveControlInputsLambda);

    // Autonomous routines that can be called
    void AutonomousDrive(DifferentialDrive::RobotDirection direction, double speed, units::second_t time);
    void AutonomousBackDrive(DifferentialDrive::RobotDirection currentDirection);
    void AutonomousBackDriveTurn(DifferentialDrive::RobotDirection currentDirection);

private:
    typedef Yta::Talon::MotorGroupControlMode MotorGroupControlMode;

    // Specialized movement routines
    bool DirectionalInch(RobotDirection inchingDirection);
    void DirectionalAlign(int povValue);

    // Prevent copy/assignment
    DifferentialDrive(const DifferentialDrive &) = delete;
    DifferentialDrive & operator=(const DifferentialDrive &) = delete;

    enum RobotDriveState
    {
        MANUAL_CONTROL,
        DIRECTIONAL_INCH,
        DIRECTIONAL_ALIGN
    };

    typedef Yta::Talon::EmptyTalon  ArcadeDriveTalonType;       // Switch to TalonMotorGroup<TalonFX, TalonFXConfiguration> for real implementation
    ArcadeDriveTalonType *          m_pLeftDriveMotors;         // Left drive motor control
    ArcadeDriveTalonType *          m_pRightDriveMotors;        // Right drive motor control
    RobotDriveState m_RobotDriveState;                          // Keep track of how the drive sequence flows

    // This is the CANBus where the differential drive motors are located
    static constexpr const std::string_view RIO_CAN_BUS_NAME = "rio";

    // Update these when using a differential drive base
    static const unsigned           LEFT_DRIVE_MOTORS_CAN_START_ID          = Yta::Drive::Config::USE_SWERVE_DRIVE ? 64 : 1;
    static const unsigned           RIGHT_DRIVE_MOTORS_CAN_START_ID         = Yta::Drive::Config::USE_SWERVE_DRIVE ? 66 : 3;

    static const unsigned           NUM_LEFT_DRIVE_MOTORS                   = 2U;
    static const unsigned           NUM_RIGHT_DRIVE_MOTORS                  = 2U;

    static constexpr double         DRIVE_THROTTLE_VALUE_RANGE              =  1.00;
    static constexpr double         DRIVE_THROTTLE_VALUE_BASE               =  0.00;
    static constexpr double         DRIVE_SLOW_THROTTLE_VALUE               =  0.35;
    static constexpr double         DRIVE_MOTOR_UPPER_LIMIT                 =  1.00;
    static constexpr double         DRIVE_MOTOR_LOWER_LIMIT                 = -1.00;

    // Constants used by the autonomous functions
    static constexpr double COUNTERACT_COAST_MOTOR_SPEED = 0.20;
    static constexpr units::second_t COUNTERACT_COAST_TIME_S = 0.25_s;



    // These indicate which motor value (+1/-1) represent
    // forward/reverse in the robot.  They are used to keep
    // autonomous movement code common without yearly updates.

    static constexpr double         LEFT_DRIVE_FORWARD_SCALAR               = -1.00;
    static constexpr double         LEFT_DRIVE_REVERSE_SCALAR               = +1.00;
    static constexpr double         RIGHT_DRIVE_FORWARD_SCALAR              = +1.00;
    static constexpr double         RIGHT_DRIVE_REVERSE_SCALAR              = -1.00;

    ////////////////////////////////////////////////////////////////
    // Inputs from joystick:
    //
    // Forward:     (0, -1)
    // Reverse:     (0, +1)
    // Left:        (-1, 0)
    // Right:       (+1, 0)
    //
    // Equations:
    //
    //     x+y   x-y   -x+y   -x-y
    // F:   -1    +1     -1     +1
    // B:   +1    -1     +1     -1
    // L:   -1    -1     +1     +1
    // R:   +1    +1     -1     -1
    //
    // Output to motors:
    //
    // Left forward/right = +1, Right forward/left  = +1:
    // Left reverse/left  = -1, Right reverse/right = -1:
    // x-y, -x-y
    //
    // Left forward/right = -1, Right forward/left  = -1:
    // Left reverse/left  = +1, Right reverse/right = +1:
    // -x+y, x+y
    //
    // Left forward/right = +1, Right forward/left  = -1:
    // Left reverse/left  = -1, Right reverse/right = +1:
    // x-y, x+y
    //
    // Left forward/right = -1, Right forward/left  = +1:
    // Left reverse/left  = +1, Right reverse/right = -1:
    // -x+y, -x-y
    ////////////////////////////////////////////////////////////////

    inline static constexpr double LeftDriveEquation(double xInput, double yInput)
    {
        double leftValue = 0.0;

        if (static_cast<int>(LEFT_DRIVE_FORWARD_SCALAR) == 1)
        {
            leftValue = xInput - yInput;
        }
        else
        {
            leftValue = -xInput + yInput;
        }
        
        return leftValue;
    }

    inline static constexpr double RightDriveEquation(double xInput, double yInput)
    {
        double rightValue = 0.0;

        if (static_cast<int>(RIGHT_DRIVE_FORWARD_SCALAR) == 1)
        {
            rightValue = -xInput - yInput;
        }
        else
        {
            rightValue = xInput + yInput;
        }
        
        return rightValue;
    }
};

#endif // DIFFERENTIALDRIVE_HPP
