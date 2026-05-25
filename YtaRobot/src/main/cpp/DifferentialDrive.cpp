////////////////////////////////////////////////////////////////////////////////
/// @file   DifferentialDrive.cpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a differential drive robot base.  This file is
/// largely legacy due to the prevalance of swerve drive robots.
///
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "DifferentialDrive.hpp"                // for class declaration
#include "RobotUtils.hpp"                       // for Limit(), DEBUG_PRINTS

using namespace frc;


////////////////////////////////////////////////////////////////
/// @method DifferentialDrive::DifferentialDrive
///
/// Constructor.  Creates the member objects needed for a
/// differential drive base.
///
////////////////////////////////////////////////////////////////
DifferentialDrive::DifferentialDrive(const std::function<const CANBus&(std::string_view)>& rGetCanBusReferenceLambda) :
    m_pLeftDriveMotors(new ArcadeDriveTalonType("Left Drive", NUM_LEFT_DRIVE_MOTORS, LEFT_DRIVE_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW, NeutralModeValue::Brake, rGetCanBusReferenceLambda(RIO_CAN_BUS_NAME))),
    m_pRightDriveMotors(new ArcadeDriveTalonType("Right Drive", NUM_RIGHT_DRIVE_MOTORS, RIGHT_DRIVE_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW, NeutralModeValue::Brake, rGetCanBusReferenceLambda(RIO_CAN_BUS_NAME))),
    m_RobotDriveState(MANUAL_CONTROL)
{
}



////////////////////////////////////////////////////////////////
/// @method DifferentialDrive::DriveSequence
///
/// This method contains the main workflow for drive control.
/// It will gather input from the drive joystick (via the passed
/// in lambda) and then filter those values to ensure they are
/// past a certain threshold and within range to send to the
/// speed controllers and finally actually set them.
///
/// Note: Due to prominence of swerve, much of this code has not
///       recently been tested and may not work as expected.
///
////////////////////////////////////////////////////////////////
void DifferentialDrive::DriveSequence(std::function<const DriveControlInputs & ()> & rGetDriveControlInputsLambda)
{
    // Retrieve the drive control inputs
    const DriveControlInputs & driveControlInputs = rGetDriveControlInputsLambda();

    if (Yta::Drive::Config::DIRECTIONAL_ALIGN_ENABLED)
    {
        // Check for a directional align first
        DirectionalAlign(driveControlInputs.m_PovValue);

        // If an align is in progress, do not accept manual driver input
        if (m_RobotDriveState == DIRECTIONAL_ALIGN)
        {
            return;
        }
    }

    if (Yta::Drive::Config::DIRECTIONAL_INCH_ENABLED)
    {
        // If a directional inch occurred, just return
        if (DirectionalInch(driveControlInputs.m_InchingDirection))
        {
            return;
        }
    }

    // Computes what the maximum drive speed could be
    double throttleControl = (driveControlInputs.m_Throttle * DRIVE_THROTTLE_VALUE_RANGE) + DRIVE_THROTTLE_VALUE_BASE;

    // All the controllers are normalized
    // to represent the x and y axes with
    // the following values:
    //   -1
    //    |
    // -1---+1
    //    |
    //   +1

    // Get driver X/Y inputs
    double xAxisDrive = driveControlInputs.m_xAxis * throttleControl;
    double yAxisDrive = driveControlInputs.m_yAxis * throttleControl;

    if (RobotUtils::DEBUG_PRINTS)
    {
        SmartDashboard::PutNumber("x-axis input", driveControlInputs.m_xAxis);
        SmartDashboard::PutNumber("y-axis input", driveControlInputs.m_yAxis);
        SmartDashboard::PutNumber("x-axis throttled", xAxisDrive);
        SmartDashboard::PutNumber("y-axis throttled", yAxisDrive);
    }

    // If the swap direction button was pressed, negate y value
    if (driveControlInputs.m_bDriveSwap)
    {
        yAxisDrive *= -1.0;
    }

    // By default, the drive equations cause the x-axis input
    // to be flipped when going reverse.  Correct that here,
    // if configured.  Remember, y-axis full forward is negative.
    if ((!Yta::Drive::Config::USE_INVERTED_REVERSE_CONTROLS) && (yAxisDrive > 0.0))
    {
        xAxisDrive *= -1.0;
    }

    if (Yta::Drive::Config::SLOW_DRIVE_ENABLED)
    {
        // Get the slow drive control joystick input
        double xAxisSlowDrive = driveControlInputs.m_xAxisSlow * DRIVE_SLOW_THROTTLE_VALUE;
        
        // If the normal x-axis drive is non-zero, use it.  Otherwise use the slow drive input, which could also be zero.
        xAxisDrive = (xAxisDrive != 0.0) ? xAxisDrive : xAxisSlowDrive;
    }

    // Filter motor speeds
    double leftSpeed = RobotUtils::Limit((LeftDriveEquation(xAxisDrive, yAxisDrive)), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    double rightSpeed = RobotUtils::Limit(RightDriveEquation(xAxisDrive, yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);

    // Set motor speed
    m_pLeftDriveMotors->SetDutyCycle(leftSpeed);
    m_pRightDriveMotors->SetDutyCycle(rightSpeed);

    if (RobotUtils::DEBUG_PRINTS)
    {
        SmartDashboard::PutNumber("Left drive speed", leftSpeed);
        SmartDashboard::PutNumber("Right drive speed", rightSpeed);
    }

    SmartDashboard::PutBoolean("Drive swap", driveControlInputs.m_bDriveSwap);
    m_pLeftDriveMotors->DisplayStatusInformation();
    m_pRightDriveMotors->DisplayStatusInformation();
}



////////////////////////////////////////////////////////////////
/// @method DifferentialDrive::DirectionalInch
///
/// This method contains the main workflow for drive directional
/// inching.  Based on input direction, it will briefly move the
/// robot a slight amount in that direction.
///
////////////////////////////////////////////////////////////////
bool DifferentialDrive::DirectionalInch(RobotDirection inchingDirection)
{
    static Timer * pInchingDriveTimer = new Timer();
    static constexpr units::second_t INCHING_DRIVE_DELAY_S = 0.10_s;
    static constexpr double INCHING_DRIVE_SPEED = 0.25;

    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    switch (inchingDirection)
    {
        case RobotDirection::ROBOT_FORWARD:
        {
            leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        case RobotDirection::ROBOT_REVERSE:
        {
            leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        case RobotDirection::ROBOT_LEFT:
        {
            leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        case RobotDirection::ROBOT_RIGHT:
        {
            leftSpeed = INCHING_DRIVE_SPEED * LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed = INCHING_DRIVE_SPEED * RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        default:
        {
            break;
        }
    }
    
    if ((leftSpeed == 0.0) && (rightSpeed == 0.0))
    {
        // No directional inch input, just return
        return false;
    }
    
    // Start the timer
    pInchingDriveTimer->Reset();
    pInchingDriveTimer->Start();
    
    // Motors on
    m_pLeftDriveMotors->SetDutyCycle(leftSpeed);
    m_pRightDriveMotors->SetDutyCycle(rightSpeed);
    
    while (pInchingDriveTimer->Get() < INCHING_DRIVE_DELAY_S)
    {
    }
    
    // Motors back off
    m_pLeftDriveMotors->SetDutyCycle(0.0);
    m_pRightDriveMotors->SetDutyCycle(0.0);
    
    // Stop the timer
    pInchingDriveTimer->Stop();
    pInchingDriveTimer->Reset();

    return true;
}



////////////////////////////////////////////////////////////////
/// @method DifferentialDrive::DirectionalAlign
///
/// This method contains the main workflow for automatically
/// aligning the robot to an angle based on input from the
/// driver.  The angles are relative to the robot at the start
/// of the match (when power is applied to the gyro and zero
/// is set).  The robot angle is reported as follows:
///
///     0
///     |
/// 270---90
///     |
///    180
///
/// The POV input is used to pick the angle to align to.  The
/// corresponding input on the d-pad maps 1:1 to the drawing.
///
////////////////////////////////////////////////////////////////
void DifferentialDrive::DirectionalAlign(int povValue)
{
    static Timer * pDirectionalAlignTimer = new Timer();
    static constexpr units::second_t DIRECTIONAL_ALIGN_MAX_TIME_S = 3.00_s;
    static constexpr double DIRECTIONAL_ALIGN_DRIVE_SPEED = 0.55;

    // Retain the last POV value between function invocations
    static int lastPovValue = -1;
    
    // Indicate whether or not a change between align/no align is allowed
    static bool bStateChangeAllowed = false;
    
    // Check if it changed since last function call
    if (povValue != lastPovValue)
    {
        // Something changed, figure out what
        
        // POV button was released
        if (povValue == -1)
        {
            // State change not allowed until next button press
            bStateChangeAllowed = false;
        }
        // POV button was pressed
        else if (lastPovValue == -1)
        {
            // State change allowed since button is now pressed
            bStateChangeAllowed = true;
        }
        // There was some change in the already pressed POV value, which doesn't matter
        else
        {
        }
    }
    
    const int POV_NORMALIZATION_ANGLE = 45;
    const int ANGLE_90_DEGREES = 90;
    const int ANGLE_180_DEGREES = 180;
    const int ANGLE_360_DEGREES = 360;
    
    // Save off a new last POV value
    lastPovValue = povValue;
    
    // This alignment uses the following from the POV input:
    //
    // ///////////////////////
    // //   315      45     //
    // //     \  up  /      //
    // // left |    | right //
    // //     / down \      //
    // //   225      135    //
    // ///////////////////////
    //
    // The input value (0 -> 360) will be normalized such that
    // angle 315 is interpreted as zero.
    static int destinationAngle = -1;
    
    switch (m_RobotDriveState)
    {
        case MANUAL_CONTROL:
        {
            // Only start an align if a state change is allowed
            if (bStateChangeAllowed)
            {                
                // @todo: Switch this logic to use GetPovAsDirection().
                //        Also requires updating the POV state change logic above.

                // This gives a value between 45 -> 405
                povValue += POV_NORMALIZATION_ANGLE;
                
                // Normalize between 0 -> 360 (maps 0:360 in to 45:360:0:45 out)
                if (povValue >= ANGLE_360_DEGREES)
                {
                    povValue -= ANGLE_360_DEGREES;
                }
                
                // Now at value between 0 -> 360, where:
                // 0 -> 89 = align up
                // 90 -> 179 = align right
                // 180 -> 269 = align down
                // 270 -> 359 = align left
                // Get a scalar multiplier to find the destination angle.
                // Making this volatile to prevent the compiler from trying
                // to optimize the division followed by multliplication of
                // the same constant.  Integer division is deliberate.
                // This gives a scalar multiplier of 0 -> 3
                volatile int degreeMultiplier = (povValue / ANGLE_90_DEGREES);
                
                // Find the destination angle.
                // This gives a value of 0, 90, 180 or 270
                destinationAngle = ANGLE_90_DEGREES * degreeMultiplier;
                
                // Read the starting angle
                // @todo: Use Pigeon2 to get angle (requires using the lambda to pass it in).
                int startingAngle = 0;
                
                // Do some angle math to figure out which direction is faster to turn.
                // Examples:
                // Starting: 45, 180    Destination: 0, 90, 180, 270
                // 45 - 0 = 45          180 - 0 = 180
                // 45 - 90 = -45        180 - 90 = 90
                // 45 - 180 = -135      180 - 180 = 0
                // 45 - 270 = -225      180 - 270 = -90
                int angleDistance = startingAngle - destinationAngle;
                int absValueAngleDistance = std::abs(angleDistance);
                
                // Variables to indicate which way to turn
                bool bTurnLeft = false;
                bool bTurnRight = false;
                
                // Figure out which way to turn
                if (angleDistance > 0)
                {
                    // Target is to the left of where we are
                    bTurnLeft = true;
                }
                else
                {
                    // Target is to the right of where we are
                    bTurnRight = true;
                }

                // If the target distance is more than halfway around, it's actually faster to turn the other way 
                if (absValueAngleDistance > ANGLE_180_DEGREES)
                {
                    bTurnLeft = !bTurnLeft;
                    bTurnRight = !bTurnRight;
                }
                
                // The destination angle and direction is now known, time to do the move
                if (bTurnLeft)
                {
                    m_pLeftDriveMotors->SetDutyCycle(DIRECTIONAL_ALIGN_DRIVE_SPEED * LEFT_DRIVE_REVERSE_SCALAR);
                    m_pRightDriveMotors->SetDutyCycle(DIRECTIONAL_ALIGN_DRIVE_SPEED * RIGHT_DRIVE_FORWARD_SCALAR);
                }
                if (bTurnRight)
                {
                    m_pLeftDriveMotors->SetDutyCycle(DIRECTIONAL_ALIGN_DRIVE_SPEED * LEFT_DRIVE_FORWARD_SCALAR);
                    m_pRightDriveMotors->SetDutyCycle(DIRECTIONAL_ALIGN_DRIVE_SPEED * RIGHT_DRIVE_REVERSE_SCALAR);
                }
                
                // Start the safety timer
                pDirectionalAlignTimer->Start();

                // Indicate a state change is not allowed until POV release
                bStateChangeAllowed = false;
                
                // Indicate a directional align is in process
                m_RobotDriveState = DIRECTIONAL_ALIGN;
            }
            
            break;
        }
        case DIRECTIONAL_ALIGN:
        {   
            // Three conditions for stopping the align:
            // 1. Destination angle is reached
            // 2. Safety timer expires
            // 3. User cancels the operation
            // @todo: Is it a problem that (destinationAngle - 1) can be negative when angle == zero?
            // @todo: Use Pigeon2 to get angle.
            int currentAngle = 0;
            if (((currentAngle >= (destinationAngle - 1)) && (currentAngle <= (destinationAngle + 1))) ||
                (pDirectionalAlignTimer->Get() > DIRECTIONAL_ALIGN_MAX_TIME_S) ||
                (bStateChangeAllowed))
            {
                // Motors 0.0
                m_pLeftDriveMotors->SetDutyCycle(0.0);
                m_pRightDriveMotors->SetDutyCycle(0.0);
                
                // Reset the safety timer
                pDirectionalAlignTimer->Stop();
                pDirectionalAlignTimer->Reset();
                
                // Clear this just to be safe
                destinationAngle = -1;
                
                // Indicate a state change is not allowed until POV release
                bStateChangeAllowed = false;
                
                // Align done, back to manual control
                m_RobotDriveState = MANUAL_CONTROL;
            }
            
            break;
        }
        default:
        {
            break;
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method DifferentialDrive::AutonomousDrive
///
/// Drives autonomously for a specified amount of time.
///
////////////////////////////////////////////////////////////////
void DifferentialDrive::AutonomousDrive(RobotDirection direction, double speed, units::second_t time)
{
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    switch (direction)
    {
        case RobotDirection::ROBOT_FORWARD:
        {
            leftSpeed = speed * LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        case RobotDirection::ROBOT_REVERSE:
        {
            leftSpeed = speed * LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        case RobotDirection::ROBOT_LEFT:
        {
            leftSpeed = speed * LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        case RobotDirection::ROBOT_RIGHT:
        {
            leftSpeed = speed * LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed = speed * RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        default:
        {
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            break;
        }
    }

    // First turn the motors on
    m_pLeftDriveMotors->SetDutyCycle(leftSpeed);
    m_pRightDriveMotors->SetDutyCycle(rightSpeed);

    // Time it
    Wait(time);

    // Motors back off
    m_pLeftDriveMotors->SetDutyCycle(0.0);
    m_pRightDriveMotors->SetDutyCycle(0.0);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DifferentialDrive
///
/// Back drives the motors to abruptly stop the robot.
///
////////////////////////////////////////////////////////////////
void DifferentialDrive::AutonomousBackDrive(RobotDirection currentDirection)
{
    double leftSpeed = COUNTERACT_COAST_MOTOR_SPEED;
    double rightSpeed = COUNTERACT_COAST_MOTOR_SPEED;

    switch (currentDirection)
    {
        // If we are currently going forward, back drive is reverse
        case RobotDirection::ROBOT_FORWARD:
        {
            leftSpeed *= LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed *= RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        // If we are currently going reverse, back drive is forward
        case RobotDirection::ROBOT_REVERSE:
        {
            leftSpeed *= LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed *= RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        default:
        {
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            break;
        }
    }
    
    // Counteract coast
    m_pLeftDriveMotors->SetDutyCycle(leftSpeed);
    m_pRightDriveMotors->SetDutyCycle(rightSpeed);
    
    // Delay
    Wait(COUNTERACT_COAST_TIME_S);
    
    // Motors off
    m_pLeftDriveMotors->SetDutyCycle(0.0);
    m_pRightDriveMotors->SetDutyCycle(0.0);
}



////////////////////////////////////////////////////////////////
/// @method DifferentialDrive::AutonomousBackDriveTurn
///
/// Back drives the motors to abruptly stop the robot during
/// a turn.
///
////////////////////////////////////////////////////////////////
void DifferentialDrive::AutonomousBackDriveTurn(RobotDirection currentDirection)
{
    double leftSpeed = COUNTERACT_COAST_MOTOR_SPEED;
    double rightSpeed = COUNTERACT_COAST_MOTOR_SPEED;

    switch (currentDirection)
    {
        // If the turn is left, counteract is right
        case RobotDirection::ROBOT_LEFT:
        {
            leftSpeed *= LEFT_DRIVE_FORWARD_SCALAR;
            rightSpeed *= RIGHT_DRIVE_REVERSE_SCALAR;
            break;
        }
        // If the turn is right, counteract is left
        case RobotDirection::ROBOT_RIGHT:
        {
            leftSpeed *= LEFT_DRIVE_REVERSE_SCALAR;
            rightSpeed *= RIGHT_DRIVE_FORWARD_SCALAR;
            break;
        }
        default:
        {
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            break;
        }
    }
    
    // Counteract coast
    m_pLeftDriveMotors->SetDutyCycle(leftSpeed);
    m_pRightDriveMotors->SetDutyCycle(rightSpeed);
    
    // Delay
    Wait(COUNTERACT_COAST_TIME_S);
    
    // Motors off
    m_pLeftDriveMotors->SetDutyCycle(0.0);
    m_pRightDriveMotors->SetDutyCycle(0.0);
}
