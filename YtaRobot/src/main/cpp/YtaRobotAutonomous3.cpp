////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous3.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 3 for YtaRobot.
///
/// Copyright (c) 2024 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotUtils.hpp"               // for DisplayMessage()
#include "YtaRobot.hpp"                 // for robot class declaration
#include "YtaRobotAutonomous.hpp"       // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousRoutine3
///
/// Autonomous routine 3.  Shoots from the amp side, picks up
/// a second note and shoots again.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine3()
{
    // Local objects needed during autonomous
    TalonFX * pPivotLeaderTalon = m_pPivotMotors->GetMotorObject(PIVOT_MOTORS_CAN_START_ID);
    PositionVoltage pivotPositionVoltage(0.0_tr);

    double shooterSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_CLOSE_CCW_SPEED : SHOOTER_MOTOR_SPEAKER_CLOSE_CW_SPEED;
    double shooterOffsetSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_CCW_OFFSET_SPEED : SHOOTER_MOTOR_SPEAKER_CW_OFFSET_SPEED;

    // First start ramping up the shooter motors
    m_pShooterMotors->Set(shooterSpeed, shooterOffsetSpeed);

    // Pivot the mechanism to the desired angle
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_TOUCHING_SPEAKER));

    // Wait a bit for everything to be ready
    AutonomousDelay(1.5_s);

    // Feeder motor to take the shot
    m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
    AutonomousDelay(0.5_s);

    // Shooter and feeder motors off
    m_pShooterMotors->Set(0.0);
    m_pFeederMotor->SetDutyCycle(0.0);

    // Intake motor on and reposition pivot before moving to the next note
    m_pIntakeMotor->SetDutyCycle(INTAKE_MOTOR_SPEED);
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_INTAKE_NOTE));

    // Compute the direction of movement based on alliance color since the field is not symmetrical
    RobotStrafe autoLeaveByAmpStrafe = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? RobotStrafe::ROBOT_STRAFE_LEFT : RobotStrafe::ROBOT_STRAFE_RIGHT;
    RobotRotation autoLeaveByAmpRotation = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? RobotRotation::ROBOT_COUNTER_CLOCKWISE : RobotRotation::ROBOT_CLOCKWISE;
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, autoLeaveByAmpStrafe, autoLeaveByAmpRotation);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.15, 0.18, 0.05, 2.5_s, true);
    AutonomousDelay(1.0_s);

    // Note should be picked up, intake off
    m_pIntakeMotor->SetDutyCycle(0.0);

    // Start the far shot

    // First start ramping up the shooter motors
    shooterSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_PODIUM_CCW_SPEED : SHOOTER_MOTOR_SPEAKER_PODIUM_CW_SPEED;
    shooterOffsetSpeed = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? SHOOTER_MOTOR_SPEAKER_CCW_OFFSET_SPEED : SHOOTER_MOTOR_SPEAKER_CW_OFFSET_SPEED;
    m_pShooterMotors->Set(shooterSpeed, shooterOffsetSpeed);

    // Pivot the mechanism to the desired angle
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_FROM_PODIUM - 9.0_deg));

    // Adjust robot angle toward speaker
    RobotRotation towardSpeakerRotation = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? RobotRotation::ROBOT_CLOCKWISE : RobotRotation::ROBOT_COUNTER_CLOCKWISE;
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, RobotStrafe::ROBOT_NO_STRAFE, towardSpeakerRotation);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.0, 0.0, 0.08, 1.0_s, true);

    // Feeder motor to take the shot
    m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
    AutonomousDelay(1.0_s);

    // Shooter motor off, feeder off, pivot down
    m_pShooterMotors->Set(0.0);
    m_pFeederMotor->SetDutyCycle(0.0);
    (void)pPivotLeaderTalon->SetControl(pivotPositionVoltage.WithPosition(PIVOT_ANGLE_RUNTIME_BASE));

    // Set the pigeon angle relative to final robot position with zero down field
    //units::angle::degree_t gyroYaw = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? -23.0_deg : 23.0_deg;
    //m_pPigeon->SetYaw(gyroYaw);
    //return;


    // Extended auto to try and clear the midline to disrupt the other alliance's auto.
    // Remember with the extended auto that robot zero degrees still points from the starting position at the speaker.


    // Back up again to try and clear out the center notes
    RobotStrafe autoMoveToMidlineStrafe = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? RobotStrafe::ROBOT_STRAFE_LEFT : RobotStrafe::ROBOT_STRAFE_RIGHT;
    RobotRotation autoMoveToMidlineRotation = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? RobotRotation::ROBOT_CLOCKWISE : RobotRotation::ROBOT_COUNTER_CLOCKWISE;
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, autoMoveToMidlineStrafe, autoMoveToMidlineRotation);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.25, 0.50, 0.08, 1.75_s, true);
    AutonomousDelay(0.1_s);

    // The robot is roughly on the midline now, facing the amp wall.
    // Zero the gyro to try and keep the next motion simpler
    m_pPigeon->SetYaw(0.0_deg);

    // Now move along the midline while rotating to disrupt the notes
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, autoMoveToMidlineRotation);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.4, 0.0, 0.2, 3.0_s, true);
    AutonomousDelay(0.1_s);

    // Come back onto the alliance side of the field to avoid getting penalized
    RobotStrafe autoMoveBackToAllianceStrafe = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? RobotStrafe::ROBOT_STRAFE_RIGHT : RobotStrafe::ROBOT_STRAFE_LEFT;
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, autoMoveBackToAllianceStrafe, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.0, 0.3, 0.0, 0.75_s, true);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 3 done.");
}
