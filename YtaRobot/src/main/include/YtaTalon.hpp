////////////////////////////////////////////////////////////////////////////////
/// @file   YtaTalon.hpp
/// @author David Stalter
///
/// @details
/// Custom functionality for easier robot programming of CTRE Talon controllers.
///
/// Copyright (c) 2024 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTATALON_HPP
#define YTATALON_HPP

// SYSTEM INCLUDES
#include <cstdio>                               // for std::snprintf

// C INCLUDES
#include "ctre/phoenix6/TalonFX.hpp"            // for CTRE TalonFX API
#include "frc/smartdashboard/SmartDashboard.h"  // for interacting with the smart dashboard

// C++ INCLUDES
#include "RobotUtils.hpp"                       // for ConvertCelsiusToFahrenheit

using namespace frc;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::signals;


////////////////////////////////////////////////////////////////
/// @namespace Yta::Talon
///
/// Namespace that contains declarations for interacting with
/// Talon speed controllers specific to YTA.
///
////////////////////////////////////////////////////////////////
namespace Yta
{
namespace Talon
{
    // Represents how a motor will be controlled
    enum MotorGroupControlMode
    {
        LEADER,                 // First motor in a group
        FOLLOW,                 // Motor follows the leader
        FOLLOW_INVERSE,         // Motor follows the leader, but inverse
        INDEPENDENT,            // Motor needs to be set independently
        INVERSE,                // Motor is the inverse value of the leader
        INDEPENDENT_OFFSET,     // Motor is set independently, but with a different value from leader
        INVERSE_OFFSET,         // Motor is set independently, but with the a different inverse value from leader
        CUSTOM                  // Motor needs to be set later to an option above
    };

    // Represents a combination of objects to use with a TalonFX motor controller
    struct TalonFxMotorController
    {
        // The Phoenix 6 API requires using different objects with SetControl()
        // function calls.  Create different possible objects to the main robot
        // code doesn't have to worry about it.
        TalonFX * m_pTalonFx;
        DutyCycleOut m_DutyCycleOut;
        PositionVoltage m_PositionVoltage;

        // Constructor
        TalonFxMotorController(int canId) :
            m_pTalonFx(new TalonFX(canId)),
            m_DutyCycleOut(0.0),
            m_PositionVoltage(0.0_tr)
        {}

        // Set the output using duty cycle
        void SetDutyCycle(double dutyCycle)
        {
            (void)m_pTalonFx->SetControl(m_DutyCycleOut.WithOutput(dutyCycle));
        }

        // Set the output to hold a specified position
        void SetPositionVoltage(double angle)
        {
            units::angle::degree_t degrees(angle);
            units::angle::turn_t turns(degrees);
            (void)m_pTalonFx->SetControl(m_PositionVoltage.WithPosition(turns));
        }
    };

    // A structure that doesn't create real TalonFX objects.
    // Intended to be used to keep multiple robot configuration
    // options available (i.e. interchange with TalonMotorGroup).
    struct EmptyTalonFx
    {
        EmptyTalonFx(const char *, unsigned, unsigned, MotorGroupControlMode, NeutralModeValue, bool) {}
        inline void Set(double) {}
        inline void Set(double, double) {}
        inline void DisplayStatusInformation() {}
    };

    static const bool CURRENT_LIMITING_ENABLED = false;
}
}



////////////////////////////////////////////////////////////////
/// @class TalonMotorGroup
///
/// Class that provides methods for interacting with a group of
/// Talon speed controllers.
///
/// @todo: Remove template.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
class TalonMotorGroup
{
public:

    typedef Yta::Talon::MotorGroupControlMode MotorGroupControlMode;
    
    // Constructor
    TalonMotorGroup(
                     const char * pName,
                     unsigned numMotors,
                     unsigned leaderCanId,
                     MotorGroupControlMode nonLeaderControlMode,
                     NeutralModeValue neutralMode,
                     bool bIsDriveMotor = false
                   );

    // Retrieve a specific motor object
    TalonType * GetMotorObject(unsigned canId = GROUP_LEADER_CAN_ID);

    // Adds a new motor to a group
    bool AddMotorToGroup(MotorGroupControlMode controlMode, bool bIsDriveMotor = false);
    
    // Function to set the speed of each motor in the group
    void Set(double value, double offset = 0.0);
    
    // Function to set the motor group output to hold specified angle
    void SetAngle(double angle);
    
    // Sets the control mode of a motor in a group (intended for use with the CUSTOM group control mode)
    bool SetMotorInGroupControlMode(unsigned canId, MotorGroupControlMode controlMode);
    
    // Change Talon mode between brake/coast
    void SetCoastMode();
    void SetBrakeMode();
    
    // Return the value of the sensor connected to the Talon
    int GetEncoderValue();
    void TareEncoder();

    // Displays information to the driver station about the motor group
    void DisplayStatusInformation();
    
private:

    // Represents information about a single motor in a group
    struct MotorInfo
    {
        // Storage space for strings for the smart dashboard
        struct DisplayStrings
        {
            static const unsigned MAX_MOTOR_DISPLAY_STRING_LENGTH = 64U;
            char m_CurrentTemperatureString[MAX_MOTOR_DISPLAY_STRING_LENGTH];
            char m_HighestTemperatureString[MAX_MOTOR_DISPLAY_STRING_LENGTH];
            char m_ResetOccurredString[MAX_MOTOR_DISPLAY_STRING_LENGTH];
        };

        // Member data
        TalonType * m_pTalon;
        DutyCycleOut m_DutyCycleOut;
        PositionVoltage m_PositionVoltage;
        const char * m_pName;
        MotorGroupControlMode m_ControlMode;
        unsigned m_CanId;
        double m_CurrentTemperature;
        double m_HighestTemperature;
        bool m_bResetOccurred;
        bool m_bIsDriveMotor;
        DisplayStrings m_DisplayStrings;
        
        MotorInfo(const char * pName, MotorGroupControlMode controlMode, NeutralModeValue neutralMode, unsigned canId, unsigned groupNumber, bool bIsDriveMotor = false) :
            m_pTalon(new TalonType(static_cast<int>(canId))),
            m_DutyCycleOut(0.0),
            m_PositionVoltage(0.0_tr),
            m_pName(pName),
            m_ControlMode(controlMode),
            m_CanId(canId),
            m_CurrentTemperature(0.0),
            m_HighestTemperature(0.0),
            m_bResetOccurred(false),
            m_bIsDriveMotor(bIsDriveMotor)
        {
            m_pTalon->SetNeutralMode(neutralMode);

            if (controlMode == Yta::Talon::FOLLOW_INVERSE)
            {
                m_pTalon->SetInverted(true);
            }

            // @todo: Move in sensor too?
            if (Yta::Talon::CURRENT_LIMITING_ENABLED && bIsDriveMotor)
            {
                // Limits were 40.0, 55.0, 0.1
                CurrentLimitsConfigs driveMotorCurrentLimits;
                driveMotorCurrentLimits.SupplyCurrentLimit = 55.0;
                driveMotorCurrentLimits.SupplyCurrentThreshold = 60.0;
                driveMotorCurrentLimits.SupplyTimeThreshold = 0.1;
                driveMotorCurrentLimits.SupplyCurrentLimitEnable = true;
                (void)m_pTalon->GetConfigurator().Apply(driveMotorCurrentLimits);
            }

            // Build the strings to use in the display method
            std::snprintf(&m_DisplayStrings.m_CurrentTemperatureString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, groupNumber, "temperature (F)");
            std::snprintf(&m_DisplayStrings.m_HighestTemperatureString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, groupNumber, "highest temperature (F)");
            std::snprintf(&m_DisplayStrings.m_ResetOccurredString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, groupNumber, "reset occurred");
        }

        // Helper routine for configuring some settings on follower talons
        void SetAsFollower(unsigned leaderCanId, bool bInvert)
        {
            // Follower will honor invert control, StrictFollower ignores invert control
            Follower follower(leaderCanId, bInvert);
            (void)m_pTalon->SetControl(follower);

            // Phoenix 6 Example: Get the StatusSignal objects and call SetUpdateFrequency() on them.
            //(void)m_pTalon->GetDeviceTemp().SetUpdateFrequency(100_Hz);
        }
    };

    static const unsigned MAX_NUMBER_OF_MOTORS = 4;
    static const unsigned GROUP_LEADER_CAN_ID = 0xFF;

    // Member variables
    unsigned m_NumMotors;                                   // Number of motors in the group
    unsigned m_LeaderCanId;                                 // Keep track of the CAN ID of the leader Talon in the group
    // @todo: No array, linked list?
    MotorInfo * m_pMotorsInfo[MAX_NUMBER_OF_MOTORS];        // The motor objects
    
    // Prevent default construction/deletion/copy/assignment
    TalonMotorGroup();
    ~TalonMotorGroup();
    TalonMotorGroup( const TalonMotorGroup& ) = delete;
    TalonMotorGroup & operator=( const TalonMotorGroup& ) = delete;
};



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::GetMotorObject
///
/// Retrieves a specific Talon motor object from the motor
/// group.  By default it will return the first motor object in
/// the group (the leader Talon).  If a CAN ID is specified, it
/// will retrieve that object instead.  This purpose of this
/// function is to allow robot code to make specific calls on a
/// motor object that may only apply to one motor in a group or
/// a specific motor type since this is a template class.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
TalonType * TalonMotorGroup<TalonType>::GetMotorObject(unsigned canId)
{
    TalonType * pTalonObject = nullptr;

    // By default, return the first object in the group
    if (canId == GROUP_LEADER_CAN_ID)
    {
        pTalonObject = m_pMotorsInfo[0]->m_pTalon;
    }
    // If a specific CAN ID was given
    else
    {
        // Loop through the motors
        for (unsigned i = 0U; i < m_NumMotors; i++)
        {
            // Check if this is the right motor
            if (m_pMotorsInfo[i]->m_CanId == canId)
            {
                pTalonObject = m_pMotorsInfo[i]->m_pTalon;
                break;
            }
        }
    }

    return pTalonObject;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::TalonMotorGroup
///
/// Constructor.  Creates the number of motors specified
/// starting from the CAN ID passed in.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
TalonMotorGroup<TalonType>::TalonMotorGroup(const char * pName, unsigned numMotors, unsigned leaderCanId,
                                            MotorGroupControlMode nonLeaderControlMode, NeutralModeValue neutralMode, bool bIsDriveMotor) :
    m_NumMotors(numMotors),
    m_LeaderCanId(leaderCanId)
{
    // Loop for each motor to create
    for (unsigned i = 0U; (i < numMotors) && (i < MAX_NUMBER_OF_MOTORS); i++)
    {
        // Group IDs are used in creating the strings and are not zero based
        unsigned groupId = i + 1U;

        // The leader Talon is unique
        if (i == 0U)
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(pName, Yta::Talon::LEADER, neutralMode, leaderCanId, groupId, bIsDriveMotor);
        }
        // Non-leader Talons
        else
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(pName, nonLeaderControlMode, neutralMode, (leaderCanId + i), groupId, bIsDriveMotor);

            // Only set follow for Talon groups that will be configured as
            // such.  The CTRE Phoenix library now passes the control mode in
            // the Set() method, so we only need to set the followers here.
            if ((nonLeaderControlMode == Yta::Talon::FOLLOW) || (nonLeaderControlMode == Yta::Talon::FOLLOW_INVERSE))
            {
                bool bInvert = (nonLeaderControlMode == Yta::Talon::FOLLOW) ? false : true;
                m_pMotorsInfo[i]->SetAsFollower(leaderCanId, bInvert);
            }
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::AddMotorToGroup
///
/// Method to add a new motor to a motor group.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
bool TalonMotorGroup<TalonType>::AddMotorToGroup(MotorGroupControlMode controlMode, bool bIsDriveMotor)
{
    bool bResult = false;

    // Make sure there's room for another motor in this group
    if (m_NumMotors < MAX_NUMBER_OF_MOTORS)
    {
        // The new motor CAN ID is the first motor's ID + current number of group motors present
        unsigned newMotorCanId = m_pMotorsInfo[0]->m_CanId + m_NumMotors;

        // m_NumMotors can be leveraged as the index, as it represents the next unused array element
        // All motors in a group have the same name, so we use the existing one.  Group ID is computed from m_NumMotors.
        m_pMotorsInfo[m_NumMotors] = new MotorInfo(m_pMotorsInfo[0]->m_pName, controlMode, newMotorCanId, (m_NumMotors + 1), bIsDriveMotor);
        
        // If this Talon will be a follower, be sure to call Set() to enable it
        if ((controlMode == Yta::Talon::FOLLOW) || (controlMode == Yta::Talon::FOLLOW_INVERSE))
        {
            bool bInvert = (controlMode == Yta::Talon::FOLLOW) ? false : true;
            m_pMotorsInfo[m_NumMotors]->SetAsFollower(m_LeaderCanId, bInvert);
        }

        // Increase the number of motors
        m_NumMotors++;
        
        // Indicate success
        bResult = true;
    }

    return bResult;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetMotorInGroupControlMode
///
/// Method to set the control mode of a motor in a group.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
bool TalonMotorGroup<TalonType>::SetMotorInGroupControlMode(unsigned canId, MotorGroupControlMode controlMode)
{
    bool bResult = false;
    
    // Search for the correct motor in the group
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        // If it matches...
        if (m_pMotorsInfo[i]->m_CanId == canId)
        {
            // ...set the control mode
            m_pMotorsInfo[i]->m_ControlMode = controlMode;

            // If this Talon will be a follower, be sure to call Set() to enable it
            if ((controlMode == Yta::Talon::FOLLOW) || (controlMode == Yta::Talon::FOLLOW_INVERSE))
            {
                bool bInvert = (controlMode == Yta::Talon::FOLLOW) ? false : true;
                m_pMotorsInfo[i]->SetAsFollower(m_LeaderCanId, bInvert);
            }
            else
            {
                // The previous mode might have had follower frame rates, so they need to be reset
                // Phoenix 6 Example: Get the StatusSignal objects and call SetUpdateFrequency() on them.
                //(void)m_pMotorsInfo[i]->m_pTalon->GetDeviceTemp().SetUpdateFrequency(100_Hz);
            }

            // Update the inverted status.  Only FOLLOW_INVERSE uses the built-in invert.
            if (controlMode == Yta::Talon::FOLLOW_INVERSE)
            {
                m_pMotorsInfo[i]->m_pTalon->SetInverted(true);
            }
            else
            {
                m_pMotorsInfo[i]->m_pTalon->SetInverted(false);
            }
            
            // Indicate success
            bResult = true;
        }
    }

    return bResult;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetCoastMode
///
/// Method to change a talon to coast mode.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::SetCoastMode()
{
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        m_pMotorsInfo[i]->m_pTalon->SetNeutralMode(NeutralModeValue::Coast);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetBrakeMode
///
/// Method to change a talon to brake mode.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::SetBrakeMode()
{
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        m_pMotorsInfo[i]->m_pTalon->SetNeutralMode(NeutralModeValue::Brake);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::Set
///
/// Method to set the speed of each motor in the group.  The
/// offset parameter is only valid for motor groups configured
/// as *_OFFSET.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::Set(double value, double offset)
{
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        // Setting motor values for groups assumes that the first half of
        // motors in a group should always get the same value, and the second
        // half of motors in a group could be different (such as inverse or offset).
        // Keep track of which segment of the motor group this motor is in.
        
        // Most modes wil need to call Set() later, but some won't
        bool bCallSet = true;
        
        // The value that will be passed to Set()
        double valueToSet = 0.0;
        
        // Check what the control mode of this motor is.  Most CAN Talons
        // will be set to follow, but some may be independent or inverse (such
        // as if they need to drive in different directions).
        switch (m_pMotorsInfo[i]->m_ControlMode)
        {
            case Yta::Talon::LEADER:
            case Yta::Talon::INDEPENDENT:
            {
                // The leader always gets set via duty cycle, as do motors
                // that are independently controlled (not follow or inverse).
                valueToSet = value;
                break;
            }
            case Yta::Talon::FOLLOW:
            case Yta::Talon::FOLLOW_INVERSE:
            {
                // Nothing to do, motor had SetControl() called during object construction
                bCallSet = false;
                break;
            }
            case Yta::Talon::INVERSE:
            {
                // Motor is attached to drive in opposite direction of leader
                valueToSet = -value;
                break;
            }
            case Yta::Talon::INDEPENDENT_OFFSET:
            {
                // The non-leader motor has a different value in this case
                valueToSet = value + offset;
                break;
            }
            case Yta::Talon::INVERSE_OFFSET:
            {
                // The non-leader motor has a different value in this case
                valueToSet = -(value + offset);
                break;
            }
            default:
            {
                // Can reach here with CUSTOM motors still set.  Calling code should
                // update those motors to a different control mode via class API calls.
                break;
            }
        };
            
        if (bCallSet)
        {
            // Set the value in the Talon
            m_pMotorsInfo[i]->m_pTalon->SetControl(m_pMotorsInfo[i]->m_DutyCycleOut.WithOutput(valueToSet));
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetAngle
///
/// Method to set the output of a motor to hold a specified
/// angle.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::SetAngle(double angle)
{
    // The first entry in the motor info array should always
    // be a leader, so that one will be the only one updated.
    // When holding position, we don't want to have to manage
    // the target angles for multiple motors since it would
    // require tracking different set points based on the
    // encoders.  For a motor group to successfully hold
    // position, one of them needs to be a follower.
    // @todo: Check that this configuration is only applied to follower groups.

    // Set the control output
    units::angle::degree_t degrees(angle);
    units::angle::turn_t turns(degrees);
    m_pMotorsInfo[0]->m_pTalon->SetControl(m_pMotorsInfo[0]->m_PositionVoltage.WithPosition(turns));
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::DisplayStatusInformation
///
/// Sends status information to the smart dashboard.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::DisplayStatusInformation()
{
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        m_pMotorsInfo[i]->m_CurrentTemperature = RobotUtils::ConvertCelsiusToFahrenheit(m_pMotorsInfo[i]->m_pTalon->GetDeviceTemp().GetValueAsDouble());
        if (m_pMotorsInfo[i]->m_CurrentTemperature > m_pMotorsInfo[i]->m_HighestTemperature)
        {
            m_pMotorsInfo[i]->m_HighestTemperature = m_pMotorsInfo[i]->m_CurrentTemperature;
        }

        // @todo: Also consider sticky faults?
        m_pMotorsInfo[i]->m_bResetOccurred = m_pMotorsInfo[i]->m_pTalon->HasResetOccurred();

        SmartDashboard::PutNumber(m_pMotorsInfo[i]->m_DisplayStrings.m_CurrentTemperatureString, m_pMotorsInfo[i]->m_CurrentTemperature);
        SmartDashboard::PutNumber(m_pMotorsInfo[i]->m_DisplayStrings.m_HighestTemperatureString, m_pMotorsInfo[i]->m_HighestTemperature);
        SmartDashboard::PutBoolean(m_pMotorsInfo[i]->m_DisplayStrings.m_ResetOccurredString, m_pMotorsInfo[i]->m_bResetOccurred);
    }
}

#endif // YTATALON_HPP
