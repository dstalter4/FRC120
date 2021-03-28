////////////////////////////////////////////////////////////////////////////////
/// @file   TalonMotorGroup.hpp
/// @author David Stalter
///
/// @details
/// A class designed to work with a group of CAN Talon speed controllers working
/// in tandem.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef TALONMOTORGROUP_HPP
#define TALONMOTORGROUP_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "ctre/Phoenix.h"               // for CTRE library interfaces

// C++ INCLUDES
// (none)


////////////////////////////////////////////////////////////////
/// @namespace YtaTalon
///
/// Namespace that contains declarations for interacting with
/// Talon speed controllers specific to YTA.
///
////////////////////////////////////////////////////////////////
namespace YtaTalon
{
    // Represents how a motor will be controlled
    enum MotorGroupControlMode
    {
        MASTER,                 // First motor in a group
        FOLLOW,                 // Motor follows the master
        INDEPENDENT,            // Motor needs to be set independently
        INVERSE,                // Motor is the inverse value of the master
        INDEPENDENT_OFFSET,     // Motor is set independently, but with a different value from master
        INVERSE_OFFSET,         // Motor is set independently, but with the a different inverse value from master
        CUSTOM                  // Motor needs to be set later to an option above
    };
}



////////////////////////////////////////////////////////////////
/// @class TalonMotorGroup
///
/// Class that provides methods for interacting with a group of
/// Talon speed controllers.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
class TalonMotorGroup
{
public:

    typedef YtaTalon::MotorGroupControlMode MotorGroupControlMode;
    
    // Constructor
    TalonMotorGroup(
                     int numMotors,
                     int masterCanId,
                     MotorGroupControlMode nonMasterControlMode,
                     FeedbackDevice sensor = FeedbackDevice::None
                   );

    // Retrieve a specific motor object
    TalonType * GetMotorObject(int canId = GROUP_MASTER_CAN_ID);

    // Adds a new motor to a group
    bool AddMotorToGroup(MotorGroupControlMode controlMode);
    
    // Function to set the speed of each motor in the group
    void Set( double value, double offset = 0.0 );
    
    // Sets the control mode of a motor in a group (intended for use with the CUSTOM group control mode)
    bool SetMotorInGroupControlMode(int canId, MotorGroupControlMode controlMode);
    
    // Change Talon mode between brake/coast
    void SetCoastMode();
    void SetBrakeMode();
    
    // Return the value of the sensor connected to the Talon
    int GetEncoderValue();
    void TareEncoder();
    
private:
    
    // Represents information about a single motor in a group
    struct MotorInfo
    {
        TalonType * m_pTalon;
        MotorGroupControlMode m_ControlMode;
        int m_CanId;
        
        MotorInfo(MotorGroupControlMode controlMode, int canId) :
            m_pTalon(new TalonType(canId)),
            m_ControlMode(controlMode),
            m_CanId(canId)
        {
        }
    };

    static const int MAX_NUMBER_OF_MOTORS = 4;
    static const int GROUP_MASTER_CAN_ID = 0xFF;

    // Member variables
    int m_NumMotors;                                        // Number of motors in the group
    int m_MasterCanId;                                      // Keep track of the CAN ID of the master Talon in the group
    FeedbackDevice m_Sensor;                                // Keep track of the sensor attached to the Talon (assumes one sensor per group)
    MotorInfo *  m_pMotorsInfo[MAX_NUMBER_OF_MOTORS];       // The motor objects
    
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
/// the group (the master Talon).  If a CAN ID is specified, it
/// will retrieve that object instead.  This purpose of this
/// function is to allow robot code to make specific calls on a
/// motor object that may only apply to one motor in a group or
/// a specific motor type since this is a template class.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
TalonType * TalonMotorGroup<TalonType>::GetMotorObject(int canId)
{
    TalonType * pTalonObject = nullptr;

    // By default, return the first object in the group
    if (canId == GROUP_MASTER_CAN_ID)
    {
        pTalonObject = m_pMotorsInfo[0]->m_pTalon;
    }
    // If a specific CAN ID was given
    else
    {
        // Loop through the motors
        for (int i = 0; i < m_NumMotors; i++)
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
TalonMotorGroup<TalonType>::TalonMotorGroup( int numMotors, int masterCanId, MotorGroupControlMode nonMasterControlMode, FeedbackDevice sensor ) :
    m_NumMotors(numMotors),
    m_MasterCanId(masterCanId),
    m_Sensor(sensor)
{
    // Loop for each motor to create
    for ( int i = 0; (i < numMotors) && (i < MAX_NUMBER_OF_MOTORS); i++ )
    {
        // The master Talon is unique
        if (i == 0)
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(YtaTalon::MASTER, masterCanId);
            
            // This assumes only the first controller in a group has a sensor
            if (sensor != FeedbackDevice::None)
            {
                // Sensor initialization (feedbackDevice, pidIdx, timeoutMs)
                m_pMotorsInfo[0]->m_pTalon->ConfigSelectedFeedbackSensor(sensor, 0, 0);
            }
        }
        // Non-master Talons
        else
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(nonMasterControlMode, (masterCanId + i));

            // Only set follow for Talon groups that will be configured as
            // such.  The CTRE Phoenix library now passes the control mode in
            // the Set() method, so we only need to set the followers here.
            if (nonMasterControlMode == YtaTalon::FOLLOW)
            {
                m_pMotorsInfo[i]->m_pTalon->Set(ControlMode::Follower, masterCanId);
            }
        }
        
        // Override to always coast
        m_pMotorsInfo[i]->m_pTalon->SetNeutralMode(NeutralMode::Coast);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::AddMotorToGroup
///
/// Method to add a new motor to a motor group.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
bool TalonMotorGroup<TalonType>::AddMotorToGroup(MotorGroupControlMode controlMode)
{
    bool bResult = false;

    // Make sure there's room for another motor in this group
    if (m_NumMotors < MAX_NUMBER_OF_MOTORS)
    {
        // The new motor CAN ID is the first motor's ID + current number of group motors present
        int newMotorCanId = m_pMotorsInfo[0]->m_CanId + m_NumMotors;

        // m_NumMotors can be leveraged as the index, as it represents the next unused array element
        m_pMotorsInfo[m_NumMotors] = new MotorInfo(controlMode, newMotorCanId);
        
        // If this Talon will be a follower, be sure to call Set() to enable it
        if (controlMode == YtaTalon::FOLLOW)
        {
            m_pMotorsInfo[m_NumMotors]->m_pTalon->Set(ControlMode::Follower, m_MasterCanId);
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
bool TalonMotorGroup<TalonType>::SetMotorInGroupControlMode(int canId, MotorGroupControlMode controlMode)
{
    bool bResult = false;
    
    // Search for the correct motor in the group
    for (int i = 0; i < m_NumMotors; i++)
    {
        // If it matches...
        if (m_pMotorsInfo[i]->m_CanId == canId)
        {
            // ...set the control mode
            m_pMotorsInfo[i]->m_ControlMode = controlMode;

            // If this Talon will be a follower, be sure to call Set() to enable it
            if (controlMode == YtaTalon::FOLLOW)
            {
                m_pMotorsInfo[i]->m_pTalon->Set(ControlMode::Follower, m_MasterCanId);
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
    for (int i = 0; i < m_NumMotors; i++)
    {
        m_pMotorsInfo[i]->m_pTalon->SetNeutralMode(NeutralMode::Coast);
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
    for (int i = 0; i < m_NumMotors; i++)
    {
        m_pMotorsInfo[i]->m_pTalon->SetNeutralMode(NeutralMode::Brake);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::TareEncoder
///
/// Method to tare the value on an encoder feedback device
/// connected to a Talon controller.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::TareEncoder()
{
    if (m_Sensor == FeedbackDevice::CTRE_MagEncoder_Relative)
    {
        // sensorPos, pidIdx, timeoutMs
        m_pMotorsInfo[0]->m_pTalon->SetSelectedSensorPosition(0, 0, 0);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::GetEncoderValue
///
/// Method to get the value from an encoder feedback device
/// connected to a Talon controller.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
int TalonMotorGroup<TalonType>::GetEncoderValue()
{
    int sensorValue = 0;

    if (m_Sensor == FeedbackDevice::CTRE_MagEncoder_Relative)
    {
        // pidIdx
        sensorValue = m_pMotorsInfo[0]->m_pTalon->GetSelectedSensorPosition(0);
    }
    
    return sensorValue;
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
void TalonMotorGroup<TalonType>::Set( double value, double offset )
{
    for (int i = 0; i < m_NumMotors; i++)
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
            case YtaTalon::MASTER:
            case YtaTalon::INDEPENDENT:
            {
                // The master always gets set via percent voltage, as do
                // motors that are independently controlled (not follow or inverse).
                valueToSet = value;
                break;
            }
            case YtaTalon::FOLLOW:
            {
                // Nothing to do, motor had Set() called during object construction
                bCallSet = false;
                break;
            }
            case YtaTalon::INVERSE:
            {
                // Motor is attached to drive in opposite direction of master
                valueToSet = -value;
                break;
            }
            case YtaTalon::INDEPENDENT_OFFSET:
            {
                // The non-master motor has a different value in this case
                valueToSet = value + offset;
                break;
            }
            case YtaTalon::INVERSE_OFFSET:
            {
                // The non-master motor has a different value in this case
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
            m_pMotorsInfo[i]->m_pTalon->Set(ControlMode::PercentOutput, valueToSet);
        }
    }
}

#endif // TALONMOTORGROUP_HPP
