////////////////////////////////////////////////////////////////////////////////
/// @file TalonMotorGroup.cpp
///
/// A class designed to create a group of CAN Talons working in tandem.
///
/// CMSD FRC 2016
/// Author: David Stalter
/// @Edit History
/// - dts   03-JAN-2015 Created from 2014.
/// - dts   17-JAN-2015 Ported to CAN Talons.
/// - dts   06-FEB-2015 Support for follow and inverse control.
///
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "TalonMotorGroup.hpp"      // For class declaration



////////////////////////////////////////////////////////////////
// @method TalonMotorGroup::TalonMotorGroup
///
/// Constructor.  Creates the number of motors specified on the
/// port numbers passed in.
///
////////////////////////////////////////////////////////////////
TalonMotorGroup::TalonMotorGroup( int numInstances, int firstCANId, NeutralMode neutralMode, ControlMode controlMode )
: m_pMotors()
, m_ControlMode(controlMode)
, m_pFeedbackDev(NULL)
, m_NumMotors(numInstances)
, m_EncoderCreationValues()
, m_EncoderTargetValue()
, m_EncoderDirection(FORWARD)
, m_bActionInProgress(false)
{
    // Allocate the necessary storage for all of the objects by invoking operator new[]
    // Assign the returned memory block to the first pointer in the array
    //m_pMotors[0] =  reinterpret_cast<CANTalon *>( operator new[] (numInstances * sizeof(CANTalon)) );

    // CAN Talons can be set to follow, which the motor groups
    // will do, so save off the first id as the master
    int masterId = firstCANId;

    // Loop for each motor to create
    for ( int i = 0; i < numInstances; i++ )
    {
        // Create it
        m_pMotors[i] = new CANTalon(firstCANId++);

        // Only set follow for Talon groups that will be configured
        // as such.  Otherwise just set everything to kPercentVbus.
        // Master is the first object for following, set its mode to
        // percent voltage bus.
        if ((i == 0) || (controlMode != FOLLOW))
        {
            m_pMotors[i]->SetControlMode(CANSpeedController::kPercentVbus);
        }
        // Otherwise set to follow
        else
        {
            m_pMotors[i]->SetControlMode(CANSpeedController::kFollower);
            m_pMotors[i]->Set(masterId);
        }

        m_pMotors[i]->ConfigNeutralMode(neutralMode);
    }
}



void TalonMotorGroup::CreateEncoderFeedbackDevice(FeedbackDevice feedbackDev)
{    
    FeedbackDeviceStatus status = FeedbackDeviceStatus::FeedbackStatusUnknown;
    
    for ( int i = 0; i < m_NumMotors; i++ )
    {
        // Make sure the sensor is present
        status = m_pMotors[i]->IsSensorPresent(feedbackDev);
        
        if (status != FeedbackDeviceStatus::FeedbackStatusPresent)
        {
            break;
        }
        
        // Save off its original position
        m_EncoderCreationValues[i] = m_pMotors[i]->GetPulseWidthPosition();
        
        // Set the sensor type and start it at zero
        m_pMotors[i]->SetFeedbackDevice(feedbackDev);
        m_pMotors[i]->SetPulseWidthPosition(0);
    }
    
    // Only create the pointer if all the expected sensors are present.
    // The loop will exit early if one fails.
    if (status == FeedbackDeviceStatus::FeedbackStatusPresent)
    {
        m_pFeedbackDev = new FeedbackDevice(feedbackDev);
    }
}

void TalonMotorGroup::TareEncoder()
{
    if (m_pFeedbackDev == NULL)
    {
        return;
    }
    
    for ( int i = 0; i < m_NumMotors; i++ )
    {
        m_pMotors[i]->SetPulseWidthPosition(0);
    }
}

void TalonMotorGroup::StartEncoderMove(int targetValue)//, Direction direction)
{
    /*
    Timer * pTimer = new Timer();
    pTimer->Reset();
    pTimer->Start();
    int currentPosition = m_pMotors[0]->GetPulseWidthPosition();
    
    // Down is positive
    if (currentPosition < targetValue)
    {
        Set(.5F);
        while (m_pMotors[0]->GetPulseWidthPosition() < targetValue && pTimer->Get() < 3.0F) {}
    }
    else if (currentPosition > targetValue)
    {
        Set(-.5F);
        while (m_pMotors[0]->GetPulseWidthPosition() > targetValue && pTimer->Get() < 3.0F) {}
    }
    else
    {
    }
    
    Set(0.0F);
    pTimer->Stop();
    delete pTimer;
    */
    /*
    // Make sure an action isn't ongoing
    if (m_bActionInProgress)
    {
        return;
    }
    
    // Save off destination position
    m_EncoderTargetValue = targetValue;
    m_EncoderDirection = direction;
    
    if (direction == FORWARD)
    {
        Set(ENCODER_AUTO_MOVE_SPEED);
    }
    else
    {
        Set(-ENCODER_AUTO_MOVE_SPEED);
    }
    
    m_bActionInProgress = true;
    */
}

void TalonMotorGroup::EncoderSequence(bool bCancelMove)
{
    /*
    // A device needs to be registered
    if (m_pFeedbackDev == NULL)
    {
        return;
    }
    
    switch (*m_pFeedbackDev)
    {
        case FeedbackDevice::CtreMagEncoder_Absolute:
        {
            if (m_bActionInProgress)
            {            
                int sensorVals[m_NumMotors] = {0};
                for ( int i = 0; i < m_NumMotors; i++ )
                {
                    sensorVals[i] = m_pMotors[i]->GetPulseWidthPosition();
                    
                    // 2016 - First encoder is backward
                    if (i == 0)
                    {
                        sensorVals[i] *= -1;
                    }
                }
                
                if (  m_EncoderDirection == FORWARD
                   && (sensorVals[0] >= m_EncoderTargetValue || sensorVals[1] >= m_EncoderTargetValue))
                {
                    Set(0.0F);
                    m_bActionInProgress = false;
                }
                else if (  m_EncoderDirection == REVERSE
                        && (sensorVals[0] <= m_EncoderTargetValue || sensorVals[1] <= m_EncoderTargetValue))
                {
                    Set(0.0F);
                    m_bActionInProgress = false;
                }
                else
                {
                }
                
                int encoderDifference = sensorVals[0] - sensorVals[1];
                if (encoderDifference <= 0)
                {
                    encoderDifference *= -1;
                }
                
                if (encoderDifference >= ENCODER_SLOP_VALUE || bCancelMove)
                {
                    Set(0.0F);
                    m_bActionInProgress = false;
                }
            }
            break;
        }
        default:
            break;
    };
    */
}




////////////////////////////////////////////////////////////////
// @method TalonMotorGroup::SetSpeed
///
/// Method to set the speed of each motor in the group.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::Set( float value )
{
    // Check what kind of group this is.  Most
    // CAN Talons will be set to follow, but some
    // may be independent (such as if they need
    // to drive in different directions).
    switch (m_ControlMode)
    {
        // Typical case, just update the master
        case FOLLOW:
        {
            m_pMotors[0]->Set(value);
            break;
        }
        // Motors are attached to drive in
        // opposite directions
        case INVERSE:
        {
            // Assumes each half of motors need to go the same direction
            // (i.e. 1:n motors, 1:n/2 forward, n/2:n reverse
            for (int i = 0; i < m_NumMotors / 2; i++)
            {
               m_pMotors[i]->Set(value);
            }
            for (int i = m_NumMotors / 2; i < m_NumMotors; i++)
            {
                   m_pMotors[i]->Set(-value);
            }
            break;
        }
        case INDEPENDENT:
        {
            for (int i = 0; i < m_NumMotors; i++)
            {
                m_pMotors[i]->Set(value);
            }
        }
        default:
            break;
    };
}
