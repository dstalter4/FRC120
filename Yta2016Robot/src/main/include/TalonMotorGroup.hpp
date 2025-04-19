////////////////////////////////////////////////////////////////////////////////
/// @file TalonMotorGroup.hpp
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

#ifndef TALONMOTORGROUP_HPP
#define TALONMOTORGROUP_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "WPILib.h"     // For FRC library declarations

// C++ INCLUDES
// (none)

class TalonMotorGroup
{
public:

    typedef CANTalon::NeutralMode NeutralMode;
    typedef CANTalon::FeedbackDevice FeedbackDevice;
    typedef CANTalon::FeedbackDeviceStatus FeedbackDeviceStatus;

    enum ControlMode
    {
        INVERSE,
        FOLLOW,
        INDEPENDENT
    };
    
    enum Direction
    {
        FORWARD,
        REVERSE
    };

    // Constructor
    TalonMotorGroup( int numInstances, int firstCANId, NeutralMode neutralMode, ControlMode controlMode );
    
    // Function to set the speed of each motor in the group
    void Set( float value );
    
    // Register a sensor type for this motor group
    void CreateEncoderFeedbackDevice(FeedbackDevice feedbackDev);
    
    // Zero out an encoder
    void TareEncoder();
    
    // Start an encoder controlled movement
    void StartEncoderMove(int targetValue);//, Direction direction);
    
    // Main flow for processing info from an encoder
    void EncoderSequence(bool bCancelMove);
    
private:
    static const int        MAX_NUMBER_OF_MOTORS    = 10;
    static const int        ENCODER_SLOP_VALUE      = 100;
    static constexpr float  ENCODER_AUTO_MOVE_SPEED = .25F;

    // Member variables
    CANTalon *  m_pMotors[MAX_NUMBER_OF_MOTORS];        // The motor objects
    ControlMode m_ControlMode;                          // Keep track of the configuration of this Talon group
    FeedbackDevice * m_pFeedbackDev;                    // Any sensor potentially connected to a Talon
    int m_NumMotors;                                    // Number of motors in the group
    int m_EncoderCreationValues[MAX_NUMBER_OF_MOTORS];  // The original value of an encoder
    int m_EncoderTargetValue;                           // Where we want to move an encoder to
    Direction m_EncoderDirection;                       // Which way the encoder is moving
    bool m_bActionInProgress;                           // Keep track of any sensor operations
    
    // Prevent default construction/deletion/copy/assignment
    TalonMotorGroup();
    ~TalonMotorGroup();
    TalonMotorGroup( const TalonMotorGroup& );
    TalonMotorGroup & operator=( const TalonMotorGroup& );
};

#endif // TALONMOTORGROUP_HPP
