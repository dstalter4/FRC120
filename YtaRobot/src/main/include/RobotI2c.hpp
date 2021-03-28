////////////////////////////////////////////////////////////////////////////////
/// @file   RobotI2c.hpp
/// @author David Stalter
///
/// @details
/// Contains declarations for interacting with and controlling I2C on the robot.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef ROBOTI2C_HPP
#define ROBOTI2C_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/DigitalInput.h"                               // for DigitalInput type
#include "frc/DigitalOutput.h"                              // for DigitalOutput type
#include "frc/I2C.h"                                        // for interacting with an I2C port

// C++ INCLUDES
#include "RobotUtils.hpp"                                   // for DisplayMessage()
#include "../../Rioduino/RoborioRioduinoSharedData.hpp"     // for shared data structures

/// CAUTION: Forcibly remove build objects if changes       ///
///          are made to the shared header!  The build      ///
///          system is not properly updating dependencies   ///
///          when there is a relative path!                 ///

using namespace frc;
using namespace RoborioRioduinoSharedData;


////////////////////////////////////////////////////////////////
/// @class RobotI2c
///
/// Class that provides methods for interacting with I2C.
///
////////////////////////////////////////////////////////////////
class RobotI2c
{
public:
    
    // The vision thread
    static void I2cThread();
    static void ManualTrigger();
    
    // Send a command via I2C
    static void SendCommand(I2cCommandSelection command);
    
    // Set the rate for how fast the thread should run
    inline static void SetThreadUpdateRate(unsigned updateRateMs);
    
    // Retrieve the I2C sonar data
    inline static SonarI2cData * GetSonarData();
    
    // Retrieve the I2C gyro data
    inline static GyroI2cData * GetGyroData();

private:
    
    // Tracks which state the thread is currently in
    enum ThreadPhase
    {
        TRIGGER_INTERRUPT,
        COLLECT_DATA,
        SEND_COMMAND,
        DELAY
    };
    
    // Build an I2C command
    static void BuildI2cCommand();
    
    // Send an I2C command
    inline static void SendI2cCommand();
    
    // Unpack the received I2C data
    static void UnpackI2cData();
    
    // Update the I2c data structures
    inline static void UpdateI2cData();

    // Constructor
    RobotI2c();

    // Destructor, copy constructor, assignment operator
    ~RobotI2c();

    RobotI2c(const RobotI2c &) = delete;
    RobotI2c & operator=(const RobotI2c &) = delete;
    
    // MEMBER VARIABLES
    
    // Digital I/O signals for I2C communication
    static DigitalOutput    m_DigitalOutputToRioduino;
    static DigitalInput     m_DigitalInputFromRioduino;
    
    // I2C transactions
    static I2cCommand       m_I2cRioduinoCommand;
    static I2cData          m_I2cRioduinoData;
    static I2C              m_I2cRioduino;
    static bool             m_bI2cDataValid;
    static bool             m_bI2cCommandReady;
    
    // Thread configuration
    static ThreadPhase      m_ThreadPhase;
    static unsigned int     m_ThreadUpdateRateMs;
    //std::lock_guard<wpi::mutex> lock(digitalI2CMXPMutex);
    
    // Counters
    static unsigned int     m_NumValidTransactions;
    static unsigned int     m_NumInvalidTransactions;
    
    static const int        ROBORIO_SIGNAL_DIO_PIN  = 8;
    static const int        RIODUINO_SIGNAL_DIO_PIN = 9;
    static const uint8_t    I2C_BUFFER_MARKER       = 0xAA;
    static const bool       DEBUG_I2C_TRANSACTIONS  = false;
    static const unsigned   DEFAULT_UPDATE_RATE_MS  = 60U;
    static const unsigned   INITIALIZING_DELAY_MS   = 20U;
};



////////////////////////////////////////////////////////////////
/// @method RobotI2c::SetThreadUpdateRate
///
/// Sets how fast the I2C thread should get new data from the
/// RIOduino.
///
////////////////////////////////////////////////////////////////
inline void RobotI2c::SetThreadUpdateRate(unsigned updateRateMs)
{
    m_ThreadUpdateRateMs = updateRateMs;
}



////////////////////////////////////////////////////////////////
/// @method RobotI2c::SendI2cCommand
///
/// Sends a I2C command to the RIOduino.
///
////////////////////////////////////////////////////////////////
inline void RobotI2c::SendI2cCommand()
{
    // Send the command
    static_cast<void>(m_I2cRioduino.WriteBulk(reinterpret_cast<uint8_t *>(&m_I2cRioduinoCommand), sizeof(I2cCommand)));
}



////////////////////////////////////////////////////////////////
/// @method RobotI2c::UpdateI2cData
///
/// Gets new I2C data from the RIOduino.
///
////////////////////////////////////////////////////////////////
inline void RobotI2c::UpdateI2cData()
{
    // Clear the buffer for new data
    std::memset(&m_I2cRioduinoData, I2C_BUFFER_MARKER, sizeof(m_I2cRioduinoData));
    
    // Get the data from the riodiuino
    static_cast<void>(m_I2cRioduino.ReadOnly(sizeof(m_I2cRioduinoData), reinterpret_cast<uint8_t *>(&m_I2cRioduinoData)));
}



////////////////////////////////////////////////////////////////
/// @method RobotI2c::GetSonarData
///
/// Returns the I2C sonar data from the RIOduino or a nullptr if
/// the data isn't valid.
///
////////////////////////////////////////////////////////////////
inline SonarI2cData * RobotI2c::GetSonarData()
{
    SonarI2cData * pSonarData = nullptr;

    if (m_I2cRioduinoData.m_DataSelection == I2cDataSelection::SONAR_DATA)
    {
        pSonarData = &m_I2cRioduinoData.m_DataBuffer.m_SonarData;
    }
    else
    {
        // Since this is an interface method, getting here means the I2C thread is in the middle
        // of an update, so there's no valid data to return (between memset and transaction calls).
    }
    
    return pSonarData;
}



////////////////////////////////////////////////////////////////
/// @method RobotI2c::GetGyroData
///
/// Returns the I2C gyro data from the RIOduino or a nullptr if
/// the data isn't valid.
///
////////////////////////////////////////////////////////////////
inline GyroI2cData * RobotI2c::GetGyroData()
{
    GyroI2cData * pGyroData = nullptr;

    if (m_I2cRioduinoData.m_DataSelection == I2cDataSelection::GYRO_DATA)
    {
        pGyroData = &m_I2cRioduinoData.m_DataBuffer.m_GyroData;
    }
    else
    {
        // Since this is an interface method, getting here means the I2C thread is in the middle
        // of an update, so there's no valid data to return (between memset and transaction calls).
    }
    
    return pGyroData;
}

#endif // ROBOTI2C_HPP
