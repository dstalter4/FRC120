////////////////////////////////////////////////////////////////////////////////
/// @file   RoborioRioduinoSharedData.hpp
/// @author David Stalter
///
/// @details
/// This file contains shared data structures between the roboRIO and the
/// RIOduino.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef ROBORIORIODUINOSHAREDDATA_HPP
#define ROBORIORIODUINOSHAREDDATA_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
// (none)

#define PACKED __attribute__((packed))


////////////////////////////////////////////////////////////////
/// @namespace RoborioRioduinoSharedData
///
/// Namespace that contains declarations for the shared data
/// structures between the roboRIO and RIOduino.  Sizes of
/// members are deliberately set due to the differences in the
/// underlying architectures.
///
////////////////////////////////////////////////////////////////
namespace RoborioRioduinoSharedData
{
    // A type for the header/footer of I2C transactions
    typedef uint8_t HeaderFooterType;
    static const HeaderFooterType I2C_HEADER_DATA = 120; // 0x78
    static const HeaderFooterType I2C_FOOTER_DATA = 021; // 0x15
    
    // The roboRIO and RIOduino talk over I2C at this address
    static const int I2C_DEVICE_ADDRESS = 4U;
    
    // A constant indicating the max I2C transaction size that can occur
    static const unsigned int I2C_MAX_TRANSACTION_SIZE = 16U;
    
    // There are two types of transfers:
    // 1. Commands (master to slave, i.e. roboRIO to RIOduino)
    // 2. Data Selection (slave to master, i.e. RIOduino to roboRIO)
    
    
    ////////////////////////////////////////////////////////////////
    /// COMMANDS
    ///
    /// Layout:
    /// |   0    |    1    |  2   |  3   |      4-6          7    |
    /// | Header | Command | arg0 | arg1 | Unused buffer | Footer |
    ///
    ////////////////////////////////////////////////////////////////
    
    // Controls which data structure is active in a transfer
    enum I2cCommandSelection : uint8_t
    {
        GYRO_READ_NEW_CENTER = 1
    };
    
    // Contains (optional) parameters that can se sent with the command
    struct I2cCommandParameters
    {
      // This should be reworked to more specific types/parameters when needed
      uint8_t m_Arg0;
      uint8_t m_Arg1;
    };
    
    // Represents the I2C command that will be transferred
    struct I2cCommand
    {
        static const uint8_t I2C_COMMAND_EXPECTED_SIZE_BYTES = 8U;
        static const uint8_t I2C_COMMAND_BUFFER_SIZE_BYTES = I2C_COMMAND_EXPECTED_SIZE_BYTES - (2 * sizeof(HeaderFooterType)) - sizeof(I2cCommandSelection);
        
        // Represents different possible ways of interpreting the data buffer
        union PACKED DataBuffer
        {
            I2cCommandParameters  m_CommandParameters;
            uint8_t               m_RawBuffer[I2C_COMMAND_BUFFER_SIZE_BYTES];
        };
        
        // Make sure the compiler created the shared buffer correctly
        static_assert(sizeof(DataBuffer) == I2C_COMMAND_BUFFER_SIZE_BYTES, "I2C command buffer wrong size.");
        
        // Make sure the overlayed data structures fit in the space set aside
        static_assert(sizeof(I2cCommandParameters) <= I2C_COMMAND_BUFFER_SIZE_BYTES, "I2C command buffer not big enough for parameters.");
        
        // The members of the I2C transfer data structure
        HeaderFooterType    m_Header;
        I2cCommandSelection m_CommandSelection;
        DataBuffer          m_CommandBuffer;
        HeaderFooterType    m_Footer;
    };
    
    // Make sure the overall data structure has the correct size
    static_assert(sizeof(I2cCommand) == I2cCommand::I2C_COMMAND_EXPECTED_SIZE_BYTES, "I2C command does not equal expected size.");
    static_assert(sizeof(I2cCommand) <= I2C_MAX_TRANSACTION_SIZE, "I2C command exceeds maximum allowed size.");
    
    
    ////////////////////////////////////////////////////////////////
    /// DATA SELECTIONS
    ///
    /// Layout:
    /// |   0    |     1     |       2-10      |   11   |
    /// | Header | Selection | Response buffer | Footer |
    ///
    /// Sonar buffer:
    /// |   2   |   3   |  4   |  5   |  6   |  7   |  8   |  9   |
    /// | frntA | frntB | lftA | lftB | bckA | bckB | rgtA | rgtB |
    ///
    /// Gyro buffer:
    /// |  2   |  3-4 |  5   |  6-7 |  8   | 9-10 |
    /// | xNeg | xAng | yNeg | yAng | zNeg | zAng |
    ///
    ////////////////////////////////////////////////////////////////
    
    // Controls which data structure is active in a transfer
    enum I2cDataSelection : uint8_t
    {
        SONAR_DATA = 1,
        GYRO_DATA = 2
    };
    
    // Data structure for transferring sonar data
    struct SonarI2cData
    {
        struct Distances
        {
            uint8_t m_SonarA;
            uint8_t m_SonarB;
        };

        Distances m_FrontDistances;
        Distances m_LeftDistances;
        Distances m_BackDistances;
        Distances m_RightDistances;
    };
    
    // Data structure for transferring BNO055 gyro data
    struct GyroI2cData
    {
        struct AxisInfo
        {
          // Using uint16_t as the roboRIO compiler is padding uint8_t
          uint16_t m_bIsNegative;
          uint16_t m_Angle;
        };
        
        AxisInfo m_xAxisInfo;
        AxisInfo m_yAxisInfo;
        AxisInfo m_zAxisInfo;
        
        static const uint16_t MAX_VALID_ANGLE_VALUE = 360U;
    };
    
    // Represents the I2C data that will be transferred
    struct I2cData
    {
        // From debugging, it looks like I2C transactions should be limited to low sizes
        static const uint8_t I2C_DATA_EXPECTED_SIZE_BYTES = 16U;
        static const uint8_t I2C_DATA_BUFFER_SIZE_BYTES = I2C_DATA_EXPECTED_SIZE_BYTES - (2 * sizeof(HeaderFooterType)) - sizeof(I2cDataSelection);
        
        // Represents different possible ways of interpreting the data buffer
        union PACKED DataBuffer
        {
            SonarI2cData    m_SonarData;
            GyroI2cData     m_GyroData;
            uint8_t         m_RawBuffer[I2C_DATA_BUFFER_SIZE_BYTES];
        };
        
        // Make sure the compiler created the shared buffer correctly
        static_assert(sizeof(DataBuffer) == I2C_DATA_BUFFER_SIZE_BYTES, "I2C data buffer wrong size.");
        
        // Make sure the overlayed data structures fit in the space set aside
        static_assert(sizeof(SonarI2cData) <= I2C_DATA_BUFFER_SIZE_BYTES, "I2C data buffer not big enough for sonar data.");
        static_assert(sizeof(GyroI2cData)  <= I2C_DATA_BUFFER_SIZE_BYTES, "I2C data buffer not big enough for gyro data.");
        
        // The members of the I2C transfer data structure
        HeaderFooterType    m_Header;
        I2cDataSelection    m_DataSelection;
        DataBuffer          m_DataBuffer;
        HeaderFooterType    m_Footer;
    };
    
    // Make sure the overall data structure has the correct size
    static_assert(sizeof(I2cData) == I2cData::I2C_DATA_EXPECTED_SIZE_BYTES, "I2C data does not equal expected size.");
    static_assert(sizeof(I2cData) <= I2C_MAX_TRANSACTION_SIZE, "I2C data exceeds maximum allowed size.");
}

#endif // ROBORIORIODUINOSHAREDDATA_HPP
