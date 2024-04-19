////////////////////////////////////////////////////////////////////////////////
/// @file   RobotUtils.hpp
/// @author David Stalter
///
/// @details
/// Contains declarations of utility macros/routines for the robot code.
///
/// Copyright (c) 2024 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef ROBOTUTILS_HPP
#define ROBOTUTILS_HPP

// SYSTEM INCLUDES
#include <cassert>                              // for assert()
#include <cstdarg>                              // for va_*
#include <cstdio>                               // for printf
#include <iostream>                             // for cout

// C INCLUDES
// (none)

// C++ INCLUDES
// (none)

// MACROS
#define ASSERT(condition)                                   \
    do                                                      \
    {                                                       \
        if (!(condition))                                   \
        {                                                   \
            std::cout << "Robot code ASSERT!" << std::endl; \
            std::cout << "File: " << __FILE__ << std::endl; \
            std::cout << "Line: " << __LINE__ << std::endl; \
            assert(false);                                  \
        }                                                   \
    }                                                       \
    while (false);

extern void ConstantExpressionAssert();
#define ASSERT_CONSTEXPR(condition)                         \
do                                                          \
{                                                           \
    if (!(condition))                                       \
    {                                                       \
        ConstantExpressionAssert();                         \
    }                                                       \
}                                                           \
while (false);

#define STRINGIFY(s) #s

#define DISABLE_WARNING(flag)                               \
    _Pragma("GCC diagnostic push")                          \
    _Pragma(STRINGIFY(GCC diagnostic ignored flag))

#define ENABLE_WARNING(flag)                                \
    _Pragma("GCC diagnostic pop")

////////////////////////////////////////////////////////////////
/// @namespace RobotUtils
///
/// Namespace that contains utility functions for supporting
/// robot development.
///
////////////////////////////////////////////////////////////////
namespace RobotUtils
{
    // For use with creating more advanced SmartDashboard prints
    struct DebugDigits
    {
        unsigned m_1;
        unsigned m_10;
        unsigned m_100;
        unsigned m_1000;
        unsigned m_10000;
        unsigned m_100000;
        unsigned m_1000000;
        DebugDigits() : m_1(), m_10(), m_100(), m_1000(), m_10000(), m_100000(), m_1000000() {}
        inline unsigned Value() { return (m_1 + m_10 + m_100 + m_1000 + m_10000 + m_100000 + m_1000000); }
    };

    static const bool DEBUG_PRINTS = true;
    
    ////////////////////////////////////////////////////////////////
    /// @method RobotUtils::DisplayMessage
    ///
    /// Displays a message to the RioLog as long as debug prints are
    /// enabled.
    ///
    ////////////////////////////////////////////////////////////////
    inline void DisplayMessage(const char * pMessage)
    {
        if (DEBUG_PRINTS)
        {
            std::cout << pMessage << std::endl;
        }
    }
    
    ////////////////////////////////////////////////////////////////
    /// @method RobotUtils::DisplayMessage
    ///
    /// Displays a message to the RioLog as long as debug prints are
    /// enabled.
    ///
    ////////////////////////////////////////////////////////////////
    inline void DisplayFormattedMessage(const char * pMessage, ...)
    {
        if (DEBUG_PRINTS)
        {
            va_list argPtr;
            va_start(argPtr, pMessage);
            
            // It's ok to pass the pointer here instead of a formatted
            // string since the callers of this function are well defined.
            DISABLE_WARNING("-Wformat-nonliteral")
            vprintf(pMessage, argPtr);
            ENABLE_WARNING("-Wformat-nonliteral")
            
            va_end(argPtr);
        }
    }

    ////////////////////////////////////////////////////////////////
    /// @method RobotUtils::Limit
    ///
    /// This method is used to prevent a value outside the range
    /// specified by upper and lower from being sent to physical
    /// devices.
    ///
    ////////////////////////////////////////////////////////////////
    inline double Limit( double num, double upper, double lower )
    {
        if ( num > upper )
        {
            return upper;
        }
        else if ( num < lower )
        {
            return lower;
        }

        return num;
    }

    ////////////////////////////////////////////////////////////////
    /// @method RobotUtils::Trim
    ///
    /// This method is used to ensure a signal value is above a
    /// certain threshold to ensure there is actual input to a
    /// device and not just noise/jitter.
    ///
    ////////////////////////////////////////////////////////////////
    inline double Trim( double num, double upper, double lower )
    {
        if ( (num < upper) && (num > lower) )
        {
            return 0.0;
        }
        
        return num;
    }

    ////////////////////////////////////////////////////////////////
    /// @method RobotUtils::ConvertCelsiusToFahrenheit
    ///
    /// This method converts a temperature from Celsius to
    /// Fahrenheit.
    ///
    ////////////////////////////////////////////////////////////////
    inline static constexpr double ConvertCelsiusToFahrenheit(double degreesC)
    {
        return ((degreesC * 9.0/5.0) + 32.0);
    }
    
}

#endif // ROBOTUTILS_HPP
