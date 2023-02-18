////////////////////////////////////////////////////////////////////////////////
/// @file   DriveConfiguration.hpp
/// @author David Stalter
///
/// @details
/// Declarations describing the drive configuration.
///
/// Copyright (c) 2023 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef DRIVECONFIGURATION_HPP
#define DRIVECONFIGURATION_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
// (none)


////////////////////////////////////////////////////////////////
/// @namespace Yta::Drive::Config
///
/// Provides configuration information about the drive system.
///
////////////////////////////////////////////////////////////////
namespace Yta
{
namespace Drive
{
namespace Config
{
    static const bool               USE_SWERVE_DRIVE                        = true;
    static const bool               USE_INVERTED_REVERSE_CONTROLS           = true;
    static const bool               DRIVE_MOTOR_COOLING_ENABLED             = true;
    static const bool               DRIVE_SWAP_ENABLED                      = false;
    static const bool               SLOW_DRIVE_ENABLED                      = false;
    static const bool               DIRECTIONAL_ALIGN_ENABLED               = false;
    static const bool               DIRECTIONAL_INCH_ENABLED                = false;

    static_assert((DIRECTIONAL_ALIGN_ENABLED && DIRECTIONAL_INCH_ENABLED) != true, "Only directional align OR directional inch can be enabled.");
}
}
}

#endif // DRIVECONFIGURATION_HPP
