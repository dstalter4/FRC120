////////////////////////////////////////////////////////////////////////////////
/// @file   YtaMusic.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for playing music on a robot.
///
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTAMUSIC_HPP
#define YTAMUSIC_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "ctre/phoenix6/TalonFX.hpp"            // for TalonFX type

using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::hardware;


namespace Yta::Music::Config
{
    static constexpr const bool PLAYING_TONES_ENABLED = false;
}


////////////////////////////////////////////////////////////////
/// @class YtaMusicController
///
/// Declarations for managing music/tones on a robot.
///
////////////////////////////////////////////////////////////////
class YtaMusicController
{
public:
    static bool PlayTones(TalonFX * pTalonFx);
};

#endif // YTAMUSIC_HPP
