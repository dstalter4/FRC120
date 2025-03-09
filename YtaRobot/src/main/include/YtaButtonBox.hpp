////////////////////////////////////////////////////////////////////////////////
/// @file   YtaButtonBox.hpp
/// @author David Stalter
///
/// @details
/// A class designed to provide a button box HID device.
///
///
/// Copyright (c) 2025 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTABUTTONBOX_HPP
#define YTABUTTONBOX_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/GenericHID.h"             // for base class declaration

// C++ INCLUDES
// (none)

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class YtaButtonBox
///
/// Class that provides methods for interacting with a button
/// box.  It is derived from GenericHID.  There is no custom
/// functionality, it is a pass through to the base class.  It
/// exists for use with the YtaController template class.
///
////////////////////////////////////////////////////////////////
class YtaButtonBox : public GenericHID
{
public:

    // Constructor/destructor
    explicit YtaButtonBox(int controllerPort) : GenericHID(controllerPort) {}
    virtual ~YtaButtonBox() = default;
    
private:

    // Prevent copying/assignment
    YtaButtonBox(const YtaButtonBox&) = delete;
    YtaButtonBox& operator=(const YtaButtonBox&) = delete;
};

#endif // YTABUTTONBOX_HPP
