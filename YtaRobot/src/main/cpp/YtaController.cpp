////////////////////////////////////////////////////////////////////////////////
/// @file   YtaController.cpp
/// @author David Stalter
///
/// @details
/// A class designed to interface to several controller types (Logitech Gamepad,
/// Xbox GameSir) with custom responses.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/smartdashboard/SmartDashboard.h"  // for interacting with the smart dashboard

// C++ INCLUDES
#include "YtaController.hpp"                    // for class declaration
#include "RobotUtils.hpp"                       // for ASSERT, DEBUG_PRINTS

// STATIC MEMBER DATA
// (none)



////////////////////////////////////////////////////////////////
/// @method YtaController::YtaController
///
/// Constructor.
///
////////////////////////////////////////////////////////////////
YtaController::YtaController(CustomControllerType controllerType, int port, bool bIsDriveController)
: GenericHID(port)
, CONTROLLER_TYPE(controllerType)
, CONTROLLER_MAPPINGS(GetControllerMapping(controllerType))
, IS_DRIVE_CONTROLLER(bIsDriveController)
, m_ThrottleValue(1.0)
{
    ASSERT(CONTROLLER_MAPPINGS != nullptr);
}



////////////////////////////////////////////////////////////////
/// @method YtaController::GetX
///
/// Returns x-axis input.  This method is pure virtual in the
/// base class and must be implemented.
///
////////////////////////////////////////////////////////////////
double YtaController::GetX(JoystickHand hand) const
{
    // x-axis controls are very sensitive on this
    // controller, so scale them back.
    double xAxisValue = GetRawAxis(CONTROLLER_MAPPINGS->AXIS_MAPPINGS.LEFT_X_AXIS);
    
    if (IS_DRIVE_CONTROLLER)
    {
        xAxisValue *= X_AXIS_DRIVE_SENSITIVITY_SCALING;
    }

    return xAxisValue;
}



////////////////////////////////////////////////////////////////
/// @method YtaController::GetY
///
/// Returns y-axis input.  This method is pure virtual in the
/// base class and must be implemented.
///
////////////////////////////////////////////////////////////////
double YtaController::GetY(JoystickHand hand) const
{
    // In order to keep the drive logic the same across
    // all joysticks, full forward is represented by -1
    // and full reverse is represented by +1.
    
    // Left trigger is the 'reverse' value input.
    double leftTriggerValue = GetRawAxis(CONTROLLER_MAPPINGS->AXIS_MAPPINGS.LEFT_TRIGGER);
    
    // Right trigger is the 'forward' value input.
    double rightTriggerValue = GetRawAxis(CONTROLLER_MAPPINGS->AXIS_MAPPINGS.RIGHT_TRIGGER);

    if (RobotUtils::DEBUG_PRINTS)
    {
        SmartDashboard::PutNumber("Raw left trigger", leftTriggerValue);
        SmartDashboard::PutNumber("Raw right trigger", rightTriggerValue);
    }

    // Normalize (controller specific code).
    // After this, left will be 0->+1, right will be -1->0.
    NormalizeTriggers(leftTriggerValue, rightTriggerValue);

    if (RobotUtils::DEBUG_PRINTS)
    {
        SmartDashboard::PutNumber("Normalized left trigger", leftTriggerValue);
        SmartDashboard::PutNumber("Normalized right trigger", rightTriggerValue);
    }
    
    // Hopefully only one trigger is being pushed, but in
    // case both are being pressed, the value will be combined.
    double yAxisValue =  leftTriggerValue + rightTriggerValue;
    
    if (IS_DRIVE_CONTROLLER)
    {
        yAxisValue *= Y_AXIS_DRIVE_SENSITIVITY_SCALING;
    }

    return yAxisValue;
}



////////////////////////////////////////////////////////////////
/// @method YtaController::GetThrottle
///
/// Returns throttle control.  Most controllers do not have an
/// axis that retains its position when not being manipulated by
/// the user.  This requires throttle control to be implemented
/// and remembered in software.
///
////////////////////////////////////////////////////////////////
double YtaController::GetThrottle() const
{
    // Not implemented yet, just return the default value
    return m_ThrottleValue;
}
