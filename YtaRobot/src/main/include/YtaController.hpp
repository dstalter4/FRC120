////////////////////////////////////////////////////////////////////////////////
/// @file   YtaController.hpp
/// @author David Stalter
///
/// @details
/// A class designed to interface to several controller types (Logitech Gamepad,
/// Xbox GameSir) with custom responses.
///
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTACONTROLLER_HPP
#define YTACONTROLLER_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/GenericHID.h"             // for base class declaration

// C++ INCLUDES
// (none)

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class YtaController
///
/// Class that provides methods for interacting with a Logitech
/// Gamepad controller.  Derives from GenericHID.  Inheriting
/// from GamepadBase.h is deprecated, so GenericHID is used
/// directly.
///
////////////////////////////////////////////////////////////////
class YtaController : public GenericHID
{
    
public:
    
    // Represents the possible custom controller configurations
    enum CustomControllerType
    {
        LOGITECH,
        PLAY_STATION
    };
    
    // Structure representing the mapping of axes
    struct AxisMappings
    {
        const unsigned LEFT_X_AXIS;
        const unsigned LEFT_Y_AXIS;
        const unsigned LEFT_TRIGGER;
        const unsigned RIGHT_X_AXIS;
        const unsigned RIGHT_Y_AXIS;
        const unsigned RIGHT_TRIGGER;
    };

    // Structure representing the mapping of buttons
    struct ButtonMappings
    {
        const unsigned NO_BUTTON;
        const unsigned UP_BUTTON;
        const unsigned DOWN_BUTTON;
        const unsigned LEFT_BUTTON;
        const unsigned RIGHT_BUTTON;
        const unsigned SELECT;
        const unsigned START;
        const unsigned LEFT_BUMPER;
        const unsigned RIGHT_BUMPER;
        const unsigned LEFT_STICK_CLICK;
        const unsigned RIGHT_STICK_CLICK;
    };

    // Mappings for a controller (axes and buttons)
    struct ControllerMappings
    {
        const AxisMappings AXIS_MAPPINGS;
        const ButtonMappings BUTTON_MAPPINGS;
    };
    
    // Constant expression function to retrieve the mapping for a controller at compile time
    static constexpr const ControllerMappings * GetControllerMapping(CustomControllerType controllerType)
    {
        switch (controllerType)
        {
            case LOGITECH:
            {
                return &LOGITECH_CONTROLLER_MAPPINGS;
                break;
            }
            case PLAY_STATION:
            {
                return &PLAY_STATION_CONTROLLER_MAPPINGS;
                break;
            }
            default:
            {
                return nullptr;
                break;
            }
        }
    }
    
    // Constructor/destructor
    explicit YtaController(CustomControllerType controllerType, int port, bool bIsDriveController);
    virtual ~YtaController() = default;
    
    virtual double GetX(JoystickHand hand = kLeftHand) const override;
    virtual double GetY(JoystickHand hand = kLeftHand) const override;
    
    double GetThrottle() const;
    
private:
    
    // Structure representing a Logitech controller
    struct LogitechController
    {
        // Joystick axes inputs are:
        // LT: 0->+1, RT: 0->+1
        // x-axis: -1    +1
        
        enum RawAxes
        {
            LEFT_X_AXIS         = 0,
            LEFT_Y_AXIS         = 1,
            LT                  = 2,
            RT                  = 3,
            RIGHT_X_AXIS        = 4,
            RIGHT_Y_AXIS        = 5
        };
        
        enum RawButtons
        {
            NO_BUTTON           = 0,
            A                   = 1,
            B                   = 2,
            X                   = 3,
            Y                   = 4,
            LB                  = 5,
            RB                  = 6,
            SELECT              = 7,
            START               = 8,
            LEFT_STICK_CLICK    = 9,
            RIGHT_STICK_CLICK   = 10
        };

        static void NormalizeTriggers(double & rLeftTrigger, double & rRightTrigger)
        {
            // LT: in 0->+1, out 0->+1
            // RT: in 0->+1, out -1->0
            rRightTrigger *= -1.0;
        }
    };

    // Structure representing a PlayStation controller
    struct PlayStationController
    {
        // Joystick axes inputs are:
        // L2: -1->+1, R2: +1->-1
        // x-axis: -1    +1
        
        enum RawAxes
        {
            LEFT_X_AXIS         = 0,
            LEFT_Y_AXIS         = 1,
            RIGHT_X_AXIS        = 2,
            L2                  = 3,
            R2                  = 4,
            RIGHT_Y_AXIS        = 5
        };
        
        enum RawButtons
        {
            NO_BUTTON           = 0,
            SQUARE              = 1,
            X                   = 2,
            CIRCLE              = 3,
            TRIANGLE            = 4,
            L1                  = 5,
            R1                  = 6,
            SHARE               = 9,
            OPTIONS             = 10,
            LEFT_STICK_CLICK    = 11,
            RIGHT_STICK_CLICK   = 12
        };

        static void NormalizeTriggers(double & rLeftTrigger, double & rRightTrigger)
        {
            // L2: in -1->+1, out 0->+1
            // R2: in -1->+1, out -1->0
            rLeftTrigger = (rLeftTrigger + 1.0) / 2.0;
            rRightTrigger = (rRightTrigger + 1.0) / -2.0;
        }
    };
    
    // Constant expression mapping the Logitech controller axes/buttons
    static constexpr const ControllerMappings LOGITECH_CONTROLLER_MAPPINGS =
    {
        {
            LogitechController::RawAxes::LEFT_X_AXIS,
            LogitechController::RawAxes::LEFT_Y_AXIS,
            LogitechController::RawAxes::LT,
            LogitechController::RawAxes::RIGHT_X_AXIS,
            LogitechController::RawAxes::RIGHT_Y_AXIS,
            LogitechController::RawAxes::RT
        },
        {
            LogitechController::RawButtons::NO_BUTTON,
            LogitechController::RawButtons::Y,
            LogitechController::RawButtons::A,
            LogitechController::RawButtons::X,
            LogitechController::RawButtons::B,
            LogitechController::RawButtons::SELECT,
            LogitechController::RawButtons::START,
            LogitechController::RawButtons::LB,
            LogitechController::RawButtons::RB,
            LogitechController::RawButtons::LEFT_STICK_CLICK,
            LogitechController::RawButtons::RIGHT_STICK_CLICK
        }
    };

    // Constant expression mapping the PlayStation controller axes/buttons
    static constexpr const ControllerMappings PLAY_STATION_CONTROLLER_MAPPINGS =
    {
        {
            PlayStationController::RawAxes::LEFT_X_AXIS,
            PlayStationController::RawAxes::LEFT_Y_AXIS,
            PlayStationController::RawAxes::L2,
            PlayStationController::RawAxes::RIGHT_X_AXIS,
            PlayStationController::RawAxes::RIGHT_Y_AXIS,
            PlayStationController::RawAxes::R2
        },
        {
            PlayStationController::RawButtons::NO_BUTTON,
            PlayStationController::RawButtons::TRIANGLE,
            PlayStationController::RawButtons::X,
            PlayStationController::RawButtons::SQUARE,
            PlayStationController::RawButtons::CIRCLE,
            PlayStationController::RawButtons::SHARE,
            PlayStationController::RawButtons::OPTIONS,
            PlayStationController::RawButtons::L1,
            PlayStationController::RawButtons::R1,
            PlayStationController::RawButtons::LEFT_STICK_CLICK,
            PlayStationController::RawButtons::RIGHT_STICK_CLICK,
        }
    };

    ////////////////////////////////////////////////////////////////
    /// @method YtaController::NormalizeTriggers
    ///
    /// Function to normalize the trigger inputs to expected output
    /// ranges.  The controller logic wants the left trigger to be
    /// a value between [-1:0] and the right trigger to be a value
    /// between [0:+1].  This function will invoke code specific to
    /// a controller to adjust the triggers since each controller
    /// will be unique in how the axes values are reported.
    ///
    ////////////////////////////////////////////////////////////////
    inline void NormalizeTriggers(double & rLeftTrigger, double & rRightTrigger) const
    {
        switch (CONTROLLER_TYPE)
        {
            case LOGITECH:
            {
                LogitechController::NormalizeTriggers(rLeftTrigger, rRightTrigger);
                break;
            }
            case PLAY_STATION:
            {
                PlayStationController::NormalizeTriggers(rLeftTrigger, rRightTrigger);
                break;
            }
            default:
            {
                break;
            }
        }
    }
    
    const CustomControllerType CONTROLLER_TYPE;
    const ControllerMappings * const CONTROLLER_MAPPINGS;
    const bool IS_DRIVE_CONTROLLER;
    double m_ThrottleValue;
    
    static constexpr double X_AXIS_DRIVE_SENSITIVITY_SCALING = 0.75;
    static constexpr double Y_AXIS_DRIVE_SENSITIVITY_SCALING = 1.00;
    
    // Prevent copying/assignment
    YtaController(const YtaController&) = delete;
    YtaController& operator=(const YtaController&) = delete;
};

#endif // YTACONTROLLER_HPP
