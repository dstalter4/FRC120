////////////////////////////////////////////////////////////////////////////////
/// @file   YtaLed.cpp
/// @author David Stalter
///
/// @details
/// Implements functionality for controlling LEDs on a robot.
///
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaLed.hpp"           // class declaration


////////////////////////////////////////////////////////////////
/// @method YtaLedController::YtaLedController
///
/// Constructor
///
////////////////////////////////////////////////////////////////
YtaLedController::YtaLedController(uint32_t numLeds, int candleCanId, const CANBus & rCandleCanBus) :
    m_AllianceColor(),
    m_pCandle(new CANdle(candleCanId, rCandleCanBus)),
    m_LedStripSolidColor(0, (numLeds - 1)),
    m_RainbowAnimation (0, (numLeds - 1))
{
    // Alliance color must manually be set later because
    // it may not be known when constructors run.

    CANdleConfiguration candleConfig;
    candleConfig.LED.StripType = StripTypeValue::RGBW;
    m_pCandle->GetConfigurator().Apply(candleConfig);

    // Default behavior at construction is the rainbow animation
    m_pCandle->SetControl(m_RainbowAnimation);
}



////////////////////////////////////////////////////////////////
/// @method YtaLedController::SetAnimation
///
/// This method will set the LED strip to a specified animation.
///
////////////////////////////////////////////////////////////////
void YtaLedController::SetAnimation(LedAnimation ledAnimation)
{
    switch (ledAnimation)
    {
        case LedAnimation::LED_RAINBOW_ANIMATION:
        {
            m_pCandle->SetControl(m_RainbowAnimation);
            break;
        }
        default:
        {
            break;
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaLedController::MarioKartLights
///
/// This method will generate LED behavior that mimicks
/// drifting in Mario Kart.  It watches for non-zero rotational
/// inputs while driving/strafing.
///
////////////////////////////////////////////////////////////////
void YtaLedController::MarioKartLights(double translation, double strafe, double rotate)
{
    enum DriftState
    {
        DRIFT_OFF,
        DRIFT_BLUE,
        DRIFT_YELLOW,
        DRIFT_PURPLE,
        DRIFT_DISABLED,
        NUM_DRIFT_STATES
    };

    static DriftState driftState = DRIFT_OFF;
    static Timer * pDriftTimer = new Timer();
    static units::second_t lastTimeStamp = 0.0_s;
    static bool bLastDriftValue = false;
    static const double MIN_TRANSLATION_OR_STRAFE_VALUE = 0.25;

    // First check if the robot is moving in a way that qualifies for "drift"
    bool bDrifting = false;
    if (((std::abs(translation) > MIN_TRANSLATION_OR_STRAFE_VALUE) || (std::abs(strafe) > MIN_TRANSLATION_OR_STRAFE_VALUE)) && (std::abs(rotate) > 0.0))
    {
        bDrifting = true;
    }

    // See if there was a state change in drift status
    if (bDrifting != bLastDriftValue)
    {
        // Now drifting, previously were not
        if (bDrifting)
        {
            // Start the timer, clear the LEDs
            pDriftTimer->Start();
            lastTimeStamp = pDriftTimer->Get();
            m_pCandle->SetControl(m_LedStripSolidColor.WithColor(RGBW_OFF));
        }
        // Not drifting, previously were
        else
        {
            // Stop the timer, turn the LEDs back on
            pDriftTimer->Stop();
            pDriftTimer->Reset();
            driftState = DRIFT_OFF;
            SetLedsToAllianceColor();
        }
        bLastDriftValue = bDrifting;
    }

    // B: {132, 132, 255}
    // Y: {255, 240, 0}
    // P: {240, 73, 241}
    const RGBWColor MARIO_KART_LED_COLORS[NUM_DRIFT_STATES] =
    {
        {   0,   0,   0,   0},
        { 132, 132, 255,   0},
        { 255, 240,   0,   0},
        { 240,  73, 241,   0},
        {   0,   0,   0,   0}
    };

    // Light up the LEDs based on state
    if (bDrifting)
    {
        units::second_t currentTimeStamp = pDriftTimer->Get();
        switch (driftState)
        {
            case DRIFT_OFF:
            {
                // Transition to blue (total time 0.5 seconds)
                if ((currentTimeStamp - lastTimeStamp) > 0.5_s)
                {
                    m_pCandle->SetControl(m_LedStripSolidColor.WithColor(MARIO_KART_LED_COLORS[DRIFT_BLUE]));
                    driftState = DRIFT_BLUE;
                    lastTimeStamp = currentTimeStamp;
                }
                break;
            }
            case DRIFT_BLUE:
            {
                // Transition to yellow (total time 1.5 seconds)
                if ((currentTimeStamp - lastTimeStamp) > 1.0_s)
                {
                    m_pCandle->SetControl(m_LedStripSolidColor.WithColor(MARIO_KART_LED_COLORS[DRIFT_YELLOW]));
                    driftState = DRIFT_YELLOW;
                    lastTimeStamp = currentTimeStamp;
                }
                break;
            }
            case DRIFT_YELLOW:
            {
                // Transition to purple (total time 2.5 seconds)
                if ((currentTimeStamp - lastTimeStamp) > 1.0_s)
                {
                    m_pCandle->SetControl(m_LedStripSolidColor.WithColor(MARIO_KART_LED_COLORS[DRIFT_PURPLE]));
                    driftState = DRIFT_PURPLE;
                    lastTimeStamp = currentTimeStamp;
                }
                break;
            }
            case DRIFT_PURPLE:
            {
                // @todo: Implement some kind of 'burst' pattern
                driftState = DRIFT_DISABLED;
                break;
            }
            case DRIFT_DISABLED:
            default:
            {
                break;
            }
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaLedController::BlinkMorseCodePattern
///
/// This method contains the main workflow for blinking a Morse
/// code pattern through the robot LEDs.
///
////////////////////////////////////////////////////////////////
void YtaLedController::BlinkMorseCodePattern()
{
    enum MorseCodeSignal
    {
        END_MARKER,
        DOT,
        DASH,
        EMPTY,
        INVALID
    };

    // Characters include the terminating break.
    // The word break only contains four empty signals because
    // the characters always end with the first three empty signals.
    constexpr const MorseCodeSignal MORSE_A[] = {DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_B[] = {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_C[] = {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_D[] = {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_E[] = {DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_F[] = {DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_G[] = {DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_H[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_I[] = {DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_J[] = {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_K[] = {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_L[] = {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_M[] = {DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_N[] = {DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_O[] = {DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_P[] = {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_Q[] = {DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_R[] = {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_S[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_T[] = {DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_U[] = {DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_V[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_W[] = {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_X[] = {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_Y[] = {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_Z[] = {DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_0[] = {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_1[] = {DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_2[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_3[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_4[] = {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_5[] = {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_6[] = {DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_7[] = {DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_8[] = {DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_9[] = {DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_PERIOD[] =            {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_COMMA[] =             {DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_QUESTION_MARK[] =     {DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_SINGLE_QUOTE[] =      {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_FORWARD_SLASH[] =     {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_OPEN_PARENTHESIS[] =  {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_CLOSE_PARENTHESIS[] = {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_COLON[] =             {DASH, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_EQUAL[] =             {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_PLUS[] =              {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_HYPHEN[] =            {DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_DOUBLE_QUOTE[] =      {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_AT[] =                {DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_EXCLAMATION[] =       {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_AMPERSAND[] =         {DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_SEMICOLON[] =         {DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_UNDERSCORE[] =        {DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DASH, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_DOLLAR_SIGN[] =       {DOT, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, DOT, EMPTY, DOT, EMPTY, DASH, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_WORD_BREAK[] =        {EMPTY, EMPTY, EMPTY, EMPTY, END_MARKER};
    constexpr const MorseCodeSignal MORSE_MESSAGE_END[] =       {END_MARKER};
    constexpr const MorseCodeSignal MORSE_INVALID[] =           {INVALID};

    // This table is kept in the same order as the ASCII table to facilitate easy conversion/indexing
    const MorseCodeSignal * MORSE_SIGNALS[] = {
                                                // 0 - 31
                                                MORSE_MESSAGE_END, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID,
                                                MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID,
                                                MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID,
                                                MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID,
                                                // 32 - 63
                                                MORSE_WORD_BREAK, MORSE_EXCLAMATION, MORSE_DOUBLE_QUOTE, MORSE_INVALID, MORSE_DOLLAR_SIGN, MORSE_INVALID, MORSE_AMPERSAND, MORSE_SINGLE_QUOTE,
                                                MORSE_OPEN_PARENTHESIS, MORSE_CLOSE_PARENTHESIS, MORSE_INVALID, MORSE_PLUS, MORSE_COMMA, MORSE_HYPHEN, MORSE_PERIOD, MORSE_FORWARD_SLASH,
                                                MORSE_0, MORSE_1, MORSE_2, MORSE_3, MORSE_4, MORSE_5, MORSE_6, MORSE_7,
                                                MORSE_8, MORSE_9, MORSE_COLON, MORSE_SEMICOLON, MORSE_INVALID, MORSE_EQUAL, MORSE_INVALID, MORSE_QUESTION_MARK,
                                                // 64 - 95
                                                MORSE_AT, MORSE_A, MORSE_B, MORSE_C, MORSE_D, MORSE_E, MORSE_F, MORSE_G,
                                                MORSE_H, MORSE_I, MORSE_J, MORSE_K, MORSE_L, MORSE_M, MORSE_N, MORSE_O,
                                                MORSE_P, MORSE_Q, MORSE_R, MORSE_S, MORSE_T, MORSE_U, MORSE_V, MORSE_W,
                                                MORSE_X, MORSE_Y, MORSE_Z, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_UNDERSCORE,
                                                // 96-127
                                                MORSE_INVALID, MORSE_A, MORSE_B, MORSE_C, MORSE_D, MORSE_E, MORSE_F, MORSE_G,
                                                MORSE_H, MORSE_I, MORSE_J, MORSE_K, MORSE_L, MORSE_M, MORSE_N, MORSE_O,
                                                MORSE_P, MORSE_Q, MORSE_R, MORSE_S, MORSE_T, MORSE_U, MORSE_V, MORSE_W,
                                                MORSE_X, MORSE_Y, MORSE_Z, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID, MORSE_INVALID
                                              };

    // Old approach commented out (uses a constant variable length array of pointers instead of fixed length array/string conversion).
    //static const MorseCodeSignal * const MORSE_MESSAGE[] = {MORSE_S, MORSE_O, MORSE_S, MORSE_WORD_BREAK, MORSE_MESSAGE_END};
    static const size_t MORSE_MSG_MAX_LENGTH = 128U;
    static const MorseCodeSignal * MORSE_MESSAGE[MORSE_MSG_MAX_LENGTH] = {};
    static const char MORSE_STRING[] = "120";
    static const size_t MORSE_STRING_SIZE = (sizeof(MORSE_STRING) / sizeof(MORSE_STRING[0]));
    static_assert((MORSE_STRING_SIZE <= MORSE_MSG_MAX_LENGTH), "Morse message is too long!");

    static Timer * pMorseTimer = new Timer();
    static bool bInit = false;

    // Perform one time initialization logic
    if (!bInit)
    {
        // Build the morse message by converting it from the human readable string
        size_t messageOutputPosition = 0U;
        for (size_t i = 0U; i < MORSE_STRING_SIZE; i++)
        {
            // Convert the character to its integer representation
            uint8_t charVal = static_cast<uint8_t>(MORSE_STRING[i]);
            const uint8_t NUM_ASCII_TABLE_ENTRIES = 128U;

            // Make sure the index is in range
            if (charVal >= NUM_ASCII_TABLE_ENTRIES)
            {
                // A not basic ASCII value was found at the current character position, move on
                continue;
            }

            // Get the character's Morse code signal
            const MorseCodeSignal * pThisCharacterMorseSignal = MORSE_SIGNALS[charVal];

            // If the Morse character is valid, add it to the message
            if (pThisCharacterMorseSignal[0] != INVALID)
            {
                // The only character with end marker first is the end of message marker.
                // End of message also needs an end of word inserted, so we have to manually
                // handle that here.
                if (pThisCharacterMorseSignal[0] == END_MARKER)
                {
                    MORSE_MESSAGE[messageOutputPosition++] = MORSE_WORD_BREAK;
                }
                MORSE_MESSAGE[messageOutputPosition++] = pThisCharacterMorseSignal;
            }
        }

        // Start with the LEDs off
        m_pCandle->SetControl(m_LedStripSolidColor.WithColor(RGBW_OFF));

        // Start the timer
        pMorseTimer->Reset();
        pMorseTimer->Start();

        bInit = true;
    }

    // Deliberately start this index as all Fs so the initial state change rolls over.
    static uint32_t currentCharacterSignalIndex = 0xFFFFFFFFU;
    static uint32_t currentCharacterIndex = 0U;
    static const MorseCodeSignal * pCurrentMorseCharacter = MORSE_MESSAGE[0];

    // Signal display and time variables
    constexpr const units::time::second_t SIGNAL_DISPLAY_TIME_UNIT_S = 0.25_s;
    constexpr const units::time::second_t SIGNAL_DISPLAY_TIME_DOT = SIGNAL_DISPLAY_TIME_UNIT_S;
    constexpr const units::time::second_t SIGNAL_DISPLAY_TIME_DASH = SIGNAL_DISPLAY_TIME_UNIT_S * 3.0;
    static units::time::second_t currentSignalDisplayLengthSeconds = 1.0_s;

    // Check if a signal change is required
    if (pMorseTimer->Get() > currentSignalDisplayLengthSeconds)
    {
        // Move on to the next signal
        currentCharacterSignalIndex++;
        pMorseTimer->Reset();
    }
    else
    {
        // The timer has not reached a point for a change, do nothing different
        return;
    }

    // If we make it here, a state change is needed

    // Examine the next signal
    bool bLedsOn = false;
    switch (pCurrentMorseCharacter[currentCharacterSignalIndex])
    {
        case DOT:
        {
            currentSignalDisplayLengthSeconds = SIGNAL_DISPLAY_TIME_DOT;
            bLedsOn = true;
            break;
        }
        case DASH:
        {
            currentSignalDisplayLengthSeconds = SIGNAL_DISPLAY_TIME_DASH;
            bLedsOn = true;
            break;
        }
        case EMPTY:
        {
            currentSignalDisplayLengthSeconds = SIGNAL_DISPLAY_TIME_UNIT_S;
            bLedsOn = false;
            break;
        }
        // At the end of the current character signals
        case END_MARKER:
        default:
        {
            currentSignalDisplayLengthSeconds = 0.0_s;
            bLedsOn = false;

            // Move to the next character
            pCurrentMorseCharacter = MORSE_MESSAGE[++currentCharacterIndex];
            currentCharacterSignalIndex = 0xFFFFFFFFU;

            // Check if the next character is actually the end of message marker
            if (pCurrentMorseCharacter[0] == END_MARKER)
            {
                // Back to the start of the message
                currentCharacterIndex = 0U;
                pCurrentMorseCharacter = MORSE_MESSAGE[0];
            }

            break;
        }
    }

    // Update the state of the LEDs
    if (bLedsOn)
    {
        if (m_AllianceColor == DriverStation::Alliance::kRed)
        {
            constexpr const RGBWColor RGBW_RED{255, 0, 0, 0};
            m_pCandle->SetControl(m_LedStripSolidColor.WithColor(RGBW_RED));
        }
        else
        {
            constexpr const RGBWColor RGBW_BLUE{0, 0, 255, 0};
            m_pCandle->SetControl(m_LedStripSolidColor.WithColor(RGBW_BLUE));
        }
    }
    else
    {
        m_pCandle->SetControl(m_LedStripSolidColor.WithColor(RGBW_OFF));
    }
}
