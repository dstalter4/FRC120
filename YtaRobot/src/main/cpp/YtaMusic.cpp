////////////////////////////////////////////////////////////////////////////////
/// @file   YtaLed.cpp
/// @author David Stalter
///
/// @details
/// Implements functionality for playing music on a robot.
///
/// Copyright (c) 2026 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/Timer.h"              // for Timer class

// C++ INCLUDES
#include "YtaMusic.hpp"             // class declaration

using namespace frc;


////////////////////////////////////////////////////////////////
/// @method YtaMusicController::MusicSequence
///
/// This method contains the main workflow for controlling
/// any hardware capable of playing music (e.g. TalonFX).
/// Returns true if tones are being played, false if all tones
/// in the sequence have been played.
///
////////////////////////////////////////////////////////////////
bool YtaMusicController::PlayTones(TalonFX * pTalonFx)
{
    // Note: The control mode for the motors can only be one
    //       thing at a time.  Using a motor for acutal motion
    //       will not work at the same time as playing tones.

    // Add more of these as needed
    static const MusicTone noNote(units::frequency::hertz_t(0));
    static const MusicTone cNote(units::frequency::hertz_t(262));
    static const MusicTone dNote(units::frequency::hertz_t(294));
    static const MusicTone eNote(units::frequency::hertz_t(330));
    static const MusicTone fNote(units::frequency::hertz_t(349));
    static const MusicTone gNote(units::frequency::hertz_t(392));
    static const MusicTone aNote(units::frequency::hertz_t(440));
    static const MusicTone bNote(units::frequency::hertz_t(494));
    static const MusicTone CNote(units::frequency::hertz_t(523));

    static Timer * pMusicTimer = new Timer();
    static units::time::second_t lastNotePlayTime = 0.0_s;
    static bool bInit = false;
    static uint32_t noteIndex = 0U;

    if (!bInit)
    {
        pMusicTimer->Start();
        bInit = true;
    }

    struct NoteControl
    {
        MusicTone m_Tone;
        units::time::second_t m_LengthSeconds;
    };

    // Change the notes and the lengths in here to make a song.
    // You can add or remove or change as many you want.
    // If you need more tones (frequencies in hertz), add them up above.
    // If you want a pause, use noNote and a length.
    static NoteControl scaleNotes[] =
    {
        {noNote, 100_ms},
        {cNote, 100_ms},
        {dNote, 100_ms},
        {eNote, 100_ms},
        {fNote, 100_ms},
        {gNote, 100_ms},
        {aNote, 100_ms},
        {bNote, 100_ms},
        {CNote, 100_ms},
        {noNote, 100_ms},
    };
    static const size_t MAX_NOTE_INDEX = sizeof(scaleNotes) / sizeof (NoteControl);

    bool bMusicPlaying = true;
    if ((pMusicTimer->Get() - lastNotePlayTime) > scaleNotes[noteIndex].m_LengthSeconds)
    {
        noteIndex++;
        if (noteIndex >= MAX_NOTE_INDEX)
        {
            noteIndex = 0U;
            bMusicPlaying = false;
        }
        // Pick a motor to play a sound
        pTalonFx->SetControl(scaleNotes[noteIndex].m_Tone);
        lastNotePlayTime = pMusicTimer->Get();
    }

    return bMusicPlaying;
}
