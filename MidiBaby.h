#pragma once
#include <arduino.h>

struct Command
{
    uint8_t id;
    uint8_t midiEventType;
    uint8_t channel;
    uint8_t data1;
    uint8_t data2;
};

struct Settings
{
    bool allowMidiUsb: 1;
    bool allowMidiThru: 1;
    bool allowExp1: 1;
    bool allowExp2: 1;
    byte expPedalChannel1;
    byte expPedalControl1;
    byte expPedalChannel2;
    byte expPedalControl2;
    byte buttonHoldTimeout;
    byte buttonClickTimeout;

};

enum buttonEvent
{
    Press,
    Release,
    Click,
    Single_click,
    Double_click,
    Hold
};

enum midiEvent
{
    Nope,
    NoteOn = 0x90,
    NoteOff = 0x80,
    Control_change = 0xB0,
    Program_change = 0xC0
};

enum serialCommand
{
    Echo = 0xA0,
    WriteSettings = 0xA1,
    WriteButtonEvent = 0xA2,
    WriteAllButtonsEvents = 0xA3,
    ReadSettings = 0xA4,
    ReadButtonEvents = 0xA5
};