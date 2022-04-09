#include <MIDI.h>
#include <MIDIUSB.h>
#include "MidiBaby.h"
#include <GyverButton.h>
#include <ExpressionPedal.h>
#include <EEPROM.h>

#define BUTTONS_COUNT 1

#define BUTTON_DEBOUNCE 50

#define COMMAND_PER_EVENT_COUNT 4 // event per button

#define SETTINGS_EEPROM_ADDRESS 5
#define COMMANDS_EEPROM_ADDRESS 100

//Settings
Settings settings;

//Expression Pedal
ExpPedal pedal1;
//ExpPedal pedal2(A2,A3);

//Button
GButton buttons[BUTTONS_COUNT] = {(12)};

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, Midi);

Command buttonCommands[BUTTONS_COUNT][Hold + 1][COMMAND_PER_EVENT_COUNT]; //[button count][button event count][event command count]

void setup()
{
  //EEPROM;
  EEPROM.get(SETTINGS_EEPROM_ADDRESS, settings);
  EEPROM.get(COMMANDS_EEPROM_ADDRESS, buttonCommands);

  //interfaces
  Serial.begin(115200);
  Midi.begin();

  //buttons_setup
  for (uint8_t i = 0; i < BUTTONS_COUNT; i++)
  {
    buttons[i].setDebounce(BUTTON_DEBOUNCE);
    buttons[i].setTimeout(settings.buttonHoldTimeout * 10);
    buttons[i].setClickTimeout(settings.buttonClickTimeout * 10);
    buttons[i].setType(HIGH_PULL);
    buttons[i].setDirection(NORM_OPEN);
  }

  // delay(1200);
  // for (uint8_t j = 0; j < COMMAND_PER_EVENT_COUNT; j++)
  // {
  //   Serial.println(settings.buttonHoldTimeout * 10);
  //  Serial.println(settings.buttonClickTimeout * 10);
  //   Serial.println(buttonCommands[0][0][j].channel, HEX);
  //   Serial.println(buttonCommands[0][0][j].data1, HEX);
  //   Serial.println(buttonCommands[0][0][j].data2, HEX);
  // }
}

void loop()
{
  //Pedal1 loop
  if (settings.allowExp1)
  {
    pedal1.tick();
    if (pedal1.valueChanged())
    {
      Midi.sendControlChange(settings.expPedalControl1, pedal1.getMidiValue(), settings.expPedalChannel1);
      if (settings.allowMidiUsb)
        sendUsbControlChange(settings.expPedalControl1, pedal1.getMidiValue(), settings.expPedalChannel1);
    }
  }

  //Pedal2 loop
  // if (settings.allowExp2)
  // {
  //   pedal2.tick();
  //   if (pedal2.valueChanged())
  //   {
  //     Midi.sendControlChange(settings.expPedalControl2, pedal2.getMidiValue(), settings.expPedalChannel2);
  //     if (settings.allowMidiUsb)
  //       sendUsbControlChange(settings.expPedalControl2, pedal2.getMidiValue(), settings.expPedalChannel2);
  //   }
  // }

  //Button loop
  for (uint8_t i = 0; i < BUTTONS_COUNT; i++)
  {
    buttons[i].tick();

    if (buttons[i].isPress())
    {
      runButtonCommand(i, Press);
      if (settings.allowMidiUsb)
        MidiUSB.flush();
    }

    if (buttons[i].isRelease())
    {
      runButtonCommand(i, Release);
      if (settings.allowMidiUsb)
        MidiUSB.flush();
    }

    if (buttons[i].isClick())
    {
      runButtonCommand(i, Click);
      if (settings.allowMidiUsb)
        MidiUSB.flush();
    }

    if (buttons[i].isSingle())
    {
      runButtonCommand(i, Single_click);
      if (settings.allowMidiUsb)
        MidiUSB.flush();
    }

    if (buttons[i].isDouble())
    {
      runButtonCommand(i, Double_click);
      if (settings.allowMidiUsb)
        MidiUSB.flush();
    }

    if (buttons[i].isHold())
    {
      runButtonCommand(i, Hold);
      if (settings.allowMidiUsb)
        MidiUSB.flush();
    }
  }

  //Midi USB to Midi thru loop
  if (settings.allowMidiThru && settings.allowMidiUsb)
  {
    midiEventPacket_t rx;
    do
    {
      rx = MidiUSB.read();
      if (rx.header != 0)
      {
        switch (rx.byte1 >> 4)
        {
        case 0x09:
          Midi.sendNoteOn(rx.byte2, rx.byte3, (rx.byte1 ^ NoteOn) + 1);
          break;

        case 0x08:
          Midi.sendNoteOff(rx.byte2, rx.byte3, (rx.byte1 ^ NoteOff) + 1);
          break;

        case 0x0b:
          Midi.sendControlChange(rx.byte2, rx.byte3, (rx.byte1 ^ Control_change) + 1);
          break;

        case 0x0c:
          Midi.sendProgramChange(rx.byte2, (rx.byte1 ^ Program_change) + 1);
          break;

        default:
          break;
        }
      }
      if (rx.header != 0)
      {
        Serial.print("Received: ");
        Serial.print(rx.header, HEX);
        Serial.print("-");
        Serial.print(rx.byte1, HEX);
        Serial.print("-");
        Serial.print(rx.byte2, HEX);
        Serial.print("-");
        Serial.println(rx.byte3, HEX);
      }
    } while (rx.header != 0);
  }

  //Serial
  if (Serial.available() > 0)
  {

    switch (Serial.read())
    {

    case Echo:
      Serial.write(0xA0);
      break;


    // byte expPedalChannel1;
    // byte expPedalControl1;
    // byte expPedalChannel2;
    // byte expPedalControl2;
    // byte buttonHoldTimeout;
    // byte buttonClickTimeout;

    case WriteSettings:
    {
      byte body[sizeof(settings)];
      Serial.readBytes(body, sizeof(settings));

      //flags
      settings.allowMidiUsb = body[0] & 1 ? true : false;
      settings.allowMidiThru = body[0] & 2 ? true : false;
      settings.allowExp1 = body[0] & 4 ? true : false;
      settings.allowExp2 = body[0] & 8 ? true : false;

      //pedal settings
      settings.expPedalChannel1 = body[1];
      settings.expPedalControl1 = body[2];
      settings.expPedalChannel2 = body[3];
      settings.expPedalControl2 = body[4];

      //button settings
      settings.buttonHoldTimeout = body[5];
      settings.buttonClickTimeout = body[6];

      for (uint8_t i = 0; i < BUTTONS_COUNT; i++)
      {
        buttons[i].setTimeout(settings.buttonHoldTimeout * 10);
        buttons[i].setClickTimeout(settings.buttonClickTimeout * 10);
      }

      EEPROM.put(SETTINGS_EEPROM_ADDRESS, settings);
    }
    break;

    case ReadSettings:
    {
      byte buffer[sizeof(settings)];

      if (settings.allowMidiUsb) buffer[0] |= (1 << 0);
      if (settings.allowMidiThru) buffer[0] |= (1 << 1);
      if (settings.allowExp1) buffer[0] |= (1 << 2);
      if (settings.allowExp2) buffer[0] |= (1 << 3);

      buffer[1] = settings.expPedalChannel1;
      buffer[2] = settings.expPedalControl1;
      buffer[3] = settings.expPedalChannel2;
      buffer[4] = settings.expPedalControl2;
      buffer[5] = settings.buttonHoldTimeout;
      buffer[6] = settings.buttonClickTimeout;

      Serial.write(buffer,sizeof(buffer));
    }
      break;

    case WriteButtonEvent:
    {
      // 0x00, //button num             | header
      // 0x00, //press BUTTON event     |
      // 0x02, //events count           |
      // 0x01, // behavior id
      // 0xb0, // cc midi event
      // 0x01, // channel
      // 0x01, // cc num
      // 0x7F, // 127 val

      byte header[3], body[5];
      Serial.readBytes(header, 3);

      for (uint8_t i = 0; i < header[2]; i++)
      {
        Serial.readBytes(body, sizeof(Command));

        buttonCommands[header[0]][header[1]][i].id = body[0];
        buttonCommands[header[0]][header[1]][i].midiEventType = body[1];
        buttonCommands[header[0]][header[1]][i].channel = body[2];
        buttonCommands[header[0]][header[1]][i].data1 = body[3];
        buttonCommands[header[0]][header[1]][i].data2 = body[4];

        EEPROM.put(header[1] + COMMANDS_EEPROM_ADDRESS + i * sizeof(Command), buttonCommands[header[0]][header[1]][i]);

        // Serial.println("Address:" + String(header[1] + COMMANDS_EEPROM_ADDRESS + i * (int)sizeof(Command)));
        // Serial.println(buttonCommands[header[0]][header[1]][i].id, HEX);
        // Serial.println(buttonCommands[header[0]][header[1]][i].midiEventType, HEX);
        // Serial.println(buttonCommands[header[0]][header[1]][i].channel, HEX);
        // Serial.println(buttonCommands[header[0]][header[1]][i].data1, HEX);
        // Serial.println(buttonCommands[header[0]][header[1]][i].data2, HEX);
        // Serial.println("---------------------------");
      }

      for (uint8_t i = header[2]; i < COMMAND_PER_EVENT_COUNT; i++)
      {
        buttonCommands[header[0]][header[1]][i].id = 0;
        buttonCommands[header[0]][header[1]][i].midiEventType = 0;
        buttonCommands[header[0]][header[1]][i].channel = 0;
        buttonCommands[header[0]][header[1]][i].data1 = 0;
        buttonCommands[header[0]][header[1]][i].data2 = 0;

        EEPROM.put(header[1] + COMMANDS_EEPROM_ADDRESS + i * sizeof(Command), buttonCommands[header[0]][header[1]][i]);
      }
    }
    break;

    case ReadButtonEvents:
    {
      byte buttonNo = Serial.read();
      byte buffer[sizeof(Command)];
      for (uint8_t i = 0; i < Hold + 1; i++)
      {
        for (uint8_t j = 0; j < COMMAND_PER_EVENT_COUNT; j++)
        {
          buffer[0] = buttonCommands[buttonNo][i][j].id;
          buffer[1] = buttonCommands[buttonNo][i][j].midiEventType;
          buffer[2] = buttonCommands[buttonNo][i][j].channel;
          buffer[3] = buttonCommands[buttonNo][i][j].data1;
          buffer[4] = buttonCommands[buttonNo][i][j].data2;

          Serial.write(buffer, sizeof(buffer));
        }
      }
    }
    break;

    default:
      break;
    }
  }
}

void runButtonCommand(uint8_t button, uint8_t buttonEvent)
{
  for (uint8_t i = 0; i < COMMAND_PER_EVENT_COUNT; i++)
  {
    switch (buttonCommands[button][buttonEvent][i].midiEventType)
    {
    case NoteOn:
      Midi.sendNoteOn(buttonCommands[button][buttonEvent][i].data1, buttonCommands[button][buttonEvent][i].data2, buttonCommands[button][buttonEvent][i].channel);
      if (settings.allowMidiUsb)
        sendUsbNoteOn(buttonCommands[button][buttonEvent][i].data1, buttonCommands[button][buttonEvent][i].data2, buttonCommands[button][buttonEvent][i].channel);
      break;

    case NoteOff:
      Midi.sendNoteOff(buttonCommands[button][buttonEvent][i].data1, buttonCommands[button][buttonEvent][i].data2, buttonCommands[button][buttonEvent][i].channel);
      if (settings.allowMidiUsb)
        sendUsbNoteOff(buttonCommands[button][buttonEvent][i].data1, buttonCommands[button][buttonEvent][i].data2, buttonCommands[button][buttonEvent][i].channel);
      break;

    case Control_change:
      Midi.sendControlChange(buttonCommands[button][buttonEvent][i].data1, buttonCommands[button][buttonEvent][i].data2, buttonCommands[button][buttonEvent][i].channel);
      if (settings.allowMidiUsb)
        sendUsbControlChange(buttonCommands[button][buttonEvent][i].data1, buttonCommands[button][buttonEvent][i].data2, buttonCommands[button][buttonEvent][i].channel);
      break;

    case Program_change:
      Midi.sendProgramChange(buttonCommands[button][buttonEvent][i].data1, buttonCommands[button][buttonEvent][i].channel);
      if (settings.allowMidiUsb)
        sendUsbProgramChange(buttonCommands[button][buttonEvent][i].data1, buttonCommands[button][buttonEvent][i].channel);
      break;

    default:
      break;
    }
  }
}

//Midi USB Interface

void sendUsbNoteOn(uint8_t pitch, uint8_t velocity, uint8_t channel)
{
  midiEventPacket_t noteOn = {0x09, NoteOn | channel - 1, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void sendUsbNoteOff(uint8_t pitch, uint8_t velocity, uint8_t channel)
{
  midiEventPacket_t noteOff = {0x08, NoteOff | channel - 1, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

void sendUsbControlChange(uint8_t control, uint8_t value, uint8_t channel)
{
  midiEventPacket_t event = {0x0B, Control_change | channel - 1, control, value};
  MidiUSB.sendMIDI(event);
}

void sendUsbProgramChange(uint8_t program, uint8_t channel)
{
  midiEventPacket_t event = {0x0C, Program_change | channel - 1, program, 0};
  MidiUSB.sendMIDI(event);
}