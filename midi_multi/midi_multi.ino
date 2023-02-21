// Using Teensyduino IDE https://www.pjrc.com/teensy/td_download.html
// https://www.pjrc.com/teensy/td_157/Teensyduino_MacOS_Catalina.zip Set Tools
// >> USB Type >> MIDI
#define TCAADDR 0x70
#include "Adafruit_VL6180X.h"
#include <Wire.h>

#define midiCh 0

#define vl1Port 0 // north
#define vl2Port 3 // south

Adafruit_VL6180X vl1 = Adafruit_VL6180X();
Adafruit_VL6180X vl2 = Adafruit_VL6180X();

uint8_t prev_range_vl1 =
    0; // store previous value of vel --> needed for derivates
uint8_t prev_range_vl2 = 0;
uint8_t dt = 50; // ms

// calibration
uint8_t range_min = 5;
uint8_t range_max = 170;
uint8_t vel_min = 0;
uint8_t vel_max = 122;
uint8_t diff_min = 0;
uint8_t diff_max = range_max - range_min;
uint8_t avg_min = 0;
uint8_t avg_max = range_max;

// lib
void vlselect(uint8_t i);
uint8_t vlbegin(Adafruit_VL6180X *vlsensor, uint8_t i);
uint8_t vlread(Adafruit_VL6180X *vlsensor, uint8_t i);
uint8_t midi_map(int range, int rangeMin, int rangeMax);

////

void setup() {
  Serial.begin(115200);

  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }

  Wire.begin();

  /* Initialise the 1st sensor */
  if (!vlbegin(&vl1, vl1Port)) {
    return;
  }
  /* Initialise the 2nd sensor */
  if (!vlbegin(&vl2, vl2Port)) {
    return;
  }
}

void loop() {

  // range
  uint8_t range_vl1 = vlread(&vl1, vl1Port);
  uint8_t range_vl2 = vlread(&vl2, vl2Port);
  uint8_t midi_range_vl1 = midi_map(range_vl1, range_min, range_max);
  uint8_t midi_range_vl2 = midi_map(range_vl2, range_min, range_max);

  //  uint8_t prev_midi_range_vl1 = midi_map(range_vl1, range_min, range_max);
  //  // debug

  // velocity
  int vel_vl1 = (range_vl1 - prev_range_vl1) * 100 / dt;
  int vel_vl2 = (range_vl2 - prev_range_vl2) * 100 / dt;
  uint8_t midi_vel_vl1 = midi_map(vel_vl1, vel_min, vel_max);
  uint8_t midi_vel_vl2 = midi_map(vel_vl2, vel_min, vel_max);
  uint8_t midi_sign_vel_vl1 = midi_sign(vel_vl1);
  uint8_t midi_sign_vel_vl2 = midi_sign(vel_vl2);

  // calc

  uint8_t diff_vl1vl2 = range_vl1 - range_vl2;
  uint8_t midi_diff_vl1vl2 = midi_map(diff_vl1vl2, diff_min, diff_max);
  float avg_vl1vl2 = (range_vl1 + range_vl2) / 2; // gets cast into int
  uint8_t midi_avg_vl1vl2 = midi_map(avg_vl1vl2, avg_min, avg_max);

  //  // prints
  //  Serial.println("///////");
  //  Serial.print("range value1: ");
  //  Serial.print(range_vl1);
  //  Serial.print(", prev range 1: ");
  //  Serial.println(prev_range_vl1);
  //  Serial.print("range midi val: ");
  //  Serial.print(midi_range_vl1);
  //  Serial.print(", prev midi 1: ");
  //  Serial.println(prev_midi_range_vl1);
  //  Serial.print("range value2: ");
  //  Serial.print(range_vl2);
  //  Serial.print(", prev range 2: ");
  //  Serial.println(prev_range_vl2);
  //  Serial.print("range midi val 2: ");
  //  Serial.println(midi_range_vl2);
  //  Serial.print("vel 1: ");
  //  Serial.print(vel_vl1);
  //  Serial.print(", midi vel 1: ");
  //  Serial.print(midi_vel_vl1);
  //  Serial.print(", midi sign: ");
  //  Serial.println(midi_sign_vel_vl1);
  //  Serial.print("midi diff: ");
  //  Serial.print(midi_diff_vl1vl2);
  //  Serial.print(", midi avg: ");
  //  Serial.println(midi_avg_vl1vl2);

  // update prev
  prev_range_vl1 = range_vl1;
  prev_range_vl2 = range_vl2;

  // send midi cc (control, value, channel)
  usbMIDI.sendControlChange(0, midi_range_vl1, midiCh);
  usbMIDI.sendControlChange(1, midi_range_vl2, midiCh);
  usbMIDI.sendControlChange(2, midi_vel_vl1, midiCh);
  usbMIDI.sendControlChange(3, midi_vel_vl2, midiCh);
  usbMIDI.sendControlChange(4, midi_sign_vel_vl1, midiCh);
  usbMIDI.sendControlChange(5, midi_sign_vel_vl2, midiCh);
  usbMIDI.sendControlChange(4, midi_diff_vl1vl2, midiCh);
  usbMIDI.sendControlChange(5, midi_avg_vl1vl2, midiCh);

  delay(dt);
}

////lib

void vlselect(uint8_t i) { // select multiplexer port
  if (i > 3)
    return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

uint8_t vlbegin(Adafruit_VL6180X *vlsensor, uint8_t i) {
  vlselect(i);

  if (!vlsensor->begin()) {
    Serial.println("Failed to find sensor");
    return 0;
  }
  return 1;
}

uint8_t vlread(Adafruit_VL6180X *vlsensor, uint8_t i) {
  vlselect(i);
  return vlsensor->readRange();
}

uint8_t midi_map(int value, int range_min, int range_max) {
  uint8_t midi_min = 0;
  uint8_t midi_max = 127;

  uint8_t mapped_value =
      map(abs(value), range_min, range_max, midi_min, midi_max);
  uint8_t constrained_value = constrain(mapped_value, midi_min, midi_max);

  return constrained_value;
}

uint8_t midi_sign(int value) {
  if (value >= 0) {
    return 1; // pos
  } else {
    return 0; // neg
  }
}
