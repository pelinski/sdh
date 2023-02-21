// Using Teensyduino IDE https://www.pjrc.com/teensy/td_download.html
// https://www.pjrc.com/teensy/td_157/Teensyduino_MacOS_Catalina.zip Set Tools
// >> USB Type >> MIDI
#include "Adafruit_VL6180X.h"
#include "multi/multi.h"
#include <Wire.h>

#define vl1Port 0 
#define vl2Port 3 


Adafruit_VL6180X vl1 = Adafruit_VL6180X();
Adafruit_VL6180X vl2 = Adafruit_VL6180X();

void vlselect(uint8_t i);
uint8_t vlbegin(Adafruit_VL6180X *vlsensor, uint8_t i);
int vlread(Adafruit_VL6180X *vlsensor, uint8_t i);
uint vlmap(int range, int rangeMin, int rangeMax);

void setup() {
  Serial.begin(115200);

  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }

  Wire.begin();

  /* Initialise the 1st sensor */
  if (!vlbegin(&vl1,vl1Port)) {
    return;
  }
  /* Initialise the 2nd sensor */
  if (!vlbegin(&vl2,vl2Port)) {
    return;
  }
}

void loop() {
  uint8_t range1 = vlread(&vl1,vl1Port);
  uint8_t range2 = vlread(&vl2,vl2Port);

  int rangeMin = 5;
  int rangeMax = 170;

  int value1 = vlmap(range1, rangeMin, rangeMax);
  int value2 = vlmap(range2, rangeMin, rangeMax);

  usbMIDI.sendControlChange(0, value1, 1); // control, value, channel
  usbMIDI.sendControlChange(1, value2, 1); // control, value, channel

  Serial.print("Midi value1: ");
  Serial.println(value1);
  Serial.print("Midi value2: ");
  Serial.println(value2);

  delay(50);
}



////
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
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Failed to find sensor");
    return 0;
  }
  return 1;
}

int vlread(Adafruit_VL6180X *vlsensor, uint8_t i) {
  vlselect(i);
  return vlsensor->readRange();
}

uint vlmap(int range, int rangeMin, int rangeMax) {
  int midiMin = 0;
  int midiMax = 127;

  int mappedValue = map(range, rangeMin, rangeMax, midiMin, midiMax);
  int constrainedValue = constrain(mappedValue, midiMin, midiMax);

  return constrainedValue;
}
