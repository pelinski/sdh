#include <Wire.h>
#include "Adafruit_VL6180X.h"


Adafruit_VL6180X vl = Adafruit_VL6180X();


void setup() {
  Serial.begin(115200);

  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");


}



void loop() {
  // float lux = vl.readLux(VL6180X_ALS_GAIN_5);
  // Serial.print("Lux: "); Serial.println(lux);
  //Handle USB communication
	// USBMIDI.poll();
  uint8_t range = vl.readRange();

  int rangeMin = 5;
  int rangeMax = 170;
  int midiMin = 0;
  int midiMax = 127;
  int value = map(range, rangeMin, rangeMax, 0, 127);
  value = constrain(value, midiMin, midiMax);

  usbMIDI.sendControlChange(0, value, 1); // control, value, channel

  Serial.print("Midi value: "); Serial.println(value);

  delay(50);
}
