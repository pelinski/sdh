#include <Wire.h>
#include "Adafruit_VL6180X.h"
// #include <midi_serialization.h>
// #include <usbmidi.h>

Adafruit_VL6180X vl = Adafruit_VL6180X();

// void sendCC(uint8_t channel, uint8_t control, uint8_t value) {
// 	USBMIDI.write(0xB0 | (channel & 0xf));
// 	USBMIDI.write(control & 0x7f);
// 	USBMIDI.write(value & 0x7f);
// }


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

  // sendCC(0, 0, value);

  Serial.print("Midi value: "); Serial.println(value);

  delay(50);
}