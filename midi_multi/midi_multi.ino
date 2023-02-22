// Using Teensyduino IDE https://www.pjrc.com/teensy/td_download.html
// https://www.pjrc.com/teensy/td_157/Teensyduino_MacOS_Catalina.zip Set Tools
// >> USB Type >> MIDI
#define TCAADDR 0x70
#include "Adafruit_VL6180X.h"
#include "Adafruit_MLX90393.h"
#include <Wire.h>

#define midiCh 0

#define vl1Port 0 // north
#define vl2Port 3 // south
#define mg1Addr 0x0D
#define mg2Addr 0x0F
#define mg3Addr 0x0C
#define mg4Addr 0x0E


#define MLX90393_CS 10


Adafruit_VL6180X vl1 = Adafruit_VL6180X();
Adafruit_VL6180X vl2 = Adafruit_VL6180X();
Adafruit_MLX90393 mg1 = Adafruit_MLX90393();
Adafruit_MLX90393 mg2 = Adafruit_MLX90393();
Adafruit_MLX90393 mg3 = Adafruit_MLX90393();
Adafruit_MLX90393 mg4 = Adafruit_MLX90393();


uint8_t prev_range_vl1 =
    0; // store previous value of vel --> needed for derivates
uint8_t prev_range_vl2 = 0;
uint8_t dt = 50; // ms

// calibration vl
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
uint8_t mgbegin(Adafruit_VL6180X *vlsensor, uint8_t i);
uint8_t vlbegin(Adafruit_MLX90393 *mgsensor, uint8_t mgAddr);
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

  // /* Initialise the 1st sensor */
  // if (!vlbegin(&vl1, vl1Port)) {
  //   Serial.println("couldn't intialise vl1");
  //   return;
  // }
  // /* Initialise the 2nd sensor */
  // if (!vlbegin(&vl2, vl2Port)) {
  //   Serial.println("couldn't intialise vl2");
  //   return;
  // }

  if (!mgbegin(&mg1, mg1Addr)) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("couldn't intialise mg1");
    return;
  }
  if(!mgbegin(&mg2, mg2Addr)) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("couldn't intialise mg2");
    return;
  }
    if (!mgbegin(&mg3, mg3Addr)) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("couldn't intialise mg3");
    return;
  }
    if (!mgbegin(&mg4, mg4Addr)) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("couldn't intialise mg3");
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

  // magnetic
  float x,y,z;
    if (mg1.readData(&x, &y, &z)) {
      Serial.print("X: "); Serial.print(x, 4); Serial.print(" uT\t");
      Serial.print("Y: "); Serial.print(y, 4); Serial.print(" uT\t");
      Serial.print("Z: "); Serial.print(z, 4); Serial.println(" uT");
  } else {
      Serial.println("Unable to read XYZ data from the sensor.");
  }

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
  usbMIDI.sendControlChange(6, midi_diff_vl1vl2, midiCh);
  usbMIDI.sendControlChange(7, midi_avg_vl1vl2, midiCh);

  delay(dt);
}

//// >>>> lib <<<<<

// vl

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

// magnetic sensors 
uint8_t mgbegin(Adafruit_MLX90393 *mgsensor, uint8_t mgAddr) {

  if (!mgsensor->begin_I2C(mgAddr)) {
    return 0;
  }
  return 1;

  mgsensor->setGain(MLX90393_GAIN_5X);
  
  // Set resolution, per axis
  mgsensor->setResolution(MLX90393_X, MLX90393_RES_19);
  mgsensor->setResolution(MLX90393_Y, MLX90393_RES_19);
  mgsensor->setResolution(MLX90393_Z, MLX90393_RES_19);

  // Set oversampling
  // mgsensor->setOversampling(MLX90393_OSR_2);

  // Set digital filtering
  mgsensor->setFilter(MLX90393_FILTER_6);

}



// midi
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
