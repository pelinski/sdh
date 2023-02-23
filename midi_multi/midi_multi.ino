// Using Teensyduino IDE https://www.pjrc.com/teensy/td_download.html
// https://www.pjrc.com/teensy/td_157/Teensyduino_MacOS_Catalina.zip Set Tools
// >> USB Type >> MIDI
#include <Wire.h>
#include "sensor_utils.h"

/* MIDI CHANNEL */
#define midiCh 0

/* ADDRESSES */


// Serial speed
#define SERIAL_SPEED 115200

//#define USE_TOF_SENSORS
#define USE_MAGNETIC_SENSORS
#define USE_MIDI
#define VERBOSE

/* SENSORS */
// Time of Flight Sensors
#define NUM_TOF_SENSORS  2
VL6180X_calibration vl_calibration;
Adafruit_VL6180X vl1 = Adafruit_VL6180X();
VL6180X_processing vl1_proc = VL6180X_processing();
Adafruit_VL6180X vl2 = Adafruit_VL6180X();
VL6180X_processing vl2_proc = VL6180X_processing();

// 3D Magnetic Sensors
#define NUM_MAG_SENSORS 4
Adafruit_MLX90393 mg1 = Adafruit_MLX90393();
MLX90393_config mg1_config;
Adafruit_MLX90393 mg2 = Adafruit_MLX90393();
MLX90393_config mg2_config;
Adafruit_MLX90393 mg3 = Adafruit_MLX90393();
MLX90393_config mg3_config;
Adafruit_MLX90393 mg4 = Adafruit_MLX90393();
MLX90393_config mg4_config;

uint8_t dt = 50; // ms


void setup() {

	Serial.begin(SERIAL_SPEED);

	// wait for serial port to open on native usb devices
	while (!Serial) {
		delay(1);
	}
	delay(50);

	Serial.println("Hello!");

	Wire.begin();

	bool tof_sensors_ready = true;

#ifdef USE_TOF_SENSORS
	/* Initialise the 1st ToF sensor */
	if (vl_begin(&vl1, vl1Port) != 0) {
		Serial.println("Couldn't intialise vl1");
		tof_sensors_ready = false;
		true;
	}
	/* Initialise the 2nd ToF sensor */
	if (vl_begin(&vl2, vl2Port) != 0) {
		Serial.println("couldn't intialise vl2");
		tof_sensors_ready = false;
		true;
	}
#endif

	bool mg_sensors_ready = true;

#ifdef USE_MAGNETIC_SENSORS
	if (mg_begin(&mg1, mg1Addr) != 0) {          // hardware I2C mode, can pass in address & alt Wire
		Serial.println("couldn't intialise mg1");
		mg_sensors_ready = false;
		//return;
	} else {
		mg_config(&mg1, &mg1_config);
	}
	delay(50);

	if(mg_begin(&mg2, mg2Addr) != 0) {          // hardware I2C mode, can pass in address & alt Wire
		Serial.println("couldn't intialise mg2");
		mg_sensors_ready = false;
		//return;
	} else {
		mg_config(&mg2, &mg2_config);
	}
	delay(50);

	if (mg_begin(&mg3, mg3Addr) != 0) {          // hardware I2C mode, can pass in address & alt Wire
		Serial.println("couldn't intialise mg3");
		mg_sensors_ready = false;
		//return;
	} else {
		mg_config(&mg3, &mg3_config);
	}
	delay(50);

	if (mg_begin(&mg4, mg4Addr) != 0) {          // hardware I2C mode, can pass in address & alt Wire
		Serial.println("couldn't intialise mg4");
		mg_sensors_ready = false;
		//return;
	} else {
		mg_config(&mg4, &mg4_config);
	}
	delay(50);

#endif
	if(!(tof_sensors_ready && mg_sensors_ready)) {
		Serial.println("One or more sensors are not working... cannot initialise sketch");
		while(1) { delay(10); };
	}
  Serial.println("Exiting setup()");
}

void loop() {

#ifdef USE_TOF_SENSORS
	/* VL1 */
	// Range
	uint8_t vl1_range = vl_read(&vl1, vl1Port);
	vl1_proc.get_midi_range(vl1_range, &vl_calibration);
	// Velocity
	vl1_proc.get_midi_velocity(vl1_range, dt, &vl_calibration);
	// Velocity Sign
	vl1_proc.get_midi_velocity_sign();

	/* VL2 */
	// Range
	uint8_t vl2_range = vl_read(&vl2, vl2Port);
	vl2_proc.get_midi_range(vl2_range, &vl_calibration);
	// Velocity
	vl2_proc.get_midi_velocity(vl2_range, dt, &vl_calibration);
	// Velocity Sign
	vl2_proc.get_midi_velocity_sign();

	/* VL AGGREGATES */
	// Diff
	vl1_proc.get_midi_diff(vl1_range, vl2_range, &vl_calibration);
	// Avg
	vl1_proc.get_midi_avg(vl1_range, vl2_range, &vl_calibration);

#endif
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


	// Send midi CC (control, value, channel)
	//usbMIDI.sendControlChange(0, midi_range_vl1, midiCh);
	//usbMIDI.sendControlChange(1, midi_range_vl2, midiCh);
	//usbMIDI.sendControlChange(2, midi_vel_vl1, midiCh);
	//usbMIDI.sendControlChange(3, midi_vel_vl2, midiCh);
	//usbMIDI.sendControlChange(4, midi_sign_vel_vl1, midiCh);
	//usbMIDI.sendControlChange(5, midi_sign_vel_vl2, midiCh);
	//usbMIDI.sendControlChange(6, midi_diff_vl1vl2, midiCh);
	//usbMIDI.sendControlChange(7, midi_avg_vl1vl2, midiCh);

	delay(dt);
}

