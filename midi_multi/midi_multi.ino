// Using Teensyduino IDE https://www.pjrc.com/teensy/td_download.html
// https://www.pjrc.com/teensy/td_157/Teensyduino_MacOS_Catalina.zip Set Tools
// >> USB Type >> MIDI
#include <Wire.h>
//#include "sensor_utils.h"
#include "tof_sensor_utils.h"
#include "magnetic_sensor_utils.h"

/* MIDI CHANNEL */
const int midiChannel = 1;
int midiCable = 0;

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
#define USE_MG1
//#define USE_MG2
//#define USE_MG3
//#define USE_MG4

Adafruit_MLX90393 mg1 = Adafruit_MLX90393();
MLX90393_config mg1_config(MLX90393_GAIN_2_5X, MLX90393_OSR_3, MLX90393_FILTER_7, MLX90393_RES_19, MLX90393_RES_19, MLX90393_RES_19);

AxisParameters mg1_param_x(-400.0, 250.0, 0.0, 0.0, 0.0);
AxisParameters mg1_param_y(-250.0, 450, 0.0, 0.0, 0.0);
AxisParameters mg1_param_z(105.0, 1800.0, 0.0, 0.0, 0.0);
MLX90393_processing mg1_proc = MLX90393_processing(&mg1_param_x, &mg1_param_y, &mg1_param_z);
MLX90393_calibration mg1_cal = MLX90393_calibration();

Adafruit_MLX90393 mg2 = Adafruit_MLX90393();
MLX90393_config mg2_config(MLX90393_GAIN_2_5X, MLX90393_OSR_3, MLX90393_FILTER_7, MLX90393_RES_19, MLX90393_RES_19, MLX90393_RES_19);

AxisParameters mg2_param_x(-400.0, 250.0, 0.0, 0.0, 0.0);
AxisParameters mg2_param_y(-250.0, 450, 0.0, 0.0, 0.0);
AxisParameters mg2_param_z(105.0, 1800.0, 0.0, 0.0, 0.0);
MLX90393_processing mg2_proc = MLX90393_processing(&mg2_param_x, &mg2_param_y, &mg2_param_z);
MLX90393_calibration mg2_cal = MLX90393_calibration();

Adafruit_MLX90393 mg3 = Adafruit_MLX90393();
MLX90393_config mg3_config(MLX90393_GAIN_2_5X, MLX90393_OSR_3, MLX90393_FILTER_7, MLX90393_RES_19, MLX90393_RES_19, MLX90393_RES_19);

AxisParameters mg3_param_x(-400.0, 250.0, 0.0, 0.0, 0.0);
AxisParameters mg3_param_y(-250.0, 450, 0.0, 0.0, 0.0);
AxisParameters mg3_param_z(105.0, 1800.0, 0.0, 0.0, 0.0);
MLX90393_processing mg3_proc = MLX90393_processing(&mg3_param_x, &mg3_param_y, &mg3_param_z);
MLX90393_calibration mg3_cal = MLX90393_calibration();

Adafruit_MLX90393 mg4 = Adafruit_MLX90393();
MLX90393_config mg4_config(MLX90393_GAIN_2_5X, MLX90393_OSR_3, MLX90393_FILTER_7, MLX90393_RES_19, MLX90393_RES_19, MLX90393_RES_19);

AxisParameters mg4_param_x(-400.0, 250.0, 0.0, 0.0, 0.0);
AxisParameters mg4_param_y(-250.0, 450, 0.0, 0.0, 0.0);
AxisParameters mg4_param_z(105.0, 1800.0, 0.0, 0.0, 0.0);
MLX90393_processing mg4_proc = MLX90393_processing(&mg4_param_x, &mg4_param_y, &mg4_param_z);
MLX90393_calibration mg4_cal = MLX90393_calibration();

uint8_t dt = 50; // ms


enum midiCC
{
	kVl1_range = 0,
	kVl1_velocity = 1,
	kVl1_velocity_sign = 2,
	kVl2_range = 3,
	kVl2_velocity = 4,
	kVl2_velocity_sign = 5,
	kVl_diff = 6,
	kVl_diff_sign = 7,
	kVl_avg = 8,
	kMg1_x = 9,
	kMg1_y = 10,
	kMg1_z = 11,
	kMg2_x = 12,
	kMg2_y = 13,
	kMg2_z = 14,
	kMg3_x = 15,
	kMg3_y = 16,
	kMg3_z = 17,
	kMg4_x = 18,
	kMg4_y = 19,
	kMg4_z = 20
};

void setup() {

	Serial.begin(SERIAL_SPEED);

	Serial.println("Hello!");

	Wire.begin();

	bool tof_sensors_ready = true;

#ifdef USE_TOF_SENSORS
	/* Initialise the 1st ToF sensor */
	if (vl_begin(&vl1, vl1Port) != 0) {
		Serial.println("Couldn't intialise vl1");
		tof_sensors_ready = false;
	}
	/* Initialise the 2nd ToF sensor */
	if (vl_begin(&vl2, vl2Port) != 0) {
		Serial.println("couldn't intialise vl2");
		tof_sensors_ready = false;
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
	// if(!(tof_sensors_ready && mg_sensors_ready)) {
	// 	Serial.println("One or more sensors are not working... cannot initialise sketch");
	// 	while(1) { delay(10); };
	// }
	Serial.println("Exiting setup()");
	// while(1) { delay(10); };  
}

void loop() {

#ifdef USE_TOF_SENSORS
	/* VL1 */
	// Range
	uint8_t vl1_range = vl_read(&vl1, vl1Port);
#if 0
	// Send MIDI CC
	usbMIDI.sendControlChange(midiCC::kVl1_range, vl1_proc.get_midi_range(vl1_range, &vl_calibration), midiChannel, midiCable);
	//vl1_proc.get_midi_range(vl1_range, &vl_calibration);

	// Velocity
	// Send MIDI CC
	usbMIDI.sendControlChange(midiCC::kVl1_velocity, vl1_proc.get_midi_velocity(vl1_range, dt, &vl_calibration), midiChannel, midiCable);
	//vl1_proc.get_midi_velocity(vl1_range, dt, &vl_calibration);

	// Velocity Sign
	usbMIDI.sendControlChange(midiCC::kVl1_velocity_sign, vl1_proc.get_midi_velocity_sign(), midiChannel, midiCable);
	//vl1_proc.get_midi_velocity_sign();
#endif
	/* VL2 */
	// Range
	uint8_t vl2_range = vl_read(&vl2, vl2Port);
#if 0
	usbMIDI.sendControlChange(midiCC::kVl2_range, vl2_proc.get_midi_range(vl2_range, &vl_calibration), midiChannel, midiCable);
	//vl2_proc.get_midi_range(vl2_range, &vl_calibration);

	// Velocity
	usbMIDI.sendControlChange(midiCC::kVl2_velocity, vl2_proc.get_midi_velocity(vl2_range, dt, &vl_calibration), midiChannel, midiCable);
	//vl2_proc.get_midi_velocity(vl2_range, dt, &vl_calibration);

	// Velocity Sign
	usbMIDI.sendControlChange(midiCC::kVl2_velocity_sign, vl2_proc.get_midi_velocity_sign(), midiChannel, midiCable);
	//vl2_proc.get_midi_velocity_sign();

#endif
	/* VL AGGREGATES */
	// Diff
	usbMIDI.sendControlChange(midiCC::kVl_diff, vl1_proc.get_midi_diff(vl1_range, vl2_range, &vl_calibration), midiChannel, midiCable);
	//vl1_proc.get_midi_diff(vl1_range, vl2_range, &vl_calibration);

	// Diff Sign
	usbMIDI.sendControlChange(midiCC::kVl_diff_sign, vl1_proc.get_midi_diff_sign(vl1_range, vl2_range), midiChannel, midiCable);
	//vl1_proc.get_midi_diff_sign(vl1_range, vl2_range);

	// Avg
	usbMIDI.sendControlChange(midiCC::kVl_avg, vl1_proc.get_midi_avg(vl1_range, vl2_range, &vl_calibration), midiChannel, midiCable);
	//vl1_proc.get_midi_avg(vl1_range, vl2_range, &vl_calibration);

#endif

#ifdef USE_MAGNETIC_SENSORS
	// magnetic
#ifdef USE_MG1
	float mg1_x, mg1_y, mg1_z;
/*
		Serial.print("X: "); Serial.print(mg1_x, 4); Serial.print(" uT\t");
		Serial.print("Y: "); Serial.print(mg1_y, 4); Serial.print(" uT\t");
		Serial.print("Z: "); Serial.print(mg1_z, 4); Serial.println(" uT");
*/
	if (mg1.readData(&mg1_x, &mg1_y, &mg1_z)) {
		if(!mg1_cal.avgReady)
		{
			mg1_cal.calcAverage(mg1_x, mg1_y, mg1_z);
			mg1_cal.updateRange(mg1_x, mg1_y, mg1_z);
		}
		else
		{
			//mg1_cal.printRanges();
			//while(1);
			mg1_x -= mg1_cal.avgX;
			mg1_y -= mg1_cal.avgY;
			mg1_z -= mg1_cal.avgZ;

			uint8_t mg1_z_midi = mg1_proc.midi_z_range(mg1_z);
#ifdef VERBOSE
			Serial.print("Z: "); Serial.print(mg1_z, 4); Serial.print(" uT\t");
			Serial.print("Z: "); Serial.print(mg1_z_midi); Serial.println(" \t");
#endif
			if(mg1_z_midi != 0)
				usbMIDI.sendControlChange(kMg1_z, mg1_proc.midi_z_range(mg1_z), midiChannel, midiCable);
#ifdef VERBOSE
			Serial.print("X: "); Serial.print(mg1_x, 4); Serial.print(" uT\t");
			Serial.print("X: "); Serial.print(mg1_proc.midi_x_range(mg1_x)); Serial.print(" \t");
#endif
			if(mg1_z_midi != 0)
				usbMIDI.sendControlChange(kMg1_x, mg1_proc.midi_x_range(mg1_x), midiChannel, midiCable);

#ifdef VERBOSE
			Serial.print("Y: "); Serial.print(mg1_y, 4); Serial.print(" uT\t");
			Serial.print("Y: "); Serial.print(mg1_proc.midi_y_range(mg1_y)); Serial.print(" \t");
#endif
			if(mg1_z_midi != 0)
				usbMIDI.sendControlChange(kMg1_y, mg1_proc.midi_y_range(mg1_y), midiChannel, midiCable);

		}
	} else {
		Serial.println("Unable to read XYZ data from the sensor.");
	}
#endif

#ifdef USE_MG2
	float mg2_x, mg2_y, mg2_z;
/*
		Serial.print("X: "); Serial.print(mg2_x, 4); Serial.print(" uT\t");
		Serial.print("Y: "); Serial.print(mg2_y, 4); Serial.print(" uT\t");
		Serial.print("Z: "); Serial.print(mg2_z, 4); Serial.println(" uT");
*/
	if (mg2.readData(&mg2_x, &mg2_y, &mg2_z)) {
		if(!mg2_cal.avgReady)
		{
			mg2_cal.calcAverage(mg2_x, mg2_y, mg2_z);
			mg2_cal.updateRange(mg2_x, mg2_y, mg2_z);
		}
		else
		{
			//mg2_cal.printRanges();
			//while(1);
			mg2_x -= mg2_cal.avgX;
			mg2_y -= mg2_cal.avgY;
			mg2_z -= mg2_cal.avgZ;

			uint8_t mg2_z_midi = mg2_proc.midi_z_range(mg2_z);
#ifdef VERBOSE
			Serial.print("Z: "); Serial.print(mg2_z, 4); Serial.print(" uT\t");
			Serial.print("Z: "); Serial.print(mg2_z_midi); Serial.println(" \t");
#endif
			if(mg2_z_midi != 0)
				usbMIDI.sendControlChange(kMg2_z, mg2_proc.midi_z_range(mg2_z), midiChannel, midiCable);
#ifdef VERBOSE
			Serial.print("X: "); Serial.print(mg2_x, 4); Serial.print(" uT\t");
			Serial.print("X: "); Serial.print(mg2_proc.midi_x_range(mg2_x)); Serial.print(" \t");
#endif
			if(mg2_z_midi != 0)
				usbMIDI.sendControlChange(kMg2_x, mg2_proc.midi_x_range(mg2_x), midiChannel, midiCable);

#ifdef VERBOSE
			Serial.print("Y: "); Serial.print(mg2_y, 4); Serial.print(" uT\t");
			Serial.print("Y: "); Serial.print(mg2_proc.midi_y_range(mg2_y)); Serial.print(" \t");
#endif
			if(mg2_z_midi != 0)
				usbMIDI.sendControlChange(kMg2_y, mg2_proc.midi_y_range(mg2_y), midiChannel, midiCable);

		}
	} else {
		Serial.println("Unable to read XYZ data from the sensor.");
	}
#endif
#ifdef USE_MG3
	float mg3_x, mg3_y, mg3_z;
/*
		Serial.print("X: "); Serial.print(mg3_x, 4); Serial.print(" uT\t");
		Serial.print("Y: "); Serial.print(mg3_y, 4); Serial.print(" uT\t");
		Serial.print("Z: "); Serial.print(mg3_z, 4); Serial.println(" uT");
*/
	if (mg3.readData(&mg3_x, &mg3_y, &mg3_z)) {
		if(!mg3_cal.avgReady)
		{
			mg3_cal.calcAverage(mg3_x, mg3_y, mg3_z);
			mg3_cal.updateRange(mg3_x, mg3_y, mg3_z);
		}
		else
		{
			//mg3_cal.printRanges();
			//while(1);
			mg3_x -= mg3_cal.avgX;
			mg3_y -= mg3_cal.avgY;
			mg3_z -= mg3_cal.avgZ;

			uint8_t mg3_z_midi = mg3_proc.midi_z_range(mg3_z);
#ifdef VERBOSE
			Serial.print("Z: "); Serial.print(mg3_z, 4); Serial.print(" uT\t");
			Serial.print("Z: "); Serial.print(mg3_z_midi); Serial.println(" \t");
#endif
			if(mg3_z_midi != 0)
				usbMIDI.sendControlChange(kMg3_z, mg3_proc.midi_z_range(mg3_z), midiChannel, midiCable);

#ifdef VERBOSE
			Serial.print("X: "); Serial.print(mg3_x, 4); Serial.print(" uT\t");
			Serial.print("X: "); Serial.print(mg3_proc.midi_x_range(mg3_x)); Serial.print(" \t");
#endif
			if(mg3_z_midi != 0)
				usbMIDI.sendControlChange(kMg3_x, mg3_proc.midi_x_range(mg3_x), midiChannel, midiCable);

#ifdef VERBOSE
			Serial.print("Y: "); Serial.print(mg3_y, 4); Serial.print(" uT\t");
			Serial.print("Y: "); Serial.print(mg3_proc.midi_y_range(mg3_y)); Serial.print(" \t");
#endif
			if(mg3_z_midi != 0)
				usbMIDI.sendControlChange(kMg3_y, mg3_proc.midi_y_range(mg3_y), midiChannel, midiCable);

}
	} else {
		Serial.println("Unable to read XYZ data from the sensor.");
	}
#endif

#ifdef USE_MG4
	float mg4_x, mg4_y, mg4_z;
/*
		Serial.print("X: "); Serial.print(mg4_x, 4); Serial.print(" uT\t");
		Serial.print("Y: "); Serial.print(mg4_y, 4); Serial.print(" uT\t");
		Serial.print("Z: "); Serial.print(mg4_z, 4); Serial.println(" uT");
*/
	if (mg4.readData(&mg4_x, &mg4_y, &mg4_z)) {
		if(!mg4_cal.avgReady)
		{
			mg4_cal.calcAverage(mg4_x, mg4_y, mg4_z);
			mg4_cal.updateRange(mg4_x, mg4_y, mg4_z);
		}
		else
		{
			//mg4_cal.printRanges();
			//while(1);
			mg4_x -= mg4_cal.avgX;
			mg4_y -= mg4_cal.avgY;
			mg4_z -= mg4_cal.avgZ;

			uint8_t mg4_z_midi = mg4_proc.midi_z_range(mg4_z);

#ifdef VERBOSE
			Serial.print("Z: "); Serial.print(mg4_z, 4); Serial.print(" uT\t");
			Serial.print("Z: "); Serial.print(mg4_z_midi); Serial.println(" \t");
#endif
			if(mg4_z_midi != 0)
				usbMIDI.sendControlChange(kMg4_z, mg4_proc.midi_z_range(mg4_z), midiChannel, midiCable);

#ifdef VERBOSE
			Serial.print("X: "); Serial.print(mg4_x, 4); Serial.print(" uT\t");
			Serial.print("X: "); Serial.print(mg4_proc.midi_x_range(mg4_x)); Serial.print(" \t");
#endif

			if(mg4_z_midi != 0)
				usbMIDI.sendControlChange(kMg4_x, mg4_proc.midi_x_range(mg4_x), midiChannel, midiCable);

#ifdef VERBOSE
			Serial.print("Y: "); Serial.print(mg4_y, 4); Serial.print(" uT\t");
			Serial.print("Y: "); Serial.print(mg4_proc.midi_y_range(mg4_y)); Serial.print(" \t");
#endif
			if(mg4_z_midi != 0)
				usbMIDI.sendControlChange(kMg4_y, mg4_proc.midi_y_range(mg4_y), midiChannel, midiCable);

		}
	} else {
		Serial.println("Unable to read XYZ data from the sensor.");
	}
#endif
#endif


	// Send midi CC (control, value, channel)
//  usbMIDI.sendControlChange(15, 127, midiChannel, midiCable);

	delay(dt);
}

