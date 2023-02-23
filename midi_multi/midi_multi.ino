// Using Teensyduino IDE https://www.pjrc.com/teensy/td_download.html
// https://www.pjrc.com/teensy/td_157/Teensyduino_MacOS_Catalina.zip Set Tools
// >> USB Type >> MIDI
#include "Adafruit_VL6180X.h"
#include "Adafruit_MLX90393.h"
#include <Wire.h>

/* MIDI CHANNEL */
#define midiCh 0

/* ADDRESSES */

// I2C Multiplexer
#define TCAADDR 0x70

// VL6180X ports (on multiplexer), default address is 0x29
#define vl1Port 0 // north
#define vl2Port 3 // south

// MLX90393
#define mg1Addr 0x0D
#define mg2Addr 0x0F
#define mg3Addr 0x0C
#define mg4Addr 0x0E

#define MLX90393_CS 10

// Serial speed
#define SERIAL_SPEED 115200

//#define USE_TOF_SENSORS
#define USE_MAGNETIC_SENSORS
#define USE_MIDI
#define VERBOSE

uint8_t midi_map(int value, int min_val, int max_val);
uint8_t midi_sign(int value);

struct VL6180X_calibration {
	uint8_t range[2];
	uint8_t velocity[2];
	uint8_t diff[2];
	uint8_t avg[2];

	VL6180X_calibration(uint8_t range_min = 5, uint8_t range_max = 170, uint8_t vel_min = 0, uint8_t vel_max = 122)
	{
		range[0] = range_min;
		range[1] = range_max;
		velocity[0] = vel_min;
		velocity[1] = vel_max;
		diff[0] = 0;
		diff[1] = range[1] - range[0];
		avg[0] = 0;
		avg[1] = range[1];
	};
};

struct VL6180X_processing {
	uint8_t prev_reading;
	int velocity;

	VL6180X_processing():prev_reading(0){}

	uint8_t get_midi_range(uint8_t reading, VL6180X_calibration* calibration)
	{
		return midi_map(reading, calibration->range[0], calibration->range[1]);
	}

	void calc_velocity(uint8_t reading, uint8_t dt)
	{
		velocity = (reading - prev_reading) * 100 / dt;
	}

	uint8_t get_midi_velocity(VL6180X_calibration* calibration)
	{
		return midi_map(velocity, calibration->velocity[0], calibration->velocity[1]);
	}

	uint8_t get_midi_velocity(uint8_t reading, uint8_t dt, VL6180X_calibration* calibration)
	{
		calc_velocity(reading, dt);
		return midi_map(velocity, calibration->velocity[0], calibration->velocity[1]);
	}

	uint8_t get_midi_velocity_sign()
	{
		return midi_sign(velocity);
	}

	uint8_t get_midi_diff(uint8_t reading1, uint8_t reading2, VL6180X_calibration* calibration)
	{
		uint8_t diff = reading1- reading2;
		return midi_map(diff, calibration->diff[0], calibration->diff[1]);
	}

	uint8_t get_midi_avg(uint8_t reading1, uint8_t reading2, VL6180X_calibration* calibration)
	{
		float avg = (reading1 + reading2) / 2;
		return midi_map(avg, calibration->avg[0], calibration->avg[1]);
	}
};

struct MLX90393_config {
	enum mlx90393_gain gain_;
	enum mlx90393_resolution resX_, resY_, resZ_;
	enum mlx90393_oversampling oversampling_;
	enum mlx90393_filter filter_;

	MLX90393_config(mlx90393_gain gain = MLX90393_GAIN_5X, mlx90393_resolution resX = MLX90393_RES_19, mlx90393_resolution resY = MLX90393_RES_19, mlx90393_resolution resZ = MLX90393_RES_19,
			mlx90393_oversampling oversampling = MLX90393_OSR_2, mlx90393_filter filter = MLX90393_FILTER_6) :
		gain_(gain), resX_(resX), resY_(resY), resZ_(resZ), oversampling_(oversampling), filter_(filter) {}
};

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


uint8_t prev_range_vl1 = 0; // store previous value of vel --> needed for derivates
uint8_t prev_range_vl2 = 0;
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

//// >>>> lib <<<<<

/* TIME OF FLIGHT SENSORS */
void vl_select(uint8_t i) { // select multiplexer port
	if (i > 3)
		return;
	Wire.beginTransmission(TCAADDR);
	Wire.write(1 << i);
	Wire.endTransmission();
}

uint8_t vl_begin(Adafruit_VL6180X *vlsensor, uint8_t i) {
	vl_select(i);

	if (!vlsensor->begin()) {
		Serial.println("Failed to find sensor");
		return 1;
	}
	return 0;
}

uint8_t vl_read(Adafruit_VL6180X *vlsensor, uint8_t i) {
	vl_select(i);
	return vlsensor->readRange();
}

/* MAGNETIC SENSORS */
uint8_t mg_begin(Adafruit_MLX90393 *mgsensor, uint8_t mgAddr) {

  Serial.print("Address:"); Serial.println(mgAddr);
	if (!mgsensor->begin_I2C(mgAddr)) {
    Serial.println("Couldn't initialise sensor");
		return 1;
	}
	return 0;
}

void mg_config(Adafruit_MLX90393 *mgsensor, MLX90393_config *config) {

	mgsensor->setGain(config->gain_);

	// Set resolution, per axis
	mgsensor->setResolution(MLX90393_X, config->resX_);
	mgsensor->setResolution(MLX90393_Y, config->resY_);
	mgsensor->setResolution(MLX90393_Z, config->resZ_);

	// Set oversampling
	mgsensor->setOversampling(config->oversampling_);

	// Set digital filtering
	mgsensor->setFilter(config->filter_);
}


/* MIDI */
uint8_t midi_map(int value, int min_val, int max_val) {
	uint8_t midi_min = 0;
	uint8_t midi_max = 127;

	uint8_t mapped_value = map(abs(value), min_val, max_val, midi_min, midi_max);
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
