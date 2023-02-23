#include "Adafruit_VL6180X.h"
#include "Adafruit_MLX90393.h"

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
