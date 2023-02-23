#pragma once
#include "Adafruit_VL6180X.h"
#include <Wire.h>
#include "midi_utils.h"

// I2C Multiplexer
#define TCAADDR 0x70

// VL6180X ports (on multiplexer), default address is 0x29
#define vl1Port 0 // north
#define vl2Port 3 // south

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
		return midi_map(abs(velocity), calibration->velocity[0], calibration->velocity[1]);
	}

	uint8_t get_midi_velocity_sign()
	{
		return midi_sign(velocity);
	}

	uint8_t get_midi_diff(uint8_t reading1, uint8_t reading2, VL6180X_calibration* calibration)
	{
		int diff = reading1 - reading2;
		return midi_map(abs(diff), calibration->diff[0], calibration->diff[1]);
	}

	uint8_t get_midi_diff_sign(uint8_t reading1, uint8_t reading2)
	{
		int diff = reading1 - reading2;
		return midi_sign(diff);
	}

	uint8_t get_midi_avg(uint8_t reading1, uint8_t reading2, VL6180X_calibration* calibration)
	{
		float avg = (reading1 + reading2) / 2;
		return midi_map(avg, calibration->avg[0], calibration->avg[1]);
	}
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
		return 1;
	}
	Serial.print("VL6180X sensor found at port ");Serial.println(i);
	return 0;
}

uint8_t vl_read(Adafruit_VL6180X *vlsensor, uint8_t i) {
	vl_select(i);
	return vlsensor->readRange();
}
