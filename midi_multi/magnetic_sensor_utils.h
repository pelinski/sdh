#pragma once
#include "Adafruit_MLX90393.h"
#include "midi_utils.h"

// MLX90393
#define mg1Addr 0x0D
#define mg2Addr 0x0F
#define mg3Addr 0x0C
#define mg4Addr 0x0E

#define MLX90393_CS 10

#define MLX90393_MAX_VAL 32768

struct MLX90393_config {
	enum mlx90393_gain gain_;
	enum mlx90393_oversampling oversampling_;
	enum mlx90393_filter filter_;
	enum mlx90393_resolution resX_, resY_, resZ_;

	MLX90393_config(mlx90393_gain gain, mlx90393_oversampling oversampling, mlx90393_filter filter , mlx90393_resolution resX, mlx90393_resolution resY, mlx90393_resolution resZ) :
		gain_{gain}, oversampling_{oversampling}, filter_{filter}, resX_{resX}, resY_{resY}, resZ_{resZ} {}
};

/* MAGNETIC SENSORS */
uint8_t mg_begin(Adafruit_MLX90393 *mgsensor, uint8_t mgAddr) {

	if (!mgsensor->begin_I2C(mgAddr)) {
		return 1;
	}
	Serial.print("MLX90303 sensor found at address ");Serial.println(mgAddr);
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
struct AxisParameters {
	float min;
	float max;
	float avg;
	//float hysteresis[2];
	float hysteresis[2];

	AxisParameters(float min, float max, float avg, float hysteresis1, float hysteresis2) : min{min}, max{max}, avg{avg}
	{
		hysteresis[0] = hysteresis1;
		hysteresis[1] = hysteresis2;
	}
};

struct MLX90393_processing {
	AxisParameters* x_param;
	AxisParameters* y_param;
	AxisParameters* z_param;

	MLX90393_processing(AxisParameters* x_param, AxisParameters* y_param, AxisParameters* z_param) : x_param{x_param}, z_param{z_param}, y_param{y_param} {}

	uint8_t midi_x_range(float x)
	{
		int  _x = x + fabs(x_param->min);
		int _max = x_param->max + fabs(x_param->min);
		if(_x < 0)
			x = 0;
		return midi_map(_x, 0, _max);
	}

	uint8_t midi_y_range(float y)
	{
		int  _y = y + fabs(y_param->min);
		int _max = y_param->max + fabs(y_param->min);
		if(_y < 0)
			y = 0;
		return midi_map(_y, 0, _max);
	}

	uint8_t midi_z_range(float z)
	{
		int _z = 0;
		if(z > 0.0)
			_z = (int)z;
		_z = constrain(_z, (int)z_param->min, (int)z_param->max);
		return midi_map(_z, (int)z_param->min, (int)z_param->max);
	}


};

struct MLX90393_calibration {
	uint8_t calCycles;
	uint8_t cycleCount;
	bool avgReady;
	float minX;
	float maxX;
	float avgX;
	float hysteresisX[2];

	float minY;
	float maxY;
	float avgY;
	float hysteresisY[2];

	float minZ;
	float maxZ;
	float avgZ;
	float hysteresisZ[2];

	MLX90393_calibration(uint8_t calibrationCycles = 20) : calCycles{calibrationCycles}, cycleCount{0}, minX{MLX90393_MAX_VAL}, maxX{0}, avgX{0}, minY{MLX90393_MAX_VAL}, maxY{0}, avgY{0}, minZ{MLX90393_MAX_VAL}, maxZ{0}, avgZ{0}, avgReady{false}{
		hysteresisX[0] = 0.0;
		hysteresisX[1] = 0.0;
		hysteresisY[0] = 0.0;
		hysteresisY[1] = 0.0;
		hysteresisZ[0] = 0.0;
		hysteresisZ[1] = 0.0;
	}

	bool calcAverage(float x, float y, float z)
	{
		avgX += x;
		avgY += y;
		avgZ += z;

		cycleCount++;

		if(cycleCount >= calCycles)
		{
			avgX /= (float) calCycles;
			avgY /= (float) calCycles;
			avgZ /= (float) calCycles;

			printRanges();
			avgReady = true;
		}
		return avgReady;
	}

	void updateRange(float x, float y, float z)
	{
		if(x < minX) minX = x;
		if(x > maxX) maxX = x;

		if(y < minY) minY = y;
		if(y > maxY) maxY = y;

		if(z < minZ) minZ = z;
		if(z > maxZ) maxZ = z;
	}

	void set_range_X(float min, float max)
	{
		minX = min;
		maxX = max;
	}

	void set_range_Y(float min, float max)
	{
		minY = min;
		maxY = max;
	}

	void set_range_Z(float min, float max)
	{
		minZ = min;
		maxZ = max;
	}

	void printRanges()
	{
		Serial.print("avgX: "); Serial.print(avgX, 4); Serial.print(", ");
		Serial.print("minX: "); Serial.print(minX, 4); Serial.print(", ");
		Serial.print("maxX: "); Serial.print(maxZ, 4); Serial.print("\n");

		Serial.print("avgY: "); Serial.print(avgY, 4); Serial.print(", ");
		Serial.print("minY: "); Serial.print(minY, 4); Serial.print(", ");
		Serial.print("maxY: "); Serial.print(maxY, 4); Serial.print("\n");

		Serial.print("avgZ: "); Serial.print(avgZ, 4); Serial.print(", ");
		Serial.print("minZ: "); Serial.print(minZ, 4); Serial.print(", ");
		Serial.print("maxZ: "); Serial.print(maxZ, 4); Serial.print("\n");
	}
};

