#include "NaiveOscillator.h"
#include <cmath>
#include <libraries/math_neon/math_neon.h>

void NaiveOscillator::setup(float fs, NaiveOscillator::Type type)
{
	invSampleRate_ = 1.0 / fs;
	setType(type);
	phase_ = 0;
}

float NaiveOscillator::process(float frequency) {
	setFrequency(frequency);
	return process();
}

float NaiveOscillator::process() {
	float out;
	switch(type_) {
		default:
		case sine:
			out = sinf_neon(phase_);
			break;
		case triangle:
			if (phase_ > 0) {
			      out = -1 + (2 * phase_ / (float)M_PI);
			} else {
			      out = -1 - (2 * phase_/ (float)M_PI);
			}
			break;
		case square:
			if (phase_ > 0) {
			      out = 1;
			} else {
			      out = -1;
			}
			break;
		case sawtooth:
			out = 1.f / (float)M_PI * phase_;
			break;
	}
	computePhase();
	return out;
}

void NaiveOscillator::computePhase(){
	phase_ += phaseIncrement_;
	if(phase_ > M_PI)
		phase_ -= 2.0f * (float)M_PI;
}

void NaiveOscillator::computePhaseIncrement()
{
	phaseIncrement_ = 2.0f * (float)M_PI * frequency_ * invSampleRate_;
}
