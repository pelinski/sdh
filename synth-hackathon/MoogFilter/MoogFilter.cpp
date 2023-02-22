#include "MoogFilter.h"
#include <libraries/math_neon/math_neon.h>
#include <cmath>

MoogFilter::MoogFilter(float fs)
{
	setup(fs);
}

void MoogFilter::setup(float fs)
{
	invSampleRate_ =  1.0 / fs;
}

void MoogFilter::setFrequency(float frequency)
{
	frequency_ = frequency;
	calcCoefficients();
	calcResonance();
}

void MoogFilter::setResonance(float resonance)
{
	resonance_ = resonance;
	calcResonance();
}

void MoogFilter::calcDiscreteFreq()
{
	discreteFreq_ =	2.0 * M_PI * frequency_ * invSampleRate_;
}

void MoogFilter::calcCoefficients()
{
	calcDiscreteFreq();
	float g = 0.9892 * discreteFreq_ - 0.4342 * powf(discreteFreq_, 2) + 0.1381 * powf(discreteFreq_, 3) - 0.0202 * powf(discreteFreq_, 4);

	// y[n] = g((0.3/1.3)x[n-1] + (1/1.3)x[n]) + (1 - g)y[n-1]
	// y[n] = (g / 1.3)x[n] + (0.3 * g / 1.3)x[n-1] + (1-g)y[n-1]
	coeffB0_ = g / 1.3;
	coeffB1_= 0.3 * g / 1.3;
	coeffA1_ = -(1.0 - g);
}

void MoogFilter::calcResonance()
{
	internalRes_ = resonance_ * (1.0029 + 0.0526 * discreteFreq_ - 0.0926 * powf(discreteFreq_, 2) + 0.0218 * powf(discreteFreq_, 3));
}

float MoogFilter::process(float input)
{
	// Calculate the input to the nonlinearity based on the global feedback loop
	float feedbackIn = input - 4.0 * internalRes_ * (lastOut_[MOOG_FILTER_ORDER - 1] - gcomp_ * input);
	// Apply the nonlinearity
	float unitIn = tanhf_neon(feedbackIn);
	float out = 0;
	for(unsigned int i = 0; i < MOOG_FILTER_ORDER; i++)
	{
		// Calculate output for this section
		out = unitIn * coeffB0_ + lastIn_[i] * coeffB1_ - lastOut_[i] * coeffA1_;	
		// Update previous state for this section and propagate the output of this
		// section to the input of the next one
		lastIn_[i] = unitIn;
		lastOut_[i] = out;
		unitIn = out;
	}

	return out;
}

