#include "math.h"
#include "PolyBlepOscillator.h"

void PolyBlepOscillator::setup(float fs, PolyBlepOscillator::Type type)
{
	osc_ = new NaiveOscillator(fs);		
	integrator_ = new LeakyIntegrator();
	setType(type);
}

float PolyBlepOscillator::process()
{
	float t = osc_->getPhase() / (2.0 * M_PI);
	float ret = 0.0;
	if(type_ == sine)
	{
		ret = osc_->process();
	}
	else if(type_ == sawtooth)
	{
		ret = osc_->process();
		// ret -= PolyBlep(t);
	}
	else
	{	
		// SQUARE
		ret = osc_->process();
	// 	ret += PolyBlep(t);
	// 	ret -= PolyBlep(fmod(t + 0.5, 1.0));
	// 	// TRIANGLE
		if(type_ == triangle)
		{
			ret = integrator_->process(ret);
		}
	}
	return ret;
}

float PolyBlepOscillator::process(float frequency)
{
	setFrequency(frequency);
	return process();
}

float PolyBlepOscillator::PolyBlep(float t)
{
	float dt = frequency_ * osc_->getInverseFs();
	//float dt = osc_->getPhaseIncrement() / (2.0 * M_PI);
	// 0 <= t < 1
	if (t < dt)
	{
		t /= dt;
		return t+t - t*t - 1.0;
	}
	// -1 < t < 0
	else if (t > 1.0 - dt)
	{
		t = (t - 1.0) / dt;
		return t*t + t+t + 1.0;
	}
	// 0 otherwise
	else
	{
		return 0.0;
	}
}

PolyBlepOscillator::~PolyBlepOscillator()
{
	delete osc_;
	delete integrator_;
}
