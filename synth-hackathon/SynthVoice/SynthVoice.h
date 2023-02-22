#pragma once
#include "PolyBlepOscillator/PolyBlepOscillator.h"
#include <vector>

class SynthVoice {
	public:
		SynthVoice();
		~SynthVoice();

		SynthVoice(float fs, unsigned int numberOsc, PolyBlepOscillator::Type type);
		void setup(float fs, unsigned int numberOsc, PolyBlepOscillator::Type type);

		void setCentreFrequency(float frequency);
		float getCentreFrequency() { return centreFreq_; }

		void setDetuningRatio(float detuning);
		float getDetuningRatio() { return detuningRatio_; }

		void setType(PolyBlepOscillator::Type type);
		void setOscillatorType(unsigned int oscIdx, PolyBlepOscillator::Type type);

		float process();
	private:
		PolyBlepOscillator::Type type_;
		float centreFreq_ = 0.0;
		float detuningRatio_ = 0.0;
		unsigned int nOsc_ = 1;
		float invertNOsc_ = 1.0;
		unsigned int nDetunedOsc_ = 0;
		float invNDetunedOsc_ = 0.0;
		std::vector<PolyBlepOscillator*> oscillators_;

		void calcDetunedFrequencies();
};

SynthVoice::SynthVoice(float fs, unsigned int numberOsc, PolyBlepOscillator::Type type)
{
	setup(fs, numberOsc, type);
}
void SynthVoice::setup(float fs, unsigned int nOsc, PolyBlepOscillator::Type type)
{
	nOsc_ = nOsc;
	if ( nOsc_ % 2 == 0)
		nOsc_ += 1;
		
	invertNOsc_ = 1.0/(float) nOsc_;

	nDetunedOsc_ = nOsc_ / 2;
	invNDetunedOsc_ = 1.0 / (float) nDetunedOsc_;

	for(unsigned int i = 0; i < nOsc_; i++)
	{
		oscillators_.push_back(new PolyBlepOscillator(fs, type));
	}
}

void SynthVoice::setCentreFrequency(float frequency)
{
	centreFreq_ = frequency;
	oscillators_[0]->setFrequency(centreFreq_);
	calcDetunedFrequencies();	
}

void SynthVoice::calcDetunedFrequencies()
{
	unsigned int idx = 1;
	for(unsigned int i = 0; i < nDetunedOsc_; i++)
	{
		float f = centreFreq_* (1.0 + detuningRatio_ * i * invNDetunedOsc_);
		oscillators_[idx]->setFrequency(f);
		idx += 1;
	}
	for(unsigned int i = 0; i < nDetunedOsc_; i++)
	{
		float f = centreFreq_* (1.0 - detuningRatio_ * i * invNDetunedOsc_);
		oscillators_[idx]->setFrequency(f);
		idx += 1;
	}
}

void SynthVoice::setDetuningRatio(float detuning)
{
	detuningRatio_ = detuning;
	calcDetunedFrequencies();
}

void SynthVoice::setType(PolyBlepOscillator::Type type)
{
	for(unsigned int i = 0; i < nOsc_; i++)
	{
		oscillators_[i] -> setType(type);
	}
}
void SynthVoice::setOscillatorType(unsigned int oscIdx, PolyBlepOscillator::Type type)
{
	if(oscIdx < nOsc_)
		oscillators_[oscIdx]->setType(type);
}

float SynthVoice::process()
{
	float out = 0.0;
	for(unsigned int i = 0; i < nOsc_; i++)
	{
		out += invertNOsc_*oscillators_[i]->process();
	}
	return out;
}
