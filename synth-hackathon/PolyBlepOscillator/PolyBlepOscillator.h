/*
 * Based on https://www.martin-finke.de/articles/audio-plugins-018-polyblep-oscillator/
 */
#pragma once
#include <NaiveOscillator/NaiveOscillator.h>
#include "LeakyIntegrator.h"


class PolyBlepOscillator {
	public:
		typedef enum {
			sine,
			triangle,
			square,
			sawtooth,
			numOscTypes,
		} Type;

		PolyBlepOscillator(){};
		PolyBlepOscillator(float fs, PolyBlepOscillator::Type type = sine)
		{
			setup(fs, type);
		}
		~PolyBlepOscillator();

		void setup(float fs, PolyBlepOscillator::Type type = sine);

		float process();
		float process(float frequency);
		void setType(PolyBlepOscillator::Type type) {
			type_ = type;
			if(type_ == triangle)
			{
				osc_->setType((NaiveOscillator::Type) square);
			}
			else
			{
				osc_->setType((NaiveOscillator::Type) type_);
			}
		}
		void setFrequency(float frequency) {
			frequency_ = frequency;
			osc_->setFrequency(frequency_);
			integrator_->setAlpha(osc_ -> getPhaseIncrement());
		}

		float getFrequency() { return frequency_; }
		int getType() { return type_; }

	private:
		LeakyIntegrator* integrator_;
		NaiveOscillator* osc_;
		float frequency_;
		unsigned int type_ = sine;
		float PolyBlep(float t);
};
