#pragma once

class NaiveOscillator {
public:
	typedef enum {
		sine,
		triangle,
		square,
		sawtooth,
		numOscTypes,
	} Type;

	NaiveOscillator(){};
	NaiveOscillator(float fs, NaiveOscillator::Type type = sine)
	{
		setup(fs, type);
	}
	~NaiveOscillator(){};

	void setup(float fs, NaiveOscillator::Type type = sine);

	float process();
	float process(float frequency);
	void setType(NaiveOscillator::Type type) {
		type_ = type;
	}
	void setFrequency(float frequency) {
		frequency_ = frequency;
		computePhaseIncrement();
	}

	void setPhase(float phase) {
		phase_ = phase;
	}

	float getPhase() { return phase_; }
	float getFrequency() { return frequency_; }
	int getType() { return type_; }
	
	float getPhaseIncrement() { return phaseIncrement_; }
	float getInverseFs() { return invSampleRate_; }

private:
	float phase_;
	float phaseIncrement_;
	float frequency_;
	float invSampleRate_;
	unsigned int type_ = sine;
	void computePhase();
	void computePhaseIncrement();
};
