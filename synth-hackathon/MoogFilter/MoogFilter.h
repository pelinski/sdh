#define MOOG_FILTER_ORDER 4
class MoogFilter {
	public:
		MoogFilter(){};
		~MoogFilter(){};

		MoogFilter(float fs);
		void setup(float fs);

		void setFrequency(float frequency);
		float getFrequency() { return frequency_; }
		void setResonance(float resonance);
		float getResonance() { return resonance_; }

		float process(float input);
	private:
		float frequency_ = 0.0;
		float resonance_ = 0.0; // Normalised 0 -> 1
		float internalRes_ = 0.0;
		float invSampleRate_;
		float discreteFreq_ = 0.0;

		float coeffB0_ = 1.0;
		float coeffB1_ = 0.0;
		float coeffA1_ = 0.0;

		void calcDiscreteFreq();
		void calcCoefficients();
		void calcResonance();

		const float gcomp_ = 0.5;
		float lastOut_[MOOG_FILTER_ORDER];
		float lastIn_[MOOG_FILTER_ORDER];

};

