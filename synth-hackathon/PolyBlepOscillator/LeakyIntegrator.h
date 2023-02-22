/*
 * Leaky integrator: y[n] = A * x[n] + (1 - A) * y[n-1]
 */
#pragma once


class LeakyIntegrator {
	public:
		LeakyIntegrator(){};
		~LeakyIntegrator(){};

		LeakyIntegrator(float alpha) { setup(alpha); }
		void setup(float alpha) { setAlpha(alpha); }
	
		void setAlpha(float alpha) { alpha_ = alpha; }
		float getAlpha() { return alpha_; }

		float process(float in)
		{
			float out = lastOut_ + alpha_ * (in - lastOut_);
			lastOut_ = out;
			return out;
		}	

	private:
		float alpha_ = 1.0;
		float lastOut_ = 0.0;


};
