#include <Bela.h>
#include <cmath>
#include "MoogFilter/MoogFilter.h"
#include "SynthVoice/SynthVoice.h"

#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <libraries/Scope/Scope.h>

#include <vector>

Gui gGui;
GuiController gGuiController;
// Browser-based oscilloscope to visualise signal
Scope gScope;


#define NUM_VOICES 1
#define SUPERSAW_OSC 7
std::vector<SynthVoice*> gSynthVoices;

PolyBlepOscillator testOsc;

MoogFilter gFilter;

bool setup(BelaContext *context, void *userData)
{
	for(unsigned int i = 0; i < NUM_VOICES; i++)
	{
		gSynthVoices.push_back(new SynthVoice(context->audioSampleRate, SUPERSAW_OSC, PolyBlepOscillator::square));
	}
	
	testOsc.setup(context->audioSampleRate, PolyBlepOscillator::sawtooth);
	
	gFilter.setup(context->audioSampleRate);
	// Set up the GUI
	gGui.setup(context->projectName);
	gGuiController.setup(&gGui, "Oscillator and Filter Controls");	
	
	// Arguments: name, default value, minimum, maximum, increment
	// Create sliders for oscillator and filter settings
	gGuiController.addSlider("Oscillator Frequency", 440, 40, 8000, 0);
	gGuiController.addSlider("Oscillator Amplitude", 0.5, 0, 1.0, 0);
	gGuiController.addSlider("Oscillator Detuning", 0.5, 0, 1.0, 0);
	gGuiController.addSlider("Filter Frequency", 1000, 100, 5000, 0);
	gGuiController.addSlider("Filter Resonance", 0, 0, 1.1, 0);
	
	// Set up the scope
	gScope.setup(2, context->audioSampleRate);
	return true;
}

void render(BelaContext *context, void *userData)
{
	float oscFrequency = gGuiController.getSliderValue(0);	
	float oscAmplitude = gGuiController.getSliderValue(1);	
	float oscDetuning = gGuiController.getSliderValue(2);	
	float filterFrequency = gGuiController.getSliderValue(3);
	float filterResonance = gGuiController.getSliderValue(4);


	
	for(unsigned int i = 0; i < NUM_VOICES; i++)
	{
		gSynthVoices[i]->setCentreFrequency(oscFrequency);
		gSynthVoices[i]->setDetuningRatio(oscDetuning);
	}
	
	gFilter.setFrequency(filterFrequency);
	gFilter.setResonance(filterResonance);
	// testOsc.setFrequency(oscFrequency);
	for(unsigned int n = 0; n < context->audioFrames; n++) {
		
		float out = 0.0;
		for(unsigned int i = 0; i < NUM_VOICES; i++)
		{
			out += oscAmplitude * gSynthVoices[i]->process();
		}
		out = gFilter.process(out);
		// out = testOsc.process();
		for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
			// Write the sample to every audio output channel
			audioWrite(context, n, channel, out);
		}
		gScope.log(out);
	}
}

void cleanup(BelaContext *context, void *userData)
{}
