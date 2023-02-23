#pragma once
/* MIDI */
uint8_t midi_map(int value, int val_min, int val_max) {
	uint8_t midi_min = 0;
	uint8_t midi_max = 127;
	uint8_t constrained_input_value = constrain(value, val_min, val_max); 
	uint8_t midi_value = map(constrained_input_value, val_min, val_max, midi_min, midi_max);
	uint8_t constrained_midi_value = constrain(midi_value, midi_min, midi_max);
	return constrained_midi_value;
}

uint8_t midi_sign(int value) {
	if (value >= 0) {
		return 1; // pos
	} else {
		return 0; // neg
	}
}
