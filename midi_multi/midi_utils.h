#pragma once
/* MIDI */
uint8_t midi_map(int value, int min_val, int max_val) {
	uint8_t midi_min = 0;
	uint8_t midi_max = 127;

	uint8_t mapped_value = map(value, min_val, max_val, midi_min, midi_max);
	uint8_t constrained_value = constrain(mapped_value, midi_min, midi_max);

	return constrained_value;
}

uint8_t midi_sign(int value) {
	if (value >= 0) {
		return 1; // pos
	} else {
		return 0; // neg
	}
}
