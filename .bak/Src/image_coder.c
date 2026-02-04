#include "stdio.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "image_coder.h"

void encode_image_vector(uint8_t *image_chunk, size_t chunk_size,
                         float max_amplitude, uint8_t *encoded_chunk) {
  for (size_t i = 0; i < chunk_size; i++) {
    float normalized = (float)image_chunk[i] / 255.0f;
    float scaled = normalized * max_amplitude;
    if (scaled > max_amplitude) {
      scaled = max_amplitude;
    } else if (scaled < 0) {
      scaled = 0;
    }
    encoded_chunk[i] = (uint8_t)(scaled / max_amplitude * 255.0f);
  }
}

void encode_image(Image *image, float max_amplitude,
                  EncodedImage *encoded_image) {
  for (size_t i = 0; i < image->height; i++) {
    for (size_t j = 0; j < image->width; j++) {
      size_t idx = i * image->width + j;
      float normalized = (float)image->data[idx] / 255.0f;
      float scaled = normalized * max_amplitude;

      // Clamp
      if (scaled > max_amplitude)
        scaled = max_amplitude;
      else if (scaled < 0)
        scaled = 0;

      encoded_image->data[idx] = scaled;
    }
  }
}

// function generate_carrier_wave(
// 	period::Number,
// 	amps::AbstractVector{<:Number};
// 	duty_cycle::Number = 0.5,
// 	fs::Number = 1e3,
// )
// 	n_periods = length(amps)
// 	dt = 1 / fs
// 	total_time = period * n_periods

// #Pre - allocate arrays for efficiency
// 	t = 0:dt:(total_time-dt)
// 	signal = zeros(Float64, length(t))

// #Generate periodic rectangular wave
// 	pulse_width = duty_cycle * period
// 	for (i, ti) in enumerate(t)
// #Check if we're in the "on" portion of current period
// 		phase = mod(ti, period)
// 		signal[i] = (phase < pulse_width ? 1.0 : 0.0) * amps[floor(Int,
// ti / period)+1]
// #@show floor(ti / period) #Debug : show current period index
// 	end

// 	return collect(t), signal
// end

float *generate_waveform(const float *encoded_row, size_t row_size,
                         uint32_t clock_freq_hz, float period, float fill_ratio,
                         size_t *out_size) {
  // Validation
  if (!encoded_row || !out_size || clock_freq_hz == 0 || period <= 0 ||
      fill_ratio < 0 || fill_ratio > 1.0f) {
    if (out_size)
      *out_size = 0;
    return NULL;
  }

  size_t n_periods = row_size;
  float dt = 1.0f / (float)clock_freq_hz;
  float total_time = period * (float)n_periods;

  // Match Julia's range: 0:dt:(total_time-dt)
  *out_size = (size_t)(total_time / dt);

  if (*out_size == 0) {
    return NULL;
  }

  float *signal = (float *)malloc((*out_size) * sizeof(float));
  if (!signal) {
    *out_size = 0;
    return NULL;
  }

  float pulse_width = fill_ratio * period;

  for (size_t i = 0; i < *out_size; i++) {
    float t = (float)i * dt;
    float phase = fmodf(t, period); // Reuse t instead of recalculating

    size_t period_index = (size_t)(t / period);

    // Bounds check
    if (period_index >= n_periods) {
      period_index = n_periods - 1; // Clamp instead of zero
    }

    signal[i] = (phase < pulse_width ? 1.0f : 0.0f) * encoded_row[period_index];
  }

  return signal;
}

uint16_t *generate_dac_waveform(const uint8_t *image_vector, size_t size,
                                float ref_voltage, float max_amplitude,
                                uint32_t clock_freq_hz, float period,
                                float fill_ratio, size_t *out_size) {
  // Validation
  if (!image_vector || !out_size || size == 0 || max_amplitude <= 0) {
    if (out_size)
      *out_size = 0;
    return NULL;
  }

  // Encode image vector to voltage amplitudes
  float *encoded_row = (float *)malloc(size * sizeof(float));
  if (!encoded_row) {
    *out_size = 0;
    return NULL;
  }

  for (size_t i = 0; i < size; i++) {
    // Normalize to [0, 1] and scale to max_amplitude
    float normalized = (float)image_vector[i] / 255.0f;
    float scaled = normalized * max_amplitude;

    // Clamp to [0, max_amplitude] (though normalization should prevent issues)
    if (scaled > max_amplitude) {
      scaled = max_amplitude;
    } else if (scaled < 0) {
      scaled = 0;
    }

    encoded_row[i] = scaled;
  }

  // Generate analog waveform
  float *waveform = generate_waveform(encoded_row, size, clock_freq_hz, period,
                                      fill_ratio, out_size);
  free(encoded_row);

  if (!waveform) {
    return NULL;
  }

  // Convert to DAC values (16-bit: 0-65535)
  uint16_t *dac_waveform = (uint16_t *)malloc((*out_size) * sizeof(uint16_t));
  if (!dac_waveform) {
    free(waveform);
    *out_size = 0;
    return NULL;
  }

  for (size_t i = 0; i < *out_size; i++) {
    float amplitude = waveform[i];

    // Clamp to [0, max_amplitude]
    if (amplitude > max_amplitude) {
      amplitude = max_amplitude;
    } else if (amplitude < 0) {
      amplitude = 0;
    }

    // Convert to 16-bit DAC value with proper clamping
    float dac_value = (amplitude / ref_voltage) * 65535.0f;

    // Ensure we don't overflow uint16_t
    if (dac_value > 65535.0f) {
      dac_waveform[i] = 65535;
    } else if (dac_value < 0.0f) {
      dac_waveform[i] = 0;
    } else {
      dac_waveform[i] = (uint16_t)dac_value;
    }
  }

  free(waveform);
  return dac_waveform;
}

/*
Generate DAC value from amplitude
*/
uint16_t generate_dac_value(float amplitude, float max_amplitude) {
  if (amplitude > max_amplitude) {
    amplitude = max_amplitude;
  } else if (amplitude < 0) {
    amplitude = 0;
  }
  return (uint16_t)((amplitude / max_amplitude) * 65535.0f);
}

uint16_t byte_to_dac_value(uint8_t amplitude, float max_pulse_amplitude,
                           float max_amplitude) {

  float scaled_amplitude = ((float)amplitude / 255.0f) * max_pulse_amplitude;
  return generate_dac_value(scaled_amplitude, max_amplitude);
}
