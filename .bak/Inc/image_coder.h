#ifndef MY_IMAGE_CODER_H
#define MY_IMAGE_CODER_H
#include <stdint.h>
#include <stdio.h>

typedef struct Image {
  size_t width;
  size_t height;
  uint8_t *data;
} Image;

typedef struct EncodedImage {
  size_t width;
  size_t height;
  float *data;
} EncodedImage;

void encode_image(Image *image, float max_amplitude,
                  EncodedImage *encoded_image);

// Returns dynamically allocated waveform. Caller must free().
// Sets *out_size to the number of samples generated.
float *generate_waveform(const float *encoded_row, size_t row_size,
                         uint32_t clock_freq_hz, float period, float fill_ratio,
                         size_t *out_size);

uint16_t generate_dac_value(float amplitude, float max_amplitude);
void encode_image_vector(uint8_t *image_chunk, size_t chunk_size,
                         float max_amplitude, uint8_t *encoded_chunk);

uint16_t *generate_dac_waveform(const uint8_t *image_vector, size_t size,
                                float ref_voltage, float max_amplitude,
                                uint32_t clock_freq_hz, float period,
                                float fill_ratio, size_t *out_size);

#endif // MY_IMAGE_CODER_H