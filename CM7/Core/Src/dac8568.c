
#include "dac8568.h"
#include <stdint.h>
#include <stdio.h>

#define PI 3.142

#define BYTE_TO_BINARY(byte)                                                   \
  ((byte) & 0x80 ? '1' : '0'), ((byte) & 0x40 ? '1' : '0'),                    \
      ((byte) & 0x20 ? '1' : '0'), ((byte) & 0x10 ? '1' : '0'),                \
      ((byte) & 0x08 ? '1' : '0'), ((byte) & 0x04 ? '1' : '0'),                \
      ((byte) & 0x02 ? '1' : '0'), ((byte) & 0x01 ? '1' : '0')

uint32_t set_command_word(const DACcommand *command) {
  uint32_t word = 0;
  word |= (uint32_t)(command->feature) << 0;
  word |= (uint32_t)(command->data & 0xFFFF) << 4;
  word |= (uint32_t)(command->channel & 0x0F) << 20;
  word |= (uint32_t)(command->control & 0x0F) << 24;

  return word;
}
