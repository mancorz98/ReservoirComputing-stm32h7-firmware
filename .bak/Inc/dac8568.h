#ifndef MY_DAC8568_H
#define MY_DAC8568_H
#include <stdint.h>

typedef enum control_bits {
  CMD_WRITE = 0x00,
  CMD_UPDATE = 0x01,
  CMD_WRITE_TO_CLEAR = 0x05,
  CMD_WRITE_UPDATE_ALL = 0x02,
  CMD_WRITE_UPDATE_SELECT = 0x03,
  CMD_POWER_DOWN = 0x04,
  CMD_POWER_UP_INTERNAL_REF = 0b1000,

} ControlBits;

typedef enum channel {
  CHANNEL_A = 0x00,
  CHANNEL_B = 0x01,
  CHANNEL_C = 0x02,
  CHANNEL_D = 0x03,
  CHANNEL_E = 0x04,
  CHANNEL_F = 0x05,
  CHANNEL_G = 0x06,
  CHANNEL_H = 0x07,
  CHANNEL_ALL = 0x0F,
} Channel;

typedef enum feature {
  FEATURE_NO_OPERATION = 0x00,
  FEATURE_SETUP_INTERNAL_REF = 0x01,
} Feature;

typedef struct DACcommand {
  ControlBits control;
  Channel channel;
  uint16_t data;
  Feature feature;
} DACcommand;

typedef struct DACValueCommand {
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
  uint16_t ch4;
  uint16_t ch5;
  uint16_t ch6;
  uint16_t ch7;
  uint16_t ch8;
} DACValueCommand;
;

uint32_t set_command_word(const DACcommand *command);

#endif // MY_DAC8568_H