/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dac8568.h"
#include "image_coder.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_BUFFLEN 256

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_NUM_CONVERSIONS 8
#define BUFF_MULTIPLY 8
#define IMAGE_SIZE 8
#define EXPECTED_PACKET_SIZE 71
#define READOUT_VOLTAGE_V 0.15f
#define READOUT_SAMPLES 100

#define APB_CLOCK_HZ 65000000U
#define TIMER8_FREQ_HZ 100000U // 100kHz ADC trigger
#define TIMER1_FREQ_HZ 1000U   // 1kHz DAC trigger

#define DAC_REF_VOLTAGE_V 2.5f
#define ADC_REF_VOLTAGE_V 3.3f
#define PULSE_AMPLITUDE_V 1.5f

#define DAC_PERIOD_S (1.0f / TIMER1_FREQ_HZ) // 0.001s = 1ms
#define SUPPLY_PERIOD_MS 10.0f               // 10ms

// Calculate compile-time constants
#define SUPPLY_PERIOD_S (SUPPLY_PERIOD_MS / 1000.0f) // 0.01s
#define TOTAL_SUPPLY_CYCLES                                                    \
  ((size_t)(TIMER1_FREQ_HZ * SUPPLY_PERIOD_S)) // 10 cycles
#define EXPECTED_SAMPLES_PER_ROW                                               \
  (IMAGE_SIZE * TOTAL_SUPPLY_CYCLES) // 100 samples

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
uint8_t usbTxBuf[USB_BUFFLEN];
uint16_t usbTxBufLen;

uint8_t usbRxBuf[USB_BUFFLEN];
uint16_t usbRxBufLen;
uint8_t usbDataReceived = 0;
uint8_t imageReceived = 0;

uint8_t image_buffer[IMAGE_SIZE][IMAGE_SIZE];

// Accumulator for fragmented USB packets
volatile uint8_t accumulator[USB_BUFFLEN];
volatile uint16_t accum_len = 0;
volatile uint8_t packet_ready = 0;

// Debug counters
volatile uint32_t rx_callback_count = 0;
volatile uint32_t packets_processed = 0;

typedef enum {
  DAC_STATE_IDLE,
  DAC_STATE_SENDING_CHANNELS,
  DAC_STATE_SENDING_UPDATE,
  DAC_STATE_COMPLETE,
  DAC_STATE_WAITING_TIMER,
  DAC_STATE_BUSY,
  DAC_STATE_WRITE_COMPLETE, // ← New: Write phase done
  DAC_STATE_READOUT_INIT,   // ← New: Preparing readout
  DAC_STATE_READOUT_ACTIVE, // ← New: Reading ADC
  DAC_STATE_ZEROING,        //

} DAC_State_t;

volatile DAC_State_t dac_state = DAC_STATE_IDLE;
volatile uint8_t current_channel = 0;
volatile size_t current_pixel = 0;
volatile size_t total_pixels = 0;
volatile DACValueCommand *current_dac_values = NULL;

volatile uint32_t readout_sample_count = 0;
volatile uint8_t readout_complete = 0;

volatile uint8_t readout_pulse_channel = 0;
volatile uint8_t readout_pulse_complete = 0;
volatile uint8_t callback_count = 0;

// Channel lookup table
const uint8_t channels[8] = {CHANNEL_A, CHANNEL_B, CHANNEL_C, CHANNEL_D,
                             CHANNEL_E, CHANNEL_F, CHANNEL_G, CHANNEL_H};
const float adc_period = 1.0f / (float)TIMER8_FREQ_HZ;
const float dac_period = 1.0f / (float)TIMER1_FREQ_HZ;

static const float supply_period = SUPPLY_PERIOD_S;
static const size_t total_supply_cycles = TOTAL_SUPPLY_CYCLES;
static const size_t expected_samples_per_row = EXPECTED_SAMPLES_PER_ROW;
static DACValueCommand dac_values_timed[EXPECTED_SAMPLES_PER_ROW];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM8_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t dac_ready = 1; // ← CRITICAL: Add this!

__attribute__((section(
    ".adcarray"))) uint16_t ADC_VAL[ADC_NUM_CONVERSIONS * READOUT_SAMPLES];

__attribute__((section(".adcarray"))) uint32_t TX_buffer[1] = {0};

volatile float value[ADC_NUM_CONVERSIONS];
int count = 0;

uint16_t get_channel_value(volatile DACValueCommand *dac_val, uint8_t ch) {
  switch (ch) {
  case 0:
    return dac_val->ch1;
  case 1:
    return dac_val->ch2;
  case 2:
    return dac_val->ch3;
  case 3:
    return dac_val->ch4;
  case 4:
    return dac_val->ch5;
  case 5:
    return dac_val->ch6;
  case 6:
    return dac_val->ch7;
  case 7:
    return dac_val->ch8;
  default:
    return 0;
  }
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  // Clamp input to valid range
  if (x < in_min)
    return out_min;
  if (x > in_max)
    return out_max;

  // Use long long to prevent overflow during multiplication
  return (long)(((long long)(x - in_min) * (out_max - out_min)) /
                (in_max - in_min)) +
         out_min;
}

float mapf(long x, long in_min, long in_max, float out_min, float out_max) {
  // Clamp input to valid range
  if (x < in_min)
    return out_min;
  if (x > in_max)
    return out_max;

  // Calculate with float precision
  return (float)(x - in_min) * (out_max - out_min) / (in_max - in_min) +
         out_min;
}

size_t create_csv_record(char *buffer, size_t buffer_size,
                         size_t num_decimals) {
  if (buffer_size == 0)
    return 0;

  size_t offset = 0;

  // Optimization: Pre-calculate multiplier to avoid pow() in loop
  int multiplier = 1;
  for (size_t k = 0; k < num_decimals; k++) {
    multiplier *= 10;
  }

  for (size_t i = 0; i < ADC_NUM_CONVERSIONS; i++) {
    float val = value[i];

    // 1. Handle Sign explicitly to fix the "-0.5" bug
    char sign = '\0';
    if (val < 0) {
      sign = '-';
      val = -val; // Make positive for calculation
    }

    // 2. Rounding and Integer/Frac split
    // Adding 0.5f before casting effects "rounding to nearest" rather than
    // truncation
    int whole = (int)val;
    int frac = (int)((val - whole) * multiplier + 0.5f);

    // Handle edge case: rounding rolls over (e.g., 1.999 -> 2.000)
    if (frac >= multiplier) {
      whole++;
      frac = 0;
    }

    // 3. Dynamic Format String Logic
    // We cannot use %03d because num_decimals varies.
    // We verify space exists for worst case (sign + whole + . + frac + comma)
    int written;

    // Note: '%.*d' allows dynamic width for the integer precision (padding
    // zeros)
    written = snprintf(
        buffer + offset, buffer_size - offset, "%c%d.%.*d",
        (sign ? sign : '\0'), // Hack to print sign only if needed (or nothing)
        whole, (int)num_decimals, frac);

    // Note: snprintf returns what it *would* have written.
    // We must check if it actually fit.
    if (written < 0 || (size_t)written >= buffer_size - offset) {
      // Buffer full or error
      buffer[buffer_size - 1] = '\0'; // Ensure termination
      return offset;
    }
    offset += written;

    // 4. Append Comma
    if (i < ADC_NUM_CONVERSIONS - 1) {
      if (offset + 1 < buffer_size) {
        buffer[offset++] = ',';
      } else {
        buffer[buffer_size - 1] = '\0';
        return offset;
      }
    }
  }

  // 5. Safe Line Termination (Checking for 3 chars: \r, \n, \0)
  if (offset + 2 < buffer_size) {
    buffer[offset++] = '\r';
    buffer[offset++] = '\n';
    buffer[offset] = '\0';
  } else {
    // Force null termination if we ran out of space for CRLF
    buffer[buffer_size - 1] = '\0';
  }

  return offset;
}

bool receive_image_8x8(uint8_t *buffer, uint16_t buffer_size,
                       uint8_t image_out[IMAGE_SIZE][IMAGE_SIZE]) {
  // Verify header
  if (buffer[0] != 0xAA || buffer[1] != 0x55) {
    return false;
  }

  // Parse dimensions (2 bytes each, little-endian)
  uint16_t h = buffer[2] | (buffer[3] << 8);
  uint16_t w = buffer[4] | (buffer[5] << 8);

  // Verify it's 8x8
  if (h != IMAGE_SIZE || w != IMAGE_SIZE) {
    return false;
  }

  // Check buffer size
  if (buffer_size < 71) {
    return false;
  }

  // Calculate checksum
  uint8_t calc_checksum = 0;
  for (uint8_t i = 0; i < 64; i++) {
    calc_checksum += buffer[6 + i];
  }

  if (calc_checksum != buffer[70]) {
    return false; // Checksum mismatch
  }

  // Copy image data (starts at byte 6)
  memcpy(image_out, &buffer[6], 64);

  return true;
}

float median_of_4(uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
  // Sort 4 values and return average of middle two
  uint16_t arr[4] = {a, b, c, d};

  // Simple bubble sort for 4 elements
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3 - i; j++)
      if (arr[j] > arr[j + 1]) {
        uint16_t temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }

  // Return median (average of middle two values)
  return ((arr[1] + arr[2]) / 2.0f) * ADC_REF_VOLTAGE_V / 65535.0f;
}

void Set_DAC_Outputs_Zero(void) {
  DACcommand cmd;
  cmd.control = CMD_WRITE_UPDATE_ALL;
  cmd.feature = FEATURE_NO_OPERATION;
  cmd.channel = CHANNEL_ALL;
  cmd.data = 0x0000; // 0V

  TX_buffer[0] = set_command_word(&cmd);

  // Use blocking transmission (called after readout complete)
  HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)TX_buffer, 1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc != &hadc2)
    return;
  callback_count++;
  if (dac_state == DAC_STATE_READOUT_ACTIVE) {
    // ✅ Circular DMA complete = 100 scans done

    // Just stop TIM8 (ADC keeps running but no new triggers)
    HAL_TIM_Base_Stop(&htim8);

    // Zero DAC
    DACcommand cmd;
    cmd.control = CMD_WRITE_UPDATE_ALL;
    cmd.channel = CHANNEL_ALL;
    cmd.data = 0x0000;

    TX_buffer[0] = set_command_word(&cmd);
    dac_state = DAC_STATE_ZEROING;
    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)TX_buffer, 1);

    readout_complete = 1;
  }
}

void Send_Readout_Pulse_Start(void) {
  // Reset state
  readout_pulse_channel = 0;
  readout_pulse_complete = 0;

  // Convert 0.15V to DAC value
  uint16_t readout_dac_value =
      (uint16_t)(READOUT_VOLTAGE_V / DAC_REF_VOLTAGE_V * 65535.0f);

  // Send first channel
  DACcommand cmd;
  cmd.control = CMD_WRITE;
  cmd.feature = FEATURE_NO_OPERATION;
  cmd.channel = channels[0];
  cmd.data = readout_dac_value;

  TX_buffer[0] = set_command_word(&cmd);
  HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)TX_buffer, 1);

  readout_pulse_channel++;
}

void Send_Readout_Pulse_Continue(void) {
  uint16_t readout_dac_value =
      (uint16_t)(READOUT_VOLTAGE_V / DAC_REF_VOLTAGE_V * 65535.0f);
  DACcommand cmd;

  if (readout_pulse_channel < 8) {
    // Send next channel
    cmd.control = CMD_WRITE;
    cmd.feature = FEATURE_NO_OPERATION;
    cmd.channel = channels[readout_pulse_channel];
    cmd.data = readout_dac_value;

    TX_buffer[0] = set_command_word(&cmd);
    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)TX_buffer, 1);

    readout_pulse_channel++;

  } else if (readout_pulse_channel == 8) {
    // All channels sent, send UPDATE
    cmd.control = CMD_UPDATE;
    cmd.channel = CHANNEL_ALL;
    cmd.data = 0;

    TX_buffer[0] = set_command_word(&cmd);
    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)TX_buffer, 1);

    readout_pulse_channel++;

  } else {
    // UPDATE sent, done
    readout_pulse_complete = 1;
  }
}

void Start_Readout_Phase(void) {
  // Reset readout state
  // readout_sample_count = 0;
  readout_complete = 0;
  dac_state = DAC_STATE_READOUT_INIT;

  // ✅ Start non-blocking readout pulse
  Send_Readout_Pulse_Start();

  // Rest will happen in SPI callback
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi != &hspi1)
    return;

  DACcommand cmd;

  switch (dac_state) {

  case DAC_STATE_SENDING_CHANNELS:
    if (current_channel < 8) {
      cmd.control = CMD_WRITE;
      cmd.feature = FEATURE_NO_OPERATION;
      cmd.channel = channels[current_channel];
      cmd.data = get_channel_value(&current_dac_values[current_pixel],
                                   current_channel);

      TX_buffer[0] = set_command_word(&cmd);
      HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)TX_buffer, 1);

      current_channel++;
    } else {
      dac_state = DAC_STATE_SENDING_UPDATE;

      cmd.control = CMD_UPDATE;
      cmd.channel = CHANNEL_ALL;
      cmd.feature = FEATURE_NO_OPERATION;
      cmd.data = 0;

      TX_buffer[0] = set_command_word(&cmd);
      HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)TX_buffer, 1);
    }
    break;

  case DAC_STATE_SENDING_UPDATE:
    current_pixel++;
    dac_state = DAC_STATE_WAITING_TIMER;
    break;

  case DAC_STATE_READOUT_INIT:
    Send_Readout_Pulse_Continue();

    if (readout_pulse_complete) {
      // Start trigger
      HAL_TIM_Base_Start(&htim8);

      dac_state = DAC_STATE_READOUT_ACTIVE;
    }
    break;

  case DAC_STATE_ZEROING: // ✅ Add this case
    // Zero command complete
    dac_state = DAC_STATE_COMPLETE;
    break;

  default:
    break;
  }
}

HAL_StatusTypeDef DAC_Start_Update(DACValueCommand *dac_values,
                                   size_t pixel_count) {
  // Check if already busy
  if (dac_state != DAC_STATE_IDLE && dac_state != DAC_STATE_COMPLETE) {
    return HAL_BUSY;
  }

  // Initialize state
  dac_state = DAC_STATE_SENDING_CHANNELS;
  current_channel = 0;
  current_pixel = 0;
  total_pixels = pixel_count;
  current_dac_values = dac_values;

  // Send first channel of first pixel
  DACcommand cmd;
  cmd.control = CMD_WRITE;
  cmd.feature = FEATURE_NO_OPERATION;
  cmd.channel = CHANNEL_A;
  cmd.data = get_channel_value(&dac_values[0], 0);
  TX_buffer[0] = set_command_word(&cmd);

  HAL_StatusTypeDef status =
      HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)TX_buffer, 1);

  if (status != HAL_OK) {
    dac_state = DAC_STATE_IDLE;
    return status;
  }

  current_channel++;

  return HAL_OK;
}

HAL_StatusTypeDef DAC_Start_Timer_Sequence(DACValueCommand *dac_values,
                                           size_t pixel_count) {
  // ✅ Check if busy (allow IDLE or COMPLETE)
  if (dac_state != DAC_STATE_IDLE && dac_state != DAC_STATE_COMPLETE) {
    return HAL_BUSY;
  }

  // Initialize state
  dac_state = DAC_STATE_WAITING_TIMER;
  current_channel = 0;
  current_pixel = 0;
  total_pixels = pixel_count;
  current_dac_values = dac_values;

  // Start TIM1 with interrupt
  HAL_TIM_Base_Start_IT(&htim1);

  return HAL_OK;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  __NOP();

  if (htim->Instance == TIM1) {

    if (dac_state == DAC_STATE_WAITING_TIMER) {

      if (current_pixel < total_pixels) {
        // Continue write phase
        DACcommand cmd;
        cmd.control = CMD_WRITE;
        cmd.channel = CHANNEL_A;
        cmd.data = get_channel_value(&current_dac_values[current_pixel], 0);

        TX_buffer[0] = set_command_word(&cmd);
        dac_state = DAC_STATE_SENDING_CHANNELS;
        current_channel = 1;

        HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)TX_buffer, 1);

      } else {
        // Write phase complete - transition to readout
        HAL_TIM_Base_Stop_IT(&htim1);
        dac_state = DAC_STATE_WRITE_COMPLETE;

        // ✅ Start readout phase
        Start_Readout_Phase();
      }
    }
  }
}

void DAC_Stop_Timer_Sequence(void) {
  HAL_TIM_Base_Stop_IT(&htim1);
  dac_state = DAC_STATE_IDLE;
  current_pixel = 0;
}

uint8_t DAC_Is_Busy(void) {
  return (dac_state != DAC_STATE_IDLE && dac_state != DAC_STATE_COMPLETE);
}
uint32_t DAC_Get_Current_Pixel(void) { return current_pixel; }

float DAC_Get_Progress_Percent(void) {
  if (total_pixels == 0)
    return 0.0f;
  return ((float)current_pixel / (float)total_pixels) * 100.0f;
}
void DAC_Reset(void) {
  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIM_Base_Stop(&htim8);
  // ❌ DON'T stop ADC - let it keep running

  dac_state = DAC_STATE_IDLE;
  current_channel = 0;
  current_pixel = 0;
  total_pixels = 0;
  current_dac_values = NULL;

  readout_complete = 0;
  readout_pulse_channel = 0;
  readout_pulse_complete = 0;
}

void USB_RXCallback(uint8_t *buf, uint32_t len) {
  rx_callback_count++;

  // Accumulate incoming data
  for (uint32_t i = 0; i < len && accum_len < USB_BUFFLEN; i++) {
    accumulator[accum_len++] = buf[i];
  }

  // Search for complete packet (71 bytes starting with 0xAA 0x55)
  if (accum_len >= EXPECTED_PACKET_SIZE) {
    for (uint16_t i = 0; i <= accum_len - EXPECTED_PACKET_SIZE; i++) {
      if (accumulator[i] == 0xAA && accumulator[i + 1] == 0x55) {
        // Found header! Copy complete packet
        memcpy(usbRxBuf, &accumulator[i], EXPECTED_PACKET_SIZE);
        usbRxBufLen = EXPECTED_PACKET_SIZE;
        packet_ready = 1;

        // Remove processed data from accumulator
        uint16_t remaining = accum_len - (i + EXPECTED_PACKET_SIZE);
        if (remaining > 0) {
          memmove((void *)accumulator, &accumulator[i + EXPECTED_PACKET_SIZE],
                  remaining);
          accum_len = remaining;
        } else {
          accum_len = 0;
        }

        break;
      }
    }

    // Prevent overflow - reset if no valid packet found
    if (accum_len >= USB_BUFFLEN - 64) {
      accum_len = 0;
    }
  }
}

void process_image(uint8_t image[IMAGE_SIZE][IMAGE_SIZE]) {
  static DACValueCommand
      dac_values[IMAGE_SIZE * 10]; // Static to avoid stack overflow
  size_t out_size =
      IMAGE_SIZE * (size_t)(TIMER1_FREQ_HZ * supply_period); // 100ms

  // Generate waveforms for all rows
  for (size_t i = 0; i < IMAGE_SIZE; i++) {
    uint16_t *dac_waveform = generate_dac_waveform(
        image[i], IMAGE_SIZE, DAC_REF_VOLTAGE_V, PULSE_AMPLITUDE_V,
        TIMER1_FREQ_HZ, supply_period, 0.5f, &out_size);

    for (size_t j = 0; j < out_size; j++) {
      switch (i) {
      case 0:
        dac_values[j].ch1 = dac_waveform[j];
        break;
      case 1:
        dac_values[j].ch2 = dac_waveform[j];
        break;
      case 2:
        dac_values[j].ch3 = dac_waveform[j];
        break;
      case 3:
        dac_values[j].ch4 = dac_waveform[j];
        break;
      case 4:
        dac_values[j].ch5 = dac_waveform[j];
        break;
      case 5:
        dac_values[j].ch6 = dac_waveform[j];
        break;
      case 6:
        dac_values[j].ch7 = dac_waveform[j];
        break;
      case 7:
        dac_values[j].ch8 = dac_waveform[j];
        break;
      }
    }
    free(dac_waveform);
  }

  // Start non-blocking DAC update
  if (DAC_Start_Update(dac_values, out_size) != HAL_OK) {
    // Handle error
    Error_Handler();
  }

  // Function returns immediately!
  // DMA callback will handle everything
}
void process_image_timed(uint8_t image[IMAGE_SIZE][IMAGE_SIZE]) {
  // ✅ Use the file-scope buffer instead of local static
  size_t out_size = EXPECTED_SAMPLES_PER_ROW;

  // Generate waveforms for all rows
  for (size_t row = 0; row < IMAGE_SIZE; row++) {
    size_t actual_samples = out_size;

    uint16_t *dac_waveform = generate_dac_waveform(
        image[row], IMAGE_SIZE, DAC_REF_VOLTAGE_V, PULSE_AMPLITUDE_V,
        TIMER1_FREQ_HZ, supply_period, 0.5f, &actual_samples);

    if (dac_waveform == NULL) {
      Error_Handler();
      return;
    }

    if (actual_samples > EXPECTED_SAMPLES_PER_ROW) {
      free(dac_waveform);
      Error_Handler();
      return;
    }

    if (row == 0) {
      out_size = actual_samples;
    }

    // Direct pointer to the channel for this row
    uint16_t *channel_base = ((uint16_t *)dac_values_timed) + row;

    for (size_t j = 0; j < actual_samples; j++) {
      channel_base[j * 8] = dac_waveform[j];
    }

    free(dac_waveform);
  }

  readout_complete = 0;
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  if (DAC_Start_Timer_Sequence(dac_values_timed, out_size) != HAL_OK) {
    Error_Handler();
  }
}

void process_usb_packet(void) {
  if (!packet_ready) {
    return;
  }

  // ✅ Check if system is already busy
  if (DAC_Is_Busy()) {
    // System busy, can't process new packet yet
    // Keep packet_ready flag set for retry
    return;
  }

  packet_ready = 0;
  packets_processed++;

  if (receive_image_8x8(usbRxBuf, usbRxBufLen, image_buffer)) {
    uint8_t ack = 0x06;
    CDC_Transmit_FS(&ack, 1);

    // Safe to start - we checked DAC_Is_Busy() above
    process_image_timed(image_buffer);

  } else {
    uint8_t nack = 0x15;
    CDC_Transmit_FS(&nack, 1);

    DAC_Reset(); // Clean up state
  }
}

void Process_Readout_Data(void) {
  // Average all samples per channel in scan mode
  for (uint8_t ch = 0; ch < ADC_NUM_CONVERSIONS; ch++) {
    float sum_channel = 0.0f;

    // In scan mode: data is [CH0,CH1,...,CH7, CH0,CH1,...,CH7, ...]
    for (uint32_t sample = 0; sample < READOUT_SAMPLES; sample++) {
      // ✅ Your original indexing was correct!
      uint16_t adc_raw = ADC_VAL[ch + sample * ADC_NUM_CONVERSIONS];
      sum_channel += (adc_raw * ADC_REF_VOLTAGE_V) / 65535.0f;
    }

    value[ch] = sum_channel / (float)READOUT_SAMPLES;
  }

  // Create and send CSV record
  usbTxBufLen = create_csv_record((char *)usbTxBuf, USB_BUFFLEN, 4);
  CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
       /* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0))
    ;
  if (timeout < 0) {
    Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
       /* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* When system initialization is finished, Cortex-M7 will release Cortex-M4
  by means of HSEM notification */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /*Take HSEM */
  HAL_HSEM_FastTake(HSEM_ID_0);
  /*Release HSEM in order to notify the CPU2(CM4)*/
  HAL_HSEM_Release(HSEM_ID_0, 0);
  /* wait until CPU2 wakes up from stop mode */
  timeout = 0xFFFF;
  while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0))
    ;
  if (timeout < 0) {
    Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
       /* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // HAL_SYSCFG_EnableVREFBUF();

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY,
                              ADC_SINGLE_ENDED);

  // HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_FACTOR_LINEARITY_REGOFFSET,
  //                             ADC_SINGLE_ENDED);

  // HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_FACTOR_OFFSET_REGOFFSET,
  //                             ADC_SINGLE_ENDED);

  // if (HAL_OK == HAL_ADCEx_Calibration_Start(&hadc2,
  // ADC_CALIB_OFFSET_LINEARITY,
  //                                           ADC_SINGLE_ENDED)) {
  //   uint32_t calibFactor =
  //       HAL_ADCEx_Calibration_GetValue(&hadc2, ADC_SINGLE_ENDED);
  //   HAL_ADCEx_Calibration_SetValue(&hadc2, ADC_SINGLE_ENDED, calibFactor);
  // }

  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC_VAL,
                    ADC_NUM_CONVERSIONS * READOUT_SAMPLES);

  // HAL_TIM_Base_Start_IT(&htim8); // Start timer for ADC triggering

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time
   * it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity
   */
  BspCOMInit.BaudRate = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits = COM_STOPBITS_1;
  BspCOMInit.Parity = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Disable internal reference (use external 3.3V)
  TX_buffer[0] = 0x07000000;
  HAL_SPI_Transmit(&hspi1, (uint8_t *)TX_buffer, 1, 100); // CS automatic!
  HAL_Delay(1);

  // Enable flexible mode (optional)
  TX_buffer[0] = 0x090A0000;
  HAL_SPI_Transmit(&hspi1, (uint8_t *)TX_buffer, 1, 100); // CS automatic!
  HAL_Delay(10);

  DACcommand cmd;
  cmd.control = CMD_WRITE_UPDATE_ALL;
  cmd.feature = FEATURE_NO_OPERATION;
  cmd.channel = CHANNEL_ALL;
  cmd.data = 0x8000; // Set all channels to midscale
  TX_buffer[0] = set_command_word(&cmd);
  HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)TX_buffer, 1);
  HAL_Delay(10);

  __NOP();
  while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    process_usb_packet();
    // Wait for complete cycle (write + readout)
    if (readout_complete) {
      // Can do other work here
      readout_complete = 0;
      Process_Readout_Data();
      DAC_Reset();
      // DAC_Reset();
    }

    // Process readout data
  }

  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
   */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 8;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_32BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 64 - 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10 - 1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
