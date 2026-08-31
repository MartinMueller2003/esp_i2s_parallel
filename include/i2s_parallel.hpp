#pragma once

#include <stdbool.h>
#include <sys/types.h>

#include <freertos/FreeRTOS.h>
#include <driver/i2s.h>

#include <esp_err.h>
#include <rom/lldesc.h>
// #include "hal/i2s_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// #define I2S_PARALLEL_CLOCK_HZ 80000000L
#define I2S_PARALLEL_CLOCK_HZ 160000000L

enum I2sParallelDmaBufferOwner
{
  I2sParallelDmaBufferOwnerDMA = 0,
  I2sParallelDmaBufferOwnerCPU = 1,
};

typedef enum
{
  I2S_PARALLEL_WIDTH_8,
  I2S_PARALLEL_WIDTH_16,
  I2S_PARALLEL_WIDTH_24,
  I2S_PARALLEL_WIDTH_MAX
} i2s_parallel_sample_width_t;

typedef struct
{
  int                         gpio_clk;
  int                         gpios_bus[24]; // The parallel GPIOs to use, set gpio to -1 to disable
  i2s_parallel_sample_width_t sample_width;
  int                         sample_rate;
} i2s_parallel_config_t;

#ifndef I2S_PIN_NO_CHANGE
#define I2S_PIN_NO_CHANGE gpio_num_t(-1)
#endif // ndef I2S_PIN_NO_CHANGE

void i2s_parallel_iomux_set_signal(int gpio, int signal);

esp_err_t i2s_parallel_driver_install(i2s_port_t port, i2s_parallel_config_t* conf, bool invert_clk, intr_handler_t irq_hndlr, void* priv);

esp_err_t i2s_parallel_send_dma(i2s_port_t port, lldesc_t* dma_descriptor);

i2s_dev_t* i2s_parallel_get_dev(i2s_port_t port);

double i2s_parallel_driver_get_clockNS();

static inline int i2s_parallel_get_memory_width(i2s_port_t port, i2s_parallel_sample_width_t width) {
  switch(width)
  {
    case I2S_PARALLEL_WIDTH_8:
      // IS21 supports space saving single byte 8 bit parallel access
      if(port == I2S_NUM_1)
      {
        return 1;
      }
    case I2S_PARALLEL_WIDTH_16:
      return 2;
    case I2S_PARALLEL_WIDTH_24:
      return 4;
    default:
      return -ESP_ERR_INVALID_ARG;
  }
}

#ifdef __cplusplus
}
#endif
