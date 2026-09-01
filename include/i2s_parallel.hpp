#pragma once

#include <stdbool.h>
#include <sys/types.h>

#include <freertos/FreeRTOS.h>

#include <esp_err.h>
extern "C"
{
    #include "hal/i2s_hal.h"
    #include <driver/i2s.h>
    #include <rom/lldesc.h>
}

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

typedef void (*I2S_intr_handler_t)(void *arg1, void * buffer);
typedef struct
{
    i2s_port_t port;
    int gpio_clk;
    gpio_num_t gpios_bus[24]; // The parallel GPIOs to use, set gpio to -1 to disable
    i2s_parallel_sample_width_t sample_width;
    int sample_rate;
    bool invert_clk;
    I2S_intr_handler_t irq_hndlr;
    void * arg1;
} i2s_parallel_config_t;

#ifndef I2S_PIN_NO_CHANGE
#define I2S_PIN_NO_CHANGE gpio_num_t (-1)
#endif // ndef I2S_PIN_NO_CHANGE

class esp_i2s_parallel
{
private:
    #define I2S_PARALLEL_CLOCK_HZ 160000000L

    i2s_hal_context_t       I2S_hal;
    double                  ClockTimeNS;
    i2s_parallel_config_t   conf;

    void        SetClockValues ();
    void        dev_reset ();
    void        dma_reset ();
    void        fifo_reset ();
    int         get_memory_width ();
    int         get_bus_width ();

public:
    esp_i2s_parallel () {}
    virtual  ~esp_i2s_parallel () {}

    esp_err_t   driver_install   (i2s_parallel_config_t & conf);
    void        iomux_set_signal (gpio_num_t gpio, int signal);
    esp_err_t   send_dma         (lldesc_t *dma_descriptor);
    double      get_clockNS      () {return ClockTimeNS;}

    // private irq handler called from a static method.
    void ISR_Handler ();

}; // esp_i2s_parallel
