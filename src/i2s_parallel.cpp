#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <rom/gpio.h>
#include <soc/gpio_sig_map.h>
#include <math.h>

#include <i2s_parallel.h>
#include "driver/i2s.h"
#include "hal/i2s_hal.h"
#include "hal/i2s_ll.h"

static double ClockTimeNS = 0;

static void SetClockValues(i2s_dev_t* dev, i2s_parallel_config_t * conf, uint32_t SampleWidth);

static i2s_dev_t* I2S[I2S_NUM_MAX] = {&I2S0, &I2S1};

static i2s_hal_context_t I2S_hal;

static intr_handler_t irq_hndlr;

//----------------------------------------------------------------------------
static void IRAM_ATTR ISR_Handler (void * arg)
{
    // i2s_obj_t *p_i2s = static_cast<i2s_obj_t *> (arg);
    // uint32_t status = i2s_hal_get_intr_status (static_cast<i2s_hal_context_t*>(arg));
    i2s_dev_t * dev = arg;
    uint32_t status = dev->int_st.val;

    do // once
    {
        if (status == 0)
        {
            // ignore spurious interrupts
            break;
        }

        // i2s_hal_clear_intr_status (& (p_i2s->hal), status);
        dev->int_clr.val = status;

        // is there a DMA buffer complete interrupt?
        if (status & I2S_OUT_EOF_INT_ST)
        {
            // get the last completed buffer
            // i2s_hal_get_out_eof_des_addr (& (p_i2s->hal), &finish_desc);

            // invoke the next level up handler
            (*irq_hndlr)((void*)(dev->out_eof_des_addr));
        }

    } while (false);

} // ISR_Handler

static inline int get_bus_width(i2s_parallel_sample_width_t width) {
  switch(width) {
    case I2S_PARALLEL_WIDTH_8:
      return 8;
    case I2S_PARALLEL_WIDTH_16:
      return 16;
    case I2S_PARALLEL_WIDTH_24:
      return 24;
    default:
      return -ESP_ERR_INVALID_ARG;
  }
}

void i2s_parallel_iomux_set_signal(int gpio, int signal)
{
    if(gpio < 0)
    {
        return;
    }
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction(gpio, GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(gpio, signal, false, false);
}

static void dma_reset(i2s_hal_context_t * dev)
{
  i2s_hal_tx_reset_dma(dev); 
  i2s_hal_rx_reset_dma(dev);
}

static void fifo_reset(i2s_hal_context_t * dev)
{
  i2s_hal_reset_tx_fifo(dev);
  i2s_hal_reset_rx_fifo(dev);
}

static void dev_reset(i2s_hal_context_t * dev)
{
  fifo_reset(dev);
  dma_reset(dev);
  i2s_hal_tx_reset(dev);
  i2s_hal_rx_reset(dev);
} // dev_reset

esp_err_t i2s_parallel_driver_install(i2s_port_t port, i2s_parallel_config_t* conf, bool invert_clk, intr_handler_t _irq_hndlr, void* priv)
{
  if(port < I2S_NUM_0 || port >= I2S_NUM_MAX)
  {
    // printf("Invalid port number %u\n", port);
    return ESP_ERR_INVALID_ARG;
  }
  if(conf->sample_width < I2S_PARALLEL_WIDTH_8 || conf->sample_width >= I2S_PARALLEL_WIDTH_MAX) {
    // printf("Invalid width %u\n", conf->sample_width);
    return ESP_ERR_INVALID_ARG;
  }
  if(conf->sample_rate > I2S_PARALLEL_CLOCK_HZ || conf->sample_rate < 1) {
    // printf("Invalid Sample Rate %u\n", conf->sample_width);
    return ESP_ERR_INVALID_ARG;
  }
  
  uint32_t clk_div_main = I2S_PARALLEL_CLOCK_HZ / conf->sample_rate / i2s_parallel_get_memory_width(port, conf->sample_width);
  if(clk_div_main < 2 || clk_div_main > 0xFF)
  {
    // printf("Invalid target sample rate %u\n", clk_div_main);
    return ESP_ERR_INVALID_ARG;
  }

  volatile int iomux_signal_base;
  volatile int iomux_clock;
  int irq_source;
  // Initialize I2S peripheral
  if (port == I2S_NUM_0)
  {
      periph_module_reset(PERIPH_I2S0_MODULE);
      periph_module_enable(PERIPH_I2S0_MODULE);
      iomux_clock = I2S0O_WS_OUT_IDX;
      irq_source = ETS_I2S0_INTR_SOURCE;

      switch(conf->sample_width)
      {
        case I2S_PARALLEL_WIDTH_8:
        case I2S_PARALLEL_WIDTH_16:
          iomux_signal_base = I2S0O_DATA_OUT8_IDX;
          break;
        case I2S_PARALLEL_WIDTH_24:
          iomux_signal_base = I2S0O_DATA_OUT0_IDX;
          break;
        case I2S_PARALLEL_WIDTH_MAX:
          return ESP_ERR_INVALID_ARG;
      }
  }
  else
  {
      periph_module_reset(PERIPH_I2S1_MODULE);
      periph_module_enable(PERIPH_I2S1_MODULE);
      iomux_clock = I2S1O_WS_OUT_IDX;
      irq_source = ETS_I2S1_INTR_SOURCE;

      switch(conf->sample_width)
      {
        case I2S_PARALLEL_WIDTH_16:
          iomux_signal_base = I2S1O_DATA_OUT8_IDX;
          break;
        case I2S_PARALLEL_WIDTH_8:
        case I2S_PARALLEL_WIDTH_24:
          iomux_signal_base = I2S1O_DATA_OUT0_IDX;
          break;
        case I2S_PARALLEL_WIDTH_MAX:
        default:
          return ESP_ERR_INVALID_ARG;
      }
  }

  // Setup GPIOs
  int bus_width = get_bus_width(conf->sample_width);
  for(int i = 0; i < bus_width; i++)
  {
    i2s_parallel_iomux_set_signal(conf->gpios_bus[i], iomux_signal_base + i);
  }
  // TBD Restore i2s_parallel_iomux_set_signal(conf->gpio_clk, iomux_clock);

  i2s_hal_init(&I2S_hal, port);

  // Setup I2S peripheral
  dev_reset();

  // Set i2s mode to LCD mode
  I2S_hal.dev->conf2.val = 0;
  i2s_ll_enable_camera_en(I2S_hal.dev, false);
  i2s_ll_enable_lcd_tx_wrx2(I2S_hal.dev, false);
  i2s_ll_enable_lcd_tx_sdx2(I2S_hal.dev, false);
  i2s_ll_enable_data_enable_test(I2S_hal.dev, false);
  i2s_ll_enable_data(I2S_hal.dev, false);
  i2s_ll_enable_lcd_mode(I2S_hal.dev, true);
  i2s_ll_enable_ext_adc_start(I2S_hal.dev);
  i2s_ll_enable_inter_valid(I2S_hal.dev);

  // Setup i2s clock
  I2S_hal.dev->sample_rate_conf.val = 0;
  // Third stage config, width of data to be written to IO (I think this should always be the actual data width?)
  I2S_hal.dev->sample_rate_conf.rx_bits_mod = bus_width;
  I2S_hal.dev->sample_rate_conf.tx_bits_mod = bus_width;
  I2S_hal.dev->sample_rate_conf.rx_bck_div_num = 1;
  I2S_hal.dev->sample_rate_conf.tx_bck_div_num = 1;

  SetClockValues(conf, i2s_parallel_get_memory_width(conf->sample_width));

  // Some fifo conf I don't quite understand 
  I2S_hal.dev->fifo_conf.val = 0;
  // Dictated by datasheet
  I2S_hal.dev->fifo_conf.rx_fifo_mod_force_en = 1;
  I2S_hal.dev->fifo_conf.tx_fifo_mod_force_en = 1;
  // Not really described for non-pcm modes, although datasheet states it should be set correctly even for LCD mode
  // First stage config. Configures how data is loaded into fifo
  if(conf->sample_width == I2S_PARALLEL_WIDTH_24)
  {
    // Mode 0, single 32-bit channel, linear 32 bit load to fifo
    I2S_hal.dev->fifo_conf.tx_fifo_mod = 3;
  }
  else
  {
    // Mode 1, single 16-bit channel, load 16 bit sample(*) into fifo and pad to 32 bit with zeros
    // *Actually a 32 bit read where two samples are read at once. Length of fifo must thus still be word-alligned
    I2S_hal.dev->fifo_conf.tx_fifo_mod = 1;
  }
  // Probably relevant for buffering from the DMA controller
  I2S_hal.dev->fifo_conf.rx_data_num = 32; //Thresholds. 
  I2S_hal.dev->fifo_conf.tx_data_num = 32;
  // Enable DMA support
  I2S_hal.dev->fifo_conf.dscr_en = 1;

  I2S_hal.dev->conf1.val = 0;
  I2S_hal.dev->conf1.tx_stop_en = 1;
  I2S_hal.dev->conf1.tx_pcm_bypass = 1;
  
  // Second stage config
  I2S_hal.dev->conf_chan.val = 0;
  // Tx in mono mode, read 32 bit per sample from fifo
  I2S_hal.dev->conf_chan.tx_chan_mod = 1;
  I2S_hal.dev->conf_chan.rx_chan_mod = 1;
  
  I2S_hal.dev->conf.tx_right_first = !!invert_clk;
  I2S_hal.dev->conf.rx_right_first = !!invert_clk;
  
  I2S_hal.dev->timing.val = 0;

  if(_irq_hndlr)
  {
    esp_err_t err = esp_intr_alloc(irq_source, ESP_INTR_FLAG_IRAM, ISR_Handler, (void*)&I2S_hal, NULL);
    if(err)
    {
      return err;
    }
    irq_hndlr = _irq_hndlr;
    I2S_hal.dev->int_ena.val = 0;
    I2S_hal.dev->int_ena.out_eof = 1;
  }

  return ESP_OK;
}

static void SetClockValues(i2s_parallel_config_t * conf, uint32_t SampleWidth)
{
    double TargetDivisor = ((((double)I2S_PARALLEL_CLOCK_HZ) / ((double)(conf->sample_rate))) / (double)(SampleWidth));
    // printf("       clock_rate: %u\n", I2S_PARALLEL_CLOCK_HZ);
    // printf("      sample_rate: %u\n", conf->sample_rate);
    // printf("     Sample Width: %u\n", conf->sample_width);
    // printf("        Mem Width: %u\n", SampleWidth);
    // printf("    TargetDivisor: %f\n", TargetDivisor);

    uint8_t integralPart = (uint8_t)TargetDivisor;
    // printf("     integralPart: %u\n", integralPart);
    double fractionalPart = TargetDivisor - (double)integralPart;
    // printf("   fractionalPart: %f\n",  fractionalPart);
    uint32_t denominator = 100.0;
    // printf("      denominator: %u\n", denominator);
    uint32_t numerator = (int)(fractionalPart * denominator);
    // printf("        numerator: %u\n", numerator);
    // printf("      denominator: %u\n", denominator);

    if(numerator == 0)
    {
        denominator = 0.0;
        numerator = 0.0;
    }

    I2S_hal.dev->clkm_conf.val = 0;
    I2S_hal.dev->clkm_conf.clka_en = 0;
    I2S_hal.dev->clkm_conf.clkm_div_a = denominator;
    I2S_hal.dev->clkm_conf.clkm_div_b = numerator;
    I2S_hal.dev->clkm_conf.clkm_div_num = integralPart;

    ClockTimeNS = (double)integralPart;
    if(numerator)
    {
      ClockTimeNS += (double)numerator / (double)denominator;
    }
    // printf("      ClockTimeNS: %f\n", ClockTimeNS);
    ClockTimeNS = (1.0 / ((double)I2S_PARALLEL_CLOCK_HZ)) * ClockTimeNS;
    // printf("      ClockTimeNS: %f\n", ClockTimeNS);
    ClockTimeNS *= 1000000000; // NS in a sec
    // printf("      ClockTimeNS: %f\n", ClockTimeNS);

} // Set clock values

double i2s_parallel_driver_get_clockNS()
{
  return ClockTimeNS;
}

esp_err_t i2s_parallel_send_dma(lldesc_t* dma_descriptor)
{
  // Stop all ongoing DMA operations
  I2S_hal.dev->out_link.stop = 1;
  I2S_hal.dev->out_link.start = 0;
  I2S_hal.dev->conf.tx_start = 0;
  dev_reset(&I2S_hal);

  // Configure DMA burst mode
  I2S_hal.dev->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;
  // Set address of DMA descriptor
  I2S_hal.dev->out_link.addr = (uint32_t)dma_descriptor;
  // Start DMA operation
  I2S_hal.dev->out_link.start = 1;
  I2S_hal.dev->conf.tx_start = 1;

  return ESP_OK;
}
