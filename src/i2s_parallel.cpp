#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <rom/gpio.h>
#include <soc/gpio_sig_map.h>
#include <math.h>

#include "i2s_parallel.hpp"

static i2s_dev_t *I2S[I2S_NUM_MAX] = {&I2S0, &I2S1};

//----------------------------------------------------------------------------
static void IRAM_ATTR IRQ_Handler(void *arg)
{
    static_cast<esp_i2s_parallel*>(arg)->ISR_Handler();
}

//----------------------------------------------------------------------------
void IRAM_ATTR esp_i2s_parallel::ISR_Handler()
{
    uint32_t status = i2s_hal_get_intr_status (&I2S_hal);

    do // once
    {
        if (status == 0)
        {
            // ignore spurious interrupts
            break;
        }

        i2s_hal_clear_intr_status (&I2S_hal, status);

        // is there a DMA buffer complete interrupt?
        if (status & I2S_OUT_EOF_INT_ST)
        {
            // get the last completed buffer
            uint32_t AddressOfLatestBuffer;
            i2s_hal_get_out_eof_des_addr(&I2S_hal, &AddressOfLatestBuffer);

            // invoke the next level up handler
            (*conf.irq_hndlr)(conf.arg1, (void *)(AddressOfLatestBuffer));
        }

    } while (false);

} // ISR_Handler

//----------------------------------------------------------------------------
esp_err_t esp_i2s_parallel::driver_install(i2s_parallel_config_t & _conf)
{
    // save a copy of the config
    conf = _conf;
    if (conf.port < I2S_NUM_0 || conf.port >= I2S_NUM_MAX)
    {
        // printf("Invalid port number %u\n", port);
        return ESP_ERR_INVALID_ARG;
    }
    if (conf.sample_width < I2S_PARALLEL_WIDTH_8 || conf.sample_width >= I2S_PARALLEL_WIDTH_MAX)
    {
        // printf("Invalid width %u\n", conf.sample_width);
        return ESP_ERR_INVALID_ARG;
    }
    if (conf.sample_rate > I2S_PARALLEL_CLOCK_HZ || conf.sample_rate < 1)
    {
        // printf("Invalid Sample Rate %u\n", conf.sample_width);
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t clk_div_main = I2S_PARALLEL_CLOCK_HZ / conf.sample_rate / get_memory_width();
    if (clk_div_main < 2 || clk_div_main > 0xFF)
    {
        // printf("Invalid target sample rate %u\n", clk_div_main);
        return ESP_ERR_INVALID_ARG;
    }

    i2s_hal_init(&I2S_hal, conf.port);

    volatile int iomux_signal_base;
    volatile int iomux_clock;
    int irq_source;
    // Initialize I2S peripheral
    if (conf.port == I2S_NUM_0)
    {
        periph_module_reset(PERIPH_I2S0_MODULE);
        periph_module_enable(PERIPH_I2S0_MODULE);
        iomux_clock = I2S0O_WS_OUT_IDX;
        irq_source = ETS_I2S0_INTR_SOURCE;

        switch (conf.sample_width)
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

        switch (conf.sample_width)
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
    int bus_width = get_bus_width();
    for (int i = 0; i < bus_width; i++)
    {
        iomux_set_signal(conf.gpios_bus[i], iomux_signal_base + i);
    }
    // TBD Restore i2s_parallel_iomux_set_signal(conf.gpio_clk, iomux_clock);

    // Setup I2S peripheral
    dev_reset();

    // Set i2s mode to LCD mode
    i2s_hal_enable_module_clock(&I2S_hal);   // set conf2 to 0;
    i2s_ll_disable_clock(I2S_hal.dev);  // turn clock off
    i2s_ll_enable_lcd(I2S_hal.dev, true);

    SetClockValues();

    // Some fifo conf I don't quite understand
    I2S_hal.dev->fifo_conf.val = 0;
    // Dictated by datasheet
    i2s_ll_tx_force_enable_fifo_mod(I2S_hal.dev, 1);
    i2s_ll_rx_force_enable_fifo_mod(I2S_hal.dev, 1);

    // Not really described for non-pcm modes, although datasheet states it should be set correctly even for LCD mode
    // First stage config. Configures how data is loaded into fifo
    i2s_ll_tx_enable_mono_mode(I2S_hal.dev, true);

    // Probably relevant for buffering from the DMA controller
    I2S_hal.dev->fifo_conf.rx_data_num = 32; // Thresholds.
    I2S_hal.dev->fifo_conf.tx_data_num = 32;

    // Enable DMA support
    i2s_ll_enable_dma(I2S_hal.dev, true);

    I2S_hal.dev->conf1.val = 0;
    i2s_ll_tx_stop_on_fifo_empty(I2S_hal.dev, true);
    i2s_ll_tx_bypass_pcm(I2S_hal.dev, true);

    // Second stage config
    i2s_hal_enable_builtin_adc(&I2S_hal); // zero out register
    // Tx in mono mode, read 32 bit per sample from fifo
    i2s_ll_tx_set_chan_mod(I2S_hal.dev, 1);
    i2s_ll_rx_set_chan_mod(I2S_hal.dev, 1);

    i2s_ll_tx_enable_right_first(I2S_hal.dev, !!conf.invert_clk);
    i2s_ll_rx_enable_right_first(I2S_hal.dev, !!conf.invert_clk);

    I2S_hal.dev->timing.val = 0;

    if (conf.irq_hndlr)
    {
        esp_err_t err = esp_intr_alloc(irq_source, ESP_INTR_FLAG_IRAM, IRQ_Handler, this, NULL);
        if (err)
        {
            return err;
        }
        i2s_ll_enable_intr(I2S_hal.dev, 0, false); // zero out the interrupt register
        i2s_hal_enable_tx_intr(&I2S_hal);
    }

    return ESP_OK;
}

//----------------------------------------------------------------------------
void esp_i2s_parallel::SetClockValues( )
{
    // Setup i2s clock
    I2S_hal.dev->sample_rate_conf.val = 0;

    i2s_ll_tx_set_bits_mod(I2S_hal.dev, get_bus_width());
    // i2s_ll_rx_set_bits_mod(I2S_hal.dev, get_bus_width());
    i2s_ll_tx_set_bck_div_num(I2S_hal.dev, 1);
    i2s_ll_rx_set_bck_div_num(I2S_hal.dev, 1);

    uint32_t SampleWidth = get_memory_width();
    double TargetDivisor = ((((double)I2S_PARALLEL_CLOCK_HZ) / ((double)(conf.sample_rate))) / (double)(SampleWidth));
    // printf("       clock_rate: %u\n", I2S_PARALLEL_CLOCK_HZ);
    // printf("      sample_rate: %u\n", conf.sample_rate);
    // printf("     Sample Width: %u\n", conf.sample_width);
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

    if (numerator == 0)
    {
        denominator = 0.0;
        numerator = 0.0;
    }

    I2S_hal.dev->clkm_conf.val = 0;
    i2s_ll_tx_clk_set_src(I2S_hal.dev, i2s_clock_src_t::I2S_CLK_D2CLK);

    i2s_ll_mclk_div_t ClockValues;
    ClockValues.a = denominator;
    ClockValues.b = numerator;
    ClockValues.mclk_div = integralPart;
    i2s_ll_tx_set_clk(I2S_hal.dev, &ClockValues);

    ClockTimeNS = (double)integralPart;
    if (numerator)
    {
        ClockTimeNS += (double)numerator / (double)denominator;
    }
    // printf("      ClockTimeNS: %f\n", ClockTimeNS);
    ClockTimeNS = (1.0 / ((double)I2S_PARALLEL_CLOCK_HZ)) * ClockTimeNS;
    // printf("      ClockTimeNS: %f\n", ClockTimeNS);
    ClockTimeNS *= 1000000000; // NS in a sec
    // printf("      ClockTimeNS: %f\n", ClockTimeNS);

} // SetClockValues

//----------------------------------------------------------------------------
esp_err_t esp_i2s_parallel::send_dma(lldesc_t *dma_descriptor)
{
    // Stop all ongoing DMA operations
    i2s_ll_tx_stop_link(I2S_hal.dev);
    i2s_ll_tx_stop(I2S_hal.dev);
    dev_reset();

    // Configure DMA burst mode
    I2S_hal.dev->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;
    // Set address of DMA descriptor
    // Start DMA operation
    i2s_hal_start_tx_link(&I2S_hal, (uint32_t)dma_descriptor);
    i2s_ll_tx_start(I2S_hal.dev);

    return ESP_OK;
} // send_dma

//----------------------------------------------------------------------------
void esp_i2s_parallel::dma_reset()
{
    i2s_hal_reset_txdma(&I2S_hal);
    i2s_hal_reset_rxdma(&I2S_hal);
} // dma_reset

//----------------------------------------------------------------------------
void esp_i2s_parallel::fifo_reset()
{
    i2s_hal_reset_tx_fifo(&I2S_hal);
    i2s_hal_reset_rx_fifo(&I2S_hal);
} // fifo_reset

//----------------------------------------------------------------------------
void esp_i2s_parallel::dev_reset()
{
    fifo_reset();
    dma_reset();
    i2s_hal_reset_tx(&I2S_hal);
    i2s_hal_reset_rx(&I2S_hal);
} // dev_reset

//----------------------------------------------------------------------------
int esp_i2s_parallel::get_memory_width()
{
    i2s_parallel_sample_width_t width = conf.sample_width;
    switch (width)
    {
        case I2S_PARALLEL_WIDTH_8:
        {
            // IS21 supports space saving single byte 8 bit parallel access
            if (conf.port == I2S_NUM_1)
            {
                return 1;
            }
        }
        case I2S_PARALLEL_WIDTH_16:
        {
            return 2;
        }
        case I2S_PARALLEL_WIDTH_24:
        {
            return 4;
        }
        default:
        {
            return -ESP_ERR_INVALID_ARG;
        }
    } // switch (width)
} // get_memory_width

//----------------------------------------------------------------------------
int esp_i2s_parallel::get_bus_width()
{
    switch (conf.sample_width)
    {
    case I2S_PARALLEL_WIDTH_8:
        return 8;
    case I2S_PARALLEL_WIDTH_16:
        return 16;
    case I2S_PARALLEL_WIDTH_24:
        return 24;
    default:
        return -ESP_ERR_INVALID_ARG;
    }
} // get_bus_width

//----------------------------------------------------------------------------
void esp_i2s_parallel::iomux_set_signal(gpio_num_t gpio, int signal)
{
    if (gpio < 0)
    {
        return;
    }
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction(gpio, gpio_mode_t::GPIO_MODE_OUTPUT);
    gpio_matrix_out(gpio, signal, false, false);
} // iomux_set_signal

