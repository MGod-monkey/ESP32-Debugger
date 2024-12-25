#include "sdkconfig.h"

#include <stdbool.h>

#include "components/DAP/include/cmsis_compiler.h"
#include "components/DAP/include/spi_switch.h"
#include "components/DAP/include/gpio_common.h"

#define DAP_SPI GPSPI2


#define ENTER_CRITICAL() portENTER_CRITICAL()
#define EXIT_CRITICAL() portEXIT_CRITICAL()

typedef enum {
    SPI_40MHz_DIV = 2,
    // SPI_80MHz_DIV = 1, //// FIXME: high speed clock
} spi_clk_div_t;

void DAP_SPI_Init()
{
    periph_ll_enable_clk_clear_rst(PERIPH_SPI2_MODULE);

    // In esp32, the driving of GPIO should be stopped,
    // otherwise there will be issue in the spi
    gpio_ll_set_level(&GPIO, GPIO_NUM_12, 0);
    gpio_ll_set_level(&GPIO, GPIO_NUM_11, 0);

    // We will use IO_MUX to get the maximum speed.
    gpio_ll_iomux_in(&GPIO, GPIO_NUM_12,FSPICLK_IN_IDX);
    gpio_ll_iomux_out(&GPIO, GPIO_NUM_12, FUNC_GPIO12_FSPICLK, 0);

    gpio_ll_iomux_in(&GPIO, GPIO_NUM_11,FSPID_IN_IDX);
    gpio_ll_iomux_out(&GPIO, GPIO_NUM_11, FUNC_GPIO11_FSPID, 0);

    // Not using DMA
    DAP_SPI.user.usr_conf_nxt = 0;
    DAP_SPI.slave.usr_conf = 0;
    DAP_SPI.dma_conf.dma_rx_ena = 0;
    DAP_SPI.dma_conf.dma_tx_ena = 0;

    // Set to Master mode
    DAP_SPI.slave.slave_mode = false;

    // use all 64 bytes of the buffer
    DAP_SPI.user.usr_mosi_highpart = false;
    DAP_SPI.user.usr_miso_highpart = false;

    // Disable cs pin
    DAP_SPI.user.cs_setup = false;
    DAP_SPI.user.cs_hold = false;

    // Disable CS signal
    DAP_SPI.misc.cs0_dis = 1;
    DAP_SPI.misc.cs1_dis = 1;
    DAP_SPI.misc.cs2_dis = 1;
    DAP_SPI.misc.cs3_dis = 1;
    DAP_SPI.misc.cs4_dis = 1;
    DAP_SPI.misc.cs5_dis = 1;

    // Duplex transmit
    DAP_SPI.user.doutdin = false;  // half dulex

    // Set data bit order
    DAP_SPI.ctrl.wr_bit_order = 1;   // SWD -> LSB
    DAP_SPI.ctrl.rd_bit_order = 1;   // SWD -> LSB

    // Set dummy
    DAP_SPI.user.usr_dummy = 0; // not use

    // Set spi clk: 40Mhz 50% duty
    // CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX_CONF_U, SPI1_CLK_EQU_SYS_CLK);

    // See TRM `SPI_CLOCK_REG`
    DAP_SPI.clock.clk_equ_sysclk = false;
    DAP_SPI.clock.clkdiv_pre = 0;
    DAP_SPI.clock.clkcnt_n = SPI_40MHz_DIV - 1;
    DAP_SPI.clock.clkcnt_h = SPI_40MHz_DIV / 2 - 1;
    DAP_SPI.clock.clkcnt_l = SPI_40MHz_DIV - 1;

    // MISO delay setting
    DAP_SPI.user.rsck_i_edge = true;
    DAP_SPI.din_mode.din0_mode = 0;
    DAP_SPI.din_mode.din1_mode = 0;
    DAP_SPI.din_mode.din2_mode = 0;
    DAP_SPI.din_mode.din3_mode = 0;
    DAP_SPI.din_num.din0_num = 0;
    DAP_SPI.din_num.din1_num = 0;
    DAP_SPI.din_num.din2_num = 0;
    DAP_SPI.din_num.din3_num = 0;

    // Set the clock polarity and phase CPOL = 1, CPHA = 0
    DAP_SPI.misc.ck_idle_edge = 1;  // HIGH while idle
    DAP_SPI.user.ck_out_edge = 0;

    // enable spi clock
    DAP_SPI.clk_gate.clk_en = 1;
    DAP_SPI.clk_gate.mst_clk_active = 1;
    DAP_SPI.clk_gate.mst_clk_sel = 1;

    // No command and addr for now
    DAP_SPI.user.usr_command = 0;
    DAP_SPI.user.usr_addr = 0;
}


__FORCEINLINE void DAP_SPI_Deinit()
{
    gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[GPIO_NUM_12], PIN_FUNC_GPIO);
    gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[GPIO_NUM_11], PIN_FUNC_GPIO); // MOSI
    //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[12], PIN_FUNC_GPIO); // MISO

    // enable SWCLK output
    gpio_ll_output_enable(&GPIO, GPIO_NUM_12);

    // enable MOSI output & input
    // gpio_ll_output_enable(&GPIO, GPIO_NUM_11);
    GPIO.enable_w1ts |= (0x1 << GPIO_NUM_11);
    gpio_ll_input_enable(&GPIO, GPIO_NUM_11);
}


/**
 * @brief Gain control of SPI
 *
 */
__FORCEINLINE void DAP_SPI_Acquire()
{
    gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[GPIO_NUM_12], FUNC_GPIO12_FSPICLK);
}


/**
 * @brief Release control of SPI
 *
 */
__FORCEINLINE void DAP_SPI_Release()
{
    gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[GPIO_NUM_12], FUNC_GPIO12_GPIO12);
}


/**
 * @brief Use SPI acclerate
 *
 */
void DAP_SPI_Enable()
{
    // may be unuse
    return;
}


/**
 * @brief Disable SPI
 * Drive capability not yet known
 */
__FORCEINLINE void DAP_SPI_Disable()
{
    ;
    //CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX_MTCK_U, (PERIPHS_IO_MUX_FUNC << PERIPHS_IO_MUX_FUNC_S));
    // may be unuse
    // gpio_pin_reg_t pin_reg;
    // GPIO.enable_w1ts |= (0x1 << 13);
    // GPIO.pin[13].driver = 0; // OD Output
    // pin_reg.val = READ_PERI_REG(GPIO_PIN_REG(13));
    // pin_reg.pullup = 1;
    // WRITE_PERI_REG(GPIO_PIN_REG(13), pin_reg.val);
}

