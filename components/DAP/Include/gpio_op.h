#ifndef __GPIO_OP_H__
#define __GPIO_OP_H__

#include "sdkconfig.h"
#include "cmsis_compiler.h"
#include "gpio_common.h"

__STATIC_INLINE __UNUSED void GPIO_FUNCTION_SET(int io_num)
{
  gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[io_num], PIN_FUNC_GPIO);
}

__STATIC_INLINE __UNUSED void GPIO_SET_DIRECTION_NORMAL_OUT(int io_num)
{
    gpio_ll_output_enable(&GPIO, io_num);
    // PP out
    gpio_ll_od_disable(&GPIO, io_num);
}

__STATIC_INLINE __UNUSED void GPIO_SET_LEVEL_HIGH(int io_num)
{
  gpio_ll_set_level(&GPIO, io_num, 1);
}
__STATIC_INLINE __UNUSED void GPIO_SET_LEVEL_LOW(int io_num)
{
  gpio_ll_set_level(&GPIO, io_num, 0);
}

__STATIC_INLINE __UNUSED int GPIO_GET_LEVEL(int io_num)
{
  return gpio_ll_get_level(&GPIO, io_num);
}

__STATIC_INLINE __UNUSED void GPIO_PULL_UP_ONLY_SET(int io_num)
{
  // disable pull down
  gpio_ll_pulldown_dis(&GPIO, io_num);
  // enable pull up
  gpio_ll_pullup_en(&GPIO, io_num);
}


// static void GPIO_SET_DIRECTION_NORMAL_IN(int io_num)
// {
//   GPIO.enable_w1tc |= (0x1 << io_num);
// }



#endif