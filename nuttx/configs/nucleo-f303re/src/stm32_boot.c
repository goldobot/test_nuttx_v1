/****************************************************************************
 * configs/nucleo-f303re/src/stm32_boot.c
 *
 *   Copyright (C) 2013, 2015 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include <debug.h>

#include "nucleo-f303re.h"

/* FIXME : DEBUG : HACK GOLDO */
void stm32_i2c_register(int bus);
int board_pwm_setup(void);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This
 *   entry point is called early in the intitialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
#ifdef CONFIG_SPI
  if (stm32_spidev_initialize != NULL)
    {
      stm32_spidev_initialize();
    }
#endif

#ifdef CONFIG_CANUTILS_LIBUAVCAN
  (void)stm32_configgpio(GPIO_CAN1_RX);
  (void)stm32_configgpio(GPIO_CAN1_TX);
#endif

  /* GOLDOBOT : Robot Goldorak. */

  /* GOLDOBOT : GPIOs pour moteur 1 (droite) */
  (void)stm32_configgpio(GPIO_MAXON1_PWM_IDDLE);
  (void)stm32_configgpio(GPIO_MAXON1_DIR_P);
  (void)stm32_configgpio(GPIO_MAXON1_DIS);

  /* GOLDOBOT : GPIOs pour moteur 2 (gauche) */
  (void)stm32_configgpio(GPIO_MAXON2_PWM_IDDLE);
  (void)stm32_configgpio(GPIO_MAXON2_DIR_P);
  (void)stm32_configgpio(GPIO_MAXON2_DIS);

#if 1 /* FIXME : DEBUG : HACK homologation 2017 */
  stm32_configgpio(GPIO_GOLDO_START);
  stm32_configgpio(GPIO_GOLDO_OBSTACLE);
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif
}

/* GOLDOBOT : Goldorak needs board_initialize() */
#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{
  /* Perform NSH initialization here instead of from the NSH.  This
   * alternative NSH initialization is necessary when NSH is ran in user-space
   * but the initialization function must run in kernel space.
   */

#if defined(CONFIG_NSH_LIBRARY) && !defined(CONFIG_LIB_BOARDCTL)
  board_app_initialize(0);
#endif

#if 0 /* FIXME : DEBUG : HACK homologation 2017 */
  stm32_i2c_register(1);
#endif
  //board_pwm_setup();

  /* CC3000 wireless initialization */

#ifdef CONFIG_WL_CC3000
  //wireless_archinitialize(0);
#endif
}
#endif

#if 0 /* FIXME : DEBUG : HACK homologation 2017 */
void
stm32_i2c_register(int bus)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      serr("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          serr("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          stm32_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

#if 1 /* FIXME : DEBUG : HACK homologation 2017 */
int goldo_get_start_gpio_state(void)
{
  return stm32_gpioread(GPIO_GOLDO_START);
}

int goldo_get_obstacle_gpio_state(void)
{
  return stm32_gpioread(GPIO_GOLDO_OBSTACLE);
}
#endif

