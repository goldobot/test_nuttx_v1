/****************************************************************************
 * configs/nucleo-f303re/src/stm32_spi.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32.h"
#include "nucleo-f303re.h"

#if 1 /* FIXME : DEBUG : HACK GOLDO */
extern void up_lowputc(char); /* FIXME : DEBUG */
#endif

#ifdef CONFIG_SPI

/*****************************************************************************
 * Public Data
 *****************************************************************************/
/* Global driver instances */

#if 1 /* FIXME : DEBUG : HACK GOLDO */
#ifdef CONFIG_STM32_SPI1
struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32_SPI2
struct spi_dev_s *g_spi2;
#endif
#ifdef CONFIG_STM32_SPI3
struct spi_dev_s *g_spi3;
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if 1 /* FIXME : DEBUG : HACK GOLDO */
/****************************************************************************
 * Name: stm32_spidev_initialize_goldo
 *
 * Description:
 *   Called to configure SPI devices for goldobot.
 *
 ****************************************************************************/

void stm32_spidev_initialize_goldo(void)
{
  //up_lowputc(0x0a); /* FIXME : DEBUG */

#ifdef CONFIG_STM32_SPI1
  up_lowputc('$'); /* FIXME : DEBUG */
  g_spi1 = stm32_spibus_initialize(1);
  if (!g_spi1)
    {
      spierr("ERROR: FAILED to initialize SPI port 1\n");
      up_lowputc('!'); /* FIXME : DEBUG */
    }
  SPI_SETBITS(g_spi1, 8);
#endif

#ifdef CONFIG_STM32_SPI2
  up_lowputc('$'); /* FIXME : DEBUG */
  g_spi2 = stm32_spibus_initialize(2);
  if (!g_spi2)
    {
      spierr("ERROR: FAILED to initialize SPI port 2\n");
      up_lowputc('!'); /* FIXME : DEBUG */
    }
  SPI_SETBITS(g_spi2, 8);
#endif

#ifdef CONFIG_STM32_SPI3
  up_lowputc('$'); /* FIXME : DEBUG */
  g_spi3 = stm32_spibus_initialize(3);
  if (!g_spi3)
    {
      spierr("ERROR: FAILED to initialize SPI port 3\n");
      up_lowputc('!'); /* FIXME : DEBUG */
    }
  SPI_SETBITS(g_spi3, 8);
#endif

  //up_lowputc(0x0a); /* FIXME : DEBUG */
}
#endif

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board. (legacy)
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void)
{
#if defined(CONFIG_LCD_SSD1351)
  (void)stm32_configgpio(GPIO_OLED_CS); /* OLED chip select */
  (void)stm32_configgpio(GPIO_OLED_DC); /* OLED Command/Data */
#endif
}

/****************************************************************************
 * Name: stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).  All other methods (including
 *   stm32_spibus_initialize()) are provided by common STM32 logic.  To use this
 *   common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to bind
 *      the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

#if defined(CONFIG_LCD_SSD1351)
  if (devid == SPIDEV_DISPLAY)
    {
      stm32_gpiowrite(GPIO_OLED_CS, !selected);
    }
#endif
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32_SPI3
void stm32_spi3select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_spi1cmddata
 *
 * Description:
 *   Set or clear the SSD1351 D/C n bit to select data (true) or command
 *   (false).  This function must be provided by platform-specific logic.
 *   This is an implementation of the cmddata method of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *   spi   - SPI device that controls the bus the device that requires the
 *           CMD/DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *           to select cmd or data.  NOTE:  This design restricts, for
 *           example, one SPI display per SPI bus.
 *   cmd   - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_STM32_SPI1
int stm32_spi1cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                      bool cmd)
{
#ifdef CONFIG_LCD_SSD1351
  if (devid == SPIDEV_DISPLAY)
    {
      (void)stm32_gpiowrite(GPIO_OLED_DC, !cmd);
      return OK;
    }
#endif

  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32_SPI2
int stm32_spi2cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                      bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32_SPI3
int stm32_spi3cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                      bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_SPI_CMDDATA */

#endif /* CONFIG_SPI */
