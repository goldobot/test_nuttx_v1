/****************************************************************************
 * configs/nucleo-f303re/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2013, 2016 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIG_STM32F3DISCOVERY_INCLUDE_BOARD_H
#define __CONFIG_STM32F3DISCOVERY_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#ifdef __KERNEL__
#  include "stm32.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - Internal 8 MHz RC Oscillator
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul        /* X1 on board */

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000            /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768            /* X2 on board */

/* PLL source is HSE/1, PLL multipler is 9: PLL frequency is 8MHz (XTAL) x 9 = 72MHz */

#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx9
#define STM32_PLL_FREQUENCY     (9*STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (72MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY      /* same as above, to satisfy compiler */

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)   /* Timers 1 and 8, 15-17 */

/* APB2 timers 1 and 8, 15-17 will receive PCLK2. */

/* Timers driven from APB2 will be PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)

#if 0 /* NOTE : HACK GOLDO */
#define STM32_APB1_TIM15_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM16_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM17_CLKIN  (STM32_PCLK2_FREQUENCY)
#else
#define STM32_APB2_TIM15_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (STM32_PCLK2_FREQUENCY)
#endif

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* APB1 timers 2-7 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)

/* USB divider -- Divide PLL clock by 1.5 */

#define STM32_CFGR_USBPRE       0

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM3_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM4_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM5_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM6_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM7_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM8_FREQUENCY   STM32_HCLK_FREQUENCY

#if 1 /* NOTE : HACK GOLDO */
#define BOARD_TIM15_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM16_FREQUENCY   STM32_HCLK_FREQUENCY
#endif

/* LED definitions **********************************************************/
/* The Nucleo F303RE board has three LEDs.  Two of these are controlled by
 * logic on the board and are not available for software control:
 *
 * LD1 COM:  LD1 default status is red.  LD1 turns to green to indicate that
 *           communications are in progress between the PC and the
 *           ST-LINK/V2-1.
 * LD3 PWR:  red LED indicates that the board is powered.
 *
 * And one can be controlled by software:
 *
 * User LD2: green LED is a user LED connected to the I/O PA5 of the
 *           STM32F303RET6.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1       0 /* User LD2 */
#define BOARD_NLEDS      1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT   (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Nucleo F303RE.  The following definitions describe how NuttX controls
 * the LED:
 *
 *   SYMBOL              Meaning                  LED1 state
 *   ------------------  -----------------------  ----------
 *   LED_STARTED         NuttX has been started   OFF
 *   LED_HEAPALLOCATE    Heap has been allocated  OFF
 *   LED_IRQSENABLED     Interrupts enabled       OFF
 *   LED_STACKCREATED    Idle stack created       ON
 *   LED_INIRQ           In an interrupt          No change
 *   LED_SIGNAL          In a signal handler      No change
 *   LED_ASSERTION       An assertion failed      No change
 *   LED_PANIC           The system has crashed   Blinking
 *   LED_IDLE            STM32 is is sleep mode   Not used
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        2
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Button definitions *******************************************************/
/* The Nucleo F303RE supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to the I/O PC13 of the STM32F303RET6.
 *   B2 RESET: push button connected to NRST is used to RESET the
 *             STM32F303RET6.
 */

#define BUTTON_USER      0
#define NUM_BUTTONS      1

#define BUTTON_USER_BIT  (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* GPIO "generalistes" ******************************************************/

#define GPIO_GPIO_IN_C9   (GPIO_INPUT|GPIO_FLOAT|GPIO_PULLUP| \
                           GPIO_PORTC|GPIO_PIN9)  /* PC.9 */
#define GPIO_GPIO_OUT_C9  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN9)
#define GPIO_GPIO_IN_C8   (GPIO_INPUT|GPIO_FLOAT|GPIO_PULLUP| \
                           GPIO_PORTC|GPIO_PIN8)  /* PC.8 */
#define GPIO_GPIO_OUT_C8  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN8)
#define GPIO_GPIO_IN_C6   (GPIO_INPUT|GPIO_FLOAT|GPIO_PULLUP| \
                           GPIO_PORTC|GPIO_PIN6)  /* PC.6 */
#define GPIO_GPIO_OUT_C6  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
/* /!\ ATTENTION : GPIO_C13 connecte au bouton bleu de la carte Nucleo */
#define GPIO_GPIO_IN_C13  (GPIO_INPUT|GPIO_FLOAT|GPIO_PULLUP| \
                           GPIO_PORTC|GPIO_PIN13) /* PC.13 */
#define GPIO_GPIO_OUT_C13 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
#define GPIO_GPIO_IN_C14  (GPIO_INPUT|GPIO_FLOAT|GPIO_PULLUP| \
                           GPIO_PORTC|GPIO_PIN14) /* PC.14 */
#define GPIO_GPIO_OUT_C14 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN14)
#define GPIO_GPIO_IN_B0   (GPIO_INPUT|GPIO_FLOAT|GPIO_PULLUP| \
                           GPIO_PORTB|GPIO_PIN0)  /* PB.0 */
#define GPIO_GPIO_OUT_B0  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)

/* FIXME : TODO : temporaire pour compatibilite avec le code 2017 */
#define GPIO_GOLDO_START    GPIO_GPIO_IN_C9
#define GPIO_GOLDO_OBSTACLE GPIO_GPIO_IN_C8


/* CAN **********************************************************************/

#define GPIO_CAN1_RX GPIO_CAN_RX_2 /* PA.11 */
#define GPIO_CAN1_TX GPIO_CAN_TX_2 /* PA.12 */


/* I2C **********************************************************************/

#define GPIO_I2C1_SCL GPIO_I2C1_SCL_3 /* PB.8 */
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_3 /* PB.9 */

#if 0 /* FIXME : TODO : virer (conf AVANT ECO Jean) */
#define GPIO_I2C2_SCL GPIO_I2C2_SCL_2 /* PF.1 */
#define GPIO_I2C2_SDA GPIO_I2C2_SDA_2 /* PF.0 */
#endif


/* SPI **********************************************************************/

#define GPIO_SPI1_MISO GPIO_SPI1_MISO_1 /* PA.6 */
#define GPIO_SPI1_MOSI GPIO_SPI1_MOSI_1 /* PA.7 */
#define GPIO_SPI1_SCK  GPIO_SPI1_SCK_1  /* PA.5 */
#define GPIO_SPI1_NSS  GPIO_SPI1_NSS_2  /* PA.4 */

//#define GPIO_SPI2_MISO GPIO_SPI2_MISO   /* PB.14 */
//#define GPIO_SPI2_MOSI GPIO_SPI2_MOSI   /* PB.15 */
#define GPIO_SPI2_SCK  GPIO_SPI2_SCK_1  /* PB.13 */
#define GPIO_SPI2_NSS  GPIO_SPI2_NSS_1  /* PB.12 */

#define GPIO_SPI3_MISO GPIO_SPI3_MISO_1 /* PB.4 */
#define GPIO_SPI3_MOSI GPIO_SPI3_MOSI_1 /* PB.5 */
#if 0 /* FIXME : TODO : virer (conf AVANT ECO Jean) */
#define GPIO_SPI3_SCK  GPIO_SPI3_SCK_1  /* PB.3 */
#else
#define GPIO_SPI3_SCK  GPIO_SPI3_SCK_2  /* PC.10 */
#endif
//#define GPIO_SPI3_NSS  GPIO_SPI3_NSS_2  /* PA.4 */


/* TIM (timers generalistes) ************************************************/

#if 0 /* FIXME : TODO : virer (conf AVANT ECO Jean) */
/* PA.13 pris par le JTAG ds la conf de Jean */
#define GPIO_TIM4_CH3OUT GPIO_TIM4_CH3OUT_1 /* PA.13 */
#define GPIO_TIM4_CH3_DISABLE (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                               GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN13)

/* PB.7 pris par le QEI2_CH_A ds la conf de Jean */
#define GPIO_TIM4_CH2OUT GPIO_TIM4_CH2OUT_2 /* PB.7 */
#define GPIO_TIM4_CH2_DISABLE (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                               GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN7)

/* PB.6 pris par le QEI2_CH_B ds la conf de Jean */
#define GPIO_TIM4_CH1OUT GPIO_TIM4_CH1OUT_2 /* PB.6 */
#define GPIO_TIM4_CH1_DISABLE (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                               GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN6)
#endif

/* FIXME : TODO : on peut garder T3.4 ici (pour le routage des pins ) mais on 
   pourra pas l'utiliser avec notre version de Nuttx (pas de support du 
   multichan PWM). TODO : trouver une solution.. */
#define GPIO_TIM3_CH4OUT GPIO_TIM3_CH4OUT_2 /* PB.1 */
#define GPIO_TIM3_CH4_DISABLE (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                               GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)


/* FIXME : TODO : temporaire pour compatibilite avec le code 2017 */
#define GPIO_PUMP1_PWM_IDDLE GPIO_TIM3_CH4_DISABLE
#define GPIO_PUMP2_PWM_IDDLE GPIO_TIM3_CH4_DISABLE
/* FIXME : TODO : ne pas oublier d'associer PWM_POMPE1 avec T3.4 et 
   PWM_POMPE2 avec T3.4 (voir configs/nucleo-f303re/src/stm32_pwm.c) */


/* U(S)ART ******************************************************************/

/* USART servo Dynamixel */
#define GPIO_USART1_RX GPIO_USART1_RX_1     /* PA.10 */
#define GPIO_USART1_TX GPIO_USART1_TX_3     /* PC.4 */
#define GPIO_USART1_RS485_DIR (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                               GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN5)
                                            /* PC.5 */

/* USART console */
#define GPIO_USART2_RX GPIO_USART2_RX_2     /* PA.3 */
#define GPIO_USART2_TX GPIO_USART2_TX_2     /* PA.2 */

/* USART3 */
#define GPIO_USART3_RX GPIO_USART3_RX_1     /* PB.11 */
#define GPIO_USART3_TX GPIO_USART3_TX_1     /* PB.10 */

#if 0 /* FIXME : TODO : virer (conf AVANT ECO Jean) */
/* UART4 disparait dans la conf de Jean */
/* UART4 */
//#define GPIO_UART4_RX GPIO_UART4_RX       /* PC.11 */
//#define GPIO_UART4_TX GPIO_UART4_TX       /* PC.10 */
#endif

/* UART5 */
//#define GPIO_UART5_RX GPIO_UART5_RX       /* PD.2 */
//#define GPIO_UART5_TX GPIO_UART5_TX       /* PC.12 */


/* ADC (avec leur GPIO de trig) *********************************************/

/* ADC1.8 */
//#define GPIO_ADC1_IN8      GPIO_ADC1_IN8     /* PC.2 */
#define GPIO_ADC1_IN8_TRIG (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                            GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)
                                             /* PC.1 */

/* ADC1.9 */
//#define GPIO_ADC1_IN9      GPIO_ADC1_IN9     /* PC.3 */
#define GPIO_ADC1_IN9_TRIG (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                            GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN0)
                                             /* PC.0 */


/* Fonctions "speciales" ****************************************************/

/* PWM moteurs */
/* DROITE */
/* FIXME : TODO : il faut s'assurer que T3.1 n'est PAS utilise (voir definition de "g_pwm1dev" dans arch/arm/src/stm32/stm32_pwm.c et de "goldo_pwm_update_duty" dans le meme fichier (il faut que channels[0] corresponde au timer utilise ici dans board.h)) */
#define GPIO_TIM3_CH2OUT GPIO_TIM3_CH2OUT_4  /* PC.7 */
#define GPIO_MAXON1_PWM_IDDLE (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
			       GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN7)
#define GPIO_MAXON1_DIR_N     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
			       GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN2)
#define GPIO_MAXON1_DIR_P     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
			       GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN2)
                                            /* PB.2 */
#define GPIO_MAXON1_DIS       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
			       GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)
#define GPIO_MAXON1_EN        (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
			       GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
                                            /* PC.15 */

/* GAUCHE */
#define GPIO_TIM2_CH2OUT GPIO_TIM2_CH2OUT_1 /* PA.1 */
#define GPIO_MAXON2_PWM_IDDLE (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
			       GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#define GPIO_MAXON2_DIR_N     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
			       GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN0)
#define GPIO_MAXON2_DIR_P     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
			       GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
                                            /* PA.0 */
#define GPIO_MAXON2_DIS       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
			       GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)
#define GPIO_MAXON2_EN        (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
			       GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
                                            /* PC.15 */


/* QEI (odometrie) */

/* DROITE */
#define GPIO_TIM1_CH2IN  GPIO_TIM1_CH2IN_2  /* PA.9 */
#define GPIO_TIM1_CH1IN  GPIO_TIM1_CH1IN_2  /* PA.8 */

#if 0 /* FIXME : TODO : virer (conf AVANT ECO Jean) */
/* PA.14 pris par le JTAG ds la conf de Jean */
/* PA.15 pris par le JTAG ds la conf de Jean */
/* GAUCHE */
#define GPIO_TIM8_CH1IN  GPIO_TIM8_CH1IN_1  /* PA.15 */
#define GPIO_TIM8_CH2IN  GPIO_TIM8_CH2IN_3  /* PA.14 */
#else
#define GPIO_TIM4_CH1IN  GPIO_TIM4_CH1IN_2  /* PB.6 */
#define GPIO_TIM4_CH2IN  GPIO_TIM4_CH2IN_2  /* PB.7 */
#endif



/* DMA channels *************************************************************/
/* ADC */

#define ADC1_DMA_CHAN DMACHAN_ADC1
#define ADC2_DMA_CHAN DMACHAN_ADC2_1
#define ADC3_DMA_CHAN DMACHAN_ADC3
#define ADC4_DMA_CHAN DMACHAN_ADC4_1

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIG_NUCLEO_F303RE_INCLUDE_BOARD_H */
