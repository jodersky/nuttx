/************************************************************************************
 * configs/kwikstik-k40/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
#undef BOARD_EXTCLOCK						/* No external clock */
#define BOARD_EXTAL_FREQ     16000000       /* 16MHz Oscillator */
#define BOARD_XTAL32_FREQ    32768          /* 32KHz RTC Oscillator */

/* PLL Configuration.  NOTE: Only even frequency crystals are supported that will
 * produce a 2MHz reference clock to the PLL.
 *
 *   PLL Input frequency:   PLLIN  = REFCLK/PRDIV = 16MHz/8 = 2MHz
 *   PLL Output frequency:  PLLOUT = PLLIN*VDIV   = 2Mhz*48 = 96MHz
 *   MCG Frequency:         PLLOUT = 96MHz
 */
 
#define BOARD_PRDIV          8             /* PLL External Reference Divider */
#define BOARD_VDIV           48            /* PLL VCO Divider (frequency multiplier) */

#define BOARD_PLLIN_FREQ     (BOARD_EXTAL_FREQ / BOARD_PRDIV)
#define BOARD_PLLOUT_FREQ    (BOARD_PLLIN_FREQ * BOARD_VDIV)
#define BOARD_MCG_FREQ       BOARD_PLLOUT_FREQ

/* SIM CLKDIV1 dividers */

#define BOARD_OUTDIV1        1              /* Core        = MCG, 96MHz */
#define BOARD_OUTDIV2        2              /* Bus         = MCG/2, 48MHz */
#define BOARD_OUTDIV3        2              /* FlexBus     = MCG/2, 48MHz */
#define BOARD_OUTDIV4        4              /* Flash clock = MCG/4, 24MHz */

#define BOARD_CORECLK_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV1)
#define BOARD_BUS_FREQ      (BOARD_MCG_FREQ / BOARD_OUTDIV2)
#define BOARD_FLEXBUS_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV3)
#define BOARD_FLASHCLK_FREQ (BOARD_MCG_FREQ / BOARD_OUTDIV4)

/* LED definitions ******************************************************************/
#define LED_STARTED       0
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   2
#define LED_STACKCREATED  3
#define LED_INIRQ         4
#define LED_SIGNAL        5
#define LED_ASSERTION     6
#define LED_PANIC         7

/* Alternative pin resolution *******************************************************/

//Telemetry
#define PIN_UART0_RX	(PIN_PORTB | PIN16 | PIN_ALT3)
#define PIN_UART0_TX	(PIN_PORTB | PIN17 | PIN_ALT3)

//GPS
#define PIN_UART2_RX	(PIN_PORTD | PIN2  | PIN_ALT3)
#define PIN_UART2_TX	(PIN_PORTD | PIN3  | PIN_ALT3)

//AUX0
#define PIN_FTM0_CH2	(PIN_PORTC | PIN3  | PIN_ALT4)

//AUX1
#define PIN_FTM0_CH3	(PIN_PORTC | PIN4  | PIN_ALT4)

//M0
#define PIN_FTM0_CH5	(PIN_PORTD | PIN5  | PIN_ALT4)

//M1
#define PIN_FTM0_CH6	(PIN_PORTD | PIN6  | PIN_ALT4)

//M2
#define PIN_FTM0_CH0	(PIN_PORTC | PIN1  | PIN_ALT4)

//M3
#define PIN_FTM0_CH1	(PIN_PORTC | PIN2  | PIN_ALT4)

//Battery voltage
#define PIN_ADC0_SE8    (PIN_PORTB | PIN1  | PIN_ANALOG)


/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: kinetis_boardinitialize
 *
 * Description:
 *   All kinetis architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void kinetis_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
