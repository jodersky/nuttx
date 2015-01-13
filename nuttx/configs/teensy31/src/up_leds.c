/****************************************************************************
 * configs/kwikstik-k40/src/up_leds.c
 * arch/arm/src/board/up_leds.c
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <debug.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_LEDS enables debug output from this file (needs CONFIG_DEBUG
 * with CONFIG_DEBUG_VERBOSE too)
 */

#ifdef CONFIG_DEBUG_LEDS
#  define leddbg  lldbg
#  define ledvdbg llvdbg
#else
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

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
 * Name: board_led_initialize
 *
 * Description:
 *   Initialize LED GPIOs so that LEDs can be controlled.
 *
 ****************************************************************************/

#include "kinetis_internal.h"

#define GPIO_LED0 (PIN_PORTC | PIN5 | GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE)

#define PORTC_PCR5		(*(volatile uint32_t *)0x4004B014) // Pin Control Register n
#define GPIOC_PDOR		(*(volatile uint32_t *)0x400FF080) // Port Data Output Register
#define GPIOC_PSOR		(*(volatile uint32_t *)0x400FF084) // Port Set Output Register
#define GPIOC_PCOR		(*(volatile uint32_t *)0x400FF088) // Port Clear Output Register
#define GPIOC_PTOR		(*(volatile uint32_t *)0x400FF08C) // Port Toggle Output Register
#define GPIOC_PDIR		(*(volatile uint32_t *)0x400FF090) // Port Data Input Register
#define GPIOC_PDDR		(*(volatile uint32_t *)0x400FF094) // Port Data Direction Register

 #define PORT_PCR_SRE			((uint32_t)0x00000004)		// Slew Rate Enable
 #define PORT_PCR_DSE			((uint32_t)0x00000040)		// Drive Strength Enable
 #define PORT_PCR_MUX(n)		((uint32_t)(((n) & 7) << 8))	// Pin Mux Control

#ifdef CONFIG_ARCH_LEDS
void board_led_initialize(void)
{
	//kinetis_pinconfig(GPIO_LED0);
	//kinetis_gpiowrite(GPIO_LED0, true);
	

	//kinetis_pinconfig(PIN_PORTC | PIN5 | GPIO_OUTPUT | GPIO_HIGHDRIVE);
  	PORTC_PCR5 = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1); // set to gpio
	GPIOC_PDDR |= (1 << 5); // set to output
	GPIOC_PSOR |= (1 << 5); // set high
	while(1) {
		
	}
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
	//kinetis_gpiowrite(GPIO_LED0, true);
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led)
{
  //kinetis_gpiowrite(GPIO_LED0, true);
}

#endif /* CONFIG_ARCH_LEDS */
