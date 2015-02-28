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
#include "up_internal.h"

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

#ifdef CONFIG_ARCH_LEDS

#define GPIO_LED0 (PIN_PORTC | PIN5 | GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED1 (PIN_PORTD | PIN1 | GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED2 (PIN_PORTC | PIN0 | GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED3 (PIN_PORTB | PIN0 | GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED4 (PIN_PORTB | PIN1 | GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED5 (PIN_PORTB | PIN3 | GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED6 (PIN_PORTB | PIN2 | GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED7 (PIN_PORTD | PIN5 | GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE)
#define GPIO_LED8 (PIN_PORTD | PIN6 | GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE)

/* Delay used to blink all leds once (to verify that system is up) */
#define LED_DELAY_LOOPS 1000000

static void set_led(int led, bool value)
{
	switch(led) {
	case LED_STARTED:
		kinetis_gpiowrite(GPIO_LED0, value);
		break;
	case LED_HEAPALLOCATE:
		kinetis_gpiowrite(GPIO_LED1, value);
		break;
	case LED_IRQSENABLED:
		kinetis_gpiowrite(GPIO_LED2, value);
		break;
	case LED_STACKCREATED:
		kinetis_gpiowrite(GPIO_LED3, value);
		break;
	case LED_INIRQ:
		kinetis_gpiowrite(GPIO_LED4, value);
		break;
	case LED_SIGNAL:
		kinetis_gpiowrite(GPIO_LED5, value);
		break;
	case LED_ASSERTION:
		kinetis_gpiowrite(GPIO_LED6, value);
		break;
	case LED_PANIC:
		kinetis_gpiowrite(GPIO_LED7, value);
		break;
	default:
		kinetis_gpiowrite(GPIO_LED8, value);
		break;
	}
}

void board_led_initialize(void)
{
	volatile unsigned long counter = 0;

	kinetis_pinconfig(GPIO_LED0);
	kinetis_pinconfig(GPIO_LED1);
	kinetis_pinconfig(GPIO_LED2);
	kinetis_pinconfig(GPIO_LED3);
	kinetis_pinconfig(GPIO_LED4);
	kinetis_pinconfig(GPIO_LED5);
	kinetis_pinconfig(GPIO_LED6);
	kinetis_pinconfig(GPIO_LED7);
	kinetis_pinconfig(GPIO_LED8);

	set_led(LED_STARTED, true);
	set_led(LED_HEAPALLOCATE, true);
	set_led(LED_IRQSENABLED, true);
	set_led(LED_STACKCREATED, true);
	set_led(LED_INIRQ, true);
	set_led(LED_SIGNAL, true);
	set_led(LED_ASSERTION, true);
	set_led(LED_PANIC, true);
	kinetis_gpiowrite(GPIO_LED8, true);

	counter = 0;
	while(counter < LED_DELAY_LOOPS) {
		++counter;
	}

	set_led(LED_HEAPALLOCATE, false);
	set_led(LED_IRQSENABLED, false);
	set_led(LED_STACKCREATED, false);
	set_led(LED_INIRQ, false);
	set_led(LED_SIGNAL, false);
	set_led(LED_ASSERTION, false);
	set_led(LED_PANIC, false);
	kinetis_gpiowrite(GPIO_LED8, false);

	counter = 0;
	while(counter < LED_DELAY_LOOPS) {
		++counter;
	}
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
	set_led(led, true);
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led)
{
  set_led(led, false);
}

#endif /* CONFIG_ARCH_LEDS */
