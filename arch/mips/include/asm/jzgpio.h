/*
 * Copyright (C) 2009 Ignacio Garcia Perez <iggarpe@gmail.com>
 *
 * Author: <iggarpe@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#ifndef __ASM_MIPS_JZGPIO_H__
#define __ASM_MIPS_JZGPIO_H__

int gpio_request(unsigned gpio, const char *label);
void gpio_free(unsigned gpio);
int gpio_to_irq(unsigned gpio);
int gpio_get_value(unsigned gpio);
void gpio_set_value(unsigned gpio, int value);
int gpio_direction_input(unsigned gpio);
int gpio_direction_output(unsigned gpio, int value);

/* cansleep wrappers */
#include <asm-generic/gpio.h>

#endif /* __ASM_JZ4740_GPIO_H */
