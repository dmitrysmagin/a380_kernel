/*
 *  Copyright (C) 2012, Antony Pavlov <antonynpavlov@gmail.com>
 *  JZ4750D serial support
 *
 *  based on JZ4740 serial support
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/io.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>

void jz4750d_serial_out(struct uart_port *p, int offset, int value)
{
	switch (offset) {
	case UART_FCR:
		value |= 0x10; /* Enable uart module */
		break;
	case UART_IER:
		value |= (value & 0x4) << 2;
		break;
	default:
		break;
	}
	writeb(value, p->membase + (offset << p->regshift));
}
