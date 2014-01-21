/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General	 Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __ASM_JZ4750D_CLOCK_H__
#define __ASM_JZ4750D_CLOCK_H__

enum jz4750d_wait_mode {
	JZ4750D_WAIT_MODE_IDLE,
	JZ4750D_WAIT_MODE_SLEEP,
};

void jz4750d_clock_set_wait_mode(enum jz4750d_wait_mode mode);

void jz4750d_clock_udc_enable_auto_suspend(void);
void jz4750d_clock_udc_disable_auto_suspend(void);

#endif
