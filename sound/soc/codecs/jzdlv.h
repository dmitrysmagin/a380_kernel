/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DLV_H
#define _DLV_H

int read_codec_file(int addr);
int write_codec_file_bit(int addr, int bitval, int mask_bit);
void write_codec_file(int addr, int val);

extern struct snd_soc_dai jz4750_codec_dai;
extern struct snd_soc_codec_device soc_codec_dev_jzdlv;

#endif
