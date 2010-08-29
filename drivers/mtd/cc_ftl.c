/*
 *  Copyright (C) 2010, Maarten ter Huurne <maarten@treewalker.org>
 *  Flash Translation Layer for media players using firmware from China Chip.
 *
 *  This initial implementation provides read-only access.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mtd/blktrans.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <mtd/mtd-abi.h>


#define SECTOR_SIZE		512

struct cc_ftl_partition {
	struct mtd_blktrans_dev mbd;

	uint32_t *map;
};

static int cc_ftl_readsect(struct mtd_blktrans_dev *dev, unsigned long block,
			   char *buffer)
{
	struct cc_ftl_partition *partition = (struct cc_ftl_partition *)dev;
	struct mtd_info *mtd = dev->mtd;
	uint64_t log_offs, phy_offs;
	uint32_t log_blk, phy_blk;
	size_t retlen;
	int ret;

	/* Find physical location. */
	if (block >= dev->size)
		return -EIO;
	log_offs = ((uint64_t)block) * SECTOR_SIZE;
	log_blk = mtd_div_by_eb(log_offs, mtd);
	phy_blk = partition->map[log_blk];
	if (phy_blk == (uint32_t)-1)
		return -EIO;
	phy_offs = (uint64_t)phy_blk * mtd->erasesize;
	phy_offs += mtd_mod_by_eb(log_offs, mtd);

	/* Read data. */
	ret = mtd_read(mtd, phy_offs, SECTOR_SIZE, &retlen, buffer);
	if (ret == -EUCLEAN) /* sector contains correctable errors */
		ret = 0;
	if (ret)
		return ret;
	if (retlen != SECTOR_SIZE)
		return -EIO;

	return 0;
}

uint32_t *cc_ftl_build_block_map(struct mtd_info *mtd)
{
	uint32_t num_blk = mtd_div_by_eb(mtd->size, mtd);
	uint32_t blk_found;
	uint32_t pages_per_blk = mtd_div_by_ws(mtd->erasesize, mtd);
	uint32_t phy_blk;
	uint8_t	oob_buf[mtd->oobsize];
	unsigned int seq_offs = mtd->oobsize - 4;
	uint32_t *map;

	// TODO: Is it worth reading multiple oobs at once?
	//       Reading two will at least help against bit errors.
	struct mtd_oob_ops oob_ops = {
		.mode		= MTD_OPS_RAW,
		.len		= 0,
		.ooblen		= mtd->oobsize,
		.datbuf		= NULL,
		.oobbuf		= oob_buf,
	};

	map = kmalloc(sizeof(uint32_t) * num_blk, GFP_KERNEL);
	if (!map)
		return NULL;
	memset(map, 0xFF, sizeof(uint32_t) * num_blk);

	blk_found = 0;
	for (phy_blk = 0; phy_blk < num_blk; phy_blk++) {
		loff_t ofs = (loff_t)phy_blk << mtd->erasesize_shift;
		uint16_t signature;
		uint16_t last_page;
		uint32_t log_blk;
		int err;

		if (mtd_block_isbad(mtd, ofs))
			continue;
		err = mtd_read_oob(mtd, ofs, &oob_ops);
		if (err)
			continue;
		signature = oob_buf[0] | ((uint16_t)oob_buf[1] << 8);
		if (signature != 0x00FF)
			continue;
		last_page = oob_buf[2] | ((uint16_t)oob_buf[3] << 8);
		if (last_page >= pages_per_blk)
			continue;
		log_blk = oob_buf[seq_offs] |
			  ((uint32_t)oob_buf[seq_offs + 1] << 8) |
			  ((uint32_t)oob_buf[seq_offs + 2] << 16) |
			  ((uint32_t)oob_buf[seq_offs + 3] << 24);
		if (log_blk == 0xFFFFFFFF)
			continue;
		if (log_blk >= num_blk) {
			printk(KERN_WARNING "physical block %d claims "
					"logical block %d which is beyond "
					"partition end %d\n",
					phy_blk, log_blk, num_blk
					);
			continue;
		}
		if (map[log_blk] != 0xFFFFFFFF) {
			// TODO: Version number might sort this out.
			printk(KERN_WARNING "physical block %d and %d both "
					"claim logical block %d\n",
					map[log_blk], phy_blk, log_blk
					);
			continue;
		}
		map[log_blk] = phy_blk;
		blk_found++;
	}

	if (blk_found == 0) {
		kfree(map);
		return NULL;
	}

	if (0) {
		uint32_t log_blk;
		for (log_blk = 0; log_blk < num_blk; log_blk++) {
			if (map[log_blk] != 0xFFFFFFFF) {
				printk("%04X:%04X ", log_blk, map[log_blk]);
			}
		}
		printk("\n");
	}

	return map;
}

static void cc_ftl_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	struct cc_ftl_partition *partition;
	uint32_t *map;

	/* Check for NAND first, so we know we can use the "chip" pointer. */
	if (mtd->type != MTD_NANDFLASH)
		return;

	/* Erase size must be a power of two. */
	if (mtd->erasesize_shift == 0)
		return;

	/* Erase size must be a multiple of sector size. */
	if ((mtd->erasesize & (SECTOR_SIZE - 1)) != 0)
		return;

	/* Probably this translation layer can work with different oob sizes,
	 * but for now we only accept the layout it was tested with.
	 */
	if (mtd->oobsize != 128)
		return;

	map = cc_ftl_build_block_map(mtd);
	if (!map)
		return;

	partition = kzalloc(sizeof(struct cc_ftl_partition), GFP_KERNEL);
	if (!partition)
		goto err_map;

	partition->mbd.mtd = mtd;
	// TODO: More reasonable guess.
	partition->mbd.size = mtd->size / SECTOR_SIZE;
	partition->mbd.tr = tr;
	partition->mbd.devnum = -1;
	partition->map = map;

	if (add_mtd_blktrans_dev((struct mtd_blktrans_dev *)partition))
		goto err_partition;

	return;

err_partition:
	kfree(partition);
err_map:
	kfree(map);
}

static void cc_ftl_remove_dev(struct mtd_blktrans_dev *dev)
{
	struct cc_ftl_partition *partition = (struct cc_ftl_partition *)dev;

	if (del_mtd_blktrans_dev(dev))
		return;

	kfree(partition->map);
	kfree(partition);
}

static struct mtd_blktrans_ops cc_ftl_tr = {
	.name		= "ccnand",
	.major		= 242,	/* TODO: Register an official major number. */
	.part_bits	= 4,
	.blksize 	= SECTOR_SIZE,

	.readsect	= cc_ftl_readsect,
	.add_mtd	= cc_ftl_add_mtd,
	.remove_dev	= cc_ftl_remove_dev,

	.owner		= THIS_MODULE,
};

static int __init cc_ftl_init(void)
{
	return register_mtd_blktrans(&cc_ftl_tr);
}
module_init(cc_ftl_init);

static void __exit cc_ftl_exit(void)
{
	deregister_mtd_blktrans(&cc_ftl_tr);
}
module_exit(cc_ftl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maarten ter Huurne <maarten@treewalker.org>");
MODULE_DESCRIPTION("Flash Translation Layer for media players "
		   "using firmware from China Chip");
