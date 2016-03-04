/*
 * 88pm886-debug.c - driver for taking 88pm886 regsiter dumps
 *
 * Copyright (C) 2014, Samsung Electronics, Co., Ltd. All Rights Reserved.
 * Abilash Chandrasekar <abilash.c@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#include <linux/mfd/88pm886.h>
#include "88pm886-debug.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define MODULE_NAME	"88PM886_Register_Dump_Driver"

static struct dentry *dbg_dir;
static u8 ldo_enable[16] ;

static int get_ldo_voltage(int id, char *mode, u8 val);
static void get_enable_status(u8 *buf);
static void dump_regs(struct seq_file *sf, u8 *buf);

static int get_ldo_voltage(int id, char *mode, u8 val)
{
		int index;

		if (val == 0)
			return 0;

		if (!strcmp(mode, "sleep"))
			index = (val&0xF0)>>4;
		else
			index =  val&0x0F;

		if (id == PM886_LDO1_VOUT ||
				id == PM886_LDO2_VOUT || id == PM886_LDO3_VOUT)
			return vset1_3[index];
		else if (id == PM886_LDO16_VOUT)
			return vset16[index];
		else
			return vset4_15[index];
}

static void get_enable_status(u8 *buf)
{
	int bit;

	for (bit = 0; bit < 8; bit++)
		ldo_enable[bit] = buf[PM886_LDO_EN1] >> bit;

	for (bit = 0; bit < 8; bit++)
		ldo_enable[bit+8] = buf[PM886_LDO_EN2] >> bit;
}

static void dump_regs(struct seq_file *sf, u8 *buf)
{
	int id, i;
	char *mode;

	seq_printf(sf,
	"\n\n**************** 88PM886 LDO Registers Dump *****************\n\n");

	seq_printf(sf,
	"---------------------------------------------------------\n");
	seq_printf(sf,
		"\nAddr\t| Name\t| Value\t| Status| uV \t\t| Mode |\n");
	seq_printf(sf,
	"---------------------------------------------------------\n");

	for (id = PM886_LDO1_VOUT, i = 0;
				id <= PM886_LDO16_VOUT; id = (id+0x6), i++) {

		if (((buf[id+1]&0x30) >> 4) == 2)
			mode = "sleep";
		else if ((((buf[id+1]&0x30) >> 4) == 1) ||
					(((buf[id+1]&0x30) >> 4) == 3))
			mode = "active";
		else{
			printk(KERN_ERR
				"Error detecting sleep mode for %0x\n", id);
			continue;
		}

		if (ldo_enable[i])
			seq_printf(sf,
				"0x%x\t|%s\t| 0x%x\t| ON\t| %d\t| %-6s|\n",
				id, names_registers[i], buf[id],
				get_ldo_voltage(id, mode, buf[id]), mode);
		else
			seq_printf(sf,
				"0x%x\t|%s\t| 0x%x\t| OFF\t| \t\t|\t|\n",
				id,	names_registers[i], buf[id]);
	}
	seq_printf(sf,
	"---------------------------------------------------------\n");
}

static int reg_dump_main(struct seq_file *sf, void *unused)
{
	u8 buf[200];

	/* powerpage registers dump*/
	power_regmap_bulk_read(buf, START_ADDR, TOTAL_REGS);

	/* get ldo enable/disable status */
	get_enable_status(buf);

	/* dump the formatted registers output*/
	dump_regs(sf, buf);

	return 0;
}

static int pm886_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, reg_dump_main, inode->i_private);
}

static const struct file_operations pm886_dbg_fops = {
	.open = pm886_dbg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init pm886_reg_init(void)
{

#ifdef CONFIG_DEBUG_FS
	/* Creating debugfs directory for 88pm886 register dump */
	dbg_dir = debugfs_create_dir("88pm886_ldodebug", NULL);
	if (IS_ERR(dbg_dir))
		return PTR_ERR(dbg_dir);

	if (IS_ERR_OR_NULL(debugfs_create_file("ldo_dump",
		S_IRUGO, dbg_dir, "88pm886", &pm886_dbg_fops)))
		pr_err("Failed to create file 88pm886_ldodebug/LDOs\n");

#else
	pr_info("\nCONFIG_DEBUG_FS is not defined\n");
#endif
	return 0;
}

static void __exit pm886_reg_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(dbg_dir);
#endif
}

module_init(pm886_reg_init);
module_exit(pm886_reg_exit);

