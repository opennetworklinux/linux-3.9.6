/*
 * Quanta LBx platform setup
 *
 * Copyright 2013 Big Switch Networks, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/pci.h>
#include <asm/udbg.h>
#include <asm/mpic.h>
#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>

#include "mpc85xx.h"

static void quanta_lb_restart(char *cmd)
{
	void __iomem *immap = ioremap(get_immrbase(), 0x1000000);
	if (immap) {
		unsigned long pdata = in_be32(immap + 0x90d10);
		pdata &= ~(1 << 7);
		out_be32(immap + 0x90d10, pdata);
	}
	fsl_rstcr_restart(NULL);
}

static void __init quanta_lb_pic_init(void)
{
	struct mpic *mpic = mpic_alloc(NULL, 0, MPIC_BIG_ENDIAN, 0, 256,
				       " OpenPIC  ");
	BUG_ON(!mpic);
	mpic_init(mpic);
}

static void __init quanta_lb_setup_arch(void)
{
	if (ppc_md.progress)
		ppc_md.progress("quanta_lb_setup_arch()", 0);
	fsl_pci_assign_primary();
}

static void quanta_lb_show_cpuinfo(struct seq_file *m)
{
	seq_printf(m, "PVR\t\t: 0x%lx\n", mfspr(SPRN_PVR));
	seq_printf(m, "SVR\t\t: 0x%lx\n", mfspr(SPRN_SVR));
	seq_printf(m, "PLL\t\t: 0x%lx\n", (mfspr(SPRN_HID1) >> 24) & 0x3f);
}

static int __init quanta_lb_probe(void)
{
        return of_flat_dt_is_compatible(of_get_flat_dt_root(), "quanta-lb");
}

machine_arch_initcall(quanta_lb, mpc85xx_common_publish_devices);

define_machine(quanta_lb) {
	.name		= "Quanta LBx",
	.probe		= quanta_lb_probe,
	.setup_arch	= quanta_lb_setup_arch,
	.init_IRQ	= quanta_lb_pic_init,
	.show_cpuinfo	= quanta_lb_show_cpuinfo,
	.get_irq	= mpic_get_irq,
	.restart	= quanta_lb_restart,
	.calibrate_decr = generic_calibrate_decr,
	.progress	= udbg_progress,
};
