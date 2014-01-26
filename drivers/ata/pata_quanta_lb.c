/*
 * <bsn.cl fy=2013 v=gpl>
 * 
 *        Copyright 2013, 2014 BigSwitch Networks, Inc.        
 * 
 * This program is free software; you can redistribute it
 * and/or modify it under  the terms ofthe GNU General Public License as
 * published by the Free Software Foundation;  either version 2 of the  License,
 * or (at your option) any later version.
 * 
 * 
 * </bsn.cl>
 *
 *
 * Quanta LBx platform PATA driver
 */
#include <linux/module.h>
#include <linux/libata.h>
#include <linux/of_platform.h>

#define DRV_NAME "quanta-lb-ata"

static unsigned int
quanta_lb_ata_data_xfer(struct ata_device *dev, unsigned char *buf,
		unsigned int buflen, int rw)
{
	unsigned long irq_flags;
	void __iomem *data_addr = dev->link->ap->ioaddr.data_addr;
	unsigned short *ptr = (unsigned short *) buf;
	unsigned int count = (buflen + 1) / 2;

	local_irq_save(irq_flags);

	if (rw == READ) {
		while (count--) {
			*ptr++ = cpu_to_le16(__raw_readw(data_addr));
		}
	}
	else {
		while (count--) {
			__raw_writew(le16_to_cpu(*ptr), data_addr);
			ptr++;
		}
	}

	local_irq_restore(irq_flags);

	return buflen;
}

static int
quanta_lb_ata_set_mode(struct ata_link *link, struct ata_device **unused)
{
	struct ata_device *dev;

	ata_for_each_dev(dev, link, ENABLED) {
		/* We don't really care */
		dev->pio_mode = dev->xfer_mode = XFER_PIO_0;
		dev->xfer_shift = ATA_SHIFT_PIO;
		dev->flags |= ATA_DFLAG_PIO;
		ata_dev_info(dev, "configured for PIO\n");
	}
	return 0;
}

static struct scsi_host_template quanta_lb_ata_sht = {
	ATA_PIO_SHT(DRV_NAME),
};

static struct ata_port_operations quanta_lb_ata_port_ops = {
	.inherits		= &ata_sff_port_ops,
	.sff_data_xfer		= quanta_lb_ata_data_xfer,
	.cable_detect		= ata_cable_unknown,
	.set_mode		= quanta_lb_ata_set_mode,
};

static int
quanta_lb_ata_probe(struct platform_device *op)
{
	int rv;
	struct resource io_res, ctl_res, *irq_res;
	int irq = 0;
	void __iomem *io_mem, *ctl_mem;
	struct ata_host *host;
	struct ata_port *ap;

	rv = of_address_to_resource(op->dev.of_node, 0, &io_res);
	if (rv) {
		dev_err(&op->dev, "could not determine io base\n");
		return rv;
	}
	io_mem = devm_ioremap(&op->dev, io_res.start, resource_size(&io_res));
	if (!io_mem) {
		dev_err(&op->dev, "could not map io base\n");
		return -ENOMEM;
	}

	rv = of_address_to_resource(op->dev.of_node, 1, &ctl_res);
	if (rv) {
		dev_err(&op->dev, "could not determine ctl base\n");
		return rv;
	}
	ctl_mem = devm_ioremap(&op->dev, ctl_res.start, resource_size(&ctl_res));
	if (!ctl_mem) {
		dev_err(&op->dev, "could not map ctl base\n");
		return -ENOMEM;
	}

	irq_res = platform_get_resource(op, IORESOURCE_IRQ, 0);
	if (irq_res)
		irq = irq_res->start;

	host = ata_host_alloc(&op->dev, 1);
	if (!host)
		return -ENOMEM;
	ap = host->ports[0];

	ap->ops = &quanta_lb_ata_port_ops;
	ap->pio_mask = ATA_PIO6;
	ap->flags |= ATA_FLAG_SLAVE_POSS;

	if (!irq) {
		ap->flags |= ATA_FLAG_PIO_POLLING;
		ata_port_desc(ap, "no IRQ, using PIO polling");
	}

	ap->ioaddr.cmd_addr = io_mem;
	ap->ioaddr.data_addr = io_mem + 0x02;
	ap->ioaddr.error_addr = io_mem + 0x07;
	ap->ioaddr.feature_addr = io_mem + 0x07;
	ap->ioaddr.nsect_addr = io_mem + 0x0b;
	ap->ioaddr.lbal_addr = io_mem + 0x0f;
	ap->ioaddr.lbam_addr = io_mem + 0x13;
	ap->ioaddr.lbah_addr = io_mem + 0x17;
	ap->ioaddr.device_addr = io_mem + 0x1b;
	ap->ioaddr.status_addr = io_mem + 0x1f;
	ap->ioaddr.command_addr = io_mem + 0x1f;
	ap->ioaddr.altstatus_addr = ctl_mem + 0x1b;
	ap->ioaddr.ctl_addr = ctl_mem + 0x1b;

	ata_port_desc(ap, "mmio cmd 0x%llx ctl 0x%llx",
		      (unsigned long long) io_res.start,
		      (unsigned long long) ctl_res.start);

	return ata_host_activate(host, irq, irq ? ata_sff_interrupt : NULL,
				 0, &quanta_lb_ata_sht);
}

static int __exit
quanta_lb_ata_remove(struct platform_device *op)
{
	struct ata_host *host = dev_get_drvdata(&op->dev);
	ata_host_detach(host);
	return 0;
}

static struct of_device_id quanta_lb_ata_of_match[] = {
	{ .compatible = "quanta-lb-ata", },
	{},
};

static struct platform_driver quanta_lb_ata_of_platform_driver = {
	.probe		= quanta_lb_ata_probe,
	.remove		= __exit_p(quanta_lb_ata_remove),
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = quanta_lb_ata_of_match,
	},
};

module_platform_driver(quanta_lb_ata_of_platform_driver);

MODULE_AUTHOR("Big Switch Networks <support@bigswitch.com>");
MODULE_DESCRIPTION("Quanta LBx platform PATA driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, quanta_lb_ata_of_match);
