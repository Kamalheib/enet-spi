/*
 * ENET SPI controller driver (master mode only)
 *
 * Author: Kamal Heib
 *	kamalheib1@gmail.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/spi/spi.h>
#include "enet-spi.h"

struct enet_dev {
	struct pci_dev *pdev;
	void __iomem *hw_addr;
};

static int enet_spi_probe(struct pci_dev *pdev,
			  const struct pci_device_id *id)
{
	const char *driver_name = pdev->driver->name;
	struct enet_dev *edev;
	int err;

	edev = kzalloc(sizeof(*edev), GFP_KERNEL);
	if (!edev)
		return -ENOMEM;

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to enable device\n");
		goto err_pci_enable_device;
	}

	err = pci_request_regions(pdev, driver_name);
	if (err) {
		dev_err(&pdev->dev, "Failed to request regions\n");
		goto err_pci_request_regions;
	}

	edev->hw_addr = ioremap(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0));
	if (!edev->hw_addr) {
		dev_err(&pdev->dev, "Failed to ioremap bar0\n");
		err = -EIO;
		goto err_ioremap;
	}

	pci_set_master(pdev);

	edev->pdev = pdev;
	pci_set_drvdata(pdev, edev);

	return 0;

err_ioremap:
	pci_release_regions(pdev);
err_pci_request_regions:
	pci_disable_device(pdev);
err_pci_enable_device:
	kfree(edev);
	return err;
}

static void enet_spi_remove(struct pci_dev *pdev)
{
	struct enet_dev *edev = pci_get_drvdata(pdev);

	iounmap(edev->hw_addr);
	pci_release_regions(edev->pdev);
	pci_disable_device(edev->pdev);
	kfree(edev);
}

static const struct pci_device_id enet_spi_pci_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_ENET, PCI_DEVICE_ID_ENET) },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, enet_spi_pci_id_table);

static struct pci_driver enet_spi_driver = {
	.name		= DRV_NAME,
	.id_table	= enet_spi_pci_id_table,
	.probe		= enet_spi_probe,
	.remove		= enet_spi_remove,
};

static int __init enet_spi_init_module(void)
{
	pr_info("ENET SPI driver - version %s\n", DRV_VERSION);

	return pci_register_driver(&enet_spi_driver);
}
module_init(enet_spi_init_module);

static void __exit enet_spi_exit_module(void)
{
	pci_unregister_driver(&enet_spi_driver);
}
module_exit(enet_spi_exit_module);


MODULE_DESCRIPTION("ENET SPI bus driver");
MODULE_AUTHOR("Kamal Heib");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
