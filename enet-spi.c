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
#include <linux/spi/spi_bitbang.h>
#include "enet-spi.h"

struct xilinx_spi {
	/* bitbang has to be first */
	struct spi_bitbang bitbang;
	struct completion done;
	void __iomem	*regs;	/* virt. address of the control registers */

	int		irq;

	u8 *rx_ptr;		/* pointer in the Tx buffer */
	const u8 *tx_ptr;	/* pointer in the Rx buffer */
	u8 bytes_per_word;
	int buffer_size;	/* buffer size in words */
	u32 cs_inactive;	/* Level of the CS pins when inactive*/
	unsigned int (*read_fn)(void __iomem *);
	void (*write_fn)(u32, void __iomem *);
};

struct enet_dev {
	struct pci_dev *pdev;
	void __iomem *hw_addr;

	struct spi_master *master;
};

static void xspi_write32(u32 val, void __iomem *addr)
{
	iowrite32(val, SPI_BASE_ADDR + addr);
}

static unsigned int xspi_read32(void __iomem *addr)
{
	return ioread32(SPI_BASE_ADDR + addr);
}

static void xspi_write32_be(u32 val, void __iomem *addr)
{
	iowrite32be(val, SPI_BASE_ADDR + addr);
}

static unsigned int xspi_read32_be(void __iomem *addr)
{
	return ioread32be(SPI_BASE_ADDR + addr);
}

static void xilinx_spi_tx(struct xilinx_spi *xspi)
{
	u32 data = 0;

	if (!xspi->tx_ptr) {
		xspi->write_fn(0, xspi->regs + XSPI_TXD_OFFSET);
		return;
	}

	switch (xspi->bytes_per_word) {
	case 1:
		data = *(u8 *)(xspi->tx_ptr);
		break;
	case 2:
		data = *(u16 *)(xspi->tx_ptr);
		break;
	case 4:
		data = *(u32 *)(xspi->tx_ptr);
		break;
	}

	xspi->write_fn(data, xspi->regs + XSPI_TXD_OFFSET);
	xspi->tx_ptr += xspi->bytes_per_word;
}

static void xilinx_spi_rx(struct xilinx_spi *xspi)
{
	u32 data = xspi->read_fn(xspi->regs + XSPI_RXD_OFFSET);

	if (!xspi->rx_ptr)
		return;

	switch (xspi->bytes_per_word) {
	case 1:
		*(u8 *)(xspi->rx_ptr) = data;
		break;
	case 2:
		*(u16 *)(xspi->rx_ptr) = data;
		break;
	case 4:
		*(u32 *)(xspi->rx_ptr) = data;
		break;
	}

	xspi->rx_ptr += xspi->bytes_per_word;
}

static void xspi_init_hw(struct xilinx_spi *xspi)
{
	void __iomem *regs_base = xspi->regs;

	/* Reset the SPI device */
	xspi->write_fn(XIPIF_V123B_RESET_MASK,
		regs_base + XIPIF_V123B_RESETR_OFFSET);
	/* Enable the transmit empty interrupt, which we use to determine
	 * progress on the transmission.
	 */
	xspi->write_fn(XSPI_INTR_TX_EMPTY,
			regs_base + XIPIF_V123B_IIER_OFFSET);
	/* Disable the global IPIF interrupt */
	xspi->write_fn(0, regs_base + XIPIF_V123B_DGIER_OFFSET);
	/* Deselect the slave on the SPI bus */
	xspi->write_fn(0xffff, regs_base + XSPI_SSR_OFFSET);
	/* Disable the transmitter, enable Manual Slave Select Assertion,
	 * put SPI controller into master mode, and enable it */
	xspi->write_fn(XSPI_CR_MANUAL_SSELECT |	XSPI_CR_MASTER_MODE |
		XSPI_CR_ENABLE | XSPI_CR_TXFIFO_RESET |	XSPI_CR_RXFIFO_RESET,
		regs_base + XSPI_CR_OFFSET);
}

static void xilinx_spi_chipselect(struct spi_device *spi, int is_on)
{
	struct xilinx_spi *xspi = spi_master_get_devdata(spi->master);
	u16 cr;
	u32 cs;

	if (is_on == BITBANG_CS_INACTIVE) {
		/* Deselect the slave on the SPI bus */
		xspi->write_fn(xspi->cs_inactive, xspi->regs + XSPI_SSR_OFFSET);
		return;
	}

	/* Set the SPI clock phase and polarity */
	cr = xspi->read_fn(xspi->regs + XSPI_CR_OFFSET)	& ~XSPI_CR_MODE_MASK;
	if (spi->mode & SPI_CPHA)
		cr |= XSPI_CR_CPHA;
	if (spi->mode & SPI_CPOL)
		cr |= XSPI_CR_CPOL;
	if (spi->mode & SPI_LSB_FIRST)
		cr |= XSPI_CR_LSB_FIRST;
	if (spi->mode & SPI_LOOP)
		cr |= XSPI_CR_LOOP;
	xspi->write_fn(cr, xspi->regs + XSPI_CR_OFFSET);

	/* We do not check spi->max_speed_hz here as the SPI clock
	 * frequency is not software programmable (the IP block design
	 * parameter)
	 */

	cs = xspi->cs_inactive;
	cs ^= BIT(spi->chip_select);

	/* Activate the chip select */
	xspi->write_fn(cs, xspi->regs + XSPI_SSR_OFFSET);
}

/* spi_bitbang requires custom setup_transfer() to be defined if there is a
 * custom txrx_bufs().
 */
static int xilinx_spi_setup_transfer(struct spi_device *spi,
		struct spi_transfer *t)
{
	struct xilinx_spi *xspi = spi_master_get_devdata(spi->master);

	if (spi->mode & SPI_CS_HIGH)
		xspi->cs_inactive &= ~BIT(spi->chip_select);
	else
		xspi->cs_inactive |= BIT(spi->chip_select);

	return 0;
}

static int xilinx_spi_txrx_bufs(struct spi_device *spi, struct spi_transfer *t)
{
	struct xilinx_spi *xspi = spi_master_get_devdata(spi->master);
	int remaining_words;	/* the number of words left to transfer */
	bool use_irq = false;
	u16 cr = 0;

	/* We get here with transmitter inhibited */

	xspi->tx_ptr = t->tx_buf;
	xspi->rx_ptr = t->rx_buf;
	remaining_words = t->len / xspi->bytes_per_word;

	if (xspi->irq >= 0 &&  remaining_words > xspi->buffer_size) {
		u32 isr;
		use_irq = true;
		/* Inhibit irq to avoid spurious irqs on tx_empty*/
		cr = xspi->read_fn(xspi->regs + XSPI_CR_OFFSET);
		xspi->write_fn(cr | XSPI_CR_TRANS_INHIBIT,
			       xspi->regs + XSPI_CR_OFFSET);
		/* ACK old irqs (if any) */
		isr = xspi->read_fn(xspi->regs + XIPIF_V123B_IISR_OFFSET);
		if (isr)
			xspi->write_fn(isr,
				       xspi->regs + XIPIF_V123B_IISR_OFFSET);
		/* Enable the global IPIF interrupt */
		xspi->write_fn(XIPIF_V123B_GINTR_ENABLE,
				xspi->regs + XIPIF_V123B_DGIER_OFFSET);
		reinit_completion(&xspi->done);
	}

	while (remaining_words) {
		int n_words, tx_words, rx_words;
		u32 sr;
		int stalled;

		n_words = min(remaining_words, xspi->buffer_size);

		tx_words = n_words;
		while (tx_words--)
			xilinx_spi_tx(xspi);

		/* Start the transfer by not inhibiting the transmitter any
		 * longer
		 */

		if (use_irq) {
			xspi->write_fn(cr, xspi->regs + XSPI_CR_OFFSET);
			wait_for_completion(&xspi->done);
			/* A transmit has just completed. Process received data
			 * and check for more data to transmit. Always inhibit
			 * the transmitter while the Isr refills the transmit
			 * register/FIFO, or make sure it is stopped if we're
			 * done.
			 */
			xspi->write_fn(cr | XSPI_CR_TRANS_INHIBIT,
				       xspi->regs + XSPI_CR_OFFSET);
			sr = XSPI_SR_TX_EMPTY_MASK;
		} else
			sr = xspi->read_fn(xspi->regs + XSPI_SR_OFFSET);

		/* Read out all the data from the Rx FIFO */
		rx_words = n_words;
		stalled = 10;
		while (rx_words) {
			if (rx_words == n_words && !(stalled--) &&
			    !(sr & XSPI_SR_TX_EMPTY_MASK) &&
			    (sr & XSPI_SR_RX_EMPTY_MASK)) {
				dev_err(&spi->dev,
					"Detected stall. Check C_SPI_MODE and C_SPI_MEMORY\n");
				xspi_init_hw(xspi);
				return -EIO;
			}

			if ((sr & XSPI_SR_TX_EMPTY_MASK) && (rx_words > 1)) {
				xilinx_spi_rx(xspi);
				rx_words--;
				continue;
			}

			sr = xspi->read_fn(xspi->regs + XSPI_SR_OFFSET);
			if (!(sr & XSPI_SR_RX_EMPTY_MASK)) {
				xilinx_spi_rx(xspi);
				rx_words--;
			}
		}

		remaining_words -= n_words;
	}

	if (use_irq) {
		xspi->write_fn(0, xspi->regs + XIPIF_V123B_DGIER_OFFSET);
		xspi->write_fn(cr, xspi->regs + XSPI_CR_OFFSET);
	}

	return t->len;
}


/* This driver supports single master mode only. Hence Tx FIFO Empty
 * is the only interrupt we care about.
 * Receive FIFO Overrun, Transmit FIFO Underrun, Mode Fault, and Slave Mode
 * Fault are not to happen.
 */
static irqreturn_t xilinx_spi_irq(int irq, void *dev_id)
{
	struct xilinx_spi *xspi = dev_id;
	u32 ipif_isr;

	/* Get the IPIF interrupts, and clear them immediately */
	ipif_isr = xspi->read_fn(xspi->regs + XIPIF_V123B_IISR_OFFSET);
	xspi->write_fn(ipif_isr, xspi->regs + XIPIF_V123B_IISR_OFFSET);

	if (ipif_isr & XSPI_INTR_TX_EMPTY) {	/* Transmission completed */
		complete(&xspi->done);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int xilinx_spi_find_buffer_size(struct xilinx_spi *xspi)
{
	u8 sr;
	int n_words = 0;

	/*
	 * Before the buffer_size detection we reset the core
	 * to make sure we start with a clean state.
	 */
	xspi->write_fn(XIPIF_V123B_RESET_MASK,
		xspi->regs + XIPIF_V123B_RESETR_OFFSET);

	/* Fill the Tx FIFO with as many words as possible */
	do {
		xspi->write_fn(0, xspi->regs + XSPI_TXD_OFFSET);
		sr = xspi->read_fn(xspi->regs + XSPI_SR_OFFSET);
		n_words++;
	} while (!(sr & XSPI_SR_TX_FULL_MASK));

	return n_words;
}

static int enet_spi_probe(struct pci_dev *pdev,
			  const struct pci_device_id *id)
{
	const char *driver_name = pdev->driver->name;
	struct xilinx_spi *xspi;
	struct enet_dev *edev;
	int bits_per_word = 8;
	int err;
	u32 tmp;

	edev = kzalloc(sizeof(*edev), GFP_KERNEL);
	if (!edev)
		return -ENOMEM;

	edev->master = spi_alloc_master(&pdev->dev, sizeof(struct xilinx_spi));
	if (!edev->master) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "Failed to allocate spi master\n");
		goto err_spi_alloc_master;
	}

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

	edev->master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST | SPI_LOOP |
				  SPI_CS_HIGH;

	xspi = spi_master_get_devdata(edev->master);
	xspi->cs_inactive = 0xffffffff;
	xspi->bitbang.master = edev->master;
	xspi->bitbang.chipselect = xilinx_spi_chipselect;
	xspi->bitbang.setup_transfer = xilinx_spi_setup_transfer;
	xspi->bitbang.txrx_bufs = xilinx_spi_txrx_bufs;
	init_completion(&xspi->done);

	xspi->regs = edev->hw_addr;
	edev->master->num_chipselect = 0x1;
	edev->master->dev.of_node = pdev->dev.of_node;

	xspi->read_fn = xspi_read32;
	xspi->write_fn = xspi_write32;


	xspi->write_fn(XSPI_CR_LOOP, xspi->regs + XSPI_CR_OFFSET);
	tmp = xspi->read_fn(xspi->regs + XSPI_CR_OFFSET);
	tmp &= XSPI_CR_LOOP;
	if (tmp != XSPI_CR_LOOP) {
		xspi->read_fn = xspi_read32_be;
		xspi->write_fn = xspi_write32_be;
	}

	edev->master->bits_per_word_mask = SPI_BPW_MASK(bits_per_word);
	xspi->bytes_per_word = bits_per_word / 8;
	xspi->buffer_size = xilinx_spi_find_buffer_size(xspi);

	err = request_irq(pci_irq_vector(pdev, 0),
			  xilinx_spi_irq, 0, driver_name, xspi);
	if (err) {
		dev_err(&pdev->dev, "Failed to request irq\n");
		goto err_request_irq;
	}

	xspi_init_hw(xspi);

	err = spi_bitbang_start(&xspi->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to start bit bang\n");
		goto err_spi_bitbang_start;
	}

	dev_info(&pdev->dev, "regs 0x%p, irq=%d\n", xspi->regs, xspi->irq);

	return 0;

err_spi_bitbang_start:
	free_irq(pci_irq_vector(edev->pdev, 0), xspi);
err_request_irq:
	iounmap(edev->hw_addr);
err_ioremap:
	pci_release_regions(pdev);
err_pci_request_regions:
	pci_disable_device(pdev);
err_pci_enable_device:
	spi_master_put(edev->master);
err_spi_alloc_master:
	kfree(edev);
	return err;
}

static void enet_spi_remove(struct pci_dev *pdev)
{
	struct enet_dev *edev = pci_get_drvdata(pdev);
	struct xilinx_spi *xspi = spi_master_get_devdata(edev->master);
	void __iomem *regs_base = xspi->regs;

	spi_bitbang_stop(&xspi->bitbang);


	/* Disable all the interrupts just in case */
	xspi->write_fn(0, regs_base + XIPIF_V123B_IIER_OFFSET);
	/* Disable the global IPIF interrupt */
	xspi->write_fn(0, regs_base + XIPIF_V123B_DGIER_OFFSET);

	free_irq(pci_irq_vector(edev->pdev, 0), xspi);
	spi_unregister_master(edev->master);
	iounmap(edev->hw_addr);
	pci_release_regions(edev->pdev);
	pci_disable_device(edev->pdev);
	spi_master_put(edev->master);
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
