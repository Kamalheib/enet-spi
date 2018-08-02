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

#ifndef _SPI_ENET_H_
#define _SPI_ENET_H_

#define DRV_NAME 		"enet-spi"
#define DRV_VERSION		"0.1"
#define PCI_VENDOR_ID_ENET	0x10ee
#define PCI_DEVICE_ID_ENET	0x7022
#define SPI_BASE_ADDR		0xb000

/* Register definitions as per "OPB Serial Peripheral Interface (SPI) (v1.00e)
 * Product Specification", DS464
 */
#define XSPI_CR_OFFSET		0x60	/* Control Register */

#define XSPI_CR_LOOP		0x01
#define XSPI_CR_ENABLE		0x02
#define XSPI_CR_MASTER_MODE	0x04
#define XSPI_CR_CPOL		0x08
#define XSPI_CR_CPHA		0x10
#define XSPI_CR_MODE_MASK	(XSPI_CR_CPHA | XSPI_CR_CPOL | \
				 XSPI_CR_LSB_FIRST | XSPI_CR_LOOP)
#define XSPI_CR_TXFIFO_RESET	0x20
#define XSPI_CR_RXFIFO_RESET	0x40
#define XSPI_CR_MANUAL_SSELECT	0x80
#define XSPI_CR_TRANS_INHIBIT	0x100
#define XSPI_CR_LSB_FIRST	0x200

#define XSPI_SR_OFFSET		0x64	/* Status Register */

#define XSPI_SR_RX_EMPTY_MASK	0x01	/* Receive FIFO is empty */
#define XSPI_SR_RX_FULL_MASK	0x02	/* Receive FIFO is full */
#define XSPI_SR_TX_EMPTY_MASK	0x04	/* Transmit FIFO is empty */
#define XSPI_SR_TX_FULL_MASK	0x08	/* Transmit FIFO is full */
#define XSPI_SR_MODE_FAULT_MASK	0x10	/* Mode fault error */

#define XSPI_TXD_OFFSET		0x68	/* Data Transmit Register */
#define XSPI_RXD_OFFSET		0x6c	/* Data Receive Register */

#define XSPI_SSR_OFFSET		0x70	/* 32-bit Slave Select Register */

/* Register definitions as per "OPB IPIF (v3.01c) Product Specification", DS414
 * IPIF registers are 32 bit
 */
#define XIPIF_V123B_DGIER_OFFSET	0x1c	/* IPIF global int enable reg */
#define XIPIF_V123B_GINTR_ENABLE	0x80000000

#define XIPIF_V123B_IISR_OFFSET		0x20	/* IPIF interrupt status reg */
#define XIPIF_V123B_IIER_OFFSET		0x28	/* IPIF interrupt enable reg */

#define XSPI_INTR_MODE_FAULT		0x01	/* Mode fault error */
#define XSPI_INTR_SLAVE_MODE_FAULT	0x02	/* Selected as slave while
						 * disabled */
#define XSPI_INTR_TX_EMPTY		0x04	/* TxFIFO is empty */
#define XSPI_INTR_TX_UNDERRUN		0x08	/* TxFIFO was underrun */
#define XSPI_INTR_RX_FULL		0x10	/* RxFIFO is full */
#define XSPI_INTR_RX_OVERRUN		0x20	/* RxFIFO was overrun */
#define XSPI_INTR_TX_HALF_EMPTY		0x40	/* TxFIFO is half empty */

#define XIPIF_V123B_RESETR_OFFSET	0x40	/* IPIF reset register */
#define XIPIF_V123B_RESET_MASK		0x0a	/* the value to write */

#define BITBANG_CS_ACTIVE		1
#define BITBANG_CS_INACTIVE		0

#endif
