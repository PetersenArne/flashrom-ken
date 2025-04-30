/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2023 Christopher Lentocha
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

/* Driver for the VIA VL805 programmer hardware by VIA.
 * See http://www.via.com/ for more info.
 */

#include <stdlib.h>

#include "flash.h"
#include "programmer.h"
#include "platform/pci.h"
#include "spi.h"

/* Some of the registers have unknown purpose and are just used inside the init sequence replay. */
#define VL805_REG_0x30004		0x00030004
#define VL805_REG_STOP_POLLING		0x0004000c
#define VL805_REG_WB_EN			0x00040020
#define VL805_REG_SPI_OUTDATA		0x000400d0
#define VL805_REG_SPI_INDATA		0x000400e0
#define VL805_REG_SPI_TRANSACTION	0x000400f0
#define VL805_REG_CLK_DIV		0x000400f8
#define VL805_REG_SPI_CHIP_ENABLE_LEVEL	0x000400fc

struct vl805_spi_data {
    struct pci_dev *dev;
};

static struct dev_entry devs_vl805[] = {
	{0x1106, 0x3483, OK, "VIA", "VL805"},
	{0},
};

static void vl805_setregval(struct pci_dev *dev, int reg, uint32_t val)
{
	pci_write_long(dev, 0x78, reg);
	pci_write_long(dev, 0x7c, val);
}

static uint32_t vl805_getregval(struct pci_dev *dev, int reg)
{
	pci_write_long(dev, 0x78, reg);
	return pci_read_long(dev, 0x7c);
}

/* Send a SPI command to the flash chip. */
static int vl805_spi_send_command(const struct flashctx *flash,
			unsigned int writecnt,
			unsigned int readcnt,
			const unsigned char *writearr,
			unsigned char *readarr)
{
	struct vl805_spi_data *data = (struct vl805_spi_data *)flash->mst->spi.data;

	vl805_setregval(data->dev, VL805_REG_SPI_CHIP_ENABLE_LEVEL, 0x00000000);

	for (unsigned int i = 0; i < writecnt; i += 4) {
		const uint32_t curwritecnt = min(4, writecnt - i);
		uint32_t outdata = 0;
		for (unsigned int j = 0; j < curwritecnt; j++) {
			outdata <<= 8;
			outdata |= writearr[j + i];
		}
		vl805_setregval(data->dev, VL805_REG_SPI_OUTDATA, outdata);
		vl805_setregval(data->dev, VL805_REG_SPI_TRANSACTION, 0x00000580 | (curwritecnt << 3));
	}

	/* Superfluous, the original driver doesn't do that, but we want to have a quiet bus during read. */
	vl805_setregval(data->dev, VL805_REG_SPI_OUTDATA, 0);

	for (unsigned int i = 0; i < readcnt; i += 4) {
		const uint32_t curreadcnt = min(4, readcnt - i);
		vl805_setregval(data->dev, VL805_REG_SPI_TRANSACTION, 0x00000580 | (curreadcnt << 3));
		uint32_t indata = vl805_getregval(data->dev, VL805_REG_SPI_INDATA);
		for (unsigned int j = 0; j < curreadcnt; j++) {
			unsigned pos = curreadcnt - (j + 1);
			readarr[j + i] = (indata >> (8 * pos)) & 0xff;
		}
	}

	vl805_setregval(data->dev, VL805_REG_SPI_CHIP_ENABLE_LEVEL, 0x00000001);

	return 0;
}

static void vl805_mcu_active(struct pci_dev *dev, uint8_t val)
{
	pci_write_byte(dev, 0x43, val);
}


static int vl805_shutdown(void *data)
{
	struct vl805_spi_data *vl805_data = (struct vl805_spi_data *)data;
	vl805_mcu_active(vl805_data->dev, 0x0);
	free(data);
	return 0;
}

static const struct spi_master spi_master_vl805 = {
	.max_data_read	= MAX_DATA_READ_UNLIMITED,
	.max_data_write	= MAX_DATA_WRITE_UNLIMITED,
	.command	= vl805_spi_send_command,
	.multicommand	= default_spi_send_multicommand,
	.read		= default_spi_read,
	.write_256	= default_spi_write_256,
	.probe_opcode   = default_spi_probe_opcode,
	.features       = SPI_MASTER_4BA,
	.shutdown       = vl805_shutdown,
};


static int vl805_init(const struct programmer_cfg *cfg)
{
	uint32_t val;
	struct pci_dev *dev = pcidev_init(cfg, devs_vl805, PCI_BASE_ADDRESS_0);
	if (!dev)
		return 1;

	struct vl805_spi_data *data = calloc(1, sizeof(*data));
	if (!data) {
		msg_perr("cannot allocate memory for vl805_data\n");
		return 1;
	}

	vl805_mcu_active(dev, 0x1);
	val = pci_read_long(dev, 0x50);
	msg_pdbg("VL805 firmware version %#08"PRIx32"\n", val);

	vl805_setregval(dev, VL805_REG_SPI_CHIP_ENABLE_LEVEL, 0x00000001);
	vl805_setregval(dev, VL805_REG_0x30004, 0x00000200);
	vl805_setregval(dev, VL805_REG_WB_EN, 0xffffff01);
	vl805_setregval(dev, VL805_REG_STOP_POLLING, 0x00000001);

	/* We send 4 uninitialized(?) bytes to the flash chip here. */
	vl805_setregval(dev, VL805_REG_SPI_TRANSACTION, 0x000005a0);
	vl805_setregval(dev, VL805_REG_CLK_DIV, 0x0000000a);

	/* Some sort of cleanup sequence, just copied from the logs. */
	vl805_setregval(dev, VL805_REG_SPI_TRANSACTION, 0x00000000);

	data->dev = dev;
	return register_spi_master(&spi_master_vl805, data);
}

const struct programmer_entry programmer_vl805 = {
		.name			= "vl805",
		.type			= PCI,
		.devs.dev		= devs_vl805,
		.init			= vl805_init,
};
