/*
 * cros_ec_lpc_mec - LPC variant I/O for Microchip EC
 *
 * Copyright (C) 2016 Google, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This driver uses the Chrome OS EC byte-level message-based protocol for
 * communicating the keyboard state (which keys are pressed) from a keyboard EC
 * to the AP over some bus (such as i2c, lpc, spi).  The EC does debouncing,
 * but everything else (including deghosting) is done here.  The main
 * motivation for this is to keep the EC firmware as simple as possible, since
 * it cannot be easily upgraded and EC flash/IRAM space is relatively
 * expensive.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include "cros_ec_lpc_mec.h"

/*
 * This mutex must be held while accessing the EMI unit. We can't rely on the
 * EC mutex because memmap data may be accessed without it being held.
 */
static struct mutex io_mutex;
static u16 mec_emi_base, mec_emi_end;

/*
 * cros_ec_lpc_mec_emi_write_address
 *
 * Initialize EMI read / write at a given address.
 *
 * @addr:        Starting read / write address
 * @access_type: Type of access, typically 32-bit auto-increment
 */
static void cros_ec_lpc_mec_emi_write_address(u16 addr,
			enum cros_ec_lpc_mec_emi_access_mode access_type)
{
	outb((addr & 0xfc) | access_type, MEC_EMI_EC_ADDRESS_B0(mec_emi_base));
	outb((addr >> 8) & 0x7f, MEC_EMI_EC_ADDRESS_B1(mec_emi_base));
}

/**
 * cros_ec_lpc_mec_in_range() - Determine if addresses are in MEC EMI range.
 *
 * @offset: Address offset
 * @length: Number of bytes to check
 *
 * Return: 1 if in range, 0 if not, and -EINVAL on failure
 *         such as the mec range not being initialized
 */
int cros_ec_lpc_mec_in_range(unsigned int offset, unsigned int length)
{
	if (length == 0)
		return -EINVAL;

	if (WARN_ON(mec_emi_base == 0 || mec_emi_end == 0))
		return -EINVAL;

	if (offset >= mec_emi_base && offset < mec_emi_end) {
		if (WARN_ON(offset + length - 1 >= mec_emi_end))
			return -EINVAL;
		return 1;
	}

	if (WARN_ON(offset + length > mec_emi_base && offset < mec_emi_end))
		return -EINVAL;

	return 0;
}

/*
 * cros_ec_lpc_io_bytes_mec - Read / write bytes to MEC EMI port
 *
 * @io_type: MEC_IO_READ or MEC_IO_WRITE, depending on request
 * @offset:  Base read / write address
 * @length:  Number of bytes to read / write
 * @buf:     Destination / source buffer
 *
 * @return 8-bit checksum of all bytes read / written
 */
u8 cros_ec_lpc_io_bytes_mec(enum cros_ec_lpc_mec_io_type io_type,
			    unsigned int offset, unsigned int length,
			    u8 *buf)
{
	int i = 0;
	int io_addr;
	u8 sum = 0;
	enum cros_ec_lpc_mec_emi_access_mode access, new_access;

	/* Return checksum of 0 if window is not initialized */
	WARN_ON(mec_emi_base == 0 || mec_emi_end == 0);
	if (mec_emi_base == 0 || mec_emi_end == 0)
		return 0;

	/*
	 * Long access cannot be used on misaligned data since reading B0 loads
	 * the data register and writing B3 flushes.
	 */
	if (offset & 0x3 || length < 4)
		access = ACCESS_TYPE_BYTE;
	else
		access = ACCESS_TYPE_LONG_AUTO_INCREMENT;

	mutex_lock(&io_mutex);

	/* Initialize I/O at desired address */
	cros_ec_lpc_mec_emi_write_address(offset, access);

	/* Skip bytes in case of misaligned offset */
	io_addr = MEC_EMI_EC_DATA_B0(mec_emi_base) + (offset & 0x3);
	while (i < length) {
		while (io_addr <= MEC_EMI_EC_DATA_B3(mec_emi_base)) {
			if (io_type == MEC_IO_READ)
				buf[i] = inb(io_addr++);
			else
				outb(buf[i], io_addr++);

			sum += buf[i++];
			offset++;

			/* Extra bounds check in case of misaligned length */
			if (i == length)
				goto done;
		}

		/*
		 * Use long auto-increment access except for misaligned write,
		 * since writing B3 triggers the flush.
		 */
		if (length - i < 4 && io_type == MEC_IO_WRITE)
			new_access = ACCESS_TYPE_BYTE;
		else
			new_access = ACCESS_TYPE_LONG_AUTO_INCREMENT;

		if (new_access != access ||
		    access != ACCESS_TYPE_LONG_AUTO_INCREMENT) {
			access = new_access;
			cros_ec_lpc_mec_emi_write_address(offset, access);
		}

		/* Access [B0, B3] on each loop pass */
		io_addr = MEC_EMI_EC_DATA_B0(mec_emi_base);
	}

done:
	mutex_unlock(&io_mutex);

	return sum;
}
EXPORT_SYMBOL(cros_ec_lpc_io_bytes_mec);

void cros_ec_lpc_mec_init(unsigned int base, unsigned int end)
{
	mutex_init(&io_mutex);
	mec_emi_base = base;
	mec_emi_end = end;
}
EXPORT_SYMBOL(cros_ec_lpc_mec_init);

void cros_ec_lpc_mec_destroy(void)
{
	mutex_destroy(&io_mutex);
}
EXPORT_SYMBOL(cros_ec_lpc_mec_destroy);
