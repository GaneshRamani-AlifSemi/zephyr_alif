/*
 * Copyright (C) 2026 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mxicy_mx66uw_mspi
/*
 * This driver supports MX66UW devices in 1S-1S-1S mode during
 * initialization and 8D-8D-8D mode for normal operation.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/flash.h>
#include "spi_nor.h"

LOG_MODULE_REGISTER(flash_mspi_mx66uw, CONFIG_FLASH_LOG_LEVEL);

#define MX66UW_WRITE_SIZE                2
#define NOR_ERASE_VALUE                  0xff

#define MX66UW_VENDOR_ID                 0xC2

#define MX66UW_PAGE_SIZE                 0x100
#define MX66UW_SECTOR_SIZE               0x1000

#define MX66UW_SPI_WRITE_CR2_CMD         0x72
#define MX66UW_OPI_WRITE_CR2_CMD         0x728D
#define MX66UW_OPI_WREN_CMD              0x06F9
#define MX66UW_OPI_WRDI_CMD              0x04FB
#define MX66UW_OPI_RDSR_CMD              0x05FA
#define MX66UW_OPI_RDID_CMD              0x9F60
#define MX66UW_OPI_READ_SECURITY_CMD     0x2BD4
#define MX66UW_OPI_SECTOR_ERASE_CMD      0x21DE
#define MX66UW_OPI_CHIP_ERASE_CMD        0x609F
#define MX66UW_OPI_READ_CMD              0xEE11
#define MX66UW_OPI_PAGE_PROGRAM_CMD      0x12ED

#define MX66UW_CR2_MODE_ADDR             0x00000000
#define MX66UW_CR2_DUMMY_ADDR            0x00000300
#define MX66UW_CR2_DTR_OPI_ENABLE        BIT(1)

#define MX66UW_STATUS_DDR_DUMMY          4
#define MX66UW_SECURITY_ERASE_ERROR      BIT(6)
#define MX66UW_SECURITY_PROGRAM_ERROR    BIT(5)

#define MX66UW_DATA_XFER_MODE            MSPI_PIO

static bool mx66uw_is_octal_ddr_cfg(const struct mspi_dev_cfg *dev_cfg)
{
	return dev_cfg->io_mode == MSPI_IO_MODE_OCTAL &&
	       dev_cfg->data_rate == MSPI_DATA_RATE_DUAL;
}

struct flash_mspi_mx66uw_config {
	uint32_t                            mem_size;
	struct flash_parameters             flash_param;
	struct flash_pages_layout           page_layout;

	const struct device                 *bus;
	struct mspi_dev_id                  dev_id;
	struct mspi_dev_cfg                 serial_cfg;
	struct mspi_dev_cfg                 tar_dev_cfg;

	MSPI_XIP_CFG_STRUCT_DECLARE(tar_xip_cfg)
	MSPI_XIP_BASE_ADDR_DECLARE(xip_base_addr)
	MSPI_SCRAMBLE_CFG_STRUCT_DECLARE(tar_scramble_cfg)
	MSPI_TIMING_CFG_STRUCT_DECLARE(tar_timing_cfg)
	MSPI_TIMING_PARAM_DECLARE(timing_cfg_mask)

	bool                                sw_multi_periph;

	struct gpio_dt_spec                 reset_gpio;
	uint32_t                            reset_pulse_us;
	uint32_t                            reset_recovery_us;
};

struct flash_mspi_mx66uw_data {
	struct mspi_dev_cfg                 dev_cfg;
	bool                                octal_ddr;
	struct mspi_xip_cfg                 xip_cfg;
	struct mspi_scramble_cfg            scramble_cfg;
	struct mspi_timing_cfg              timing_cfg;
	struct mspi_xfer                    trans;
	struct mspi_xfer_packet             packet;

	struct k_sem                        lock;
	uint8_t                             id[20];
};

static int mx66uw_get_dummy_code(uint8_t rx_dummy, uint8_t *dummy_code)
{
	if (rx_dummy < 6 || rx_dummy > 20 || (rx_dummy & 1U) != 0U) {
		return -EINVAL;
	}

	*dummy_code = (20U - rx_dummy) / 2U;
	return 0;
}

static int flash_mspi_mx66uw_enter_command_mode(const struct device *flash)
{
	const struct flash_mspi_mx66uw_config *cfg  = flash->config;
	struct flash_mspi_mx66uw_data         *data = flash->data;
	int ret = 0;

	/* Command-mode switching is not needed after entering Octal DDR mode. */
	if (data->octal_ddr) {
		return 0;
	}

	if (cfg->serial_cfg.io_mode == data->dev_cfg.io_mode) {
		return 0;
	}

	ret = mspi_dev_config(cfg->bus, &cfg->dev_id,
			      MSPI_DEVICE_CONFIG_ALL, &cfg->serial_cfg);
	if (ret) {
		LOG_ERR("Failed to enter command mode/%u", __LINE__);
		return -EIO;
	}
	return 0;
}

static int flash_mspi_mx66uw_exit_command_mode(const struct device *flash)
{
	const struct flash_mspi_mx66uw_config *cfg  = flash->config;
	struct flash_mspi_mx66uw_data         *data = flash->data;
	int ret;

	if (data->octal_ddr) {
		return 0;
	}

	if (cfg->serial_cfg.io_mode == data->dev_cfg.io_mode) {
		return 0;
	}

	ret = mspi_dev_config(cfg->bus, &cfg->dev_id,
			      MSPI_DEVICE_CONFIG_ALL, &data->dev_cfg);
	if (ret) {
		LOG_ERR("Failed to exit command mode/%u", __LINE__);
		return -EIO;
	}
	return 0;
}

static int flash_mspi_mx66uw_command_write_len(const struct device *flash, uint16_t cmd,
						  uint8_t cmd_len, uint32_t addr,
						  uint16_t addr_len, uint32_t tx_dummy,
						  uint8_t *wdata, uint32_t length)
{
	const struct flash_mspi_mx66uw_config *cfg  = flash->config;
	struct flash_mspi_mx66uw_data         *data = flash->data;
	int ret;

	data->packet.dir              = MSPI_TX;
	data->packet.cmd              = cmd;
	data->packet.address          = addr;
	data->packet.data_buf         = wdata;
	data->packet.num_bytes        = length;

	data->trans.async             = false;
	data->trans.xfer_mode         = MSPI_PIO;
	data->trans.tx_dummy          = tx_dummy;
	data->trans.rx_dummy          = data->dev_cfg.rx_dummy;
	data->trans.cmd_length        = cmd_len;
	data->trans.addr_length       = addr_len;
	data->trans.hold_ce           = false;
	data->trans.packets           = &data->packet;
	data->trans.num_packet        = 1;
	data->trans.timeout           = 10;

	ret = mspi_transceive(cfg->bus, &cfg->dev_id, (const struct mspi_xfer *)&data->trans);
	if (ret) {
		LOG_ERR("MSPI write transaction failed with code: %d/%u", ret, __LINE__);
		return -EIO;
	}
	return ret;
}

static int flash_mspi_mx66uw_command_write(const struct device *flash, uint16_t cmd,
					      uint32_t addr, uint16_t addr_len, uint32_t tx_dummy,
					      uint8_t *wdata, uint32_t length)
{
	struct flash_mspi_mx66uw_data *data = flash->data;

	return flash_mspi_mx66uw_command_write_len(flash, cmd,
						      data->octal_ddr ? 2 : 1, addr, addr_len,
						      tx_dummy, wdata, length);
}

static int flash_mspi_mx66uw_command_read(const struct device *flash, uint16_t cmd,
					     uint32_t addr, uint16_t addr_len, uint32_t rx_dummy,
					     uint8_t *rdata, uint32_t length)
{
	const struct flash_mspi_mx66uw_config *cfg  = flash->config;
	struct flash_mspi_mx66uw_data         *data = flash->data;
	uint8_t                                   status_frame[2];
	bool                                      octal_status_read = false;
	int ret;

	/* Octal DDR status reads return a two-byte status frame. */
	if (data->octal_ddr && length == 1U) {
		octal_status_read = true;
	}

	data->packet.dir              = MSPI_RX;
	data->packet.cmd              = cmd;
	data->packet.address          = addr;
	data->packet.data_buf         = octal_status_read ? status_frame : rdata;
	data->packet.num_bytes        = octal_status_read ? sizeof(status_frame) : length;

	data->trans.async             = false;
	data->trans.xfer_mode         = MSPI_PIO;
	data->trans.rx_dummy          = rx_dummy;
	data->trans.tx_dummy          = data->dev_cfg.tx_dummy;
	data->trans.cmd_length        = data->octal_ddr ? 2 : 1;
	data->trans.addr_length       = addr_len;
	data->trans.hold_ce           = false;
	data->trans.packets           = &data->packet;
	data->trans.num_packet        = 1;
	data->trans.timeout           = 10;

	ret = mspi_transceive(cfg->bus, &cfg->dev_id, (const struct mspi_xfer *)&data->trans);
	if (ret) {
		LOG_ERR("MSPI read transaction failed with code: %d/%u", ret, __LINE__);
		return -EIO;
	}

	if (octal_status_read) {
		/* Extract the first byte from the 16-bit Octal DTR frame. */
		*rdata = status_frame[0];
	}

	return ret;
}

static void acquire(const struct device *flash)
{
	const struct flash_mspi_mx66uw_config *cfg  = flash->config;
	struct flash_mspi_mx66uw_data         *data = flash->data;

	k_sem_take(&data->lock, K_FOREVER);

	if (cfg->sw_multi_periph) {
		while (mspi_dev_config(cfg->bus, &cfg->dev_id,
				       MSPI_DEVICE_CONFIG_ALL, &data->dev_cfg)) {
			;
		}
	} else {
		while (mspi_dev_config(cfg->bus, &cfg->dev_id,
				       MSPI_DEVICE_CONFIG_NONE, NULL)) {
			;
		}
	}
}

static void release(const struct device *flash)
{
	const struct flash_mspi_mx66uw_config *cfg  = flash->config;
	struct flash_mspi_mx66uw_data         *data = flash->data;

	while (mspi_get_channel_status(cfg->bus, 0)) {
		;
	}

	k_sem_give(&data->lock);
}

static int flash_mspi_mx66uw_write_enable(const struct device *flash)
{
	struct flash_mspi_mx66uw_data *data = flash->data;
	int ret;

	LOG_DBG("Enabling write");
	ret = flash_mspi_mx66uw_command_write(flash,
					     data->octal_ddr ? MX66UW_OPI_WREN_CMD : SPI_NOR_CMD_WREN,
					     0, 0, 0, NULL, 0);

	return ret;
}

static int flash_mspi_mx66uw_write_disable(const struct device *flash)
{
	struct flash_mspi_mx66uw_data *data = flash->data;
	int ret;

	LOG_DBG("Disabling write");
	ret = flash_mspi_mx66uw_command_write(flash,
					     data->octal_ddr ? MX66UW_OPI_WRDI_CMD : SPI_NOR_CMD_WRDI,
					     0, 0, 0, NULL, 0);

	return ret;
}

static int flash_mspi_mx66uw_is_ready(const struct device *flash)
{
	uint8_t status = 0;
	struct flash_mspi_mx66uw_data *data = flash->data;
	uint32_t rx_dummy  = 0;
	uint32_t timeout   = 400; /* max tSSE */
	int ret;

	/* Update dummy cycles for Octal DDR mode. */
	if (data->octal_ddr) {
		rx_dummy = MX66UW_STATUS_DDR_DUMMY;
	}

	do {
		ret = flash_mspi_mx66uw_command_read(flash,
						    data->octal_ddr ? MX66UW_OPI_RDSR_CMD :
						    SPI_NOR_CMD_RDSR,
						    0, data->octal_ddr ? 4 : 0, rx_dummy,
						    &status, 1);
		if (ret) {
			LOG_ERR("Could not read status register");
			return ret;
		}

		if ((status & SPI_NOR_WIP_BIT) == 0U) {
			LOG_DBG("Device is ready");
			break;
		}

		k_sleep(K_MSEC(1));
		timeout--;
	} while (timeout);

	if (timeout == 0) {
		LOG_ERR("Operation timed out");
		return -ETIMEDOUT;
	}

	return ret;
}

static int flash_mspi_mx66uw_reset(const struct device *flash)
{
	const struct flash_mspi_mx66uw_config *cfg  = flash->config;
	struct flash_mspi_mx66uw_data         *data = flash->data;
	int ret;

	LOG_DBG("RESETTING");

	if (cfg->reset_gpio.port) {
		if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
			LOG_ERR("Device %s is not ready",
				cfg->reset_gpio.port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->reset_gpio,
					   GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to activate RESET: %d", ret);
			return -EIO;
		}

		if (cfg->reset_pulse_us != 0) {
			k_busy_wait(cfg->reset_pulse_us);
		}

		ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
		if (ret < 0) {
			LOG_ERR("Failed to deactivate RESET: %d", ret);
			return -EIO;
		}

		if (cfg->reset_recovery_us != 0) {
			k_busy_wait(cfg->reset_recovery_us);
		}
	} else {
		/* A system reset does not reset the flash. If the previous boot left
		 * it in Octal DDR mode, reset it with the 16-bit octal opcodes first.
		 * An octal reset returns the flash to its default serial mode.
		 */
		if (mx66uw_is_octal_ddr_cfg(&cfg->tar_dev_cfg)) {
			ret = mspi_dev_config(cfg->bus, &cfg->dev_id,
					      MSPI_DEVICE_CONFIG_ALL, &cfg->tar_dev_cfg);
			if (ret) {
				return ret;
			}
			data->dev_cfg = cfg->tar_dev_cfg;
			data->octal_ddr = true;

			ret = flash_mspi_mx66uw_command_write_len(flash,
					SPI_NOR_OCMD_RESET_EN, 2, 0, 0, 0, NULL, 0);
			if (ret) {
				return ret;
			}
			ret = flash_mspi_mx66uw_command_write_len(flash,
					SPI_NOR_OCMD_RESET_MEM, 2, 0, 0, 0, NULL, 0);
			if (ret) {
				return ret;
			}
		}

		ret = mspi_dev_config(cfg->bus, &cfg->dev_id,
				      MSPI_DEVICE_CONFIG_ALL, &cfg->serial_cfg);
		if (ret) {
			return ret;
		}
		data->dev_cfg = cfg->serial_cfg;
		data->octal_ddr = false;

		ret = flash_mspi_mx66uw_command_write(flash, SPI_NOR_CMD_RESET_EN,
							 0, 0, 0, NULL, 0);
		if (ret) {
			return ret;
		}
		ret = flash_mspi_mx66uw_command_write(flash, SPI_NOR_CMD_RESET_MEM,
							 0, 0, 0, NULL, 0);
		if (ret) {
			return ret;
		}
	}

	ret = flash_mspi_mx66uw_is_ready(flash);

	return ret;
}

static int flash_mspi_mx66uw_get_vendor_id(const struct device *flash, uint8_t *vendor_id)
{
	struct flash_mspi_mx66uw_data *data = flash->data;
	int ret;

	if (vendor_id == NULL) {
		return -EINVAL;
	}

	LOG_DBG("Reading id");
	ret = flash_mspi_mx66uw_command_read(flash,
						data->octal_ddr ? MX66UW_OPI_RDID_CMD :
						SPI_NOR_CMD_RDID,
						0, data->octal_ddr ? 4 : 0,
						data->octal_ddr ? MX66UW_STATUS_DDR_DUMMY : 0,
						(uint8_t *)data->id,
						data->octal_ddr ? 1 : 3);
	*vendor_id = data->id[0];

	return ret;
}

static int flash_mspi_mx66uw_erase_sector(const struct device *flash, off_t addr)
{
	struct flash_mspi_mx66uw_data *data = flash->data;
	int ret;

	LOG_DBG("Erasing sector at 0x%08zx", (ssize_t)addr);

	ret = flash_mspi_mx66uw_command_write(flash, MX66UW_OPI_SECTOR_ERASE_CMD, addr,
						 data->dev_cfg.addr_length, 0, NULL, 0);

	return ret;
}

static int flash_mspi_mx66uw_erase_chip(const struct device *flash)
{
	int ret;

	LOG_DBG("Erasing chip");

	ret = flash_mspi_mx66uw_command_write(flash, MX66UW_OPI_CHIP_ERASE_CMD,
						 0, 0, 0, NULL, 0);

	return ret;
}

static int flash_mspi_mx66uw_page_program(const struct device *flash, off_t offset,
					     void *wdata, size_t len)
{
	const struct flash_mspi_mx66uw_config *cfg  = flash->config;
	struct flash_mspi_mx66uw_data         *data = flash->data;
	int ret;

	data->packet.dir              = MSPI_TX;
	data->packet.cmd              = MX66UW_OPI_PAGE_PROGRAM_CMD;
	data->packet.address          = offset;
	data->packet.data_buf         = wdata;
	data->packet.num_bytes        = len;

	data->trans.async             = false;
	data->trans.xfer_mode         = MX66UW_DATA_XFER_MODE;
	data->trans.tx_dummy          = data->dev_cfg.tx_dummy;
	data->trans.rx_dummy          = data->dev_cfg.rx_dummy;
	data->trans.cmd_length        = data->dev_cfg.cmd_length;
	data->trans.addr_length       = data->dev_cfg.addr_length;
	data->trans.hold_ce           = false;
	data->trans.priority          = MSPI_XFER_PRIORITY_MEDIUM;
	data->trans.packets           = &data->packet;
	data->trans.num_packet        = 1;
	data->trans.timeout           = CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE;

	LOG_DBG("Page programming %d bytes to 0x%08zx", len, (ssize_t)offset);

	ret = mspi_transceive(cfg->bus, &cfg->dev_id, (const struct mspi_xfer *)&data->trans);
	if (ret) {
		LOG_ERR("MSPI write transaction failed with code: %d/%u", ret, __LINE__);
		return -EIO;
	}
	return ret;
}

static int flash_mspi_mx66uw_busy_wait(const struct device *flash, unsigned int timeout)
{
	struct flash_mspi_mx66uw_data *data = flash->data;
	uint8_t status = 0;
	uint8_t security = 0;
	uint32_t rx_dummy  = 0;
	int ret;

	if (data->octal_ddr) {
		rx_dummy = MX66UW_STATUS_DDR_DUMMY;
	}

	do {
		LOG_DBG("Reading status register");
		ret = flash_mspi_mx66uw_command_read(flash,
						    data->octal_ddr ? MX66UW_OPI_RDSR_CMD :
						    SPI_NOR_CMD_RDSR,
						    0, data->octal_ddr ? 4 : 0, rx_dummy,
						    &status, 1);
		if (ret) {
			LOG_ERR("Could not read status");
			return ret;
		}

		k_sleep(K_MSEC(1));
		timeout--;
	} while ((status & SPI_NOR_WIP_BIT) && timeout);

	if (timeout == 0) {
		LOG_ERR("Operation timed out");
		return -ETIMEDOUT;
	}

	if (data->octal_ddr) {
		ret = flash_mspi_mx66uw_command_read(flash,
						    MX66UW_OPI_READ_SECURITY_CMD,
						    0, 4, MX66UW_STATUS_DDR_DUMMY,
						    &security, 1);
		if (ret) {
			return ret;
		}

		if (security & (MX66UW_SECURITY_ERASE_ERROR |
				MX66UW_SECURITY_PROGRAM_ERROR)) {
			LOG_ERR("Flash operation failed, security: 0x%x", security);
			return -EIO;
		}
	}

	return ret;
}

static int flash_mspi_mx66uw_read(const struct device *flash, off_t offset, void *rdata,
				     size_t len)
{
	const struct flash_mspi_mx66uw_config *cfg  = flash->config;
	struct flash_mspi_mx66uw_data         *data = flash->data;
	int ret = 0;

	acquire(flash);

#if CONFIG_FLASH_MSPI_XIP_READ
	if (cfg->tar_xip_cfg.enable) {
		uint32_t xip_addr = cfg->xip_base_addr + cfg->tar_xip_cfg.address_offset + offset;

		memcpy(rdata, (void *)xip_addr, len);
	} else {
#endif /* CONFIG_FLASH_MSPI_XIP_READ */

		data->packet.dir              = MSPI_RX;
		data->packet.cmd              = MX66UW_OPI_READ_CMD;
		data->packet.address          = offset;
		data->packet.data_buf         = rdata;
		data->packet.num_bytes        = len;

		data->trans.async             = false;
		data->trans.xfer_mode         = MX66UW_DATA_XFER_MODE;
		data->trans.tx_dummy          = data->dev_cfg.tx_dummy;
		data->trans.rx_dummy          = data->dev_cfg.rx_dummy;
		data->trans.cmd_length        = data->dev_cfg.cmd_length;
		data->trans.addr_length       = data->dev_cfg.addr_length;
		data->trans.hold_ce           = false;
		data->trans.priority          = MSPI_XFER_PRIORITY_MEDIUM;
		data->trans.packets           = &data->packet;
		data->trans.num_packet        = 1;
		data->trans.timeout           = CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE;

		LOG_DBG("Read %d bytes from 0x%08zx", len, (ssize_t)offset);

		ret = mspi_transceive(cfg->bus, &cfg->dev_id,
				      (const struct mspi_xfer *)&data->trans);
		if (ret) {
			LOG_ERR("MSPI read transaction failed with code: %d/%u", ret, __LINE__);
			ret = -EIO;
			goto out;
		}

#if CONFIG_FLASH_MSPI_XIP_READ
	}
#endif /* CONFIG_FLASH_MSPI_XIP_READ */

out:
	release(flash);

	return ret;
}

static int flash_mspi_mx66uw_write(const struct device *flash, off_t offset, const void *wdata,
				      size_t len)
{
	const struct flash_mspi_mx66uw_config *cfg = flash->config;
	int ret = 0;
	uint8_t                               *src = (uint8_t *)wdata;
	int                                   i;

	if (!IS_ALIGNED(offset, cfg->flash_param.write_block_size) ||
	    !IS_ALIGNED(len, cfg->flash_param.write_block_size)) {
		LOG_ERR("Write offset and size must be aligned to %u bytes",
			cfg->flash_param.write_block_size);
		return -EINVAL;
	}

	acquire(flash);

	while (len) {
		/* If the offset isn't a multiple of the NOR page size, we first need
		 * to write the remaining part that fits, otherwise the write could
		 * be wrapped around within the same page
		 */
		i = MIN(MX66UW_PAGE_SIZE - (offset % MX66UW_PAGE_SIZE), len);

		ret = flash_mspi_mx66uw_enter_command_mode(flash);
		if (ret) {
			goto out;
		}

		ret = flash_mspi_mx66uw_write_enable(flash);
		if (ret) {
			goto out;
		}

		ret = flash_mspi_mx66uw_exit_command_mode(flash);
		if (ret) {
			goto out;
		}

		ret = flash_mspi_mx66uw_page_program(flash, offset, src, i);
		if (ret) {
			goto out;
		}

		ret = flash_mspi_mx66uw_enter_command_mode(flash);
		if (ret) {
			goto out;
		}

		ret = flash_mspi_mx66uw_busy_wait(flash, 100);
		if (ret) {
			goto out;
		}

		ret = flash_mspi_mx66uw_exit_command_mode(flash);
		if (ret) {
			goto out;
		}

		src    += i;
		offset += i;
		len    -= i;
	}

	ret = flash_mspi_mx66uw_write_disable(flash);
	if (ret) {
		goto out;
	}

out:
	release(flash);

	return ret;
}

static int flash_mspi_mx66uw_erase(const struct device *flash, off_t offset, size_t size)
{
	const struct flash_mspi_mx66uw_config *cfg = flash->config;
	int ret = 0;
	const size_t num_sectors     = size / SPI_NOR_SECTOR_SIZE;
	int          i;

	acquire(flash);

	if (offset % SPI_NOR_SECTOR_SIZE) {
		LOG_ERR("Invalid offset");
		ret = -EINVAL;
		goto out;
	}

	if (size % SPI_NOR_SECTOR_SIZE) {
		LOG_ERR("Invalid size");
		ret = -EINVAL;
		goto out;
	}

	ret = flash_mspi_mx66uw_enter_command_mode(flash);
	if (ret) {
		goto out;
	}

	if ((offset == 0) && (size == cfg->mem_size)) {
		ret = flash_mspi_mx66uw_write_enable(flash);
		if (ret) {
			goto out;
		}

		ret = flash_mspi_mx66uw_erase_chip(flash);
		if (ret) {
			goto out;
		}

		ret = flash_mspi_mx66uw_busy_wait(flash, 2 * 60 * 1000);
		if (ret) {
			goto out;
		}
	} else {
		for (i = 0; i < num_sectors; i++) {
			ret = flash_mspi_mx66uw_write_enable(flash);
			if (ret) {
				goto out;
			}

			ret = flash_mspi_mx66uw_erase_sector(flash, offset);
			if (ret) {
				goto out;
			}

			ret = flash_mspi_mx66uw_busy_wait(flash, 400);
			if (ret) {
				goto out;
			}

			offset += SPI_NOR_SECTOR_SIZE;
		}
	}

	ret = flash_mspi_mx66uw_exit_command_mode(flash);
	if (ret) {
		goto out;
	}

out:
	release(flash);

	return ret;
}

static const struct flash_parameters *
flash_mspi_mx66uw_get_parameters(const struct device *flash)
{
	const struct flash_mspi_mx66uw_config *cfg = flash->config;

	return &cfg->flash_param;
}

static int flash_mspi_mx66uw_get_size(const struct device *flash, uint64_t *size)
{
	const struct flash_mspi_mx66uw_config *cfg = flash->config;

	*size = cfg->mem_size;
	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_mspi_mx66uw_pages_layout(const struct device              *flash,
					      const struct flash_pages_layout **layout,
					      size_t                           *layout_size)
{
	const struct flash_mspi_mx66uw_config *cfg = flash->config;

	*layout      = &cfg->page_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static int flash_mspi_mx66uw_init(const struct device *flash)
{
	const struct flash_mspi_mx66uw_config *cfg  = flash->config;
	struct flash_mspi_mx66uw_data         *data = flash->data;
	bool                                      target_octal_ddr;
	uint8_t                                   vendor_id;
	uint8_t                                   reg_dummy;
	uint8_t                                   mode = MX66UW_CR2_DTR_OPI_ENABLE;
	uint8_t                                   dummy_data[2];

	if (!device_is_ready(cfg->bus)) {
		LOG_ERR("Controller device is not ready.");
		return -ENODEV;
	}

	target_octal_ddr = mx66uw_is_octal_ddr_cfg(&cfg->tar_dev_cfg);

	switch (cfg->tar_dev_cfg.io_mode) {
	case MSPI_IO_MODE_SINGLE:
	case MSPI_IO_MODE_OCTAL:
	case MSPI_IO_MODE_OCTAL_1_1_8:
	case MSPI_IO_MODE_OCTAL_1_8_8:
		break;
	default:
		LOG_ERR("bus mode %d not supported/%u", cfg->tar_dev_cfg.io_mode, __LINE__);
		return -EIO;
	}

	if (!target_octal_ddr) {
		LOG_ERR("MX66UW requires Octal DTR target configuration");
		return -ENOTSUP;
	}

	if (cfg->tar_dev_cfg.cmd_length != 2 ||
	    cfg->tar_dev_cfg.addr_length != 4 ||
	    cfg->tar_dev_cfg.endian != MSPI_XFER_BIG_ENDIAN ||
	    !cfg->tar_dev_cfg.dqs_enable) {
		LOG_ERR("MX66UW requires 16-bit commands, 32-bit addresses, "
			"big endian, and DQS");
		return -ENOTSUP;
	}

	if (mspi_dev_config(cfg->bus, &cfg->dev_id, MSPI_DEVICE_CONFIG_ALL, &cfg->serial_cfg)) {
		LOG_ERR("Failed to config mspi controller/%u", __LINE__);
		return -EIO;
	}
	data->dev_cfg = cfg->serial_cfg;
	data->octal_ddr = false;

	if (flash_mspi_mx66uw_reset(flash)) {
		LOG_ERR("Could not reset Flash/%u", __LINE__);
		return -EIO;
	}

	if (flash_mspi_mx66uw_get_vendor_id(flash, &vendor_id)) {
		LOG_ERR("Could not read vendor id/%u", __LINE__);
		return -EIO;
	}
	LOG_DBG("Vendor id: 0x%0x", vendor_id);
	if (vendor_id != MX66UW_VENDOR_ID) {
		LOG_WRN("Vendor ID does not match expected value of 0x%0x/%u",
			MX66UW_VENDOR_ID, __LINE__);
	}

	if (mx66uw_get_dummy_code(cfg->tar_dev_cfg.rx_dummy, &reg_dummy)) {
		LOG_ERR("Unsupported read dummy cycle count: %u",
			cfg->tar_dev_cfg.rx_dummy);
		return -ENOTSUP;
	}

	if (flash_mspi_mx66uw_write_enable(flash)) {
		return -EIO;
	}

	/* Switch the flash to 8D-8D-8D mode from its serial reset state. */
	if (flash_mspi_mx66uw_command_write(flash, MX66UW_SPI_WRITE_CR2_CMD,
					       MX66UW_CR2_MODE_ADDR, 4, 0,
					       &mode, 1)) {
		return -EIO;
	}
	data->octal_ddr = true;

	if (mspi_dev_config(cfg->bus, &cfg->dev_id,
			    MSPI_DEVICE_CONFIG_ALL, &cfg->tar_dev_cfg)) {
		LOG_ERR("Failed to config mspi controller/%u", __LINE__);
		return -EIO;
	}
	data->dev_cfg = cfg->tar_dev_cfg;

	if (flash_mspi_mx66uw_write_enable(flash)) {
		return -EIO;
	}

	/* CR2 is transferred as a 16-bit data frame in Octal DTR mode. */
	dummy_data[0] = reg_dummy;
	dummy_data[1] = 0;
	if (flash_mspi_mx66uw_command_write(flash, MX66UW_OPI_WRITE_CR2_CMD,
					       MX66UW_CR2_DUMMY_ADDR, 4, 0,
					       dummy_data, sizeof(dummy_data))) {
		return -EIO;
	}

	if (flash_mspi_mx66uw_busy_wait(flash, 100)) {
		return -EIO;
	}

	if (flash_mspi_mx66uw_get_vendor_id(flash, &vendor_id) ||
	    vendor_id != MX66UW_VENDOR_ID) {
		LOG_ERR("Could not verify MX66UW in Octal DTR mode");
		return -ENODEV;
	}

#if CONFIG_MSPI_TIMING
	if (mspi_timing_config(cfg->bus, &cfg->dev_id, cfg->timing_cfg_mask,
				(void *)&cfg->tar_timing_cfg)) {
		LOG_ERR("Failed to config mspi timing/%u", __LINE__);
		return -EIO;
	}
	data->timing_cfg = cfg->tar_timing_cfg;
#endif

#if CONFIG_MSPI_XIP
	if (cfg->tar_xip_cfg.enable) {
		if (mspi_xip_config(cfg->bus, &cfg->dev_id, &cfg->tar_xip_cfg)) {
			LOG_ERR("Failed to enable XIP/%u", __LINE__);
			return -EIO;
		}
		data->xip_cfg = cfg->tar_xip_cfg;
	}
#endif

#if CONFIG_MSPI_SCRAMBLE
	if (cfg->tar_scramble_cfg.enable) {
		if (mspi_scramble_config(cfg->bus, &cfg->dev_id, &cfg->tar_scramble_cfg)) {
			LOG_ERR("Failed to enable scrambling/%u", __LINE__);
			return -EIO;
		}
		data->scramble_cfg = cfg->tar_scramble_cfg;
	}
#endif

	release(flash);

	return 0;
}

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_mspi_mx66uw_read_jedec_id(const struct device *flash, uint8_t *id)
{
	struct flash_mspi_mx66uw_data *data = flash->data;

	memcpy(id, data->id, 3);
	return 0;
}
#endif /* CONFIG_FLASH_JESD216_API */

static DEVICE_API(flash, flash_mspi_mx66uw_api) = {
	.erase          = flash_mspi_mx66uw_erase,
	.write          = flash_mspi_mx66uw_write,
	.read           = flash_mspi_mx66uw_read,
	.get_parameters = flash_mspi_mx66uw_get_parameters,
	.get_size       = flash_mspi_mx66uw_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout    = flash_mspi_mx66uw_pages_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.read_jedec_id  = flash_mspi_mx66uw_read_jedec_id,
#endif
};

#if defined(CONFIG_SOC_SERIES_E1C) || defined(CONFIG_SOC_SERIES_B1)
#define MX66UW_SERIAL_FREQUENCY 10000000U
#else
#define MX66UW_SERIAL_FREQUENCY 12000000U
#endif

#define MSPI_DEVICE_CONFIG_SERIAL(n)                                                              \
	{                                                                                         \
		.ce_num             = DT_INST_PROP(n, mspi_hardware_ce_num),                      \
		.freq               = MX66UW_SERIAL_FREQUENCY,                                 \
		.io_mode            = MSPI_IO_MODE_SINGLE,                                        \
		.data_rate          = MSPI_DATA_RATE_SINGLE,                                      \
		.cpp                = MSPI_CPP_MODE_0,                                            \
		.endian             = DT_INST_ENUM_IDX_OR(n, mspi_endian,                         \
							  MSPI_XFER_LITTLE_ENDIAN),               \
		.ce_polarity        = MSPI_CE_ACTIVE_LOW,                                         \
		.dqs_enable         = false,                                                      \
		.rx_dummy           = 8,                                                          \
		.tx_dummy           = 0,                                                          \
		.read_cmd           = SPI_NOR_CMD_READ_FAST,                                      \
		.write_cmd          = SPI_NOR_CMD_PP,                                             \
		.cmd_length         = 1,                                                          \
		.addr_length        = 3,                                                          \
		.mem_boundary       = 0,                                                          \
		.time_to_break      = 0,                                                          \
	}

#define MSPI_TIMING_CONFIG(n) {}

#define MSPI_TIMING_CONFIG_MASK(n) MSPI_TIMING_PARAM_DUMMY

#define FLASH_MSPI_MX66UW(n)                                                                   \
	static const struct flash_mspi_mx66uw_config flash_mspi_mx66uw_config_##n = {       \
		.mem_size    = DT_INST_PROP(n, size) / 8,                                         \
		.flash_param =                                                                    \
			{                                                                         \
				.write_block_size = DT_INST_PROP_OR(n, write_block_size,          \
								    MX66UW_WRITE_SIZE),           \
				.erase_value      = NOR_ERASE_VALUE,                              \
			},                                                                        \
		.page_layout =                                                                    \
			{                                                                         \
				.pages_count = DT_INST_PROP(n, size) / 8 / MX66UW_SECTOR_SIZE,     \
				.pages_size  = MX66UW_SECTOR_SIZE,                                 \
			},                                                                        \
		.bus                = DEVICE_DT_GET(DT_INST_BUS(n)),                              \
		.dev_id             = MSPI_DEVICE_ID_DT_INST(n),                                  \
		.serial_cfg         = MSPI_DEVICE_CONFIG_SERIAL(n),                               \
		.tar_dev_cfg        = MSPI_DEVICE_CONFIG_DT_INST(n),                              \
		MSPI_OPTIONAL_CFG_STRUCT_INIT(CONFIG_MSPI_XIP,                                    \
					      tar_xip_cfg, MSPI_XIP_CONFIG_DT_INST(n))            \
		MSPI_XIP_BASE_ADDR_INIT(xip_base_addr, DT_INST_BUS(n))                            \
		MSPI_OPTIONAL_CFG_STRUCT_INIT(CONFIG_MSPI_SCRAMBLE,                               \
					      tar_scramble_cfg, MSPI_SCRAMBLE_CONFIG_DT_INST(n))  \
		MSPI_OPTIONAL_CFG_STRUCT_INIT(CONFIG_MSPI_TIMING,                                 \
					      tar_timing_cfg, MSPI_TIMING_CONFIG(n))              \
		MSPI_OPTIONAL_CFG_STRUCT_INIT(CONFIG_MSPI_TIMING,                                 \
					      timing_cfg_mask, MSPI_TIMING_CONFIG_MASK(n))        \
		.sw_multi_periph    = DT_PROP(DT_INST_BUS(n), software_multiperipheral),          \
		.reset_gpio         = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),              \
		.reset_pulse_us     = DT_INST_PROP_OR(n, t_reset_pulse, 0),                       \
		.reset_recovery_us  = DT_INST_PROP_OR(n, t_reset_recovery, 0),                    \
	};                                                                                        \
	static struct flash_mspi_mx66uw_data flash_mspi_mx66uw_data_##n = {                 \
		.lock = Z_SEM_INITIALIZER(flash_mspi_mx66uw_data_##n.lock, 0, 1),              \
	};                                                                                        \
	DEVICE_DT_INST_DEFINE(n,                                                                  \
			      flash_mspi_mx66uw_init,                                          \
			      NULL,                                                               \
			      &flash_mspi_mx66uw_data_##n,                                     \
			      &flash_mspi_mx66uw_config_##n,                                   \
			      POST_KERNEL,                                                        \
			      CONFIG_FLASH_INIT_PRIORITY,                                         \
			      &flash_mspi_mx66uw_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_MSPI_MX66UW)
