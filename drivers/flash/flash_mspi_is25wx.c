/*
 * Copyright (C) 2024 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT issi_xspi_flash_controller

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(OSPI_FLASH, CONFIG_FLASH_LOG_LEVEL);

#define OSPI_FLASH_NODE DT_NODELABEL(ospi_flash)
#define OSPI_CTRL_NODE  DT_PARENT(OSPI_FLASH_NODE)

#define ADDR_IS_SEC_ALIGNED(addr, _bits) ((addr) & BIT_MASK(_bits))
#define FLASH_SEC_SIZE_BIT               12

#define MAX_SEM_TIMEOUT   100
#define FLASH_READY_TIMEOUT_MS 5000U
#define OSPI_MAX_RX_COUNT 256
#define OSPI_MAX_TX_COUNT 128

#define OSPI_FLASH_CMD_READ_STATUS_ERR (0x02)

/* SPI Data Flash Commands */
#define CMD_READ_JEDEC_ID   (0x9FU)
#define CMD_WRITE_VOL_CONFIG (0x81U)
#define CMD_READ_DATA        (0x7CU)
#define CMD_READ_STATUS      (0x05U)
#define CMD_WRITE_ENABLE     (0x06U)
#define CMD_PAGE_PROGRAM     (0x84U)
#define CMD_READ_FLAG_STATUS (0x70U)
#define CMD_SECTOR_ERASE     (0x21U)

#define IO_MODE_ADDRESS    0x00000000U
#define WAIT_CYCLE_ADDRESS 0x00000001U

#define OCTAL_DDR           (0xE7U)
#define DEFAULT_WAIT_CYCLES (0x10U)

/* Mirror the old HAL OSPI timing profile:
 * - status polling uses 8 dummy cycles
 * - data reads use 16 dummy cycles once octal DDR is enabled
 */
#define FLASH_STATUS_DUMMY_CYCLES 8U
#define FLASH_DATA_DUMMY_CYCLES   16U

/* Flash Driver Status */
#define FLAG_STATUS_BUSY  0x80U
#define FLAG_STATUS_ERROR 0x30U

struct alif_flash_mspi_config {
	const struct device *bus;
	struct flash_parameters flash_param;
	struct flash_pages_layout flash_layout;
	struct mspi_dev_id dev_id;
	struct mspi_dev_cfg serial_cfg;
	struct mspi_dev_cfg octal_cfg;
};

struct alif_flash_mspi_data {
	struct k_sem sem;
	bool octal_enabled;
	bool flash_power_ready;
};

static int mspi_configure_mode(const struct device *dev, bool octal)
{
	const struct alif_flash_mspi_config *cfg = dev->config;
	const struct mspi_dev_cfg *mode_cfg = octal ? &cfg->octal_cfg : &cfg->serial_cfg;
	int ret;

	do {
		ret = mspi_dev_config(cfg->bus, &cfg->dev_id, MSPI_DEVICE_CONFIG_ALL, mode_cfg);
	} while (ret == -EBUSY);

	return ret;
}

static int mspi_do_xfer(const struct device *dev, bool octal, enum mspi_xfer_direction dir,
			uint8_t command, uint32_t address, uint8_t cmd_len, uint8_t addr_len,
			uint16_t dummy, uint8_t *buffer, size_t length)
{
	const struct alif_flash_mspi_config *cfg = dev->config;
	struct mspi_xfer_packet packet = {
		.dir = dir,
		.cb_mask = MSPI_BUS_NO_CB,
		.cmd = command,
		.address = address,
		.num_bytes = length,
		.data_buf = buffer,
	};
	struct mspi_xfer xfer = {
		.async = false,
		.xfer_mode = MSPI_PIO,
		.tx_dummy = (dir == MSPI_TX) ? dummy : 0,
		.rx_dummy = (dir == MSPI_RX) ? dummy : 0,
		.cmd_length = cmd_len,
		.addr_length = addr_len,
		.hold_ce = false,
		.priority = 0,
		.packets = &packet,
		.num_packet = 1,
		.timeout = MAX_SEM_TIMEOUT,
	};
	int ret;

	ret = mspi_configure_mode(dev, octal);
	if (ret != 0) {
		return ret;
	}

	ret = mspi_transceive(cfg->bus, &cfg->dev_id, &xfer);
	if (ret != 0) {
		LOG_DBG("MSPI transfer failed: %d", ret);
	}

	return ret;
}

static int read_status_reg(const struct device *dev, bool octal, uint8_t command,
			   uint8_t *status)
{
	uint8_t status_buf[2] = { 0U, 0U };
	int ret;

	if (status == NULL) {
		return -EINVAL;
	}

	ret = mspi_do_xfer(dev, octal, MSPI_RX, command, 0,
			   1U, 0, FLASH_STATUS_DUMMY_CYCLES, status_buf,
			   sizeof(status_buf));
	if (ret == 0) {
		/* Match the legacy HAL path: read a packed response and use the
		 * returned word directly rather than scanning for a matching lane.
		 */
		*status = status_buf[0];
		LOG_INF("Status raw: %02x %02x -> %02x for cmd 0x%02x",
			status_buf[0], status_buf[1], *status, command);
	}

	return ret;
}

static int set_write_enable(const struct device *dev)
{
	const struct alif_flash_mspi_data *dev_data = dev->data;
	uint8_t val = 0;
	int ret;

	ret = mspi_do_xfer(dev, dev_data->octal_enabled, MSPI_TX, CMD_WRITE_ENABLE, 0,
			   1U, 0, 0, NULL, 0);
	if (ret != 0) {
		return ret;
	}

	/* Only verify WEL after the flash has fully completed octal bring-up.
	 * Keep this as a best-effort check so we do not reject a valid WREN
	 * sequence just because the status-read path is still settling.
	 */
	if (dev_data->octal_enabled && dev_data->flash_power_ready) {
		ret = read_status_reg(dev, dev_data->octal_enabled, CMD_READ_STATUS, &val);
		if (ret == 0) {
			if ((val & OSPI_FLASH_CMD_READ_STATUS_ERR) == 0) {
				LOG_WRN("WEL bit not set after WREN, status=0x%02x", val);
			}
		} else {
			LOG_WRN("WEL status read failed after WREN: %d", ret);
		}
	}

	return ret;
}

static int flash_wait_ready(const struct device *dev)
{
	const struct alif_flash_mspi_data *dev_data = dev->data;
	uint8_t val = 0;
	int ret;
	int64_t deadline = k_uptime_get() + FLASH_READY_TIMEOUT_MS;

	do {
		ret = read_status_reg(dev, dev_data->octal_enabled, CMD_READ_FLAG_STATUS, &val);
		if (ret != 0) {
			return ret;
		}

		if ((val & FLAG_STATUS_ERROR) != 0U) {
			return -EIO;
		}
		/* Match the legacy HAL OSPI poll semantics for ISSI flag status. */
		if ((val & FLAG_STATUS_BUSY) == 0U) {
			k_msleep(1);
			continue;
		}

		return 0;
	} while (k_uptime_get() < deadline);

	LOG_ERR("Timed out waiting for flash ready, status=0x%02x", val);
	return -ETIMEDOUT;
}

static int flash_write_vol_config(const struct device *dev, bool octal, uint32_t address,
				  const uint8_t *data, size_t length)
{
	/* Match the legacy HAL flow:
	 * - serial mode uses a 24-bit address
	 * - octal mode uses a 32-bit address
	 */
	uint8_t addr_len = octal ? 4U : 3U;

	return mspi_do_xfer(dev, octal, MSPI_TX, CMD_WRITE_VOL_CONFIG, address,
			    1U, addr_len, 0, (uint8_t *)data, length);
}

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_mspi_is25wx_read_jedec_id(const struct device *dev, uint8_t *id)
{
	int ret;

	if (id == NULL) {
		return -EINVAL;
	}

	ret = mspi_do_xfer(dev, false, MSPI_RX, CMD_READ_JEDEC_ID, 0, 1U, 0, 0, id, 3U);
	if (ret != 0) {
		return ret;
	}

	return 0;
}
#endif

static int flash_enter_octal_mode(const struct device *dev)
{
	uint8_t mode_cfg = OCTAL_DDR;
	uint8_t wait_cfg[2] = { DEFAULT_WAIT_CYCLES, DEFAULT_WAIT_CYCLES };
	struct alif_flash_mspi_data *dev_data = dev->data;
	int ret;

	/*
	 * Follow the existing OSPI initialization flow:
	 * 1. Use serial mode to switch the flash into octal DDR mode.
	 * 2. Program the wait cycle configuration in serial mode.
	 * 3. Switch the controller to octal mode and repeat the wait cycle write.
	 */
	ret = set_write_enable(dev);
	if (ret != 0) {
		return ret;
	}

	ret = flash_write_vol_config(dev, false, IO_MODE_ADDRESS, &mode_cfg, 1);
	if (ret != 0) {
		return ret;
	}

	ret = flash_write_vol_config(dev, false, WAIT_CYCLE_ADDRESS, wait_cfg, sizeof(wait_cfg));
	if (ret != 0) {
		return ret;
	}

	dev_data->octal_enabled = true;

	ret = set_write_enable(dev);
	if (ret != 0) {
		return ret;
	}

	ret = flash_write_vol_config(dev, true, WAIT_CYCLE_ADDRESS, wait_cfg, sizeof(wait_cfg));
	if (ret != 0) {
		return ret;
	}

	dev_data->flash_power_ready = true;
	return 0;
}

static int flash_is25wx_mspi_read(const struct device *dev, off_t address, void *buffer,
				  size_t length)
{
	const struct alif_flash_mspi_config *cfg = dev->config;
	const struct flash_parameters *f_param = &cfg->flash_param;
	struct alif_flash_mspi_data *dev_data = dev->data;
	uint8_t *data_ptr = buffer;
	size_t remaining = length;
	int ret;

	if ((buffer == NULL) || (address < 0) ||
	    ((address + length) > (f_param->num_of_sector * f_param->sector_size))) {
		return -EINVAL;
	}

	ret = k_sem_take(&dev_data->sem, K_MSEC(MAX_SEM_TIMEOUT));
	if (ret != 0) {
		return ret;
	}

	/* Flash is expected to be in octal mode after init. */
	if (!dev_data->octal_enabled) {
		ret = flash_enter_octal_mode(dev);
		if (ret != 0) {
			goto out;
		}
	}

	while (remaining > 0) {
		size_t chunk = MIN(remaining, (size_t)OSPI_MAX_RX_COUNT * f_param->write_block_size);

		ret = mspi_do_xfer(dev, true, MSPI_RX, CMD_READ_DATA, (uint32_t)address, 1U, 4,
				   FLASH_DATA_DUMMY_CYCLES,
				   data_ptr, chunk);
		if (ret != 0) {
			break;
		}

		remaining -= chunk;
		address += chunk;
		data_ptr += chunk;
	}

out:
	k_sem_give(&dev_data->sem);
	return ret;
}

static int flash_is25wx_mspi_write(const struct device *dev, off_t address, const void *buffer,
				   size_t length)
{
	const struct alif_flash_mspi_config *cfg = dev->config;
	const struct flash_parameters *f_param = &cfg->flash_param;
	struct alif_flash_mspi_data *dev_data = dev->data;
	const uint8_t *data_ptr = buffer;
	size_t remaining = length;
	size_t block_size = f_param->write_block_size;
	int ret;

	if ((buffer == NULL) || (address < 0) ||
	    ((address + length) > (f_param->num_of_sector * f_param->sector_size))) {
		return -EINVAL;
	}

	if ((length % block_size) || !IS_ALIGNED(buffer, block_size)) {
		return -EINVAL;
	}

	ret = k_sem_take(&dev_data->sem, K_MSEC(MAX_SEM_TIMEOUT));
	if (ret != 0) {
		return ret;
	}

	if (!dev_data->octal_enabled) {
		ret = flash_enter_octal_mode(dev);
		if (ret != 0) {
			goto out;
		}
	}

	while (remaining > 0) {
		size_t max_chunk = OSPI_MAX_TX_COUNT * block_size;
		size_t chunk = MIN(remaining, max_chunk - ((size_t)address % max_chunk));

		ret = set_write_enable(dev);
		if (ret != 0) {
			break;
		}

		ret = mspi_do_xfer(dev, true, MSPI_TX, CMD_PAGE_PROGRAM, (uint32_t)address, 1U, 4,
				   0, (uint8_t *)data_ptr, chunk);
		if (ret != 0) {
			break;
		}

		ret = flash_wait_ready(dev);
		if (ret != 0) {
			break;
		}

		address += chunk;
		data_ptr += chunk;
		remaining -= chunk;
	}

out:
	k_sem_give(&dev_data->sem);
	return ret;
}

static int flash_is25wx_mspi_erase(const struct device *dev, off_t addr, size_t size)
{
	const struct alif_flash_mspi_config *cfg = dev->config;
	const struct flash_parameters *f_param = &cfg->flash_param;
	struct alif_flash_mspi_data *dev_data = dev->data;
	int ret;

	if ((addr < 0) || ((addr + size) > (f_param->num_of_sector * f_param->sector_size))) {
		return -EINVAL;
	}

	if ((size % f_param->sector_size) ||
	    ADDR_IS_SEC_ALIGNED(addr, FLASH_SEC_SIZE_BIT)) {
		return -EINVAL;
	}

	ret = k_sem_take(&dev_data->sem, K_MSEC(MAX_SEM_TIMEOUT));
	if (ret != 0) {
		return ret;
	}

	if (!dev_data->octal_enabled) {
		ret = flash_enter_octal_mode(dev);
		if (ret != 0) {
			goto out;
		}
	}

	while (size > 0) {
		LOG_DBG("Issuing WREN before sector erase at 0x%08x", (uint32_t)addr);
		ret = set_write_enable(dev);
		if (ret != 0) {
			break;
		}

		LOG_DBG("Erasing sector at address 0x%08x", (uint32_t)addr);
		ret = mspi_do_xfer(dev, true, MSPI_TX, CMD_SECTOR_ERASE, (uint32_t)addr, 1U, 4,
				   0, NULL, 0);
		if (ret != 0) {
			LOG_ERR("Sector erase command failed at 0x%08x: %d", (uint32_t)addr, ret);
			break;
		}

		ret = flash_wait_ready(dev);
		if (ret != 0) {
			LOG_ERR("Sector erase wait failed at 0x%08x: %d", (uint32_t)addr, ret);
			break;
		}

		addr += f_param->sector_size;
		size -= f_param->sector_size;
	}

out:
	k_sem_give(&dev_data->sem);
	return ret;
}

static int flash_is25wx_mspi_get_size(const struct device *dev, uint64_t *size)
{
	const struct alif_flash_mspi_config *cfg = dev->config;

	if (size == NULL) {
		return -EINVAL;
	}

	*size = (uint64_t)cfg->flash_param.num_of_sector * cfg->flash_param.sector_size;
	return 0;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void flash_is25wx_mspi_page_layout(const struct device *dev,
					  const struct flash_pages_layout **layout,
					  size_t *layout_size)
{
	const struct alif_flash_mspi_config *cfg = dev->config;

	*layout = &cfg->flash_layout;
	*layout_size = 1;
}
#endif

static const struct flash_parameters *flash_is25wx_mspi_get_parameters(const struct device *dev)
{
	const struct alif_flash_mspi_config *cfg = dev->config;

	return &cfg->flash_param;
}

static int flash_is25wx_mspi_init(const struct device *dev)
{
	const struct alif_flash_mspi_config *cfg = dev->config;
	struct alif_flash_mspi_data *dev_data = dev->data;
	int ret;

	if (!device_is_ready(cfg->bus)) {
		LOG_ERR("MSPI controller is not ready");
		return -ENODEV;
	}

	k_sem_init(&dev_data->sem, 1, 1);
	dev_data->octal_enabled = false;
	dev_data->flash_power_ready = false;

	/* Put the device into octal DDR mode using the serial path first. */
	ret = flash_enter_octal_mode(dev);
	if (ret != 0) {
		LOG_ERR("Failed to initialize flash into octal mode (%d)", ret);
		return ret;
	}

	if (IS_ENABLED(CONFIG_ALIF_OSPI_FLASH_XIP)) {
		LOG_WRN("XIP is not enabled in the MSPI flash path");
	}

	return 0;
}

static const struct flash_driver_api flash_is25wx_mspi_api = {
	.read = flash_is25wx_mspi_read,
	.write = flash_is25wx_mspi_write,
	.erase = flash_is25wx_mspi_erase,
#if defined(CONFIG_FLASH_JESD216_API)
	.read_jedec_id = flash_mspi_is25wx_read_jedec_id,
#endif
	.get_parameters = flash_is25wx_mspi_get_parameters,
	.get_size = flash_is25wx_mspi_get_size,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_is25wx_mspi_page_layout,
#endif
};

static struct alif_flash_mspi_data flash_mspi_is25wx_data = {
	.octal_enabled = false,
	.flash_power_ready = false,
};

static const struct alif_flash_mspi_config flash_mspi_is25wx_config = {
	.bus = DEVICE_DT_GET(DT_PARENT(OSPI_FLASH_NODE)),
	.dev_id = {
		.ce = { 0 },
		.dev_idx = DT_PROP(OSPI_CTRL_NODE, cs_pin),
	},
	.serial_cfg = {
		.ce_num = DT_PROP(OSPI_CTRL_NODE, cs_pin),
		.freq = 12000000,
		.io_mode = MSPI_IO_MODE_SINGLE,
		.data_rate = MSPI_DATA_RATE_SINGLE,
		.cpp = MSPI_CPP_MODE_0,
		.endian = MSPI_XFER_BIG_ENDIAN,
		.ce_polarity = MSPI_CE_ACTIVE_LOW,
		.dqs_enable = false,
		.rx_dummy = 0,
		.tx_dummy = 0,
		.read_cmd = CMD_READ_DATA,
		.write_cmd = CMD_PAGE_PROGRAM,
		.cmd_length = 1,
		.addr_length = 0,
		.mem_boundary = 0,
		.time_to_break = 0,
	},
	.octal_cfg = {
		.ce_num = DT_PROP(OSPI_CTRL_NODE, cs_pin),
		.freq = DT_PROP(OSPI_CTRL_NODE, bus_speed),
		/*
		 * Match the legacy HAL OSPI bring-up most closely:
		 * octal framing with DDR on the SPI path, but SDR instruction.
		 */
		.io_mode = MSPI_IO_MODE_OCTAL,
		.data_rate = MSPI_DATA_RATE_S_D_D,
		.cpp = MSPI_CPP_MODE_0,
		.endian = MSPI_XFER_BIG_ENDIAN,
		.ce_polarity = MSPI_CE_ACTIVE_LOW,
		.dqs_enable = false,
		.rx_dummy = FLASH_DATA_DUMMY_CYCLES,
		.tx_dummy = 0,
		.read_cmd = CMD_READ_DATA,
		.write_cmd = CMD_PAGE_PROGRAM,
		.cmd_length = 1,
		.addr_length = 4,
		.mem_boundary = 0,
		.time_to_break = 0,
	},
	.flash_param = {
		.write_block_size = DT_PROP(OSPI_FLASH_NODE, write_block_size),
		.erase_value = DT_PROP(OSPI_FLASH_NODE, erase_value),
		.num_of_sector = DT_PROP(OSPI_FLASH_NODE, num_of_sector),
		.sector_size = DT_PROP(OSPI_FLASH_NODE, sector_size),
		.page_size = DT_PROP(OSPI_FLASH_NODE, page_size),
	},
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.flash_layout = {
		.pages_size = DT_PROP(OSPI_FLASH_NODE, page_size),
		.pages_count = DT_PROP(OSPI_FLASH_NODE, num_of_sector) *
			       DT_PROP(OSPI_FLASH_NODE, sector_size) /
			       DT_PROP(OSPI_FLASH_NODE, page_size),
	},
#endif
};

DEVICE_DT_DEFINE(OSPI_FLASH_NODE, flash_is25wx_mspi_init, NULL,
		 &flash_mspi_is25wx_data, &flash_mspi_is25wx_config,
		 POST_KERNEL, 71,
		 &flash_is25wx_mspi_api);
