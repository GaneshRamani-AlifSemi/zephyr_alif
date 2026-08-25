/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT apmemory_aps512xxn

#include <zephyr/device.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/mspi/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(memc_mspi_aps512xxn, CONFIG_MEMC_LOG_LEVEL);

#define APS512XXN_LINEAR_READ          0x2020U
#define APS512XXN_LINEAR_WRITE         0xA0A0U
#define APS512XXN_GLOBAL_RESET         0xFFFFU
#define APS512XXN_REGISTER_READ        0x4040U
#define APS512XXN_REGISTER_WRITE       0xC0C0U

#define APS512XXN_MR0                  0x00U
#define APS512XXN_MR2                  0x02U
#define APS512XXN_MR4                  0x04U
#define APS512XXN_MR8                  0x08U

#define APS512XXN_MR2_EXPECTED         0xDEU
#define APS512XXN_MR0_RLC_POS          2U
#define APS512XXN_MR4_WLC_POS          5U
#define APS512XXN_MR8_X16              BIT(6)
#define APS512XXN_MR8_LINEAR_X8        BIT(0)
#define APS512XXN_TRANSFER_TIMEOUT_MS  10U
#define APS512XXN_RESET_DELAY_US       2U
#define APS512XXN_RESET_DATA_BYTES     2U
#define APS512XXN_REGISTER_READ_DUMMY  4U

struct memc_mspi_aps512xxn_config {
	const struct device *bus;
	struct mspi_dev_id dev_id;
	struct mspi_dev_cfg command_cfg;
	struct mspi_dev_cfg target_cfg;
	struct mspi_xip_cfg xip_cfg;
	uintptr_t xip_base;
	uint32_t mem_size;
	uint8_t latency_code;
};

struct memc_mspi_aps512xxn_data {
	void *mem_base;
};

static int aps512xxn_transfer(const struct device *dev, enum mspi_xfer_direction direction,
			      uint16_t command, uint32_t address, uint8_t *buffer,
			      size_t length, uint16_t dummy_cycles)
{
	const struct memc_mspi_aps512xxn_config *cfg = dev->config;
	struct mspi_xfer_packet packet = {
		.dir = direction,
		.cmd = command,
		.address = address,
		.num_bytes = length,
		.data_buf = buffer,
	};
	const struct mspi_xfer xfer = {
		.async = false,
		.xfer_mode = MSPI_PIO,
		.tx_dummy = direction == MSPI_TX ? dummy_cycles : 0U,
		.rx_dummy = direction == MSPI_RX ? dummy_cycles : 0U,
		.cmd_length = 2U,
		.addr_length = 4U,
		.hold_ce = false,
		.packets = &packet,
		.num_packet = 1U,
		.timeout = APS512XXN_TRANSFER_TIMEOUT_MS,
	};

	return mspi_transceive(cfg->bus, &cfg->dev_id, &xfer);
}

static int aps512xxn_write_register(const struct device *dev, uint8_t reg, uint8_t value)
{
	uint16_t data = value;

	return aps512xxn_transfer(dev, MSPI_TX, APS512XXN_REGISTER_WRITE, reg,
				   (uint8_t *)&data, sizeof(data), 0U);
}

static int aps512xxn_read_register(const struct device *dev, uint8_t reg, uint8_t *value)
{
	uint16_t data = 0U;
	int ret;

	ret = aps512xxn_transfer(dev, MSPI_RX, APS512XXN_REGISTER_READ, reg,
				(uint8_t *)&data, sizeof(data), APS512XXN_REGISTER_READ_DUMMY);
	if (ret == 0) {
		*value = (uint8_t)data;
	}

	return ret;
}

static int aps512xxn_reset(const struct device *dev)
{
	uint16_t dummy = 0U;
	int ret;

	/* Complete the four-clock global-reset frame after the command phase. */
	ret = aps512xxn_transfer(dev, MSPI_TX, APS512XXN_GLOBAL_RESET, 0U,
				 (uint8_t *)&dummy, APS512XXN_RESET_DATA_BYTES, 0U);
	if (ret == 0) {
		k_busy_wait(APS512XXN_RESET_DELAY_US);
	}

	return ret;
}

static int aps512xxn_write_latency_registers(const struct device *dev)
{
	const struct memc_mspi_aps512xxn_config *cfg = dev->config;
	static const uint8_t write_latency_code[] = {
		[3] = 0x0U,
		[4] = 0x4U,
		[5] = 0x2U,
		[6] = 0x6U,
		[7] = 0x1U,
	};
	uint8_t mr0 = (cfg->latency_code - 3U) << APS512XXN_MR0_RLC_POS;
	uint8_t mr4 = write_latency_code[cfg->latency_code] << APS512XXN_MR4_WLC_POS;
	int ret;

	ret = aps512xxn_write_register(dev, APS512XXN_MR0, mr0);
	if (ret != 0) {
		LOG_ERR("Failed to configure MR0: %d", ret);
		return ret;
	}

	ret = aps512xxn_write_register(dev, APS512XXN_MR4, mr4);
	if (ret != 0) {
		LOG_ERR("Failed to configure MR4: %d", ret);
	}

	return ret;
}

static int aps512xxn_device_init(const struct device *dev)
{
	const struct memc_mspi_aps512xxn_config *cfg = dev->config;
	uint8_t mr2 = 0U;
	uint8_t mr8;
	int ret;

	ret = aps512xxn_reset(dev);
	if (ret != 0) {
		LOG_ERR("Global reset failed: %d", ret);
		return ret;
	}

	ret = aps512xxn_read_register(dev, APS512XXN_MR2, &mr2);
	if (ret != 0 || mr2 != APS512XXN_MR2_EXPECTED) {
		LOG_ERR("Unexpected MR2 value 0x%02x: %d", mr2, ret);
		return ret != 0 ? ret : -ENODEV;
	}

	ret = aps512xxn_write_latency_registers(dev);
	if (ret != 0) {
		return ret;
	}

	mr8 = cfg->target_cfg.io_mode == MSPI_IO_MODE_HEX_8_8_16 ?
		APS512XXN_MR8_X16 : APS512XXN_MR8_LINEAR_X8;
	ret = aps512xxn_write_register(dev, APS512XXN_MR8, mr8);
	if (ret != 0) {
		LOG_ERR("Failed to configure MR8: %d", ret);
	}

	return ret;
}

static int memc_mspi_aps512xxn_init(const struct device *dev)
{
	const struct memc_mspi_aps512xxn_config *cfg = dev->config;
	struct memc_mspi_aps512xxn_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->bus)) {
		LOG_ERR("MSPI controller is not ready");
		return -ENODEV;
	}

	if ((cfg->target_cfg.io_mode != MSPI_IO_MODE_OCTAL &&
	     cfg->target_cfg.io_mode != MSPI_IO_MODE_HEX_8_8_16) ||
	    cfg->target_cfg.data_rate != MSPI_DATA_RATE_DUAL ||
	    !cfg->target_cfg.dqs_enable) {
		LOG_ERR("APS512XXN requires Octal DDR mode with DQS enabled");
		return -EINVAL;
	}

	if (cfg->target_cfg.rx_dummy != cfg->latency_code - 1U ||
	    cfg->target_cfg.tx_dummy != cfg->latency_code - 1U) {
		LOG_ERR("XIP dummy cycles must be latency-code minus one");
		return -EINVAL;
	}

	if (!cfg->xip_cfg.enable || cfg->xip_cfg.permission != MSPI_XIP_READ_WRITE) {
		LOG_ERR("APS512XXN requires a read/write memory-mapped window");
		return -EINVAL;
	}

	ret = mspi_dev_config(cfg->bus, &cfg->dev_id, MSPI_DEVICE_CONFIG_ALL,
			      &cfg->command_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to configure APS512XXN command mode: %d", ret);
		return ret;
	}

	ret = aps512xxn_device_init(dev);
	if (ret != 0) {
		return ret;
	}

	ret = mspi_dev_config(cfg->bus, &cfg->dev_id, MSPI_DEVICE_CONFIG_ALL,
			      &cfg->target_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to configure APS512XXN target mode: %d", ret);
		return ret;
	}

	ret = mspi_xip_config(cfg->bus, &cfg->dev_id, &cfg->xip_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to enable APS512XXN mapped window: %d", ret);
		return ret;
	}

	data->mem_base = (void *)(cfg->xip_base + cfg->xip_cfg.address_offset);
	LOG_INF("APS512XXN mapped at %p (%u bytes), latency %u, %s mode",
		data->mem_base, cfg->mem_size, cfg->latency_code,
		cfg->target_cfg.io_mode == MSPI_IO_MODE_HEX_8_8_16 ? "x16" : "x8");

	return 0;
}

#define APS512XXN_COMMAND_CONFIG(inst)                                                 \
	{                                                                              \
		.ce_num = DT_INST_PROP(inst, mspi_hardware_ce_num),                     \
		.freq = DT_INST_PROP(inst, mspi_max_frequency),                         \
		.io_mode = MSPI_IO_MODE_OCTAL,                                          \
		.data_rate = MSPI_DATA_RATE_DUAL,                                       \
		.cpp = DT_INST_ENUM_IDX(inst, mspi_cpp_mode),                            \
		.endian = DT_INST_ENUM_IDX(inst, mspi_endian),                           \
		.ce_polarity = DT_INST_ENUM_IDX(inst, mspi_ce_polarity),                 \
		.dqs_enable = true,                                                      \
		.rx_dummy = APS512XXN_REGISTER_READ_DUMMY,                               \
		.tx_dummy = 0U,                                                          \
		.read_cmd = APS512XXN_REGISTER_READ,                                     \
		.write_cmd = APS512XXN_REGISTER_WRITE,                                   \
		.cmd_length = 2U,                                                        \
		.addr_length = 4U,                                                       \
	}

#define MEMC_MSPI_APS512XXN(inst)                                                     \
	BUILD_ASSERT(IS_ENABLED(CONFIG_MSPI_XIP), "APS512XXN requires CONFIG_MSPI_XIP"); \
	static const struct memc_mspi_aps512xxn_config memc_mspi_aps512xxn_cfg_##inst = { \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),                                  \
		.dev_id = MSPI_DEVICE_ID_DT_INST(inst),                                   \
		.command_cfg = APS512XXN_COMMAND_CONFIG(inst),                             \
		.target_cfg = MSPI_DEVICE_CONFIG_DT_INST(inst),                            \
		.xip_cfg = MSPI_XIP_CONFIG_DT_INST(inst),                                 \
		.xip_base = DT_REG_ADDR_BY_IDX(DT_INST_BUS(inst), 1),                      \
		.mem_size = DT_INST_PROP(inst, size),                                      \
		.latency_code = DT_INST_PROP(inst, latency_code),                          \
	};                                                                                 \
	static struct memc_mspi_aps512xxn_data memc_mspi_aps512xxn_data_##inst;            \
	DEVICE_DT_INST_DEFINE(inst, memc_mspi_aps512xxn_init, NULL,                         \
			      &memc_mspi_aps512xxn_data_##inst,                              \
			      &memc_mspi_aps512xxn_cfg_##inst, POST_KERNEL,                  \
			      CONFIG_MEMC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MEMC_MSPI_APS512XXN)
