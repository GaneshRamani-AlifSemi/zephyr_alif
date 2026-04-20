/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT issi_wvh64m8eall_bll

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control.h>

#include "ospi_hal.h"
#include "ospi.h"

LOG_MODULE_REGISTER(memc_alif_hyperram, CONFIG_MEMC_LOG_LEVEL);

#define DEVICE_NODE DT_NODELABEL(hyperram0)
#define CONTROLLER_NODE DT_PARENT(DEVICE_NODE)

/* ISSI HyperRAM identification */
#define HYPERRAM_VENDOR_ID 0x3U

/* HyperBus register access CA[47:32] words */
#define HYPERRAM_CMD_READ_REG  0xE0E0U
#define HYPERRAM_CMD_READ_REG_ALT 0xC0C0U

/* HyperRAM register addresses */
#define HYPERRAM_ID0_ADDR      0x00000000U

/* HyperRAM OSPI macros */
#define HYPERRAM_OSPI_RX_SAMPLE_DELAY    0U
#define HYPERRAM_OSPI_RX_FIFO_THRESHOLD   0U
#define HYPERRAM_OSPI_DFS                16U

typedef void (*irq_config_func_t)(const struct device *dev);

struct alif_ospi_hyperram_config {
	struct ospi_regs *regs;
	struct ospi_aes_regs *aes_regs;
	const struct device *clk_dev;
	clock_control_subsys_t clkid;
	uint32_t bus_speed;
	uint32_t cs_pin;
	const struct pinctrl_dev_config *pcfg;
	irq_config_func_t irq_config;
	uint8_t rxds_delay;
};

struct alif_ospi_hyperram_data {
	uint32_t freq;
	HAL_OSPI_Handle_T ospi_handle;
	struct ospi_trans_config trans_conf;
	struct k_event event;
};

static int32_t err_map_alif_hal_to_zephyr(int32_t err)
{
	switch (err) {
	case OSPI_ERR_NONE:
		return 0;
	case OSPI_ERR_INVALID_PARAM:
	case OSPI_ERR_INVALID_HANDLE:
		return -EINVAL;
	case OSPI_ERR_INVALID_STATE:
		return -EPERM;
	case OSPI_ERR_CTRL_BUSY:
		return -EBUSY;
	default:
		return -EIO;
	}
}

static void ospi_hal_event_update(uint32_t event_status, void *user_data)
{
	struct alif_ospi_hyperram_data *dev_data = (struct alif_ospi_hyperram_data *)user_data;

	k_event_post(&dev_data->event, event_status);
}

static int hyperram_read_id_cmd(const struct device *dev, uint16_t *value, uint8_t wait_cycles,
				uint16_t ca_hi_word)
{
	const struct alif_ospi_hyperram_config *config = dev->config;
	struct alif_ospi_hyperram_data *data = dev->data;
	int32_t ret;
	/* HyperRAM register access uses a 48-bit CA phase:
	 * 16-bit command word + 32-bit address/control word.
	 * The HAL emits that as inst_len=16 and addr_len=32.
	 */
	uint32_t ca_buff[2];
	uint16_t data_buff = 0U;
	uint32_t event;

	data->trans_conf.addr_len = OSPI_ADDR_LENGTH_32_BITS;
	data->trans_conf.wait_cycles = wait_cycles;

	ret = alif_hal_ospi_prepare_transfer(data->ospi_handle, &data->trans_conf);
	if (ret != 0) {
		return err_map_alif_hal_to_zephyr(ret);
	}

	k_event_clear(&data->event, OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST);

	ospi_control_ss(config->regs, config->cs_pin, SPI_SS_STATE_ENABLE);

	ca_buff[0] = ca_hi_word;
	ca_buff[1] = HYPERRAM_ID0_ADDR;

	ret = alif_hal_ospi_transfer(data->ospi_handle, ca_buff, &data_buff, 1U);
	if (ret != 0) {
		ret = err_map_alif_hal_to_zephyr(ret);
		goto out;
	}

	event = k_event_wait(&data->event,
			     OSPI_EVENT_TRANSFER_COMPLETE | OSPI_EVENT_DATA_LOST,
			     false, K_FOREVER);
	if (!(event & OSPI_EVENT_TRANSFER_COMPLETE)) {
		ret = -EIO;
	}

out:
	ospi_control_ss(config->regs, config->cs_pin, SPI_SS_STATE_DISABLE);

	/* HyperBus register values are returned in the upper byte. */
	*value = (uint16_t)(data_buff >> 8);
	return ret;
}

static int hyperram_read_id(const struct device *dev, uint16_t *value, uint8_t wait_cycles)
{
	int ret;

	ret = hyperram_read_id_cmd(dev, value, wait_cycles, HYPERRAM_CMD_READ_REG);
	if (ret == 0 && ((*value & 0x0FU) == HYPERRAM_VENDOR_ID)) {
		return 0;
	}

	/* Some HyperRAM parts accept either the C0h or E0h read opcode. */
	return hyperram_read_id_cmd(dev, value, wait_cycles, HYPERRAM_CMD_READ_REG_ALT);
}

static int memc_alif_ospi_hyperram_init(const struct device *dev)
{
	const struct alif_ospi_hyperram_config *config = dev->config;
	struct alif_ospi_hyperram_data *data = dev->data;
	struct ospi_init init_config;
	uint16_t id_reg = 0U;
	int32_t ret;

	if (config->bus_speed == 0U) {
		LOG_ERR("OSPI bus speed can't be zero");
		return -EINVAL;
	}

	k_event_init(&data->event);

	pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	/* IRQ init */
	config->irq_config(dev);

	memset(&init_config, 0, sizeof(init_config));
	memset(&data->trans_conf, 0, sizeof(data->trans_conf));

	if (!device_is_ready(config->clk_dev)) {
		LOG_ERR("clock controller device not ready");
		return -ENODEV;
	}

#if defined(CONFIG_ENSEMBLE_GEN2)
	ret = clock_control_configure(config->clk_dev, config->clkid, NULL);
	if (ret != 0) {
		LOG_ERR("Unable to configure clock: err:%d", ret);
		return ret;
	}

	ret = clock_control_on(config->clk_dev, config->clkid);
	if (ret != 0) {
		LOG_ERR("Unable to turn on clock: err:%d", ret);
		return ret;
	}
#endif

	ret = clock_control_get_rate(config->clk_dev, config->clkid, &data->freq);
	if (ret != 0) {
		LOG_ERR("Unable to get clock rate: err:%d", ret);
		return ret;
	}

	init_config.core_clk = data->freq;
	init_config.bus_speed = config->bus_speed;
	init_config.tx_fifo_threshold = DT_PROP(CONTROLLER_NODE, tx_fifo_threshold);
	init_config.rx_fifo_threshold = HYPERRAM_OSPI_RX_FIFO_THRESHOLD;
	init_config.rx_sample_delay = HYPERRAM_OSPI_RX_SAMPLE_DELAY;
	init_config.ddr_drive_edge = DT_PROP(CONTROLLER_NODE, ddr_drive_edge);
	init_config.cs_pin = config->cs_pin;
	init_config.rx_ds_delay = config->rxds_delay;
	init_config.baud2_delay = DT_PROP(CONTROLLER_NODE, baud2_delay);
	init_config.base_regs = (uint32_t *)config->regs;
	init_config.aes_regs = (uint32_t *)config->aes_regs;
	init_config.event_cb = ospi_hal_event_update;
	init_config.user_data = data;

	ret = alif_hal_ospi_initialize(&data->ospi_handle, &init_config);
	if (ret != 0) {
		LOG_ERR("Error in OSPI initialize");
		return err_map_alif_hal_to_zephyr(ret);
	}

	/* HyperRAM read/write transactions are octal DDR commands with 16-bit frames. */
	data->trans_conf.frame_size = HYPERRAM_OSPI_DFS;
	data->trans_conf.frame_format = OSPI_FRF_OCTAL;
	data->trans_conf.ddr_enable = (OSPI_DDR_ENABLE | OSPI_INST_DDR_ENABLE);
	data->trans_conf.inst_len = OSPI_INST_LENGTH_16_BITS;
	data->trans_conf.rx_ds_enable = 0U;

	ret = alif_hal_ospi_prepare_transfer(data->ospi_handle, &data->trans_conf);
	if (ret != 0) {
		LOG_ERR("Error in OSPI initial configuration");
		return err_map_alif_hal_to_zephyr(ret);
	}

	ret = hyperram_read_id(dev, &id_reg, DT_PROP(CONTROLLER_NODE, xip_wait_cycles));
	if (ret != 0) {
		LOG_ERR("HyperRAM read ID failed");
		return ret;
	}

	LOG_DBG("HyperRAM ID0 (CA48): 0x%04x", id_reg);
	if ((id_reg & 0x0FU) != HYPERRAM_VENDOR_ID) {
		LOG_WRN("Unexpected HyperRAM vendor ID nibble: 0x%02x", id_reg & 0x0FU);
	}

	/*
	 * HyperRAM parts in this family power up in a usable default state.
	 * The controller-side XiP timing is driven from the OSPI node's
	 * xip-wait-cycles property.
	 */
	ospi_hyperbus_xip_init(config->regs, DT_PROP(CONTROLLER_NODE, xip_wait_cycles), false);
	aes_enable_xip(config->aes_regs);

	return 0;
}

static void OSPI_IRQHandler(const struct device *dev)
{
	struct alif_ospi_hyperram_data *data = (struct alif_ospi_hyperram_data *)dev->data;

	alif_hal_ospi_irq_handler(data->ospi_handle);
}

/* PINCTRL Definition Macro for Node */
PINCTRL_DT_DEFINE(CONTROLLER_NODE);

static void ospi_irq_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_IRQN(CONTROLLER_NODE), DT_IRQ(CONTROLLER_NODE, priority),
		    OSPI_IRQHandler, DEVICE_DT_GET(DEVICE_NODE), 0);
	irq_enable(DT_IRQN(CONTROLLER_NODE));
}

static const struct alif_ospi_hyperram_config hyperram_config = {
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(CONTROLLER_NODE),
	.regs = (struct ospi_regs *)DT_REG_ADDR(CONTROLLER_NODE),
	.aes_regs = (struct ospi_aes_regs *)DT_PROP_BY_IDX(CONTROLLER_NODE, aes_reg, 0),
	.bus_speed = DT_PROP(CONTROLLER_NODE, bus_speed),
	.rxds_delay = DT_PROP(CONTROLLER_NODE, rx_ds_delay),
	.irq_config = ospi_irq_config_func,
	.clk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(CONTROLLER_NODE)),
	.clkid = (clock_control_subsys_t)DT_CLOCKS_CELL(CONTROLLER_NODE, clkid)
};

static struct alif_ospi_hyperram_data hyperram_data;

DEVICE_DT_DEFINE(DEVICE_NODE,
		 &memc_alif_ospi_hyperram_init,
		 NULL,
		 &hyperram_data,
		 &hyperram_config,
		 POST_KERNEL,
		 CONFIG_MEMC_INIT_PRIORITY,
		 NULL);
