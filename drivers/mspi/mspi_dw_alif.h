/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Alif-specific helpers for the DesignWare MSPI/OSPI driver.
 *
 * This header is included only by mspi_dw.c after the DW register accessors
 * and driver private data structures are defined.
 */

#ifndef _MSPI_DW_ALIF_SPECIFIC_H_
#define _MSPI_DW_ALIF_SPECIFIC_H_

struct alif_ospi_aes_regs {
	uint32_t aes_ctrl;
	uint32_t aes_intr;
	uint32_t aes_intr_mask;
	uint32_t aes_clk_dis;
	uint32_t aes_addr_control;
	uint32_t aes_res_0;
	uint32_t aes_res_1;
	uint32_t aes_res_2;
	uint32_t aes_rxds_dly;
};

struct alif_mspi_vendor_data {
	volatile struct alif_ospi_aes_regs *aes_regs;
	uint8_t ddr_drive_edge;
	uint8_t rx_ds_delay;
	uint8_t baud2_delay;
	uint8_t hyperbus_dfs;
	bool xip_rxds_vl_en;
	bool x16_address_shim;
	bool hyperbus_mode;
};

#define ALIF_SPECIFIC_DATA_DEFINE(inst)						\
	static const struct alif_mspi_vendor_data mspi_dw_alif_##inst##_vendor_data = {	\
		.aes_regs = (void *)DT_INST_REG_ADDR_BY_NAME(inst, aes),		\
		.ddr_drive_edge = DT_INST_PROP_OR(inst, ddr_drive_edge, 0),		\
		.rx_ds_delay = DT_INST_PROP_OR(inst, rx_ds_delay, 0),			\
		.baud2_delay = DT_INST_PROP_OR(inst, baud2_delay, 0),			\
		.hyperbus_dfs = DT_INST_PROP_OR(inst, hyperbus_dfs, 16),			\
		.xip_rxds_vl_en = DT_INST_PROP(inst, xip_rxds_vl_en),			\
		.x16_address_shim = DT_INST_PROP(inst, x16_address_shim),		\
		.hyperbus_mode = DT_INST_PROP(inst, hyperbus_mode),			\
	}

#define ALIF_SPECIFIC_DATA_GET(inst) ((void *)&mspi_dw_alif_##inst##_vendor_data)

static inline const struct alif_mspi_vendor_data *
alif_vendor_data_get(const struct device *dev)
{
	const struct mspi_dw_config *config = dev->config;

	return config->vendor_specific_data;
}

#define ALIF_AES_CTRL_XIP_EN		BIT(4U)
#define ALIF_AUX_BAUD2_DELAY_MASK	BIT(30U)
#define ALIF_XIP_CTRL_RXDS_VL_EN_BIT	BIT(30U)

#define ALIF_AES_ADDR_ARRAY_MASK_POS       0U
#define ALIF_AES_ADDR_SS0_ARRAY_MODE_POS   18U
#define ALIF_AES_ADDR_SS1_ARRAY_MODE_POS   19U
#define ALIF_AES_ADDR_ARRAY_SHIFT_POS      22U
#define ALIF_AES_ADDR_ARRAY_SPLIT_POS      28U

static inline int alif_configure_x16_address_shim(const struct device *dev,
						   const struct mspi_dev_cfg *cfg)
{
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);
	uint32_t value;

	if (data == NULL || data->aes_regs == NULL || !data->x16_address_shim ||
	    cfg->io_mode != MSPI_IO_MODE_HEX_8_8_16) {
		return 0;
	}

	if (cfg->ce_num > 1U) {
		LOG_ERR("x16 address shim supports only CE0 and CE1");
		return -EINVAL;
	}

	/*
	 * SPI address signalling              Corresponding HADDR/AxADDR bits
	 *
	 *  --  RA12 RA4  CA7                       --  23 15  8
	 *  --  RA11 RA3  CA6                       --  22 14  7
	 *  --  RA10 RA2  CA5                       --  21 13  6
	 *  --  RA9  RA1  CA4                       --  20 12  5
	 *  --  RA8  RA0  CA3                       --  19 11  4
	 *  --  RA7  --   CA2                       --  18 --  3
	 * RA14 RA6  CA9  CA1                       25  17 10  2
	 * RA13 RA5  CA8  CA0 (=0)                  24  16  9  1 (=0)
	 *
	 * Insert a gap between CA9 and RA0 by splitting HADDR at bit 11.
	 * The controller shifts the address down by one in x16 mode because
	 * each device address selects a 16-bit word rather than a byte:
	 *
	 * Logical byte address    Word address    Selected word
	 * 0x000000                0x000000        First
	 * 0x000002                0x000001        Second
	 * 0x000004                0x000002        Third
	 */
	value = (0x7FFU << ALIF_AES_ADDR_ARRAY_MASK_POS) |
		(12U << ALIF_AES_ADDR_ARRAY_SHIFT_POS) |
		(11U << ALIF_AES_ADDR_ARRAY_SPLIT_POS);
	value |= cfg->ce_num == 0U ? BIT(ALIF_AES_ADDR_SS0_ARRAY_MODE_POS) :
		BIT(ALIF_AES_ADDR_SS1_ARRAY_MODE_POS);
	data->aes_regs->aes_addr_control = value;

	return 0;
}

static inline void alif_aes_set_rxds_delay(volatile struct alif_ospi_aes_regs *aes,
					   uint8_t delay)
{
#if defined(CONFIG_ENSEMBLE_GEN2)
	aes->aes_rxds_dly = (uint32_t)delay | ((uint32_t)delay << 8U);
#else
	aes->aes_rxds_dly = delay;
#endif
}

static inline void alif_aes_set_baud2_delay(volatile struct alif_ospi_aes_regs *aes,
					    uint8_t baud2_delay, uint32_t baudr)
{
#if defined(CONFIG_SOC_SERIES_E1C) || defined(CONFIG_SOC_SERIES_B1)
	bool enable;

	switch (baud2_delay) {
	case 1U:
		enable = true;
		break;
	case 2U:
		enable = (baudr == 2U);
		break;
	default:
		enable = false;
		break;
	}

	if (enable) {
		aes->aes_intr_mask |= ALIF_AUX_BAUD2_DELAY_MASK;
	} else {
		aes->aes_intr_mask &= ~ALIF_AUX_BAUD2_DELAY_MASK;
	}
#else
	ARG_UNUSED(aes);
	ARG_UNUSED(baud2_delay);
	ARG_UNUSED(baudr);
#endif
}

static inline void alif_apply_timing_config(const struct device *dev)
{
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);
	const struct mspi_dw_data *dev_data = dev->data;

	if (data == NULL || data->aes_regs == NULL) {
		return;
	}

	alif_aes_set_rxds_delay(data->aes_regs, data->rx_ds_delay);
	alif_aes_set_baud2_delay(data->aes_regs, data->baud2_delay, dev_data->baudr);
}

static inline uint8_t alif_ddr_drive_edge(const struct device *dev)
{
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);

	if (data != NULL) {
		return data->ddr_drive_edge;
	}

	return 0U;
}

static inline int alif_validate_dev_config(const struct device *dev,
					   enum mspi_dev_cfg_mask param_mask,
					   const struct mspi_dev_cfg *cfg)
{
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);
	struct mspi_dw_data *dev_data = dev->data;

	if (data == NULL) {
		return 0;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_IO_MODE) != 0U) {
		int ret = alif_configure_x16_address_shim(dev, cfg);

		if (ret != 0) {
			return ret;
		}
	}

	if (!data->hyperbus_mode) {
		return 0;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_IO_MODE) &&
	    cfg->io_mode != MSPI_IO_MODE_OCTAL &&
	    cfg->io_mode != MSPI_IO_MODE_HEX_8_8_16) {
		LOG_ERR("Alif HyperBus supports only Octal/HEX_8_8_16 I/O mode");
		return -ENOTSUP;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_DATA_RATE) &&
	    cfg->data_rate != MSPI_DATA_RATE_DUAL) {
		LOG_ERR("Alif HyperBus supports only dual data rate");
		return -ENOTSUP;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_DQS) && !cfg->dqs_enable) {
		LOG_ERR("Alif HyperBus requires RWDS");
		return -ENOTSUP;
	}

	/*
	 * On the Alif DesignWare OSPI integration, SPI_CTRLR0 bit 24 selects
	 * HyperBus framing for indirect transfers. It shares the bit position
	 * named SPI_DM_EN by the generic register definitions.
	 */
	dev_data->spi_ctrlr0 |= SPI_CTRLR0_SPI_DM_EN_BIT;

	return 0;
}

#if defined(CONFIG_MSPI_XIP)
static inline void alif_xip_update_ctrl(const struct device *dev, struct xip_ctrl *ctrl,
					const struct mspi_xip_cfg *cfg)
{
	struct mspi_dw_data *dev_data = dev->data;
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);

	if (data->hyperbus_mode) {
		uint32_t dfs = data->hyperbus_dfs - 1U;
		uint32_t trans_type = XIP_CTRL_TRANS_TYPE_TT2;

		if (dev_data->xip_params_active.io_mode == MSPI_IO_MODE_HEX_8_8_16) {
			trans_type = XIP_CTRL_TRANS_TYPE_TT3;
		}

		/* HyperBus generates its command/address phase from the mapped address. */
		ctrl->read = (ctrl->read & XIP_CTRL_WAIT_CYCLES_MASK) |
			     XIP_CTRL_XIP_HYBERBUS_EN_BIT |
			     XIP_CTRL_RXDS_SIG_EN_BIT |
			     XIP_CTRL_DFS_HC_BIT |
			     FIELD_PREP(XIP_CTRL_TRANS_TYPE_MASK, trans_type);
		ctrl->write = (ctrl->write & XIP_WRITE_CTRL_WAIT_CYCLES_MASK) |
			      XIP_WRITE_CTRL_DFS_HC_BIT |
			      XIP_WRITE_CTRL_DM_EN_BIT |
			      XIP_WRITE_CTRL_RXDS_SIG_EN_BIT |
			      XIP_WRITE_CTRL_HYBERBUS_EN_BIT |
			      FIELD_PREP(XIP_WRITE_CTRL_TRANS_TYPE_MASK, trans_type);

		dev_data->ctrlr0 &= ~(CTRLR0_TMOD_MASK |
				      CTRLR0_DFS_MASK |
				      CTRLR0_DFS32_MASK);
		dev_data->ctrlr0 |= FIELD_PREP(CTRLR0_DFS_MASK, dfs) |
				   FIELD_PREP(CTRLR0_DFS32_MASK, dfs) |
				   FIELD_PREP(CTRLR0_TMOD_MASK, CTRLR0_TMOD_RX);
		dev_data->spi_ctrlr0 = SPI_CTRLR0_SPI_DM_EN_BIT;
		return;
	}

	ctrl->read |= XIP_CTRL_DFS_HC_BIT;

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_SPI_DDR_EN_BIT) {
		ctrl->read |= XIP_CTRL_DDR_EN_BIT;
	}

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_INST_DDR_EN_BIT) {
		ctrl->read |= XIP_CTRL_INST_DDR_EN_BIT;
	}

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_SPI_RXDS_EN_BIT) {
		ctrl->read |= XIP_CTRL_RXDS_EN_BIT;
	}

	if (data->xip_rxds_vl_en) {
		ctrl->read |= ALIF_XIP_CTRL_RXDS_VL_EN_BIT;
	}

	/* TODO: XiP write support ? */
	if (cfg->permission == MSPI_XIP_READ_WRITE) {
		ctrl->write |= XIP_WRITE_CTRL_DFS_HC_BIT;

		if (dev_data->spi_ctrlr0 & SPI_CTRLR0_SPI_DDR_EN_BIT) {
			ctrl->write |= XIP_WRITE_CTRL_SPI_DDR_EN_BIT;
		}

		if (dev_data->spi_ctrlr0 & SPI_CTRLR0_INST_DDR_EN_BIT) {
			ctrl->write |= XIP_WRITE_CTRL_INST_DDR_EN_BIT;
		}
	}
}

static inline int alif_xip_enable(const struct device *dev,
				  const struct mspi_dev_id *dev_id,
				  const struct mspi_xip_cfg *cfg)
{
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);

	ARG_UNUSED(dev_id);
	ARG_UNUSED(cfg);

	if (data == NULL || data->aes_regs == NULL) {
		return -ENODEV;
	}

	data->aes_regs->aes_ctrl |= ALIF_AES_CTRL_XIP_EN;
	return 0;
}

static inline int alif_xip_disable(const struct device *dev,
				   const struct mspi_dev_id *dev_id,
				   const struct mspi_xip_cfg *cfg)
{
	struct mspi_dw_data *dev_data = dev->data;
	const struct alif_mspi_vendor_data *data = alif_vendor_data_get(dev);

	ARG_UNUSED(cfg);

	if (data == NULL || data->aes_regs == NULL) {
		return -ENODEV;
	}

	if ((dev_data->xip_enabled & ~BIT(dev_id->dev_idx)) == 0U) {
		data->aes_regs->aes_ctrl &= ~ALIF_AES_CTRL_XIP_EN;
	}

	return 0;
}
#endif /* CONFIG_MSPI_XIP */

#endif /* _MSPI_DW_ALIF_SPECIFIC_H_ */
