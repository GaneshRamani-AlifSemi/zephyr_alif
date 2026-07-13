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

#ifndef ZEPHYR_DRIVERS_MSPI_MSPI_DW_ALIF_H_
#define ZEPHYR_DRIVERS_MSPI_MSPI_DW_ALIF_H_

struct alif_ospi_aes_regs {
	uint32_t AES_CTRL;
	uint32_t AES_INTR;
	uint32_t AES_INTR_MASK;
	uint32_t AES_CLK_DIS;
	uint32_t AES_ADDR_CONTROL;
	uint32_t AES_RES_0;
	uint32_t AES_RES_1;
	uint32_t AES_RES_2;
	uint32_t AES_RXDS_DLY;
};

#define ALIF_AES_CTRL_XIP_EN BIT(4U)
#define ALIF_AUX_BAUD2_DELAY_MASK BIT(30U)

#if defined(CONFIG_ENSEMBLE_GEN2)
#define ALIF_AUX_SIGNAL_0_DELAY_POS 0U
#define ALIF_AUX_SIGNAL_1_DELAY_POS 8U
static inline void alif_aes_set_rxds_delay(volatile struct alif_ospi_aes_regs *aes,
					   uint8_t delay)
{
	aes->AES_RXDS_DLY = ((uint32_t)delay << ALIF_AUX_SIGNAL_0_DELAY_POS) |
			    ((uint32_t)delay << ALIF_AUX_SIGNAL_1_DELAY_POS);
}
#else
static inline void alif_aes_set_rxds_delay(volatile struct alif_ospi_aes_regs *aes,
					   uint8_t delay)
{
	aes->AES_RXDS_DLY = delay;
}
#endif

static inline void alif_aes_set_baud2_delay(volatile struct alif_ospi_aes_regs *aes,
					    uint8_t baud2_delay, uint32_t baudr)
{
#if defined(CONFIG_SOC_SERIES_E1C) || defined(CONFIG_SOC_SERIES_B1)
	bool enable = false;

	switch (baud2_delay) {
	case 0U:
		enable = false;
		break;
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
		aes->AES_INTR_MASK |= ALIF_AUX_BAUD2_DELAY_MASK;
	} else {
		aes->AES_INTR_MASK &= ~ALIF_AUX_BAUD2_DELAY_MASK;
	}
#else
	ARG_UNUSED(aes);
	ARG_UNUSED(baud2_delay);
	ARG_UNUSED(baudr);
#endif
}

#if defined(CONFIG_MSPI_XIP)
static inline void alif_xip_update_ctrl(const struct mspi_dw_data *dev_data,
					struct xip_ctrl *ctrl)
{
#if defined(CONFIG_SOC_FAMILY_ENSEMBLE) || defined(CONFIG_SOC_FAMILY_BALLETTO)
	/*
	 * Mirror the controller-side settings already selected through
	 * mspi_dev_config(). On Alif OSPI, the XIP path uses separate XIP
	 * control registers, so DDR and RXDS/DQS need to be reflected here
	 * explicitly.
	 */
	ctrl->read |= XIP_CTRL_DFS_HC_BIT;
	ctrl->write |= XIP_WRITE_CTRL_DFS_HC_BIT;

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_SPI_DDR_EN_BIT) {
		ctrl->read |= XIP_CTRL_DDR_EN_BIT;
		ctrl->write |= XIP_WRITE_CTRL_SPI_DDR_EN_BIT;
	}

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_INST_DDR_EN_BIT) {
		ctrl->read |= XIP_CTRL_INST_DDR_EN_BIT;
		ctrl->write |= XIP_WRITE_CTRL_INST_DDR_EN_BIT;
	}

	if (dev_data->spi_ctrlr0 & SPI_CTRLR0_SPI_RXDS_EN_BIT) {
		ctrl->read |= XIP_CTRL_RXDS_EN_BIT;
	}
#else
	ARG_UNUSED(dev_data);
	ARG_UNUSED(ctrl);
#endif
}

static inline void alif_xip_prepare_registers(const struct device *dev)
{
#if defined(CONFIG_SOC_FAMILY_ENSEMBLE) || defined(CONFIG_SOC_FAMILY_BALLETTO)
	const struct mspi_dw_config *dev_config = dev->config;
	const struct mspi_dw_data *dev_data = dev->data;

	write_rx_sample_dly(dev, (dev_data->spi_ctrlr0 & SPI_CTRLR0_SPI_RXDS_EN_BIT) ?
			     0U : dev_data->rx_sample_dly);

	if (dev_config->aes_regs != NULL) {
		volatile struct alif_ospi_aes_regs *aes = dev_config->aes_regs;

		alif_aes_set_rxds_delay(aes, dev_config->rx_ds_delay);
		alif_aes_set_baud2_delay(aes, dev_config->baud2_delay,
					 dev_data->baudr);
	}

#if defined(CONFIG_MSPI_DW_DDR)
	write_txd_drive_edge(dev, dev_config->ddr_drive_edge);
#endif

	write_xip_mode_bits(dev, 0);
	write_xip_cnt_time_out(dev, 100);
#else
	ARG_UNUSED(dev);
#endif
}
#endif

#if defined(CONFIG_MSPI_XIP) && \
	(defined(CONFIG_SOC_FAMILY_ENSEMBLE) || defined(CONFIG_SOC_FAMILY_BALLETTO))
static inline int alif_xip_select(const struct device *dev,
				  const struct mspi_dev_id *dev_id,
				  bool enable)
{
	uint32_t mask;

	if (dev_id->dev_idx >= 32U) {
		return -EINVAL;
	}

	mask = BIT(dev_id->dev_idx);

	if (enable) {
		write_xip_ser(dev, read_xip_ser(dev) | mask);
	} else {
		write_xip_ser(dev, read_xip_ser(dev) & ~mask);
	}

	return 0;
}

static inline int alif_xip_enable(const struct device *dev,
				  const struct mspi_dev_id *dev_id,
				  const struct mspi_xip_cfg *cfg)
{
	const struct mspi_dw_config *dev_config = dev->config;
	volatile struct alif_ospi_aes_regs *aes = dev_config->aes_regs;
	int ret;

	ARG_UNUSED(cfg);

	if (aes == NULL) {
		return -ENODEV;
	}

	ret = alif_xip_select(dev, dev_id, true);
	if (ret < 0) {
		return ret;
	}

	aes->AES_CTRL |= ALIF_AES_CTRL_XIP_EN;

	return 0;
}

static inline int alif_xip_disable(const struct device *dev,
				   const struct mspi_dev_id *dev_id,
				   const struct mspi_xip_cfg *cfg)
{
	const struct mspi_dw_config *dev_config = dev->config;
	struct mspi_dw_data *dev_data = dev->data;
	volatile struct alif_ospi_aes_regs *aes = dev_config->aes_regs;
	int ret;

	ARG_UNUSED(cfg);

	if (aes == NULL) {
		return -ENODEV;
	}

	ret = alif_xip_select(dev, dev_id, false);
	if (ret < 0) {
		return ret;
	}

	if ((dev_data->xip_enabled & ~BIT(dev_id->dev_idx)) == 0U) {
		aes->AES_CTRL &= ~ALIF_AES_CTRL_XIP_EN;
	}

	return 0;
}
#endif

#endif /* ZEPHYR_DRIVERS_MSPI_MSPI_DW_ALIF_H_ */
