/*
 * Copyright (C) 2026 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_MEMC_ALIF_OSPI_SIGNAL_DELAYS_H_
#define ZEPHYR_DRIVERS_MEMC_ALIF_OSPI_SIGNAL_DELAYS_H_

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <ospi_hal.h>

/* The including driver defines OSPI_CTRL_NODE as its parent controller. */
/* Apply a complete delay configuration only when explicitly requested in DT. */
#define OSPI_HAS_SIGNAL_DELAYS (\
	DT_NODE_HAS_PROP(OSPI_CTRL_NODE, txd_delays) || \
	DT_NODE_HAS_PROP(OSPI_CTRL_NODE, rxd_delays) || \
	DT_NODE_HAS_PROP(OSPI_CTRL_NODE, ssi_oe_n_delays) || \
	DT_NODE_HAS_PROP(OSPI_CTRL_NODE, rx_ds_delays) || \
	DT_NODE_HAS_PROP(OSPI_CTRL_NODE, txd_dm_delays) || \
	DT_NODE_HAS_PROP(OSPI_CTRL_NODE, dm_oe_n_delays) || \
	DT_NODE_HAS_PROP(OSPI_CTRL_NODE, ss_n_delays) || \
	DT_NODE_HAS_PROP(OSPI_CTRL_NODE, sclk_delay) || \
	DT_NODE_HAS_PROP(OSPI_CTRL_NODE, sclkn_delay))

#if OSPI_HAS_SIGNAL_DELAYS
BUILD_ASSERT(IS_ENABLED(CONFIG_ENSEMBLE_GEN2),
	     "OSPI per-signal delays require Ensemble Gen2");

#define OSPI_ASSERT_DELAY(node_id, prop, idx) \
	BUILD_ASSERT(DT_PROP_BY_IDX(node_id, prop, idx) <= OSPI_SIGNAL_DELAY_MAX, \
		     "OSPI signal delay must be in range 0..23");

#if DT_NODE_HAS_PROP(OSPI_CTRL_NODE, txd_delays)
BUILD_ASSERT(DT_PROP_LEN(OSPI_CTRL_NODE, txd_delays) == 16,
	     "txd-delays must contain 16 entries");
DT_FOREACH_PROP_ELEM(OSPI_CTRL_NODE, txd_delays, OSPI_ASSERT_DELAY)
#endif

#if DT_NODE_HAS_PROP(OSPI_CTRL_NODE, rxd_delays)
BUILD_ASSERT(DT_PROP_LEN(OSPI_CTRL_NODE, rxd_delays) == 16,
	     "rxd-delays must contain 16 entries");
DT_FOREACH_PROP_ELEM(OSPI_CTRL_NODE, rxd_delays, OSPI_ASSERT_DELAY)
#endif

#if DT_NODE_HAS_PROP(OSPI_CTRL_NODE, ssi_oe_n_delays)
BUILD_ASSERT(DT_PROP_LEN(OSPI_CTRL_NODE, ssi_oe_n_delays) == 16,
	     "ssi-oe-n-delays must contain 16 entries");
DT_FOREACH_PROP_ELEM(OSPI_CTRL_NODE, ssi_oe_n_delays, OSPI_ASSERT_DELAY)
#endif

#if DT_NODE_HAS_PROP(OSPI_CTRL_NODE, rx_ds_delays)
BUILD_ASSERT(DT_PROP_LEN(OSPI_CTRL_NODE, rx_ds_delays) == 2,
	     "rx-ds-delays must contain 2 entries");
DT_FOREACH_PROP_ELEM(OSPI_CTRL_NODE, rx_ds_delays, OSPI_ASSERT_DELAY)
#endif

#if DT_NODE_HAS_PROP(OSPI_CTRL_NODE, txd_dm_delays)
BUILD_ASSERT(DT_PROP_LEN(OSPI_CTRL_NODE, txd_dm_delays) == 2,
	     "txd-dm-delays must contain 2 entries");
DT_FOREACH_PROP_ELEM(OSPI_CTRL_NODE, txd_dm_delays, OSPI_ASSERT_DELAY)
#endif

#if DT_NODE_HAS_PROP(OSPI_CTRL_NODE, dm_oe_n_delays)
BUILD_ASSERT(DT_PROP_LEN(OSPI_CTRL_NODE, dm_oe_n_delays) == 2,
	     "dm-oe-n-delays must contain 2 entries");
DT_FOREACH_PROP_ELEM(OSPI_CTRL_NODE, dm_oe_n_delays, OSPI_ASSERT_DELAY)
#endif

#if DT_NODE_HAS_PROP(OSPI_CTRL_NODE, ss_n_delays)
BUILD_ASSERT(DT_PROP_LEN(OSPI_CTRL_NODE, ss_n_delays) == 2,
	     "ss-n-delays must contain 2 entries");
DT_FOREACH_PROP_ELEM(OSPI_CTRL_NODE, ss_n_delays, OSPI_ASSERT_DELAY)
#endif

#if DT_NODE_HAS_PROP(OSPI_CTRL_NODE, sclk_delay)
BUILD_ASSERT(DT_PROP(OSPI_CTRL_NODE, sclk_delay) <= OSPI_SIGNAL_DELAY_MAX,
	     "sclk-delay must be in range 0..23");
#endif

#if DT_NODE_HAS_PROP(OSPI_CTRL_NODE, sclkn_delay)
BUILD_ASSERT(DT_PROP(OSPI_CTRL_NODE, sclkn_delay) <= OSPI_SIGNAL_DELAY_MAX,
	     "sclkn-delay must be in range 0..23");
#endif

#if !DT_NODE_HAS_PROP(OSPI_CTRL_NODE, rx_ds_delays)
BUILD_ASSERT(DT_PROP(OSPI_CTRL_NODE, rx_ds_delay) <= OSPI_SIGNAL_DELAY_MAX,
	     "rx-ds-delay must be in range 0..23");
#endif

static const struct ospi_signal_delay_config signal_delays = {
	.txd = DT_PROP_OR(OSPI_CTRL_NODE, txd_delays, {0}),
	.rxd = DT_PROP_OR(OSPI_CTRL_NODE, rxd_delays, {0}),
	.ssioen = DT_PROP_OR(OSPI_CTRL_NODE, ssi_oe_n_delays, {0}),
	.rxds = COND_CODE_1(DT_NODE_HAS_PROP(OSPI_CTRL_NODE, rx_ds_delays),
		(DT_PROP(OSPI_CTRL_NODE, rx_ds_delays)),
		({DT_PROP(OSPI_CTRL_NODE, rx_ds_delay), DT_PROP(OSPI_CTRL_NODE, rx_ds_delay)})),
	.txddm = DT_PROP_OR(OSPI_CTRL_NODE, txd_dm_delays, {0}),
	.dmoen = DT_PROP_OR(OSPI_CTRL_NODE, dm_oe_n_delays, {0}),
	.ssn = DT_PROP_OR(OSPI_CTRL_NODE, ss_n_delays, {0}),
	.sclk = DT_PROP_OR(OSPI_CTRL_NODE, sclk_delay, 0),
	.sclkn = DT_PROP_OR(OSPI_CTRL_NODE, sclkn_delay, 0),
};
#endif

#endif /* ZEPHYR_DRIVERS_MEMC_ALIF_OSPI_SIGNAL_DELAYS_H_ */
