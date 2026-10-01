/*
 * SPDX-FileCopyrightText: Copyright 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_NRF_HFPLL_MUX_H_
#define ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_NRF_HFPLL_MUX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

struct nrf_hfpll_mux_data {
	uint8_t mux;
};

#define Z_CLOCK_MANAGEMENT_DATA_DEFINE_nordic_nrf_hfpll_mux(node_id, prop, idx)			\
	const struct nrf_hfpll_mux_data CONCAT(clk_data, DT_DEP_ORD(node_id)) = {		\
		.mux = DT_PHA_BY_IDX(node_id, prop, idx, mux),					\
	};

#define Z_CLOCK_MANAGEMENT_DATA_GET_nordic_nrf_hfpll_mux(node_id, prop, idx) \
	&CONCAT(clk_data, DT_DEP_ORD(node_id))

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_CLOCK_MANAGEMENT_NRF_HFPLL_MUX_H_ */
