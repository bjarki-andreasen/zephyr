/*
 * SPDX-FileCopyrightText: Copyright 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <nrfx_clock_xo.h>
#include "nrf_hfpll_mux.h"

#define DT_DRV_COMPAT nordic_nrf_hfpll_mux

struct driver_data {
	MUX_CLK_SUBSYS_DATA_DEFINE
	bool xo_started;
};

static int driver_configure(const struct clk *clk_hw, const void *data)
{
	struct driver_data *clk_hw_data = clk_hw->hw_data;
	const struct nrf_hfpll_mux_data *clk_data = data;

	if (clk_data->mux) {
		nrfx_clock_xo_start();
		clk_hw_data->xo_started = true;
	} else {
		nrfx_clock_xo_stop();
		clk_hw_data->xo_started = false;
	}

	return 0;
}

static int driver_get_parent(const struct clk *clk_hw)
{
	struct driver_data *clk_hw_data = clk_hw->hw_data;

	return clk_hw_data->xo_started ? 1 : 0;
}

static const struct clock_management_mux_api driver_api = {
	.shared.configure = driver_configure,
	.get_parent = driver_get_parent,
};

#define GET_MUX_INPUT(node_id, prop, idx) \
	CLOCK_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

#define DRIVER_DEFINE(inst)									\
	static const struct clk *CONCAT(parents, inst)[] = {					\
		DT_INST_FOREACH_PROP_ELEM(							\
			inst,									\
			input_sources,								\
			GET_MUX_INPUT								\
		)										\
	};											\
												\
	static struct driver_data CONCAT(data, inst) = {					\
		MUX_CLK_SUBSYS_DATA_INIT(							\
			CONCAT(parents, inst),							\
			DT_INST_PROP_LEN(inst, input_sources)					\
		)										\
	};											\
												\
	MUX_CLOCK_DT_INST_DEFINE(								\
		inst,										\
		&CONCAT(data, inst),								\
		&driver_api									\
	);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_DEFINE)
