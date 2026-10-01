/*
 * SPDX-FileCopyrightText: Copyright 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <nrfx_clock_lfclk.h>
#include "nrf_lfclk_mux.h"

#define DT_DRV_COMPAT nordic_nrf_lfclk_mux

BUILD_ASSERT(NRF_CLOCK_LFCLK_RC == 0);
BUILD_ASSERT(NRF_CLOCK_LFCLK_XTAL == 1);
BUILD_ASSERT(NRF_CLOCK_LFCLK_SYNTH == 2);

#ifdef NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES
BUILD_ASSERT(NRF_CLOCK_LFCLK_XTAL_LOW_SWING == 3);
BUILD_ASSERT(NRF_CLOCK_LFCLK_XTAL_FULL_SWING == 4);
#endif

#ifdef NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES
#define MAX_MUX NRF_CLOCK_LFCLK_XTAL_FULL_SWING
#else
#define MAX_MUX NRF_CLOCK_LFCLK_SYNTH
#endif

struct driver_data {
	MUX_CLK_SUBSYS_DATA_DEFINE
};

static int driver_configure(const struct clk *clk_hw, const void *data)
{
	const struct nrf_lfclk_mux_data *clk_data = data;
	nrf_clock_lfclk_t lfclksrc = (nrf_clock_lfclk_t)clk_data->mux;

	if (lfclksrc > MAX_MUX) {
		return -EINVAL;
	}

	if (!nrfx_clock_lfclk_init_check()) {
		nrfx_clock_lfclk_init(NULL);
	}

	nrf_clock_lf_src_set(NRF_CLOCK, lfclksrc);

	if (lfclksrc == NRF_CLOCK_LFCLK_RC) {
		nrfx_clock_lfclk_stop();
	} else {
		nrfx_clock_lfclk_start();
	}

	return 0;
}

static int driver_get_parent(const struct clk *clk_hw)
{
	return (int)nrf_clock_lf_src_get(NRF_CLOCK);
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
