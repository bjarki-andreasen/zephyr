#include <zephyr/drivers/clock_management.h>
#include <zephyr/drivers/clock_management/clock_helpers.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT nordic_nrf_fixed_factor

struct driver_data {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	int32_t multiplier;
	int32_t divider;
};

static clock_freq_t driver_recalc_rate(const struct clk *clk_hw, clock_freq_t parent_rate)
{
	const struct driver_data *clk_data = clk_hw->hw_data;
	int64_t rate = parent_rate;

	rate *= clk_data->multiplier;
	rate /= clk_data->divider;

	return (clock_freq_t)MIN(rate, INT32_MAX);
}

static const struct clock_management_standard_api driver_api = {
	.recalc_rate = driver_recalc_rate,
};

#define DRIVER_DEFINE(inst)									\
	BUILD_ASSERT(DT_INST_PROP_OR(inst, multiplier, 1) != 0);				\
	BUILD_ASSERT(DT_INST_PROP_OR(inst, divider, 1) != 0);					\
												\
	static const struct driver_data CONCAT(data, inst) = {					\
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PHANDLE(inst, input)))	\
		.multiplier = DT_INST_PROP_OR(inst, multiplier, 1),				\
		.divider = DT_INST_PROP_OR(inst, divider, 1),					\
	};											\
												\
	CLOCK_DT_INST_DEFINE(									\
		inst,										\
		&CONCAT(data, inst),								\
		&driver_api									\
	);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_DEFINE)
