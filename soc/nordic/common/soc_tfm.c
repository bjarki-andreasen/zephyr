/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <hal/nrf_gpio.h>
#include <soc_tee.h>

#include "tfm_platform_api.h"
#include "tfm_ioctl_api.h"

#if NRF_GPIO_HAS_SEL
void soc_tee_gpio_pin_mcu_select_custom(uint32_t pin_number, uint32_t mcu)
{
	uint32_t result;
	enum tfm_platform_err_t err;

	err = tfm_platform_gpio_pin_mcu_select(pin_number, mcu, &result);
	__ASSERT(err == TFM_PLATFORM_ERR_SUCCESS, "TFM platform error (%d)", err);
	__ASSERT(result == 0, "GPIO service error (%d)", result);
}
#endif
