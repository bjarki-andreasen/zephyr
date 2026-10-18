/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_NORDIC_COMMON_SOC_GPIO_H_
#define SOC_NORDIC_COMMON_SOC_GPIO_H_

#include <zephyr/types.h>
#include <hal/nrf_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

void soc_gpio_pin_mcu_select(uint32_t pin_number, nrf_gpio_pin_sel_t mcu);

#ifdef __cplusplus
}
#endif

#endif /* SOC_NORDIC_COMMON_SOC_GPIO_H_ */
