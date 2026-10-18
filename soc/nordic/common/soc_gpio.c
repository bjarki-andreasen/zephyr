/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc_gpio.h>
#include <soc_tee.h>

#if CONFIG_SOC_TEE_GPIO_PIN_MCU_SELECT_NATIVE
#if NRF_GPIO_HAS_SEL
static void soc_tee_gpio_pin_mcu_select_native(uint32_t pin_number, nrf_gpio_pin_sel_t mcu)
{
	nrf_gpio_pin_control_select(pin_number, mcu);
}
#endif
#endif

void soc_gpio_pin_mcu_select(uint32_t pin_number, nrf_gpio_pin_sel_t mcu)
{
#if CONFIG_SOC_TEE_GPIO_PIN_MCU_SELECT_NATIVE
	soc_tee_gpio_pin_mcu_select_native(pin_number, mcu);
#elif CONFIG_SOC_TEE_GPIO_PIN_MCU_SELECT_CUSTOM
	soc_tee_gpio_pin_mcu_select_custom(pin_number, (uint32_t)mcu);
#endif
}
