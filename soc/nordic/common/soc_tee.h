/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_NORDIC_COMMON_SOC_TEE_H_
#define SOC_NORDIC_COMMON_SOC_TEE_H_

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

void soc_tee_reboot_custom(void);

void soc_tee_poweroff_custom(void);

void soc_tee_gpio_pin_mcu_select_custom(uint32_t pin_number, uint32_t mcu);

#ifdef __cplusplus
}
#endif

#endif /* SOC_NORDIC_COMMON_SOC_TEE_H_ */
