/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_NORDIC_COMMON_SOC_HOOKS_H_
#define SOC_NORDIC_COMMON_SOC_HOOKS_H_

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

void soc_tee_reboot_custom(void);

void soc_tee_poweroff_custom(void);

#ifdef __cplusplus
}
#endif

#endif /* SOC_NORDIC_COMMON_MRAM_LATENCY_H_ */
