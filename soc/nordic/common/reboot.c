/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#if defined(CONFIG_HAS_NORDIC_RAM_CTRL)
#include <helpers/nrfx_ram_ctrl.h>
#endif

#include <soc_tee.h>

#if CONFIG_SOC_TEE_REBOOT_NATIVE
static void soc_tee_reboot_native(void)
{
#ifdef CONFIG_NRF_FORCE_RAM_ON_REBOOT
	uint8_t *ram_start;
	size_t ram_size;

#if defined(NRF_MEMORY_RAM_BASE)
	ram_start = (uint8_t *)NRF_MEMORY_RAM_BASE;
#else
	ram_start = (uint8_t *)NRF_MEMORY_RAM0_BASE;
#endif

	ram_size = 0;
#if defined(NRF_MEMORY_RAM_SIZE)
	ram_size += NRF_MEMORY_RAM_SIZE;
#endif
#if defined(NRF_MEMORY_RAM0_SIZE)
	ram_size += NRF_MEMORY_RAM0_SIZE;
#endif
#if defined(NRF_MEMORY_RAM1_SIZE)
	ram_size += NRF_MEMORY_RAM1_SIZE;
#endif
#if defined(NRF_MEMORY_RAM2_SIZE)
	ram_size += NRF_MEMORY_RAM2_SIZE;
#endif
#if defined(NRF_MEMORY_RAM3_SIZE)
	ram_size += NRF_MEMORY_RAM3_SIZE;
#endif

	/* Power on all RAM blocks */
	nrfx_ram_ctrl_power_enable_set(ram_start, ram_size, true);
#endif

	NVIC_SystemReset();
}
#endif

void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

#if CONFIG_SOC_TEE_REBOOT_NATIVE
	soc_tee_reboot_native();
#elif CONFIG_SOC_TEE_REBOOT_CUSTOM
	soc_tee_reboot_custom();
#endif
}
