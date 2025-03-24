/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <nrf_sys_event.h>
#include <soc.h>

int main(void)
{
	/* Workaround for HM-24529
	* Make sure HSFLL is on whenever RADIO does DMA
	*/
	NRF_LRCCONF000->CLKCTRL[0].ALWAYSRUN = 1;
	NRF_LRCCONF000->TASKS_REQCLKSRC[0]   = 1;

	while (1) {
		k_msleep(200);
		nrf_clock_control_hfxo_request();
		k_usleep(1300);
		//nrf_sys_event_request_global_constlat();
		k_usleep(100);
		k_busy_wait(300);
		//nrf_sys_event_release_global_constlat();
		nrf_clock_control_hfxo_release();
	}

	return 0;
}
