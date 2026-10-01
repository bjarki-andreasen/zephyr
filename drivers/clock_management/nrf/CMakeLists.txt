# SPDX-FileCopyrightText: Copyright 2026 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_CLOCK_MANAGEMENT_NRF_FIXED_FACTOR)
  zephyr_library_sources(nrf_fixed_factor.c)
endif()

if(CONFIG_CLOCK_MANAGEMENT_NRF_LFCLK_MUX)
  zephyr_library_sources(nrf_lfclk_mux.c)
  add_clock_management_header(nrf_lfclk_mux.h)
endif()

if(CONFIG_CLOCK_MANAGEMENT_NRF_HFPLL_MUX)
  zephyr_library_sources(nrf_hfpll_mux.c)
  add_clock_management_header(nrf_hfpll_mux.h)
endif()
