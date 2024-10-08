#!/usr/bin/env bash
# Copyright 2023 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Compile all the applications needed by the bsim tests in these subfolders

#set -x #uncomment this line for debugging
set -ue
: "${ZEPHYR_BASE:?ZEPHYR_BASE must be set to point to the zephyr root directory}"

source ${ZEPHYR_BASE}/tests/bsim/compile.source

app=tests/bsim/bluetooth/host/gatt/authorization compile
app=tests/bsim/bluetooth/host/gatt/caching compile
app=tests/bsim/bluetooth/host/gatt/caching conf_overlay=psa_overlay.conf compile
app=tests/bsim/bluetooth/host/gatt/general compile
app=tests/bsim/bluetooth/host/gatt/notify compile
app=tests/bsim/bluetooth/host/gatt/notify_multiple compile
run_in_background ${ZEPHYR_BASE}/tests/bsim/bluetooth/host/gatt/settings/compile.sh
run_in_background ${ZEPHYR_BASE}/tests/bsim/bluetooth/host/gatt/ccc_store/compile.sh
run_in_background ${ZEPHYR_BASE}/tests/bsim/bluetooth/host/gatt/sc_indicate/compile.sh

wait_for_background_jobs
