#!/bin/bash
set -e
west build -p -b nrf54h20dk/nrf54h20/cpuapp hello_world_ram/
python3 bin2hex.py build/zephyr/zephyr.signed.bin 0xe0a4000 hello_world_ram/nrf54h20dk_nrf54h20_cpuapp.signed.hex
west build -p -b nrf54h20dk/nrf54h20/cpuapp bootloader/mcuboot/boot/zephyr/ -DDTC_OVERLAY_FILE=$PWD/hello_world_ram/mcuboot_nrf54h20dk_nrf54h20_cpuapp.overlay -DEXTRA_CONF_FILE=$PWD/hello_world_ram/mcuboot_nrf54h20dk_nrf54h20_cpuapp.conf
mv build/zephyr/uicr_merged.hex build/zephyr/zephyr_unmerged.hex
./zephyr/scripts/build/mergehex.py build/zephyr/zephyr_unmerged.hex hello_world_ram/nrf54h20dk_nrf54h20_cpuapp.signed.hex -o build/zephyr/uicr_merged.hex
west flash
