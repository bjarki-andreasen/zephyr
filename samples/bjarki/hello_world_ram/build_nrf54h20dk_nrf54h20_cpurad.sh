#!/bin/bash
set -e
west build -p -b nrf54h20dk/nrf54h20/cpurad hello_world_ram/
python3 bin2hex.py build/zephyr/zephyr.signed.bin 0xe064000 hello_world_ram/nrf54h20dk_nrf54h20_cpurad.signed.hex
west build -p -b nrf54h20dk/nrf54h20/cpurad bootloader/mcuboot/boot/zephyr/ -DDTC_OVERLAY_FILE=$PWD/hello_world_ram/mcuboot_nrf54h20dk_nrf54h20_cpurad.overlay -DEXTRA_CONF_FILE=$PWD/hello_world_ram/mcuboot_nrf54h20dk_nrf54h20_cpurad.conf
mv build/zephyr/uicr_merged.hex build/zephyr/zephyr_unmerged.hex
./zephyr/scripts/build/mergehex.py build/zephyr/zephyr_unmerged.hex hello_world_ram/nrf54h20dk_nrf54h20_cpurad.signed.hex -o build/zephyr/uicr_merged.hex
west flash
