#!/bin/bash
set -e
west build -p -b nrf52840dk/nrf52840 hello_world_ram/
python3 bin2hex.py build/zephyr/zephyr.signed.bin 0xc000 hello_world_ram/nrf52840dk_nrf52840.signed.hex
west build -p -b nrf52840dk/nrf52840 bootloader/mcuboot/boot/zephyr/ -DEXTRA_CONF_FILE=$PWD/hello_world_ram/mcuboot_nrf52840dk_nrf52840.conf
mv build/zephyr/zephyr.hex build/zephyr/zephyr_unmerged.hex
./zephyr/scripts/build/mergehex.py build/zephyr/zephyr_unmerged.hex hello_world_ram/nrf52840dk_nrf52840.signed.hex -o build/zephyr/zephyr.hex
west flash
