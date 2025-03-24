nrfutil device x-boot-mode-set --boot-mode safe --traits jlink --serial-number $1
nrfutil device erase --all --traits jlink --log-level trace --core Application --serial-number $1
nrfutil device erase --all --traits jlink --log-level trace --core Network --serial-number $1
nrfutil device x-boot-mode-set --boot-mode normal --traits jlink --serial-number $1
nrfutil device program --firmware ./modules/hal/nordic/zephyr/blobs/suit/bin/suit_manifest_starter.hex --serial-number $1
nrfutil device program --options chip_erase_mode=ERASE_NONE --firmware ./hello_world_ram/bicr.hex --core Application --serial-number $1
