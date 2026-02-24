/*
 * Copyright 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>

struct my_flash_area {
	const struct device *dev;
	size_t offset;
	size_t size;
	size_t write_block_size;
	size_t erase_block_size;
	size_t memory_mapped_address;
	bool memory_mapped;
};

#define MY_FLASH_AREA_INIT(node_id)								\
	{											\
		.dev = DEVICE_DT_GET(DT_PARENT(node_id)),					\
		.offset = DT_REG_ADDR(node_id),							\
		.size = DT_REG_SIZE(node_id),							\
		.write_block_size = DT_PROP(node_id, write_block_size),				\
		.erase_block_size = DT_PROP(node_id, erase_block_size),				\
		IF_ENABLED(									\
			DT_NODE_HAS_PROP(node_id, memory),					\
			(.memory_mapped_address = DT_REG_ADDR(DT_PHANDLE(node_id, memory)),)	\
		)										\
		.memory_mapped = DT_NODE_HAS_PROP(node_id, memory),				\
	},

static const struct my_flash_area sample_areas[] = {
	DT_FOREACH_STATUS_OKAY(soc_nv_flash, MY_FLASH_AREA_INIT)
};

int main(void)
{
	ARRAY_FOR_EACH_PTR(sample_areas, area) {
		printk("\nflash area:\n");
		printk("  controller: %s \n", area->dev->name);
		printk("  offset: %zu \n", area->offset);
		printk("  size: %zu \n", area->size);
		printk("  write block size: %zu \n", area->write_block_size);
		printk("  erase block size: %zu \n", area->erase_block_size);
		if (area->memory_mapped) {
			printk("  memory-mapped address: %zu \n", area->memory_mapped_address);
		}
	}

	return 0;
}
