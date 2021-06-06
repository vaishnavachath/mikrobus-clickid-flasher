/*
 * Copyright (c) 2021 Vaishnav M A. BeagleBoard.org Foundation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <zephyr.h>

#define DT_DRV_COMPAT zephyr_mikrobusid

#include <device.h>
#include <devicetree.h>
#include <errno.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <stdlib.h>
#include <sys/byteorder.h>
#include <stddef.h>

#include "w1-gpio.h"

static unsigned char mikrobus_manifest_click_builtin[] = {
#include "mikrobus_click_manifest.inc"
};

LOG_MODULE_REGISTER(mikrobus, CONFIG_MIKROBUS_LOG_LEVEL);

#define W1_SKIP_ROM_CMD 0xCC
#define MIKROBUS_ID_EEPROM_WRITE_SCRATCHPAD_CMD 0x0F
#define MIKROBUS_ID_EEPROM_READ_SCRATCHPAD_CMD 0xAA
#define MIKROBUS_ID_EEPROM_COPY_SCRATCHPAD_CMD 0x55
#define MIKROBUS_ID_EEPROM_READ_MEMORY_CMD 0xF0

#define US_TO_SYS_CLOCK_HW_CYCLES(us) \
	((uint64_t)sys_clock_hw_cycles_per_sec() * (us) / USEC_PER_SEC + 1)

#define MIKROBUS_ENTER_ID_MODE_DELAY US_TO_SYS_CLOCK_HW_CYCLES(1000)

struct mikrobusid_config
{
	unsigned int id;
	const char *cs_gpio_name;
	const char *rst_gpio_name;
	gpio_pin_t cs_pin;
	gpio_pin_t rst_pin;
	gpio_dt_flags_t cs_flags;
	gpio_dt_flags_t rst_flags;
};

struct mikrobusid_context
{
	const struct device *cs_gpio;
	const struct device *rst_gpio;
	gpio_pin_t cs_pin;
	gpio_pin_t rst_pin;
};

static void inline mikrobusid_delay(unsigned int cycles_to_wait)
{
	uint32_t start = k_cycle_get_32();

	/* Wait until the given number of cycles have passed */
	while (k_cycle_get_32() - start < cycles_to_wait)
	{
	}
}

static int mikrobusid_enter_id_mode(struct mikrobusid_context *context)
{

	if (!context)
		return -EINVAL;
	/* set RST LOW */
	gpio_pin_set(context->rst_gpio, context->rst_pin, 0);
	return 0;
}

// static int mikrobusid_exit_id_mode(struct mikrobusid_context *context) {
// 	int i;

// 	if(!context)
// 		return -EINVAL;
// 	/* set RST LOW */
// 	gpio_pin_set(context->rst_gpio, context->rst_pin, 1);
// 	return 0;
// }

/*
 * performs full page write sequence -> write scratchpad, verify, copy to memory
 * only supports full scratchpad size for now(32 bytes)
 */
static int mikrobus_write_memory(uint8_t *data, uint16_t target_address, struct w1_io_context *w1_io_context, struct w1_io *w1_gpio_io)
{

	uint8_t TA1, TA2, ES, readbyte[32];
	int found, iter = 0;

	found = w1_gpio_io->reset_bus(w1_io_context);
	if (found)
	{
		LOG_ERR("w1 device not found");
		return -ENODEV;
	}
	w1_gpio_io->write_8(w1_io_context, W1_SKIP_ROM_CMD);
	w1_gpio_io->write_8(w1_io_context, MIKROBUS_ID_EEPROM_WRITE_SCRATCHPAD_CMD);
	w1_gpio_io->write_8(w1_io_context, target_address & 0xFF);
	w1_gpio_io->write_8(w1_io_context, (target_address >> 8) & 0xFF);

	for (iter = 0; iter < 32; iter++)
	{
		w1_gpio_io->write_8(w1_io_context, data[iter]);
	}

	found = w1_gpio_io->reset_bus(w1_io_context);
	if (found)
	{
		LOG_ERR("w1 device not found");
		return -ENODEV;
	}
	w1_gpio_io->write_8(w1_io_context, W1_SKIP_ROM_CMD);
	w1_gpio_io->write_8(w1_io_context, MIKROBUS_ID_EEPROM_READ_SCRATCHPAD_CMD);

	TA1 = w1_gpio_io->read_8(w1_io_context);
	TA2 = w1_gpio_io->read_8(w1_io_context);
	ES = w1_gpio_io->read_8(w1_io_context);

	for (iter = 0; iter < 32; iter++)
	{
		readbyte[iter] = w1_gpio_io->read_8(w1_io_context);
	}
	k_sleep(K_MSEC(10));

	found = w1_gpio_io->reset_bus(w1_io_context);
	if (found)
	{
		LOG_ERR("w1 device not found");
		return -ENODEV;
	}

	w1_gpio_io->write_8(w1_io_context, W1_SKIP_ROM_CMD);
	w1_gpio_io->write_8(w1_io_context, MIKROBUS_ID_EEPROM_COPY_SCRATCHPAD_CMD);
	w1_gpio_io->write_8(w1_io_context, target_address & 0xFF);
	w1_gpio_io->write_8(w1_io_context, (target_address >> 8) & 0xFF);
	w1_gpio_io->write_8(w1_io_context, ES);
	k_sleep(K_MSEC(10));
	return 0;
}

/*
 * performs full block write sequence -> multiple times write scratchpad, verify, copy to memory
 * only supports multiples of scratchpad size, if input data is smaller, will be padded while writing
 */
static int mikrobus_write_block(uint8_t *data, unsigned int count, struct w1_io_context *w1_io_context, struct w1_io *w1_gpio_io)
{
	uint16_t wraddr = 0;
	uint16_t len = count - (count % 32);
	uint8_t scratchpad_write[32];

	while (len > 0)
	{
		mikrobus_write_memory(data + wraddr, wraddr, w1_io_context, w1_gpio_io);
		wraddr += 32;
		len -= 32;
	}

	if (count % 32)
	{
		memcpy(scratchpad_write, data + wraddr, count % 32);
		mikrobus_write_memory(scratchpad_write, wraddr, w1_io_context, w1_gpio_io);
	}
	return 0;
}

/*
 * performs read block data from memory
 */
static int mikrobus_read_block(uint8_t *rdata, unsigned int count, uint16_t address, struct w1_io_context *w1_io_context, struct w1_io *w1_gpio_io)
{
	int iter, found;

	found = w1_gpio_io->reset_bus(w1_io_context);
	if (found)
	{
		LOG_ERR("w1 device not found");
		return -ENODEV;
	}
	w1_gpio_io->write_8(w1_io_context, W1_SKIP_ROM_CMD);
	w1_gpio_io->write_8(w1_io_context, MIKROBUS_ID_EEPROM_READ_MEMORY_CMD);
	w1_gpio_io->write_8(w1_io_context, address & 0xFF);
	w1_gpio_io->write_8(w1_io_context, (address >> 8) & 0xFF);
	for (iter = 0; iter < count; iter++)
	{
		rdata[iter] = w1_gpio_io->read_8(w1_io_context);
	}
	return 0;
}

static int mikrobusid_init(const struct device *dev)
{

	struct mikrobusid_context *context = dev->data;
	const struct mikrobusid_config *const config =
		(const struct mikrobusid_config *)dev->config;
	struct w1_gpio_master *w1_master;
	struct w1_io_context *w1_io_context;
	struct w1_io *w1_gpio_io;
	int err;
	uint8_t readdata[512];

	context->cs_gpio = device_get_binding(config->cs_gpio_name);
	if (!context->cs_gpio)
	{
		LOG_ERR("failed to get CS GPIO device");
		return -EINVAL;
	}
	err = gpio_config(context->cs_gpio, config->cs_pin,
					  config->cs_flags | GPIO_INPUT);
	if (err)
	{
		LOG_ERR("failed to configure CS GPIO pin (err %d)", err);
		return err;
	}
	context->rst_gpio = device_get_binding(config->rst_gpio_name);
	if (!context->rst_gpio)
	{
		LOG_ERR("failed to get RST GPIO device");
		return -EINVAL;
	}
	err = gpio_config(context->rst_gpio, config->rst_pin,
					  config->rst_flags | GPIO_OUTPUT_LOW);
	if (err)
	{
		LOG_ERR("failed to configure RST GPIO pin (err %d)", err);
		return err;
	}
	context->cs_pin = config->cs_pin;
	context->rst_pin = config->rst_pin;

	err = mikrobusid_enter_id_mode(context);
	if (err)
	{
		LOG_ERR("failed to enter mikrobus ID mode (err %d)", err);
		return err;
	}

	w1_master = malloc(sizeof(struct w1_gpio_master));
	if (!w1_master)
		return -ENOMEM;
	w1_master->io_context = malloc(sizeof(struct w1_io_context));
	if (!w1_master->io_context)
		return -ENOMEM;
	w1_io_context = w1_master->io_context;
	w1_io_context->w1_gpio = context->cs_gpio;
	w1_io_context->w1_pin = context->cs_pin;
	w1_gpio_init(w1_master);
	w1_gpio_io = w1_master->w1_gpio_io;
	k_sleep(K_MSEC(100));

	mikrobus_write_block(mikrobus_manifest_click_builtin, sizeof(mikrobus_manifest_click_builtin), w1_io_context, w1_gpio_io);

	mikrobus_read_block(readdata, 512, 0x0000, w1_io_context, w1_gpio_io);

	LOG_HEXDUMP_INF(readdata, sizeof(readdata), "manifest data:");

	return 0;
}

#define DEFINE_MIKROBUS(_num)                                     \
                                                                  \
	static struct mikrobusid_context mikrobusid_dev_data_##_num;  \
                                                                  \
	static const struct mikrobusid_config                         \
		mikrobusid_config_##_num = {                              \
			.id = DT_INST_PROP(_num, id),                         \
			.cs_gpio_name = DT_INST_GPIO_LABEL(_num, cs_gpios),   \
			.rst_gpio_name = DT_INST_GPIO_LABEL(_num, rst_gpios), \
			.cs_pin = DT_INST_GPIO_PIN(_num, cs_gpios),           \
			.rst_pin = DT_INST_GPIO_PIN(_num, rst_gpios),         \
			.cs_flags = DT_INST_GPIO_FLAGS(_num, cs_gpios),       \
			.rst_flags = DT_INST_GPIO_FLAGS(_num, rst_gpios),     \
	};                                                            \
                                                                  \
	DEVICE_DT_INST_DEFINE(_num,                                   \
						  mikrobusid_init,                        \
						  NULL,                                   \
						  &mikrobusid_dev_data_##_num,            \
						  &mikrobusid_config_##_num,              \
						  POST_KERNEL,                            \
						  52, NULL);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_MIKROBUS);
