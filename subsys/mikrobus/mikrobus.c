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

static unsigned char mikrobus_manifest_click_fixed_builtin[] = {
#include "mikrobus_click_fixed_manifest.inc"
};

LOG_MODULE_REGISTER(mikrobus, CONFIG_MIKROBUS_LOG_LEVEL);

#define W1_SKIP_ROM_CMD 0xCC
#define MIKROBUS_ID_EEPROM_WRITE_SCRATCHPAD_CMD 0x0F
#define MIKROBUS_ID_EEPROM_READ_SCRATCHPAD_CMD 0xAA
#define MIKROBUS_ID_EEPROM_COPY_SCRATCHPAD_CMD 0x55
#define MIKROBUS_ID_EEPROM_READ_MEMORY_CMD 0xF0
#define MIKROBUS_ID_EEPROM_SCRATCHPAD_SIZE 32
#define MIKROBUS_ID_EEPROM_BLOCK_PROT_CONTROL_SIZE 10
#define MIKROBUS_ID_EEPROM_PROTECTION_CTRL_ADDR 0x0A00
#define MIKROBUS_ID_USER_EEPROM_ADDR 0x0A0A
#define MIKROBUS_ID_USER_EEPROM_SIZE 18

#define MIKROBUS_FIXED_MANIFEST_START_ADDR 0x0000

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

static int mikrobusid_exit_id_mode(struct mikrobusid_context *context) 
{
	if(!context)
		return -EINVAL;
	/* set RST HIGH */
	gpio_pin_set(context->rst_gpio, context->rst_pin, 1);
	return 0;
}

/*
 * performs full page write sequence -> write scratchpad, verify, copy to memory
 * only supports full scratchpad size for now(32 bytes)
 */
static int mikrobus_write_memory(uint8_t *data, uint16_t target_address, struct w1_io_context *w1_io_context, struct w1_io *w1_gpio_io)
{

	uint8_t TA1, TA2, ES, readbyte[MIKROBUS_ID_EEPROM_SCRATCHPAD_SIZE];
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

	for (iter = 0; iter < MIKROBUS_ID_EEPROM_SCRATCHPAD_SIZE; iter++)
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

	for (iter = 0; iter < MIKROBUS_ID_EEPROM_SCRATCHPAD_SIZE; iter++)
	{
		readbyte[iter] = w1_gpio_io->read_8(w1_io_context);
	}

	if (memcmp(readbyte, data, MIKROBUS_ID_EEPROM_SCRATCHPAD_SIZE)) {
		LOG_ERR("mikrobus scratchpad verify mismatch");
		return -EINVAL;
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
static int mikrobus_write_block(uint8_t *data, unsigned int count, uint16_t writeaddr, struct w1_io_context *w1_io_context, struct w1_io *w1_gpio_io)
{
	uint16_t wraddr = 0;
	uint16_t len = count - (count % MIKROBUS_ID_EEPROM_SCRATCHPAD_SIZE);
	uint8_t scratchpad_write[MIKROBUS_ID_EEPROM_SCRATCHPAD_SIZE];

	while (len > 0)
	{
		mikrobus_write_memory(data + wraddr, wraddr + writeaddr, w1_io_context, w1_gpio_io);
		wraddr += MIKROBUS_ID_EEPROM_SCRATCHPAD_SIZE;
		len -= MIKROBUS_ID_EEPROM_SCRATCHPAD_SIZE;
	}

	if (count % MIKROBUS_ID_EEPROM_SCRATCHPAD_SIZE)
	{
		memcpy(scratchpad_write, data + wraddr, count % MIKROBUS_ID_EEPROM_SCRATCHPAD_SIZE);
		mikrobus_write_memory(scratchpad_write, wraddr + writeaddr, w1_io_context, w1_gpio_io);
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

static int mikrobus_setup_eeprom_protection_control(uint8_t fixed_blocks, struct w1_io_context *w1_io_context, struct w1_io *w1_gpio_io) {

	uint8_t protcontrol_read[MIKROBUS_ID_EEPROM_BLOCK_PROT_CONTROL_SIZE];
	uint8_t protcontrol_write[MIKROBUS_ID_EEPROM_BLOCK_PROT_CONTROL_SIZE] = {0};
	uint8_t TA1, TA2, ES, readbyte[MIKROBUS_ID_EEPROM_BLOCK_PROT_CONTROL_SIZE];
	int found, iter = 0;
	int i;


	mikrobus_read_block(protcontrol_read, MIKROBUS_ID_EEPROM_BLOCK_PROT_CONTROL_SIZE, MIKROBUS_ID_EEPROM_PROTECTION_CTRL_ADDR, w1_io_context, w1_gpio_io);
	LOG_HEXDUMP_DBG(protcontrol_read, sizeof(protcontrol_read), "mikrobus eeprom protection control bytes [0x0A00]:");

	/* write protect blocks 7-9 (fixed manifest) */
	for( i = 0; i < fixed_blocks; i++)
		 protcontrol_write[i] = 0x55;

    LOG_HEXDUMP_DBG(protcontrol_write, sizeof(protcontrol_write), "mikrobus eeprom protection control write bytes [0x0A00]:");

	/* check if we really need to update protection status */
	if (memcmp(protcontrol_read, protcontrol_write, MIKROBUS_ID_EEPROM_BLOCK_PROT_CONTROL_SIZE)) {

		found = w1_gpio_io->reset_bus(w1_io_context);
		if (found)
		{
			LOG_ERR("w1 device not found");
			return -ENODEV;
		}
		w1_gpio_io->write_8(w1_io_context, W1_SKIP_ROM_CMD);
		w1_gpio_io->write_8(w1_io_context, MIKROBUS_ID_EEPROM_WRITE_SCRATCHPAD_CMD);
		w1_gpio_io->write_8(w1_io_context, MIKROBUS_ID_EEPROM_PROTECTION_CTRL_ADDR & 0xFF);
		w1_gpio_io->write_8(w1_io_context, (MIKROBUS_ID_EEPROM_PROTECTION_CTRL_ADDR >> 8) & 0xFF);

		for (iter = 0; iter < MIKROBUS_ID_EEPROM_BLOCK_PROT_CONTROL_SIZE; iter++)
		{
			w1_gpio_io->write_8(w1_io_context, protcontrol_write[iter]);
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

		for (iter = 0; iter < MIKROBUS_ID_EEPROM_BLOCK_PROT_CONTROL_SIZE; iter++)
		{
			readbyte[iter] = w1_gpio_io->read_8(w1_io_context);
		}

		if (memcmp(readbyte, protcontrol_write, MIKROBUS_ID_EEPROM_BLOCK_PROT_CONTROL_SIZE)) {
			LOG_ERR("mikrobus scratchpad verify mismatch");
			return -EINVAL;
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
		w1_gpio_io->write_8(w1_io_context, MIKROBUS_ID_EEPROM_PROTECTION_CTRL_ADDR & 0xFF);
		w1_gpio_io->write_8(w1_io_context, (MIKROBUS_ID_EEPROM_PROTECTION_CTRL_ADDR >> 8) & 0xFF);
		w1_gpio_io->write_8(w1_io_context, ES);
		k_sleep(K_MSEC(10));
	}
	mikrobus_read_block(protcontrol_read, MIKROBUS_ID_EEPROM_BLOCK_PROT_CONTROL_SIZE, MIKROBUS_ID_EEPROM_PROTECTION_CTRL_ADDR, w1_io_context, w1_gpio_io);
	LOG_HEXDUMP_DBG(protcontrol_read, sizeof(protcontrol_read), "mikrobus eeprom protection control bytes [0x0A00]:");

	return 0;
}

static int mikrobus_write_user_eeprom(uint8_t fixed_blocks, struct w1_io_context *w1_io_context, struct w1_io *w1_gpio_io) {

	uint8_t eeprom_read[MIKROBUS_ID_USER_EEPROM_SIZE];
	uint8_t eeprom_write[MIKROBUS_ID_USER_EEPROM_SIZE] = {0};
	uint8_t TA1, TA2, ES, readbyte[MIKROBUS_ID_USER_EEPROM_SIZE];
	int found, iter = 0;
	int i;

	mikrobus_read_block(eeprom_read, MIKROBUS_ID_USER_EEPROM_SIZE, MIKROBUS_ID_USER_EEPROM_ADDR, w1_io_context, w1_gpio_io);
	LOG_HEXDUMP_DBG(eeprom_read, sizeof(eeprom_read), "mikrobus user eeprom bytes [0x0A0A]:");

	eeprom_write[0] = fixed_blocks;
	found = w1_gpio_io->reset_bus(w1_io_context);
	if (found)
	{
		LOG_ERR("w1 device not found");
		return -ENODEV;
	}
	w1_gpio_io->write_8(w1_io_context, W1_SKIP_ROM_CMD);
	w1_gpio_io->write_8(w1_io_context, MIKROBUS_ID_EEPROM_WRITE_SCRATCHPAD_CMD);
	w1_gpio_io->write_8(w1_io_context, MIKROBUS_ID_USER_EEPROM_ADDR & 0xFF);
	w1_gpio_io->write_8(w1_io_context, (MIKROBUS_ID_USER_EEPROM_ADDR >> 8) & 0xFF);

	for (iter = 0; iter < MIKROBUS_ID_USER_EEPROM_SIZE; iter++)
	{
		w1_gpio_io->write_8(w1_io_context, eeprom_write[iter]);
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

	for (iter = 0; iter < MIKROBUS_ID_USER_EEPROM_SIZE; iter++)
	{
		readbyte[iter] = w1_gpio_io->read_8(w1_io_context);
	}

	if (memcmp(readbyte, eeprom_write, MIKROBUS_ID_USER_EEPROM_SIZE)) {
		LOG_ERR("mikrobus scratchpad verify mismatch");
		return -EINVAL;
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
	w1_gpio_io->write_8(w1_io_context, MIKROBUS_ID_USER_EEPROM_ADDR & 0xFF);
	w1_gpio_io->write_8(w1_io_context, (MIKROBUS_ID_USER_EEPROM_ADDR >> 8) & 0xFF);
	w1_gpio_io->write_8(w1_io_context, ES);
	k_sleep(K_MSEC(10));
	
	mikrobus_read_block(eeprom_read, MIKROBUS_ID_USER_EEPROM_SIZE, MIKROBUS_ID_USER_EEPROM_ADDR, w1_io_context, w1_gpio_io);
	LOG_HEXDUMP_DBG(eeprom_read, sizeof(eeprom_read), "mikrobus user eeprom bytes [0x0A0A]:");
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
	uint16_t mikrobus_variable_manifest_start_addr;

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

	/* write and verify fixed mikrobus manifest at start of eeprom */
	LOG_INF("writing mikrobus fixed manifest at 0x%x ", MIKROBUS_FIXED_MANIFEST_START_ADDR);
	err = mikrobus_write_block(mikrobus_manifest_click_fixed_builtin, sizeof(mikrobus_manifest_click_fixed_builtin), MIKROBUS_FIXED_MANIFEST_START_ADDR,  w1_io_context, w1_gpio_io);
	if(err) {
		LOG_ERR("mikrobus fixed manfest write failed");
		return err;
	}
	err = mikrobus_read_block(readdata, sizeof(mikrobus_manifest_click_fixed_builtin), MIKROBUS_FIXED_MANIFEST_START_ADDR, w1_io_context, w1_gpio_io);
	if(err) {
		LOG_ERR("mikrobus fixed manfest read failed");
		return err;
	}
	if (memcmp(readdata, mikrobus_manifest_click_fixed_builtin, sizeof(mikrobus_manifest_click_fixed_builtin))) {
		LOG_ERR("mikrobus fixed manfest read mismatch");
		return -EINVAL;
	}
	LOG_HEXDUMP_INF(readdata, sizeof(mikrobus_manifest_click_fixed_builtin), "mikrobus fixed manifest read data:");

	mikrobus_variable_manifest_start_addr = ((sizeof(mikrobus_manifest_click_fixed_builtin) >> 8) + 1) << 8;
	LOG_INF("writing mikrobus variable manifest at 0x%x ", mikrobus_variable_manifest_start_addr);
	
	/* write and verify variable mikrobus manifest at start of eeprom */
	err = mikrobus_write_block(mikrobus_manifest_click_builtin, sizeof(mikrobus_manifest_click_builtin), mikrobus_variable_manifest_start_addr,  w1_io_context, w1_gpio_io);
	if(err) {
		LOG_ERR("mikrobus variable manfest write failed");
		return err;
	}
	err = mikrobus_read_block(readdata, sizeof(mikrobus_manifest_click_builtin), mikrobus_variable_manifest_start_addr, w1_io_context, w1_gpio_io);
	if(err) {
		LOG_ERR("mikrobus variable manfest read failed");
		return err;
	}

	if (memcmp(readdata, mikrobus_manifest_click_builtin, sizeof(mikrobus_manifest_click_builtin))) {
		LOG_ERR("mikrobus variable manfest read mismatch");
		return -EINVAL;
	}
	LOG_HEXDUMP_INF(readdata, sizeof(mikrobus_manifest_click_builtin), "mikrobus variable manifest read data:");

	/*
	 * only enable write protection if really necessary (this is irreversible)
	 */
	//mikrobus_setup_eeprom_protection_control( mikrobus_variable_manifest_start_addr >> 8, w1_io_context, w1_gpio_io);

    mikrobus_write_user_eeprom(mikrobus_variable_manifest_start_addr >> 8, w1_io_context, w1_gpio_io);

	err = mikrobusid_exit_id_mode(context);
	if (err)
	{
		LOG_ERR("failed to exit mikrobus ID mode (err %d)", err);
		return err;
	}

	LOG_INF("MIKROBUS %s MANIFEST written succesfully \n", CONFIG_MIKROBUS_FLASHER_CLICK_NAME);

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
