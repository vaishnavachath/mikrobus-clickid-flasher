/*
 * Copyright (c) 2021 Vaishnav M A. BeagleBoard.org Foundation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <zephyr.h>

#include <errno.h>
#include <kernel.h>
#include <stdlib.h>
#include <stddef.h>

#include <drivers/gpio.h>
#include "w1-gpio.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(w1_gpio, CONFIG_MIKROBUS_LOG_LEVEL);

#define DELAY_NOP "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"

#define DELAY_US                                   \
    do                                             \
    {                                              \
        __asm volatile(DELAY_NOP                   \
                           DELAY_NOP               \
                               DELAY_NOP           \
                                   DELAY_NOP       \
                                       DELAY_NOP); \
    } while (0)

#define US_TO_SYS_CLOCK_HW_CYCLES(us) ((uint64_t)sys_clock_hw_cycles_per_sec() * (us) / USEC_PER_SEC + 1)

static inline void w1_delay(unsigned int cycles_to_wait)
{
    cycles_to_wait = US_TO_SYS_CLOCK_HW_CYCLES(cycles_to_wait);
    uint32_t start = k_cycle_get_32();

    /* Wait until the given number of cycles have passed */
    while (k_cycle_get_32() - start < cycles_to_wait)
    {
    }
}

static inline void w1_delay_precise(unsigned int us)
{
    us -= 2;
    us -= us / 4;
    while (us--)
        DELAY_US;
}

static inline void w1_write_bit(struct w1_io_context *io_context, uint8_t bit)
{
    gpio_config(io_context->w1_gpio, io_context->w1_pin,
                GPIO_PULL_UP | GPIO_OUTPUT);
    if (bit)
    {
        gpio_pin_set(io_context->w1_gpio, io_context->w1_pin, 0);
        w1_delay_precise(6);
        gpio_pin_set(io_context->w1_gpio, io_context->w1_pin, 1);
        w1_delay_precise(64);
    }
    else
    {
        gpio_pin_set(io_context->w1_gpio, io_context->w1_pin, 0);
        w1_delay_precise(60);
        gpio_pin_set(io_context->w1_gpio, io_context->w1_pin, 1);
        w1_delay_precise(10);
    }
}

static inline uint8_t w1_read_bit(struct w1_io_context *io_context)
{
    int result;

    gpio_config(io_context->w1_gpio, io_context->w1_pin,
                GPIO_PULL_UP | GPIO_OUTPUT);
    gpio_pin_set(io_context->w1_gpio, io_context->w1_pin, 0);
    w1_delay_precise(6);
    gpio_pin_set(io_context->w1_gpio, io_context->w1_pin, 1);
    w1_delay_precise(9);

    gpio_config(io_context->w1_gpio, io_context->w1_pin,
                GPIO_PULL_UP | GPIO_INPUT);

    result = gpio_pin_get(io_context->w1_gpio, io_context->w1_pin) & 0x1;

    w1_delay_precise(55);

    return result;
}

static inline uint8_t w1_touch_bit(struct w1_io_context *io_context, int bit)
{
    if (bit)
        return w1_read_bit(io_context);
    else
    {
        w1_write_bit(io_context, 0);
        return 0;
    }
}

static inline void w1_write_8(struct w1_io_context *io_context, uint8_t byte)
{
    unsigned int key;
    int i;

    key = irq_lock();
    for (i = 0; i < 8; i++)
        w1_write_bit(io_context, (byte >> i) & 0x1);
    irq_unlock(key);
}

static inline uint8_t w1_read_8(struct w1_io_context *io_context)
{
    int i;
    uint8_t res = 0;
    unsigned int key;

    key = irq_lock();
    for (i = 0; i < 8; i++)
    {
        res >>= 1;
        if (w1_read_bit(io_context))
        {
            res |= 0x80;
        }
    }
    irq_unlock(key);
    return res;
}

static inline int w1_reset_bus(struct w1_io_context *io_context)
{
    int result = 0;
    unsigned int key;

    key = irq_lock();
    gpio_config(io_context->w1_gpio, io_context->w1_pin,
                GPIO_PULL_UP | GPIO_OUTPUT);
    gpio_pin_set(io_context->w1_gpio, io_context->w1_pin, 0);
    w1_delay_precise(500);
    gpio_pin_set(io_context->w1_gpio, io_context->w1_pin, 1);
    w1_delay_precise(70);
    gpio_config(io_context->w1_gpio, io_context->w1_pin,
                GPIO_PULL_UP | GPIO_INPUT);
    result = gpio_pin_get(io_context->w1_gpio, io_context->w1_pin) ? 1 : 0;
    irq_unlock(key);
    k_sleep(K_MSEC(1));
    return result;
}

int w1_gpio_init(struct w1_gpio_master *w1_master)
{
    w1_master->w1_gpio_io = malloc(sizeof(struct w1_io));
    if (!w1_master->w1_gpio_io)
        return -ENOMEM;
    struct w1_io *w1_gpio_io = w1_master->w1_gpio_io;
    struct w1_io_context *io_context = w1_master->io_context;
    w1_gpio_io->write_8 = w1_write_8;
    w1_gpio_io->read_8 = w1_read_8;
    w1_gpio_io->reset_bus = w1_reset_bus;

    gpio_pin_set(io_context->w1_gpio, io_context->w1_pin, 1);
    return 0;
}
