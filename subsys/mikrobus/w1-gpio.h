/*
 * Copyright (c) 2021 Vaishnav M A. BeagleBoard.org Foundation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

struct w1_io_context
{
    const struct device *w1_gpio;
    gpio_pin_t w1_pin;
};

struct w1_io
{
    void (*write_8)(struct w1_io_context *io_context, uint8_t byte);
    uint8_t (*read_8)(struct w1_io_context *io_context);
    int (*reset_bus)(struct w1_io_context *io_context);
};

struct w1_gpio_master
{
    struct w1_io_context *io_context;
    struct w1_io *w1_gpio_io;
};

int w1_gpio_init(struct w1_gpio_master *w1_master);