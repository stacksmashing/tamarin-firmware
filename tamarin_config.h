/*
 * Licensed under GNU Public License v3
 * Copyright (c) 2022 Thomas Roth <code@stacksmashing.net>
 * Based on Picoprobe by:
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 */

#ifndef PICOPROBE_H_
#define PICOPROBE_H_

#if true
#define tamarin_info(format,args...) serprint(format, ## args)
#else
#define picoprobe_info(format,...) ((void)0)
#endif


#if true
#define tamarin_debug(format,args...) serprint(format, ## args)
#else
#define tamarin_debug(format,...) ((void)0)
#endif

#if true
#define tamarin_dump(format,args...) serprint(format, ## args)
#else
#define tamarin_dump(format,...) ((void)0)
#endif


// PIO config
#define PROBE_PIO pio0
#define PROBE_SM 0
#define PROBE_PIN_OFFSET 2
#define PROBE_PIN_SWCLK PROBE_PIN_OFFSET + 0 // 2
#define PROBE_PIN_SWDIO PROBE_PIN_OFFSET + 1 // 3

// Target reset config
// TODO: We do not support reset.
#define PROBE_PIN_RESET 6

// LED config
#ifndef PICOPROBE_LED

#ifndef PICO_DEFAULT_LED_PIN
#error PICO_DEFAULT_LED_PIN is not defined, run PICOPROBE_LED=<led_pin> cmake
#elif PICO_DEFAULT_LED_PIN == -1
#error PICO_DEFAULT_LED_PIN is defined as -1, run PICOPROBE_LED=<led_pin> cmake
#else
#define PICOPROBE_LED PICO_DEFAULT_LED_PIN
#endif

#endif

#endif
