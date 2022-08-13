#include "util.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "pico/stdio.h"
#include "pico/multicore.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/timer.h"

#include "bsp/board.h"
#include "tusb.h"

void serprint(const char* format, ...) {
    char buf[128];
    va_list args;
    va_start (args, format);
    vsnprintf(buf, 128, format, args);
    va_end (args);
    tud_cdc_n_write_str(ITF_CONSOLE, buf);
    tud_cdc_n_write_flush(ITF_CONSOLE);
    tud_task();
}
