/*
 * Licensed under GNU Public License v3
 * Copyright (c) 2022 Thomas Roth <code@stacksmashing.net>
 * Based on Picoprobe by:
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 */

#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

#include <hardware/clocks.h>
#include <hardware/gpio.h>

#include "tamarin_config.h"
#include "probe.pio.h"
#include "tusb.h"
#include "util.h"

// Disable all prints for now
#define serprint(...)

enum TAMARIN_CMDS {
    TAMARIN_INVALID      = 0, // Invalid command
    TAMARIN_READ = 1,
    TAMARIN_WRITE = 2,
    TAMARIN_LINE_RESET = 3,
    TAMARIN_SET_FREQ = 4
};

// This struct is the direct struct that is sent to
// the probe.
struct __attribute__((__packed__)) tamarin_cmd_hdr {
    // Currently unused
	uint8_t id;
    // One of TAMARIN_CMDS
    uint8_t cmd;
    // The full (incl. start/stop/parity) SWD command
    // Unused for LINE_RESET and SET_FREQ.
    uint8_t request;
    // The data for writes (unused otherwise)
    uint32_t data;
    // Number of (8 bit) idle cycles to perform after this op
	uint8_t idle_cycles;
};

// This is the struture returned by the probe for each command
struct __attribute__((__packed__)) tamarin_res_hdr {
    // Unused
	uint8_t id;
    // The (3 bit) result: OK/WAIT/FAULT
    uint8_t res;
    // The data for reads (undefined otherwise)
    uint32_t data;
};

#define PROBE_BUF_SIZE 8192
struct _probe {
    // Data received from computer
    struct tamarin_cmd_hdr probe_cmd;

    // Data that will be sent back to the computer
    struct tamarin_res_hdr probe_res;

    // PIO offset
    uint offset;
};

static struct _probe probe;

void probe_set_swclk_freq(uint freq_khz) {
    tamarin_info("Setting SWD frequency to %d kHz\r\n", freq_khz);
    uint clk_sys_freq_khz = clock_get_hz(clk_sys) / 1000;
    // Worked out with saleae
    uint32_t divider = clk_sys_freq_khz / freq_khz / 2;
    pio_sm_set_clkdiv_int_frac(pio0, PROBE_SM, divider, 0);
}

static inline void probe_write_bits(uint bit_count, uint32_t data_byte) {

    serprint(">> %X (%d)\r\n", data_byte, bit_count);
    pio_sm_put_blocking(pio0, PROBE_SM, bit_count - 1);
    pio_sm_put_blocking(pio0, PROBE_SM, data_byte);
    pio_sm_get_blocking(pio0, PROBE_SM);
}

static inline uint32_t probe_read_bits(uint bit_count) {
    pio_sm_put_blocking(pio0, PROBE_SM, bit_count - 1);
    uint32_t data = pio_sm_get_blocking(pio0, PROBE_SM);
    data = data >> (32-bit_count);
    serprint("<< %X (%d)\r\n", data, bit_count);
    return data;
}

static void probe_read_mode(void) {
    pio_sm_exec(pio0, PROBE_SM, pio_encode_jmp(probe.offset + probe_offset_in_posedge));
    while(pio0->dbg_padoe & (1 << PROBE_PIN_SWDIO));
}

static void probe_write_mode(void) {
    pio_sm_exec(pio0, PROBE_SM, pio_encode_jmp(probe.offset + probe_offset_out_negedge));
    while(!(pio0->dbg_padoe & (1 << PROBE_PIN_SWDIO)));
}

void tamarin_start_probe() {
    // set to output
    pio_sm_set_consecutive_pindirs(pio0, PROBE_SM, PROBE_PIN_OFFSET, 2, true);
    // Enable SM
    pio_sm_set_enabled(pio0, PROBE_SM, 1);

    // Jump to write program
    // probe_write_mode();
    probe_read_mode();
    // probe_enabled = true;
}

void tamarin_probe_init() {
    // Funcsel pins
    pio_gpio_init(pio0, PROBE_PIN_SWCLK);
    pio_gpio_init(pio0, PROBE_PIN_SWDIO);
    // Make sure SWDIO has a pullup on it. Idle state is high
    gpio_pull_up(PROBE_PIN_SWDIO);

    // Target reset pin: pull up, input to emulate open drain pin
    gpio_pull_up(PROBE_PIN_RESET);
    // gpio_init will leave the pin cleared and set as input
    gpio_init(PROBE_PIN_RESET);

    uint offset = pio_add_program(pio0, &probe_program);
    probe.offset = offset;

    pio_sm_config sm_config = probe_program_get_default_config(offset);

    // Set SWCLK as a sideset pin
    sm_config_set_sideset_pins(&sm_config, PROBE_PIN_SWCLK);

    // Set SWDIO offset
    sm_config_set_out_pins(&sm_config, PROBE_PIN_SWDIO, 1);
    sm_config_set_set_pins(&sm_config, PROBE_PIN_SWDIO, 1);
    sm_config_set_in_pins(&sm_config, PROBE_PIN_SWDIO);

    // Set pins to input to ensure we don't pull the line low unnecessarily
    pio_sm_set_consecutive_pindirs(pio0, PROBE_SM, PROBE_PIN_OFFSET, 2, false);

    // shift output right, autopull off, autopull threshold
    sm_config_set_out_shift(&sm_config, true, false, 0);
    // shift input right as swd data is lsb first, autopush off
    sm_config_set_in_shift(&sm_config, true, false, 0);

    // Init SM with config
    pio_sm_init(pio0, PROBE_SM, offset, &sm_config);

    // Set up divisor
    probe_set_swclk_freq(1000);

    tamarin_start_probe();
}

void tamarin_probe_deinit() {
    pio_sm_set_enabled(pio0, PROBE_SM, false);
    pio_clear_instruction_memory(pio0);
    gpio_disable_pulls(PROBE_PIN_SWDIO);
}

int __not_in_flash_func(tamarin_tx_read_bare)(uint8_t request, uint32_t *value);

void tamarin_line_reset() {
        probe_write_mode();
        probe_write_bits(32, 0xFFFFFFFF);
        probe_write_bits(32, 0xFFFFFFFF);
        probe_write_bits(16, 0xe79e);
        probe_write_bits(32, 0xFFFFFFFF);
        probe_write_bits(32, 0xFFFFFFFF);
        // probe_write_bits(8, 0x0);
        uint32_t foo;
        tamarin_tx_read_bare(0xa5, &foo);
    probe_read_mode();
}

int __not_in_flash_func(tamarin_tx_read_bare)(uint8_t request, uint32_t *value) {

    // uint8_t data_reversed = reverse(request);
    uint32_t result;
    for(int i=0; i < 5; i++) {
        serprint("Read loop");
        probe_write_mode();

        // Idle cycles just in case
        probe_write_bits(32, 0x0);

        // probe_write_bits(32, 0x0);
        probe_write_bits(32, 0x0);
        probe_write_bits(32, 0x0);
        probe_write_bits(32, 0x0);
        
        probe_write_bits(8, request);
        // Read parity bit
        probe_write_bits(1, 1);
        probe_read_mode();
        result = probe_read_bits(3);
        if(result == 2) {
            serprint("Read - Probe received wait. Try: %d\r\n", i);
            // because we probably have overrun on we need to perform a
            // data phase.
            uint32_t read_data = probe_read_bits(32);
            uint32_t result_parity = probe_read_bits(1);
            continue;
        }
        if(result != 1) {
            return result;
        }
        break;
    }

    
    uint32_t read_data = probe_read_bits(32);
    uint32_t result_parity = probe_read_bits(1);
    *value = read_data;
    serprint("READ - Result: %d Data: %08X (%d)\r\n", result, read_data, result_parity);
    return result;
}


int __not_in_flash_func(tamarin_tx_write_bare)(uint8_t request, uint32_t value) {
    uint32_t value_parity = __builtin_parity(value);
    uint32_t result;
    for(int i=0; i < 5; i++) {
        serprint("Write loop");
        probe_write_mode();

        // Idle cycles... These can be reduced
        probe_write_bits(32, 0x0);
        probe_write_bits(32, 0x0);
        probe_write_bits(32, 0x0);
        probe_write_bits(32, 0x0);
        
        probe_write_bits(8, request);

        // Read parity bit
        probe_write_bits(1, 1);

        probe_read_mode();
        result = probe_read_bits(3);
        if(result == 2) {
            serprint("Write - Probe received wait. Try: %d\r\n", i);
            // With overrun detection we still have to do the datapahse
            probe_write_mode();
            // turn
            probe_write_bits(1, 0x1);
            probe_write_bits(32, 0x0);
            probe_write_bits(1, 0x1);
            // uint32_t result_parity = probe_read_bits(1);
            continue;
        }
        if(result != 1) {
            return result;
        }
        break;
    }

    // Another turn!
    probe_write_mode();
    probe_write_bits(1, 1);
    // Write the actual data
    probe_write_bits(32, value);
    // Write parity bit
    probe_write_bits(1, value_parity);

    probe_read_mode();
    serprint("READ - Result: %d\r\n", result);
    return result;
}

bool probe_enabled = false;
void probe_handle_pkt(void) {
    struct tamarin_cmd_hdr *cmd = &probe.probe_cmd;

    tamarin_debug("Processing packet: ID: %u Command: %u Request: 0x%02X Data: 0x%08X Idle: %d\r\n", cmd->id, cmd->cmd, cmd->request, cmd->data, cmd->idle_cycles);
    int result = 0;
    uint32_t data;
    switch(cmd->cmd) {
        case TAMARIN_READ:
            tamarin_debug("Executing read\r\n");
            
            result = tamarin_tx_read_bare(cmd->request, &data);
            tamarin_debug("Read: %d 0x%08X", result, data);
            break;
        case TAMARIN_WRITE:
            tamarin_debug("Executing write\r\n");
            result = tamarin_tx_write_bare(cmd->request, cmd->data);
            tamarin_debug("Wrrite: %d 0x%08X", result, cmd->data);
            break;
        case TAMARIN_LINE_RESET:
            tamarin_debug("Executing line reset\r\n");
            tamarin_line_reset();
            break;
        case TAMARIN_SET_FREQ:
            tamarin_debug("Executing set frequency\r\n");
            probe_set_swclk_freq(cmd->data);
            break;
        default:
            tamarin_debug("UNKNOWN COMMAND!\r\n");
            break;
    }

    // For now we just reply with a default command
    struct tamarin_res_hdr res;
    res.id = 33;
    res.data = data;
    res.res = result;


    tud_task();
    tud_vendor_write((char*)&res, sizeof(res));
    tud_task();
    tud_vendor_flush();
    tud_task();
}

// USB bits
void tamarin_probe_task(void) {
    if ( tud_vendor_available() ) {
        char tmp_buf[64];
        uint count = tud_vendor_read(tmp_buf, 64);
        if (count == 0) {
            return;
        }

        memcpy(&probe.probe_cmd, tmp_buf, sizeof(struct tamarin_cmd_hdr));
        probe_handle_pkt();
    }
}
