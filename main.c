#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>


#include "pico/stdio.h"
#include "pico/multicore.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/timer.h"

#include "lightning_rx.pio.h"
#include "lightning_tx.pio.h"

#include "bsp/board.h"
#include "tusb.h"

#include "tamarin_probe.h"
#include "util.h"


#define PIN_SDQ 3

// Used to initialize picoprobe
volatile bool probe_active = 0;
volatile bool probe_initialized = 0;


unsigned char reverse_byte(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

enum state {
    IDLE,
    WAITING_FOR_INIT,
    READING_TRISTAR_REQUEST,
    HANDLE_TRISTAR_REQUEST,
};

enum command {
    CMD_JTAG = 0,
    CMD_DCSD,
    CMD_RESET,
    CMD_AUTO_DFU,
    // Used as 'second stage' for the DFU command. Saves us a state machine.
    CMD_INTERNAL_AUTO_DFU_2,
    CMD_MAX,
};

enum responses {
    // JTAG mode
    RSP_USB_UART_JTAG,
    // DCSD mode
    RSP_USB_UART,
    // Reset
    RSP_RESET,
    // DFU
    RSP_DFU,
    RSP_MAX
};

// To generate CRCs:
// hex(pwnlib.util.crc.generic_crc(b"\x75\x00\x00\x02\x00\x00\x00", 0x31, 8, 0xff, True, True, False))
const uint8_t bootloader_response[RSP_MAX][8] = {
    [RSP_USB_UART_JTAG] = {0x75, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40},
    [RSP_USB_UART] = {0x75, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaa},
    [RSP_RESET] = {0x75, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x83},
    [RSP_DFU] = {0x75, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x1b},
};

void set_idbus_high_impedance() {
    gpio_pull_up (PIN_SDQ);
    gpio_init(PIN_SDQ);
    gpio_set_dir(PIN_SDQ, GPIO_IN);
}

#define DCSD_UART uart0
#define DCSD_TX_PIN 0
#define DCSD_RX_PIN 1

void dcsd_mode(PIO pio, uint sm) {
    uart_init(uart0, 115200);
    gpio_set_function(DCSD_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DCSD_RX_PIN, GPIO_FUNC_UART);

    serprint("DCSD mode active.\r\n");
    serprint("Connect to the second serial port of the\r\n");
    serprint("Tamarin Cable to access the monitor.\r\n");
    while(1) {
        if(uart_is_readable(uart0)) {
            tud_cdc_n_write_char(ITF_DCSD, uart_getc(uart0));
        }
        if(tud_cdc_n_available(ITF_DCSD)) {
            uart_putc_raw(uart0, tud_cdc_n_read_char(ITF_DCSD));
        }
    }
}

void jtag_mode(PIO pio, uint sm) {
    // wait a second for this to be settled, then switch to high-z for JeTAG
    sleep_ms(500);
    set_idbus_high_impedance();
    pio_sm_set_enabled(pio, sm, false);
    probe_active = true;
    serprint("JTAG mode active, ID pin in Hi-Z.\r\n");
    serprint("You can now connect with an SWD debugger.\r\n");
    while(1) {}
}

void respond_lightning(PIO pio, uint sm, const uint8_t *data, size_t data_length) {
    pio_sm_set_enabled(pio, sm, false);
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &lightning_tx_program);
    pio_sm_config c = lightning_tx_program_init(pio, sm, offset, PIN_SDQ, 125.0/2.0);
    for(size_t i=0; i < data_length; i++) {
        pio_sm_put_blocking(pio, sm, data[i]);
    }
}

void configure_rx(PIO pio, uint sm) {
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &lightning_rx_program);
    pio_sm_config c = lightning_rx_program_init(pio, sm, offset, PIN_SDQ, 125.0/2.0);
}

void output_state_machine() {
    PIO pio = pio1;
    uint sm = pio_claim_unused_sm(pio, true);
    configure_rx(pio, sm);

    uint8_t i = 0;
    uint8_t buf[4];

    // This is what DCSD receives before it answers.
    // Retrieved via logic analyzer.
    const uint8_t bootloader_command[4] = {
        0x74, 0x00, 0x07, 0x20
    };

    enum state state = IDLE;
    enum command op;
    uint32_t value, value_b;
    while(1) {
        switch(state) {
            case IDLE:
                op = multicore_fifo_pop_blocking();
                state = WAITING_FOR_INIT;
                break;
            case WAITING_FOR_INIT:
                value = pio_sm_get_blocking(pio, sm);
                value_b = reverse_byte(value & 0xFF);

                if(value_b == 0x74) {
                    serprint("Tristar request received...\r\n");
                    state = READING_TRISTAR_REQUEST;
                    buf[0] = value_b;
                    i = 1;
                }
                break;
            case READING_TRISTAR_REQUEST:
                value = pio_sm_get_blocking(pio, sm);
                value_b = reverse_byte(value & 0xFF);

                buf[i++] = value_b;
                if(i == 4) {
                    // if(memcmp(buf, bootloader_command, 4) == 0) {
                        state = HANDLE_TRISTAR_REQUEST;
                    // } else {
                    //     state = WAITING_FOR_INIT;
                    //     i = 0;
                    // }
                }
                break;
            case HANDLE_TRISTAR_REQUEST:
                switch(op) {
                    case CMD_JTAG:
                        respond_lightning(pio, sm, bootloader_response[RSP_USB_UART_JTAG], 8);
                        jtag_mode(pio, sm);
                        break;
                    case CMD_DCSD:
                        respond_lightning(pio, sm, bootloader_response[RSP_USB_UART], 8);
                        dcsd_mode(pio, sm);
                        break;
                    case CMD_RESET:
                        respond_lightning(pio, sm, bootloader_response[RSP_RESET], 8);
                        serprint("Resetting device!\r\n");
                        sleep_us(1000);
                        // op = CMD_INTERNAL_AUTO_DFU_2;
                        state = IDLE;
                        // Put SDQ back into receiving mode
                        configure_rx(pio, sm);
                        break;
                    case CMD_AUTO_DFU:
                        respond_lightning(pio, sm, bootloader_response[RSP_RESET], 8);
                        // Measured with logic analyzer
                        sleep_us(900);
                        serprint("Resetting device!\r\n");
                        // We wait for the next enumeration, then we send the DFU command
                        op = CMD_INTERNAL_AUTO_DFU_2;
                        state = WAITING_FOR_INIT;
                        // Put SDQ back into receiving mode
                        configure_rx(pio, sm);
                        break;
                    case CMD_INTERNAL_AUTO_DFU_2:
                        respond_lightning(pio, sm, bootloader_response[RSP_DFU], 8);
                        serprint("Device should now be in DFU mode.\r\n");
                        while(1) {}
                        break;
                    default:
                        serprint("UNKNOWN MODE. Send help. Locking up.\r\n");
                        while(1) {}
                        break;
                }
        }
    }
}

void print_menu() {
    serprint("Good morning!\r\n\r\n");
    serprint("1: JTAG mode\r\n");
    serprint("2: DCSD mode\r\n");
    serprint("3: Reset device\r\n");
    serprint("4: Reset and enter DFU mode\r\n");
    serprint("R: Reset Tamarin cable\r\n");
    serprint("> ");
}

void print_small_menu() {
    serprint("R: Reset Tamarin cable\r\n");
    serprint("> ");
}

enum SHELL_STATE {
    READY,
    BLOCKED
};

enum SHELL_STATE shell_state;
#define AIRCR_Register (*((volatile uint32_t*)(PPB_BASE + 0x0ED0C)))
void shell_task() {
    switch(shell_state) {
        case READY:
            if(tud_cdc_n_available(ITF_CONSOLE)) {
                char c = tud_cdc_n_read_char(ITF_CONSOLE);
                switch(c) {
                    case '1':
                        serprint("\r\nEnabling JTAG mode.\r\n");
                        multicore_fifo_push_blocking(CMD_JTAG);
                        shell_state = BLOCKED;
                        break;
                    case '2':
                        serprint("\r\nEnabling DCSD mode.\r\n");
                        multicore_fifo_push_blocking(CMD_DCSD);
                        shell_state = BLOCKED;
                        break;
                    case '3':
                        serprint("\r\nResetting.\r\n");
                        multicore_fifo_push_blocking(CMD_RESET);
                        // shell_state = BLOCKED;
                        break;
                    case '4':
                        serprint("\r\nEnabling DFU mode.\r\n");
                        multicore_fifo_push_blocking(CMD_AUTO_DFU);
                        // shell_state = BLOCKED;
                        break;
                    case 'R':
                        // set SYSRESETREQ
                        AIRCR_Register = 0x05FA0004;
                        break;
                    default:
                        print_menu();
                        break;
                }
            }
            break;
        case BLOCKED:
            if(tud_cdc_n_available(ITF_CONSOLE)) {
                char c = tud_cdc_n_read_char(ITF_CONSOLE);
                switch(c) {
                    case 'R':
                        // set SYSRESETREQ
                        AIRCR_Register = 0x05FA0004;
                        break;
                    default:
                        print_small_menu();
                        break;
                }
            }
            break;
    }
}


int main() {
    board_init();
    tusb_init();
    // Initialize state variables
    shell_state = READY;

    multicore_launch_core1(output_state_machine);
    
    // set_idbus_high_impedance();
    // pio_sm_set_enabled(pio, sm, false);
    probe_active = false;
    while(1) {
        tud_task();
        shell_task();
        if(probe_active) {
            if(!probe_initialized) {
                tamarin_probe_init();
                tud_task();
                probe_initialized = 1;
            }
            tamarin_probe_task();
        }
    }
}
