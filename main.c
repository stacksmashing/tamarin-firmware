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
#include "hardware/watchdog.h"

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

volatile bool serialEnabled = false;

void configure_rx(PIO pio, uint sm) {
    pio_sm_set_enabled(pio, sm, false);
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &lightning_rx_program);
    pio_sm_config c = lightning_rx_program_init(pio, sm, offset, PIN_SDQ, 125.0/2.0);
}

void leave_dcsd() {
    uart_deinit(uart0);
    serialEnabled = false;
}

void tamarin_reset_tristar(PIO pio, uint sm) {
    leave_dcsd();
    
    // All pins connected to shell (GND) means cable is currently being inserted
    for (int i = 0; i < 4; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_OUT);
        gpio_put(i, 0);
    }
    
    sleep_us(1000);
    for (int i = 0; i < 4; i++) {
        gpio_set_dir(i, GPIO_IN);
    }
    
    configure_rx(pio, sm);
}

unsigned char reverse_byte(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

enum state {
    RESTART_ENUMERATION,
    WAITING_FOR_INIT,
    READING_TRISTAR_REQUEST,
    HANDLE_TRISTAR_REQUEST,
};

volatile enum state gState = RESTART_ENUMERATION;

enum command {
    CMD_DEFAULT,
    CMD_RESET,
    CMD_AUTO_DFU,
    // Used as 'second stage' for the DFU command. Saves us a state machine.
    CMD_INTERNAL_AUTO_DFU_2,
    CMD_MAX,
};

enum default_command {
    DEFAULT_CMD_DCSD = 0,
    DEFAULT_CMD_JTAG
};

// Command that should be sent automatically (on reboot, plugin, etc.)
// Automatically changed when selecting DCSD/JTAG
enum default_command gDefaultCommand = DEFAULT_CMD_DCSD;

// Next command to send
enum command gCommand = CMD_DEFAULT;

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
    [RSP_USB_UART] = {0x75, 0x20, 0x00, 0x10, 0x00, 0x00, 0x00, 0x92},
    [RSP_RESET] = {0x75, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x83},
    [RSP_DFU] = {0x75, 0x20, 0x00, 0x02, 0x00, 0x00, 0x00, 0xad},
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
    
    serialEnabled = true;
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
    while (!pio_sm_is_tx_fifo_empty(pio, sm)) {
        sleep_us(500);
    }
}

void output_state_machine() {
    PIO pio = pio1;
    uint sm = pio_claim_unused_sm(pio, true);

    uint8_t i = 0;
    uint8_t buf[4];

    uint32_t value, value_b;
    while(1) {
        switch(gState) {
            case RESTART_ENUMERATION:
                serprint("Restarting enumeration!\r\n");
                tamarin_reset_tristar(pio, sm);
                serprint("Done restarting enumeration!\r\n");
                gState = WAITING_FOR_INIT;
                break;
                
            case WAITING_FOR_INIT:
                if (pio_sm_is_rx_fifo_empty(pio, sm)) break;
                value = pio_sm_get_blocking(pio, sm);
                value_b = reverse_byte(value & 0xFF);

                if(value_b == 0x74) {
                    leave_dcsd();
                    gState = READING_TRISTAR_REQUEST;
                    buf[0] = value_b;
                    i = 1;
                } else {
                    serprint("Tristar >> 0x%x (unknown, ignoring)\r\n", value_b);
                }
                
                sleep_us(100); // Breaks without this...
                
                break;
            case READING_TRISTAR_REQUEST:
                if (pio_sm_is_rx_fifo_empty(pio, sm)) break;
                value = pio_sm_get_blocking(pio, sm);
                value_b = reverse_byte(value & 0xFF);

                buf[i++] = value_b;
                if(i == 4) {
                    gState = HANDLE_TRISTAR_REQUEST;
                    i = 0;
                }
                
                sleep_us(100); // Breaks without this...
                
                break;
            case HANDLE_TRISTAR_REQUEST:
                switch(gCommand) {
                    case CMD_DEFAULT:
                        switch (gDefaultCommand) {
                            case DEFAULT_CMD_DCSD:
                                respond_lightning(pio, sm, bootloader_response[RSP_USB_UART], 8);
                                dcsd_mode(pio, sm);
                                break;
                                
                            case DEFAULT_CMD_JTAG:
                                respond_lightning(pio, sm, bootloader_response[RSP_USB_UART_JTAG], 8);
                                jtag_mode(pio, sm);
                                break;
                        }
                        break;
                    case CMD_RESET:
                        respond_lightning(pio, sm, bootloader_response[RSP_RESET], 8);
                        sleep_us(1000);
                        gCommand = CMD_DEFAULT;
                        break;
                    case CMD_AUTO_DFU:
                        respond_lightning(pio, sm, bootloader_response[RSP_RESET], 8);
                        // Measured with logic analyzer
                        sleep_us(900);
                        gCommand = CMD_INTERNAL_AUTO_DFU_2;
                        break;
                    case CMD_INTERNAL_AUTO_DFU_2:
                        respond_lightning(pio, sm, bootloader_response[RSP_DFU], 8);
                        serprint("Device should now be in DFU mode.\r\n");
                        break;
                    default:
                        serprint("UNKNOWN MODE. Send help. Locking up.\r\n");
                        while(1) {}
                        break;
                }
                
                gState = WAITING_FOR_INIT;
                configure_rx(pio, sm);
                break;
        }
    }
}

void print_menu() {
    serprint("Good morning!\r\n\r\n");
    serprint("1: JTAG mode [currently unavailable]\r\n");
    serprint("2: DCSD mode\r\n");
    serprint("3: Reset device\r\n");
    serprint("4: Reset and enter DFU mode\r\n");
    serprint("R: Reset Tamarin cable\r\n");
    serprint("> ");
}

void shell_task() {
    if (tud_cdc_n_available(ITF_CONSOLE)) {
        char c = tud_cdc_n_read_char(ITF_CONSOLE);
        switch(c) {
            case '1':
                /*serprint("\r\nEnabling JTAG mode.\r\n");
                gCommand = CMD_DEFAULT;
                gDefaultCommand = DEFAULT_CMD_JTAG;
                gState = RESTART_ENUMERATION;*/
                serprint("\r\nJTAG mode not yet available!\r\n");
                break;
            case '2':
                serprint("\r\nEnabling DCSD mode.\r\n");
                gCommand = CMD_DEFAULT;
                gDefaultCommand = DEFAULT_CMD_DCSD;
                gState = RESTART_ENUMERATION;
                break;
            case '3':
                serprint("\r\nResetting.\r\n");
                gCommand = CMD_RESET;
                gState = RESTART_ENUMERATION;
                break;
            case '4':
                serprint("\r\nEnabling DFU mode.\r\n");
                gCommand = CMD_AUTO_DFU;
                gState = RESTART_ENUMERATION;
                break;
            case 'R':
            case 'r':
                watchdog_enable(100, 1);
                break;
            default:
                print_menu();
                break;
        }
    }
}


int main() {
    board_init();
    tusb_init();
    
    uart_init(uart1, 115200);
    gpio_set_function(8, GPIO_FUNC_UART);
    gpio_set_function(9, GPIO_FUNC_UART);

    multicore_launch_core1(output_state_machine);
    
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
        
        if (serialEnabled) {
            if (uart_is_readable(uart0)) {
                tud_cdc_n_write_char(ITF_DCSD, uart_getc(uart0));
                tud_cdc_n_write_flush(ITF_DCSD);
            }
            
            if (tud_cdc_n_available(ITF_DCSD)) {
                uart_putc_raw(uart0, tud_cdc_n_read_char(ITF_DCSD));
            }
        }
    }
}
