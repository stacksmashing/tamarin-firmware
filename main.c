#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#include "pico/stdio.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"

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
#include "crc.h"

#define PIN_SDQ 3

volatile bool serialEnabled = false;
volatile bool jtagInited = false;
volatile bool jtagEnabled = false;

void configure_rx(PIO pio, uint sm) {
    pio_sm_set_enabled(pio, sm, false);
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &lightning_rx_program);
    pio_sm_config c = lightning_rx_program_init(pio, sm, offset, PIN_SDQ, 125.0/2.0);
}

void leave_dcsd() {
    serialEnabled = false;
    
    uart_deinit(uart0);
}

void leave_jtag() {
    jtagEnabled = false;
    jtagInited  = false;
    
    tamarin_probe_deinit();
}

void lightning_send_wake() {
    gpio_init(PIN_SDQ);
    gpio_set_dir(PIN_SDQ, GPIO_OUT);
    gpio_put(PIN_SDQ, 0);
    
    sleep_us(20);
    
    gpio_set_dir(PIN_SDQ, GPIO_IN);
    
    sleep_us(50);
}

void tamarin_reset_tristar(PIO pio, uint sm) {
    leave_dcsd();
    leave_jtag();
    
    lightning_send_wake();
    
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
    READING_POWER_REQUEST,
    HANDLE_POWER_REQUEST,
    READING_TRISTAR_UNKNOWN_76,
    HANDLE_TRISTAR_UNKNOWN_76,
    HANDLE_JTAG,
    // Force JTAG mode if the cable is already in JTAG mode
    // (e.g. after the tamarin cable was reset but not the device)
    FORCE_JTAG
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
    DEFAULT_CMD_JTAG,
    DEFAULT_CMD_CHARGING
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
    RSP_USB_A_CHARGING_CABLE,
    RSP_MAX
};

enum TRISTAR_REQUESTS {
    TRISTAR_POLL = 0x74,
    TRISTAR_POWER = 0x70,
    TRISTAR_UNKNOWN_76 = 0x76,
};

// To generate CRCs:
// hex(pwnlib.util.crc.generic_crc(b"\x75\x00\x00\x02\x00\x00\x00", 0x31, 8, 0xff, True, True, False))
const uint8_t bootloader_response[RSP_MAX][7] = {
    [RSP_USB_UART_JTAG] = {0x75, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00},
    [RSP_USB_UART] = {0x75, 0x20, 0x00, 0x10, 0x00, 0x00, 0x00},
    [RSP_RESET] = {0x75, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00},
    [RSP_DFU] = {0x75, 0x20, 0x00, 0x02, 0x00, 0x00, 0x00},
    [RSP_USB_A_CHARGING_CABLE] = {0x75, 0x10, 0x0c, 0x00, 0x00, 0x00, 0x00},
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
    set_idbus_high_impedance();
    pio_sm_set_enabled(pio, sm, false);
    serprint("JTAG mode active, ID pin in Hi-Z.\r\n");
    serprint("You can now connect with an SWD debugger.\r\n");
    serprint("Please note: Reset/Reset to DFU will be unavailable until\r\n");
    serprint("the device is rebooted or the cable is re-plugged.\r\n");
    
    jtagInited  = false;
    jtagEnabled = true;
}

uint8_t crc_data(const uint8_t *data, size_t len) {
    crc_t crc = crc_init();
    crc = crc_update(crc, data, len);
    crc = crc_finalize(crc);
    return crc;
}

void respond_lightning(PIO pio, uint sm, const uint8_t *data, size_t data_length) {
    // Static buffer
    static uint8_t response_buffer[64];
    if(data_length > 63) {
        serprint("Lightning response too large, not sending.\r\n");
        return;
    }
    memcpy(response_buffer, data, data_length);
    response_buffer[data_length] = crc_data(data, data_length);

    pio_sm_set_enabled(pio, sm, false);
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &lightning_tx_program);
    pio_sm_config c = lightning_tx_program_init(pio, sm, offset, PIN_SDQ, 125.0/2.0);
    // We fill in 1 byte first, then the second byte, only then do we start to do
    // "get_blocking" to avoid the state machine ever running empty.
    pio_sm_put_blocking(pio, sm, response_buffer[0]);
    for(size_t i=1; i < data_length+1; i++) {
        pio_sm_put_blocking(pio, sm, response_buffer[i]);
        pio_sm_get_blocking(pio, sm);
    }
    pio_sm_get_blocking(pio, sm);
}

void output_state_machine() {
    char DFU[8];
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

                if(value_b == TRISTAR_POLL) {
                    leave_dcsd();
                    gState = READING_TRISTAR_REQUEST;
                    buf[0] = value_b;
                    i = 1;
                } else if(value_b == TRISTAR_POWER) {
                    gState = READING_POWER_REQUEST;
                    buf[0] = value_b;
                    i = 1;
                } else if(value_b == TRISTAR_UNKNOWN_76) {
                    gState = READING_TRISTAR_UNKNOWN_76;
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
                sleep_us(10); // Breaks without this...
                break;
            case READING_TRISTAR_UNKNOWN_76:
                if (pio_sm_is_rx_fifo_empty(pio, sm)) break;
                value = pio_sm_get_blocking(pio, sm);
                value_b = reverse_byte(value & 0xFF);

                buf[i++] = value_b;
                if(i == 2) {
                    gState = HANDLE_TRISTAR_UNKNOWN_76;
                    i = 0;
                }
                sleep_us(10); // Breaks without this...
                break;
            case HANDLE_TRISTAR_UNKNOWN_76:
                respond_lightning(pio, sm, "\x77\x02\x01\x02\x80\x60\x01\x39\x3a\x44\x3e\xc9", 11);
                serprint("76 request received: %02X %02X\r\n", buf[0], buf[1]);
                gState = WAITING_FOR_INIT;
                break;
            case READING_POWER_REQUEST:
                if (pio_sm_is_rx_fifo_empty(pio, sm)) break;
                value = pio_sm_get_blocking(pio, sm);
                value_b = reverse_byte(value & 0xFF);

                buf[i++] = value_b;
                if(i == 4) {
                    gState = HANDLE_POWER_REQUEST;
                    i = 0;
                }
                
                sleep_us(10); // Breaks without this...
                
                break;
            case HANDLE_POWER_REQUEST:
                respond_lightning(pio, sm, "\x71\x93", 1);
                serprint("Power request received: %02X %02X %02X %02X\r\n", buf[0], buf[1], buf[2], buf[3]);
                gState = WAITING_FOR_INIT;
                break;
            case HANDLE_TRISTAR_REQUEST:
                serprint("Tristar request received: %02X %02X %02X %02X\r\n", buf[0], buf[1], buf[2], buf[3]);
                switch(gCommand) {
                    case CMD_DEFAULT:
                        switch (gDefaultCommand) {
                            case DEFAULT_CMD_DCSD:
                                respond_lightning(pio, sm, bootloader_response[RSP_USB_UART], 7);
                                dcsd_mode(pio, sm);
                                break;
                                
                            case DEFAULT_CMD_JTAG:
                                respond_lightning(pio, sm, bootloader_response[RSP_USB_UART_JTAG], 7);
                                gState = FORCE_JTAG;
                                continue;
                            case DEFAULT_CMD_CHARGING:
                                respond_lightning(pio, sm, bootloader_response[RSP_USB_A_CHARGING_CABLE], 7);
                                serprint("Sent charging\r\n");
                                gState = WAITING_FOR_INIT;
                                break;
                        }
                        break;
                    case CMD_RESET:
                        respond_lightning(pio, sm, bootloader_response[RSP_RESET], 7);
                        sleep_us(1000);
                        gCommand = CMD_DEFAULT;
                        break;
                    case CMD_AUTO_DFU:
                        respond_lightning(pio, sm, bootloader_response[RSP_RESET], 7);
                        gCommand = CMD_INTERNAL_AUTO_DFU_2;
                        break;
                    case CMD_INTERNAL_AUTO_DFU_2:
                        respond_lightning(pio, sm, bootloader_response[RSP_DFU], 7);
                        serprint("Device should now be in DFU mode.\r\n");
                        gCommand = CMD_DEFAULT;
                        break;
                    default:
                        serprint("UNKNOWN MODE. Send help. Locking up.\r\n");
                        while(1) {}
                        break;
                }
                
                gState = WAITING_FOR_INIT;
                configure_rx(pio, sm);
                break;
                
            case HANDLE_JTAG:
                // Nothing to do
                break;
                
            case FORCE_JTAG:
                jtag_mode(pio, sm);
                dcsd_mode(pio, sm); // Also init serial
                gState = HANDLE_JTAG;
                break;
        }
    }
}

void print_menu() {
    serprint("Good morning!\r\n\r\n");
    serprint("1: JTAG mode\r\n");
    serprint("2: DCSD mode\r\n");
    serprint("3: Reset device\r\n");
    serprint("4: Reset and enter DFU mode (iPhone X and up only)\r\n");
    serprint("5: Reenumerate\r\n\r\n");
    serprint("F: Force JTAG mode without sending command\r\n");
    serprint("R: Reset Tamarin cable\r\n");
    serprint("U: Go into firmware update mode\r\n");
    serprint("> ");
}

void shell_task() {
    if (tud_cdc_n_available(ITF_CONSOLE)) {
        char c = tud_cdc_n_read_char(ITF_CONSOLE);
        tud_cdc_n_write_char(ITF_CONSOLE, c);
        switch(c) {
            case '1':
                serprint("\r\nEnabling JTAG mode.\r\n");
                gCommand = CMD_DEFAULT;
                gDefaultCommand = DEFAULT_CMD_JTAG;
                gState = RESTART_ENUMERATION;
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
            case '5':
                serprint("\r\nReenumerate\r\n");
                gCommand = CMD_DEFAULT;
                gState = RESTART_ENUMERATION;
                break;
            case 'f':
            case 'F':
                serprint("\r\nForcing JTAG mode.\r\n");
                gDefaultCommand = DEFAULT_CMD_JTAG;
                gState = FORCE_JTAG;
                break;
            case 'R':
            case 'r':
                watchdog_enable(100, 1);
                break;
            case 'U':
            case 'u':
                reset_usb_boot(0, 0);
            default:
                print_menu();
                break;
        }
    }
}

int main() {
    // Power enable for Tamarin Cable. Not required on regular Pico,
    // but also doesn't hurt.
    gpio_init(19);
    gpio_set_dir(19, GPIO_OUT);
    gpio_put(19, 1);

    board_init();
    tusb_init();

    multicore_launch_core1(output_state_machine);
    
    while(1) {
        tud_task();
        shell_task();
        
        // Handle serial
        if (serialEnabled) {
            if (uart_is_readable(uart0)) {
                tud_cdc_n_write_char(ITF_DCSD, uart_getc(uart0));
                tud_cdc_n_write_flush(ITF_DCSD);
            }
            
            if (tud_cdc_n_available(ITF_DCSD)) {
                uart_putc_raw(uart0, tud_cdc_n_read_char(ITF_DCSD));
            }
        }
        
        if (jtagEnabled) {
            if (!jtagInited) {
                tamarin_probe_init();
                tud_task();
                jtagInited = true;
            }
            
            tamarin_probe_task();
        }
    }
}
