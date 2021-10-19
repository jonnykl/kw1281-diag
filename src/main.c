/*
 * Copyright 2021 Jonathan Klamroth
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <usb/usb_device.h>

#include "kw1281.h"


#define ASYNC_ERROR_HANDLER_THREAD_STACK_SIZE       512
#define ASYNC_ERROR_HANDLER_THREAD_PRIO             5
#define ASYNC_ERROR_MSGQ_LENGTH                     16

#define STATE_MUTEX_LOCK_TIMEOUT_MS         500
#define KEEP_ALIVE_INTERVAL_MS              50


#define ABS(x)                              ((x)<0 ? -(x) : (x))


static int buf_print_float(char *buf, int buf_len, int precision, float val);
static int buf_print_measurement(char *buf, int buf_len, uint8_t id, uint8_t a, uint8_t b);

static int read_fault_codes(const struct shell *shell, int clear);
static int read_group(const struct shell *shell, uint8_t channel, uint8_t adapt);

static int cmd_scan(const struct shell *shell, size_t argc, char **argv);
static int cmd_connect(const struct shell *shell, size_t argc, char **argv);
static int cmd_read_fault_codes(const struct shell *shell, size_t argc, char **argv);
static int cmd_clear_fault_codes(const struct shell *shell, size_t argc, char **argv);
static int cmd_read_group(const struct shell *shell, size_t argc, char **argv);
static int cmd_basic_setting(const struct shell *shell, size_t argc, char **argv);
static int cmd_send_raw_block(const struct shell *shell, size_t argc, char **argv);
static int cmd_terminate(const struct shell *shell, size_t argc, char **argv);
static int cmd_baudrate(const struct shell *shell, size_t argc, char **argv);
static int cmd_inter_byte_time(const struct shell *shell, size_t argc, char **argv);
static int cmd_inter_block_time(const struct shell *shell, size_t argc, char **argv);
static int cmd_key_word_ack_delay(const struct shell *shell, size_t argc, char **argv);
static int cmd_timeout_multiplier(const struct shell *shell, size_t argc, char **argv);

static void keep_alive_thread(void *arg0, void *arg1, void *arg2);


static int init = 0;
static struct kw1281_state state;
static K_MUTEX_DEFINE(state_mutex);

static K_THREAD_DEFINE(keep_alive_tid, 1024, keep_alive_thread, NULL, NULL, NULL, 5, 0, 0);

static struct k_thread async_error_handler_data;
static K_THREAD_STACK_DEFINE(async_error_handler_stack, ASYNC_ERROR_HANDLER_THREAD_STACK_SIZE);

static char __aligned(4) async_error_msgq_buffer[ASYNC_ERROR_MSGQ_LENGTH * sizeof(struct kw1281_async_error)];
static struct k_msgq async_error_msgq;


static const struct kw1281_block ack_block = {
    .length = 3,
    .title = 0x09
};

static const struct kw1281_block end_output_block = {
    .length = 3,
    .title = 0x06
};



static int buf_print_float (char *buf, int buf_len, int precision, float val) {
    int x = 1;
    for (int i=0; i<precision; i++) {
        x *= 10;
    }

    int num = (ABS(val)+.5f) * (val >= 0 ? 1 : -1);
    int fraction = ((int) (ABS(val)*x+.5f)) % x;

    char fmt[16];
    sprintf(fmt, "%%d.%%0%dd", precision);

    return snprintf(buf, buf_len, fmt, num, fraction);
}

static int buf_print_measurement (char *buf, int buf_len, uint8_t id, uint8_t a, uint8_t b) {
    int len = 0;

    uint16_t tmp_unsigned;
    uint32_t tmp_unsigned32;
    int16_t tmp_signed;
    float tmp_float;

    switch (id) {
        case 0x01:
            tmp_float = a*b/5.f;
            len += buf_print_float(buf+len, buf_len-len, 1, tmp_float);
            break;

        case 0x02:
        case 0x03:
            tmp_float = a*b/500.f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x04:
            tmp_float = a*ABS(b-127.f)/100.f;
            len += snprintf(buf+len, buf_len-len, b < 128 ? "BTDC " : "ATDC ");
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x05:
            tmp_float = a*(b-100.f)/10.f;
            len += buf_print_float(buf+len, buf_len-len, 1, tmp_float);
            break;

        case 0x06:
        case 0x0c:
        case 0x15:
        case 0x16:
        case 0x18:
        case 0x2d:
            tmp_float = a*b/1000.f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x07:
        case 0x0f:
        case 0x13:
        case 0x23:
            tmp_float = a*b/100.f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x08:
            tmp_float = a*b/10.f;
            len += buf_print_float(buf+len, buf_len-len, 1, tmp_float);
            break;

        case 0x09:
            tmp_float = a*(b-127.f)/50.f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x0a:
            len += snprintf(buf+len, buf_len-len, b == 0 ? "COLD" : "WARM");
            break;

        case 0x0b:
            tmp_float = a*(b-128.f)/10000.f + 1.f;
            len += buf_print_float(buf+len, buf_len-len, 4, tmp_float);
            break;

        case 0x0d:
            tmp_float = a*(b-127.f)/1000.f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x0e:
            tmp_float = a*b/200.f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x10:
            tmp_unsigned = (a<<8) | b;
            for (int i=0; i<16; i++) {
                len += snprintf(buf+len, buf_len-len, "%s%c", i == 8 ? " " : "", (tmp_unsigned & 0x8000) ? '1' : '0');
                tmp_unsigned <<= 1;
            }
            break;

        case 0x11:
            len += snprintf(buf+len, buf_len-len, "%c%c", (a >= 0x20 && a <= 0x7f) ? a : '?', (b >= 0x20 && b <= 0x7f) ? b : '?');
            break;

        case 0x12:
            tmp_float = a*b/25.f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x14:
            tmp_float = a*(b-128.f)/128.f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x17:
            tmp_float = a*b/256.f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x19:
            tmp_float = (a/182.f) + (b*1.421f);
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x1a:
        case 0x1c:
            tmp_signed = b-a;
            len += snprintf(buf+len, buf_len-len, "%d", tmp_signed);
            break;

        case 0x1b:
            tmp_float = a*ABS(b-128.f)/100.f;
            len += snprintf(buf+len, buf_len-len, b < 128 ? "ATDC " : "BTDC ");
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x1e:
            tmp_float = a*b/12.f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x1f:
            tmp_float = a*b/2560.f;
            len += buf_print_float(buf+len, buf_len-len, 4, tmp_float);
            break;

        case 0x21:
            tmp_float = (a != 0) ? (b*100.f)/a : b*100.f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x22:
            tmp_float = a*(b-128.f)/100.f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x24:
            tmp_unsigned32 = ((a<<8)+b)*10UL;
            len += snprintf(buf+len, buf_len-len, "%d", tmp_unsigned32);
            break;

        case 0x25:
            switch (b) {
                case 0x00:
                    len += snprintf(buf+len, buf_len-len, "-");
                    break;

                case 0x01:
                    len += snprintf(buf+len, buf_len-len, "ADP RUN");
                    break;

                case 0x02:
                    len += snprintf(buf+len, buf_len-len, "ADP OK");
                    break;

                case 0x05:
                    len += snprintf(buf+len, buf_len-len, "Idle");
                    break;

                case 0x06:
                    len += snprintf(buf+len, buf_len-len, "Partial thr");
                    break;

                case 0x07:
                    len += snprintf(buf+len, buf_len-len, "WOT");
                    break;

                case 0x08:
                    len += snprintf(buf+len, buf_len-len, "Enrichment");
                    break;

                case 0x09:
                    len += snprintf(buf+len, buf_len-len, "Deceleration");
                    break;

                case 0x0e:
                    len += snprintf(buf+len, buf_len-len, "A/C low");
                    break;

                case 0x10:
                    len += snprintf(buf+len, buf_len-len, "Compr. OFF");
                    break;


                default:
                    len += snprintf(buf+len, buf_len-len, "unknown state: a=%02x b=%02x", a, b);
            }
            break;

        case 0x26:
            tmp_float = a*(b-128.f)/1000.f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x28:
        case 0x2a:
            tmp_float = a*25.5f+b/10.f - 400.f;
            len += buf_print_float(buf+len, buf_len-len, 1, tmp_float);
            break;

        case 0x29:
        case 0x30:
            tmp_unsigned = a*255+b;
            len += snprintf(buf+len, buf_len-len, "%d", tmp_unsigned);
            break;

        case 0x2b:
            tmp_float = 25.5f*a+b/10.f;
            len += buf_print_float(buf+len, buf_len-len, 1, tmp_float);
            break;

        case 0x2c:
            len += snprintf(buf+len, buf_len-len, "%d:%d", a, b);
            break;

        case 0x2e:
            tmp_float = (a*b-3200.f)*0.0027;
            len += buf_print_float(buf+len, buf_len-len, 4, tmp_float);
            break;

        case 0x2f:
            tmp_signed = a*(b-128);
            len += snprintf(buf+len, buf_len-len, "%d", tmp_signed);
            break;

        case 0x31:
            tmp_float = a*b/40.f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x32:
            tmp_float = (b-128.f)*100.f/(a != 0 ? a : 1);
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x33:
            tmp_float = a*(b-128.f)/255.f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x34:
            tmp_float = a*b/50.f-a;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x35:
            tmp_float = a*0.006f+(b-128.f)*1.4222f;
            len += buf_print_float(buf+len, buf_len-len, 4, tmp_float);
            break;

        case 0x36:
        case 0x38:
            tmp_unsigned = (a<<8)+b;
            len += snprintf(buf+len, buf_len-len, "%d", tmp_unsigned);
            break;

        case 0x37:
            tmp_float = a*b/200.f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x39:
            tmp_unsigned32 = (a<<8)+b+65536UL;
            len += snprintf(buf+len, buf_len-len, "%d", tmp_unsigned32);
            break;

        case 0x3a:
            tmp_float = (b < 128 ? b : 256-b)*1.0225f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x3b:
            tmp_float = ((a<<8)+b)/32768.f;
            len += buf_print_float(buf+len, buf_len-len, 6, tmp_float);
            break;

        case 0x3c:
            tmp_float = ((a<<8)+b)/100.f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x3d:
            tmp_float = (b-128.f)/(a != 0 ? a : 1);
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x3e:
            tmp_float = a*b*0.256f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x40:
            tmp_unsigned = a+b;
            len += snprintf(buf+len, buf_len-len, "%d", tmp_unsigned);
            break;

        case 0x41:
            tmp_float = a*(b-127.f)/100.f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x42:
            tmp_float = a*b/511.12f;
            len += buf_print_float(buf+len, buf_len-len, 3, tmp_float);
            break;

        case 0x43:
            tmp_float = a*640.f+b*2.5f;
            len += buf_print_float(buf+len, buf_len-len, 1, tmp_float);
            break;

        case 0x44:
            tmp_float = ((a<<8)+b)/7.365f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x45:
            tmp_float = ((a<<8)+b)*0.3254f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;

        case 0x46:
            tmp_float = ((a<<8)+b)*0.192f;
            len += buf_print_float(buf+len, buf_len-len, 2, tmp_float);
            break;


        default:
            len += snprintf(buf+len, buf_len-len, "unknown data: id=%02x a=0x%02x b=0x%02x", id, a, b);
    }


    len += snprintf(buf+len, buf_len-len, " [");

    switch (id) {
        case 0x01:
            len += snprintf(buf+len, buf_len-len, "rpm");
            break;

        case 0x02:
        case 0x14:
        case 0x17:
        case 0x21:
            len += snprintf(buf+len, buf_len-len, "%%");
            break;

        case 0x03:
        case 0x04:
        case 0x09:
        case 0x1b:
        case 0x43:
            len += snprintf(buf+len, buf_len-len, "deg");
            break;

        case 0x05:
        case 0x1f:
            len += snprintf(buf+len, buf_len-len, "deg C");
            break;

        case 0x06:
        case 0x15:
        case 0x2b:
        case 0x42:
            len += snprintf(buf+len, buf_len-len, "V");
            break;

        case 0x07:
            len += snprintf(buf+len, buf_len-len, "km/h");
            break;

        case 0x0c:
        case 0x40:
            len += snprintf(buf+len, buf_len-len, "ohm");
            break;

        case 0x0d:
        case 0x41:
            len += snprintf(buf+len, buf_len-len, "mm");
            break;

        case 0x0e:
        case 0x45:
            len += snprintf(buf+len, buf_len-len, "bar");
            break;

        case 0x0f:
        case 0x16:
        case 0x2f:
            len += snprintf(buf+len, buf_len-len, "ms");
            break;

        case 0x10:
            len += snprintf(buf+len, buf_len-len, "binary");
            break;

        case 0x12:
        case 0x32:
            len += snprintf(buf+len, buf_len-len, "mbar");
            break;

        case 0x13:
            len += snprintf(buf+len, buf_len-len, "l");
            break;

        case 0x18:
        case 0x28:
            len += snprintf(buf+len, buf_len-len, "A");
            break;

        case 0x19:
        case 0x35:
            len += snprintf(buf+len, buf_len-len, "g/s");
            break;

        case 0x1a:
            len += snprintf(buf+len, buf_len-len, "C");
            break;

        case 0x22:
            len += snprintf(buf+len, buf_len-len, "kW");
            break;

        case 0x23:
            len += snprintf(buf+len, buf_len-len, "l/h");
            break;

        case 0x24:
            len += snprintf(buf+len, buf_len-len, "km");
            break;

        case 0x25:
            len += snprintf(buf+len, buf_len-len, "state");
            break;

        case 0x1e:
        case 0x26:
        case 0x2e:
            len += snprintf(buf+len, buf_len-len, "deg k/w");
            break;

        case 0x27:
        case 0x31:
        case 0x33:
            len += snprintf(buf+len, buf_len-len, "mg/h");
            break;

        case 0x29:
            len += snprintf(buf+len, buf_len-len, "Ah");
            break;

        case 0x2a:
            len += snprintf(buf+len, buf_len-len, "Kw");
            break;

        case 0x2c:
            len += snprintf(buf+len, buf_len-len, "h:m");
            break;

        case 0x34:
            len += snprintf(buf+len, buf_len-len, "Nm");
            break;

        case 0x36:
            len += snprintf(buf+len, buf_len-len, "count");
            break;

        case 0x37:
            len += snprintf(buf+len, buf_len-len, "s");
            break;

        case 0x38:
        case 0x39:
            len += snprintf(buf+len, buf_len-len, "WSC");
            break;

        case 0x3c:
            len += snprintf(buf+len, buf_len-len, "sec");
            break;

        case 0x3e:
            len += snprintf(buf+len, buf_len-len, "S");
            break;

        case 0x44:
            len += snprintf(buf+len, buf_len-len, "deg/s");
            break;

        case 0x46:
            len += snprintf(buf+len, buf_len-len, "m/s^2");
            break;


        default:
            len += snprintf(buf+len, buf_len-len, "??");
    }

    len += snprintf(buf+len, buf_len-len, "]");


    return len;
}


// read/clear fault codes
static int read_fault_codes (const struct shell *shell, int clear) {
    struct kw1281_block block;

    if (kw1281_is_disconnected(&state)) {
        shell_error(shell, "error: not connected");
        return 1;
    }


    block.length = 3;
    block.title = !clear ? 0x07 : 0x05;

    if (!kw1281_send_block(&state, &block, 0)) {
        shell_error(shell, "error: send_block: %s", kw1281_get_error_msg(&state));
        kw1281_disconnect(&state);

        return 1;
    }

    for (;;) {
        if (!kw1281_receive_block(&state, &block)) {
            shell_error(shell, "error: receive_block: %s", kw1281_get_error_msg(&state));
            kw1281_disconnect(&state);

            return 1;
        }

        if (block.title == 0x09) {
            shell_print(shell, "all data received");
            // ack received
            break;
        }

        if (!kw1281_send_block(&state, &ack_block, 0)) {
            shell_error(shell, "error: send_block: %s", kw1281_get_error_msg(&state));
            kw1281_disconnect(&state);

            return 1;
        }

        if (block.title != 0xFC) {
            shell_error(shell, "error: block with unexpected title received: %02x", block.title);
            kw1281_disconnect(&state);

            return 1;
        }

        uint8_t data_len = block.length-3;
        if (data_len%3 != 0) {
            shell_error(shell, "error: invalid data length: %d", data_len);
            kw1281_disconnect(&state);

            return 1;
        }

        for (int i=0; i<data_len; i+=3) {
            uint16_t id = (((uint16_t) block.data[i])<<8) | block.data[i+1];
            uint8_t unknown = block.data[i+2];
            shell_print(shell, "> %d - %02x", id, unknown);
        }
    }

    return 0;
}

static int read_group (const struct shell *shell, uint8_t channel, uint8_t adapt) {
    struct kw1281_block block;

    block.length = 4;
    block.title = !adapt ? 0x29 : 0x28;
    block.data[0] = channel;

    if (!kw1281_send_block(&state, &block, 0)) {
        shell_error(shell, "error: send_block: %s", kw1281_get_error_msg(&state));
        kw1281_disconnect(&state);

        return 1;
    }

    char buf[64];

    for (;;) {
        if (!kw1281_receive_block(&state, &block)) {
            shell_error(shell, "error: receive_block: %s", kw1281_get_error_msg(&state));
            kw1281_disconnect(&state);

            return 1;
        }

        if (block.title == 0x09) {
            // ack received
            shell_print(shell, "all data received");

            break;
        }

        if (!kw1281_send_block(&state, &ack_block, 0)) {
            shell_error(shell, "error: send_block: %s", kw1281_get_error_msg(&state));
            kw1281_disconnect(&state);

            return 1;
        }

        if (block.title != 0xE7) {
            shell_error(shell, "error: block with unexpected title received: %02x", block.title);
            kw1281_disconnect(&state);

            return 1;
        }

        uint8_t data_len = block.length-3;
        if (data_len%3 != 0) {
            shell_error(shell, "error: invalid data length: %d", data_len);
            kw1281_disconnect(&state);

            return 1;
        }

        for (int i=0; i<data_len; i+=3) {
            uint8_t id = block.data[i];
            uint8_t a = block.data[i+1];
            uint8_t b = block.data[i+2];

            buf_print_measurement(buf, sizeof(buf), id, a, b);
            shell_print(shell, "%s", buf);
        }
    }

    return 0;
}


static int cmd_scan (const struct shell *shell, size_t argc, char **argv) {
    char *end = NULL;
    int start_address;
    int end_address;

    start_address = strtoul(argv[1], &end, 16);

    if (*end != '\0' || end == argv[1] || start_address < 0x00 || start_address > 0xff) {
        shell_error(shell, "error: invalid start address");
        return 1;
    }

    end_address = strtoul(argv[2], &end, 16);

    if (*end != '\0' || end == argv[1] || end_address < 0x00 || end_address > 0xff || end_address < start_address) {
        shell_error(shell, "error: invalid end address");
        return 1;
    }


    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }


    if (!kw1281_is_disconnected(&state)) {
        shell_error(shell, "error: connected");

        k_mutex_unlock(&state_mutex);
        return 1;
    }


    struct kw1281_block block;

    for (unsigned int address=start_address; address<=end_address; address++) {
        k_msleep(100);


        if (!kw1281_connect(&state, address, 1)) {
            continue;
        }

        shell_info(shell, "successfully connected to address %02x", address);

        // 1. controller id
        // 2. component
        // 3. software coding
        // 4. dealer part
        // 5. additional info

        for (;;) {
            if (!kw1281_receive_block(&state, &block)) {
                shell_error(shell, "error: receive_block: %s", kw1281_get_error_msg(&state));
                kw1281_disconnect(&state);

                break;
            }

            if (block.title == 0x09) {
                // ack received
                shell_print(shell, "all data received");

                break;
            }

            if (!kw1281_send_block(&state, &ack_block, 0)) {
                shell_error(shell, "error: send_block: %s", kw1281_get_error_msg(&state));
                kw1281_disconnect(&state);

                break;
            }

            if (block.title != 0xF6) {
                shell_error(shell, "error: block with unexpected title received: %02x", block.title);
                kw1281_disconnect(&state);

                break;
            }

            char s[KW1281_MAX_BLOCK_DATA_LEN+1];
            memcpy(s, block.data, block.length-3);
            s[block.length-3] = 0;

            shell_print(shell, "successfully received block: %s", s);
        }


        // terminate connection

        if (!kw1281_send_block(&state, &end_output_block, 1)) {
            shell_error(shell, "error: send_block: %s", kw1281_get_error_msg(&state));
        }

        kw1281_disconnect(&state);
    }


    k_mutex_unlock(&state_mutex);


    shell_print(shell, "scan done!");

    return 0;
}


static int cmd_connect (const struct shell *shell, size_t argc, char **argv) {
    char *end = NULL;
    int address = strtoul(argv[1], &end, 16);

    if (*end != '\0' || end == argv[1] || address < 0x00 || address > 0xff) {
        shell_error(shell, "error: invalid address");
        return 1;
    }


    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }


    if (!kw1281_is_disconnected(&state)) {
        shell_error(shell, "error: already connected");

        k_mutex_unlock(&state_mutex);
        return 1;
    }


    struct kw1281_block block;

    if (!kw1281_connect(&state, address, 1)) {
        shell_error(shell, "error: connect: %s", kw1281_get_error_msg(&state));
        kw1281_disconnect(&state);

        k_mutex_unlock(&state_mutex);
        return 1;
    }

    // 1. controller id
    // 2. component
    // 3. software coding
    // 4. dealer part
    // 5. additional info

    for (;;) {
        if (!kw1281_receive_block(&state, &block)) {
            shell_error(shell, "error: receive_block: %s", kw1281_get_error_msg(&state));
            kw1281_disconnect(&state);

            k_mutex_unlock(&state_mutex);
            return 1;
        }

        if (block.title == 0x09) {
            // ack received
            shell_print(shell, "all data received");

            break;
        }

        if (!kw1281_send_block(&state, &ack_block, 0)) {
            shell_error(shell, "error: send_block: %s", kw1281_get_error_msg(&state));
            kw1281_disconnect(&state);

            k_mutex_unlock(&state_mutex);
            return 1;
        }

        if (block.title != 0xF6) {
            shell_error(shell, "error: block with unexpected title received: %02x", block.title);
            kw1281_disconnect(&state);

            k_mutex_unlock(&state_mutex);
            return 1;
        }

        char s[KW1281_MAX_BLOCK_DATA_LEN+1];
        memcpy(s, block.data, block.length-3);
        s[block.length-3] = 0;

        shell_print(shell, "successfully received block: %s", s);
    }


    k_mutex_unlock(&state_mutex);


    shell_info(shell, "connected!");

    return 0;
}


static int cmd_read_fault_codes (const struct shell *shell, size_t argc, char **argv) {
    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }

    int ret = read_fault_codes(shell, 0);

    k_mutex_unlock(&state_mutex);

    return ret;
}


static int cmd_clear_fault_codes (const struct shell *shell, size_t argc, char **argv) {
    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }

    int ret = read_fault_codes(shell, 1);

    k_mutex_unlock(&state_mutex);

    return ret;
}


static int cmd_read_group (const struct shell *shell, size_t argc, char **argv) {
    char *end = NULL;
    int channel = strtoul(argv[1], &end, 16);

    if (*end != '\0' || end == argv[1] || channel < 0x00 || channel > 0xff) {
        shell_error(shell, "error: invalid channel");
        return 1;
    }


    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }

    int ret = read_group(shell, channel, 0);

    k_mutex_unlock(&state_mutex);

    return ret;
}


static int cmd_basic_setting (const struct shell *shell, size_t argc, char **argv) {
    char *end = NULL;
    int channel = strtoul(argv[1], &end, 16);

    if (*end != '\0' || end == argv[1] || channel < 0x00 || channel > 0xff) {
        shell_error(shell, "error: invalid channel");
        return 1;
    }


    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }

    int ret = read_group(shell, channel, 1);

    k_mutex_unlock(&state_mutex);

    return ret;
}


static int cmd_send_raw_block (const struct shell *shell, size_t argc, char **argv) {
    struct kw1281_block block;


    char *end = NULL;
    int title = strtoul(argv[1], &end, 16);

    if (*end != '\0' || end == argv[1] || title < 0x00 || title > 0xff) {
        shell_error(shell, "error: invalid title");
        return 1;
    }

    block.title = title;


    uint8_t data_len = argc-2;
    block.length = 3 + data_len;

    for (int i=0; i<data_len; i++) {
        char *data_str = argv[i+2];
        int data = strtoul(data_str, &end, 16);
        if (*end != '\0' || end == data_str || data < 0x00 || data > 0xff) {
            shell_error(shell, "error: invalid data (byte #%d)", i);
            return 1;
        }

        block.data[i] = data;
    }


    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }


    if (!kw1281_send_block(&state, &block, 0)) {
        shell_error(shell, "error: send_block: %s", kw1281_get_error_msg(&state));
        kw1281_disconnect(&state);

        k_mutex_unlock(&state_mutex);
        return 1;
    }


    struct kw1281_block recv_block;
    char buf[1024];
    char *buf_ptr = NULL;

    for (;;) {
        if (!kw1281_receive_block(&state, &recv_block)) {
            shell_error(shell, "error: receive_block: %s", kw1281_get_error_msg(&state));
            kw1281_disconnect(&state);

            k_mutex_unlock(&state_mutex);
            return 1;
        }

        if (recv_block.title == 0x09) {
            // ack received
            shell_print(shell, "all data received");

            break;
        }

        if (!kw1281_send_block(&state, &ack_block, 0)) {
            shell_error(shell, "error: send_block: %s", kw1281_get_error_msg(&state));
            kw1281_disconnect(&state);

            k_mutex_unlock(&state_mutex);
            return 1;
        }


        buf_ptr = buf;

        buf_ptr += sprintf(buf_ptr, "data: %02x %02x:", recv_block.length, recv_block.title);
        for (int i=0; i<(recv_block.length-3); i++) {
            buf_ptr += sprintf(buf_ptr, " %02x", recv_block.data[i]);
        }

        shell_print(shell, "%s", buf);
    }


    k_mutex_unlock(&state_mutex);

    return 0;
}


static int cmd_terminate (const struct shell *shell, size_t argc, char **argv) {
    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }


    if (kw1281_is_disconnected(&state)) {
        shell_warn(shell, "warning: not connected");
    }


    if (!kw1281_send_block(&state, &end_output_block, 1)) {
        shell_error(shell, "error: send_block: %s", kw1281_get_error_msg(&state));
        kw1281_disconnect(&state);

        k_mutex_unlock(&state_mutex);
        return 1;
    }

    kw1281_disconnect(&state);


    k_mutex_unlock(&state_mutex);


    shell_info(shell, "connection terminated!");

    return 0;
}


static int cmd_baudrate (const struct shell *shell, size_t argc, char **argv) {
    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }


    char *end = NULL;
    uint16_t baudrate = 0;

    if (argc == 2) {
        baudrate = strtoul(argv[1], &end, 10);

        if (*end != '\0' || end == argv[1]) {
            shell_error(shell, "error: invalid baudrate");
            return 1;
        }
    }

    if (baudrate > 0) {
        if (!kw1281_set_baudrate(&state, baudrate)) {
            shell_error(shell, "error: set_baudrate: %s", kw1281_get_error_msg(&state));

            k_mutex_unlock(&state_mutex);
            return 1;
        }
    }

    if (!kw1281_get_baudrate(&state, &baudrate)) {
        shell_error(shell, "error: get_baudrate: %s", kw1281_get_error_msg(&state));

        k_mutex_unlock(&state_mutex);
        return 1;
    }


    k_mutex_unlock(&state_mutex);


    shell_print(shell, "baudrate: %d", baudrate);

    return 0;
}


static int cmd_inter_byte_time (const struct shell *shell, size_t argc, char **argv) {
    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }


    char *end = NULL;
    uint8_t inter_byte_time_ms = 0;

    if (argc == 2) {
        inter_byte_time_ms = strtoul(argv[1], &end, 10);

        if (*end != '\0' || end == argv[1]) {
            shell_error(shell, "error: invalid inter byte time");
            return 1;
        }
    }

    if (inter_byte_time_ms > 0) {
        if (!kw1281_set_inter_byte_time_ms(&state, inter_byte_time_ms)) {
            shell_error(shell, "error: set_inter_byte_time_ms: %s", kw1281_get_error_msg(&state));

            k_mutex_unlock(&state_mutex);
            return 1;
        }
    }

    if (!kw1281_get_inter_byte_time_ms(&state, &inter_byte_time_ms)) {
        shell_error(shell, "error: get_inter_byte_time_ms: %s", kw1281_get_error_msg(&state));

        k_mutex_unlock(&state_mutex);
        return 1;
    }


    k_mutex_unlock(&state_mutex);


    shell_print(shell, "inter byte time (ms): %d", inter_byte_time_ms);

    return 0;
}


static int cmd_inter_block_time (const struct shell *shell, size_t argc, char **argv) {
    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }


    char *end = NULL;
    uint16_t inter_block_time_ms = 0;

    if (argc == 2) {
        inter_block_time_ms = strtoul(argv[1], &end, 10);

        if (*end != '\0' || end == argv[1]) {
            shell_error(shell, "error: invalid inter block time");
            return 1;
        }
    }

    if (inter_block_time_ms > 0) {
        if (!kw1281_set_inter_block_time_ms(&state, inter_block_time_ms)) {
            shell_error(shell, "error: set_inter_block_time_ms: %s", kw1281_get_error_msg(&state));

            k_mutex_unlock(&state_mutex);
            return 1;
        }
    }

    if (!kw1281_get_inter_block_time_ms(&state, &inter_block_time_ms)) {
        shell_error(shell, "error: get_inter_block_time_ms: %s", kw1281_get_error_msg(&state));

        k_mutex_unlock(&state_mutex);
        return 1;
    }


    k_mutex_unlock(&state_mutex);


    shell_print(shell, "inter block time (ms): %d", inter_block_time_ms);

    return 0;
}


static int cmd_key_word_ack_delay (const struct shell *shell, size_t argc, char **argv) {
    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }


    char *end = NULL;
    uint16_t key_word_ack_delay_ms = 0;

    if (argc == 2) {
        key_word_ack_delay_ms = strtoul(argv[1], &end, 10);

        if (*end != '\0' || end == argv[1]) {
            shell_error(shell, "error: invalid key word ack delay");
            return 1;
        }
    }

    if (key_word_ack_delay_ms > 0) {
        if (!kw1281_set_key_word_ack_delay_ms(&state, key_word_ack_delay_ms)) {
            shell_error(shell, "error: set_key_word_ack_delay_ms: %s", kw1281_get_error_msg(&state));

            k_mutex_unlock(&state_mutex);
            return 1;
        }
    }

    if (!kw1281_get_key_word_ack_delay_ms(&state, &key_word_ack_delay_ms)) {
        shell_error(shell, "error: get_key_word_ack_delay_ms: %s", kw1281_get_error_msg(&state));

        k_mutex_unlock(&state_mutex);
        return 1;
    }


    k_mutex_unlock(&state_mutex);


    shell_print(shell, "key word ack delay (ms): %d", key_word_ack_delay_ms);

    return 0;
}


static int cmd_timeout_multiplier (const struct shell *shell, size_t argc, char **argv) {
    if (k_mutex_lock(&state_mutex, K_MSEC(STATE_MUTEX_LOCK_TIMEOUT_MS)) != 0) {
        shell_error(shell, "error: cannot lock state");
        return 1;
    }


    char *end = NULL;
    uint16_t timeout_multiplier = 0;

    if (argc == 2) {
        timeout_multiplier = strtoul(argv[1], &end, 10);

        if (*end != '\0' || end == argv[1]) {
            shell_error(shell, "error: invalid timeout multiplier");
            return 1;
        }
    }

    if (timeout_multiplier > 0) {
        if (!kw1281_set_timeout_multiplier(&state, timeout_multiplier)) {
            shell_error(shell, "error: set_timeout_multiplier: %s", kw1281_get_error_msg(&state));

            k_mutex_unlock(&state_mutex);
            return 1;
        }
    }

    if (!kw1281_get_timeout_multiplier(&state, &timeout_multiplier)) {
        shell_error(shell, "error: get_timeout_multiplier: %s", kw1281_get_error_msg(&state));

        k_mutex_unlock(&state_mutex);
        return 1;
    }


    k_mutex_unlock(&state_mutex);


    shell_print(shell, "timeout multiplier (100x): %d", timeout_multiplier);

    return 0;
}


static void keep_alive_thread (void *arg0, void *arg1, void *arg2) {
    struct kw1281_block block;

    while (!init) k_msleep(10);

    k_mutex_lock(&state_mutex, K_FOREVER);

    for (;;) {
        k_mutex_unlock(&state_mutex);
        k_msleep(KEEP_ALIVE_INTERVAL_MS);
        k_mutex_lock(&state_mutex, K_FOREVER);

        if (!kw1281_is_disconnected(&state)) {
            if (!kw1281_send_block(&state, &ack_block, 0)) {
                shell_error(shell_backend_uart_get_ptr(), "error: send_block: %s", kw1281_get_error_msg(&state));
                kw1281_disconnect(&state);

                continue;
            }

            if (!kw1281_receive_block(&state, &block)) {
                shell_error(shell_backend_uart_get_ptr(), "error: receive_block: %s", kw1281_get_error_msg(&state));
                kw1281_disconnect(&state);

                continue;
            }

            if (block.title != 0x09) {
                shell_error(shell_backend_uart_get_ptr(), "error: block with unexpected title received: %02x", block.title);
                kw1281_disconnect(&state);

                continue;
            }
        }
    }
}

static void async_error_handler_thread (void *arg0, void *arg1, void *arg2) {
    struct kw1281_async_error async_error;

    for (;;) {
        k_msgq_get(&async_error_msgq, &async_error, K_FOREVER);
        shell_error(shell_backend_uart_get_ptr(), "async error: %s", kw1281_get_async_error_type_msg(async_error.type));
    }
}


// state_mutex may be locked here so we must not try to lock it
// may be called from an ISR context
static void async_error_callback (const struct kw1281_state *state, struct kw1281_async_error *async_error) {
    int ret;

    ret = k_msgq_put(&async_error_msgq, async_error, K_NO_WAIT);
    if (ret != 0) {
        k_oops();
    }
}



void main (void) {
    const struct device *dev = device_get_binding(CONFIG_UART_SHELL_ON_DEV_NAME);
    uint32_t dtr = 0;

    if (usb_enable(NULL)) {
        return;
    }

    while (!dtr) {
        uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
    }



    k_msgq_init(&async_error_msgq, async_error_msgq_buffer, sizeof(struct kw1281_async_error), ASYNC_ERROR_MSGQ_LENGTH);

    k_thread_create(&async_error_handler_data, async_error_handler_stack, ASYNC_ERROR_HANDLER_THREAD_STACK_SIZE,
            async_error_handler_thread, NULL, NULL, NULL, ASYNC_ERROR_HANDLER_THREAD_PRIO, 0, K_NO_WAIT);


    struct kw1281_config config = {
        .baudrate = 10400,
        .inter_byte_time_ms = 10,
        .inter_block_time_ms = 30,
        .key_word_ack_delay_ms = 30,
        .timeout_multiplier = 100,          // 1.00

        .timeout_connect_ms = 3000,
        .timeout_receive_block_ongoing_tx_ms = 1000,
        .timeout_receive_block_rx_start_ms = 1500,
        .timeout_receive_block_rx_ms = 1000,
        .timeout_transmit_block_ongoing_tx_ms = 1000,
        .timeout_transmit_block_tx_ms = 1000,

        .uart_dev = device_get_binding(DT_LABEL(DT_NODELABEL(usart2))),
        .slow_init_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpioa))),
        .slow_init_pin = 1,

        .status_led_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpioc))),
        .status_led_pin = 13,
        .status_led_active_low = 1,
        .status_led_timeout_ms = 100,

        .async_error_callback = &async_error_callback,
    };

    kw1281_init(&state, &config);


    init = 1;
}



SHELL_CMD_ARG_REGISTER(scan, NULL, "Scan for targets", cmd_scan, 3, 0);
SHELL_CMD_ARG_REGISTER(connect, NULL, "Connect to target", cmd_connect, 2, 0);
SHELL_CMD_ARG_REGISTER(read_fault_codes, NULL, "Read fault codes", cmd_read_fault_codes, 1, 0);
SHELL_CMD_ARG_REGISTER(clear_fault_codes, NULL, "Clear fault codes", cmd_clear_fault_codes, 1, 0);
SHELL_CMD_ARG_REGISTER(read_group, NULL, "Read group without adaption", cmd_read_group, 2, 0);
SHELL_CMD_ARG_REGISTER(basic_setting, NULL, "Read group with adaption", cmd_basic_setting, 2, 0);
SHELL_CMD_ARG_REGISTER(send_raw_block, NULL, "Send raw block", cmd_send_raw_block, 2, KW1281_MAX_BLOCK_DATA_LEN);
SHELL_CMD_ARG_REGISTER(terminate, NULL, "Terminate connection", cmd_terminate, 1, 0);
SHELL_CMD_ARG_REGISTER(baudrate, NULL, "Get/Set baudrate", cmd_baudrate, 1, 1);
SHELL_CMD_ARG_REGISTER(inter_byte_time, NULL, "Get/Set inter byte time", cmd_inter_byte_time, 1, 1);
SHELL_CMD_ARG_REGISTER(inter_block_time, NULL, "Get/Set inter block time", cmd_inter_block_time, 1, 1);
SHELL_CMD_ARG_REGISTER(key_word_ack_delay, NULL, "Get/Set key word ack delay", cmd_key_word_ack_delay, 1, 1);
SHELL_CMD_ARG_REGISTER(timeout_multiplier, NULL, "Get/Set timeout multiplier", cmd_timeout_multiplier, 1, 1);

