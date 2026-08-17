/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_XLH100_SERIAL_ENABLED

#include "AP_RangeFinder_XLH100_Serial.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>

#if defined(HAL_BUILD_AP_PERIPH)
extern "C" {
void can_printf(const char *fmt, ...);
}
#endif

#define XLH100_HEADER             0xA5    // frame header byte, sent twice
#define XLH100_CMD_DET_OUT        0x82    // detection target output command
#define XLH100_OP_READ            0x00    // read operation code

// startup command: header(2) + cmd(128) + write(0x01) + length(0) + checksum
// checksum = (0xA5 + 0xA5 + 0x80 + 0x01 + 0x00 + 0x00) & 0xFF = 0xCB
static const uint8_t startup_command[7] = { 0xA5, 0xA5, 0x80, 0x01, 0x00, 0x00, 0xCB };

#define STARTUP_CMD_INTERVAL_MS   5000    // gap between startup command retries
#define NO_DATA_RESET_MS          10000   // silence before resending startup command
#define DIAG_INTERVAL_MS          5000    // gap between link diagnostics reports
#define TARGET_SIZE_BYTES         8       // X(2) + Y(2) + velocity(2) + SNR(2)
#define MAX_PAYLOAD_LEN           (2 + 100 * TARGET_SIZE_BYTES)  // cycle number + up to 100 targets
#define OUT_OF_RANGE_ADD_M        1.0f    // metres added to max distance when reporting out of range

bool AP_RangeFinder_XLH100_Serial::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // the radar may power up later than this node or may be power-cycled in
    // shutdown state; keep sending the startup command at a low rate until
    // data arrives. each power command is stored to the radar's flash, so
    // the retry interval is kept well above the radar's frame period
    if (!got_first_frame) {
        if (now_ms - last_startup_cmd_ms > STARTUP_CMD_INTERVAL_MS) {
            uart->write(startup_command, sizeof(startup_command));
            last_startup_cmd_ms = now_ms;
        }
    } else if (now_ms - last_frame_ms > NO_DATA_RESET_MS) {
        // radar was talking but went silent, try waking it again
        got_first_frame = false;
        last_startup_cmd_ms = now_ms - STARTUP_CMD_INTERVAL_MS - 1;  // send immediately
    }

    // parse all bytes available, the latest complete frame wins
    bool got_frame = false;
    bool latest_has_target = false;
    uint16_t latest_min_y_cm = 0;

    while (uart->available() > 0) {
        const int16_t b = uart->read();
        if (b < 0) {
            break;
        }
        rx_bytes++;
        last_bytes[last_bytes_idx & 7] = b;
        last_bytes_idx++;
        parse_byte(b);
        if (frame_complete) {
            frame_complete = false;
            got_frame = true;
            got_first_frame = true;
            last_frame_ms = now_ms;
            latest_has_target = frame_has_target;
            latest_min_y_cm = frame_min_y_cm;
            frame_has_target = false;
        }
    }

    // report link diagnostics over CAN so serial activity can be verified remotely
#if defined(HAL_BUILD_AP_PERIPH)
    if (now_ms - last_diag_ms >= DIAG_INTERVAL_MS) {
        if (last_diag_ms > 0) {
            const uint32_t dt_ms = now_ms - last_diag_ms;
            const uint32_t rate = (rx_bytes - last_diag_rx_bytes) * 1000U / dt_ms;
            char bhex[26];
            for (uint8_t i = 0; i < 8; i++) {
                snprintf(&bhex[i*3], 4, "%02X ", last_bytes[(last_bytes_idx + i) & 7]);
            }
            bhex[24] = 0;
            can_printf("XLH100 rx=%lu %lu/s sync=%lu crc=%lu ok=%lu st=%d b:%s",
                       (unsigned long)rx_bytes, (unsigned long)rate,
                       (unsigned long)sync_count, (unsigned long)crc_errors,
                       (unsigned long)frames_ok, int(parse_state), bhex);
        }
        last_diag_ms = now_ms;
        last_diag_rx_bytes = rx_bytes;
    }
#endif

    if (!got_frame) {
        return false;
    }

    if (latest_has_target) {
        // radar faces down: nearest echo (lowest Y) is the altitude
        reading_m = latest_min_y_cm * 0.01f;
        no_signal = false;
    } else {
        // valid frame without targets: report out-of-range-high
        reading_m = max_distance() + OUT_OF_RANGE_ADD_M;
        no_signal = true;
    }

    return true;
}

void AP_RangeFinder_XLH100_Serial::parse_byte(const uint8_t b)
{
    switch (parse_state) {

    case ParseState::WAIT_HEADER1:
        if (b == XLH100_HEADER) {
            checksum = b;
            parse_state = ParseState::WAIT_HEADER2;
        }
        break;

    case ParseState::WAIT_HEADER2:
        if (b == XLH100_HEADER) {
            checksum += b;
            parse_state = ParseState::WAIT_CMD;
        } else {
            parse_state = ParseState::WAIT_HEADER1;
        }
        break;

    case ParseState::WAIT_CMD:
        if (b == XLH100_CMD_DET_OUT) {
            checksum += b;
            parse_state = ParseState::WAIT_OP;
        } else {
            // not a detection frame, resynchronise
            parse_state = ParseState::WAIT_HEADER1;
        }
        break;

    case ParseState::WAIT_OP:
        if (b == XLH100_OP_READ) {
            checksum += b;
            sync_count++;
            parse_state = ParseState::WAIT_LEN_LO;
        } else {
            parse_state = ParseState::WAIT_HEADER1;
        }
        break;

    case ParseState::WAIT_LEN_LO:
        payload_len = b;
        checksum += b;
        parse_state = ParseState::WAIT_LEN_HI;
        break;

    case ParseState::WAIT_LEN_HI:
        payload_len |= b << 8;  // little endian
        checksum += b;
        if (payload_len > MAX_PAYLOAD_LEN) {
            // implausible length, resynchronise
            parse_state = ParseState::WAIT_HEADER1;
        } else {
            payload_index = 0;
            frame_min_y_cm = 0;
            frame_has_target = false;
            target_y_valid = false;
            parse_state = ParseState::PAYLOAD;
        }
        break;

    case ParseState::PAYLOAD:
        checksum += b;
        if (payload_index >= 2) {
            // payload starts with the 2 byte cycle number, then targets:
            // each target is X(i16) Y(u16) velocity(i16) SNR(u16), all little endian
            switch ((payload_index - 2) % TARGET_SIZE_BYTES) {
            case 2:
                target_y_lo = b;
                break;
            case 3:
                // Y coordinate, little endian, cm
                target_y_cm = (uint16_t(b) << 8) | target_y_lo;
                target_y_valid = true;
                break;
            case 6:
                target_snr_lo = b;
                break;
            case TARGET_SIZE_BYTES - 1:
                // complete target: track the lowest Y of the frame.
                // unused target slots are zero padded by the radar, their
                // SNR is zero; only real echoes (SNR > 0) are considered
                target_snr = (uint16_t(b) << 8) | target_snr_lo;
                if (target_y_valid && target_snr > 0) {
                    if (!frame_has_target || target_y_cm < frame_min_y_cm) {
                        frame_min_y_cm = target_y_cm;
                    }
                    frame_has_target = true;
                    target_y_valid = false;
                }
                break;
            default:
                // X and velocity bytes are not used
                break;
            }
        }
        payload_index++;
        if (payload_index >= payload_len) {
            parse_state = ParseState::WAIT_CHECKSUM;
        }
        break;

    case ParseState::WAIT_CHECKSUM:
        // checksum covers all bytes from the frame header to the end of the payload
        if ((checksum & 0xFF) == b) {
            got_first_frame = true;
            frame_complete = true;
            frames_ok++;
        } else {
            crc_errors++;
        }
        parse_state = ParseState::WAIT_HEADER1;
        break;
    }
}

#endif  // AP_RANGEFINDER_XLH100_SERIAL_ENABLED
