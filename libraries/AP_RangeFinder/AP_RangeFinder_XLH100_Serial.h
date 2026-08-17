#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_XLH100_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/*
Driver for the XL-H100 77GHz mmWave radar (NJXL protocol) on TTL serial

The radar sends target frames in this format:

| header | header | cmd  | op   | length (LE) | payload                        | checksum |
| 0xA5   | 0xA5   | 0x82 | 0x00 | 2 bytes     | cycle(2B) + N * target(8B)     | 1 byte   |

checksum is the low 8 bits of the sum of all bytes from the frame header
to the end of the payload.

Each target (version without target ID) is:
  X coordinate    : int16, little endian, cm
  Y coordinate    : uint16, little endian, cm
  velocity        : int16, little endian, cm/s
  SNR amplitude   : uint16, little endian

The radar zero-pads unused target slots in each frame; padded targets
have an SNR of zero and are ignored.

When used for altitude measurement with the radar facing down, the
lowest Y value of the frame (nearest echo) is reported as the altitude.
A frame without targets is reported as out-of-range-high.

The radar powers up in the shutdown state by default, so a startup
command is sent until the first valid frame is received. Each
power-cycle command is stored to the radar's flash, so retries are
rate-limited and stop as soon as the radar outputs data.
*/
class AP_RangeFinder_XLH100_Serial : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_XLH100_Serial(_state, _params);
    }

protected:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    uint16_t rx_bufsize() const override { return 256; }
    uint16_t tx_bufsize() const override { return 32; }

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    int8_t get_signal_quality_pct() const override
    {
        return no_signal ? RangeFinder::SIGNAL_QUALITY_MIN : RangeFinder::SIGNAL_QUALITY_UNKNOWN;
    }

private:

    // get a reading
    bool get_reading(float &reading_m) override;

    // feed a single byte to the frame parser
    void parse_byte(uint8_t b);

    // frame parser state machine
    enum class ParseState : uint8_t {
        WAIT_HEADER1,   // waiting for first 0xA5
        WAIT_HEADER2,   // waiting for second 0xA5
        WAIT_CMD,       // waiting for 0x82 (detection target output)
        WAIT_OP,        // waiting for 0x00 (read operation)
        WAIT_LEN_LO,    // payload length low byte
        WAIT_LEN_HI,    // payload length high byte
        PAYLOAD,        // cycle number + target points
        WAIT_CHECKSUM,  // frame checksum
    };

    ParseState parse_state = ParseState::WAIT_HEADER1;

    uint16_t checksum;          // running sum of all frame bytes
    uint16_t payload_len;       // payload length of the frame being parsed
    uint16_t payload_index;     // bytes of payload consumed so far
    uint16_t frame_min_y_cm;    // lowest Y of complete targets in this frame
    uint16_t target_y_cm;       // Y of the target being assembled
    uint16_t target_snr;        // SNR of the target being assembled
    uint8_t target_y_lo;        // low byte of the target Y being assembled
    uint8_t target_snr_lo;      // low byte of the target SNR being assembled
    bool target_y_valid;        // target Y high byte received
    bool frame_has_target;      // this frame contained at least one complete target
    bool frame_complete;        // a complete frame with valid checksum was received

    // startup command handling
    uint32_t last_startup_cmd_ms;
    uint32_t last_frame_ms = 0;     // time of last valid frame from the radar
    bool got_first_frame = false;

    // link diagnostics, reported over CAN every few seconds
    uint32_t rx_bytes = 0;          // total bytes received from the radar
    uint32_t sync_count = 0;        // valid frame headers seen
    uint32_t crc_errors = 0;        // frames failing checksum
    uint32_t frames_ok = 0;         // frames passing checksum
    uint32_t last_diag_ms = 0;      // time of last diagnostics report
    uint32_t last_diag_rx_bytes = 0;// rx_bytes at last diagnostics report
    uint8_t last_bytes[8];          // rolling buffer of recent rx bytes
    uint8_t last_bytes_idx = 0;     // total bytes stored modulo 8

    bool no_signal;     // true if the latest read attempt found no valid distances
};

#endif  // AP_RANGEFINDER_XLH100_SERIAL_ENABLED
