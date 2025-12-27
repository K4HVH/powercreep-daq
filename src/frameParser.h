#pragma once

#include <Arduino.h>
#include "protocol.h"

// Callback for successfully parsed frames
typedef void (*FrameCallback)(uint8_t frame_type, const uint8_t* payload, uint8_t payload_len);

class FrameParser {
public:
    FrameParser() : state_(WAIT_SYNC1), callback_(nullptr) {}

    void setCallback(FrameCallback callback) {
        callback_ = callback;
    }

    // Feed incoming bytes to the parser
    void feedByte(uint8_t byte) {
        switch (state_) {
            case WAIT_SYNC1:
                if (byte == SYNC1) {
                    state_ = WAIT_SYNC2;
                }
                break;

            case WAIT_SYNC2:
                if (byte == SYNC2) {
                    state_ = READ_LEN_LOW;
                } else {
                    state_ = WAIT_SYNC1;  // Sync lost
                }
                break;

            case READ_LEN_LOW:
                len_low_ = byte;
                state_ = READ_LEN_HIGH;
                break;

            case READ_LEN_HIGH:
                len_high_ = byte;
                len_ = (uint16_t)len_low_ | ((uint16_t)len_high_ << 8);
                if (len_ > MAX_PAYLOAD_SIZE) {
                    // Invalid length (LEN is payload size, not including TYPE)
                    state_ = WAIT_SYNC1;
                } else {
                    state_ = READ_TYPE;
                }
                break;

            case READ_TYPE:
                type_ = byte;
                payload_len_ = len_;  // LEN is payload size (does NOT include TYPE)
                payload_index_ = 0;
                if (payload_len_ == 0) {
                    state_ = READ_CRC;
                    crc_index_ = 0;
                } else {
                    state_ = READ_PAYLOAD;
                }
                break;

            case READ_PAYLOAD:
                payload_[payload_index_++] = byte;
                if (payload_index_ >= payload_len_) {
                    state_ = READ_CRC;
                    crc_index_ = 0;
                }
                break;

            case READ_CRC:
                crc_bytes_[crc_index_++] = byte;
                if (crc_index_ >= 2) {
                    // Got full CRC, validate frame
                    validateAndDispatch();
                    state_ = WAIT_SYNC1;
                }
                break;
        }
    }

    // Process all available Serial data
    void processSerial() {
        while (Serial.available() > 0) {
            feedByte(Serial.read());
        }
    }

private:
    enum State {
        WAIT_SYNC1,
        WAIT_SYNC2,
        READ_LEN_LOW,
        READ_LEN_HIGH,
        READ_TYPE,
        READ_PAYLOAD,
        READ_CRC
    };

    static constexpr size_t MAX_PAYLOAD_SIZE = 512;

    State state_;
    uint8_t len_low_;
    uint8_t len_high_;
    uint16_t len_;
    uint8_t type_;
    uint8_t payload_[MAX_PAYLOAD_SIZE];
    uint16_t payload_len_;
    uint16_t payload_index_;
    uint8_t crc_bytes_[2];
    uint8_t crc_index_;
    FrameCallback callback_;

    void validateAndDispatch() {
        // Calculate CRC on [LEN_LOW][LEN_HIGH][TYPE][PAYLOAD]
        uint8_t crc_data[MAX_PAYLOAD_SIZE + 3];
        crc_data[0] = len_low_;
        crc_data[1] = len_high_;
        crc_data[2] = type_;
        memcpy(&crc_data[3], payload_, payload_len_);

        // CRC size = 3 (LEN_LOW + LEN_HIGH + TYPE) + payload_len
        uint16_t calculated_crc = crc16_ccitt(crc_data, 3 + payload_len_);
        uint16_t received_crc = readUint16LE(crc_bytes_);

        if (calculated_crc == received_crc) {
            // CRC valid - dispatch frame
            if (callback_) {
                callback_(type_, payload_, payload_len_);
            }
        }
        // CRC error - silently drop frame
    }
};
