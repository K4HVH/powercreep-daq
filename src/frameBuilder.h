#pragma once

#include <Arduino.h>
#include "protocol.h"

// Maximum frame size (sync + len + type + payload + crc)
constexpr size_t MAX_FRAME_SIZE = 512;

class FrameBuilder {
public:
    FrameBuilder() : payload_len_(0) {}

    // Start building a new frame
    void begin(uint8_t frame_type) {
        frame_type_ = frame_type;
        payload_len_ = 0;
    }

    // Add a single byte to payload
    void addByte(uint8_t byte) {
        if (payload_len_ < MAX_PAYLOAD_SIZE) {
            payload_[payload_len_++] = byte;
        }
    }

    // Add uint16_t (little-endian) to payload
    void addUint16(uint16_t value) {
        if (payload_len_ + 2 <= MAX_PAYLOAD_SIZE) {
            writeUint16LE(&payload_[payload_len_], value);
            payload_len_ += 2;
        }
    }

    // Add uint32_t (little-endian) to payload
    void addUint32(uint32_t value) {
        if (payload_len_ + 4 <= MAX_PAYLOAD_SIZE) {
            writeUint32LE(&payload_[payload_len_], value);
            payload_len_ += 4;
        }
    }

    // Add a string to payload (length-prefixed)
    void addString(const char* str) {
        uint8_t len = strlen(str);
        if (payload_len_ + 1 + len <= MAX_PAYLOAD_SIZE) {
            payload_[payload_len_++] = len;
            memcpy(&payload_[payload_len_], str, len);
            payload_len_ += len;
        }
    }

    // Add raw bytes to payload
    void addBytes(const uint8_t* data, size_t len) {
        if (payload_len_ + len <= MAX_PAYLOAD_SIZE) {
            memcpy(&payload_[payload_len_], data, len);
            payload_len_ += len;
        }
    }

    // Helper to get payload length (for debug)
    uint8_t getPayloadLength() const {
        return payload_len_;
    }

    // Helper to copy payload (for debug)
    void copyPayloadTo(uint8_t* dest) const {
        memcpy(dest, payload_, payload_len_);
    }

    // Send the frame over Serial (optimized for single write)
    void send() {
        // Frame structure: [SYNC1][SYNC2][LEN_LOW][LEN_HIGH][TYPE][PAYLOAD][CRC16]
        // LEN is 2-byte little-endian payload size (does NOT include TYPE byte)
        uint16_t len = (uint16_t)payload_len_;

        // Build entire frame in buffer for single write
        uint8_t frame_buffer[MAX_FRAME_SIZE];
        size_t frame_idx = 0;

        // Sync bytes
        frame_buffer[frame_idx++] = SYNC1;
        frame_buffer[frame_idx++] = SYNC2;

        // Length (2-byte little-endian)
        frame_buffer[frame_idx++] = (uint8_t)(len & 0xFF);
        frame_buffer[frame_idx++] = (uint8_t)((len >> 8) & 0xFF);

        // Type
        frame_buffer[frame_idx++] = frame_type_;

        // Payload
        memcpy(&frame_buffer[frame_idx], payload_, payload_len_);
        frame_idx += payload_len_;

        // Calculate CRC over [LEN_LOW][LEN_HIGH][TYPE][PAYLOAD]
        // CRC starts at offset 2 (after sync bytes)
        uint16_t crc = crc16_ccitt(&frame_buffer[2], 3 + payload_len_);

        // CRC (2-byte little-endian)
        frame_buffer[frame_idx++] = (uint8_t)(crc & 0xFF);
        frame_buffer[frame_idx++] = (uint8_t)((crc >> 8) & 0xFF);

        // Single write for entire frame (much faster than multiple writes)
        Serial.write(frame_buffer, frame_idx);
    }

private:
    static constexpr size_t MAX_PAYLOAD_SIZE = MAX_FRAME_SIZE - 7;  // Minus sync(2), len(2), type(1), crc(2)
    uint8_t frame_type_;
    uint8_t payload_[MAX_PAYLOAD_SIZE];
    size_t payload_len_;
};
