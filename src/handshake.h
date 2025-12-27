#pragma once

#include <Arduino.h>
#include "protocol.h"
#include "frameBuilder.h"
#include "deviceConfig.h"

class Handshake {
public:
    // Send HANDSHAKE_RESP with device metadata
    static void sendHandshakeResponse() {
        FrameBuilder frame;
        frame.begin(HANDSHAKE_RESP);

        // Protocol version
        frame.addByte(PROTOCOL_VERSION);

        // Device ID (from MAC address)
        frame.addUint32(getDeviceID());

        // Number of channels
        frame.addByte(NUM_CHANNELS);

        // Channel metadata
        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
            frame.addByte(channels[i].channel_id);

            // Flags
            uint8_t flags = channels[i].is_analog ? CHANNEL_FLAG_ANALOG : 0;
            frame.addByte(flags);

            // Default sample rate
            frame.addUint16(channels[i].default_sample_rate_hz);

            // Name and unit (length-prefixed strings)
            frame.addString(channels[i].name);
            frame.addString(channels[i].unit);
        }

        // Number of output pins
        frame.addByte(NUM_OUTPUTS);

        // Output pin metadata
        for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
            frame.addByte(outputs[i].pin_number);
            frame.addByte(outputs[i].capabilities);
            frame.addString(outputs[i].name);
        }

        frame.send();
    }
};

class Heartbeat {
public:
    Heartbeat() : last_heartbeat_ms_(0) {}

    void begin() {
        last_heartbeat_ms_ = millis();
    }

    // Check if heartbeat needs to be sent (every 1 second)
    void process() {
        unsigned long now = millis();
        if (now - last_heartbeat_ms_ >= 1000) {
            sendHeartbeat();
            last_heartbeat_ms_ = now;
        }
    }

    // Handle HEARTBEAT_ACK from host
    void handleAck() {
        // Desktop acknowledged heartbeat - connection is alive
        // Reset timeout timer (if implementing timeout detection)
        last_heartbeat_ms_ = millis();
    }

private:
    unsigned long last_heartbeat_ms_;

    void sendHeartbeat() {
        FrameBuilder frame;
        frame.begin(HEARTBEAT);
        // Heartbeat has no payload
        frame.send();
    }
};
