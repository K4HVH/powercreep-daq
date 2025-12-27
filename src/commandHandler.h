#pragma once

#include <Arduino.h>
#include "protocol.h"
#include "frameBuilder.h"
#include "deviceConfig.h"

class CommandHandler {
public:
    // Send ACK response
    static void sendACK(uint8_t command_type, uint8_t status, const char* message = "") {
        FrameBuilder frame;
        frame.begin(ACK);
        frame.addByte(command_type);
        frame.addByte(status);
        frame.addString(message);
        frame.send();
    }

    // Send ERROR frame
    static void sendError(uint8_t error_code, const char* message) {
        FrameBuilder frame;
        frame.begin(ERROR_FRAME);
        frame.addByte(error_code);
        frame.addString(message);
        frame.send();
    }

    // Handle GPIO command
    static void handleGPIO(const uint8_t* payload, uint8_t len) {
        if (len < 2) {
            sendACK(CMD_GPIO, ACK_INVALID_PARAM, "Invalid payload");
            return;
        }

        uint8_t pin = payload[0];
        uint8_t state = payload[1];

        // Validate pin
        bool pin_valid = false;
        for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
            if (outputs[i].pin_number == pin && (outputs[i].capabilities & OUTPUT_CAP_GPIO)) {
                pin_valid = true;
                break;
            }
        }

        if (!pin_valid) {
            char error_msg[64];
            snprintf(error_msg, sizeof(error_msg), "Invalid pin %d or no GPIO capability", pin);
            sendACK(CMD_GPIO, ACK_INVALID_PARAM, error_msg);
            return;
        }

        // Set GPIO
        pinMode(pin, OUTPUT);
        digitalWrite(pin, state ? HIGH : LOW);
        sendACK(CMD_GPIO, ACK_SUCCESS);
    }

    // Handle PWM command
    static void handlePWM(const uint8_t* payload, uint8_t len) {
        if (len < 4) {
            sendACK(CMD_PWM, ACK_INVALID_PARAM, "Invalid payload");
            return;
        }

        uint8_t pin = payload[0];
        uint16_t frequency_hz = readUint16LE(&payload[1]);
        uint8_t duty_percent = payload[3];

        // Validate pin
        bool pin_valid = false;
        for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
            if (outputs[i].pin_number == pin && (outputs[i].capabilities & OUTPUT_CAP_PWM)) {
                pin_valid = true;
                break;
            }
        }

        if (!pin_valid) {
            sendACK(CMD_PWM, ACK_INVALID_PARAM, "Invalid pin or no PWM capability");
            return;
        }

        if (duty_percent > 100) {
            sendACK(CMD_PWM, ACK_INVALID_PARAM, "Duty cycle must be 0-100%");
            return;
        }

        // Setup PWM using LEDC (ESP32)
        // Use channel 0-15 (map pin to channel for simplicity)
        uint8_t ledc_channel = pin % 16;
        uint8_t ledc_resolution = 10;  // 10-bit resolution (0-1023)

        ledcSetup(ledc_channel, frequency_hz, ledc_resolution);
        ledcAttachPin(pin, ledc_channel);

        // Convert duty percent to duty value
        uint32_t duty_value = (duty_percent * 1023) / 100;
        ledcWrite(ledc_channel, duty_value);

        sendACK(CMD_PWM, ACK_SUCCESS);
    }

    // Handle DAC command
    static void handleDAC(const uint8_t* payload, uint8_t len) {
        if (len < 3) {
            sendACK(CMD_DAC, ACK_INVALID_PARAM, "Invalid payload");
            return;
        }

        uint8_t channel = payload[0];
        uint16_t value_12bit = readUint16LE(&payload[1]);

        // ESP32 has 2 DAC channels: DAC1 (GPIO25), DAC2 (GPIO26)
        if (channel > 1) {
            sendACK(CMD_DAC, ACK_INVALID_PARAM, "Invalid DAC channel (0-1)");
            return;
        }

        if (value_12bit > 4095) {
            sendACK(CMD_DAC, ACK_INVALID_PARAM, "Value must be 0-4095");
            return;
        }

        // ESP32 DAC is 8-bit (0-255), scale down from 12-bit
        uint8_t dac_value = value_12bit >> 4;  // Divide by 16

        // Write to DAC
        if (channel == 0) {
            dacWrite(25, dac_value);  // DAC1 on GPIO25
        } else {
            dacWrite(26, dac_value);  // DAC2 on GPIO26
        }

        sendACK(CMD_DAC, ACK_SUCCESS);
    }

    // Handle CONFIG command
    static void handleConfig(const uint8_t* payload, uint8_t len) {
        if (len < 1) {
            sendACK(CMD_CONFIG, ACK_INVALID_PARAM, "Invalid payload");
            return;
        }

        uint8_t config_id = payload[0];

        switch (config_id) {
            case CONFIG_ENABLE_CHANNEL:
                if (len < 3) {
                    sendACK(CMD_CONFIG, ACK_INVALID_PARAM, "Invalid payload");
                    return;
                }
                {
                    uint8_t channel_id = payload[1];
                    uint8_t enabled = payload[2];

                    // Find and update channel
                    bool found = false;
                    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
                        if (channels[i].channel_id == channel_id) {
                            channels[i].enabled = (enabled != 0);
                            found = true;
                            break;
                        }
                    }

                    if (!found) {
                        sendACK(CMD_CONFIG, ACK_INVALID_PARAM, "Invalid channel ID");
                    } else {
                        sendACK(CMD_CONFIG, ACK_SUCCESS);
                    }
                }
                break;

            case CONFIG_RESET_DEVICE:
                sendACK(CMD_CONFIG, ACK_SUCCESS);
                delay(100);  // Let ACK send
                ESP.restart();
                break;

            case CONFIG_REQUEST_HANDSHAKE:
                // Desktop requesting handshake (usually after reset)
                sendACK(CMD_CONFIG, ACK_SUCCESS);
                // Handshake will be sent when next HANDSHAKE_REQ is received
                break;

            default:
                sendACK(CMD_CONFIG, ACK_UNSUPPORTED, "Unknown config ID");
                break;
        }
    }
};
