#pragma once

#include <Arduino.h>
#include "protocol.h"
#include "frameBuilder.h"
#include "deviceConfig.h"

// Forward declaration for software PWM control
extern void setSoftwarePWM(uint16_t freq, uint8_t duty_percent);

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

    // Handle GPIO command (v3: check for peripheral-reserved)
    static void handleGPIO(const uint8_t* payload, uint8_t len) {
        if (len < 2) {
            sendACK(CMD_GPIO, ACK_INVALID_PARAM, "Invalid payload");
            return;
        }

        uint8_t pin = payload[0];
        uint8_t state = payload[1];

        // Validate pin and check for peripheral-reserved (v3)
        bool pin_valid = false;
        bool is_peripheral_reserved = false;
        for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
            if (outputs[i].pin_number == pin) {
                if (outputs[i].capabilities & OUTPUT_CAP_PERIPHERAL) {
                    is_peripheral_reserved = true;
                    break;
                }
                if (outputs[i].capabilities & OUTPUT_CAP_GPIO) {
                    pin_valid = true;
                    break;
                }
            }
        }

        if (is_peripheral_reserved) {
            char error_msg[64];
            snprintf(error_msg, sizeof(error_msg), "Pin %d is peripheral-reserved", pin);
            sendACK(CMD_GPIO, ACK_UNSUPPORTED, error_msg);
            return;
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

    // Handle PWM command (v3: check for peripheral-reserved)
    static void handlePWM(const uint8_t* payload, uint8_t len) {
        if (len < 4) {
            sendACK(CMD_PWM, ACK_INVALID_PARAM, "Invalid payload");
            return;
        }

        uint8_t pin = payload[0];
        uint16_t frequency_hz = readUint16LE(&payload[1]);
        uint8_t duty_percent = payload[3];

        // Validate pin and check for peripheral-reserved (v3)
        bool pin_valid = false;
        bool is_peripheral_reserved = false;
        for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
            if (outputs[i].pin_number == pin) {
                if (outputs[i].capabilities & OUTPUT_CAP_PERIPHERAL) {
                    is_peripheral_reserved = true;
                    break;
                }
                if (outputs[i].capabilities & OUTPUT_CAP_PWM) {
                    pin_valid = true;
                    break;
                }
            }
        }

        if (is_peripheral_reserved) {
            sendACK(CMD_PWM, ACK_UNSUPPORTED, "Pin is peripheral-reserved");
            return;
        }

        if (!pin_valid) {
            sendACK(CMD_PWM, ACK_INVALID_PARAM, "Invalid pin or no PWM capability");
            return;
        }

        if (duty_percent > 100) {
            sendACK(CMD_PWM, ACK_INVALID_PARAM, "Duty cycle must be 0-100%");
            return;
        }

        // Update software PWM state (hardware update handled in main loop based on switch state)
        setSoftwarePWM(frequency_hz, duty_percent);

        sendACK(CMD_PWM, ACK_SUCCESS);
    }

    // Handle DAC command (v3: uses pin number, not channel; checks peripheral-reserved)
    static void handleDAC(const uint8_t* payload, uint8_t len) {
        if (len < 3) {
            sendACK(CMD_DAC, ACK_INVALID_PARAM, "Invalid payload");
            return;
        }

        uint8_t pin = payload[0];
        uint16_t value_12bit = readUint16LE(&payload[1]);

        // Validate pin and check for peripheral-reserved (v3)
        bool pin_valid = false;
        bool is_peripheral_reserved = false;
        for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
            if (outputs[i].pin_number == pin) {
                if (outputs[i].capabilities & OUTPUT_CAP_PERIPHERAL) {
                    is_peripheral_reserved = true;
                    break;
                }
                if (outputs[i].capabilities & OUTPUT_CAP_DAC) {
                    pin_valid = true;
                    break;
                }
            }
        }

        if (is_peripheral_reserved) {
            sendACK(CMD_DAC, ACK_UNSUPPORTED, "Pin is peripheral-reserved");
            return;
        }

        if (!pin_valid) {
            sendACK(CMD_DAC, ACK_INVALID_PARAM, "Invalid pin or no DAC capability");
            return;
        }

        if (value_12bit > 4095) {
            sendACK(CMD_DAC, ACK_INVALID_PARAM, "Value must be 0-4095");
            return;
        }

        // ESP32 DAC is 8-bit (0-255), scale down from 12-bit
        uint8_t dac_value = value_12bit >> 4;  // Divide by 16

        // ESP32 has 2 DAC pins: GPIO25 (DAC1), GPIO26 (DAC2)
        if (pin == 25 || pin == 26) {
            dacWrite(pin, dac_value);
            sendACK(CMD_DAC, ACK_SUCCESS);
        } else {
            sendACK(CMD_DAC, ACK_INVALID_PARAM, "Pin does not support DAC");
        }
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
