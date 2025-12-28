#pragma once

#include <Arduino.h>
#include <driver/adc.h>
#include "protocol.h"

// Device ID from ESP32 MAC address
uint32_t getDeviceID() {
    uint64_t mac = ESP.getEfuseMac();
    // Use lower 32 bits of MAC as device ID
    return (uint32_t)(mac & 0xFFFFFFFF);
}

// Pin mode options for digital inputs
enum PinMode : uint8_t {
    PIN_MODE_INPUT = 0,         // No pull resistor (floating)
    PIN_MODE_INPUT_PULLUP = 1,  // Internal pull-up resistor (~45kΩ)
    PIN_MODE_INPUT_PULLDOWN = 2 // Internal pull-down resistor (~45kΩ)
};

// ADC attenuation options (ESP32 specific)
enum AdcAttenuation : uint8_t {
    ADC_ATTEN_0DB = 0,    // 0dB attenuation (100mV ~ 950mV)
    ADC_ATTEN_2_5DB = 1,  // 2.5dB attenuation (100mV ~ 1250mV)
    ADC_ATTEN_6DB = 2,    // 6dB attenuation (150mV ~ 1750mV)
    ADC_ATTEN_11DB = 3    // 11dB attenuation (150mV ~ 2450mV) - default
};

// Channel configuration
struct ChannelConfig {
    uint8_t channel_id;
    const char* name;
    const char* unit;
    uint16_t default_sample_rate_hz;
    bool is_analog;
    uint8_t gpio_pin;  // GPIO pin number, 255 for simulated
    bool enabled;

    // Pin configuration (compile-time settings)
    PinMode pin_mode;              // For digital inputs: pull-up/pull-down/none
    AdcAttenuation adc_attenuation; // For analog inputs: voltage range
};

// Output pin configuration
struct OutputConfig {
    uint8_t pin_number;
    const char* name;
    uint8_t capabilities;  // Bitfield: GPIO|PWM|DAC
};

/*
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                      CHANNEL CONFIGURATION GUIDE                          ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 *
 * Format: {id, name, unit, sample_rate, is_analog, gpio_pin, enabled, pin_mode, adc_attenuation}
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ PARAMETERS                                                               │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ id                  : Channel ID (0-31), must be unique                 │
 * │ name                : Channel name (appears in desktop UI)              │
 * │ unit                : Unit string (e.g., "V", "A", "°C")                │
 * │ sample_rate         : Default sample rate in Hz                         │
 * │ is_analog           : true = ADC input, false = digital GPIO input      │
 * │ gpio_pin            : GPIO pin number (255 = simulated/no pin)          │
 * │ enabled             : true = channel active, false = disabled           │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ PIN CONFIGURATION (compile-time, not runtime)                           │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ pin_mode            : For DIGITAL inputs only                           │
 * │                       • PIN_MODE_INPUT          : No pull resistor      │
 * │                       • PIN_MODE_INPUT_PULLUP   : Pull-up ~45kΩ ✓      │
 * │                       • PIN_MODE_INPUT_PULLDOWN : Pull-down ~45kΩ      │
 * │                                                                          │
 * │ adc_attenuation     : For ANALOG inputs only (ESP32 voltage range)      │
 * │                       • ADC_ATTEN_0DB    : 100mV ~ 950mV                │
 * │                       • ADC_ATTEN_2_5DB  : 100mV ~ 1250mV               │
 * │                       • ADC_ATTEN_6DB    : 150mV ~ 1750mV               │
 * │                       • ADC_ATTEN_11DB   : 150mV ~ 2450mV ✓ (default)   │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ ESP32 ADC1 PINS (recommended for reliability - no WiFi conflict)        │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ GPIO32 (ADC1_CH4)  GPIO33 (ADC1_CH5)  GPIO34 (ADC1_CH6)                 │
 * │ GPIO35 (ADC1_CH7)  GPIO36 (ADC1_CH0)  GPIO39 (ADC1_CH3)                 │
 * │                                                                          │
 * │ ⚠ ADC2 pins conflict with WiFi - avoid GPIO 0,2,4,12-15,25-27           │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ COMMON USE CASES                                                        │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ • Button/Switch Input     : PIN_MODE_INPUT_PULLUP (normally HIGH)       │
 * │ • Active-Low Sensor       : PIN_MODE_INPUT_PULLUP                       │
 * │ • Active-High Sensor      : PIN_MODE_INPUT_PULLDOWN                     │
 * │ • Voltage Measurement     : ADC_ATTEN_11DB (0-2.45V)                    │
 * │ • Low-Voltage Sensor      : ADC_ATTEN_0DB (100-950mV)                   │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * NOTES:
 * • Digital inputs default to INPUT_PULLUP to prevent floating states
 * • Analog inputs default to 11dB attenuation for widest voltage range
 * • Pin configuration is compile-time only (cannot change at runtime)
 * • Use voltage dividers if measuring >2.45V on analog inputs
 */

// 16 Channels: 6 real ADC + 5 simulated + 5 digital inputs
ChannelConfig channels[] = {
    // Real ADC channels (ESP32 ADC1: GPIO32-39)
    // 11dB attenuation provides 0-2.45V range (suitable for 3.3V logic with voltage divider)
    {0, "ADC0_GPIO32", "V", 500, true, 32, true, PIN_MODE_INPUT, ADC_ATTEN_11DB},   // Changed to 500Hz
    {1, "ADC1_GPIO33", "V", 100, true, 33, true, PIN_MODE_INPUT, ADC_ATTEN_11DB},   // Changed to 100Hz
    {2, "ADC2_GPIO34", "V", 1000, true, 34, true, PIN_MODE_INPUT, ADC_ATTEN_11DB},
    {3, "ADC3_GPIO35", "V", 1000, true, 35, true, PIN_MODE_INPUT, ADC_ATTEN_11DB},
    {4, "ADC4_GPIO36", "V", 1000, true, 36, true, PIN_MODE_INPUT, ADC_ATTEN_11DB},
    {5, "ADC5_GPIO39", "V", 1000, true, 39, true, PIN_MODE_INPUT, ADC_ATTEN_11DB},

    // Simulated channels (sine waves, square waves, ramps)
    {6,  "SIM_Sine_1Hz", "V", 1000, true, 255, true, PIN_MODE_INPUT, ADC_ATTEN_11DB},
    {7,  "SIM_Sine_10Hz", "V", 1000, true, 255, true, PIN_MODE_INPUT, ADC_ATTEN_11DB},
    {8,  "SIM_Square_5Hz", "V", 1000, true, 255, true, PIN_MODE_INPUT, ADC_ATTEN_11DB},
    {9,  "SIM_Ramp_2Hz", "V", 1000, true, 255, true, PIN_MODE_INPUT, ADC_ATTEN_11DB},
    {10, "SIM_Noise", "V", 1000, true, 255, true, PIN_MODE_INPUT, ADC_ATTEN_11DB},

    // Digital input channels with pull-up resistors (default for floating inputs)
    {11, "DIN_GPIO12", "", 1000, false, 12, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB},
    {12, "DIN_GPIO13", "", 1000, false, 13, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB},
    {13, "DIN_GPIO14", "", 1000, false, 14, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB},
    {14, "DIN_GPIO15", "", 1000, false, 15, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB},
    {15, "DIN_GPIO27", "", 1000, false, 27, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB},
};

constexpr uint8_t NUM_CHANNELS = sizeof(channels) / sizeof(channels[0]);

// Output pins with different capabilities
OutputConfig outputs[] = {
    // GPIO-only outputs
    {2,  "LED_Builtin", OUTPUT_CAP_GPIO},
    {16, "GPIO16", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM},
    {17, "GPIO17", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM},

    // PWM-capable outputs (LEDC)
    {18, "PWM_GPIO18", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM},
    {19, "PWM_GPIO19", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM},
    {21, "PWM_GPIO21", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM},
    {22, "PWM_GPIO22", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM},
    {23, "PWM_GPIO23", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM},

    // DAC outputs (ESP32 has 2 DAC pins)
    {25, "DAC1_GPIO25", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM | OUTPUT_CAP_DAC},
    {26, "DAC2_GPIO26", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM | OUTPUT_CAP_DAC},
};

constexpr uint8_t NUM_OUTPUTS = sizeof(outputs) / sizeof(outputs[0]);

// Initialize GPIO pins for digital inputs with configured pull resistors
void initializeDigitalInputs() {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (!channels[i].is_analog && channels[i].gpio_pin != 255) {
            // Apply configured pin mode (pull-up, pull-down, or none)
            switch (channels[i].pin_mode) {
                case PIN_MODE_INPUT_PULLUP:
                    pinMode(channels[i].gpio_pin, INPUT_PULLUP);
                    break;
                case PIN_MODE_INPUT_PULLDOWN:
                    pinMode(channels[i].gpio_pin, INPUT_PULLDOWN);
                    break;
                case PIN_MODE_INPUT:
                default:
                    pinMode(channels[i].gpio_pin, INPUT);
                    break;
            }
        }
    }
}

// Initialize ADC channels with configured attenuation
void initializeADC() {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (channels[i].is_analog && channels[i].gpio_pin != 255) {
            // Map our attenuation enum to ESP32 ADC attenuation
            adc_atten_t esp_atten;
            switch (channels[i].adc_attenuation) {
                case ADC_ATTEN_0DB:
                    esp_atten = ADC_ATTEN_DB_0;
                    break;
                case ADC_ATTEN_2_5DB:
                    esp_atten = ADC_ATTEN_DB_2_5;
                    break;
                case ADC_ATTEN_6DB:
                    esp_atten = ADC_ATTEN_DB_6;
                    break;
                case ADC_ATTEN_11DB:
                default:
                    esp_atten = ADC_ATTEN_DB_12;
                    break;
            }

            // Set attenuation for this pin
            // Note: ESP32 requires pin-to-channel mapping
            adc1_channel_t adc_channel;
            switch (channels[i].gpio_pin) {
                case 32: adc_channel = ADC1_CHANNEL_4; break;
                case 33: adc_channel = ADC1_CHANNEL_5; break;
                case 34: adc_channel = ADC1_CHANNEL_6; break;
                case 35: adc_channel = ADC1_CHANNEL_7; break;
                case 36: adc_channel = ADC1_CHANNEL_0; break;
                case 39: adc_channel = ADC1_CHANNEL_3; break;
                default: continue; // Skip if not a valid ADC1 pin
            }

            adc1_config_channel_atten(adc_channel, esp_atten);
        }
    }
}

// Initialize output pins
void initializeOutputs() {
    for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
        pinMode(outputs[i].pin_number, OUTPUT);
        digitalWrite(outputs[i].pin_number, LOW);  // Default to LOW
    }
}
