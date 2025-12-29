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

// Channel configuration (Protocol v3)
struct ChannelConfig {
    uint8_t channel_id;
    const char* name;
    const char* unit;
    uint16_t default_sample_rate_hz;
    AcquisitionMethod acquisition_method;  // v3: GPIO, ADC, HX711, SPI ADC, I2C ADC, UART
    DataType data_type;                   // v3: 8-bit unsigned/signed, 16-bit unsigned/signed, 24-bit signed, 32-bit unsigned/signed/float
    uint8_t gpio_pin;  // Primary GPIO pin number
    bool enabled;

    // Pin configuration (compile-time settings)
    PinMode pin_mode;              // For digital inputs: pull-up/pull-down/none
    AdcAttenuation adc_attenuation; // For analog inputs: voltage range

    // Peripheral-specific pins (v3: for multi-pin acquisition methods)
    // These pins should be defined in outputs[] with OUTPUT_CAP_PERIPHERAL
    uint8_t peripheral_pin1;  // HX711: SCK, SPI: CS, I2C: SDA, UART: RX
    uint8_t peripheral_pin2;  // SPI: MOSI, I2C: SCL, UART: TX
    uint8_t peripheral_pin3;  // SPI: MISO
    uint8_t peripheral_pin4;  // SPI: SCK
};

// Output pin configuration
struct OutputConfig {
    uint8_t pin_number;
    const char* name;
    uint8_t capabilities;  // Bitfield: GPIO|PWM|DAC
};

/*
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                  CHANNEL CONFIGURATION GUIDE (Protocol v3)                ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 *
 * Format: {id, name, unit, sample_rate, acquisition_method, data_type, gpio_pin, enabled, pin_mode, adc_attenuation}
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ PARAMETERS                                                               │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ id                  : Channel ID (0-255), must be unique                │
 * │ name                : Channel name (appears in desktop UI)              │
 * │ unit                : Unit string (e.g., "V", "A", "°C", "N")           │
 * │ sample_rate         : Default sample rate in Hz                         │
 * │ acquisition_method  : v3 acquisition type (see below)                   │
 * │ data_type           : v3 sample value encoding (see below)              │
 * │ gpio_pin            : GPIO pin number                                    │
 * │ enabled             : true = channel active, false = disabled           │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ ACQUISITION_METHOD (v3)                                                 │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ ACQ_GPIO      : Digital GPIO input (0/1)                                │
 * │ ACQ_ADC       : Internal ADC (ESP32 12-bit: 0-4095)                     │
 * │ ACQ_HX711     : HX711 24-bit load cell ADC                              │
 * │ ACQ_SPI_ADC   : External SPI ADC (e.g., MCP3008, ADS1118)               │
 * │ ACQ_I2C_ADC   : External I2C ADC/sensor (e.g., ADS1115, BMP280)         │
 * │ ACQ_UART      : UART/Serial sensor (e.g., GPS, Modbus)                  │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ DATA_TYPE (v3) - Sample Value Encoding                                 │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ DATA_UINT8   : 8-bit unsigned (1 byte, 0-255)                          │
 * │ DATA_INT8    : 8-bit signed (1 byte, -128 to 127)                      │
 * │ DATA_UINT16  : 16-bit unsigned (2 bytes, 0-65535) ✓ ESP32 ADC          │
 * │ DATA_INT16   : 16-bit signed (2 bytes, -32768 to 32767)                │
 * │ DATA_INT24   : 24-bit signed (3 bytes, ±8.3M) ✓ HX711                  │
 * │ DATA_UINT32  : 32-bit unsigned (4 bytes, 0-4.2B)                       │
 * │ DATA_INT32   : 32-bit signed (4 bytes, ±2.1B)                          │
 * │ DATA_FLOAT32 : 32-bit IEEE 754 float (4 bytes) ✓ Calibrated sensors    │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ PIN CONFIGURATION (compile-time, not runtime)                           │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ pin_mode            : For ACQ_GPIO only                                 │
 * │                       • PIN_MODE_INPUT          : No pull resistor      │
 * │                       • PIN_MODE_INPUT_PULLUP   : Pull-up ~45kΩ ✓      │
 * │                       • PIN_MODE_INPUT_PULLDOWN : Pull-down ~45kΩ      │
 * │                                                                          │
 * │ adc_attenuation     : For ACQ_ADC only (ESP32 voltage range)            │
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
 * │ RECOMMENDED CONFIGURATIONS                                              │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ • Digital Switch/Button   : ACQ_GPIO + DATA_UINT8                       │
 * │ • ESP32 Internal ADC      : ACQ_ADC + DATA_UINT16 (12-bit: 0-4095)      │
 * │ • HX711 Load Cell         : ACQ_HX711 + DATA_INT24                      │
 * │ • ADS1115 16-bit ADC      : ACQ_I2C_ADC + DATA_INT16                    │
 * │ • BMP280 Temperature      : ACQ_I2C_ADC + DATA_FLOAT32 (calibrated °C)  │
 * │ • Encoder Counter         : ACQ_GPIO + DATA_UINT32 (large counts)       │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * NOTES:
 * • Protocol v3 supports variable-length samples based on data_type (1-4 bytes)
 * • Choose smallest data_type that fits your range for optimal bandwidth
 * • acquisition_method is immutable (cannot change at runtime)
 * • Digital inputs default to INPUT_PULLUP to prevent floating states
 * • Use voltage dividers if measuring >2.45V on analog inputs
 */

// 11 Channels: 6 real ADC + 5 digital inputs (v3 format)
// Format: {id, name, unit, sample_rate, acquisition_method, data_type, gpio_pin, enabled, pin_mode, adc_attenuation, peripheral_pin1-4}
// Note: Use 255 for unused peripheral pins
ChannelConfig channels[] = {
    // Real ADC channels (ESP32 ADC1: GPIO32-39)
    // Using DATA_UINT16 for ESP32 12-bit ADC (0-4095 range)
    {0, "ADC0_GPIO32", "V", 1000, ACQ_ADC, DATA_UINT16, 32, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 255, 255, 255, 255},
    {1, "ADC1_GPIO33", "V", 1000, ACQ_ADC, DATA_UINT16, 33, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 255, 255, 255, 255},
    {2, "ADC2_GPIO34", "V", 1000, ACQ_ADC, DATA_UINT16, 34, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 255, 255, 255, 255},
    {3, "ADC3_GPIO35", "V", 1000, ACQ_ADC, DATA_UINT16, 35, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 255, 255, 255, 255},
    {4, "ADC4_GPIO36", "V", 1000, ACQ_ADC, DATA_UINT16, 36, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 255, 255, 255, 255},
    {5, "ADC5_GPIO39", "V", 1000, ACQ_ADC, DATA_UINT16, 39, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 255, 255, 255, 255},

    // Digital input channels with pull-up resistors
    // Using DATA_UINT8 for digital inputs (0=LOW, 1=HIGH)
    {6, "DIN_GPIO12", "", 1000, ACQ_GPIO, DATA_UINT8, 12, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB, 255, 255, 255, 255},
    {7, "DIN_GPIO13", "", 1000, ACQ_GPIO, DATA_UINT8, 13, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB, 255, 255, 255, 255},
    {8, "DIN_GPIO14", "", 1000, ACQ_GPIO, DATA_UINT8, 14, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB, 255, 255, 255, 255},
    {9, "DIN_GPIO15", "", 1000, ACQ_GPIO, DATA_UINT8, 15, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB, 255, 255, 255, 255},
    {10, "DIN_GPIO27", "", 1000, ACQ_GPIO, DATA_UINT8, 27, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB, 255, 255, 255, 255},
};

constexpr uint8_t NUM_CHANNELS = sizeof(channels) / sizeof(channels[0]);

// Output pins with different capabilities
OutputConfig outputs[] = {
    // GPIO-only outputs
    {2,  "LED_Builtin", OUTPUT_CAP_GPIO},

    // Peripheral-reserved pins (v3: firmware-controlled, cannot be controlled by user via GPIO/PWM/DAC)
    // I2C (default ESP32 pins)
    {21, "I2C_SDA", OUTPUT_CAP_PERIPHERAL},
    {22, "I2C_SCL", OUTPUT_CAP_PERIPHERAL},

    // SPI (default VSPI pins)
    {5,  "SPI_CS", OUTPUT_CAP_PERIPHERAL},
    {18, "SPI_SCK", OUTPUT_CAP_PERIPHERAL},
    {19, "SPI_MISO", OUTPUT_CAP_PERIPHERAL},
    {23, "SPI_MOSI", OUTPUT_CAP_PERIPHERAL},

    // UART (Serial2 default pins)
    {16, "UART2_RX", OUTPUT_CAP_PERIPHERAL},
    {17, "UART2_TX", OUTPUT_CAP_PERIPHERAL},

    // User-controllable PWM outputs
    {4,  "PWM_GPIO4", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM},

    // DAC outputs (ESP32 has 2 DAC pins)
    {25, "DAC1_GPIO25", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM | OUTPUT_CAP_DAC},
    {26, "DAC2_GPIO26", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM | OUTPUT_CAP_DAC},
};

constexpr uint8_t NUM_OUTPUTS = sizeof(outputs) / sizeof(outputs[0]);

// Initialize GPIO pins for digital inputs with configured pull resistors
void initializeDigitalInputs() {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (channels[i].acquisition_method == ACQ_GPIO) {
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
        if (channels[i].acquisition_method == ACQ_ADC) {
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

            adc1_channel_t adc_channel;
            switch (channels[i].gpio_pin) {
                case 32: adc_channel = ADC1_CHANNEL_4; break;
                case 33: adc_channel = ADC1_CHANNEL_5; break;
                case 34: adc_channel = ADC1_CHANNEL_6; break;
                case 35: adc_channel = ADC1_CHANNEL_7; break;
                case 36: adc_channel = ADC1_CHANNEL_0; break;
                case 39: adc_channel = ADC1_CHANNEL_3; break;
                default: continue;
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
