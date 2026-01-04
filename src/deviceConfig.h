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

// Forward declare SensorType from acquisition.h
enum SensorType : uint8_t;

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

    // Sensor type for complex acquis methods (HX711, I2C, SPI sensors)
    uint8_t sensor_type;  // SensorType enum value
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

// 11 Channels: 1 ADC (pot) + 3 digital (switches) + 1 HX711 + 4 I2C sensors + 1 SPI sensor + 1 PCNT sensor (v3 format)
// Format: {id, name, unit, sample_rate, acquisition_method, data_type, gpio_pin, enabled, pin_mode, adc_attenuation, peripheral_pin1-4, sensor_type}
// Note: Use 255 for unused peripheral pins, 0 for SENSOR_NONE
ChannelConfig channels[] = {
    // B10K Potentiometer (0-3.3V on ADC)
    // Using DATA_UINT16 for ESP32 12-bit ADC (0-4095 range)
    {0, "Pot_B10K", "", 100, ACQ_ADC, DATA_UINT16, 32, true, PIN_MODE_INPUT, ADC_ATTEN_0DB, 255, 255, 255, 255, 0},

    // 2-way switch (GPIO14, pull-up resistor, 0=pressed/LOW, 1=open/HIGH)
    // Using DATA_UINT8 for digital inputs (0=LOW, 1=HIGH)
    {1, "Switch_2Way", "", 100, ACQ_GPIO, DATA_UINT8, 14, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB, 255, 255, 255, 255, 0},

    // 3-way switch (2 GPIO pins to detect 3 positions)
    // Position 1: A=LOW,  B=LOW   (00)
    // Position 2: A=HIGH, B=LOW   (10)
    // Position 3: A=LOW,  B=HIGH  (01)
    {2, "Switch_3Way_A", "", 100, ACQ_GPIO, DATA_UINT8, 15, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB, 255, 255, 255, 255, 0},
    {3, "Switch_3Way_B", "", 100, ACQ_GPIO, DATA_UINT8, 27, true, PIN_MODE_INPUT_PULLUP, ADC_ATTEN_11DB, 255, 255, 255, 255, 0},

    // HX711 Load Cell (DOUT=GPIO12, SCK=GPIO13)
    // Using DATA_INT24 for 24-bit signed load cell values
    {4, "LoadCell", "g", 10, ACQ_HX711, DATA_INT24, 12, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 13, 255, 255, 255, 1},

    // AHT10 Temperature & Humidity (I2C, address 0x38, SDA=GPIO21, SCL=GPIO22)
    // Using DATA_FLOAT32 for calibrated values (temp in °C, humidity in %)
    // Sample rate: 10Hz (sensor has 75ms measurement time)
    // Polled in background task to avoid blocking DAQ task with I2C timeouts
    {5, "AHT10_Temp", "C", 10, ACQ_I2C_ADC, DATA_FLOAT32, 21, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 21, 22, 255, 255, 10},
    {6, "AHT10_Humidity", "%", 10, ACQ_I2C_ADC, DATA_FLOAT32, 21, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 21, 22, 255, 255, 10},

    // BMP280 Pressure & Temperature (I2C, address 0x76, SDA=GPIO21, SCL=GPIO22)
    // Using DATA_FLOAT32 for calibrated values (temp in °C, pressure in Pa)
    // Sample rate: 50Hz (sensor has ~10ms measurement time at lowest oversampling)
    // Polled in background task to avoid blocking DAQ task with I2C timeouts
    {7, "BMP280_Temp", "C", 50, ACQ_I2C_ADC, DATA_FLOAT32, 21, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 21, 22, 255, 255, 11},
    {8, "BMP280_Pressure", "Pa", 50, ACQ_I2C_ADC, DATA_FLOAT32, 21, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 21, 22, 255, 255, 11},

    // MAX6675 Thermocouple (SPI, CS=GPIO5, SCK=GPIO18, MISO=GPIO19)
    // Using DATA_FLOAT32 for temperature in °C (0.25°C resolution)
    // Sample rate: 4Hz (MAX6675 has 220ms conversion time)
    // Polled in background task to avoid blocking DAQ task with SPI timeouts
    {9, "MAX6675_Temp", "C", 4, ACQ_SPI_ADC, DATA_FLOAT32, 5, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 5, 255, 255, 255, 20},

    // NJK-5002C Hall Effect RPM Sensor (PCNT, signal on GPIO26)
    // Using DATA_UINT16 for RPM (0-65535 range)
    // Sample rate: 50Hz (20ms updates, adaptive internal accumulation for accuracy)
    // Hardware PCNT counter for 0-12000 RPM range
    // NPN output pulls LOW on magnet detection (count on falling edge)
    // peripheral_pin1 = pulses_per_revolution (1 for single magnet setup)
    {10, "NJK5002C_RPM", "RPM", 50, ACQ_PCNT, DATA_UINT16, 26, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 1, 255, 255, 255, 30},

    // HP705A Clamp (Pulse output on GPIO34)
    // Using DATA_UINT16 for value
    // Sample rate: 50Hz
    // peripheral_pin1 = pulses_per_unit (1 = CPM, 60 = Hz)
    {11, "HP705A_Pulse", "PPM", 50, ACQ_PCNT, DATA_UINT16, 34, true, PIN_MODE_INPUT, ADC_ATTEN_11DB, 1, 255, 255, 255, 31},
};

constexpr uint8_t NUM_CHANNELS = sizeof(channels) / sizeof(channels[0]);

// Output pins with different capabilities
OutputConfig outputs[] = {
    // User-controllable PWM output (supports GPIO, PWM, and DAC)
    {25, "PWM_Output", OUTPUT_CAP_GPIO | OUTPUT_CAP_PWM | OUTPUT_CAP_DAC},

    // Reserved input pins (cannot be controlled by user via GPIO/PWM/DAC commands)
    {32, "ADC_Pot", OUTPUT_CAP_PERIPHERAL},          // B10K potentiometer
    {14, "GPIO_Switch2Way", OUTPUT_CAP_PERIPHERAL},  // 2-way switch
    {15, "GPIO_Switch3Way_A", OUTPUT_CAP_PERIPHERAL},// 3-way switch A
    {27, "GPIO_Switch3Way_B", OUTPUT_CAP_PERIPHERAL},// 3-way switch B
    {12, "HX711_DOUT", OUTPUT_CAP_PERIPHERAL},       // HX711 data
    {13, "HX711_SCK", OUTPUT_CAP_PERIPHERAL},        // HX711 clock
    {21, "I2C_SDA", OUTPUT_CAP_PERIPHERAL},          // I2C data (AHT10, BMP280)
    {22, "I2C_SCL", OUTPUT_CAP_PERIPHERAL},          // I2C clock (AHT10, BMP280)
    {5,  "SPI_CS", OUTPUT_CAP_PERIPHERAL},           // SPI chip select (MAX6675)
    {18, "SPI_SCK", OUTPUT_CAP_PERIPHERAL},          // SPI clock (MAX6675)
    {19, "SPI_MISO", OUTPUT_CAP_PERIPHERAL},         // SPI data in (MAX6675)
    {26, "PCNT_RPM", OUTPUT_CAP_PERIPHERAL},         // PCNT input (NJK-5002C RPM)
    {34, "PCNT_HP705A", OUTPUT_CAP_PERIPHERAL},      // PCNT input (HP705A Clamp)
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
            // Use Arduino API for attenuation to ensure analogRead() respects it
            adc_attenuation_t atten;
            switch (channels[i].adc_attenuation) {
                case ADC_ATTEN_0DB:
                    atten = ADC_0db;
                    break;
                case ADC_ATTEN_2_5DB:
                    atten = ADC_2_5db;
                    break;
                case ADC_ATTEN_6DB:
                    atten = ADC_6db;
                    break;
                case ADC_ATTEN_11DB:
                default:
                    atten = ADC_11db;
                    break;
            }
            
            analogSetPinAttenuation(channels[i].gpio_pin, atten);
        }
    }
}

// Initialize output pins
void initializeOutputs() {
    for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
        // Skip pins that are reserved for peripherals (inputs, sensors, etc.)
        // This prevents overwriting the configuration set by initializeDigitalInputs()
        if (outputs[i].capabilities & OUTPUT_CAP_PERIPHERAL) {
            continue;
        }

        pinMode(outputs[i].pin_number, OUTPUT);
        digitalWrite(outputs[i].pin_number, LOW);  // Default to LOW
    }
}
