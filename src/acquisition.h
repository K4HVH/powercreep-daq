#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "deviceConfig.h"

// HX711 implementation (24-bit load cell ADC)
class HX711Reader {
public:
    static uint32_t read(uint8_t dout_pin, uint8_t sck_pin) {
        // Wait for HX711 to be ready (DOUT goes LOW)
        unsigned long timeout = millis();
        while (digitalRead(dout_pin) == HIGH) {
            if (millis() - timeout > 100) {
                return 0; // Timeout
            }
        }

        // Read 24 bits
        uint32_t value = 0;
        for (int i = 0; i < 24; i++) {
            digitalWrite(sck_pin, HIGH);
            delayMicroseconds(1);
            value = (value << 1) | digitalRead(dout_pin);
            digitalWrite(sck_pin, LOW);
            delayMicroseconds(1);
        }

        // Send gain pulse (1 pulse = channel A gain 128)
        digitalWrite(sck_pin, HIGH);
        delayMicroseconds(1);
        digitalWrite(sck_pin, LOW);
        delayMicroseconds(1);

        // Convert to signed 24-bit
        if (value & 0x800000) {
            value |= 0xFF000000; // Sign extend
        }

        return value;
    }
};

// ADS1115 I2C ADC (16-bit)
class ADS1115Reader {
private:
    static constexpr uint8_t ADS1115_ADDR = 0x48;
    static constexpr uint8_t ADS1115_REG_CONFIG = 0x01;
    static constexpr uint8_t ADS1115_REG_CONVERT = 0x00;

public:
    static bool begin() {
        Wire.begin();
        return true;
    }

    static int16_t read(uint8_t channel) {
        if (channel > 3) return 0;

        // Config: single-shot, channel, +/-4.096V, 128 SPS
        uint16_t config = 0x8000 |  // Start single conversion
                         (channel << 12) | // MUX
                         0x0200 |  // +/-4.096V
                         0x0080 |  // Single-shot
                         0x0000;   // 128 SPS

        Wire.beginTransmission(ADS1115_ADDR);
        Wire.write(ADS1115_REG_CONFIG);
        Wire.write((config >> 8) & 0xFF);
        Wire.write(config & 0xFF);
        if (Wire.endTransmission() != 0) return 0;

        // Wait for conversion (8ms for 128 SPS)
        delay(10);

        // Read result
        Wire.beginTransmission(ADS1115_ADDR);
        Wire.write(ADS1115_REG_CONVERT);
        if (Wire.endTransmission() != 0) return 0;

        if (Wire.requestFrom((uint8_t)ADS1115_ADDR, (uint8_t)2) != 2) return 0;

        int16_t value = (Wire.read() << 8) | Wire.read();
        return value;
    }
};

// MCP3008 SPI ADC (10-bit, 8 channels)
class MCP3008Reader {
private:
    static constexpr uint32_t SPI_CLOCK = 1350000; // 1.35 MHz

public:
    static bool begin(uint8_t cs_pin) {
        pinMode(cs_pin, OUTPUT);
        digitalWrite(cs_pin, HIGH);
        SPI.begin();
        return true;
    }

    static uint16_t read(uint8_t cs_pin, uint8_t channel) {
        if (channel > 7) return 0;

        SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
        digitalWrite(cs_pin, LOW);

        // Send start bit, single-ended mode, channel select
        SPI.transfer(0x01); // Start bit
        uint8_t high = SPI.transfer((0x08 | channel) << 4) & 0x03;
        uint8_t low = SPI.transfer(0x00);

        digitalWrite(cs_pin, HIGH);
        SPI.endTransaction();

        return ((high << 8) | low);
    }
};

// UART Sensor Reader (generic)
class UARTSensorReader {
public:
    static bool begin(HardwareSerial* serial, uint32_t baud) {
        serial->begin(baud);
        return true;
    }

    static uint32_t read(HardwareSerial* serial) {
        // Generic UART read - expects 4-byte little-endian value
        if (serial->available() >= 4) {
            uint32_t value = 0;
            value |= serial->read();
            value |= (serial->read() << 8);
            value |= (serial->read() << 16);
            value |= (serial->read() << 24);
            return value;
        }
        return 0;
    }
};

// Unified acquisition interface
class Acquisition {
private:
    static bool hx711_initialized;
    static bool ads1115_initialized;
    static bool mcp3008_initialized;
    static bool uart_initialized;

public:
    static void init() {
        hx711_initialized = false;
        ads1115_initialized = false;
        mcp3008_initialized = false;
        uart_initialized = false;
    }

    static uint32_t read(const ChannelConfig& ch) {
        switch (ch.acquisition_method) {
            case ACQ_GPIO:
                return digitalRead(ch.gpio_pin) ? 1 : 0;

            case ACQ_ADC:
                return analogRead(ch.gpio_pin);

            case ACQ_HX711:
                // HX711 requires DOUT and SCK pins
                // gpio_pin = DOUT, peripheral_pin1 = SCK
                if (!hx711_initialized) {
                    if (ch.gpio_pin == 255 || ch.peripheral_pin1 == 255) return 0; // Invalid config
                    pinMode(ch.gpio_pin, INPUT);
                    pinMode(ch.peripheral_pin1, OUTPUT);
                    digitalWrite(ch.peripheral_pin1, LOW);
                    hx711_initialized = true;
                }
                return HX711Reader::read(ch.gpio_pin, ch.peripheral_pin1);

            case ACQ_I2C_ADC:
                // ADS1115 on I2C (uses hardcoded pins GPIO21=SDA, GPIO22=SCL)
                // peripheral_pin1 = SDA (21), peripheral_pin2 = SCL (22) - defined in outputs
                if (!ads1115_initialized) {
                    ADS1115Reader::begin();
                    ads1115_initialized = true;
                }
                // Use channel_id as ADS1115 channel (0-3)
                return (uint32_t)(ADS1115Reader::read(ch.channel_id & 0x03) & 0xFFFF);

            case ACQ_SPI_ADC:
                // MCP3008 - peripheral_pin1 = CS, channel_id is ADC channel
                // SPI uses hardcoded pins: GPIO5(CS), GPIO18(SCK), GPIO19(MISO), GPIO23(MOSI)
                if (!mcp3008_initialized) {
                    if (ch.peripheral_pin1 == 255) return 0; // Invalid config (need CS pin)
                    MCP3008Reader::begin(ch.peripheral_pin1);
                    mcp3008_initialized = true;
                }
                return MCP3008Reader::read(ch.peripheral_pin1, ch.channel_id & 0x07);

            case ACQ_UART:
                // UART sensor - use Serial2 (default pins GPIO16=RX, GPIO17=TX)
                // peripheral_pin1 = RX (16), peripheral_pin2 = TX (17) - defined in outputs
                if (!uart_initialized) {
                    UARTSensorReader::begin(&Serial2, 9600);
                    uart_initialized = true;
                }
                return UARTSensorReader::read(&Serial2);

            default:
                return 0;
        }
    }
};

// Static member initialization
bool Acquisition::hx711_initialized = false;
bool Acquisition::ads1115_initialized = false;
bool Acquisition::mcp3008_initialized = false;
bool Acquisition::uart_initialized = false;
